library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cordic_pkg.all;

entity dpll_top is
  generic (
    PHASE_WIDTH  : integer := 32;
    PI_WIDTH     : integer := 32;
    STAGES       : integer := 16
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;
    ce        : in  std_logic;
    ax        : in  sfixed_t;
    ay        : in  sfixed_t;
    cos_out   : out sfixed_t;
    sin_out   : out sfixed_t;
    locked    : out std_logic;
    dbg_phase_err : out sfixed_t
  );
end entity;

architecture rtl of dpll_top is
    type sfixed_t_array is array (natural range <>) of sfixed_t;
    
      -- UNWRAP / debug sizes
    constant UNWRAP_GROW : integer := 8;  -- extra bits to hold wrap multiples (tune if you expect many wraps)
    constant UNWRAP_WIDTH : integer := sfixed_t'length + UNWRAP_GROW;
  
    constant PI_COUNTS   : integer := to_integer(SFIXED_PI);         -- already used elsewhere
    constant TWO_PI_CNT  : integer := 2 * PI_COUNTS;
  
    signal phi_prev      : sfixed_t := (others => '0');             -- last raw PD output
    signal wrap_cnt      : integer range -2**28 to 2**28 := 0;       -- accumulator for wraps (choose range safely)
    signal unwrapped_sf  : signed(UNWRAP_WIDTH-1 downto 0) := (others => '0'); -- widened signed unwrapped phase
    signal unwrapped_iir : signed(UNWRAP_WIDTH-1 downto 0) := (others => '0'); -- optional IIR result
 
      
    constant LP_SHIFT     : integer := 2;  -- determines filter speed
    constant PI_MAX       : signed(PI_WIDTH-1 downto 0) := to_signed(50000000, PI_WIDTH);
    constant PI_MIN       : signed(PI_WIDTH-1 downto 0) := to_signed(-50000000, PI_WIDTH);
    constant LOCK_COUNT : integer := 50;  -- adjust as needed
    constant LOCK_THRESH  : sfixed_t := to_signed(320, sfixed_t'length);  -- threshold
    constant ERR_GAIN_SHIFT : integer := 2;  -- pre-gain to speed locking
    constant LOCK_TOL_COUNTS : integer := 3628;  -- ~5 degree error
    constant LOCK_COUNT_MAX  : integer := 1024;
    constant LOCK_TOL_DEG         : integer := 5;      -- allowed phase error (deg) when "locked"
    constant LOCK_THRESH_COUNTS   : integer := (LOCK_TOL_DEG * PI_COUNTS) / 180;
    constant ALPHA_SHIFT          : integer := 1;      -- IIR: larger = slower (2^4 = 16)
    constant LOCK_HOLD_SAMPLES    : integer := 2048;   -- samples of staying below threshold to assert lock
      -----------------------------------------------------------------------------
    -- Lock detector constants
    -----------------------------------------------------------------------------
    constant DBG_HOLD_SAMPLES  : integer := 256;   -- quick debug hold
    -- Select which one you want before compiling:
    constant EFFECTIVE_HOLD    : integer := DBG_HOLD_SAMPLES;
    -- constant EFFECTIVE_HOLD    : integer := LOCK_HOLD_SAMPLES;

  -------------------------------------------------------------------
  -- Signals
  -------------------------------------------------------------------
    signal nco_sin_buf, nco_cos_buf : sfixed_t;
    signal tuning_word               : unsigned(PHASE_WIDTH-1 downto 0);
    signal pi_out_signed             : signed(PI_WIDTH-1 downto 0);
    signal pi_input                  : signed(PI_WIDTH-1 downto 0);
    signal phase_err_pd              : sfixed_t;
    signal pd_valid                  : std_logic;
    signal nco_seed                  : unsigned(PHASE_WIDTH-1 downto 0) := to_unsigned(42_949_673, PHASE_WIDTH);
    signal pi_clipped     : signed(PI_WIDTH-1 downto 0);
    signal phase_err_filt : sfixed_t := (others => '0');
    signal phase_err_pipe : sfixed_t_array(0 to STAGES-1);
    signal pi_prev          : signed(PI_WIDTH-1 downto 0) := (others => '0');
    signal pi_delta         : signed(PI_WIDTH-1 downto 0);
    signal coarse_locked    : std_logic := '0';
    signal freq_offset_acc  : signed(PI_WIDTH-1 downto 0) := (others => '0');
    signal coarse_threshold : sfixed_t := to_signed(10000, sfixed_t'length);  -- example
    signal pi_enable : std_logic;
    signal pd_valid_pipe : std_logic_vector(0 to STAGES-1);
    signal pd_valid_aligned : std_logic;
    signal nco_seed_s : signed(PHASE_WIDTH-1 downto 0);
    signal nco_sum_s  : signed(PHASE_WIDTH-1 downto 0);
    signal lock_counter : integer range 0 to LOCK_HOLD_SAMPLES := 0;
    signal locked_reg   : std_logic := '0';
    signal err_abs_counts     : integer := 0;   -- |phase_err| in integer counts
    signal err_filt_counts    : integer := 0;   -- filtered |error|
    signal pd_bias : sfixed_t := (others => '0');
    signal dbg_cnt : integer := 0;
    signal pd_bias_acc    : sfixed_t := (others => '0');
    constant PD_BIAS_SHIFT : integer := 10;  -- adjust smoothing (larger -> slower)
     
      -- Utility: absolute value in integer domain for sfixed_t
      function abs_sfixed_to_int(s : sfixed_t) return integer is
        variable v : integer := to_integer(s);
      begin
        if v < 0 then
          return -v;
        else
          return v;
        end if;
      end function;
      
begin

  process(clk)
  begin
    if rising_edge(clk) then
        if rst = '1' then
            freq_offset_acc <= (others => '0');
            coarse_locked   <= '0';
--            tuning_word     <= nco_seed;  -- reset NCO
        elsif ce = '1' and pd_valid_aligned = '1' then
            -- Integrate the phase error slowly
            freq_offset_acc <= freq_offset_acc + shift_left(resize(phase_err_pipe(STAGES-1), PI_WIDTH), 2);

            -- Check if frequency offset is within threshold
            if abs(to_integer(freq_offset_acc)) < to_integer(coarse_threshold) then
                coarse_locked <= '1';
                freq_offset_acc <= (others => '0');
            else
                coarse_locked <= '0';
            end if;
        end if;
    end if;
  end process;


  -------------------------------------------------------------------
  -- Phase Detector
  -------------------------------------------------------------------
  pd_inst: entity work.phase_detector_cordic
    generic map (
      STAGES => STAGES
    )
    port map (
      clk       => clk,
      rst       => rst,
      valid_in  => ce,
      ax        => ax,
      ay        => ay,
      bx        => nco_cos_buf,
      by        => nco_sin_buf,
      valid_out => pd_valid,
      phase_err => phase_err_pd,
      cos_delta => open,
      sin_delta => open
    );

  -------------------------------------------------------------------
  -- Scale phase error for PI filter
  -------------------------------------------------------------------
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        phase_err_pipe <= (others => (others => '0'));
        pd_valid_pipe <= (others => '0');
      else
        phase_err_pipe(0) <= phase_err_pd;
        pd_valid_pipe(0)   <= pd_valid;
        for i in 1 to STAGES-1 loop
          phase_err_pipe(i) <= phase_err_pipe(i-1);
          pd_valid_pipe(i)   <= pd_valid_pipe(i-1);
        end loop;
      end if;
    end if;
  end process;
  
-- ---------------------------------------------------------------------
  -- PD bias estimator (IIR) and bias-subtracted aligned PI input
  -- drop this into dpll_top (replace your commented pd_bias / pi_input code)
  -- ---------------------------------------------------------------------

  
--  process(clk)
--  begin
--    if rising_edge(clk) then
--      if rst = '1' then
--        pd_bias_acc <= (others => '0');
--      elsif ce = '1' and pd_valid_aligned = '1' then
--        -- IIR estimate: bias[n] = bias[n-1] + (x - bias)/2^PD_BIAS_SHIFT
--        pd_bias_acc <= pd_bias_acc + shift_right(phase_err_pipe(STAGES-1) - pd_bias_acc, PD_BIAS_SHIFT);
--      end if;
--    end if;
--  end process;
  
--  -- Produce PI input from aligned, bias-subtracted PD output.
--  -- Note: pi_enable already follows pd_valid_aligned in your top.
--  -- Use bias-subtracted and optionally downscale (srl) if needed.
--  pi_input <= shift_left(
--                resize(phase_err_pipe(STAGES-1) - pd_bias_acc, PI_WIDTH),
--                ERR_GAIN_SHIFT
--              );



  dbg_phase_err <= phase_err_pd;
 process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        pd_valid_aligned <= '0';
      else
        pd_valid_aligned <= pd_valid;
      end if;
    end if;
  end process;

--  pi_input <= shift_left(resize(phase_err_pipe(STAGES-1) srl 1, PI_WIDTH), ERR_GAIN_SHIFT);
  pi_input <= shift_left(resize(phase_err_pd, PI_WIDTH), ERR_GAIN_SHIFT);
--  pi_input <= shift_left(resize((phase_err_pipe(STAGES-1) - pd_bias) srl 1, PI_WIDTH), ERR_GAIN_SHIFT);
--  pi_enable <= '1' when pd_valid_aligned = '1' and coarse_locked = '1' else '0';
   pi_enable <= pd_valid_aligned;
  -------------------------------------------------------------------
  -- PI filter
  -------------------------------------------------------------------
  pi_inst: entity work.type2_loop_filter
    generic map (
      PHASE_WIDTH => PI_WIDTH,
      ACC_WIDTH   => 48
    )
    port map (
      clk    => clk,
      rst    => rst,
      ce     => pi_enable,
      phase_err => pi_input,
      tuning_out  => pi_out_signed
    );
    
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                pi_prev <= (others => '0');
                pi_delta <= (others => '0');
            else
                pi_delta <= pi_out_signed - pi_prev;
                pi_prev <= pi_out_signed;
            end if;
        end if;
    end process;


  -------------------------------------------------------------------
  -- Convert signed PI output to unsigned NCO tuning word
  -------------------------------------------------------------------
  
  clipping_process: process(pi_out_signed)
  begin
      if pi_out_signed > PI_MAX then
          pi_clipped <= PI_MAX;
      elsif pi_out_signed < PI_MIN then
          pi_clipped <= PI_MIN;
      else
          pi_clipped <= pi_out_signed;
      end if;
  end process clipping_process;
  
  nco_seed_s <= signed(nco_seed);
  nco_sum_s  <= nco_seed_s + resize(pi_out_signed, PHASE_WIDTH);
  tuning_word <= unsigned(nco_sum_s);

  -------------------------------------------------------------------
  -- NCO (CORDIC oscillator)
  -------------------------------------------------------------------
  nco_inst: entity work.cordic_oscillator_dds
    generic map (
      PHASE_WIDTH => PHASE_WIDTH
    )
    port map (
      clk         => clk,
      rst         => rst,
      ce          => ce,
      tuning_word => tuning_word,
      sin_out     => nco_sin_buf,
      cos_out     => nco_cos_buf,
      valid_out   => open
    );

  -------------------------------------------------------------------
  -- Drive top-level outputs from internal buffers
  -------------------------------------------------------------------
  sin_out <= nco_sin_buf;
  cos_out <= nco_cos_buf;

  -----------------------------------------------------------------------------
    -- IIR smoothing of absolute (aligned) phase error
    -----------------------------------------------------------------------------
    iir_lock_proc: process(clk)
      variable delta : sfixed_t;
    begin
      if rising_edge(clk) then
        if rst = '1' then
          phase_err_filt <= (others => '0');
        elsif ce = '1' and pd_valid = '1' then
          -- Absolute value
          if phase_err_pd < 0 then
            delta := -phase_err_pd - phase_err_filt;
          else
            delta := phase_err_pd - phase_err_filt;
          end if;
    
          -- IIR smoothing: y[n] = y[n-1] + (x[n]-y[n-1])/2^ALPHA_SHIFT
          phase_err_filt <= phase_err_filt + shift_right(delta, ALPHA_SHIFT);
        end if;
      end if;
    end process;

    
    
   -----------------------------------------------------------------------------
     -- Hold counter / locked assert (use >= for robustness)
     -----------------------------------------------------------------------------
     lock_hold_proc: process(clk)
     begin
       if rising_edge(clk) then
         if rst = '1' then
           lock_counter <= 0;
           locked_reg   <= '0';
         elsif ce = '1' and pd_valid = '1' then
           -- Check filtered error against threshold
           if phase_err_filt <= LOCK_THRESH then
             if lock_counter < EFFECTIVE_HOLD then
               lock_counter <= lock_counter + 1;
             end if;
           else
             lock_counter <= 0;
           end if;
     
           -- Assert lock after hold samples
           if lock_counter >= EFFECTIVE_HOLD then
             locked_reg <= '1';
           else
             locked_reg <= '0';
           end if;
         end if;
       end if;
     end process;
     
     locked <= locked_reg;
 
  -----------------------------------------------------------------------------
  -- Debug reporting (sane reset behavior)
  -----------------------------------------------------------------------------
   
   debug_proc : process(clk)
   begin
     if rising_edge(clk) then
       if rst = '0' and pd_valid = '1' then
         report "DBG_PULSE: phase_err_pd=" & integer'image(to_integer(phase_err_pd)) &
                "  err_filt=" & integer'image(to_integer(phase_err_filt)) &
                "  lock_cnt=" & integer'image(lock_counter) &
                "  LOCK_THRESH=" & integer'image(to_integer(LOCK_THRESH)) &
                "  pi_enable=" & std_logic'image(pi_enable) &
                "  pi_out=" & integer'image(to_integer(pi_out_signed)) &
                "  tuning=" & integer'image(to_integer(tuning_word));
          report "PI_INPUT: " & integer'image(to_integer(pi_input));
       end if;
     end if;
   end process;

    -----------------------------------------------------------------------------
  -- Unwrap monitor (debug only)
  -- Tracks phase_err_pd wraps and produces an unwrapped phase in unwrapped_sf.
  -- Gate on pd_valid to advance sample-by-sample.
  -----------------------------------------------------------------------------
  unwrap_proc : process(clk)
    variable d          : integer;
    variable raw_i      : integer;
    variable prev_i     : integer;
    variable wrap_delta : integer;
    variable tmp_signed : signed(UNWRAP_WIDTH-1 downto 0);
  begin
    if rising_edge(clk) then
      if rst = '1' then
        phi_prev      <= (others => '0');
        wrap_cnt      <= 0;
        unwrapped_sf  <= (others => '0');
        unwrapped_iir <= (others => '0');
      elsif pd_valid = '1' then
        -- convert raw sfixed to integer-count domain (same scaling as SFIXED_PI)
        raw_i  := to_integer(phase_err_pd);
        prev_i := to_integer(phi_prev);
        d := raw_i - prev_i;

        -- detect wrap direction: if jump > +PI  => previously crossed -PI -> +2PI adjustment
        wrap_delta := 0;
        if d >  PI_COUNTS then
          wrap_delta := -1;   -- note: subtract 1 because raw moved from near -pi to +pi -> wrap down
        elsif d < -PI_COUNTS then
          wrap_delta := +1;   -- raw moved from near +pi to -pi -> wrap up
        end if;

        -- update wrap counter and previous raw
        wrap_cnt <= wrap_cnt + wrap_delta;
        phi_prev <= phase_err_pd;

        -- build widened unwrapped signed value: raw + wrap_cnt * 2*pi_counts
        tmp_signed := resize(signed(phase_err_pd), UNWRAP_WIDTH)
                      + resize(to_signed(wrap_cnt * TWO_PI_CNT, UNWRAP_WIDTH), UNWRAP_WIDTH);
        unwrapped_sf <= tmp_signed;

        -- optional: small IIR on unwrapped phase so you can compare with your angle IIR
        -- unwrapped_iir <= unwrapped_iir + shift_right(tmp_signed - unwrapped_iir, ALPHA_SHIFT);
        -- (use same ALPHA_SHIFT as your lock detector for apples-to-apples)
      end if;
    end if;
  end process unwrap_proc;

end architecture;
