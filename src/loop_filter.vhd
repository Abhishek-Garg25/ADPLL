library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cordic_pkg.all;

entity type2_loop_filter is
  generic (
    PHASE_WIDTH : integer := 32;  -- input width
    ACC_WIDTH   : integer := 48   -- integral accumulator width
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;
    ce        : in  std_logic;
    phase_err : in  signed(PHASE_WIDTH-1 downto 0);      -- from phase detector
    tuning_out: out signed(PHASE_WIDTH-1 downto 0)
  );
end entity;

architecture rtl of type2_loop_filter is
  signal integrator   : signed(ACC_WIDTH-1 downto 0) := (others=>'0');
  signal prop_term    : signed(PHASE_WIDTH-1 downto 0);
  signal phase_scaled : signed(PHASE_WIDTH-1 downto 0);
  signal integ_step   : signed(ACC_WIDTH-1 downto 0);
  signal tentative    : signed(ACC_WIDTH-1 downto 0);
  signal integ_signed : signed(ACC_WIDTH-1 downto 0);
  
  -- Filter gains (fixed-point scaled to PHASE_WIDTH)
  constant Kp_SHIFT   : integer := 1;   -- proportional shift (tune)
  constant Ki_SHIFT   : integer := 8;   -- base integral shift (tune)
  -- Adaptive + leak settings (tune these)
  constant LEAK_SHIFT : integer := 8;  -- integrator leak: integrator -= integrator >> LEAK_SHIFT
  constant SMALL_ERR_THRESH : integer := 2**(PHASE_WIDTH-4); -- small error deadzone (approx), tune
--  constant ACC_MAX_CONST : integer :=  (2**(ACC_WIDTH-1) - 1);
--  constant ACC_MIN_CONST : integer := -(2**(ACC_WIDTH-1));

  -- Derived constants as signed vectors
  constant ACC_MAX : signed(ACC_WIDTH-1 downto 0) := (ACC_WIDTH-1 => '0', others => '1'); -- +max
  constant ACC_MIN : signed(ACC_WIDTH-1 downto 0) := (ACC_WIDTH-1 => '1', others => '0'); -- -max

begin

  -- Scale incoming phase error to PHASE_WIDTH resolution (optional pre-gain)
  phase_scaled <= shift_left(resize(phase_err, PHASE_WIDTH), 2);



  -- Compute integration step (ACC domain). We will adapt Ki depending on error magnitude
  process(phase_scaled)
    variable abs_err : unsigned(PHASE_WIDTH-1 downto 0);
    variable ki_eff_shift : integer := Ki_SHIFT;
    variable step_s : signed(ACC_WIDTH-1 downto 0);
  begin
    abs_err := unsigned(phase_scaled(PHASE_WIDTH-1 downto 0));
    -- adaptive: if error small, reduce integration speed (increase shift)
    if (phase_scaled >= to_signed(0, PHASE_WIDTH)) then
      if (abs_err < to_unsigned(SMALL_ERR_THRESH, PHASE_WIDTH)) then
        ki_eff_shift := Ki_SHIFT + 4;  -- slower integrate near zero (tune)
      else
        ki_eff_shift := Ki_SHIFT;
      end if;
    else
      if (abs_err < to_unsigned(SMALL_ERR_THRESH, PHASE_WIDTH)) then
        ki_eff_shift := Ki_SHIFT + 4;
      else
        ki_eff_shift := Ki_SHIFT;
      end if;
    end if;

    -- produce integrator increment in ACC domain: (phase_scaled >> ki_eff_shift) extended
    step_s := resize( shift_right( resize(phase_scaled, ACC_WIDTH), ki_eff_shift ), ACC_WIDTH );
    integ_step <= step_s;
  end process;

  -- Integrator process: anti-windup + leak + adaptive step
  process(clk)
    variable leak_term : signed(ACC_WIDTH-1 downto 0);
    variable new_int   : signed(ACC_WIDTH-1 downto 0);
  begin
    if rising_edge(clk) then
      if rst = '1' then
        integrator <= (others => '0');
        prop_term  <= (others => '0');
      elsif ce = '1' then
        -- leak: remove a small fraction of integrator to prevent DC accumulation
        leak_term := shift_right(integrator, LEAK_SHIFT);
          -- Proportional term (coarse)
        prop_term <= shift_right(phase_scaled, Kp_SHIFT);
        -- tentative accumulation (with step)
        new_int := integrator - leak_term + integ_step;
        
        -- saturate (anti-windup): keep integrator within ACC bounds
        if new_int > ACC_MAX then
          integrator <= ACC_MAX;
        elsif new_int < ACC_MIN then
          integrator <= ACC_MIN;
        else
          integrator <= new_int;
        end if;
--        integrator <= (others => '0');
--        prop_term  <= (others => '0');
      end if;
    end if;
  end process;

  -- Compose output: take top PHASE_WIDTH bits of integrator plus prop term
  -- slice integrator MSBs appropriate to produce PHASE_WIDTH
  tuning_out <= resize( integrator(ACC_WIDTH-1 downto ACC_WIDTH-PHASE_WIDTH) + prop_term, PHASE_WIDTH );

end architecture;
