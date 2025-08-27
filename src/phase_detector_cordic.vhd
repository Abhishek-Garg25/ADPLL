-- SPDX-License-Identifier: MIT
-- =============================================================================
-- Title       : CORDIC-based Phase Detector (for DPLL)
-- File        : phase_detector_cordic.vhd
-- Author      : Abhishek Garg
-- Created     : 2025-08-20
-- Description :
--   Computes phase difference Δ between two phasors A(ax,ay) and B(bx,by):
--     cosΔ = ax*bx + ay*by
--     sinΔ = ax*by - ay*bx
--   Then runs a single CORDIC in VECTORING mode on (cosΔ, sinΔ) to get Δ.
--   Output z_out is the signed phase error in the same fixed-point angle
--   format as your CORDIC uses (±π maps to your SFIXED ranges).
--
--   Latency: STAGES + 2 cycles (one pre-reg + CORDIC pipeline).
--
-- Notes :
--   - Assumes work.cordic_pkg defines:
--       * sfixed_t, sfixed_internal_t
--       * CORDIC_WIDTH, CORDIC_INTERNAL_WIDTH
--       * resize(...) that resizes sfixed to the desired internal width
--   - The "*" operator on sfixed_* is assumed to map to signed mult with
--     appropriate widening (common in numeric_std-based fixed types).
--     If your sfixed_t comes from ieee.fixed_pkg, replace the simple
--     products with to_sfixed()/resize()/shift_align as needed.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cordic_pkg.all;
use ieee.math_real.all;

entity phase_detector_cordic is
  generic (
    STAGES : integer := 16  -- pass-through to cordic_pipelined
  );
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;
    valid_in  : in  std_logic;

    -- Phasor A = (ax, ay), e.g. incoming signal’s I/Q (normalized if possible)
    ax, ay    : in  sfixed_t;

    -- Phasor B = (bx, by), e.g. NCO reference (cos, sin)
    bx, by    : in  sfixed_t;

    -- Outputs
    valid_out : out std_logic;
    phase_err : out sfixed_t;          -- Δ = atan2(sinΔ, cosΔ)
    cos_delta : out sfixed_t;          -- optional: cosΔ (for diagnostics)
    sin_delta : out sfixed_t           -- optional: sinΔ (for diagnostics)
  );
end entity;

architecture rtl of phase_detector_cordic is

  -- Internal widened signals (match your CORDIC internal width)
  signal ax_i, ay_i, bx_i, by_i : sfixed_internal_t;
  subtype sfixed_wide_t is signed(2*CORDIC_INTERNAL_WIDTH-1 downto 0);
  -- Products (widened). If your multiplication widens beyond INTERNAL,
  -- you may want an even wider type; often INTERNAL is enough in practice
  -- when inputs are already scaled to < 1.0.
  signal axbx, ayby, axby, aybx : sfixed_wide_t;

  -- cosΔ, sinΔ aligned to CORDIC inputs
  signal cos_d_i, sin_d_i : sfixed_internal_t;

  -- Registered versions to feed CORDIC cleanly
  signal cos_d_r, sin_d_r : sfixed_internal_t;
  signal vld_r            : std_logic;
  
  signal abs_cos, abs_sin       : unsigned(CORDIC_INTERNAL_WIDTH-1 downto 0);
  signal cos_raw, sin_raw       : sfixed_wide_t;
  -- CORDIC interface
  signal cordic_valid  : std_logic;
  signal cordic_x_out,
         cordic_y_out,
         cordic_z_out  : sfixed_t;  -- cordic_pipelined ports are sfixed_t width
  signal cordic_x_in, cordic_y_in : sfixed_t;
  signal cos_scaled, sin_scaled : sfixed_internal_t;
  signal max_abs : unsigned(sfixed_wide_t'length-1 downto 0);
  signal norm_sh : integer range 0 to sfixed_wide_t'length := 0;
  ------------------------------------------------------------------------------
    -- Determine normalization (how many top bits we need to shift right)
    -- We find the index of the most-significant '1' of the absolute (magnitude)
    -- of the larger of |cos_raw| and |sin_raw| and compute a shift so
    -- the value fits into CORDIC_INTERNAL_WIDTH bits (signed).
    ------------------------------------------------------------------------------
    function abs_signed_wide(x: sfixed_wide_t) return unsigned is
      variable r : unsigned(x'length-1 downto 0);
      variable t : sfixed_wide_t := x;
    begin
      if x(x'left) = '1' then
        -- negative
        t := not x + 1;
      end if;
      r := unsigned(t);
      return r;
    end;
    
    function to_i32_safe(s: signed) return integer is
        variable r : signed(31 downto 0);
      begin
        r := resize(s, 32);
        return to_integer(r);
      end;

begin

  ------------------------------------------------------------------------------
  -- Resize inputs up to internal precision (helps with headroom)
  ------------------------------------------------------------------------------
  ax_i <= resize(ax, CORDIC_INTERNAL_WIDTH);
  ay_i <= resize(ay, CORDIC_INTERNAL_WIDTH);
  bx_i <= resize(bx, CORDIC_INTERNAL_WIDTH);
  by_i <= resize(by, CORDIC_INTERNAL_WIDTH);

  ------------------------------------------------------------------------------
  -- Parallel products to build cosΔ and sinΔ
  -- cosΔ = ax*bx + ay*by
  -- sinΔ = ax*by - ay*bx
  ------------------------------------------------------------------------------
  axbx <= ax_i * bx_i;
  ayby <= ay_i * by_i;
  axby <= ax_i * by_i;
  aybx <= ay_i * bx_i;

------------------------------------------------------------------------------
-- Compute cos? and sin? and normalize to avoid overflow
------------------------------------------------------------------------------
  cos_raw <= resize(axbx, cos_raw'length) + resize(ayby, cos_raw'length);
  sin_raw <= resize(aybx, sin_raw'length) - resize(axby, sin_raw'length);
  
  process(cos_raw, sin_raw)
    variable a_cos : unsigned(sfixed_wide_t'length-1 downto 0);
    variable a_sin : unsigned(sfixed_wide_t'length-1 downto 0);
    variable i     : integer;
    variable top_i : integer := 0;
    variable found : boolean;
  begin
    a_cos := abs_signed_wide(cos_raw);
    a_sin := abs_signed_wide(sin_raw);
    -- choose larger magnitude
    if a_cos >= a_sin then
      max_abs <= a_cos;
    else
      max_abs <= a_sin;
    end if;

    -- find MSB position of max_abs (scan from top)
    found := false;
    top_i := 0;
    for i in max_abs'left downto 0 loop
      if max_abs(i) = '1' then
        top_i := i;
        found := true;
        exit;
      end if;
    end loop;

    if not found then
      norm_sh <= 0;
    else
      -- bits needed to represent magnitude = top_i+1
      -- we need signed magnitude to fit in CORDIC_INTERNAL_WIDTH-1 bits (one MSB for sign)
      if (top_i + 1) <= (CORDIC_INTERNAL_WIDTH - 1) then
        norm_sh <= 0;
      else
        norm_sh <= (top_i + 1) - (CORDIC_INTERNAL_WIDTH - 1);
      end if;
    end if;
  end process;

  
  ------------------------------------------------------------------------------
    -- Apply normalization shift plus any fixed fractional alignment
    -- Use arithmetic right shift (sra / shift_right) so sign preserved
    ------------------------------------------------------------------------------
    
    begin_scale: process(cos_raw, sin_raw, norm_sh)
      variable tmp_cos, tmp_sin : signed(cos_raw'length-1 downto 0);
      variable total_sh         : integer;
    begin
      tmp_cos := cos_raw;
      tmp_sin := sin_raw;
      total_sh := norm_sh + CORDIC_FRAC; -- first remove integer headroom then fractional
      if total_sh < 0 then
        -- shouldn't happen; treat as no shift
        cos_scaled <= resize(tmp_cos, CORDIC_INTERNAL_WIDTH);
        sin_scaled <= resize(tmp_sin, CORDIC_INTERNAL_WIDTH);
      else
        -- arithmetic shift right, then resize down to internal cordic width
        cos_scaled <= resize( shift_right(tmp_cos, total_sh), CORDIC_INTERNAL_WIDTH );
        sin_scaled <= resize( shift_right(tmp_sin, total_sh), CORDIC_INTERNAL_WIDTH );
      end if;
    end process;
  
--    cos_d_i <= resize( (axbx + ayby) srl CORDIC_FRAC, CORDIC_INTERNAL_WIDTH);
--    sin_d_i <= resize( (axby - aybx) srl CORDIC_FRAC, CORDIC_INTERNAL_WIDTH);
    -- Finally assign to the signals that feed the cordic pre-register
    cos_d_i <= cos_scaled;
    sin_d_i <= sin_scaled;
  ------------------------------------------------------------------------------
  -- Small pre-register for timing; aligns valid
  ------------------------------------------------------------------------------
  pre_reg: process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        cos_d_r <= (others => '0');
        sin_d_r <= (others => '0');
        vld_r   <= '0';
      else
        cos_d_r <= cos_d_i;
        sin_d_r <= sin_d_i;
        vld_r   <= valid_in;
      end if;
    end if;
  end process;
  
  
  cordic_x_in <= resize(cos_d_r, CORDIC_WIDTH);
  cordic_y_in <= resize(sin_d_r, CORDIC_WIDTH);
  ------------------------------------------------------------------------------
  -- Single CORDIC in VECTORING mode to get Δ = atan2(y, x)
  ------------------------------------------------------------------------------
  cordic_inst: entity work.cordic_pipelined
    generic map (
      MODE   => "VECTORING",
      STAGES => STAGES
    )
    port map (
      clk        => clk,
      rst        => rst,
      valid_in   => vld_r,
      x_in       => cordic_x_in,  -- cordic expects sfixed_t width
      y_in       => cordic_y_in,
      z_in       => (others => '0'),                -- not used in VECTORING
      valid_out  => cordic_valid,
      x_out      => open,                           -- magnitude (unused here)
      y_out      => open,
      z_out      => cordic_z_out
    );

  ------------------------------------------------------------------------------
  -- Outputs (registered by CORDIC already)
  ------------------------------------------------------------------------------
  valid_out <= cordic_valid;
  phase_err <= cordic_z_out;
  cos_delta <= resize(cos_d_r, CORDIC_WIDTH);
  sin_delta <= resize(sin_d_r, CORDIC_WIDTH);
  
--  debug_products: process(clk)
--  begin
--      if rising_edge(clk) then
--          if rst = '0' and valid_in = '1' then
--              report "PD_DEBUG: ax=" & real'image(sfixed_to_real(ax)) &
--                     " ay=" & real'image(sfixed_to_real(ay)) &
--                     " bx=" & real'image(sfixed_to_real(bx)) &
--                     " by=" & real'image(sfixed_to_real(by)) &
--                     " axbx=" & integer'image(to_integer(axbx)) &
--                     " ayby=" & integer'image(to_integer(ayby)) &
--                     " axby=" & integer'image(to_integer(axby)) &
--                     " aybx=" & integer'image(to_integer(aybx)) &
--                     " cos_d_i=" & integer'image(to_integer(cos_d_i)) &
--                     " sin_d_i=" & integer'image(to_integer(sin_d_i));
--          end if;
--      end if;
--  end process;
  
  debug_products: process(clk)
    begin
        if rising_edge(clk) then
            if rst = '0' and valid_in = '1' then
                report "PD_DEBUG: ax=" & real'image(sfixed_to_real(ax)) &
                       " ay=" & real'image(sfixed_to_real(ay)) &
                       " bx=" & real'image(sfixed_to_real(bx)) &
                       " by=" & real'image(sfixed_to_real(by)) &
                       " axbx=" & integer'image(to_i32_safe(resize(axbx,32))) &
                       " ayby=" & integer'image(to_i32_safe(resize(ayby,32))) &
                       " axby=" & integer'image(to_i32_safe(resize(axby,32))) &
                       " aybx=" & integer'image(to_i32_safe(resize(aybx,32))) &
                       " cos_raw=" & integer'image(to_i32_safe(resize(cos_raw,32))) &
                       " sin_raw=" & integer'image(to_i32_safe(resize(sin_raw,32))) &
                       " norm_sh=" & integer'image(norm_sh) &
                       " cos_d_i=" & integer'image(to_i32_safe(resize(cos_d_i,32))) &
                       " sin_d_i=" & integer'image(to_i32_safe(resize(sin_d_i,32)));
               report "PD_ZERO_TEST: cos_d_i=" & integer'image(to_integer(cos_d_i)) &
                      " sin_d_i=" & integer'image(to_integer(sin_d_i)) &
                      " cordic_z=" & integer'image(to_integer(cordic_z_out));
            end if;
        end if;
    end process;


end architecture;
