-- SPDX-License-Identifier: MIT
-- =============================================================================
-- Title       : CORDIC Oscillator (Top level)
-- File        : cordic_oscillator.vhd
-- Author      : Abhishek Garg <abhishekgarg403@gmail.com>
-- Created     : 2025-08-15
-- Last Edited : 2025-08-15
-- Version     : 1.0
-- Repository  : https://github.com/Abhishek-Garg25/cordic-oscillator
-- -----------------------------------------------------------------------------
-- Description :
--   Top-level CORDIC-based oscillator module.
--   Generates continuous sine and cosine outputs using iterative CORDIC
--   rotations. Designed for synthesizable FPGA implementation using
--   fixed-point arithmetic (no multipliers).
--
--   This file instantiates the CORDIC pipeline/stages and provides:
--     - Phase accumulator or phase input
--     - Control signals (enable, reset)
--     - Output registers for sine/cosine
--
-- -----------------------------------------------------------------------------
-- Features / Parameters :
--   - Fixed-point format: Q<WF-1>.<WF-FRAC>  (documented in cordic_pkg.vhd)
--   - Iteration depth (N) controls precision and resource usage
--   - Includes optional scaling compensation (CORDIC gain K)
--
-- -----------------------------------------------------------------------------
-- Dependencies :
--   - cordic_stage.vhd
--   - cordic_pkg.vhd   (fixed-point types, constants, angle table generator)
--   - cordic_pipelined.vhd
-- -----------------------------------------------------------------------------
-- Implementation notes :
--   - Use synchronous reset and register outputs to meet timing flow.
--   - Choose ITER to balance precision vs latency/resource usage.
--   - Phase accumulator should wrap modulo 2*pi; represent phase in same
--     fixed-point format as CORDIC angle table.
--
-- -----------------------------------------------------------------------------
-- Revision History :
--   v1.0  2025-08-15  Initial release
--
-- -----------------------------------------------------------------------------
-- License :
--   Released under the MIT License. See LICENSE file in the project root.
-- =============================================================================


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cordic_pkg.all;

entity cordic_oscillator_dds is
  generic (
    PHASE_WIDTH : integer := 32   -- Accumulator width
  );
  port (
    clk         : in  std_logic;
    rst         : in  std_logic;
    ce          : in  std_logic;
    tuning_word : in  unsigned(PHASE_WIDTH-1 downto 0); -- Frequency control word
    sin_out     : out sfixed_t;
    cos_out     : out sfixed_t;
    valid_out   : out std_logic
  );
end entity;

architecture rtl of cordic_oscillator_dds is
  constant PHASE_SCALE : real := (2.0 * MATH_PI) / (2.0**PHASE_WIDTH);
  signal phase_acc : unsigned(PHASE_WIDTH-1 downto 0) := (others => '0');
  signal z_in      : sfixed_internal_t;

  signal x_fixed, y_fixed : sfixed_t;
  signal x_out, y_out     : sfixed_t;
  signal valid_pipe       : std_logic;
  signal phase_real : real;
begin

  -------------------------------------------------------------------
  -- DDS PHASE ACCUMULATOR
  -------------------------------------------------------------------
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        phase_acc <= (others => '0');
      elsif ce = '1' then
        phase_acc <= phase_acc + tuning_word;  -- Natural wrap at 2^PHASE_WIDTH
      end if;
      phase_real <= PHASE_SCALE * real(to_integer(phase_acc));
    end if;
  end process;

  -------------------------------------------------------------------
  -- MAP TO RADIANS FOR CORDIC
  -- θ = (2π / 2^PHASE_WIDTH) * phase_acc
  -------------------------------------------------------------------
  
  
  z_in <= real_to_sfixed(phase_real, CORDIC_INTERNAL_WIDTH);


  -------------------------------------------------------------------
  -- FIXED INITIAL VECTOR FOR ROTATION MODE
  -------------------------------------------------------------------
  x_fixed <= real_to_sfixed(CORDIC_GAIN, CORDIC_WIDTH);
  y_fixed <= real_to_sfixed(0.0, CORDIC_WIDTH);

  -------------------------------------------------------------------
  -- CORDIC PIPELINED INSTANCE
  -------------------------------------------------------------------
  cordic_inst: entity work.cordic_pipelined
    port map (
      clk        => clk,
      rst        => rst,
      x_in       => x_fixed,
      y_in       => y_fixed,
      z_in       => z_in,
      x_out      => x_out,
      y_out      => y_out,
      z_out      => open,
      valid_in   => '1',
      valid_out  => valid_pipe
    );

  -------------------------------------------------------------------
  -- OUTPUTS
  -------------------------------------------------------------------
  cos_out   <= x_out;
  sin_out   <= y_out;
  valid_out <= valid_pipe;

end architecture;
