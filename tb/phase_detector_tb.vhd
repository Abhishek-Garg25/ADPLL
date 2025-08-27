-- SPDX-License-Identifier: MIT
-- =============================================================================
-- Title       : Testbench for phase_detector_cordic
-- File        : phase_detector_tb.vhd
-- Description :
--   Self-checking testbench using cordic_pkg utilities (no ieee.fixed_pkg).
--   It feeds known phasor pairs (A=?, B=?) and checks phase_err ? (? - ?).
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.cordic_pkg.all;

entity phase_detector_tb is
end entity;

architecture tb of phase_detector_tb is

  constant STAGES     : integer := 16;
  constant CLK_PERIOD : time    := 10 ns;

  -- DUT I/O
  signal clk       : std_logic := '0';
  signal rst       : std_logic := '1';
  signal valid_in  : std_logic := '0';
  signal ax, ay    : sfixed_t  := (others => '0');
  signal bx, by    : sfixed_t  := (others => '0');
  signal valid_out : std_logic;
  signal phase_err : sfixed_t;
  signal cos_delta : sfixed_t;
  signal sin_delta : sfixed_t;

  -- Test vectors
  type test_case_t is record
    phase_a : real;  -- radians (?)
    phase_b : real;  -- radians (?)
  end record;

  type test_vec_t is array (natural range <>) of test_case_t;

  constant tests : test_vec_t := (
    0 => (phase_a => 0.0,           phase_b => 0.0),            -- ? = 0
    1 => (phase_a => 0.0,           phase_b =>  MATH_PI/4.0),   -- ? = +?/4
    2 => (phase_a =>  MATH_PI/4.0,  phase_b => 0.0),            -- ? = -?/4
    3 => (phase_a =>  MATH_PI/2.0,  phase_b => -MATH_PI/2.0),   -- ? = -?
    4 => (phase_a => -MATH_PI/3.0,  phase_b =>  MATH_PI/6.0)    -- ? = +?/2
  );

  constant TOL : real := 0.01;  -- radians (~0.6°)

  -- Helpers
  impure function wrap_pi(x : real) return real is
    variable y : real := x;
  begin
    while y >  MATH_PI loop y := y - 2.0*MATH_PI; end loop;
    while y <= -MATH_PI loop y := y + 2.0*MATH_PI; end loop;
    return y;
  end;

  impure function ang_err(a, b : real) return real is
  begin
    return wrap_pi(a - b);
  end;
  
  function atan2_approx(y, x : real) return real is
    variable ang : real;
  begin
    if x > 0.0 then
      ang := ieee.math_real.arctan(y / x);
    elsif x < 0.0 and y >= 0.0 then
      ang := ieee.math_real.arctan(y / x) + MATH_PI;
    elsif x < 0.0 and y < 0.0 then
      ang := ieee.math_real.arctan(y / x) - MATH_PI;
    elsif x = 0.0 and y > 0.0 then
      ang := MATH_PI / 2.0;
    elsif x = 0.0 and y < 0.0 then
      ang := -MATH_PI / 2.0;
    else
      ang := 0.0;
    end if;
    return ang;
  end;


begin
  ---------------------------------------------------------------------------
  -- Clock
  ---------------------------------------------------------------------------
  clk <= not clk after CLK_PERIOD/2;

  ---------------------------------------------------------------------------
  -- DUT
  ---------------------------------------------------------------------------
  dut: entity work.phase_detector_cordic
    generic map ( STAGES => STAGES )
    port map (
      clk       => clk,
      rst       => rst,
      valid_in  => valid_in,
      ax        => ax,
      ay        => ay,
      bx        => bx,
      by        => by,
      valid_out => valid_out,
      phase_err => phase_err,
      cos_delta => cos_delta,
      sin_delta => sin_delta
    );

  ---------------------------------------------------------------------------
  -- Stimulus
  ---------------------------------------------------------------------------
  stim: process
    variable expected : real;
    variable actual   : real;
    variable err      : real;
    variable real_cos, real_sin, atan_real, z_real : real;
  begin
    -- Reset
    rst <= '1';
    wait for 5*CLK_PERIOD;
    rst <= '0';
    wait for CLK_PERIOD;

    for t in tests'range loop
      -- Convert phases to phasors using real_to_sfixed ? signed ? resize
      ax <= signed_to_sfixed_t(real_to_sfixed(ieee.math_real.cos(tests(t).phase_a), CORDIC_WIDTH));
      ay <= signed_to_sfixed_t(real_to_sfixed(ieee.math_real.sin(tests(t).phase_a), CORDIC_WIDTH));
      bx <= signed_to_sfixed_t(real_to_sfixed(ieee.math_real.cos(tests(t).phase_b), CORDIC_WIDTH));
      by <= signed_to_sfixed_t(real_to_sfixed(ieee.math_real.sin(tests(t).phase_b), CORDIC_WIDTH));


      -- One-cycle valid strobe
      valid_in <= '1';
      wait for CLK_PERIOD;
      valid_in <= '0';

      -- Wait for latency (STAGES+2 cycles)
      wait until rising_edge(clk) and valid_out = '1';
      
      real_cos := sfixed_to_real(cos_delta);
      real_sin := sfixed_to_real(sin_delta);
      atan_real := atan2_approx(real_sin, real_cos);
      z_real := sfixed_to_real(phase_err);
      
      

      if valid_out = '1' then
        -- Expected ? = ? - ?
        expected := wrap_pi(tests(t).phase_b - tests(t).phase_a);

        -- Convert sfixed_t back to real
        actual := sfixed_to_real(phase_err);

        -- Angular error
        err := abs(ang_err(actual, expected));
        
        report "DBG: atan2(sin,cos) = " & real'image(atan_real) &
               " phase_err (DUT) = " & real'image(z_real) &
               " expected = " & real'image(expected);
              
        report "DBG raw words: cos=" & integer'image(to_integer(cos_delta)) &
               " sin=" & integer'image(to_integer(sin_delta)) &
               " z_out=" & integer'image(to_integer(phase_err));

        if err < TOL then
          report "PASS test " & integer'image(t) &
                 " expected=" & real'image(expected) &
                 " got="      & real'image(actual) &
                 " err="      & real'image(err);
        else
          assert false report
            "FAIL test " & integer'image(t) &
            " expected=" & real'image(expected) &
            " got="      & real'image(actual) &
            " err="      & real'image(err)
          severity error;
        end if;
      else
        assert false report "valid_out was not asserted when expected"
          severity error;
      end if;

      wait for 5*CLK_PERIOD;
    end loop;

    report "All tests finished." severity note;
    wait;
  end process;

end architecture;
