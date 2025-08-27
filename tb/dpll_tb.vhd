------------------------------------------------------------------------
-- Project   : Digital Phase-Locked Loop (DPLL)
-- File      : dpll_tb.vhd
-- Author    : Abhishek Garg
-- Created   : 2025
-- Version   : Simulation-Only Release
-- License   : Custom Simulation-Only License (see LICENSE file)
--
-- Description:
--   This file is part of the simulation-verified DPLL project.
--   ⚠️ Not synthesizable. Provided for evaluation and educational use only.
--
-- Notes:
--   - Redistribution or commercial use is prohibited without prior consent.
------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cordic_pkg.all;


entity tb_dpll is
end entity;

architecture sim of tb_dpll is

  constant CLK_PERIOD : time := 10 ns;
  constant EPSILON : real := 0.001;  -- tolerance

  -- DUT signals
  signal clk       : std_logic := '0';
  signal rst       : std_logic := '1';
  signal ce        : std_logic := '1';
  signal ax, ay    : sfixed_t := (others => '0');
  signal sin_out   : sfixed_t;
  signal cos_out   : sfixed_t;
  signal locked    : std_logic;
  -- Delay registers for reference
  type sfixed_array is array (0 to 17) of sfixed_t;
  signal ax_delay, ay_delay : sfixed_array;
  -- Test signal generation
  constant TEST_FREQ     : real := 1.0e6; -- 1 MHz
  constant SAMPLE_FREQ   : real := 100.0e6; -- 100 MHz sampling
  constant TWO_PI        : real := 2.0*math_pi;
  signal sample_count    : integer := 0;
  signal test_phase      : real := 0.0;
  signal dbg_cnt : integer := 0;
  signal phase_err_pd : sfixed_t;
  
begin

  -------------------------------------------------------------------
  -- Clock generation
  -------------------------------------------------------------------
  clk_proc: process
  begin
    while true loop
      clk <= '0';
      wait for CLK_PERIOD/2;
      clk <= '1';
      wait for CLK_PERIOD/2;
    end loop;
  end process;

  -------------------------------------------------------------------
  -- DUT instantiation
  -------------------------------------------------------------------
  dut: entity work.dpll_top
    generic map (
      PHASE_WIDTH  => 32,
      PI_WIDTH     => 32,
      STAGES       => 16
    )
    port map (
      clk       => clk,
      rst       => rst,
      ce        => ce,
      ax        => ax,
      ay        => ay,
      sin_out   => sin_out,
      cos_out   => cos_out,
      locked    => locked,
      dbg_phase_err => phase_err_pd
    );

  -------------------------------------------------------------------
  -- Reset and sample counter
  -------------------------------------------------------------------
  
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '0' and ce = '1' then
        dbg_cnt <= dbg_cnt + 1;
        if dbg_cnt = 1000 then
              report "Phase Error (phase_err_pd) = " &
           integer'image(to_integer(phase_err_pd));
          dbg_cnt <= 0;
        end if;
      end if;
    end if;
  end process;
  
  -- process to shift ref values into delay line
--  process(clk)
--  begin
--    if rising_edge(clk) then
--      ax_delay(0) <= real_to_sfixed(ieee.math_real.cos(test_phase + MATH_PI/2.0), CORDIC_WIDTH);
--      ay_delay(0) <= real_to_sfixed(ieee.math_real.sin(test_phase + MATH_PI/2.0), CORDIC_WIDTH);
--      for k in 1 to 16 loop
--        ax_delay(k) <= ax_delay(k-1);
--        ay_delay(k) <= ay_delay(k-1);
--      end loop;
--    end if;
--  end process;

  
--  stimulus_proc: process
--  begin
--    -- Apply reset
--    rst <= '1';
--    wait for 100 ns;
--    rst <= '0';

--    -- Run simulation for a given number of cycles
--    for i in 0 to 60000 loop
--      wait until rising_edge(clk);

--      -- Increment sample count and generate input signal
--      sample_count <= i;
--      test_phase <= TWO_PI * TEST_FREQ * real(i) / SAMPLE_FREQ;

--      -- Convert to fixed point (sfixed_t) using cordic_pkg function
      
--        ax <= real_to_sfixed(ieee.math_real.cos(test_phase + MATH_PI/2.0), CORDIC_WIDTH);
--        ay <= real_to_sfixed(ieee.math_real.sin(test_phase + MATH_PI/2.0), CORDIC_WIDTH);
         
--    end loop;

--    -- Finish simulation
--    wait for 100 ns;
--    assert false report "Simulation finished" severity failure;
--  end process;
  
  stimulus_proc: process
    variable ref_cycle  : integer := -1;
    variable nco_cycle  : integer := -1;
    variable measured   : boolean := false;
  begin
    -- Apply reset
    rst <= '1';
    wait for 100 ns;
    rst <= '0';
  
    -- Run simulation for a given number of cycles
    for i in 0 to 60000 loop
      wait until rising_edge(clk);
  
      -- Increment sample count and generate input signal
      sample_count <= i;
      test_phase <= TWO_PI * TEST_FREQ * real(i) / SAMPLE_FREQ;
  
      -- Reference input to PD
--      ax <= real_to_sfixed(ieee.math_real.cos(test_phase), CORDIC_WIDTH);
--      ay <= real_to_sfixed(ieee.math_real.sin(test_phase), CORDIC_WIDTH);
        ax_delay(0) <= real_to_sfixed(ieee.math_real.cos(test_phase + MATH_PI/2.0), CORDIC_WIDTH);
       ay_delay(0) <= real_to_sfixed(ieee.math_real.sin(test_phase + MATH_PI/2.0), CORDIC_WIDTH);
       
        for k in 1 to 17 loop
           ax_delay(k) <= ax_delay(k-1);
           ay_delay(k) <= ay_delay(k-1);
        end loop;

          
      -----------------------------------------------------------------
      -- Latency measurement:
      -- Detect first "cos ~ +1, sin ~ 0" event on reference & NCO
      -----------------------------------------------------------------
      if (ref_cycle = -1) then
        if (abs(sfixed_to_real(ax) - 1.0) < EPSILON) and (abs(sfixed_to_real(ay)) < EPSILON) then
          ref_cycle := i;
          report "REF edge at cycle " & integer'image(ref_cycle);
        end if;
      end if;
  
      if (nco_cycle = -1) then
        if (abs(sfixed_to_real(cos_out) - 1.0) < EPSILON) and (abs(sfixed_to_real(sin_out)) < EPSILON) then
          nco_cycle := i;
          report "NCO edge at cycle " & integer'image(nco_cycle);
        end if;
      end if;
  
      if (ref_cycle /= -1) and (nco_cycle /= -1) and not measured then
        report ">>> NCO latency ? = " & integer'image(nco_cycle - ref_cycle) & " cycles <<<";
        measured := true;
      end if;
  
    end loop;
  
    -- Finish simulation
    wait for 100 ns;
    assert false report "Simulation finished" severity failure;
  end process;

  ax <= ax_delay(17);
  ay <= ay_delay(17);


end architecture;
