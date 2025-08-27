------------------------------------------------------------------------
-- Project   : Digital Phase-Locked Loop (DPLL)
-- File      : cordic_stage.vhd
-- Author    : Abhishek Garg
-- Created   : 2025
-- Version   : Simulation-Only Release
-- License   : Custom Simulation-Only License (see LICENSE file)
--
-- Description:
--   This file is part of the simulation-verified DPLL project.
--   ?? Not synthesizable. Provided for evaluation and educational use only.
--
-- Notes:
--   - Redistribution or commercial use is prohibited without prior consent.
------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.cordic_pkg.all;
use work.cordic_pkg.ANGLES;


entity cordic_stage is
  generic (
    ITERATION : integer := 0;
    MODE      : string := "ROTATION"
  );
  port (
    clk   : in  std_logic;
    x_in  : in  sfixed_internal_t;
    y_in  : in  sfixed_internal_t;
    z_in  : in  sfixed_internal_t;
    x_out : out sfixed_internal_t;
    y_out : out sfixed_internal_t;
    z_out : out sfixed_internal_t
  );
end entity;

architecture rtl of cordic_stage is

  signal x_reg, y_reg, z_reg : sfixed_internal_t;
begin

  process(clk)
    variable dx, dy   : sfixed_internal_t;
    variable angle    : sfixed_internal_t;
    variable sigma    : std_logic;
  begin
    if rising_edge(clk) then
      angle := resize(ANGLES(ITERATION), CORDIC_INTERNAL_WIDTH);

      if MODE = "ROTATION" then
        if z_in < SFIXED_ZERO then
          sigma := '1';
        else
          sigma := '0';
        end if;
      else
        if y_in < SFIXED_ZERO then
          sigma := '0';
        else
          sigma := '1';
        end if;
      end if;
      
--          report "MODE=" & MODE &
--             " | sigma=" & std_logic'image(sigma) &
--             " | x_in=" & integer'image(to_integer(x_in)) &
--             " | y_in=" & integer'image(to_integer(y_in)) &
--             " | z_in=" & integer'image(to_integer(z_in));

      dx := ieee.numeric_std.shift_right(y_in, ITERATION);
      dy := ieee.numeric_std.shift_right(x_in, ITERATION);
      

      if sigma = '0' then
        x_reg <= x_in - dx;
        y_reg <= y_in + dy;
        z_reg <= z_in - angle;
      else
        x_reg <= x_in + dx;
        y_reg <= y_in - dy;
        z_reg <= z_in + angle;
      end if;
      
--      report "CORDIC STAGE " & integer'image(ITERATION) &
--                 " | x_in=" & integer'image(to_integer(signed(x_in))) &
--                 " | y_in=" & integer'image(to_integer(signed(y_in))) &
--                 " | z_in=" & integer'image(to_integer(signed(z_in))) &
--                 " | x_out=" & integer'image(to_integer(signed(x_reg))) &
--                 " | y_out=" & integer'image(to_integer(signed(y_reg))) &
--                 " | z_out=" & integer'image(to_integer(signed(z_reg)));
    end if;
  end process;

  -- Register output (optional if latency/pipeline match matters)
  x_out <= x_reg;
  y_out <= y_reg;
  z_out <= z_reg;
    
     -- Debug Print After Update
end architecture;
