-- Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2016.2 (win64) Build 1577090 Thu Jun  2 16:32:40 MDT 2016
-- Date        : Tue Jun 06 21:05:41 2017
-- Host        : danghai running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               c:/Users/haidang/Desktop/project2/colorwheel/colorwheel.srcs/sources_1/ip/Nexys4fpga_0/Nexys4fpga_0_stub.vhdl
-- Design      : Nexys4fpga_0
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a100tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity Nexys4fpga_0 is
  Port ( 
    clk : in STD_LOGIC;
    sysreset : in STD_LOGIC;
    playlist_no_wire : in STD_LOGIC_VECTOR ( 19 downto 0 );
    alarm_beep_wire : in STD_LOGIC;
    alarm_en_wire : in STD_LOGIC;
    aud_en_wire : in STD_LOGIC;
    AUD_PWM : out STD_LOGIC;
    AUD_SD : out STD_LOGIC
  );

end Nexys4fpga_0;

architecture stub of Nexys4fpga_0 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk,sysreset,playlist_no_wire[19:0],alarm_beep_wire,alarm_en_wire,aud_en_wire,AUD_PWM,AUD_SD";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "Nexys4fpga,Vivado 2016.2";
begin
end;
