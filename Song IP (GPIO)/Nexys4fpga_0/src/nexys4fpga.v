// nexys4fpga.v - Top level module for Audio logic used in ECE 544 final project
//
// Copyright Roy Kravitz, 2008-2013, 2014, 2015, 2016
// 
// Created By:		Dheeraj chand V
// Last Modified:	6-June-2017 (DJ)
//

//
// Description:
// ------------
// Top level module for the ECE 544 final Project audio module design
// on the Nexys4 FPGA Board (Xilinx XC7A100T-CSG324)

//
// Use the pushbuttons to control the cubix menu:

//	btnCpuReset		CPU RESET Button - System reset.  Asserted low by Nexys 4 board
//
//	sw[1] 		Hourly time announcement button
// SW[0]		current time announcement button
// use Pmod ENC for setting up the time and date.		
//
// External port names match pin names in the nexys4fpga.xdc constraints file
///////////////////////////////////////////////////////////////////////////

module Nexys4fpga (
	input 				clk,                 	// 100MHz clock from on-board oscillator
										// pushbutton inputs - center button -> db_btns[5]
	input				sysreset,
	input            [19:0] playlist_no_wire,		// red pushbutton input -> db_btns[0]
   input alarm_beep_wire,
   input alarm_en_wire,
   input aud_en_wire,
    output              AUD_PWM,AUD_SD   // sound output 
      

); 
    



		
	// global assigns
	assign	sysclk = clk;
	
	
// This module is responsible for alarm tone production
 alarm_ringer(
   .clk(sysclk),
 .alarm_en(alarm_en_wire), 
 .aud_beep(alarm_beep_wire),              
 .system_reset(sysreset),
  .PWM_out_alarm(AUD_PWM_alarm_wire)
);
wire AUD_PWM_voice_wire,AUD_PWM_alarm_wire;
reg aud_pwm_reg;
assign AUD_PWM=aud_pwm_reg;

always@*		// Time announcement and alarm tone preference selection
begin
if(~aud_en_wire)
    aud_pwm_reg<=AUD_PWM_alarm_wire;
   else
    aud_pwm_reg<=AUD_PWM_voice_wire;
end

my_audio audio_test(				// Instantiation for time announcemnet module
	 	   .clk(sysclk),
	 	   .aud_en(aud_en_wire),               
	       .playlist_no(playlist_no_wire),
 		   .PWM_out(AUD_PWM_voice_wire),
 		   .system_reset(sysreset)
);

assign AUD_SD = 1;
wire aud_en_wire;

reg[14:0] int_count;
reg [8:0]int_count_1;
reg int_10khz;
reg int_1khz;
always @(posedge sysclk)
begin
    if(~sysreset)
        begin
       int_count<=15'b0;
       int_10khz<=0;
        end
    else if(int_count==15'd100000)
       begin
          int_count<=15'b0;
          int_10khz<=1;

        end
    else

    begin
         int_count<=int_count+1'b1;
         int_10khz<=0;
 
    end
end



endmodule