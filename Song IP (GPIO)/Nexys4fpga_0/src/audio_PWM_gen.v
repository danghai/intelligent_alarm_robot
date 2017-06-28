`timescale 1ns / 1ps
///////////////////////////////////////////////////////////////////////////
// Engineer: Dheeraj chand Vummidi
// 
// Create Date: 6/02/2017 02:36:46 PM
// Design Name:  CUBIX
// Module Name: Audio_PWM
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description:
//This module produces PWM output when we give it a 8 bit audio sample value. This PWM is fed to the onboard LOW pass filter and 
// That module converts the PWM signal to Mono Audio output.  	 
// 
// Revision: V 1.0
module audio_PWM_gen(
	input clk, 				// 100MHz clock.
	input reset,			// system Reset input .
	input [7:0] music_data,	// 8-bit music data
	output reg PWM_out		// PWM output. Connected this pin to FPGA A11 pin
	);
		
//internal variables	
reg [7:0] pwm_counter = 8'd0;     
			  
always @(posedge clk)
begin
	if(reset) begin
		pwm_counter <= 0;
		PWM_out <= 1;
	end
	else begin
		pwm_counter <= pwm_counter + 1;  // Sawtooth wave generation logic
		if(pwm_counter >= music_data)
			PWM_out <= 0; 				// PWM generation logic
		else PWM_out <= 1;
	end
end
endmodule