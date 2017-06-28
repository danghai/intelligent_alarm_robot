`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/06/2017 01:49:26 AM
// Design Name:  Alarm ringer module
// Module Name: alarm_ringer
// Project Name: CUBIX	
// Target Devices: Artix -7 series FPGAs
// Tool Versions: vivado 2015.2
// Description: 
//  This module is responsible for the alarm and game sounds. This module sends the PWM signal to the on board butterworth filter amplifier
// The audio will be out from the onboard 3.5 mm jack. It recieves an enable signal from the embeddedsystem GPIO and acts upon that.
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module alarm_ringer(
	input 	clk,
input alarm_en, 
input aud_beep,              
input system_reset,
output     PWM_out_alarm
);
    
//internal variables

reg [31:0] rom1_address;
reg [15:0] rom2_address;      //Address declarations for ROM files
reg [15:0] rom3_address;
                                
reg     [7:0]    AudioSample;  // Music data input to audio_PWM module
wire    [7:0]     tone1,tone2;// 8-bit Audio data from different BROMS 
// Generation of 8khz clock for reading samples of the audio
//We should use this 8Khz frequency because the audio is being resampled at 8Khz using matlab.
reg[18:0] divideCounter=0;
reg clk_8khz=0;
reg[31:0] voice_rom_begin;
reg[31:0] voice_rom_end;

parameter integer rom1_depth=53292;      //beep sound
parameter integer rom2_depth=5439;  //wakeup audio
parameter integer rom3_depth=48002;  //alarm tone
 
always@(posedge clk_8khz)
begin 	
        if(alarm_en)
        begin
        
        if(rom1_address ==rom1_depth)
           begin
              rom1_address <=14'b0;
           end
           else
              begin    
               rom1_address <= rom1_address + 14'b01;
               AudioSample<= tone1;
               end       
        end
      else if(aud_beep)
             begin
             
             if(rom2_address ==rom2_depth)
                begin
                   rom2_address <=14'b0;
                end
                else
                   begin    
                    rom2_address <= rom2_address + 14'b01;
                    AudioSample<= tone2;
                    end       
             end
            else
            AudioSample<=0;    
 end 
always @(posedge clk)
begin
    if (divideCounter == 12500) begin   //8KHZ SIGNAL
        clk_8khz <= 1'b1;
        divideCounter <= 1'b0;
    end
    else begin
       divideCounter <= divideCounter + 1'b1;
        clk_8khz <= 1'b0; 
    end
end 
            
audio_PWM_gen audioPWM(.clk(clk),.reset(0),.music_data(AudioSample),.PWM_out(PWM_out_alarm));
		
			
	// Instantiations of Block ROMS which contain the Music data
	alarm_with_voice alarm_tone(
	
	    .clka(clk),    // input wire clka
        .addra(rom1_address),  // input wire [14 : 0] addra
        .douta(tone1)
	);		

    
	blip_sound blipping_sound (
      .clka(clk),    // input wire clka
      .addra(rom2_address),  // input wire [14 : 0] addra
      .douta(tone2)  // output wire [7 : 0] douta
    );       

    
    
    
    
    
endmodule
