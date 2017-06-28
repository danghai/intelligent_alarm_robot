`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 
// Engineer: Dheeraj chand Vummidi
// 
// Create Date: 6/02/2017 05:56:46 PM
// Design Name: CUBIX
// Module Name: Audio_read

// Description:
//	  This module reads the block ROMs containing the music data.Those coe files are prduced using 
//   Matlab script.The .wav music file is downsampled to 8KHZ instead of 44.1KHz to save BROMS.\
//  So, we have to read the data from the BROMs at 8KHZ to produce the music data. 
//  The music data reaad from the BROMs is stored in a register and the data from the register is 
//  being comverted into PWM signal by another module called audio_PWM.v. Its instantiation can be 
// seen in the code.
//  This module produces the speech output when user requests to hear the current time.
// Revision: V 1.0

module my_audio(
	input 			clk,
	input aud_en,               
	input   [19:0]    playlist_no,
	input system_reset,
	output 			PWM_out
);
		
	//internal variables
   reg[19:0]playlist_num,shifted_playlist;
   	reg [14:0] rom1_address;
	reg [32:0] rom2_address;  	//Address declarations for ROM files
	reg [15:0] rom3_address;
	
	
		                             
	reg 	[7:0]	AudioSample;  // Music data input to audio_PWM module
	wire	[7:0] 	tone1,tone2,tone3,tone4; // 8-bit Audio data from different BROMS 
	// Generation of 8khz clock for reading samples of the audio
	//We should use this 8Khz frequency because the audio is being resampled at 8Khz using matlab.
	reg[18:0] divideCounter=0;
	reg clk_8khz=0;
	reg[32:0] voice_rom_begin;
	reg[32:0] voice_rom_end;
	
	parameter integer rom1_depth=11287;	// These corresponds to the data stored BROMS.
	reg [15:0] rom3_depth=16512;
		

	//------state variables -------- 
	parameter reg [2:0] reset_state=3'b000;
	parameter reg [2:0] time_is_state=3'b001; 
	parameter reg [2:0] addr_calc_state=3'b010;
	parameter reg [2:0] op_state=3'b011;
     parameter reg [2:0] ampm_state=3'b101; 
    reg[2:0] current_state,next_state;
    reg [2:0]shift_count=3'b001;
    reg [19:0] number_mask=20'hf0000;
    reg    rom3_flag_pm=1'b0;
    reg    rom3_flag_am=1'b0; 
//---- state flip flop logic---------------	
	always@(posedge clk)
	begin
        if(system_reset==1'b0)
        current_state<=reset_state;
        else
        current_state<=next_state;
    end

 	always@(posedge clk_8khz)
 	
 	begin
 	  if(current_state==reset_state)
 	  begin
 	        if(aud_en)		// enable signal for audio production; 
                begin
             
            
                 rom3_flag_pm=1'b0;
                 rom3_flag_am=1'b0;
                 number_mask=20'hf0000; // reset the number mask
                shift_count=3'b001; 	// shift count reset
                shifted_playlist=19'b0;
                next_state=time_is_state;
                
                end
 	        else
                begin
                next_state<=reset_state;
          
                end        
 	  end
 	 else if (current_state==time_is_state)
 	  begin  
 	       if(rom1_address ==rom1_depth)
               begin
                  rom1_address <=14'b0;
                  next_state<=addr_calc_state;
                end
               else
                  begin    
                   rom1_address <= rom1_address + 14'b01;
                   AudioSample<= tone1;
                   end	      
 	  end
 	 else if (current_state==addr_calc_state)
 	    begin
 	    
 	      shifted_playlist=playlist_no&number_mask;
          shifted_playlist=shifted_playlist>>(20-(4*shift_count));
          number_mask=number_mask>>(4);  
          shift_count=shift_count+1'b1; 
          if(shift_count==3'd6)
          next_state=ampm_state;
          
 	    //--- Audio selection logic. This extracts the numbers from the big COE file --//  
 	    voice_rom_begin=((122047)/13)*(shifted_playlist-1);
        voice_rom_end=((122047)/13)*(shifted_playlist);
        rom2_address=voice_rom_begin;
 	      next_state=op_state;    
 	    end
 	else if(current_state==op_state)
 	   
 	     begin
 	     if(shift_count==6)
          next_state=ampm_state;		// Determine Am/Pm
         if(rom2_address ==voice_rom_end)
         begin
            rom2_address =voice_rom_begin;
            if(shift_count==6)
            next_state=ampm_state;
            else
             next_state=addr_calc_state;
          end
         else
            begin    
             rom2_address <= rom2_address + 32'b01;
             AudioSample<= tone2;
             end
          end
     else if (current_state==ampm_state)
    begin  
        if(shifted_playlist==20'd1 &&rom3_flag_am==1'b0)
                begin
                rom3_depth=8256;
                rom3_flag_am=1'b1;
                rom3_address=15'd0;
                end
        else if(shifted_playlist==20'd2 && rom3_flag_pm==1'b0)   
                begin
                rom3_address=8257;
                rom3_depth=15'd16512;
                 rom3_flag_pm=1'b1;
                end
          else
        rom3_depth=rom3_depth;    
        if(rom3_address ==rom3_depth)
                begin
                   rom3_address <=rom3_depth;
                   next_state<=reset_state;
                   
                 end
                else
                   begin    
                    rom3_address <= rom3_address + 15'b01;
                    AudioSample<= tone3;
                    end          
                  
      end
      
                
    else
     current_state<=current_state;
           
 	end
 	
 











//---------  8 Khz clock generation------------

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
		
	// Instantiation of audio_PWM module
	audio_PWM_gen audioPWM(.clk(clk),.reset(0),.music_data(AudioSample),.PWM_out(PWM_out));
		
			
	// Instantiations of Block ROMS which contain the Music data
			
	sound1 Sound1 (
	  .clka(clk),    // input wire clka
	  .addra(rom1_address),  // input wire [14 : 0] addra
	  .douta(tone1)  // output wire [7 : 0] douta
	);
		
	fullnum fullnum_insta (
	  .clka(clk),    // input wire clka
	  .addra(rom2_address),  // input wire [15 : 0] addra
	  .douta(tone2)
	    // output wire [7 : 0] douta
    );
ampm ampm_insta
	(
	  .clka(clk),
	  .addra(rom3_address),  // input wire [15 : 0] addra
      .douta(tone3)   
	    
	);

  
endmodule

