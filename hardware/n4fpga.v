
`timescale 1ns / 1ps

// n4fpga.v - Top level module for the ECE 544 final project
//
// Copyright Roy kravitz, Portland State University, 2016
// 
// Created By: Dheeraj chand V
// Modified By:Surendra maddula
// Modified By: Hai Dang Hoang
// Date:		09-June-2017
// Version:		1.2
//
// Description:
// ------------
// This module implements the top level functionality an interconenctions for the CUBIX project.
// It has all the instantiations required for the project functionality. It interfaces the IO pins and devices of FPGA to the 
// Embeded system
//
//////////////////////////////////////////////////////////////////////
module n4fpga(
    input				clk,			// 100Mhz clock input
    input				btnC,			// center pushbutton
    input				btnU,			// UP (North) pusbhbutton
    input				btnL,			// LEFT (West) pushbutton
    input				btnD,			// DOWN (South) pushbutton  - used for system reset
    input				btnR,			// RIGHT (East) pushbutton
	input				btnCpuReset,	// CPU reset pushbutton
    input	[15:0]		sw,				// slide switches on Nexys 4
    output	[15:0] 		led,			// LEDs on Nexys 4   
    output              RGB1_Blue,      // RGB1 LED (LD16) 
    output              RGB1_Green,
    output              RGB1_Red,
    output              RGB2_Blue,      // RGB2 LED (LD17)
    output              RGB2_Green,
    output              RGB2_Red,
    output [7:0]        an,             // Seven Segment display
    output [6:0]        seg,
    output              dp,             // decimal point display on the seven segment 
   // input               pwm_clk,
  
    // Audio
     output AUD_PWM,
     
     output AUD_SD,  
	inout   [7:0]       JA,             // JA PmodOLED connector 
	                                    // both rows are used 
    inout	[7:0] 		JB,				// JB Pmod connector 
                                        // Unused. Can be used for debuggin purposes 
    inout	[7:0] 		JC,				// JC Pmod connector - debug signals
										// Can be used for debug purposes
	input	[7:0]		JD				// JD Pmod connector - PmodENC signals
);

// internal variables
// Clock and Reset 
wire				sysclk;             // 
wire				sysreset_n, sysreset;

// Rotary encoder 
wire				rotary_a, rotary_b, rotary_press, rotary_sw;

// GPIO pins 
wire	[19:0]	    gpio_in;				// embsys GPIO input port
wire	[3:0]	    gpio_in_2;				// embsys GPIO output port

// OLED pins 
wire 				pmodoledrgb_out_pin1_i, pmodoledrgb_out_pin1_io, pmodoledrgb_out_pin1_o, pmodoledrgb_out_pin1_t; 
wire 				pmodoledrgb_out_pin2_i, pmodoledrgb_out_pin2_io, pmodoledrgb_out_pin2_o, pmodoledrgb_out_pin2_t; 
wire 				pmodoledrgb_out_pin3_i, pmodoledrgb_out_pin3_io, pmodoledrgb_out_pin3_o, pmodoledrgb_out_pin3_t; 
wire 				pmodoledrgb_out_pin4_i, pmodoledrgb_out_pin4_io, pmodoledrgb_out_pin4_o, pmodoledrgb_out_pin4_t; 
wire 				pmodoledrgb_out_pin7_i, pmodoledrgb_out_pin7_io, pmodoledrgb_out_pin7_o, pmodoledrgb_out_pin7_t; 
wire 				pmodoledrgb_out_pin8_i, pmodoledrgb_out_pin8_io, pmodoledrgb_out_pin8_o, pmodoledrgb_out_pin8_t; 
wire 				pmodoledrgb_out_pin9_i, pmodoledrgb_out_pin9_io, pmodoledrgb_out_pin9_o, pmodoledrgb_out_pin9_t; 
wire 				pmodoledrgb_out_pin10_i, pmodoledrgb_out_pin10_io, pmodoledrgb_out_pin10_o, pmodoledrgb_out_pin10_t;

// PMOD RTCC
wire 				pmodrtc_out_pin1_i, pmodrtc_out_pin1_io, pmodrtc_out_pin1_o, pmodrtc_out_pin1_t; 
wire 				pmodrtc_out_pin2_i, pmodrtc_out_pin2_io, pmodrtc_out_pin2_o, pmodrtc_out_pin2_t; 
wire 				pmodrtc_out_pin3_i, pmodrtc_out_pin3_io, pmodrtc_out_pin3_o, pmodrtc_out_pin3_t; 
wire 				pmodrtc_out_pin4_i, pmodrtc_out_pin4_io, pmodrtc_out_pin4_o, pmodrtc_out_pin4_t; 
wire 				pmodrtc_out_pin7_i, pmodrtc_out_pin7_io, pmodrtc_out_pin7_o, pmodrtc_out_pin7_t; 
wire 				pmodrtc_out_pin8_i, pmodrtc_out_pin8_io, pmodrtc_out_pin8_o, pmodrtc_out_pin8_t; 
wire 				pmodrtc_out_pin9_i, pmodrtc_out_pin9_io, pmodrtc_out_pin9_o, pmodrtc_out_pin9_t; 
wire 				pmodrtc_out_pin10_i, pmodrtc_out_pin10_io, pmodrtc_out_pin10_o, pmodrtc_out_pin10_t; 
// RGB LED 



// RGB LED 
wire                w_RGB1_Red, w_RGB1_Blue, w_RGB1_Green;

// LED pins 
wire    [15:0]      led_int;                // Nexys4IO drives these outputs
wire hall_input_1,hall_input_2;
wire enable_wire,direction_wire;
wire [15:0] rpm_out;
wire count_flag_out;
wire [31:0] highcount;
wire [31:0] lowcount;
                // LEDs are driven by led
wire [7:0] gpio_motor_ch_wire;
wire gpio_obstacle_wire;
assign led[7:0]=gpio_motor_ch_wire;


// system-wide signals
assign sysclk = clk;
assign sysreset_n = btnCpuReset;		// The CPU reset pushbutton is asserted low.  The other pushbuttons are asserted high
										// but the microblaze for Nexys 4 expects reset to be asserted low
assign sysreset = ~sysreset_n;			// Generate a reset signal that is asserted high for any logic blocks expecting it.
// 
assign JA[0]=gpio_motor_ch_wire[0]; //This pin goes to Back motor direction
assign JA[1]=gpio_motor_ch_wire[1];// This pin goes to front motor direction
assign JA[2]=gpio_motor_ch_wire[2];// This pin goes to Front motor enable
// This pin goes to Front motor enable
assign JA[3]=gpio_motor_ch_wire[3];//This pin goes to  back motor enable


assign JA[5]=trig_wire; 			// Signal to trigger ultrasonic sensor
assign echo_wire=JA[4]; 			// receive echo from the ultrasonic module
assign JA[6]=gpio_motor_ch_wire[6];//This pin goes to arduino A4 aka control_in_1
assign JA[7]=gpio_motor_ch_wire[7];// This pin goes to arduino A5 aka control_in_2 

// Pmod OLED connections 
assign JB[0] = pmodoledrgb_out_pin1_io;
assign JB[1] = pmodoledrgb_out_pin2_io;
assign JB[2] = pmodoledrgb_out_pin3_io;
assign JB[3] = pmodoledrgb_out_pin4_io;
assign JB[4] = pmodoledrgb_out_pin7_io;
assign JB[5] = pmodoledrgb_out_pin8_io;
assign JB[6] = pmodoledrgb_out_pin9_io;
assign JB[7] = pmodoledrgb_out_pin10_io;

// JA Connector connections  used for RTCC 
assign JC[2]=   pmodrtc_out_pin1_io;
assign JC[3] =  pmodrtc_out_pin2_io;
// JC Connector pins can be used for debug purposes 

wire [19:0]playlist_nowire; // Sound selection wire
wire alarm_beep_wire; 	// game beep sound wire 
wire alarm_en_wire;		// Alarm sound enable wire
wire aud_en_wire,clk_wire; // connecting wires for the sound module

// sound IP instantiation
Nexys4fpga_0 song (
 .clk(clk_wire),
 .sysreset(sysreset_n),
 .playlist_no_wire(gpio_in),
 .alarm_beep_wire(gpio_in_2[0]),
 .alarm_en_wire(gpio_in_2[1]),
 .aud_en_wire(gpio_in_2[2]),
 .AUD_PWM(AUD_PWM));
 assign AUD_SD = 1;

wire clk_50,trig_wire,echo_wire;
// ultrsonic module instantiation
ultra_main ultra_test(  .clk(clk_50),
                        .rst(sysreset),
                        .trig(trig_wire),
                        .echo(echo_wire),
                        .ledt(gpio_obstacle_wire)
                                            
);





// PmodENC signals
// JD - bottom row only
// Pins are assigned such that turning the knob to the right
// causes the rotary count to increment.
assign rotary_a = JD[5];
assign rotary_b = JD[4];
assign rotary_press = JD[6];
assign rotary_sw = JD[7];
wire pwm_clk;


// instantiate the embedded system
embsys EMBSYS
       (// PMOD OLED pins 
        .PmodOLEDrgb_out_pin10_i(pmodoledrgb_out_pin10_i),
	    .PmodOLEDrgb_out_pin10_o(pmodoledrgb_out_pin10_o),
	    .PmodOLEDrgb_out_pin10_t(pmodoledrgb_out_pin10_t),
	    .PmodOLEDrgb_out_pin1_i(pmodoledrgb_out_pin1_i),
	    .PmodOLEDrgb_out_pin1_o(pmodoledrgb_out_pin1_o),
	    .PmodOLEDrgb_out_pin1_t(pmodoledrgb_out_pin1_t),
	    .PmodOLEDrgb_out_pin2_i(pmodoledrgb_out_pin2_i),
	    .PmodOLEDrgb_out_pin2_o(pmodoledrgb_out_pin2_o),
	    .PmodOLEDrgb_out_pin2_t(pmodoledrgb_out_pin2_t),
	    .PmodOLEDrgb_out_pin3_i(pmodoledrgb_out_pin3_i),
	    .PmodOLEDrgb_out_pin3_o(pmodoledrgb_out_pin3_o),
	    .PmodOLEDrgb_out_pin3_t(pmodoledrgb_out_pin3_t),
	    .PmodOLEDrgb_out_pin4_i(pmodoledrgb_out_pin4_i),
	    .PmodOLEDrgb_out_pin4_o(pmodoledrgb_out_pin4_o),
	    .PmodOLEDrgb_out_pin4_t(pmodoledrgb_out_pin4_t),
	    .PmodOLEDrgb_out_pin7_i(pmodoledrgb_out_pin7_i),
	    .PmodOLEDrgb_out_pin7_o(pmodoledrgb_out_pin7_o),
	    .PmodOLEDrgb_out_pin7_t(pmodoledrgb_out_pin7_t),
	    .PmodOLEDrgb_out_pin8_i(pmodoledrgb_out_pin8_i),
	    .PmodOLEDrgb_out_pin8_o(pmodoledrgb_out_pin8_o),
	    .PmodOLEDrgb_out_pin8_t(pmodoledrgb_out_pin8_t),
	    .PmodOLEDrgb_out_pin9_i(pmodoledrgb_out_pin9_i),
	    .PmodOLEDrgb_out_pin9_o(pmodoledrgb_out_pin9_o),
	    .PmodOLEDrgb_out_pin9_t(pmodoledrgb_out_pin9_t),
  // PMOD RTC
          .Pmod_out_pin10_i(pmodrtc_out_pin10_i),
          .Pmod_out_pin10_o(pmodrtc_out_pin10_o),
          .Pmod_out_pin10_t(pmodrtc_out_pin10_t),
          .Pmod_out_pin1_i(pmodrtc_out_pin1_i),
          .Pmod_out_pin1_o(pmodrtc_out_pin1_o),
          .Pmod_out_pin1_t(pmodrtc_out_pin1_t),
          .Pmod_out_pin2_i(pmodrtc_out_pin2_i),
          .Pmod_out_pin2_o(pmodrtc_out_pin2_o),
          .Pmod_out_pin2_t(pmodrtc_out_pin2_t),
          .Pmod_out_pin3_i(pmodrtc_out_pin3_i),
          .Pmod_out_pin3_o(pmodrtc_out_pin3_o),
          .Pmod_out_pin3_t(pmodrtc_out_pin3_t),
          .Pmod_out_pin4_i(pmodrtc_out_pin4_i),
          .Pmod_out_pin4_o(pmodrtc_out_pin4_o),
          .Pmod_out_pin4_t(pmodrtc_out_pin4_t),
          .Pmod_out_pin7_i(pmodrtc_out_pin7_i),
          .Pmod_out_pin7_o(pmodrtc_out_pin7_o),
          .Pmod_out_pin7_t(pmodrtc_out_pin7_t),
          .Pmod_out_pin8_i(pmodrtc_out_pin8_i),
          .Pmod_out_pin8_o(pmodrtc_out_pin8_o),
          .Pmod_out_pin8_t(pmodrtc_out_pin8_t),
          .Pmod_out_pin9_i(pmodrtc_out_pin9_i),
          .Pmod_out_pin9_o(pmodrtc_out_pin9_o),
          .Pmod_out_pin9_t(pmodrtc_out_pin9_t), 
	    // GPIO pins 
        .gpio_0_GPIO_tri_o(gpio_in),
        .gpio_0_GPIO2_tri_o(gpio_in_2),
      
        .gpio_rtl_1_tri_o(gpio_motor_ch_wire),
        .gpio_rtl_2_tri_i(gpio_obstacle_wire),
        
        
    //IP pins
      
        // Pmod Rotary Encoder
	    .pmodENC_A(rotary_a),
        .pmodENC_B(rotary_b),
        .pmodENC_btn(rotary_press),
        .pmodENC_sw(rotary_sw),
        // RGB1/2 Led's 
        .RGB1_Blue(RGB1_Blue),
        .RGB1_Green(RGB1_Green),
        .RGB1_Red(RGB1_Red),
        .RGB2_Blue(RGB2_Blue),
        .RGB2_Green(RGB2_Green),
        .RGB2_Red(RGB2_Red),
        // Seven Segment Display anode control  
        .an(an),
        .dp(dp),
        .led(led_int),
        .seg(seg),
        .clk_out3(clk_wire),
        .clk_out4(clk_50),
        // Push buttons and switches  
        .btnC(btnC),
        .btnD(btnD),
        .btnL(btnL),
        .btnR(btnR),
        .btnU(btnU),
        .sw(sw),
        // reset and clock 
        .sysreset_n(sysreset_n),
        .sysclk(sysclk),
        //Motor pins
        .enable(enable_wire),
        .motor_direction_out(direction_out_wire),
        .hall_input(hall_input_1),
        .hall_sensor_in_2(hall_input_2),
        
        // UART pins 
        .uart_rtl_rxd(uart_rtl_rxd),
        .uart_rtl_txd(uart_rtl_txd));
        
// Tristate buffers for the pmodOLEDrgb pins
// generated by PMOD bridge component.  Many
// of these signals are not tri-state.
IOBUF pmodoledrgb_out_pin1_iobuf
(
    .I(pmodoledrgb_out_pin1_o),
    .IO(pmodoledrgb_out_pin1_io),
    .O(pmodoledrgb_out_pin1_i),
    .T(pmodoledrgb_out_pin1_t)
);

IOBUF pmodoledrgb_out_pin2_iobuf
(
    .I(pmodoledrgb_out_pin2_o),
    .IO(pmodoledrgb_out_pin2_io),
    .O(pmodoledrgb_out_pin2_i),
    .T(pmodoledrgb_out_pin2_t)
);

IOBUF pmodoledrgb_out_pin3_iobuf
(
    .I(pmodoledrgb_out_pin3_o),
    .IO(pmodoledrgb_out_pin3_io),
    .O(pmodoledrgb_out_pin3_i),
    .T(pmodoledrgb_out_pin3_t)
);

IOBUF pmodoledrgb_out_pin4_iobuf
(
    .I(pmodoledrgb_out_pin4_o),
    .IO(pmodoledrgb_out_pin4_io),
    .O(pmodoledrgb_out_pin4_i),
    .T(pmodoledrgb_out_pin4_t)
);

IOBUF pmodoledrgb_out_pin7_iobuf
(
    .I(pmodoledrgb_out_pin7_o),
    .IO(pmodoledrgb_out_pin7_io),
    .O(pmodoledrgb_out_pin7_i),
    .T(pmodoledrgb_out_pin7_t)
);

IOBUF pmodoledrgb_out_pin8_iobuf
(
    .I(pmodoledrgb_out_pin8_o),
    .IO(pmodoledrgb_out_pin8_io),
    .O(pmodoledrgb_out_pin8_i),
    .T(pmodoledrgb_out_pin8_t)
);

IOBUF pmodoledrgb_out_pin9_iobuf
(
    .I(pmodoledrgb_out_pin9_o),
    .IO(pmodoledrgb_out_pin9_io),
    .O(pmodoledrgb_out_pin9_i),
    .T(pmodoledrgb_out_pin9_t)
);

IOBUF pmodoledrgb_out_pin10_iobuf
(
    .I(pmodoledrgb_out_pin10_o),
    .IO(pmodoledrgb_out_pin10_io),
    .O(pmodoledrgb_out_pin10_i),
    .T(pmodoledrgb_out_pin10_t)
);
// Tristate buffers for the pmodrtc pins
// generated by PMOD bridge component.  Many
// of these signals are not tri-state.
IOBUF pmodrtc_out_pin1_iobuf
(
    .I(pmodrtc_out_pin1_o),
    .IO(pmodrtc_out_pin1_io),
    .O(pmodrtc_out_pin1_i),
    .T(pmodrtc_out_pin1_t)
);

IOBUF pmodrtc_out_pin2_iobuf
(
    .I(pmodrtc_out_pin2_o),
    .IO(pmodrtc_out_pin2_io),
    .O(pmodrtc_out_pin2_i),
    .T(pmodrtc_out_pin2_t)
);

IOBUF pmodrtc_out_pin3_iobuf
(
    .I(pmodrtc_out_pin3_o),
    .IO(pmodrtc_out_pin3_io),
    .O(pmodrtc_out_pin3_i),
    .T(pmodrtc_out_pin3_t)
);

IOBUF pmodrtc_out_pin4_iobuf
(
    .I(pmodrtc_out_pin4_o),
    .IO(pmodrtc_out_pin4_io),
    .O(pmodrtc_out_pin4_i),
    .T(pmodrtc_out_pin4_t)
);

IOBUF pmodrtc_out_pin7_iobuf
(
    .I(pmodrtc_out_pin7_o),
    .IO(pmodrtc_out_pin7_io),
    .O(pmodrtc_out_pin7_i),
    .T(pmodrtc_out_pin7_t)
);

IOBUF pmodrtc_out_pin8_iobuf
(
    .I(pmodrtc_out_pin8_o),
    .IO(pmodrtc_out_pin8_io),
    .O(pmodrtc_out_pin8_i),
    .T(pmodrtc_out_pin8_t)
);

IOBUF pmodrtc_out_pin9_iobuf
(
    .I(pmodrtc_out_pin9_o),
    .IO(pmodrtc_out_pin9_io),
    .O(pmodrtc_out_pin9_i),
    .T(pmodrtc_out_pin9_t)
);

IOBUF pmodrtc_out_pin10_iobuf
(
    .I(pmodrtc_out_pin10_o),
    .IO(pmodrtc_out_pin10_io),
    .O(pmodrtc_out_pin10_i),
    .T(pmodrtc_out_pin10_t)
);


endmodule

