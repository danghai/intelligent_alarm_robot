
/**
*
* @file ece544periph_test.c
*
* @author Roy Kravitz (roy.kravitz@pdx.edu)
* @Modified by Dheerajchand V(dheeraj@pdx.edu)
* @Modified by Hai Dang Hoang (danghai@pdx.edu)
* @Modidied by surendra maddula 
* Last date file touched : June 09th 2017
*
*Version :1.0
* Description:
*This file implements the required functioning for the main project of ECE 544
* This has the GUI core logic and Navigational algorithm implemnetation
* This take the inputs from the GPIO, push buttons and makes a decision what to do
* This application also reads the date, time information for RTCC. 
*
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "PmodRTCC.h"
#include "xil_types.h"
#include "xparameters.h"
#include "xstatus.h"
#include "nexys4IO.h"
#include "pmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "PmodOLEDrgb.h"
#include "picture.h"
#ifdef XPAR_MICROBLAZE_ID
#include "microblaze_sleep.h"
#else
#include "sleep.h"
#endif


/************************** Constant Definitions ****************************/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_S00_AXI_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_OUTPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_1_CHANNEL		2

// GPIO parameters
#define GPIO_2_DEVICE_ID			XPAR_AXI_GPIO_2_DEVICE_ID
#define GPIO_2_OUTPUT_0_CHANNEL		1
#define GPIO_2_INPUT_CHANNEL		2


// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
#define Extract_red_dutycycle	0x000000ff
#define Extract_green_dutycycle	0x0000ff
#define Extract_blue_dutycycle  0x00ff

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
unsigned long timeStamp = 0;


/************************** Function Prototypes *****************************/
void usleep(u32 usecs);

void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);

void FIT_Handler(void);										
void project_1(void);
int do_init_nx4io(u32 BaseAddress);
int do_init_pmdio(u32 BaseAddress);
int AXI_Timer_initialize(void);
int do_init();
u8* Custom_OLEDrgb_BuildHSV(uint8_t hue, uint8_t sat, uint8_t val);
u32 red_0_to_1=3,red_1_to_0;

// Function for time
void printTime(u8 src);
void system_init();
void setting_time();
int dec_to_bcd (int);
void hex_to_day(int);
void hex_to_month(int);
void set_alarm(u8 src,int flag_alarm);

// Function for display
void background();
void delete_pixel(int row, int col);
void display_selection(int position);


// Function for game logic
int diff = 2;
int grade = 0;
void game_on();
int game_screen_1(int *pass);
int game_screen_2(int *pass);
int game_screen_3(int *pass);
int game_screen_4(int *pass);
int game_screen_5(int *pass);
void result_game(int, int *pass);

/*******************************************************************/
// Function for Navigation robot
u8 motor_running_flag = 0;
u8 path;
u8 sensor;
u16 compass = 0;

void set_servo_postion(u8 position);
void move_motor(u8 direction_and_enable);
void move_forward();
void move_back();
void move_right();
void move_left();
void stop_motor();
u8 look_for_path();
void navigation_algorithm();

// Function for Song
void speak_current_time();
void speak_hour();


PmodENC 	pmodENC_inst;				// PmodENC instance ref
PmodOLEDrgb	pmodOLEDrgb_inst;			// PmodOLED instance ref
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


XGpio		GPIOInst0,GPIOInst1,GPIOInst2,GPIOInst3;		// GPIO instance

uint8_t 	R=0,G=0,B=0;

// variable for time
PmodRTCC myDevice;
int year = 0, mon= 0, date= 0, day= 0, hour=0, minute=0, sec= 0, ampm =0;
int pre_year, pre_mon, pre_date, pre_day, pre_hour, pre_minute, pre_sec, pre_ampm =0;

// Variable for control OledRGB screen
int count_position = 0;
int flag_set = 0, flag_show = 0, flag_background =0, flag_back = 0, flag_alarm1=0, flag_alarm2 = 0;
int flag_game = 0;

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so its frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()
volatile unsigned int	clkfit;					// clock signal is bit[0] (rightmost) of gpio 0 output port


volatile u32			gpio_in_2;				// GPIO input port
volatile u8				gpio_in;
volatile u32			high_value_sw_red=0;	// Value of high count from software
volatile u32  			low_value_sw_red=0;		// Value of low count from software
volatile u32			high_value_sw_blue=0;	// Value of high count from software
volatile u32  			low_value_sw_blue=0;	// Value of low count from software
volatile u32			high_value_sw_green=0;	// Value of high count from software
volatile u32  			low_value_sw_green=0;	// Value of low count from software

volatile u32 			count_high_red=0;		// Initializing the count_high value to 0
volatile u32			count_low_red=0;		// Initializing the count_low value to 0
volatile u32			old_pwm_red=0;			// Store the old pwm value to enable detection of rising or falling edge
volatile u32 			count_high_blue=0;		// Initializing the count_high value to 0
volatile u32			count_low_blue=0;		// Initializing the count_low value to 0
volatile u32			old_pwm_blue=0;			// Store the old pwm value to enable detection of rising or falling edge
volatile u32 			count_high_green=0;		// Initializing the count_high value to 0
volatile u32			count_low_green=0;		// Initializing the count_low value to 0
volatile u32			old_pwm_green=0;		// Store the old pwm value to enable detection of rising or falling edge

volatile u32			pwm_red=0;				// Store the current red pwm value
volatile u32			pwm_blue=0;				// Store the current blue pwm value
volatile u32			pwm_green=0;			// Store the current green pwm value
u16 					sw = 0;
u16  					RotaryCnt,old_RotaryCnt=0xffff;
uint64_t 				timestamp = 0L;			// used in delay msec
u8						reset_rotary=255;
u32						pwm_counter=0;
volatile bool 		   red_one_flag=0,red_zero_flag=0;
volatile bool 		   green_one_flag=0,green_zero_flag=0;
volatile bool 		   blue_one_flag=0,blue_zero_flag=0;
volatile u8 			hw_dutycycle_red,hw_dutycycle_green,hw_dutycycle_blue;
volatile u8 			sw_dutycycle_red=0,sw_dutycycle_red_new=0,sw_dutycycle_red_old=0,sw_dutycycle_green=0,sw_dutycycle_blue=0;
/************************** MAIN PROGRAM ************************************/
int  RotaryIncr;
RotaryIncr = 1;
int enc_count;
bool RotaryNoNeg = false;

int main()
{
	int sts;
    init_platform();

    sts = do_init();		// initialize the peripherals
    if (XST_SUCCESS != sts)
    {
    	xil_printf("FAL!");
    	exit(1);
    }

    microblaze_enable_interrupts();
    pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);// enable the interrupts

    background(); //  Back ground for beginning

    while(1)
    {
		/* Logic control the Menu 
			0 row position --> Set time
			2 row position --> Show time
			4 row position --> Set Alarm1
			6 row position --> Set Alarm2:*/
    	if(NX4IO_isPressed(BTND))
    	{
    		delete_pixel(0,count_position);
    		if(count_position ==6)
    			count_position = -2;
    		count_position = count_position + 2;
    		display_selection(count_position);

    	}
    	if(NX4IO_isPressed(BTNU))
    	{
    		delete_pixel(0,count_position);
    		if (count_position == 0)
    			count_position = 8;
    	    count_position = count_position - 2;
    	    display_selection(count_position);

    	}
    	if(NX4IO_isPressed(BTNC)) 				// Confirm button and generate the flag for each activity
    	{
    		if(count_position == 0)				// Go to set time
    		{
    			flag_set = 1;
    			flag_show = 0;
    			flag_background = 0;
    			flag_alarm1 =0;
    			flag_alarm2 =0;
    		}
    		if (count_position == 2)			// Go to show time
    		{
    			flag_set = 0;
    			flag_show = 1;
    			flag_background = 0;
    			flag_alarm1 = 0;
    			flag_alarm2 =0;
    		}
    		if (count_position == 4)			// Go to Set alarm 1
    		{
    			flag_set = 0;
    			flag_show = 0;
    			flag_background = 0;
    			flag_alarm1 = 1;
    			flag_alarm2 =0;
    		}
    		if (count_position == 6) 			// Go to Set alarm 2
    		{
    		    flag_set = 0;
    		    flag_show = 0;
    		    flag_background = 0;
    		    flag_alarm1 = 0;
    		    flag_alarm2 =1;
    		}
    	}
    	// Logic operation out side menu
    	if(flag_set == 1)
    		setting_time();
    	if(flag_show == 1)
    	{
    		OLEDrgb_Clear(&pmodOLEDrgb_inst);
    		printTime(RTCC_RTCC);
    	}

    	if (flag_background == 1)
    		background();

    	if(flag_alarm1 == 1)
    		set_alarm(RTCC_ALM0,flag_alarm1);
    	if(flag_alarm2 == 1)
    	   set_alarm(RTCC_ALM1,flag_alarm2);


    	if((pmodENC_is_switch_on(&pmodENC_inst)) || (flag_game == 1))
    	{
    			game_on();
    	}

        // speak current time
        if (NX4IO_getSwitches()==1)
        	speak_current_time();
        else
        	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x0);  // turn off song


    } // end while(1)

    cleanup_platform();
    exit(0);
}

/*
 *  Function Background: display the menu for setting
 *  The Menu has format
 ************************
 *	> Set Time			*
 *    Show Time 		*
 *	  Set Alarm 1		*
 *	  Set Alarm 2		*
 ***********************/
void background()
{
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, count_position);
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '>');
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255)); // Set font Blue

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 0);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Set Time");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 2);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Show Time");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 4);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Set Alarm 1");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 6);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Set Alarm 2");
	flag_background = 0;
}

/*
 *  Function delete_pixel: Delete a pixel with row and col corresponding
 */
void delete_pixel(int row,int col)
{
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(0,0,0)); // Black color for delete
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, row, col);
	OLEDrgb_PutChar(&pmodOLEDrgb_inst , 'T');

}

/*
 * Function display_selection: display update the selection
 */
void display_selection(int position)
{
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, position);
	OLEDrgb_PutChar(&pmodOLEDrgb_inst, '>');
	usleep(100000);
}

/**
 * Function Name: do_init()
 *
 * Return: XST_FAILURE or XST_SUCCESS
 *
 * Description: Initialize the AXI timer, gpio, interrupt, FIT timer, Encoder,
 * 				OLED display
 */
int do_init()
{
	int sts;
	int status;

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (sts == XST_FAILURE)
	{
		exit(1);
	}

	// initialize the PMod544IO driver and the PmodENC and PmodCLP
	status = pmodENC_initialize(&pmodENC_inst, PMODENC_BASEADDR);
	if (sts == XST_FAILURE)
	{
		exit(1);
	}

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the GPIO 2 instances
	status = XGpio_Initialize(&GPIOInst3, GPIO_2_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}


	// Initialize the AXI Timer
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	// Initialize the OLED display
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	/*status = XGpio_Initialize(&GPIOInst1, GPIO_0_DEVICE_ID);
		if (status != XST_SUCCESS)
		{
			return XST_FAILURE;
		}*/
	// GPIO0 channel 1 is an 20-bit input port.
	// GPIO0 channel 2 is an 4-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x00);

	XGpio_SetDataDirection(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL, 0x00);
	XGpio_SetDataDirection(&GPIOInst3, GPIO_2_INPUT_CHANNEL, 0xFF);

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 2070);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

void project_1(void)

{

//Variable declarations and definitions
	int  RotaryIncr;
	u32 RGB_1_data;
	u8 hue=0,sat=0,val=0;
	bool RotaryNoNeg,disp_flag=1 ;
	RotaryIncr = 1;
	RotaryNoNeg = false;
	int last_hue=0,last_sat=0,last_val=0;
//Rotation_counter and RGB LED initialization
	pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
//Writing the strings to OLED_Display
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildHSV(169,255,255));
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Hue:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0,3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Sat:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0,5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Val:");


//Starting Application main loop
//It is an infinite loop as the embedded systems never quit ..

  while(1){


	  pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
	  if(old_RotaryCnt!=RotaryCnt)

	   {
		  hue=(u8)RotaryCnt;
		  old_RotaryCnt=RotaryCnt;

	   }

	   if (NX4IO_isPressed(BTNR))
		{

			sat +=1;

		  usleep(110000);
		}
	     if (NX4IO_isPressed(BTNL))
		{

				 sat -=1;

		  usleep(110000);
		}

	     if (NX4IO_isPressed(BTNU))
		{

			 val +=1;
			usleep(110000);

		}

	     if (NX4IO_isPressed(BTND))
		{

			 val -=1;
			usleep(110000);
		}
	     //Setting up the display output


	     if(disp_flag==true)
	     {
	    	 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
	    	 PMDIO_putnum(&pmodOLEDrgb_inst,hue, 10);
	    	 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4,3);
	         PMDIO_putnum(&pmodOLEDrgb_inst,sat, 10);
	         OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4,5);
	         PMDIO_putnum(&pmodOLEDrgb_inst,val, 10);
	         disp_flag=false;

	     }
	     if(( last_hue!= hue|| last_sat!= sat ||last_val!=val)&& disp_flag==false)

	     {

				 if(last_hue!=hue)
					 {
						 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);		// reset the cursor to the location after "Enc:"
						 OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
					 }
				 if(last_sat!=sat)
					 {
						 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);		// reset the cursor to the location after "Enc:"
						 OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
					 }
				 if(last_val!=val)
					 {
						 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);		// reset the cursor to the location after "Enc:"
						 OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
					 }
				 disp_flag=true;
				 last_hue=hue;
				 last_sat=sat;
				 last_val=val;



	     }
	     //This part deals with Rectangle and RGB LEDs
	      OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,65,5,90,60,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildHSV(hue,sat,val));

		  R=OLEDrgb_ExtractRFromRGB(OLEDrgb_BuildHSV(hue,sat,val));

		  G=OLEDrgb_ExtractGFromRGB(OLEDrgb_BuildHSV(hue,sat,val));
		  B=OLEDrgb_ExtractBFromRGB(OLEDrgb_BuildHSV(hue,sat,val));
		  if(sat>0 || val>0 )
				  {
					  R=(R*8)+7;
					  G=(G*8)+7;
					  B=(B*8)+7;
				  }
		  else
		  {
			  R=0;
			  G=0;
			  B=0;
		  }

		  NX4IO_RGBLED_setDutyCycle(RGB1,R,G,B);
		  NX4IO_RGBLED_setDutyCycle(RGB2,R,G,B);

  /*	 masking the input GPIO and deriving the dutycycles*/




     // Displaying the Duty cycle on segments//

		sw=NX4IO_getSwitches();	//check the status of the switch[0] to decide the source
		  if(sw&0x0001==0x0001)
		{
			NX4IO_setLEDs(0x00001);//Light up the LED[0] if HW source is selected
		  NX4IO_SSEG_setDigit(2,DIGIT7,(hw_dutycycle_red)/10);
		  NX4IO_SSEG_setDigit(2,DIGIT6,(hw_dutycycle_red)%10);
		  NX4IO_SSEG_setDigit(2,DIGIT5,	CC_BLANK);
		  NX4IO_SSEG_setDigit(2,DIGIT4,(hw_dutycycle_green)/10);
		  NX4IO_SSEG_setDigit(1,DIGIT3,(hw_dutycycle_green)%10);
		  NX4IO_SSEG_setDigit(1,DIGIT2,	CC_BLANK);
		  NX4IO_SSEG_setDigit(1,DIGIT1,(hw_dutycycle_blue)/10);
		  NX4IO_SSEG_setDigit(1,DIGIT0,(hw_dutycycle_blue)%10);
		}
		else
		{
			NX4IO_setLEDs(0x00000);//Turn off the LED[0] if SW source is selected
		  NX4IO_SSEG_setDigit(2,DIGIT7,(sw_dutycycle_red)/10);
		  NX4IO_SSEG_setDigit(2,DIGIT6,(sw_dutycycle_red)%10);
		  NX4IO_SSEG_setDigit(2,DIGIT5,	CC_BLANK);
		  NX4IO_SSEG_setDigit(2,DIGIT4,(sw_dutycycle_green)/10);
		  NX4IO_SSEG_setDigit(1,DIGIT3,(sw_dutycycle_green)%10);
		  NX4IO_SSEG_setDigit(1,DIGIT2,	CC_BLANK);
		  NX4IO_SSEG_setDigit(1,DIGIT1,(sw_dutycycle_blue)/10);
		  NX4IO_SSEG_setDigit(1,DIGIT0,(sw_dutycycle_blue)%10);
			}


   xil_printf("red ones counter=%d and zeros =%d\n",count_high_red,count_low_red);
   xil_printf("duty cycle is =%d\n",sw_dutycycle_red_old);
   xil_printf("duty cycle of green is =%d\n",sw_dutycycle_green);
   // Derive the Dutycycles using SW counts.

  }
  }




/********************** HELPER FUNCTIONS ***********************************/

/****************************************************************************/
/**
* insert delay (in microseconds) between instructions.
*
* This function should be in libc but it seems to be missing.  This emulation implements
* a delay loop with (really) approximate timing; not perfect but it gets the job done.
*
* @param	usec is the requested delay in microseconds
*
* @return	*NONE*
*
* @note
* This emulation assumes that the microblaze is running @ 100MHz and takes 15 clocks
* per iteration - this is probably totally bogus but it's a start.
*
*****************************************************************************/

static const u32	DELAY_1US_CONSTANT	= 15;	// constant for 1 microsecond delay

void usleep(u32 usec)
{
	volatile u32 i, j;

	for (i = 0; i < usec; i++)
	{
		for (j = 0; j < DELAY_1US_CONSTANT; j++);
	}
	return;
}


/****************************************************************************/
/**
* initialize the Nexys4 LEDs and seven segment display digits
*
* Initializes the NX4IO driver, turns off all of the LEDs and blanks the seven segment display
*
* @param	BaseAddress is the memory mapped address of the start of the Nexys4 registers
*
* @return	XST_SUCCESS if initialization succeeds.  XST_FAILURE otherwise
*
* @note
* The NX4IO_initialize() function calls the NX4IO self-test.  This could
* cause the program to hang if the hardware was not configured properly
*
*****************************************************************************/
int do_init_nx4io(u32 BaseAddress)
{
	int sts;

	// initialize the NX4IO driver
	sts = NX4IO_initialize(BaseAddress);
	if (sts == XST_FAILURE)
		return XST_FAILURE;

	// turn all of the LEDs off using the "raw" set functions
	// functions should mask out the unused bits..something to check w/
	// the debugger when we bring the drivers up for the first time
	NX4IO_setLEDs(0xFFF0000);
	NX4IO_RGBLED_setRGB_DATA(RGB1, 0xFF000000);
	NX4IO_RGBLED_setRGB_DATA(RGB2, 0xFF000000);
	NX4IO_RGBLED_setRGB_CNTRL(RGB1, 0xFFFFFFF0);
	NX4IO_RGBLED_setRGB_CNTRL(RGB2, 0xFFFFFFFC);

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	return XST_SUCCESS;

}


/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion,
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;

  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;

    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}


/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO_0 port which reads back the hardware generated PWM wave for the RGB Leds
*Reads the GPIO_1 port which reads back the hardware generated Dutycycle for the RGB Leds

 *****************************************************************************/

void FIT_Handler(void)
{
	
	  gpio_in= XGpio_DiscreteRead(&GPIOInst0,1);
	  //gpio_in_2 = XGpio_DiscreteRead(&GPIOInst1,2);

	  
		// Read the ultrasonic sensor data
	  sensor = XGpio_DiscreteRead(&GPIOInst3,GPIO_2_INPUT_CHANNEL);
	  	u16 temp_compass;
	  	temp_compass = compass;
	  	//To be displayed on Digits 7,6,5,4.
	  	NX4IO_SSEG_setDigit(2,DIGIT4,temp_compass%10);
	  	temp_compass = temp_compass/10;
	  	NX4IO_SSEG_setDigit(2,DIGIT5,temp_compass%10);
	  	temp_compass = temp_compass/10;
	  	NX4IO_SSEG_setDigit(2,DIGIT6,temp_compass%10);
	  	temp_compass = temp_compass/10;
	  	NX4IO_SSEG_setDigit(2,DIGIT7,0);

}
/*
 * Function dec_to_bcd : convert a decimal number to bcd
 */
int dec_to_bcd(int val)
{
	int bcdResult = 0;
	int shift = 0;
	while (val > 0) {
	      bcdResult |= (val% 10) << (shift++ << 2);
	      val /= 10;
	}
	return bcdResult;
}
/*
 *  printTime function: will print time on the RGB and SDK Terminal for debugging
 */
void printTime(u8 src)
{
   while (flag_show)
   {
	   OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,0,0,95,63,OLEDrgb_BuildRGB(255,51,0),false,OLEDrgb_BuildRGB(255,0,0));
	sec = RTCCI2C_getSec(src, &myDevice);
	minute = RTCCI2C_getMin(src, &myDevice);
	hour = RTCCI2C_getHour(src, &myDevice);
	ampm = RTCCI2C_getAmPm(src, &myDevice);
	day = RTCCI2C_getDay(src, &myDevice);
	date = RTCCI2C_getDate(src, &myDevice);
	mon = RTCCI2C_getMonth(src, &myDevice);
	year = RTCCI2C_getYear(&myDevice);

	 // print all parameter of the src
	xil_printf("%02x", mon);
	xil_printf("/");
	xil_printf("%02x", date);
	//year is only available for the RTCC
	if( src == RTCC_RTCC)
	{
		xil_printf("/");
		xil_printf("%02x", year);
	}
	xil_printf(" day");
	xil_printf("%02x", day);
	xil_printf(" ");
	xil_printf("%02x", hour);
	xil_printf(":");
	xil_printf("%02x", minute);

//second is not supported by the power fail registers
    if( src != RTCC_PWRD && src != RTCC_PWRU)
    {
    	xil_printf(":");
    	xil_printf("%02x", sec);
    }

    if(ampm)
    {
    	xil_printf(" PM");
    }
    else
    {
    	xil_printf(" AM");
    }
    // speak current time
    if (NX4IO_getSwitches()==1)
    	speak_current_time();
    // speak hour
    else if (NX4IO_getSwitches()==2)
    	speak_hour();
    else
    	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x0);  // turn off song
// Print time on PmodOLEDrgb
    OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
    OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));  // blue font

    if ((hour != pre_hour)||(flag_back ==1))
    {
    	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");	// Hour
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);
    	if(hour <= 9)
    		OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");

    	PMDIO_putnum(&pmodOLEDrgb_inst, hour,16);
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
    	OLEDrgb_PutString(&pmodOLEDrgb_inst,":");
    	pre_hour = hour;

    }

    if ((minute != pre_minute)||(flag_back ==1))
    {
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1); // Minute
    	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
    	if(minute <= 9)
    	    OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
    	PMDIO_putnum(&pmodOLEDrgb_inst, minute,16);
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
    	OLEDrgb_PutString(&pmodOLEDrgb_inst,":");
    	pre_minute = minute;

    }

    if  ((sec != pre_sec)||(flag_back ==1))
    {
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 8, 1); // Second
    	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
    	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 8, 1);
    	if(sec <= 9)
    	   OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
    	PMDIO_putnum(&pmodOLEDrgb_inst, sec,16);
    	pre_sec  = sec;
    //OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 9, 0);
    }
    OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(218, 178,76));

    if  ((day != pre_day)||(flag_back ==1))
        {
        	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 3); // day
        	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
        	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 3);

        	hex_to_day(day);
        	pre_day  = day;
        //OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 9, 0);
        }

    OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
    if  ((date != pre_date)||(flag_back ==1))
       {
       	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 5); // date
       	OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
       	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 5);
       	if(date <= 9)
       	   OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
       	PMDIO_putnum(&pmodOLEDrgb_inst, date,16);
       	OLEDrgb_PutString(&pmodOLEDrgb_inst,"/");
       	pre_date  = date;
       //OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 9, 0);
       }

    if  ((mon != pre_mon)||(flag_back ==1))
       {
       	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5); // month
       	OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
       	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
       	if(mon <= 9)
       	   OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
       	PMDIO_putnum(&pmodOLEDrgb_inst, mon,16);
    	OLEDrgb_PutString(&pmodOLEDrgb_inst,"/");
       	pre_mon  = mon;
       //OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 9, 0);
       }

    if  ((year != pre_year)||(flag_back ==1))
       {
       	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 5); // year
       	OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
       	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 5);
       	OLEDrgb_PutString(&pmodOLEDrgb_inst,"20");
       	if(year <= 9)
       	   OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
       	PMDIO_putnum(&pmodOLEDrgb_inst, year,16);
       	pre_year  = year;
       //OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 9, 0);
       }

    flag_back = 0;
    if(NX4IO_isPressed(BTNL))
    {
        	flag_show = 0;
        	flag_background = 1;
        	flag_back = 1;
    }
    // ALARM///////////////////////////
    if(RTCCI2C_checkFlag(RTCC_ALM0, &myDevice))
    {
    	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x2);  	 // Speak song for alarm
    	RTCCI2C_disableAlarm(RTCC_ALM0, &myDevice);
    	flag_game = 1;
    	flag_show = 0;
    }

    if(RTCCI2C_checkFlag(RTCC_ALM1, &myDevice))
    {
    	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x2);  	 // Speak song for alarm
        	RTCCI2C_disableAlarm(RTCC_ALM1, &myDevice);
        	flag_game = 1;
        	flag_show = 0;
    }
    ////////////////////////////////////

#ifdef XPAR_MICROBLAZE_ID
		MB_Sleep(1500);
#else
		usleep(1000000);
#endif

   }
}
void MB_Sleep_1(u32 MilliSeconds)
{
	if (((mfmsr() & 0x20U) == 0U)) {
		/*
		 * Instruction cache not enabled.
		 * Delay will be much higher than expected.
		 */
	}

	asm volatile ("\n"
			"1:               \n\t"
			"addik r7, r0, %0 \n\t"
			"2:               \n\t"
			"addik r7, r7, -1 \n\t"
			"bneid  r7, 2b    \n\t"
			"or  r0, r0, r0   \n\t"
			"bneid %1, 1b     \n\t"
			"addik %1, %1, -1 \n\t"
			:: "i"(ITERS_PER_MSEC), "d" (MilliSeconds));

}


/*
 *	Setting time function
 */
void setting_time()
{
		// Initial RTCC														// Clear OLED RGB
	    OLEDrgb_Clear(&pmodOLEDrgb_inst);
	    OLEDrgb_DrawBitmap(&pmodOLEDrgb_inst,0,0,95,55,(uint8_t*)clock);	// Draw 24-bit map image
	    usleep(500000);
	    OLEDrgb_Clear(&pmodOLEDrgb_inst);

		RTCC_begin(&myDevice, XPAR_PMODRTCC_0_AXI_LITE_IIC_BASEADDR, 0x6F); // Initial Pmod RTCC 
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 0);
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));  // blue font
		// Stop Clock for setting
		RTCCI2C_stopClock(&myDevice);

		 // Set hour
		 OLEDrgb_PutString(&pmodOLEDrgb_inst,"Hour:");	// Hour
		 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
		 OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
		 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
		 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
		 while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
		 {
			 // interface with Pmod ENC
			 pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
			 if(old_RotaryCnt!=RotaryCnt)
			 {
				 	  if(RotaryCnt > old_RotaryCnt)
				 	  {
				 		  hour++;
				 		  if(hour>24) hour = 0;
				 	  }
				 	  else
				 	  {
				 		  hour--;
				 		  if(hour<0) hour = 23;

				 	  }
					  old_RotaryCnt=RotaryCnt;
			 }

			 // interface with Button
			 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
			 if(NX4IO_isPressed(BTNU))
			 {
				 hour++;
				 if(hour > 24)
					 hour = 0;
			 }
			 if(NX4IO_isPressed(BTND))
			 {
				 if(hour <= 0)
					hour =23;
				 else
					hour--;
			 }
			 if(hour <= 9)
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
			 PMDIO_putnum(&pmodOLEDrgb_inst, hour,10);
			 usleep(50000);
		 }
		 hour = dec_to_bcd(hour);							// Convert them to bcd
		 RTCCI2C_setHour24(RTCC_RTCC, hour,&myDevice);		// Set Hour
		 usleep(50000);
		 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

		 // Finish setting hour ! --> Set Minute
		 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
		 OLEDrgb_PutString(&pmodOLEDrgb_inst,"Minute:");	// Hour
		 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
		 OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
		 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
		 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
		 while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
		 {
			 // interface with Pmod ENC
			 pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
			 if(old_RotaryCnt!=RotaryCnt)
			 {
			 	  if(RotaryCnt > old_RotaryCnt)
			 	  {
			 		 minute++;
			 		  if(minute>59) minute = 0;
			 	  }
			 	  else
			 	  {
			 		 minute--;
			 		  if(minute<0) minute = 59;

			 	  }		
					old_RotaryCnt=RotaryCnt;
				}

			 	 // interface with Button
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
				if(NX4IO_isPressed(BTNU))
				{
					minute++;
					if(minute > 60)
						minute = 0;
				}
				if(NX4IO_isPressed(BTND))
				{
					if(minute <= 0)
						minute =59;
					else
						minute--;
				}
				if(minute <= 9)
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
				PMDIO_putnum(&pmodOLEDrgb_inst,minute,10);
				usleep(50000);
		}
		 minute = dec_to_bcd(minute);
		 RTCCI2C_setMin(RTCC_RTCC, minute, &myDevice);
		 usleep(50000);
		 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

		 // Finish setting minute --> Set second
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Second:");	// Hour
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 2);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 2);
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
		while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
		{
		 		// interface with Pmod ENC
		 		pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
		 		if(old_RotaryCnt!=RotaryCnt)
		 		{

				 	  if(RotaryCnt > old_RotaryCnt)
				 	  {
				 		 sec++;
				 		  if(sec>59) sec = 0;
				 	  }
				 	  else
				 	  {
				 		 sec--;
				 		  if(sec<0) sec = 59;

				 	  }
		 				old_RotaryCnt=RotaryCnt;
		 		}
		 			 // interface with Button
		 			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 2);
		 			if(NX4IO_isPressed(BTNU))
		 			{
		 				sec++;
		 					if(sec > 60)
		 						sec = 0;
		 			}
		 				if(NX4IO_isPressed(BTND))
		 				{
		 					if(sec <= 0)
		 						sec =59;
		 					else
		 						sec--;
		 				}
		 				if(sec <= 9)
		 					OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
		 				PMDIO_putnum(&pmodOLEDrgb_inst,sec,10);
		 				usleep(50000);
		 }
				sec = dec_to_bcd(sec);
				RTCCI2C_setSec(RTCC_RTCC, sec, &myDevice);
				usleep(50000);
				 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

				 // Finish setting second --> Set Day 0x1 -> 0x7: Monday -> Sunday
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Day:");	// Hour
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
				OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
				while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
				{
						// interface with Pmod ENC
						pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
						if(old_RotaryCnt!=RotaryCnt)
						 {
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
							OLEDrgb_PutString(&pmodOLEDrgb_inst,"        ");
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);

						 	  if(RotaryCnt > old_RotaryCnt)
						 	  {
						 		 day++;
						 		  if(day>7) day = 1;
						 	  }
						 	  else
						 	  {
						 		 day--;
						 		  if(day<1) day = 7;

						 	  }
						 			old_RotaryCnt=RotaryCnt;
						 }
						 			 // interface with Button
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
						if(NX4IO_isPressed(BTNU))
						{
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
							OLEDrgb_PutString(&pmodOLEDrgb_inst,"        ");
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
						 	day++;
						 	if(day > 8)
						 			day = 1;
						}
						 	if(NX4IO_isPressed(BTND))
						 	{
						 		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
						 		OLEDrgb_PutString(&pmodOLEDrgb_inst,"        ");
						 		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
						 		if(day <= 1)
						 			day =7;
						 		else
						 			day--;
						 	}
						 	hex_to_day(day);
						 	usleep(50000);
						 }
							day = dec_to_bcd(day);
							RTCCI2C_setDay(RTCC_RTCC, day, &myDevice);
							usleep(50000);
							OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
				// Finish set day --> set Date
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Date:");	// Date
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
				 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
				while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
				{
						// interface with Pmod ENC
						pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
						if(old_RotaryCnt!=RotaryCnt)
						{
						 	  if(RotaryCnt > old_RotaryCnt)
						 	  {
						 		 date++;
						 		  if(date>31) date = 1;
						 	  }
						 	  else
						 	  {
						 		 date--;
						 		  if(date<1) date = 31;

						 	  }

								old_RotaryCnt=RotaryCnt;
						}
								// interface with Button
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
						if(NX4IO_isPressed(BTNU))
						{
								date++;
								if(date > 32)
								date = 1;
						}
								if(NX4IO_isPressed(BTND))
								{
									if(date <= 1)
										date =31;
									else
									 	date--;
								}
								if(date <= 9)
									 OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
								PMDIO_putnum(&pmodOLEDrgb_inst,date,10);
								usleep(50000);
				}
						date = dec_to_bcd(date);
						RTCCI2C_setDate(RTCC_RTCC, date, &myDevice);
						usleep(50000);
						 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

						// Finish set date --> Set month
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"Month:");	// Month
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 5);
						OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
						while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
						{
								// interface with Pmod ENC
								pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
								if(old_RotaryCnt!=RotaryCnt)
								{
								 	  if(RotaryCnt > old_RotaryCnt)
								 	  {
								 		 mon++;
								 		  if(mon>12) mon = 1;
								 	  }
								 	  else
								 	  {
								 		 mon--;
								 		  if(mon<1) mon = 12;

								 	  }

										old_RotaryCnt=RotaryCnt;
								}
										// interface with Button
								OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 5);
								if(NX4IO_isPressed(BTNU))
								{
										mon++;
										if(mon > 13)
											mon = 1;
								}
										if(NX4IO_isPressed(BTND))
										{
											if(mon <= 1)
												mon = 12;
											else
											 	mon--;
										}
										hex_to_month(mon);
										usleep(50000);
						}
						mon = dec_to_bcd(mon);
						RTCCI2C_setMonth(RTCC_RTCC, mon, &myDevice);
						usleep(50000);
						OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
				// Finish Month --> Set year

				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 6);
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"year:");	// year
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 6);
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
				OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 6);
				OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
				while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
				{
						// interface with Pmod ENC
						pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
						if(old_RotaryCnt!=RotaryCnt)
						{
							if(RotaryCnt > old_RotaryCnt)
							{
								year++;
								if(year>99) year = 0;
							}
							else
							{
								year--;
								if(year<0) year = 99;

							}
							old_RotaryCnt=RotaryCnt;
						}
							// interface with Button
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 6);
						if(NX4IO_isPressed(BTNU))
						{
							year++;
							if(year >= 100)
								year = 0;
						}
						if(NX4IO_isPressed(BTND))
						{
							if(year <= 0)
								year = 99;
							else
								year--;
						}
						OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 6);
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"20");
						if(year <= 9)
							OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
						PMDIO_putnum(&pmodOLEDrgb_inst,year,10);
						usleep(50000);
				}
						year = dec_to_bcd(year);
						RTCCI2C_setYear(year,&myDevice);
						usleep(50000);
						OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
				// Finish setting --> Start clock
				RTCCI2C_startClock(&myDevice);
				xil_printf("The time has been set \r\n");
				  //set vbat high
				RTCCI2C_enableVbat(&myDevice);
				RTCCI2C_clearPWRFAIL(&myDevice);

				// Loop until flag set --> go back 0
				while(flag_set)
				{
					if(NX4IO_isPressed(BTNL))
					{
						flag_set = 0;
						flag_background = 1;
					}
				}

}
/*
*	Function hex_to_day: LUT table to convert day --> day in calender for displaying
*/
void hex_to_day(day_display)
{
	switch(day_display)
	{
		case 1:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Monday");
				break;
		case 2:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Tuesday");
				break;
		case 3:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Wedneday");
				break;
		case 4:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Thursday");
				break;
		case 5:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Friday");
				break;
		case 6:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Saturday");
				break;
		case 7:
				OLEDrgb_PutString(&pmodOLEDrgb_inst,"Sunday");
				break;
	}
}

/*
*	Function hex_to_month: LUT table to convert month --> month in calender for displaying
*/
void hex_to_month(int month_display)
{
	switch(month_display)
	{
		case 1:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Jan");
					break;
		case 2:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Feb");
					break;
		case 3:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Mar");
					break;
		case 4:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Apr");
					break;
		case 5:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"May");
					break;
		case 6:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Jun");
					break;
		case 7:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Jul");
					break;
		case 8:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Aug");
					break;
		case 9:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Sep");
					break;
		case 10:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Oct");
					break;
		case 11:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Nov");
					break;
		case 12:
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Dec");
					break;
		}
}
/*
 * Function set alarm
 */
void set_alarm(u8 src, int flag_alarm)
{
	int hour_alarm  = 0, minute_alarm = 0, sec_alarm =0, day_alarm = 0, date_alarm= 0, mon_alarm = 0;
	diff  = 0;
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_DrawBitmap(&pmodOLEDrgb_inst,0,0,95,63,(uint8_t*)set_alarm_pic);
	usleep(700000);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 0);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));  // blue font

	 // Set hour
			 OLEDrgb_PutString(&pmodOLEDrgb_inst,"Hour:");	// Hour
			 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
			 OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
			 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
			 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
			 while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
			 {
				 // interface with Pmod ENC
				 pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
				 if(old_RotaryCnt!=RotaryCnt)
				 {
					 	  if(RotaryCnt > old_RotaryCnt)
					 	  {
					 		  hour_alarm++;
					 		  if(hour_alarm>24) hour_alarm = 0;
					 	  }
					 	  else
					 	  {
					 		 hour_alarm--;
					 		  if(hour_alarm<0) hour_alarm = 24;

					 	  }
					 	
						  old_RotaryCnt=RotaryCnt;
				 }

				 // interface with Button
				 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 0);
				 if(NX4IO_isPressed(BTNU))
				 {
					 hour_alarm++;
					 if(hour_alarm > 24)
						 hour_alarm = 0;
				 }
				 if(NX4IO_isPressed(BTND))
				 {
					 if(hour_alarm <= 0)
						 hour_alarm =23;
					 else
						 hour_alarm--;
				 }
				 if(hour_alarm <= 9)
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
				 PMDIO_putnum(&pmodOLEDrgb_inst, hour_alarm,10);
				 usleep(50000);
			 }
			 hour_alarm= dec_to_bcd(hour_alarm);
			 //RTCCI2C_setHour24(RTCC_RTCC, hour,&myDevice);
			 RTCCI2C_setHour24(src, hour_alarm, &myDevice);
			 usleep(50000);
			 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

			 // Finish setting hour ! --> Set Minute
			 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
			 OLEDrgb_PutString(&pmodOLEDrgb_inst,"Minute:");	// Hour
			 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
			 OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
			 OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
			 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
			 while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
			 {
				 // interface with Pmod ENC
				 pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
				 if(old_RotaryCnt!=RotaryCnt)
				 {
				 	  if(RotaryCnt > old_RotaryCnt)
				 	  {
				 		 minute_alarm++;
				 		  if(minute_alarm>59) minute_alarm = 0;
				 	  }
				 	  else
				 	  {
				 		 minute_alarm--;
				 		  if(minute_alarm<0) minute_alarm = 59;

				 	  }


						old_RotaryCnt=RotaryCnt;
					}

				 	 // interface with Button
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 1);
					if(NX4IO_isPressed(BTNU))
					{
						minute_alarm++;
						if(minute_alarm > 60)
							minute_alarm = 0;
					}
					if(NX4IO_isPressed(BTND))
					{
						if(minute_alarm <= 0)
							minute_alarm =59;
						else
							minute_alarm--;
					}
					if(minute_alarm <= 9)
						OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
					PMDIO_putnum(&pmodOLEDrgb_inst,minute_alarm,10);
					usleep(50000);
			}
			 minute_alarm = dec_to_bcd(minute_alarm);
			 RTCCI2C_setMin(src, minute_alarm, &myDevice);
			 usleep(50000);
			 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

			 // Finish setting minute --> Set second
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"Second:");	// Hour
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 2);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"  ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 2);
			 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
			while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
			{
			 		// interface with Pmod ENC
			 		pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
			 		if(old_RotaryCnt!=RotaryCnt)
			 		{

					 	  if(RotaryCnt > old_RotaryCnt)
					 	  {
					 		 sec_alarm++;
					 		  if(sec_alarm>59) sec_alarm = 0;
					 	  }
					 	  else
					 	  {
					 		 sec_alarm--;
					 		  if(sec_alarm<0) sec_alarm = 59;

					 	  }


			 			
			 				old_RotaryCnt=RotaryCnt;
			 		}
			 			 // interface with Button
			 			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 2);
			 			if(NX4IO_isPressed(BTNU))
			 			{
			 				sec_alarm++;
			 					if(sec_alarm> 60)
			 						sec_alarm= 0;
			 			}
			 				if(NX4IO_isPressed(BTND))
			 				{
			 					if(sec_alarm<= 0)
			 						sec_alarm =59;
			 					else
			 						sec_alarm--;
			 				}
			 				if(sec_alarm <= 9)
			 					OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
			 				PMDIO_putnum(&pmodOLEDrgb_inst,sec_alarm,10);
			 				usleep(50000);
			 }
					sec_alarm = dec_to_bcd(sec_alarm);
					RTCCI2C_setSec(src, sec_alarm, &myDevice);
					usleep(50000);
					 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

					 // Finish setting second --> Set Day 0x1 -> 0x7: Monday -> Sunday
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Day:");	// Hour
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
					OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
					while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
					{
							// interface with Pmod ENC
							pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
							if(old_RotaryCnt!=RotaryCnt)
							 {
								OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
								OLEDrgb_PutString(&pmodOLEDrgb_inst,"        ");
								OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);

							 	  if(RotaryCnt > old_RotaryCnt)
							 	  {
							 		 day_alarm++;
							 		  if(day_alarm>7) day_alarm = 1;
							 	  }
							 	  else
							 	  {
							 		 day_alarm--;
							 		  if(day_alarm<1) day_alarm = 7;

							 	  }
							 	old_RotaryCnt=RotaryCnt;
							 }
							 			 // interface with Button
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
							if(NX4IO_isPressed(BTNU))
							{
								OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
								OLEDrgb_PutString(&pmodOLEDrgb_inst,"        ");
								OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
								day_alarm++;
							 	if(day_alarm > 8)
							 		day_alarm = 1;
							}
							 	if(NX4IO_isPressed(BTND))
							 	{
							 		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
							 		OLEDrgb_PutString(&pmodOLEDrgb_inst,"        ");
							 		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
							 		if(day_alarm <= 1)
							 			day_alarm =7;
							 		else
							 			day_alarm--;
							 	}
							 	hex_to_day(day_alarm);
							 	usleep(50000);
							 }
								day_alarm= dec_to_bcd(day_alarm);
								RTCCI2C_setDay(src, day_alarm, &myDevice);
								usleep(50000);
								OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
					// Finish set day --> set Date
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"Date:");	// Date
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
					OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
					OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
					 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
					while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
					{
							// interface with Pmod ENC
							pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
							if(old_RotaryCnt!=RotaryCnt)
							{
							 	  if(RotaryCnt > old_RotaryCnt)
							 	  {
							 		 date_alarm++;
							 		  if(date_alarm>31) date_alarm = 1;
							 	  }
							 	  else
							 	  {
							 		 date_alarm--;
							 		  if(date_alarm<1) date_alarm = 31;

							 	  }
									old_RotaryCnt=RotaryCnt;
							}
									// interface with Button
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 4);
							if(NX4IO_isPressed(BTNU))
							{
								date_alarm++;
									if(date_alarm> 32)
										date_alarm = 1;
							}
									if(NX4IO_isPressed(BTND))
									{
										if(date_alarm <= 1)
											date_alarm =31;
										else
											date_alarm--;
									}
									if(date_alarm <= 9)
										 OLEDrgb_PutString(&pmodOLEDrgb_inst,"0");
									PMDIO_putnum(&pmodOLEDrgb_inst,date_alarm,10);
									usleep(50000);
					}
							date_alarm= dec_to_bcd(date_alarm);
							RTCCI2C_setDate(src, date_alarm, &myDevice);
							usleep(50000);
							 OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));

							// Finish set date --> Set month
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
							OLEDrgb_PutString(&pmodOLEDrgb_inst,"Month:");	// Month
							OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
							OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 5);
							OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
							while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
							{
									// interface with Pmod ENC
									pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
									if(old_RotaryCnt!=RotaryCnt)
									{
									 	  if(RotaryCnt > old_RotaryCnt)
									 	  {
									 		 mon_alarm++;
									 		  if(mon_alarm>12) mon_alarm = 1;
									 	  }
									 	  else
									 	  {
									 		 mon_alarm--;
									 		  if(mon_alarm<1) mon_alarm= 12;

									 	  }
											old_RotaryCnt=RotaryCnt;
									}
											// interface with Button
									OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 5);
									if(NX4IO_isPressed(BTNU))
									{
										mon_alarm++;
											if(mon_alarm> 13)
												mon_alarm= 1;
									}
											if(NX4IO_isPressed(BTND))
											{
												if(mon_alarm <= 1)
													mon_alarm= 12;
												else
													mon_alarm--;
											}
											hex_to_month(mon_alarm);
											usleep(50000);
							}
									mon_alarm = dec_to_bcd(mon_alarm);
									RTCCI2C_setMonth(RTCC_ALM0,mon_alarm, &myDevice);
									OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
									OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 6);
									OLEDrgb_PutString(&pmodOLEDrgb_inst,"Difficult:");
									OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
									OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 10, 6);
									OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,127,255));
									while((!NX4IO_isPressed(BTNC))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))  // Loop until confirm setting hour
									{
										// interface with Pmod ENC
										pmodENC_read_count(&pmodENC_inst,&RotaryCnt);
										if(old_RotaryCnt!=RotaryCnt)
										{
											if(RotaryCnt > old_RotaryCnt)
											{
												diff++;
												if(diff>4) diff = 1;
											}
											else
											{
												diff--;
											if(diff<1) diff= 3;

										}

											old_RotaryCnt=RotaryCnt;
									}
																				// interface with Button
											OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 10, 6);
										if(NX4IO_isPressed(BTNU))
										{
											diff++;
											if(diff> 4)
												diff= 1;
										}
										if(NX4IO_isPressed(BTND))
										{
											if(diff <= 1)
												diff= 3;
											else
												diff--;
										}
										PMDIO_putnum(&pmodOLEDrgb_inst,diff,10);
										usleep(50000);
					}

									usleep(50000);

		  //enables alarm 0
		  //set configuration bits to RTCC_ALM_POL | RTCC_ALMC2 | RTCC_ALMC1 | RTCC_ALMC0
		  //this will drive the MPF pin high when the alarm triggered
		  //it also sets the alarm to be triggered when the alarm matches
	      //Seconds, Minutes, Hour, Day, Date, Month of the RTCC
		  RTCCI2C_enableAlarm(src, RTCC_ALM_POL | RTCC_ALMC2 | RTCC_ALMC1 | RTCC_ALMC0, &myDevice);

			  // Loop until flag set --> go back 0
			  				while(flag_alarm)
			  				{
			  					if(NX4IO_isPressed(BTNL))
			  					{
			  						flag_alarm = 0;
			  						flag_alarm1 = 0;
			  						flag_background = 1;
			  						flag_alarm2 = 0;
			  					}
			  				}
}

/******************************************************8
 * Function: Logic game here!
 */

void game_on()
{
	navigation_algorithm();
	grade = 0;
	int pass[5];
	pass[0] = 0;
	pass[1] = 0;
	pass[2] = 0;
	pass[3] = 0;
	pass[4] = 0;
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_DrawBitmap(&pmodOLEDrgb_inst,0,0,95,63,(uint8_t*)color_game);
	while ((!NX4IO_isPressed(BTNL))&&(!NX4IO_isPressed(BTNC))&&(!NX4IO_isPressed(BTNR))
			&&(!NX4IO_isPressed(BTND)) &&(!NX4IO_isPressed(BTNU))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))
	{

	}




	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x1); // Speak song
	grade = grade + game_screen_1(pass);
	usleep(500000);
	grade = grade + game_screen_2(pass);
	usleep(500000);
	grade = grade + game_screen_3(pass);
	usleep(500000);
	grade = grade + game_screen_4(pass);
	usleep(500000);
	grade = grade + game_screen_5(pass);
	usleep(500000);
	result_game(grade,pass);
	// PASS --> Go back showing time
	if(grade>= 3)
	{
		XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x0);
		flag_game = 0;
		flag_show = 0;
		flag_background = 1;
	}
	else // fail
	{
		if(diff==0) // Finish times difficulty
		{
			flag_game = 0;
			flag_show =1;
		}
		else
		{
			diff--;
			flag_game = 1;
			while ((!NX4IO_isPressed(BTNL))&&(!NX4IO_isPressed(BTNC))&&(!NX4IO_isPressed(BTNR))
						&&(!NX4IO_isPressed(BTND)) &&(!NX4IO_isPressed(BTNU))&&(!pmodENC_is_button_pressed(&pmodENC_inst)))
			{
			}
		}
	}
	usleep(500000);

}
/*
 *  Game screen 1 --> Game screen 5: display logic question 1 to question 5 and return result back
 */
int game_screen_1(int pass[])
{

    	OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,0,0,15,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255,0,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,35,0,50,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(0,255,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,0,85,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(0, 0,255));
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Red?");
		while (1)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 7);
			OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildHSV(255,255,255));
			OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 0,0));

			if(NX4IO_isPressed(BTNL))
			{

				pass[0]  = 1;
				return 1;
			}
			if(NX4IO_isPressed(BTNC))
			{
				pass[0] = 0;
				return 0;
			}
			if(NX4IO_isPressed(BTNR))
			{
				pass[0] = 0;
				return 0;
			}

		}
		return 0;

}

int game_screen_2(int pass[])
{

		OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,0,0,15,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255,255,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,35,0,50,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(0,255,255));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,0,85,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255, 0,0));
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Cyan?");
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 0,0));
		while (1)
		{
			if(NX4IO_isPressed(BTNL))
			{
				pass[1] = 0;
				//usleep(500000);
				return 0;
			}
			if(NX4IO_isPressed(BTNC))
			{
				usleep(1000);
				pass[1] = 1;
				//usleep(500000);
				return 1;
			}
			if(NX4IO_isPressed(BTNR))
			{
				pass[1] = 0;
				//usleep(500000);
				return 0;
			}
		}
		return 0;
}

int game_screen_3(int pass[])
{

		OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,0,0,15,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255,0,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,35,0,50,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(0,0,255));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,0,85,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(0, 255,0));
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Green?");
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 0,0));
		while (1)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 7);
			if(NX4IO_isPressed(BTNL))
			{
				pass[2] = 0;
				//usleep(500000);
				return 0;
			}
			if(NX4IO_isPressed(BTNC))
			{
				pass[2] = 0;
				//usleep(500000);
				return 0;
			}
			if(NX4IO_isPressed(BTNR))
			{
				usleep(1000);
				pass[2] = 1;
				//usleep(500000);
				return 1;
			}

		}
		return 0;
}

int game_screen_4(int pass[])
{

		OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,0,0,15,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255,0,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,35,0,50,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(0,0,255));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,0,85,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255, 255,255));
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Blue?");
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 0,0));
		while (1)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 7);
			if(NX4IO_isPressed(BTNL))
			{
				pass[3] = 0;
				//usleep(500000);
				return 0;
			}
			if(NX4IO_isPressed(BTNC))
			{
				usleep(1000);
				pass[3] = 1;
				//usleep(500000);
				return 1;
			}
			if(NX4IO_isPressed(BTNR))
			{
				pass[3] = 0;
				//usleep(500000);
				return 0;
			}

		}
		return 0;
}

int game_screen_5(int pass[])
{

		OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,0,0,15,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255,255,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,35,0,50,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(255,0,0));
		usleep(1000);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,0,85,45,OLEDrgb_BuildRGB(36,50,120),true,OLEDrgb_BuildRGB(162, 61,94));
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Yellow?");
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 0,0));
		while (1)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 6, 7);
			if(NX4IO_isPressed(BTNL))
			{
				usleep(1000);
				pass[4] = 1;
				//usleep(500000);
				return 1;
			}
			if(NX4IO_isPressed(BTNC))
			{
				pass[4] = 0;
				//usleep(500000);
				return 0;
			}
			if(NX4IO_isPressed(BTNR))
			{
				pass[4] = 0;
				//usleep(500000);
				return 0;
			}

		}
		return 0;
}

void result_game(int grade, int pass[])
{
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 0,0));
	for(int i =0; i <5; i++)
	{
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, i);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Quiz ");
		PMDIO_putnum(&pmodOLEDrgb_inst,i,10);
		OLEDrgb_PutChar(&pmodOLEDrgb_inst,':');
		if(pass[i] == 1)
			OLEDrgb_PutString(&pmodOLEDrgb_inst," PASS");
		else if (pass[i] == 0)
			OLEDrgb_PutString(&pmodOLEDrgb_inst," NO");
		else
			OLEDrgb_PutString(&pmodOLEDrgb_inst," ERROR");
	}
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 6);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Overall ");
	PMDIO_putnum(&pmodOLEDrgb_inst,grade,10);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"/5");


	if(grade >=3)
	{
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Successful!");
	}
	else
	{
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 7);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Fail!");
	}
}

void speak_current_time()
{
	int result, temp1, temp2;
	temp1 = hour & 0x0F;
	temp1 = temp1 + 1;
	temp1 = temp1 <<12;
	temp2 = hour >>4;
	temp2 = temp2 + 1;
	result = temp2 <<16;  // Take first digit hour
	result = result | temp1; // Take second digit hour

	temp1 = minute &0x0F;
	temp1 = temp1 +1;
	temp2 = minute >> 4;
	temp2 = temp2 + 1;
	temp2 = temp2 <<8;
	temp1 = temp1 << 4;
	result = result|temp2; // Take first digit minute
	result = result|temp1; // Take second digit minute

	if (hour >= 0x12)
		result = result | 0x2;
	else
		result = result | 0x1;

	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x4);
	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, result);

}

void speak_hour()
{
	int result, temp1, temp2,temp;
	if(hour > 0x12)
		temp = hour - 0x12;
	else
		temp = hour;

	temp1 = temp & 0x0F;
	temp1 = temp1 + 1;
	temp2 = temp >> 4;
	temp2 = temp2 + 1;
	result = temp2<<8; // take first digit hour
	temp1 = temp1 << 4;
	result = result | temp1;
	if (hour >= 0x12)
		result = result | 0x2;
	else
		result = result | 0x1;

	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_1_CHANNEL, 0x4);
	XGpio_DiscreteWrite(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, result);
}


/********************NAVIGATIONAL ALGORITHM*****************************/

void move_forward()
{
	xil_printf("forward\n");
	u8 data,direction;
	direction = 0;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,direction);
	MB_Sleep_1(250);
	data = 12;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,data);
	motor_running_flag = 1;
}
void move_back()
{
	xil_printf("reverse\n");
	u8 data,direction;
	direction = 3;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,direction);
	MB_Sleep_1(250);
	data = 15;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,data);
	motor_running_flag = 1;

}
void move_right()
{
	xil_printf("right\n");
	u8 data,direction;

	if(compass >= 170)
	{
		stop_motor();
		MB_Sleep_1(750);
		move_back();
		MB_Sleep_1(2000);
		slight_right();

	}
	else
	{
	move_back();
	MB_Sleep_1(750);
	direction = 0;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,direction);
	MB_Sleep_1(20);
	data = 8;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,data);
	MB_Sleep_1(600);
	stop_motor();
	MB_Sleep_1(1000);
	move_forward();
	motor_running_flag = 1;
	}
	compass = compass + 30;


}
void move_left()
{
	xil_printf("left\n");
	u8 data,direction;
	direction = 0;
	if(compass <= -170)
	{
		stop_motor();
		MB_Sleep_1(600);
		move_back();
		MB_Sleep_1(2000);
		slight_left();

	}
	else
	{
		MB_Sleep_1(750);
    XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,direction);
    MB_Sleep_1(20);
	data = 4;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,data);
	MB_Sleep_1(1000);
	stop_motor();
	MB_Sleep_1(1000);
	move_forward();
	motor_running_flag = 1;
	}
	compass = compass - 30;

}

void stop_motor()
{
	xil_printf("stop\n");
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,0);
	motor_running_flag = 0;

}

void slight_right()
{
	compass = 30;
	xil_printf("sligh_right\n");
	u8 data,direction;
	direction = 0;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,direction);
	MB_Sleep_1(20);
	data = 8;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,data);
	MB_Sleep_1(1000);
	stop_motor();
	MB_Sleep_1(100);
	move_forward();
	motor_running_flag = 1;
}

void slight_left()
{
	u8 direction,data;
	compass = -30;
	xil_printf("slight_left\n");
	direction = 0;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,direction);
	MB_Sleep_1(20);
	data = 4;
	XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,data);
	MB_Sleep_1(1000);
	stop_motor();
	MB_Sleep_1(100);
	move_forward();
	motor_running_flag = 1;
}

void set_servo_position(u8 position)
{

	{
		XGpio_DiscreteWrite(&GPIOInst3, GPIO_2_OUTPUT_0_CHANNEL,position);
	}
}


u8 look_for_path() //returns the direction where there is no obstacle.
{
	u8 observation;
	//Scans all the three sides and returns below value based on the observation.
	//000 . All sides Clear.
	//001 . Left and Center are clear.
	//010 . Left and Right are clear.
	//011 . Left side is clear.
	//100 . Center and Right are clear.
	//101 . Center is clear.
	//110 .Right side is clear.
	//111 .All sides blocked.

	u8 right;
	u8 left;
	u8 center;



	set_servo_position(64);//Turn sensor Left
	MB_Sleep_1(500);
	left = sensor;
	//xil_printf("left:%d\n",sensor);


	set_servo_position(0);//sensor looks straight.
	MB_Sleep_1(500);
	center = sensor;
	//xil_printf("center:%d\n",sensor);

	set_servo_position(128);//Turn sensor right.
	MB_Sleep_1(500);
	right = sensor;
	//xil_printf("right:%d\n",sensor);


	set_servo_position(0);//sensor looks straight.
	MB_Sleep_1(500);
	center = sensor;

	center = center <<1;
	left = left << 2;


	observation = (left | center | right);
	return observation;

}


void navigation_algorithm()
{

	u16 switches;
	while(1)
{
//xil_printf("sensor = %d\n",sensor);
		if(sensor)
		{
			stop_motor();
			xil_printf("obstacle\n");
		}
		//xil_printf("while loop sensor = %d\n",sensor);

		if(!motor_running_flag)//Motor stopped.
		{
			path = look_for_path();
			//000 . All sides are clear.
			//001 . Left and Center are clear.
			//010 . Left and Right are clear.
			//011 . Left side is clear.
			//100 . Center and Right are clear.
			//101 . Center is clear.
			//110 .Right side is clear.
			//111 .All sides blocked.

			if(path == 2)
			{
				if(compass > 30)
					path = 3;
				if(compass < -30)
					path = 2;

			}


			switch(path)
			{
			case 0:
				xil_printf("All sides clear %d\n",path);
				xil_printf("DECISION: Moving forward\n");
				move_forward();
				break;
			case 1:
				xil_printf("Left and Center are clear%d\n",path);
				xil_printf("DECISION: Moving forward\n");
				move_forward();

				break;
			case 2:
				xil_printf("Left and Right are clear.%d\n",path);
				xil_printf("DECISION: Moving right\n");
					stop_motor();
					MB_Sleep_1(100);
					move_right();
				break;
			case 3:
				xil_printf("Left side is clear..%d\n",path);
				xil_printf("DECISION: Moving left\n");
					stop_motor();
					MB_Sleep_1(100);
					move_left();

				break;
			case 4:
				xil_printf("Center and Right are clear.%d\n",path);
				xil_printf("DECISION: Moving forward\n");
				move_forward();
				break;
			case 5:
				xil_printf("Center is clear.%d\n",path);
				xil_printf("DECISION: Moving forward\n");
				move_forward();
				break;
			case 6:
				xil_printf("Right side is clear..%d\n",path);
				xil_printf("DECISION: Moving right\n");

					stop_motor();
					MB_Sleep_1(100);
					move_right();
				break;
			case 7:
				stop_motor();
				MB_Sleep_1(500);
				move_back();
				MB_Sleep_1(500);
				stop_motor();
				MB_Sleep_1(500);
				move_right();
				break;
			}
		}
		switches = NX4IO_getSwitches();
		if(switches >> 15 == 1) {
			stop_motor();
			break;}
}
	return;
}


