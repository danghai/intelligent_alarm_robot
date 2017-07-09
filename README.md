# Intelligent Alarm Robot

### Introduction: 
This project is all about making our lives active in the early morning. Now a-days alarm clock is embedded into our life style. 
People use it extensively. But, there is a problem! Many of us will try to squeeze out few extra minutes of sleep even if the alarm is ringing. 
We set it to snooze simply to sleep again even if we had to wake up soon. We want to make snoozing alarm clocks difficult. This project implements
 a new kind of alarm clock with moving capabilities. If you want to turn off the alarm, first you should try to catch that 
fast moving alarm clock and play a small mind troubling game to turn off the alarm. This makes us to wake up and start the day at an active pace.

### Implementation and Architecture:
This project was implemented on Nexys 4 DDR board. It makes use of the different resources available on the board.
It uses Microblaze as the main CPU and will be running Standalone OS for its operation. We have planned well ahead to
allocate the resources required for all the differnet modules used in the system. To get a quick idea, I am listing down 
all the inputs and outputs from our system.

1.   Inputs: 

- Pushbuttons 
- Slide Switches 
- Pmod ENC
- Ultrasonic range finder
- Pmod RTCC

2.  Outputs:

- Motors
- OLED RGB 
- Speaker

3. This alarm clock has these following features:
-  Show the current time, date and week of the day
-  Set and use two alarms. 
-  Announce the current time
-  Set to announce the time every hour.

Link Video Demo: [Video Demo](https://www.youtube.com/watch?v=lEhJg26N7zI&t=169s)
