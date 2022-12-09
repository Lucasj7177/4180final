#include "mbed.h"
#include "rtos.h"
#include "ultrasonic.h"
#include "uLCD_4DGL.h"
#include "PinDetect.h"
#include "motordriver.h"
#include "RPG.h"
#include <stdio.h>

/// function definitions ///
void dist(int distance);


/// Pin Assignments ///
// left motor
Motor  left(p21, p22, p23, 1); // pwm, fwd, rev, has brake feature
// right motor
Motor right(p26, p25, p24, 1);
 // uLCD
uLCD_4DGL uLCD(p9,p10,p11); // serial tx, serial rx, reset pin;
// rpg
// InterruptIn RPG_A(p14);//encoder A and B pins/bits use interrupts
// InterruptIn RPG_B(p15);
// PinDetect RPG_PB(p16); //encode pushbutton switch "SW" on PCB
RPG rpg(p14, p15, p16);
// speaker with amp
AnalogOut speaker(p18);
Ticker Sample_Period;
// sonar
ultrasonic mu(p28, p27, .1, 1, &dist);    //Set the trigger pin to p28 and the echo pin to p27
                                        //have updates every .1 seconds and a timeout after 1
                                        //second, and call dist when the distance changes
// LEDs
DigitalOut ledPB(LED1);


/// Variables ///
// rpg variable and lookup table
volatile int old_enc = 0;
volatile int new_enc = 0;
volatile int enc_count = 0;
const int lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int count = 0;
int dirt = 0;
// time variables
volatile int curr_hour = 12;
volatile int curr_min = 0;
volatile int alarm_hour = 0;
volatile int alarm_min = 0;
bool set_curr_hour = true;
bool set_curr_min = false;
bool set_alarm_hour = false;
bool set_alarm_min = false;
bool alarm = false;
// uLCD variables
Mutex uLCD_lock;
// speaker variables
float Analog_out_data[128];
volatile int i=0;


/// helper function for sonar ///
 void dist(int distance)
{
    //put code here to execute when the distance has changed
    uLCD_lock.lock();
    uLCD.locate(0, 5);
    uLCD.printf("Distance %d mm\r\n", distance);
    uLCD_lock.unlock();
}

/// RPG rotary callback ///
//Encoder bit change interrupt service routine
//called whenever one of the two A,B encoder bits change state
void Enc_change_ISR(void)
{
    // new_enc = RPG_A<<1 | RPG_B;//current encoder bits
    // //check truth table for -1,0 or +1 added to count
    // enc_count = lookup_table[old_enc<<2 | new_enc];
    // old_enc = new_enc;
    // logic to change times might actually need to be here
    // check if a set time flag is set and then update 
    if (set_curr_hour)
    {
        if (enc_count == 1) curr_hour++;
        if (enc_count == -1) curr_hour--;
        if (curr_hour >= 24) curr_hour = 0;
        if (curr_hour <= 0) curr_hour = 0;
    }
    if (set_curr_min)
    {
        if (enc_count == 1) curr_min++;
        if (enc_count == -1) curr_min--;
        if (curr_min >= 60) curr_min = 0;
        if (curr_min <= 0) curr_min = 0;
    }
    if (set_alarm_hour)
    {
        if (enc_count == 1) alarm_hour++;
        if (enc_count == -1) alarm_hour--;
        if (alarm_hour >= 24) alarm_hour = 0;
        if (alarm_hour <= 0) alarm_hour = 0;
    }
    if (set_alarm_min)
    {
        if (enc_count == 1) alarm_min++;
        if (enc_count == -1) alarm_min--;
        if (alarm_min >= 60) alarm_min = 0;
        if (alarm_min <= 0) alarm_min = 0;
    }
    Thread::wait(500);
}

/// RPG pushbutton callback ///
void PB_callback(void)
{
    ledPB= !ledPB;
    set_curr_hour= !set_curr_hour;
    // if (alarm)
    // {
    //     alarm = false;
    //     speaker = 0;
    // }
    // else
    // {
    //     if (!set_curr_hour && !set_curr_min && !set_alarm_hour && !set_alarm_min)
    //     {
    //         set_curr_hour = true;
    //     }
    //     if (set_curr_hour)
    //     {
    //         set_curr_hour = false;
    //         set_curr_min = true;
    //     }
    //     if (set_curr_min)
    //     {
    //         set_curr_min = false;
    //         set_alarm_hour = true;
    //     }
    //     if (set_alarm_hour)
    //     {
    //         set_alarm_hour = false;
    //         set_alarm_min = true;
    //     }
    //     if (set_alarm_min)
    //     {
    //         set_alarm_min = false;
    //     }
    // }
}

/// MOVEMENT THREAD ///
void movement()
{
    while(1)
    {
        if (alarm)
        {
            
            mu.checkDistance();     //call checkDistance() as much as possible, as this is where
                                    //the class checks if dist needs to be called.
            left.speed(0.5);
            right.speed(0.5);
        }
        else
        {
            mu.checkDistance();
            left.speed(0);
            right.speed(0);
        }
        Thread::wait(100);
    }
}

/// UPDATE TIME THREAD ///
void updateTime()
{
    while(1)
    {
        curr_min++;
        if (curr_min == 60)
        {
            curr_hour++;
            curr_min = 0;
        }
        if (curr_hour == 24)
        {
            curr_hour = 0;
        }
        // wait for 60 seconds
        Thread::wait(1000);
    }
}

/// SPEAKER THREAD /// 
void speaker_interrupt()
{
    while(1)
    {
        if (alarm)
        {
            // send next analog sample out to D to A
            speaker = Analog_out_data[i];
            // increment pointer and wrap around back to 0 at 128
            i = (i+1) & 0x07F;
        }
        else
        {
            // this should also be set in the PB callback but this is just in case I suppose.
            speaker = 0;
        }
        Thread::wait(1000);
    }
}

/// MAIN FUNCTION ///
int main() {
    //uLCD.printf("curr hour %2d", curr_hour);
    //debounce RPG center pushbutton
    // RPG_PB.mode(PullDown);
    // RPG_PB.attach_deasserted(&PB_callback);
    // RPG_PB.setSampleFrequency();
    // // generate an interrupt on any change in either encoder bit (A or B)
    // RPG_A.mode(PullUp);
    // RPG_B.mode(PullUp);
    // RPG_A.rise(&Enc_change_ISR);
    // RPG_A.fall(&Enc_change_ISR);
    // RPG_B.rise(&Enc_change_ISR);
    // RPG_B.fall(&Enc_change_ISR);
    // set up speaker
    // precompute 128 sample points on one sine wave cycle 
    // used for continuous sine wave output later
    // for(int k=0; k<128; k++) {
    //     Analog_out_data[k]=((1.0 + sin((float(k)/128.0*6.28318530717959)))/2.0);
    //     // scale the sine wave from 0.0 to 1.0 - as needed for AnalogOut arg 
    // }
    // turn on timer interrupts to start sine wave output
    // sample rate is 500Hz with 128 samples per cycle on sine wave
    //Sample_Period.attach(&speaker_interrupt, 1.0/(1000.0*128));
    // init sonar
    mu.startUpdates();
    // clear uLCD
    uLCD.cls();

    // Start threads
    //Thread speaker(speaker_interrupt);
    Thread time(updateTime);
    Thread move(movement);

    /// MAIN THREAD ///
    while(1) {
        // print times to lcd scree
        dirt = rpg.dir();
        count += dirt;
        if (rpg.dir())
        {
            count = 0;
        }

        uLCD_lock.lock();
        // uLCD.locate(0, 0);
        // if (set_curr_hour) uLCD.printf("MODE: set current hour");
        // else if (set_curr_min) uLCD.printf("MODE: set current minute");
        // else if (set_alarm_hour) uLCD.printf("MODE: set alarm hour");
        // else if (set_alarm_min) uLCD.printf("MODE: set alarm minute");
        // else uLCD.printf("MODE: normal");
        // uLCD.locate(1, 1);
        // uLCD.printf("curr hour %2d", curr_hour);
        // uLCD.locate(1, 2);
        // uLCD.printf("curr min %2d", curr_min);
        // uLCD.locate(1, 3);
        // uLCD.printf("alarm hour %2d", alarm_hour);
        // uLCD.locate(1, 4);
        // uLCD.printf("alarm min %2d", alarm_min);
        // uLCD.locate(1, 6);
        // uLCD.printf("alarm %d", alarm);
        // uLCD.locate(1, 7);
        // uLCD.printf("curr hour %d", set_curr_hour);
        // uLCD.locate(1, 8);
        // uLCD.printf("curr min %d", set_curr_min);
        // uLCD.locate(1, 9);
        // uLCD.printf("alarm hour %d", set_alarm_hour);
        // uLCD.locate(1, 10);
        // uLCD.printf("alarm min %d", set_alarm_min);
        uLCD.locate(1, 11);
        uLCD.printf("dirt %i", dirt);
        uLCD.locate(1, 12);
        uLCD.printf("count %i", count);
        uLCD_lock.unlock();

        // check if times are equal
        if (curr_hour == alarm_hour && curr_min == alarm_min) alarm = true;
        else alarm = false;

        Thread::wait(50);
    }
}
