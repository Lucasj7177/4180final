#include "mbed.h"
#include "rtos.h"
#include "ultrasonic.h"
#include "uLCD_4DGL.h"
#include "PinDetect.h"
#include "motordriver.h"

#include <stdio.h>

void dist(int distance);

/// Pin Assignments ///
// left motor
Motor  left(p21, p22, p23, 1); // pwm, fwd, rev, has brake feature
// right motor
Motor right(p26, p25, p24, 1);
 // uLCD
uLCD_4DGL uLCD(p9,p10,p11); // serial tx, serial rx, reset pin;
// rpg
InterruptIn RPG_A(p14);//encoder A and B pins/bits use interrupts
InterruptIn RPG_B(p15);
PinDetect RPG_PB(p16); //encode pushbutton switch "SW" on PCB
// pushbuttons
PinDetect up(p12);
PinDetect down(p13);
//RPG rpg(p14, p15, p16);
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
// time variables
volatile int curr_hour = 12;
volatile int curr_min = 0;
volatile int alarm_hour = 0;
volatile int alarm_min = 0;
volatile bool set_curr_hour = false;
volatile bool set_curr_min = false;
volatile bool set_alarm_hour = false;
volatile bool set_alarm_min = false;
volatile bool alarm = false;
volatile bool turn = false;
// uLCD variables
Mutex uLCD_lock;
// speaker variables
float Analog_out_data[128];
volatile int i=0;


void dist(int distance)
{
    if (distance < 300)
    {
        turn = true;
    }
    else
    {
        turn = false;
    }
}

/// pushbutton callback ///
// increment time by one to be more precise
void precisionUp()
{
    if (set_curr_hour)
    {
        curr_hour++;
        if (curr_hour >= 24) curr_hour = 0;
        if (curr_hour <= 0) curr_hour = 0;
    }
    else if (set_curr_min)
    {
        curr_min++;
        if (curr_min >= 60) curr_min = 0;
        if (curr_min <= 0) curr_min = 0;
    }
    else if (set_alarm_hour)
    {
        alarm_hour++;
        if (alarm_hour >= 24) alarm_hour = 0;
        if (alarm_hour <= 0) alarm_hour = 0;
    }
    else if (set_alarm_min)
    {
        alarm_min++;
        if (alarm_min >= 60) alarm_min = 0;
        if (alarm_min <= 0) alarm_min = 0;
    }
}
// decrement time by one to be more precise
void precisionDown()
{
    if (set_curr_hour)
    {
        curr_hour--;
        if (curr_hour >= 24) curr_hour = 0;
        if (curr_hour <= 0) curr_hour = 0;
    }
    else if (set_curr_min)
    {
        curr_min--;
        if (curr_min >= 60) curr_min = 0;
        if (curr_min <= 0) curr_min = 0;
    }
    else if (set_alarm_hour)
    {
        alarm_hour--;
        if (alarm_hour >= 24) alarm_hour = 0;
        if (alarm_hour <= 0) alarm_hour = 0;
    }
    else if (set_alarm_min)
    {
        alarm_min--;
        if (alarm_min >= 60) alarm_min = 0;
        if (alarm_min <= 0) alarm_min = 0;
    }
}

/// RPG rotary callback ///
//Encoder bit change interrupt service routine
//called whenever one of the two A,B encoder bits change state
void Enc_change_ISR(void)
{
    new_enc = RPG_A<<1 | RPG_B;//current encoder bits
    //check truth table for -1,0 or +1 added to count
    enc_count = lookup_table[old_enc<<2 | new_enc];
    old_enc = new_enc;
    // logic to change times might actually need to be here
    // check if a set time flag is set and then update 
    if (set_curr_hour)
    {
        if (enc_count == 1) curr_hour++;
        if (enc_count == -1) curr_hour--;
        if (curr_hour >= 24) curr_hour = 0;
        if (curr_hour <= 0) curr_hour = 0;
    }
    else if (set_curr_min)
    {
        if (enc_count == 1) curr_min++;
        if (enc_count == -1) curr_min--;
        if (curr_min >= 60) curr_min = 0;
        if (curr_min <= 0) curr_min = 0;
    }
    else if (set_alarm_hour)
    {
        if (enc_count == 1) alarm_hour++;
        if (enc_count == -1) alarm_hour--;
        if (alarm_hour >= 24) alarm_hour = 0;
        if (alarm_hour <= 0) alarm_hour = 0;
    }
    else if (set_alarm_min)
    {
        if (enc_count == 1) alarm_min++;
        if (enc_count == -1) alarm_min--;
        if (alarm_min >= 60) alarm_min = 0;
        if (alarm_min <= 0) alarm_min = 0;
    }
}

/// RPG pushbutton callback ///
void PB_callback(void)
{
    
    if(alarm)
    {
        alarm = false;
        speaker = 0;
    }
    else
    {
        if (!set_curr_hour && !set_curr_min && !set_alarm_hour && !set_alarm_min)
        {
            set_curr_hour = true;
            ledPB= !ledPB;
        }
        else if (set_curr_hour)
        {
            set_curr_hour = false;
            set_curr_min = true;
            ledPB= !ledPB;
        }
        else if (set_curr_min)
        {
            set_curr_min = false;
            set_alarm_hour = true;
            ledPB= !ledPB;
        }
        else if (set_alarm_hour)
        {
            set_alarm_hour = false;
            set_alarm_min = true;
            ledPB= !ledPB;
        }
        else if (set_alarm_min)
        {
            set_alarm_min = false;
            ledPB= !ledPB;
        }
    }
}

/// MOVEMENT THREAD ///
void movement()
{
    while(1)
    {
        if (alarm)
        {
            mu.checkDistance();

            if(turn) left.stop(1);
            else left.speed(0.4);
            right.speed(-0.5);

            left.coast();
            right.coast();
            Thread::wait(100);

        }
        else
        {
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
        if (curr_hour == alarm_hour && curr_min == alarm_min && !set_curr_hour 
                        && !set_curr_min && !set_alarm_hour && !set_alarm_min) alarm = true;
        // wait for 60 seconds
        Thread::wait(60 * 1000);
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
    }
}
void speakerThread()
{
    if (alarm)
    {
        // turn on timer interrupts to start sine wave output
        // sample rate is 500Hz with 128 samples per cycle on sine wave
        Sample_Period.attach(&speaker_interrupt, 1.0/(1000.0*128));
    }
    else
    {
        Sample_Period.detach();
    }
    Thread::wait(1000);
}

/// MAIN FUNCTION ///
int main() {
    //debounce RPG center pushbutton and other pushbuttons
    RPG_PB.mode(PullDown);
    RPG_PB.attach_deasserted(&PB_callback);
    RPG_PB.setSampleFrequency();
    up.mode(PullUp);
    up.attach_deasserted(&precisionUp);
    up.setSampleFrequency();
    down.mode(PullUp);
    down.attach_deasserted(&precisionDown);
    down.setSampleFrequency();
    // generate an interrupt on any change in either encoder bit (A or B)
    RPG_A.mode(PullUp);
    RPG_B.mode(PullUp);
    RPG_A.rise(&Enc_change_ISR);
    RPG_A.fall(&Enc_change_ISR);
    RPG_B.rise(&Enc_change_ISR);
    RPG_B.fall(&Enc_change_ISR);
    // set up speaker
    // precompute 128 sample points on one sine wave cycle 
    // used for continuous sine wave output later
    for(int k=0; k<128; k++) {
        Analog_out_data[k]=((1.0 + sin((float(k)/128.0*6.28318530717959)))/2.0);
        // scale the sine wave from 0.0 to 1.0 - as needed for AnalogOut arg 
    }

    // clear uLCD
    uLCD.cls();
    // init sonar
    mu.startUpdates();
    
    // Start threads
    Thread speaker(speaker_interrupt);
    Thread time(updateTime);
    Thread move(movement);

    /// MAIN THREAD ///
    while(1) {
        uLCD_lock.lock();

        uLCD.locate(1, 1);
        uLCD.text_width(2);
        uLCD.text_height(2);
        uLCD.printf("%02d : %02d", curr_hour, curr_min);

        uLCD.locate(2, 3);
        uLCD.text_width(1);
        uLCD.text_height(1);
        uLCD.printf("%02d : %02d", alarm_hour, alarm_min);

        uLCD.locate(2, 8);
        if (set_curr_hour) uLCD.printf("MODE: 1");
        else if (set_curr_min) uLCD.printf("MODE: 2");
        else if (set_alarm_hour) uLCD.printf("MODE: 3");
        else if (set_alarm_min) uLCD.printf("MODE: 4");
        else uLCD.printf("MODE: 0");

        uLCD_lock.unlock();

        Thread::wait(50);
    }
}
