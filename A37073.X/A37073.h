/* 
 * File:   A37073.h
 * Author: hwanetick
 *
 * Created on February 16, 2016, 12:21 PM
 */

#ifndef A37073_H
#define	A37073_H

#include <xc.h>
#include <adc12.h>
#include <timer.h>
#include <libpic30.h>
#include <incap.h>
#include "ETM.h"
#include "FIRMWARE_VERSION.h"

#define FCY_CLK     10000000

/*
 
 
 Digital Inputs
 * 
 * FLOW PWM 1  -  PIC FLOW PWM 1  -  (PIN 54)  IC1/RD8
 * FLOW PWM 2  -  PIC FLOW PWM 2  -  (PIN 55)  IC2/RD9
 * FLOW PWM 3  -  PIC FLOW PWM 3  -  (PIN 56)  IC3/RD10
 * FLOW PWM 4  -  PIC FLOW PWM 4  -  (PIN 57)  IC4/RD11
 * FLOW PWM 5  -  PIC FLOW PWM 5  -  (PIN 64)  IC5/RD12
 * 
 * RESET DETECT  -  RG14  PIN 78

 
 Analog Outputs
 * 
 * ANALOG OUT 1  -  DAC A
 * ANALOG OUT 2  -  DAC B
 * ANALOG OUT 3  -  DAC C
 * ANALOG OUT 4  -  DAC D
 * ANALOG OUT 5  -  DAC E
 * ANALOG OUT 6  -  DAC F
 
  
 Digital Outputs
 * 
 * SPI2 SCK2 - RG6 PIN 6
 * SPI2 SDI2 - RG7 PIN 7
 * SPI2 SDO2 - RG8 PIN 8
 * 
 * DAC CD/LD - RG15 PIN 1
 * DAC LDAC  - RC1  PIN 2
 * 
 * I2C SCL - RG2 PIN 47
 * I2C SDA - RG3 PIN 46
 * 
 * LED A RED    -  RG12  PIN 79
 * LED B GREEN  -  RG13  PIN 80
 * LED C GREEN  -  RA7   PIN 77
 * 
 * TEST_POINT_A  -  RF6  PIN 45
 * TEST_POINT_B  -  RF7  PIN 44  
 * TEST_POINT_C  -  RF8  PIN 43  
 */ 

#define TEST_POINT_A                 _LATF6   // PIN 45
#define TEST_POINT_B                 _LATF7   // PIN 44   
#define TEST_POINT_C                 _LATF8   // PIN 43   

#define PIN_LED_OPERATIONAL_GREEN    _LATA7
#define PIN_LED_A_RED                _LATG12
#define PIN_LED_B_GREEN              _LATG13
#define OLL_LED_ON                     0

// State Definitions
#define STATE_STARTUP                  10
#define STATE_FAULT                    20
#define STATE_TESTING                  30
#define STATE_READY                    40

// Flow Meter Fault Setup
#define MINIMUM_FLOW_1                 7800  // TBD
#define MINIMUM_FLOW_2                 7800
#define MINIMUM_FLOW_3                 7800
#define MINIMUM_FLOW_4                 7800
#define MINIMUM_FLOW_5                 7800

#define STARTUP_LED_FLASH_TIME                400   // 4 Seconds
#define COOLING_INTERFACE_BOARD_TEST_TIME     100   // 1 second

#define FLOW_METER_ML_PER_HZ           81
#define FLOW_METER_CONSTANT            841

#define PERIOD_MAX_FREQUENCY           70    // 558 Hz
#define FLOW_METER_MIN_FREQUENCY       15
#define PWM_MAX_PERIOD                 1954  // This is equiv to 10Hz 

#define A37073_TRISA_VALUE 0b1111111100111111
#define A37073_TRISB_VALUE 0b1111111111111111
#define A37073_TRISC_VALUE 0b1111111111111111
#define A37073_TRISD_VALUE 0b1111111111111111
#define A37073_TRISF_VALUE 0b1111111000111111
#define A37073_TRISG_VALUE 0b1100111111111111


#define ICXCON_VALUE  (IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_EDGE)

/* 
   TMR1 Configuration
   Timer1 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 52.4mS, 800ns per tick
*/

#define T1CON_VALUE                    (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SOURCE_INT)
#define PR1_VALUE_10_MILLISECONDS      12500



/* 
   TMR2 Configuration
   Timer2 - PWM frequency Capture
   Period should be set to 10mS
   Maximum Period = 2^16 * 256 / Fcy = 1.6777216  seconds with 10Mhz Clock
*/

#define T2CON_VALUE                    (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_SOURCE_INT)
#define PR2_VALUE_MAX                  0xFFFF


#define DAC_TEST_VALUE                1234

#define DAC_MONITOR_FLOW_FIXED_SCALE   1.927
#define DAC_MONITOR_FLOW_FIXED_OFFSET  0

#define DAC_MONITOR_SPARE_FIXED_SCALE    1
#define DAC_MONITOR_SPARE_FIXED_OFFSET   0


typedef struct {
  unsigned int  flow_reading;           // This is the flow reading in mL per minute
  unsigned int  frequency;              // This is frequency in HZ.  It is caluclated from the period readings in the period array
  // The period array contains a history of the time between input capture events
  // Input capture events occur on the rising and falling edge so period is actually 2 of these times
  // An average period is calculated by adding these all together and diving by 8
  unsigned int  period_array[16];
  unsigned int  array_index;            // This is the index where the next period reading should go
  unsigned int  long_pulse;             // This is used to signal that the period is greater than one timer period
  unsigned int  previous_timer_reading; // The period is current timer reading - the previous timer reading
  unsigned int* ICXBUF_ptr;             // This is pointer to the input capture associate with this PWM input
  unsigned int* TMRX_ptr;               // This is pointer to the timer associated with the input capture module
  unsigned int minimum_flow;            // This is the minimum flow for this particular flow meter
  TYPE_DIGITAL_INPUT digital_fault;     // This is the digital structure used to filter the fault data
} TYPE_FLOW_READING;


typedef struct {

  unsigned int control_state;
  unsigned int startup_counter;
  unsigned int test_timer;
  unsigned int run_time_counter;                // This counts how long the unit has been running for.  It wraps every 11 minutes
  unsigned int dac_array[8];
  unsigned int control_ready;
  unsigned int dac_write_fault;
  
  AnalogOutput analog_output_flow_1;
  AnalogOutput analog_output_flow_2;
  AnalogOutput analog_output_flow_3;
  AnalogOutput analog_output_flow_4;
  AnalogOutput analog_output_flow_5;
  AnalogOutput analog_output_spare;
  

  TYPE_FLOW_READING flow_meter_1;
  TYPE_FLOW_READING flow_meter_2;
  TYPE_FLOW_READING flow_meter_3;
  TYPE_FLOW_READING flow_meter_4; 
  TYPE_FLOW_READING flow_meter_5;  
} TYPE_FLOW_GLOBALS;
 
 
 
 
 
 

#endif	/* A37073_H */

