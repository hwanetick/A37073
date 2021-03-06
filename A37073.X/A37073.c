// A37073 PWM Flow Meter Converter

#include "A37073.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);

void InitializeFlowMeter(TYPE_FLOW_READING* flow_ptr, unsigned int* ptr_icxbuf, unsigned int* ptr_tmrx);
void UpdateFlowMeterInputCapture(TYPE_FLOW_READING* flow_ptr);
void CaptureInput(TYPE_FLOW_READING* flow_ptr);
void CheckFlowMeter(TYPE_FLOW_READING* flow_ptr);

LTC265X U14_LTC2656;

TYPE_FLOW_GLOBALS global_data_A37073;

void InitializeA37073(void);
void DoStateMachine(void);
void DoA37073(void);
void FlashLEDs(void);
void EnableOutputs(void);

unsigned int capture = 0;

int main(void) {
  global_data_A37073.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {
  switch (global_data_A37073.control_state) {
    
  case STATE_STARTUP:
    InitializeA37073();
    global_data_A37073.control_ready = 0;
    global_data_A37073.startup_counter = 0;
    global_data_A37073.run_time_counter = 0;
    global_data_A37073.dac_write_fault = 0;
    while (global_data_A37073.control_state == STATE_STARTUP) {
      DoA37073();
      FlashLEDs();
      if (global_data_A37073.startup_counter >= STARTUP_LED_FLASH_TIME) {
        global_data_A37073.control_state = STATE_TESTING;	
      }
    }
    break;

  case STATE_TESTING:
    global_data_A37073.control_ready = 0;
    global_data_A37073.test_timer = 0;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_LED_B_GREEN = OLL_LED_ON;
    while (global_data_A37073.control_state == STATE_TESTING) {
      DoA37073();
      if (global_data_A37073.test_timer >= COOLING_INTERFACE_BOARD_TEST_TIME) {
        global_data_A37073.control_state = STATE_READY;
      }
      if (global_data_A37073.dac_write_fault) {
        global_data_A37073.control_state = STATE_FAULT;
      }
    }
    break;

  case STATE_READY:
    global_data_A37073.control_ready = 1;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    EnableOutputs();
    while (global_data_A37073.control_state == STATE_READY) {
      DoA37073();
      if (global_data_A37073.dac_write_fault) {
        global_data_A37073.control_state = STATE_FAULT;
      }
    }
    break;
    
  case STATE_FAULT:
    global_data_A37073.control_ready = 1;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    while (global_data_A37073.control_state == STATE_FAULT) {
      DoA37073();
      if (global_data_A37073.dac_write_fault == 0) {
        global_data_A37073.control_state = STATE_TESTING;
      }
    }
    break;

  default:
    global_data_A37073.control_state = STATE_STARTUP;
    break;
  }
}

void DoA37073(void) {

  ClrWdt();

  // This needs to happen every time through the control loop to capture high frequency PWM
  UpdateFlowMeterInputCapture(&global_data_A37073.flow_meter_1);
  UpdateFlowMeterInputCapture(&global_data_A37073.flow_meter_2);
  UpdateFlowMeterInputCapture(&global_data_A37073.flow_meter_3);
  UpdateFlowMeterInputCapture(&global_data_A37073.flow_meter_4);
  UpdateFlowMeterInputCapture(&global_data_A37073.flow_meter_5);
  
  if (_T1IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms)
    _T1IF = 0;

    global_data_A37073.run_time_counter++;

    global_data_A37073.test_timer++;
    if (global_data_A37073.test_timer > COOLING_INTERFACE_BOARD_TEST_TIME) {
      global_data_A37073.test_timer = COOLING_INTERFACE_BOARD_TEST_TIME;
    }
    
    
    global_data_A37073.startup_counter++;
    if (global_data_A37073.startup_counter > STARTUP_LED_FLASH_TIME) {
      global_data_A37073.startup_counter = STARTUP_LED_FLASH_TIME;
    }
    

    CheckFlowMeter(&global_data_A37073.flow_meter_1);
    CheckFlowMeter(&global_data_A37073.flow_meter_2);
    CheckFlowMeter(&global_data_A37073.flow_meter_3);
    CheckFlowMeter(&global_data_A37073.flow_meter_4);
    CheckFlowMeter(&global_data_A37073.flow_meter_5);
    
    
    global_data_A37073.analog_output_flow_1.set_point = global_data_A37073.flow_meter_1.flow_reading;
    global_data_A37073.analog_output_flow_2.set_point = global_data_A37073.flow_meter_2.flow_reading;
    global_data_A37073.analog_output_flow_3.set_point = global_data_A37073.flow_meter_3.flow_reading;
    global_data_A37073.analog_output_flow_4.set_point = global_data_A37073.flow_meter_4.flow_reading;
    global_data_A37073.analog_output_flow_5.set_point = global_data_A37073.flow_meter_5.flow_reading;
    global_data_A37073.analog_output_spare.set_point = DAC_TEST_VALUE;
    

    ETMAnalogScaleCalibrateDACSetting(&global_data_A37073.analog_output_flow_1);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A37073.analog_output_flow_2);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A37073.analog_output_flow_3);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A37073.analog_output_flow_4);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A37073.analog_output_flow_5);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A37073.analog_output_spare);
    
//    // Debugging Analog Outputs
//    if (global_data_A37073.control_state == STATE_READY) {
////      if (global_data_A37073.analog_output_spare.dac_setting_scaled_and_calibrated == 0) {
////        PIN_LED_A_RED = OLL_LED_ON;
////      } else if (global_data_A37073.analog_output_spare.dac_setting_scaled_and_calibrated == 0x1000) {
////        PIN_LED_B_GREEN = OLL_LED_ON; 
////      }
//      if (capture == 0) {
//        capture = 1;
//        if (global_data_A37073.analog_output_spare.dac_setting_scaled_and_calibrated == 0) {
//          PIN_LED_A_RED = OLL_LED_ON;
//        } else if (global_data_A37073.analog_output_spare.dac_setting_scaled_and_calibrated == 0x7FFF) {
//          PIN_LED_B_GREEN = OLL_LED_ON; 
//        }
//      }
//    }
    
    global_data_A37073.dac_array[0] = global_data_A37073.analog_output_flow_1.dac_setting_scaled_and_calibrated;
    global_data_A37073.dac_array[1] = global_data_A37073.analog_output_flow_2.dac_setting_scaled_and_calibrated;
    global_data_A37073.dac_array[2] = global_data_A37073.analog_output_flow_3.dac_setting_scaled_and_calibrated;
    global_data_A37073.dac_array[3] = global_data_A37073.analog_output_flow_4.dac_setting_scaled_and_calibrated;
    global_data_A37073.dac_array[4] = global_data_A37073.analog_output_flow_5.dac_setting_scaled_and_calibrated;
    global_data_A37073.dac_array[5] = global_data_A37073.analog_output_spare.dac_setting_scaled_and_calibrated;
    
    unsigned int *ptr_dac_array;
    
    ptr_dac_array = global_data_A37073.dac_array;
    
    // Write to all DAC outputs every 80ms
    if (((global_data_A37073.run_time_counter & 0b111) == 0b111) || (global_data_A37073.dac_write_fault == 1)) {
      if (WriteLTC2656AllDacChannels(&U14_LTC2656, ptr_dac_array)) {
        global_data_A37073.dac_write_fault = 1;
      } else {
        global_data_A37073.dac_write_fault = 0;   
      } 
    }
    
  }
}


void InitializeA37073(void) {

  
  // Initialize all I/O Registers
  TRISA = A37073_TRISA_VALUE;
  TRISB = A37073_TRISB_VALUE;
  TRISC = A37073_TRISC_VALUE;
  TRISD = A37073_TRISD_VALUE;
  TRISF = A37073_TRISF_VALUE;
  TRISG = A37073_TRISG_VALUE;


  // Initialize TMR2
  PR2 = PR2_VALUE_MAX;
  TMR2 = 0;
  T2CON = T2CON_VALUE;


  // Initialize TMR1
  PR1   = PR1_VALUE_10_MILLISECONDS;
  TMR1  = 0;
  _T1IF = 0;
  T1CON = T1CON_VALUE;


  // Initialize LTC DAC
  SetupLTC265X(&U14_LTC2656, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
    
  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);


  if (!ETMAnalogCheckEEPromInitialized()) {
    ETMAnalogLoadDefaultCalibration();
    ETMEEPromWriteWord(0x0180, 65100);
    ETMEEPromWriteWord(0x0181, 0x31);
  }
  
  // ----------------------- Initialize on Board DAC Outputs ---------------------------- //  
  ETMAnalogInitializeOutput(&global_data_A37073.analog_output_flow_1,
                            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_FLOW_FIXED_SCALE),
                            DAC_MONITOR_FLOW_FIXED_OFFSET,
                            ANALOG_OUTPUT_1,
                            0xFFFF,
                            0,
                            0);
  
  ETMAnalogInitializeOutput(&global_data_A37073.analog_output_flow_2,
                            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_FLOW_FIXED_SCALE),
                            DAC_MONITOR_FLOW_FIXED_OFFSET,
                            ANALOG_OUTPUT_2,
                            0xFFFF,
                            0,
                            0);
  
  ETMAnalogInitializeOutput(&global_data_A37073.analog_output_flow_3,
                            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_FLOW_FIXED_SCALE),
                            DAC_MONITOR_FLOW_FIXED_OFFSET,
                            ANALOG_OUTPUT_3,
                            0xFFFF,
                            0,
                            0);
    
  ETMAnalogInitializeOutput(&global_data_A37073.analog_output_flow_4,
                            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_FLOW_FIXED_SCALE),
                            DAC_MONITOR_FLOW_FIXED_OFFSET,
                            ANALOG_OUTPUT_4,
                            0xFFFF,
                            0,
                            0);
    
  ETMAnalogInitializeOutput(&global_data_A37073.analog_output_flow_5,
                            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_FLOW_FIXED_SCALE),
                            DAC_MONITOR_FLOW_FIXED_OFFSET,
                            ANALOG_OUTPUT_5,
                            0xFFFF,
                            0,
                            0);
    
  ETMAnalogInitializeOutput(&global_data_A37073.analog_output_spare,
                            MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_SPARE_FIXED_SCALE),
                            DAC_MONITOR_SPARE_FIXED_OFFSET,
                            ANALOG_OUTPUT_6,
                            0xFFFF,
                            0,
                            0);
  
  // Disable analog outputs  
  global_data_A37073.analog_output_flow_1.enabled = 0;
  global_data_A37073.analog_output_flow_2.enabled = 0;
  global_data_A37073.analog_output_flow_3.enabled = 0;
  global_data_A37073.analog_output_flow_4.enabled = 0;
  global_data_A37073.analog_output_flow_5.enabled = 0;
  global_data_A37073.analog_output_spare.enabled  = 0;
    
  // Set unused DAC channels
  global_data_A37073.dac_array[6] = 0;
  global_data_A37073.dac_array[7] = 0;

  // Initialize the Analog Input & Output Scaling

  InitializeFlowMeter(&global_data_A37073.flow_meter_1, (unsigned int*)&IC1BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37073.flow_meter_2, (unsigned int*)&IC2BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37073.flow_meter_3, (unsigned int*)&IC3BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37073.flow_meter_4, (unsigned int*)&IC4BUF, (unsigned int*)&TMR2);
  InitializeFlowMeter(&global_data_A37073.flow_meter_5, (unsigned int*)&IC5BUF, (unsigned int*)&TMR2);
  
  IC1CON = ICXCON_VALUE;
  IC2CON = ICXCON_VALUE;
  IC3CON = ICXCON_VALUE;
  IC4CON = ICXCON_VALUE;
  IC5CON = ICXCON_VALUE;

}


void FlashLEDs(void) {
  switch (((global_data_A37073.startup_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = !OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_B_GREEN = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_LED_A_RED = OLL_LED_ON;
    PIN_LED_B_GREEN = OLL_LED_ON;
    break;
  }
}

void EnableOutputs(void)  {
  global_data_A37073.analog_output_flow_1.enabled = 1;
  global_data_A37073.analog_output_flow_2.enabled = 1;
  global_data_A37073.analog_output_flow_3.enabled = 1;
  global_data_A37073.analog_output_flow_4.enabled = 1;
  global_data_A37073.analog_output_flow_5.enabled = 1;
  global_data_A37073.analog_output_spare.enabled  = 1;
  
}

void InitializeFlowMeter(TYPE_FLOW_READING* flow_ptr, unsigned int* ptr_icxbuf, unsigned int* ptr_tmrx) {
  flow_ptr->period_array[0] = PWM_MAX_PERIOD;
  flow_ptr->period_array[1] = PWM_MAX_PERIOD;
  flow_ptr->period_array[2] = PWM_MAX_PERIOD;
  flow_ptr->period_array[3] = PWM_MAX_PERIOD;

  flow_ptr->period_array[4] = PWM_MAX_PERIOD;
  flow_ptr->period_array[5] = PWM_MAX_PERIOD;
  flow_ptr->period_array[6] = PWM_MAX_PERIOD;
  flow_ptr->period_array[7] = PWM_MAX_PERIOD;

  flow_ptr->period_array[8] = PWM_MAX_PERIOD;
  flow_ptr->period_array[9] = PWM_MAX_PERIOD;
  flow_ptr->period_array[10] = PWM_MAX_PERIOD;
  flow_ptr->period_array[11] = PWM_MAX_PERIOD;

  flow_ptr->period_array[12] = PWM_MAX_PERIOD;
  flow_ptr->period_array[13] = PWM_MAX_PERIOD;
  flow_ptr->period_array[14] = PWM_MAX_PERIOD;
  flow_ptr->period_array[15] = PWM_MAX_PERIOD;
    
//  flow_ptr->minimum_flow = min_flow;
  flow_ptr->ICXBUF_ptr = ptr_icxbuf;
  flow_ptr->TMRX_ptr = ptr_tmrx;
  ETMDigitalInitializeInput(&flow_ptr->digital_fault, 0, 100); // Initialize to not faulted with 1 second delay.
}


void UpdateFlowMeterInputCapture(TYPE_FLOW_READING* flow_ptr) {
  unsigned int latest_tmr;

  // Check to see if there is an overflow
  latest_tmr     = *flow_ptr->TMRX_ptr;
  latest_tmr    -= flow_ptr->previous_timer_reading;
  if (latest_tmr >= PWM_MAX_PERIOD) { 
    flow_ptr->long_pulse = 1;
    flow_ptr->array_index++;
    flow_ptr->array_index &= 0x000F;
    flow_ptr->period_array[flow_ptr->array_index] = PWM_MAX_PERIOD;
  }

  // Figure out if the interupt flag is active and if so update that flow meter
  if (flow_ptr->ICXBUF_ptr == &IC1BUF) {
    if (_IC1IF) {
      _IC1IF = 0;
      CaptureInput(flow_ptr);
    }
  }
  
  if (flow_ptr->ICXBUF_ptr == &IC2BUF) {
    if (_IC2IF) {
      _IC2IF = 0;
      CaptureInput(flow_ptr);
    }
  }
  
  if (flow_ptr->ICXBUF_ptr == &IC3BUF) {
    if (_IC3IF) {
      _IC3IF = 0;
      CaptureInput(flow_ptr);
    }
  } 
  
  if (flow_ptr->ICXBUF_ptr == &IC4BUF) {
    if (_IC4IF) {
      _IC4IF = 0;
      CaptureInput(flow_ptr);
    }
  } 
  
  if (flow_ptr->ICXBUF_ptr == &IC5BUF) {
    if (_IC5IF) {
      _IC5IF = 0;
      CaptureInput(flow_ptr);
    }
  } 
  
}

void CaptureInput(TYPE_FLOW_READING* flow_ptr) {
  unsigned int latest_capture;
  unsigned int current_period;

  while((*((flow_ptr->ICXBUF_ptr) + 1)) & 0x0008) {
    //while(IC1CON & 0x0008) {
    // ICBNE bit is set

    latest_capture = *flow_ptr->ICXBUF_ptr;
    current_period = latest_capture - flow_ptr->previous_timer_reading;
    flow_ptr->previous_timer_reading = latest_capture;
    if (flow_ptr->long_pulse) {
      flow_ptr->long_pulse = 0;
      current_period = PWM_MAX_PERIOD;
    }
    flow_ptr->array_index++;
    flow_ptr->array_index &= 0x000F;
    flow_ptr->period_array[flow_ptr->array_index] = current_period;
  }
}

void CheckFlowMeter(TYPE_FLOW_READING* flow_ptr) {
  unsigned long period_lng;
  unsigned int period;
  period_lng  = flow_ptr->period_array[0];
  period_lng += flow_ptr->period_array[1];
  period_lng += flow_ptr->period_array[2];
  period_lng += flow_ptr->period_array[3];

  period_lng += flow_ptr->period_array[4];
  period_lng += flow_ptr->period_array[5];
  period_lng += flow_ptr->period_array[6];
  period_lng += flow_ptr->period_array[7];

  period_lng += flow_ptr->period_array[8];
  period_lng += flow_ptr->period_array[9];
  period_lng += flow_ptr->period_array[10];
  period_lng += flow_ptr->period_array[11];

  period_lng += flow_ptr->period_array[12];
  period_lng += flow_ptr->period_array[13];
  period_lng += flow_ptr->period_array[14];
  period_lng += flow_ptr->period_array[15];

  period_lng >>= 3;
  period = period_lng;

  if (period <= PERIOD_MAX_FREQUENCY) {
    period = PERIOD_MAX_FREQUENCY;
  }
  //period_lng = 390620;
  //period_lng /= period;
  //flow_ptr->frequency = period_lng;  // This is now in deci
  //flow_ptr->frequency = RCFilterNTau(flow_ptr->frequency, period_lng, 12);
  flow_ptr->frequency = 39062 / period;
  
  if (flow_ptr->frequency < FLOW_METER_MIN_FREQUENCY) {
    flow_ptr->flow_reading = 0;
  } else {
    flow_ptr->flow_reading = FLOW_METER_ML_PER_HZ*flow_ptr->frequency + FLOW_METER_CONSTANT;
    //flow_ptr->flow_reading = RCFilterNTau(flow_ptr->flow_reading, (FLOW_METER_ML_PER_HZ*flow_ptr->frequency + FLOW_METER_CONSTANT), RC_FILTER_256_TAU);
  }

  return;
}
