#ifndef RELATED_FUNCS_H
#define RELATED_FUNCS_H
#include "BLE_example.h"

/********************************************************************************************************************
   JUMP
    Functions:
      void setupWdt()                     : Setup WatchDogTimer (WDT) - reboots if timer2 gets to 0
                                            - Details in BLE-example_funcs.cpp scheduler_timer_init function.
      extern void am_watchdog_isr(void)   : Interrupt for WatchDogTimer - print message RE-BOOTING
      extern void timer_isr(void)         : Timer2 isr - sends e n q (HEX) to phone which replies with ACK handled in amdtps_main.c CNF function
                                              to reset the WDT
      void trigger_timers()               : Set/Check main timers/interrupts - allows sleeping/waking properly



 *******************************************************************************************************************/

int i_B4Timer2_LooperCount = 100;

/********************************************************************************************************************
                Enq Variable
 *******************************************************************************************************************/
//e = 101 dec. = 0x65, n = 110 = 0x6e, q = 113 = 0x71
uint8_t valEnq[] = {(byte)0x65, (byte)0x6e, (byte)0x71};
// *********************************************************************
// Global variables WDT
// *********************************************************************
volatile uint8_t watchdogCounter = 0; // Watchdog interrupt counter
uint32_t resetStatus = 0;             // Reset status register

// Watchdog timer configuration structure.
am_hal_wdt_config_t g_sWatchdogConfig = {
    //g_sWatchdogConfig.ui16InterruptCount = 500 ui16ResetCount

    // Configuration values for generated watchdog timer event.
    .ui32Config = AM_HAL_WDT_LFRC_CLK_16HZ | AM_HAL_WDT_ENABLE_RESET | AM_HAL_WDT_ENABLE_INTERRUPT,

    // Number of watchdog timer ticks allowed before a watchdog interrupt event is generated.
    .ui16InterruptCount = 93, // 20 Set WDT interrupt timeout for 10 seconds (80 / 16 = 5).

    // Number of watchdog timer ticks allowed before the watchdog will issue a system reset.
    .ui16ResetCount = 93 // 60 Set WDT reset timeout for 15 seconds (240 / 16 = 15). was 240
};

/********************************************************************************************************************
                Timer2 VARIABLES
 *******************************************************************************************************************/
static int myTimer = 2;
static int blinkPin = LED_BUILTIN;

int count = 0;

void setupWdt()
{
  Serial.println("############## setting up WatchDogTimer WDT ########################");
  Serial.print("Interrupt Count = ");
  Serial.print(g_sWatchdogConfig.ui16InterruptCount);
  Serial.println(" ticks");
  Serial.print("Reset Count = ");
  Serial.print(g_sWatchdogConfig.ui16ResetCount);
  Serial.println(" ticks");

  // (Note: See am_hal_reset.h for RESET status structure)
  am_hal_reset_status_t sStatus;

  // Print out reset status register.
  // (Note: Watch Dog Timer reset = 0x40)
  am_hal_reset_status_get(&sStatus);
  resetStatus = sStatus.eStatus;

  char rStatus[3];
  sprintf(rStatus, "Reset Status Register = 0x%x\n", resetStatus);
  Serial.println(rStatus);

  // Set the clock frequency.
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

  // Set the default cache configuration
  am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
  am_hal_cachectrl_enable();

  // Configure the board for low power operation.
  am_bsp_low_power_init();

  // Clear reset status register for next time we reset.
  am_hal_reset_control(AM_HAL_RESET_CONTROL_STATUSCLEAR, 0);

  // LFRC must be turned on for this example as the watchdog only runs off of the LFRC.
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
}

// *****************************************************************************
//
// Interrupt handler for the watchdog.
//
// *****************************************************************************
extern void am_watchdog_isr(void)
{
  Serial.printf("\n\n\n\n\n@@@@@@@@@@@ GOT WATCHDOG TIMER - Counter = %d !!!!!!!!RE-BOOTING!!!!!!!!\n", watchdogCounter);

  // Clear the watchdog interrupt.
  am_hal_wdt_int_clear();

  // Catch the first four watchdog interrupts, but let the fifth through untouched.
  if (watchdogCounter < 4)
  {
    // Restart the watchdog.
    //am_hal_wdt_restart(); // "Pet" the dog.
    if (watchdogCounter == 3)
    {
      Serial.printf("\n\n@@@@@@@ RE-BOOTING @@@@@@@@@@@@@@\n");
    }
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH); // Indicator that a reset will occur.
                                     //print statements won't be seen as we are already re-booting
  }

  // Increment the number of watchdog interrupts.
  watchdogCounter++;
}

extern void timer_isr(void)
{
  count++;
  uint32_t ui32Status;
  Serial.printf("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@ Timer2 ISR @@@@@@@@@@@@@@@@@=========> Count = %d\n", count);
  //Serial.printf("@@@@ WATCHDOG TIMER COUNTER = %d\n",watchdogCounter);
  ui32Status = am_hal_ctimer_int_status_get(true);
  am_hal_ctimer_int_clear(ui32Status);
  if (count % 2 == 0)
  {
    digitalWrite(blinkPin, LOW);
  }
  else
  {
    digitalWrite(blinkPin, HIGH);
  }
  i_B4Timer2_LooperCount = 10; //Used this to count number of times loop loops every wakeup
  // Restart the watchdog.
  //am_hal_wdt_restart(); // Stop re-boot
  amdtpsSendData((uint8_t *)valEnq, 3); //Sending phone Hex for e n q = uint8_t valEnq [] = {(byte)0x65,(byte)0x6e,(byte)0x71};
  if (count == 2)
  {
    am_hal_wdt_start();
    Serial.printf("\n\n\n\n\n@@@@@@@@@@@@@@@@@@@@@ STARTING WATCHDOGTIMER WDT STARTING @@@@@@@@@@@@@@@@@@@\n\n\n\n\n");
  }
  // Phone is set up to respond
}

/*
 * This routine wl update the WSF timers on a regular base
 */

void trigger_timers() //Called from loop when the chip is awakened
{
  //
  // Calculate the elapsed time from our free-running timer, and update
  // the software timers in the WSF scheduler.
  //
  update_scheduler_timers(); //We've woken up so set/check timers in BLE_exampole_funcs.cpp
  wsfOsDispatcher();         // start any handlers if event is pending on them

  //
  // Enable an interrupt to wake us up next time we have a scheduled event.
  //
  set_next_wakeup();
}
#endif