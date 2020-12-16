/*
  04/19/2020 Kerry Edited
  AndroidArtemisBleUartClient
  https://github.com/kerryeven/AndroidArtemisBleUartClient

  June 2020 Ta-Weh Yeh Edited
  https://github.com/TaWeiYeh/ArtemisBleApp

  August 2020 Alex Coy Tailor for ECE 4960
*/
// maximum length of reply / data message
#define MAXREPLY 100
#define TODO_VAL 0
// buffer to reply to client
uint8_t val[MAXREPLY];
uint8_t *val_data = &val[2]; // start of optional data area
uint8_t *val_len = &val[1];  // store length of optional data

/********************************************************************************************************************
                 INCLUDES
 *******************************************************************************************************************/
#include "BLE_example.h"
#include "commands.h"
#include "related_funcs.h"

/*
#include "SCMD.h"
#include "SCMD_config.h"
*/

/*
#define ARM_MATH_CM4
#include <arm_math.h>
#include <math.h>
*/
/********************************************************************************************************************
                  OBJECTS
  *******************************************************************************************************************/

/********************************************************************************************************************
                GLOBAL VARIABLES
 *******************************************************************************************************************/
String s_Rev = "Rev 1.0";
String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages, if 100 don't bother checking case statements
uint16_t l_Rcvd = 0;
uint8_t *m_Rcvd = NULL;
String s_AdvName = "MyRobot"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped
uint16_t package = 0;

cmd_t empty_cmd = {NOT_A_COMMAND, 1, {0}};
cmd_t *cmd = &empty_cmd;
cmd_t *res_cmd = &empty_cmd;
bt_debug_msg_t *bt_debug_head = NULL;
bt_debug_msg_t *bt_debug_tail = NULL;
//SCMD motorDriver;
present_t presentSensors = {
    .motorDriver = 0,
    .ToF_sensor = 0,
    .prox_sensor = 0,
    .IMU = 0};
int bytestream_active = 0;

/*************************************************************************************************/
/*!
     \fn     setup

     \brief  Arduino setup function.  Set up the board BLE and GPIO - BLE first...

     \param  none

     \called Arduino startup

     \return None.
 */
/*************************************************************************************************/
void setup()
{
    Serial.begin(115200);
    delay(1000);

#ifdef BLE_SHOW_DATA
//SERIAL_PORT.begin(115200);
//delay(1000);
//SERIAL_PORT.printf("Viper. Compiled: %s\n" , __TIME__);
#endif

#ifdef AM_DEBUG_PRINTF
    //
    // Enable printing to the console.
    //
    enable_print_interface();
#endif

    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    Serial.print("Revision = ");
    Serial.print(s_Rev);
    Serial.printf("  ECE 4960 Robot Compiled: %s   %s\n", __DATE__, __TIME__);
    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535 (but make max 64k or trouble)

    /********************************************************************************************************************
                    Set Advertising name:  uses global string s_AdvName set above.
     *******************************************************************************************************************/
    set_Adv_Name(); //BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h

    /********************************************************************************************************************
                     Boot the radio
                      SDK/third_party/exactle/sw/hci/apollo3/hci_drv_apollo3.c
                      = huge program to handle the ble radio stuff in this file
     *******************************************************************************************************************/
    HciDrvRadioBoot(0);

    /************************************************************************************************
          Initialize the main ExactLE stack: BLE_example_funcs.cpp
          - One time timer
          - timer for handler
          - dynamic memory buffer
          - security
          - HCI host conroller interface
          - DM device manager software
          - L2CAP data transfer management
          - ATT - Low Energy handlers
          - SMP - Low Energy security
          - APP - application handlers..global settings..etc
          - NUS - nordic location services

     ************************************************************************************************/
    exactle_stack_init();

    /*************************************************************************************************
        Set the power level to it's maximum of 15 decimal...defined in hci_drv_apollo3.h as 0xF
        needs to come after the HCI stack is initialized in previous line
          - poss. levels = 0x03=-20,0x04=-10,0x05=-5,0x08=0,0x0F=4 but have to use definitions, not these ints
            extremes make a difference of about 10 at 1 foot.
     ************************************************************************************************/
    HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm); //= 15 decimal = max power WORKS..default = 0

    /*************************************************************************************************
        Start the "Amdtp" (AmbiqMicro Data Transfer Protocol) profile. Function in amdtp_main.c

         Register for stack callbacks
         - Register callback with DM for scan and advertising events with security
         - Register callback with Connection Manager with client id
         - Register callback with ATT low energy handlers
         - Register callback with ATT low enerty connection handlers
         - Register callback with ATT CCC = client charachteristic configuration array
         - Register for app framework discovery callbacks
         - Initialize attribute server database
         - Reset the device

     ************************************************************************************************/
    AmdtpStart();

    /*************************************************************************************************
       On first boot after upload and boot from battery, pwm on pin 14 not working
        need to reset nano board several times with battery power applied to get
        working.  Delay 5 seconds works..haven't tried lesser values.
     ************************************************************************************************/
    //delay(5000);

    /************************************************************************************************
        Arduino device GPIO control setup.
          Place after board BLE setup stuff happens.  ie.:
            could not get A14 to PWM untill I moved the set_stop() call from the
            beginning of setup to this location...then works great.
     ************************************************************************************************/

    pinMode(LED_BUILTIN, OUTPUT);

    //Set a starting point...for motors, servo, and LED_BUILTIN
    //delay(1000);

    // Bluetooth would start after blinking
    for (int i = 0; i < 20; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    //setupwdt();

    pinMode(blinkPin, OUTPUT);
    digitalWrite(blinkPin, LOW);

    // Configure the watchdog.
    //setupTimerA(myTimer, 31); // timerNum, period - //moved to BLE_example_funcs.cpp scheduler_timer_init
    setupWdt();
    am_hal_wdt_init(&g_sWatchdogConfig);
    //NVIC_EnableIRQ(CTIMER_IRQn); // Enable CTIMER interrupt in nested vector interrupt controller.
    NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.

    uint8_t a = 0;
    m_Rcvd = &a;
    //Serial.printf("Size of command: %d", sizeof(cmd_t));
    am_hal_interrupt_master_enable();
    //interrupts(); // Enable interrupt operation. Equivalent to am_hal_rtc_int_enable().
    //am_hal_wdt_start();
    //am_hal_wdt_int_enable(); - freezes boot

} /*** END setup FCN ***/

void loop()
{
    //Serial.println("Loop...."); //KHE Loops constantly....no delays

    if (l_Rcvd > 1) //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
    {

        cmd = (cmd_t *)m_Rcvd;
        /*
        Serial.print("Message Buffer: ");
        for (int i = 0; i < l_Rcvd; i++)
            Serial.printf("%d ", m_Rcvd[i]);
        Serial.println();
        Serial.printf("Got command: 0x%x Length: 0x%x Data: ", cmd->command_type, cmd->length);

        for (int i = 0; i < cmd->length; i++)
        {
            Serial.printf("0x%x ", cmd->data[i]);
        }
        Serial.println();
        */

        switch (cmd->command_type)
        {
        case SET_MOTORS:

            Serial.println("Placeholder: Set Motors");

            break;
        case GET_MOTORS:

            Serial.println("Placeholder: Set Motors");
            //amdtpsSendData((uint8_t *)res_cmd, *val_len);
            break;
        case SER_RX:
            Serial.println("Got a serial message");
            pushMessage((char *)&cmd->data, cmd->length);
            break;
        case REQ_FLOAT:
            Serial.println("Going to send a float");
            //TODO: Put a float (perhaps pi) into a command response and send it.
            res_cmd->command_type = GIVE_FLOAT;
            ((float *)res_cmd->data)[0] = 3.14159265354f;
            amdtpsSendData((uint8_t *)res_cmd, 6);
            break;
        case PING:
            Serial.println("Ping Pong");
            cmd->command_type = PONG;
            amdtpsSendData(m_Rcvd, l_Rcvd);
            break;
        case START_BYTESTREAM_TX:
            bytestream_active = 1; //(int)cmd->data[0];
            //Serial.printf("Start bytestream with active %d \n", bytestream_active);
            ((uint32_t *)res_cmd->data)[0] = 0;
            break;
        case STOP_BYTESTREAM_TX:
            bytestream_active = 0;
            break;
        default:
            Serial.printf("Unsupported Command 0x%x \n", cmd->command_type);
            break;
        }

        l_Rcvd = 0;
        am_hal_wdt_restart();
        free(m_Rcvd);
    } //End if s_Rcvd != 100
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '7'))
    {
        s_Rcvd[0] = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.printf("Connected, length was %d", l_Rcvd);
    }
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '8'))
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("disconnected");
        //Decimal value of D for Disconnect
        //Serial.println("got disconnect from case in ino file - set_Stop");
        digitalWrite(LED_BUILTIN, LOW);
        //amdtps_conn_close();
        DmDevReset();
    }

    if (availableMessage())
    {
        Serial.println("Bluetooth Message:");
        Serial.println(pullMessage());
        printOverBluetooth("Message Received.");
    }

    if (bytestream_active)
    {
        res_cmd->command_type = BYTESTREAM_TX;
        res_cmd->length = 54;
        //TODO: Put an example of a 32-bit integer and a 64-bit integer
        //for the stream. Be sure to add a corresponding case in the
        //python program.
        // Serial.printf("Stream %d \n", bytestream_active);
        uint32_t bit_32 = 2147483648;
        uint64_t bit_64 = 9223372036854775806;
        memcpy(res_cmd->data,&bit_32,4);
        memcpy(res_cmd->data+4,&bit_64,8);
        memcpy(res_cmd->data+12,&bit_64,8);
        memcpy(res_cmd->data+20,&bit_64,8);
        memcpy(res_cmd->data+28,&bit_64,8);
        memcpy(res_cmd->data+36,&bit_64,8);
        memcpy(res_cmd->data+42,&bit_64,8);
        amdtpsSendData((uint8_t *)res_cmd, 54);

        // Log time
        long t = micros();
        Serial.printf("Package: %3d, Time: %3d\n",package,t);
        package++;
    }

    trigger_timers();

    // Disable interrupts.

    /*
    //Uncomment this if you want the board to go to sleep and be woken up
    //by Timer2. Not good for instruction streaming!

    am_hal_interrupt_master_disable();

    //
    // Check to see if the WSF routines are ready to go to sleep.
    //
    if (wsfOsReadyToSleep())
    {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
    // Loop stops here on sleep and wakes on Timer2 interrupt, runs about 30 loops, then sleeps again.
    // An interrupt woke us up so now enable them and take it.
    am_hal_interrupt_master_enable();
    */

    delay(10);
} //END LOOP
