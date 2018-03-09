/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC1350

 ******************************************************************************
 
 Copyright (c) 2013-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ti-ble-2.3.2-stack-sdk_1_50_xx
 Release Date: 2017-09-27 14:52:16
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <stdint.h>
#include <ti/sysbios/hal/Seconds.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
/* Driver Header files */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/PWM.h>
/* Example/Board Header files */
#include "Board.h"
#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "sysctl.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#ifdef USE_CORE_SDK
  #include <ti/display/Display.h>
#else // !USE_CORE_SDK
  #include <ti/mw/display/Display.h>
#endif // USE_CORE_SDK
#include "board_key.h"

#include "board.h"

#include "simple_peripheral.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #ifdef USE_CORE_SDK
    #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
      #define SBP_DISPLAY_TYPE Display_Type_LCD
    #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
      #define SBP_DISPLAY_TYPE Display_Type_UART
    #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
      #define SBP_DISPLAY_TYPE 0 // Option not supported
    #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #else // !USE_CORE_SDK
    #if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
      #define SBP_DISPLAY_TYPE Display_Type_LCD
    #elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
      #define SBP_DISPLAY_TYPE Display_Type_UART
    #else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
      #define SBP_DISPLAY_TYPE 0 // Option not supported
    #endif // !BOARD_DISPLAY_EXCLUDE_LCD || !BOARD_DISPLAY_EXCLUDE_UART
  #endif // USE_CORE_SDK
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define SBP_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;
struct tm ltm;
static int timeToSet[5];
static int wantedTime[2];
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;
static PIN_State buttonPinState;
static PIN_State ledPinState;
static PIN_Handle lcdHandle;
static PIN_State lcdState;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'B',
  'L',
  'E',
  'P',
  'e',
  'r',
  'i',
  'p',
  'h',
  'e',
  'r',
  'a',
  'l',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // lenght of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};
static int firstPress = 1;
// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void writeTime(char time[]);
static void resetScreen(void);
static void chooseScreen2x16(void);
static void writeSpaces(void);
static void TurnOnDisplay(void);
static void setEntryMode(void);
static void cursorToSecond(void);
static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

PIN_Config ledPinTable[] = {
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

PIN_Config lcdTable[] = {
    //Rs
    Board_DIO25_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //E
    Board_DIO26_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //Buzzer
    Board_DIO27_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //D0
    Board_DIO23_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //D1
    Board_DIO24_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //D2
    Board_DIO28_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    //D3
    Board_DIO29_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    // D4
    Board_DIO12  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    // D5
    Board_DIO15  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    // D6
    Board_DIO21  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    // D7
    Board_DIO22  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_PIN_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}
static int bytesRecieved = 0;
static char buf[80];
static int code[5];
static int dateInBinary[5] = {0,1,1,1,0};
static int codeIndex = 0;
static int prevDate = 0;
static void int_to_bin_digit(unsigned int in)
{
    /* assert: count <= sizeof(int)*CHAR_BIT */
    unsigned int mask = 1U << (4);
    int i;
    for (i = 0; i < 5; i++) {
        dateInBinary[i] = (in & mask) ? 1 : 0;
        in <<= 1;
    }
}

static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    CPUdelay(8000*50);

        if (!PIN_getInputValue(pinId)) {
            if(firstPress == 1){
                cursorToSecond();
                firstPress = 0;
            }
            /* Toggle LED based on the button pressed */
            switch (pinId) {
                case Board_PIN_BUTTON0:
                    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
                    usleep(2000);
                    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
                    usleep(200);
                    code[codeIndex] = 1;
                    break;
                case Board_PIN_BUTTON1:
                    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
                    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
                    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
                    usleep(2000);
                    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
                    usleep(200);
                    code[codeIndex] = 0;
                    break;
            }
            codeIndex++;
            if (codeIndex == 5){
                codeIndex = 0;
                cursorToSecond();
                for (int i = 0; i<5; i++){
                    if(code[i] != dateInBinary[i]){
                        PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 1);
                        writeSpaces();
                        cursorToSecond();
                        return;
                    }
                }
                PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);
                PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(lcdHandle, Board_DIO27_ANALOG, 0);//PIN_GPIO_HIGH

            }
        }
}
/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA
#ifdef Board_shutDownExtFlash
        Board_shutDownExtFlash();
    #endif
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        /* Error initializing board LED pins */
        while(1);
    }

    lcdHandle = PIN_open(&lcdState, lcdTable);
    if(!lcdHandle) {
        /* Error initializing board LCD pins */
        while(1);
    }
    resetScreen();
    chooseScreen2x16();
    TurnOnDisplay();
    setEntryMode();
    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle) {
       /* Error initializing button pins */
       while(1);
    }

       /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
       /* Error registering button callback function */
       while(1);
    }
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

  dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD_ONCHIP
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE


#ifndef FEATURE_OAD_ONCHIP
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD_ONCHIP

  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_LE_ReadMaxDataLenCmd();

#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  Display_print0(dispHandle, 0, 0, "BLE Peripheral A");
#else
  Display_print0(dispHandle, 0, 0, "BLE Peripheral B");
#endif // HAL_IMAGE_A
#else
  Display_print0(dispHandle, 0, 0, "BLE Peripheral");
#endif // FEATURE_OAD
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;

      Util_startClock(&periodicClock);

      // Perform periodic application task
      SimpleBLEPeripheral_performPeriodicTask();
    }

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, 5, 0, "MTU Size: $d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, 2, 0, "Advertising");
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Disconnected");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

static void setTime(int year, int month, int day, int hour, int min){
    memset(&ltm, 0, sizeof(struct tm));
    // Set the struct date / time to 2016 / 5 / 21 18:00:00
    ltm.tm_year = year - 1900;
    ltm.tm_mon = month - 1;
    ltm.tm_mday = day;
    ltm.tm_hour = hour;
    ltm.tm_min = min;
    ltm.tm_sec = 0;
    prevDate = day;
    int_to_bin_digit(day);
    // Convert to number of seconds, this will also fill up tm_wday and tm_yday
    time_t seconds = mktime(&ltm);

    // Convert the struct date to string
    //char* currTime = asctime(&ltm);

    // Convert the number of seconds to string, just to check
    //currTime = ctime(&seconds);

    // Set the date into the system
    Seconds_set(seconds);
}

static void getCurrentDateAndTime(){
    UInt32 seconds = Seconds_get();

    ltm = *localtime(&seconds);

    char buffer [15];
    strftime (buffer, 15, "%d/%m/%y %H:%M", &ltm);
    writeTime(buffer);
    if(ltm.tm_mday != prevDate){
        prevDate = ltm.tm_mday;
        int_to_bin_digit(ltm.tm_mday);
    }
}

static int startRing(){
    UInt32 seconds = Seconds_get();

    ltm = *localtime(&seconds);
    char currTime [6];
    strftime (currTime, 6, "%H", &ltm);
    int hours = atoi(currTime);
    strftime (currTime, 6, "%M", &ltm);
    int minutes = atoi(currTime);
    if(hours == wantedTime[0] && minutes == wantedTime[1]){
        PIN_setOutputValue(lcdHandle, Board_DIO27_ANALOG, PIN_GPIO_HIGH);//PIN_GPIO_HIGH
        return 0;
    }
    return -1;
}
char* split(char* line, char delimiter){ //for parsing config file
    char* start = strchr(line,delimiter);
    start++;
    return start;
}
static void parseTime(char* timeStr){
    char buffer[5];
    snprintf(buffer, 5, "%s", timeStr);
    timeToSet[0] = atoi(buffer);
    timeStr = split(timeStr, '/');
    char buf[3];
    snprintf(buf, 3, "%s", timeStr);
    timeToSet[1] = atoi(buf);
    timeStr = split(timeStr, '/');
    snprintf(buf, 3, "%s", timeStr);
    timeToSet[2] = atoi(buf);
    timeStr = split(timeStr, ' ');
    snprintf(buf, 3, "%s", timeStr);
    timeToSet[3] = atoi(buf);
    timeStr = split(timeStr, ':');
    snprintf(buf, 3, "%s", timeStr);
    timeToSet[4] = atoi(buf);
    timeStr = split(timeStr, ' ');
    snprintf(buf, 3, "%s", timeStr);
    wantedTime[0] = atoi(buf);
    timeStr = split(timeStr, ':');
    snprintf(buf, 3, "%s", timeStr);
    wantedTime[1] = atoi(buf);
}
static void ManageTime(){
    parseTime(buf);
    setTime(timeToSet[0], timeToSet[1], timeToSet[2], timeToSet[3], timeToSet[4]);
    int startRingFlag = 1;
    while(1){
        getCurrentDateAndTime();
        if(startRingFlag){
            if(startRing() == 0){
                startRingFlag = 0;
            }
        }
        sleep(60);
    }
}
static void resetScreen(){
    //reset screen
    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO15, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
    usleep(2000);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
    usleep(200);
}
static void chooseScreen2x16(){
    //reset screen
    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
    usleep(2000);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
    usleep(200);
}
static void TurnOnDisplay(){
    //turn on display and move one space
    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO15, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
    usleep(2000);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
    usleep(200);
}
static void setEntryMode(){
    //turn address to increment
     PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO15, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
     PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
     PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
     usleep(2000);
     PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
     usleep(200);
}
/*static void setScreen1x8(){
    //select 8 bit operation
     PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
     PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
     PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
     PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
     usleep(2000);
     PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
     usleep(200);
}*/
static void cursorToSecond(){
    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO22, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO21, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO15, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
    usleep(2000);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
    usleep(200);
}
static void cursorToFirst(){
    PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO15, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
    PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
    usleep(2000);
    PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
    usleep(200);
}

static void writeSpaces(){
    for(int i =0; i<5; i++){
        PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
        PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
        PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
        PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
        usleep(2000);
        PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
        usleep(200);
    }
}
static void writeTime(char time[]){
    int length = 14;
    cursorToFirst();
    for (int i = 0; i<length; i++){
        switch (time[i]){
        case '0':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '1':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '2':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '3':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '4':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '5':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '6':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '7':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '8':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '9':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case ':':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case '/':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        case ' ':
            PIN_setOutputValue(lcdHandle, Board_DIO25_ANALOG, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO22, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO21, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO15, PIN_GPIO_HIGH);
            PIN_setOutputValue(lcdHandle, Board_DIO12, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO29_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO28_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO24_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO23_ANALOG, 0);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, PIN_GPIO_HIGH);
            usleep(2000);
            PIN_setOutputValue(lcdHandle, Board_DIO26_ANALOG, 0);
            usleep(200);
            break;
        }
    }
}
/*static void writeTimeTest(){
    writeTime("0123456789: /");
    sleep(5);
    writeTime("10:52 11/02");
}*/
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      Display_print1(dispHandle, 4, 0, "Char 1: %d", (uint16_t)newValue);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
      /*if(newValue == 'A'){
          //PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 1);
          PIN_setOutputValue(lcdHandle, Board_DIO27_ANALOG, PIN_GPIO_HIGH);//PIN_GPIO_HIGH
      }
      else{
          if(newValue == 'B'){
              sprintf(buf,"%s", "2018/03/04 13:16 13:18\0") ;

              ManageTime();
          }
          //PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, 0);
          //PIN_setOutputValue(lcdHandle, Board_DIO27_ANALOG, 0);

      }*/
      buf[bytesRecieved] = (char)newValue;
      buf[bytesRecieved+1] = '\0';
      char b[2];
      b[0] = (char)newValue;
      b[1]  = '\0';
      writeTime(b);
      bytesRecieved++;
      if(bytesRecieved == 22){
          ManageTime();
      }

      //PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, newValue);

      Display_print1(dispHandle, 4, 0, "Char 3: %d", (uint16_t)newValue);
      break;

    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD_ONCHIP
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
#endif //!FEATURE_OAD_ONCHIP
}


#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
*********************************************************************/
