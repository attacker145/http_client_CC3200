//*****************************************************************************
// Copyright (C) 2014 Texas Instruments Incorporated
//
// All rights reserved. Property of Texas Instruments Incorporated.
// Restricted rights to use, duplicate or disclose this code are
// granted through contract.
// The program may not be used without the written permission of
// Texas Instruments Incorporated or against the terms and conditions
// stipulated in the agreement under which this program has been supplied,
// and under no circumstances can it be used with non-TI connectivity device.
//
//*****************************************************************************


//*****************************************************************************
//
// Application Name     - HTTP Client Demo
// Application Overview - This sample application demonstrates how to use
//                          HTTP Client (In Minimum mode) API for HTTP based
//                          application development.
//                          This application explain user to how to:
//                          1. Connect to an access point
//                          2. Connect to a HTTP Server with and without proxy
//                          3. Do POST, GET, PUT and DELETE
//                          4. Parse JSON data using “Jasmine JSON Parser”
// Note: To use HTTP Client in minimum mode, user need to compile library (webclient)
// 			with HTTPCli_LIBTYPE_MIN option.
//
// 			HTTP Client (minimal) library supports synchronous mode, redirection
// 			handling, chunked transfer encoding support, proxy support and TLS
// 			support (for SimpleLink Only. TLS on other platforms are disabled)
//
// 			HTTP Client (Full) library supports all the features of the minimal
// 			library + asynchronous mode and content handling support +
// 			TLS support (all platforms). To use HTTP Client in full mode user need
//			to compile library (webclient) with HTTPCli_LIBTYPE_MIN option. For full
//			mode RTOS is needed.
//
// Application Details  -
// http://processors.wiki.ti.com/index.php/
// or
// docs\examples\
//
//*****************************************************************************


#include <string.h>

// SimpleLink includes
#include "simplelink.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "utils.h"
#include "interrupt.h"
#include "adc.h"
#include "pin.h"

// common interface includes
#include "uart_if.h"
#include "uart.h"
#include "common.h"
#include "pinmux.h"
#include "i2c_if.h"
#include "hw_memmap.h"
#include "gpio.h"
//#include "serial_wifi.h" //not good
//#include "tmp006drv.h"
//#include "bma222drv.h"

#include "gpio_if.h"
#include "network_if.h"

// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>

// JSON Parser
#include "jsmn.h"

#include <math.h>

#define cred

#define APPLICATION_VERSION "1.1.1"
#define APP_NAME            "HTTP Client Rev-02"

//#define POST_REQUEST_URI 	"/post"
//#define POST_REQUEST_URI 	"/post.html"

#include "network_defines.h"

#define PROXY_IP       	    <proxy_ip>
#define PROXY_PORT          <proxy_port>

#define READ_SIZE           1450
#define MAX_BUFF_SIZE       1460
#define SH_GPIO_3           3       /* P58 - Light Sensor */
#define SH_GPIO_9           9		// Red LED
#define SH_GPIO_22          22      /* P15 - Device Mode */

//*****************************************************************************
//
// Values that can be passed to I2COpen as the ulMode parameter.
//
//*****************************************************************************
#define I2C_MASTER_MODE_STD     0
#define I2C_MASTER_MODE_FST     1

//*****************************************************************************
// BMA222 Device I2C address
//*****************************************************************************
#define BMA222_DEV_ADDR          0x18

#define BMA222_CHID_ID_NUM       0x00

//*****************************************************************************
// BMA222 Acc Data Register related macros
//*****************************************************************************
#define BMA222_ACC_DATA_X_NEW   (0x2)
#define BMA222_ACC_DATA_X   	(0x3)
#define BMA222_ACC_DATA_Y_NEW  	(0x4)
#define BMA222_ACC_DATA_Y   	(0x5)
#define BMA222_ACC_DATA_Z_NEW  	(0x6)
#define BMA222_ACC_DATA_Z   	(0x7)

//*****************************************************************************
// TMP006 Device I2C address
//*****************************************************************************
#define TMP006_DEV_ADDR         0x41

//*****************************************************************************
// TMP006 Register offset address
//*****************************************************************************
#define TMP006_VOBJECT_REG_ADDR         0x0
#define TMP006_TAMBIENT_REG_ADDR        0x1
#define TMP006_CONFIG_REG_ADDR          0x2
#define TMP006_MANUFAC_ID_REG_ADDR      0xFE
#define TMP006_DEVICE_ID_REG_ADDR       0xFF

//*****************************************************************************
// TMP006 Device details
//*****************************************************************************
#define TMP006_MANUFAC_ID       0x5449
#define TMP006_DEVICE_ID        0x0067

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define FAILURE                 -1
#define SUCCESS                 0
#define RET_IF_ERR(Func)        {int iRetVal = (Func); \
                                 if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
// BMA222 Data Interpretation macros
//*****************************************************************************
#define RESOLUTION_8BIT         ((float)(1.999 / 127))  //+-2g
#define G_VAL                   ((float)9.7798)

#define NO_OF_SAMPLES 		128 //ADC

volatile float g_accXIntervalSum = 0;
volatile float g_accYIntervalSum = 0;
volatile float g_accZIntervalSum = 0;
volatile long long g_accSampleCount = 0;

volatile float g_accTotalAvg;
volatile float g_accXAvg;
volatile float g_accYAvg;
volatile float g_accZAvg;

unsigned char pause = 0;

unsigned long pulAdcSamples[4096];//ADC
//static unsigned char g_ucDryerRunning = 0;


//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern int I2C_IF_Open(unsigned long ulMode);
extern int I2C_IF_Close();
extern int I2C_IF_Write(unsigned char ucDevAddr,
             unsigned char *pucData,
             unsigned char ucLen,
             unsigned char ucStop);
extern int I2C_IF_Read(unsigned char ucDevAddr,
            unsigned char *pucData,
            unsigned char ucLen);
extern int I2C_IF_ReadFrom(unsigned char ucDevAddr,
            unsigned char *pucWrDataBuf,
            unsigned char ucWrLen,
            unsigned char *pucRdDataBuf,
            unsigned char ucRdLen);
extern void GPIO_IF_GetPortNPin(unsigned char ucPin,
        unsigned int *puiGPIOPort,
            unsigned char *pucGPIOPin);
extern void GPIO_IF_GetPortNPin(unsigned char ucPin,
                unsigned int *puiGPIOPort,
                    unsigned char *pucGPIOPin);
extern unsigned char GPIO_IF_Get(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin);
extern void GPIO_IF_LedOff(char ledNum);



extern int BlockRead_(unsigned char ucRegAddr,
          	  	  	  unsigned char *pucBlkData,
					  unsigned char ucBlkDataSz);
extern int GetRegisterValue_(unsigned char ucRegAddr, unsigned char *pucRegValue);
extern int SetRegisterValue_(unsigned char ucRegAddr, unsigned char ucRegValue);
extern int BMA222Close_();
extern int BMA222Read_(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ);
extern int BMA222ReadNew_(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ);




extern int BMA222Open_();
extern void AccSample_();
extern void SetAccAvg_();
extern int GetRegisterValue_temp_(unsigned char ucRegAddr, unsigned short *pusRegValue);
extern int TMP006DrvOpen_();
extern double ComputeTemperature_(double dVobject, double dTAmbient);
extern int TMP006DrvGetTemp_(float *pfCurrTemp);
//extern int BMA222Open_();
//extern unsigned short itoa(short cNum, char *cString);


//*****************************************************************************

// Application specific status/error codes
typedef enum{
 /* Choosing this number to avoid overlap with host-driver's error codes */
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,       
    DEVICE_START_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
    INVALID_HEX_STRING = DEVICE_START_FAILED - 1,
    TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
    TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
    FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
    INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
    FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
    FILE_OPEN_FAILED = FORMAT_NOT_SUPPORTED - 1,
    FILE_WRITE_ERROR = FILE_OPEN_FAILED - 1,
    INVALID_FILE = FILE_WRITE_ERROR - 1,
    SERVER_CONNECTION_FAILED = INVALID_FILE - 1,
    GET_HOST_IP_FAILED = SERVER_CONNECTION_FAILED  - 1,
    
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulDestinationIP; // IP address of destination server
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned char g_buff[MAX_BUFF_SIZE+1];
long bytesReceived = 0; // variable to store the file size
//char buf[100];
char buf[120];
unsigned char g_ucUARTRecvBuffer1[80];
unsigned char ucPinValue;//02/17/2017
unsigned char Lght = 0;
unsigned int uiGPIOPort;//02/17/2017
unsigned char pucGPIOPin;//02/17/2017
//char buf[57];


#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

#define MAX_STRING_LENGTH    80
signed char cPassword[MAX_STRING_LENGTH+1];
signed char cSSID_NAME[MAX_STRING_LENGTH+1];
signed char cCharacter;
int iStringLength = 0;
//unsigned int GETChar(unsigned char *ucBuffer);
//*****************************************************************************
//                          MACROS
//*****************************************************************************
//#define APPLICATION_VERSION  "1.1.1"
//#define APP_NAME             "UART Echo"
#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80
volatile int g_iCounter = 0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
                            " BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
            "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
       switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        	switch( pSock->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                    "failed to transmit all queued packets\n\n",
                           pSock->socketAsyncEvent.SockAsyncData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED : socket %d , reason"
                        "(%d) \n\n",
                        pSock->socketAsyncEvent.SockAsyncData.sd,
                        pSock->socketAsyncEvent.SockTxFailData.status);
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************



//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);


    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return SUCCESS;
}



//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  Status value
//!
//! \return  None
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//
//	char cPassword[MAX_STRING_LENGTH+1];
//	char cSSID_NAME[80];
//
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;



//#ifdef cred
    if(ucPinValue == 1){
    	secParams.Key = (signed char *)cPassword;
    	secParams.KeyLen = strlen((const char *)cPassword);
    	secParams.Type = SECURITY_TYPE;
    }
//#else
    else{
    	secParams.Key = (signed char *)SECURITY_KEY;
    	secParams.KeyLen = strlen(SECURITY_KEY);
    	secParams.Type = SECURITY_TYPE;
    }
//#endif

//#ifdef cred
    if(ucPinValue == 1){
    	lRetVal = sl_WlanConnect((signed char *)cSSID_NAME, strlen((const char *)cSSID_NAME), 0, &secParams, 0);//Entered string - WiFi Name
    	ASSERT_ON_ERROR(lRetVal);
    }
//#else
    else{
    	lRetVal = sl_WlanConnect((signed char *)SSID_NAME, strlen((const char *)SSID_NAME), 0, &secParams, 0);//Saved string - WiFi Name
    	ASSERT_ON_ERROR(lRetVal);
    }
//#endif

    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // wait till connects to an AP
        _SlNonOsMainLoopTask();
    }

    return SUCCESS;

}


//*****************************************************************************
//
//! \brief Flush response body.
//!
//! \param[in]  httpClient - Pointer to HTTP Client instance
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int FlushHTTPResponse(HTTPCli_Handle httpClient)
{
    const char *ids[2] = {
                            HTTPCli_FIELD_NAME_CONNECTION, /* App will get connection header value. all others will skip by lib */
                            NULL
                         };
    char buf[128];
    int id;
    int len = 1;
    bool moreFlag = 0;
    char ** prevRespFilelds = NULL;


    /* Store previosly store array if any */
    prevRespFilelds = HTTPCli_setResponseFields(httpClient, ids);

    /* Read response headers */
    while ((id = HTTPCli_getResponseField(httpClient, buf, sizeof(buf), &moreFlag))
            != HTTPCli_FIELD_ID_END)
    {

        if(id == 0)
        {
            if(!strncmp(buf, "close", sizeof("close")))
            {
                UART_PRINT("Connection terminated by server\n\r");
            }
        }

    }

    /* Restore previosuly store array if any */
    HTTPCli_setResponseFields(httpClient, (const char **)prevRespFilelds);

    while(1)
    {
        /* Read response data/body */
        /* Note:
                moreFlag will be set to 1 by HTTPCli_readResponseBody() call, if more
                data is available Or in other words content length > length of buffer.
                The remaining data will be read in subsequent call to HTTPCli_readResponseBody().
                Please refer HTTP Client Libary API documenation @ref HTTPCli_readResponseBody
                for more information.
        */
        HTTPCli_readResponseBody(httpClient, buf, sizeof(buf) - 1, &moreFlag);
        ASSERT_ON_ERROR(len);

        if ((len - 2) >= 0 && buf[len - 2] == '\r' && buf [len - 1] == '\n'){
            break;
        }

        if(!moreFlag)
        {
            /* There no more data. break the loop. */
            break;
        }
    }
    return 0;
}


//*****************************************************************************
//
//! \brief Handler for parsing JSON data
//!
//! \param[in]  ptr - Pointer to http response body data
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
int ParseJSONData(char *ptr)
{
	long lRetVal = 0;
    int noOfToken;
    jsmn_parser parser;
    jsmntok_t   *tokenList;


    /* Initialize JSON PArser */
    jsmn_init(&parser);

    /* Get number of JSON token in stream as we we dont know how many tokens need to pass */
    noOfToken = jsmn_parse(&parser, (const char *)ptr, strlen((const char *)ptr), NULL, 10);
    if(noOfToken <= 0)
    {
    	UART_PRINT("Failed to initialize JSON parser\n\r");
    	return -1;

    }

    /* Allocate memory to store token */
    tokenList = (jsmntok_t *) malloc(noOfToken*sizeof(jsmntok_t));
    if(tokenList == NULL)
    {
        UART_PRINT("Failed to allocate memory\n\r");
        return -1;
    }

    /* Initialize JSON Parser again */
    jsmn_init(&parser);
    noOfToken = jsmn_parse(&parser, (const char *)ptr, strlen((const char *)ptr), tokenList, noOfToken);
    if(noOfToken < 0)
    {
    	UART_PRINT("Failed to parse JSON tokens\n\r");
    	lRetVal = noOfToken;
    }
    else
    {
    	UART_PRINT("Successfully parsed %ld JSON tokens\n\r", noOfToken);
    }

    free(tokenList);

    return lRetVal;
}

/*!
    \brief This function read respose from server and dump on console
    \param[in]      httpClient - HTTP Client object
    \return         0 on success else -ve
    \note
    \warning
*/
static int readResponse(HTTPCli_Handle httpClient)
{
	long lRetVal = 0;
	int bytesRead = 0;
	int id = 0;
	unsigned long len = 0;
	int json = 0;
	char *dataBuffer=NULL;
	bool moreFlags = 1;
	const char *ids[4] = {
	                        HTTPCli_FIELD_NAME_CONTENT_LENGTH,
			                HTTPCli_FIELD_NAME_CONNECTION,
			                HTTPCli_FIELD_NAME_CONTENT_TYPE,
			                NULL
	                     };

	/* Read HTTP POST request status code. Server status */
	lRetVal = HTTPCli_getResponseStatus(httpClient);
	if(lRetVal > 0)
	{
		switch(lRetVal)
		{
		case 200:
		{
			UART_PRINT("HTTP Status 200");
			/*
                 Set response header fields to filter response headers. All
                  other than set by this call we be skipped by library.
			 */
			HTTPCli_setResponseFields(httpClient, (const char **)ids);
			//UART_PRINT("\n\r ids : %s", ids);

			/* Read filter response header and take appropriate action. */
			/* Note:
                    1. id will be same as index of fileds in filter array setted
                    in previous HTTPCli_setResponseFields() call.
                    2. moreFlags will be set to 1 by HTTPCli_getResponseField(), if  field
                    value could not be completely read. A subsequent call to
                    HTTPCli_getResponseField() will read remaining field value and will
                    return HTTPCli_FIELD_ID_DUMMY. Please refer HTTP Client Libary API
                    documenation @ref HTTPCli_getResponseField for more information.
			 */
			while((id = HTTPCli_getResponseField(httpClient, (char *)g_buff, sizeof(g_buff), &moreFlags))
					!= HTTPCli_FIELD_ID_END)
			{
				//(char *)g_buff - value Field value string.

				switch(id)
				{
				case 0: /* HTTPCli_FIELD_NAME_CONTENT_LENGTH */
				{
					len = strtoul((char *)g_buff, NULL, 0);
					//UART_PRINT("\n\rHTTPCli_FIELD_NAME_CONTENT_LENGTH:");
					UART_PRINT("\n\rLength: %s", g_buff);
				}
				break;
				case 1: /* HTTPCli_FIELD_NAME_CONNECTION */
				{
					//UART_PRINT("\n\rHTTPCli_FIELD_NAME_CONNECTION:");
					UART_PRINT("\n\n\rConnection name: %s", g_buff);
				}
				break;
				case 2: /* HTTPCli_FIELD_NAME_CONTENT_TYPE */
				{

					//UART_PRINT("\n\r HTTPCli_FIELD_NAME_CONTENT_TYPE\n\r");
					UART_PRINT("\n\r Content type : %s", g_buff);
					if(!strncmp((const char *)g_buff, "application/json",
							sizeof("application/json")))
					{
						json = 1;
						UART_PRINT("\n\r g_buff == application/json \n\r");
					}
					else
					{
						/* Note:
                                Developers are advised to use appropriate
                                content handler. In this example all content
                                type other than json are treated as plain text.
						 */
						json = 0;
					}
					//UART_PRINT(HTTPCli_FIELD_NAME_CONTENT_TYPE);
					//UART_PRINT(" : ");
					//UART_PRINT("\n\r application/json\n\r");
				}
				break;
				default:
				{
					UART_PRINT("\n\r Wrong filter id \n\r");
					lRetVal = -1;
					goto end;
				}
				}
			}
			bytesRead = 0;
			if(len > sizeof(g_buff))
			{
				dataBuffer = (char *) malloc(len);
				if(dataBuffer)
				{
					UART_PRINT("\n\r Failed to allocate memory \n\r");
					lRetVal = -1;
					goto end;
				}
			}
			else
			{
				dataBuffer = (char *)g_buff;
			}

			/* Read response data/body */
			/* Note:
                    moreFlag will be set to 1 by HTTPCli_readResponseBody() call, if more
		            data is available Or in other words content length > length of buffer.
		            The remaining data will be read in subsequent call to HTTPCli_readResponseBody().
		            Please refer HTTP Client Libary API documenation @ref HTTPCli_readResponseBody
		            for more information
char *s;
s = strstr(buff, "hassasin");      // search for string "hassasin" in buff
if (s != NULL)                     // if successful then s now points at "hassasin"
{
     printf("Found string at index = %d\n", s - buff);
}                                  // index of "hassasin" in buff can be found by pointer subtraction
else
{
     printf("String not found\n");  // strstr returns NULL if search string not found
}
			 */

			bytesRead = HTTPCli_readResponseBody(httpClient, (char *)dataBuffer, len, &moreFlags);
			UART_PRINT("\n\n\r Received response body, POST method: \n\r %s", dataBuffer);//Print web-site response

			//--------------------------------------------------------------------------------------------------


			if(bytesRead < 0)
			{
				UART_PRINT("\n\r Failed to received response body\n\r");
				lRetVal = bytesRead;
				goto end;
			}
			else if( bytesRead < len || moreFlags)
			{
				UART_PRINT("\n\r Mismatch in content length and received data length\n\r");
				goto end;
			}
			dataBuffer[bytesRead] = '\0';

			if(json)
			{
				/* Parse JSON data */
				lRetVal = ParseJSONData(dataBuffer);
				if(lRetVal < 0)
				{
					goto end;
				}
			}
			else
			{
				/* treating data as a plain text */
			}

		}
		break;

		case 404:
			UART_PRINT("File not found. \r\n");
			/* Handle response body as per requirement.
                  Note:
                    Developers are advised to take appopriate action for HTTP
                    return status code else flush the response body.
                    In this example we are flushing response body in default
                    case for all other than 200 HTTP Status code.
			 */
			break;
		default:
			/* Note:
              Need to flush received buffer explicitly as library will not do
              for next request.Apllication is responsible for reading all the
              data.
			 */
			FlushHTTPResponse(httpClient);
			break;
		}
	}
	else
	{
		UART_PRINT("Failed to receive data from server.\r\n");
		goto end;
	}

	lRetVal = 0;

end:
    if(len > sizeof(g_buff) && (dataBuffer != NULL))
	{
	    free(dataBuffer);
    }
    return lRetVal;
}

//*****************************************************************************
//
//! \brief HTTP POST Demonstration
//!
//! \param[in]  httpClient - Pointer to http client
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int HTTPPostMethod_data(HTTPCli_Handle httpClient)
{
    bool moreFlags = 1;
    bool lastFlag = 1;
    char tmpBuf[4];
    long lRetVal = 0;
    //buf

    //HTTPCli_Field fields[4] = {
    //                            {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
    //                            {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
    //                            {HTTPCli_FIELD_NAME_CONTENT_TYPE, "application/json"},
    //                            {NULL, NULL}
    //                        };

    HTTPCli_Field fields[4] = {
                                    {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                                    {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                                    {HTTPCli_FIELD_NAME_CONTENT_TYPE, "application/x-www-form-urlencoded"},
                                    {NULL, NULL}
                                };

    /* Set request header fields to be send for HTTP request. */
    HTTPCli_setRequestFields(httpClient, fields);//Prints response from the webpage

    /* Send POST method request. */
    /* Here we are setting moreFlags = 1 as there are some more header fields need to send
       other than setted in previous call HTTPCli_setRequestFields() at later stage.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest for more information.
    */
    moreFlags = 1;
    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_POST, POST_REQUEST_URI, moreFlags);//POST & post.php
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP POST request header.\n\r");
        return lRetVal;
    }

    //acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85
    sprintf((char *)tmpBuf, "%d", (sizeof(buf)-1));  // Print Size of POST data body 99
    //sprintf((char *)tmpBuf, "%d", (sizeof(POST_DATA)-1));  // Size of POST data body

    /*
     * Here we are setting lastFlag = 1 as it is last header field.
     * Please refer HTTP Library API documentaion @ref HTTPCli_sendField for more information.
     * Send the length of sent data
    */
    lastFlag = 1;
    lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (const char *)tmpBuf, lastFlag);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP POST request header.\n\r");
        return lRetVal;
    }


    /* Send POST data/body */
    //lRetVal = HTTPCli_sendRequestBody(httpClient, POST_DATA, (sizeof(POST_DATA)-1));
    lRetVal = HTTPCli_sendRequestBody(httpClient, buf, (sizeof(buf)-1)); // POST data body
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP POST request body.\n\r");
        return lRetVal;
    }


    lRetVal = readResponse(httpClient);

    return lRetVal;
}
//*****************************************************************************
//
//! \brief HTTP DELETE Demonstration
//!
//! \param[in]  httpClient - Pointer to http client
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int HTTPDeleteMethod(HTTPCli_Handle httpClient)
{
  
    long lRetVal = 0;
    HTTPCli_Field fields[3] = {
                                {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                                {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                                {NULL, NULL}
                            };
    bool moreFlags;


    /* Set request header fields to be send for HTTP request. */
    HTTPCli_setRequestFields(httpClient, fields);

    /* Send DELETE method request. */
    /* Here we are setting moreFlags = 0 as there are no more header fields need to send
       at later stage. Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest
       for more information.
    */
    moreFlags = 0;
    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_DELETE, DELETE_REQUEST_URI, moreFlags);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP DELETE request header.\n\r");
        return lRetVal;
    }

    lRetVal = readResponse(httpClient);

    return lRetVal;
}

//*****************************************************************************
//
//! \brief HTTP PUT Demonstration
//!
//! \param[in]  httpClient - Pointer to http client
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int HTTPPutMethod(HTTPCli_Handle httpClient)
{
  
    long lRetVal = 0;
    HTTPCli_Field fields[4] = {
                                {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                                {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                                {HTTPCli_FIELD_NAME_CONTENT_TYPE, "text/html"},
                                {NULL, NULL}
                            };
    bool        moreFlags = 1;
    bool        lastFlag = 1;
    char        tmpBuf[4];
    
    
    /* Set request header fields to be send for HTTP request. */
    HTTPCli_setRequestFields(httpClient, fields);

    /* Send PUT method request. */
    /* Here we are setting moreFlags = 1 as there are some more header fields need to send
       other than setted in previous call HTTPCli_setRequestFields() at later stage.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest for more information.
    */
    moreFlags = 1;
    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_PUT, PUT_REQUEST_URI, moreFlags);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP PUT request header.\n\r");
        return lRetVal;
    }

    sprintf((char *)tmpBuf, "%d", (sizeof(PUT_DATA)-1));

    /* Here we are setting lastFlag = 1 as it is last header field.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendField for more information.
    */
    lastFlag = 1;
    lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (char *)tmpBuf, lastFlag);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP PUT request header.\n\r");
        return lRetVal;
    }

    /* Send PUT data/body */
    lRetVal = HTTPCli_sendRequestBody(httpClient, PUT_DATA, (sizeof(PUT_DATA)-1));
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP PUT request body.\n\r");
        return lRetVal;
    }

    lRetVal = readResponse(httpClient);

    return lRetVal;
}

//*****************************************************************************
//
//! \brief HTTP GET Demonstration
//!
//! \param[in]  httpClient - Pointer to http client
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int HTTPGetMethod(HTTPCli_Handle httpClient)
{
  
    long lRetVal = 0;
    /*
     * GET /get.php HTTP/1.1
     * This line specifies an HTTP command, called a method, followed by the address of a
	 * document and the version of the HTTP protocol being used. In this case, the request
     * is using the GET method to ask for the index.html document using HTTP 1.1. After
     * this initial line, the request can contain optional header information that gives the server
     * additional data about the request. For example:
     * User-Agent: Mozilla/5.0 (Windows 2000; U) Opera 6.0 [en]
     *
     */

    //GET /get.php HTTP/1.1
    // optional header information that gives the server additional data about the request.
    //Accept: image/gif, image/jpeg, text/*, */* <---- "*/*" indicating all media types

    HTTPCli_Field fields[4] = {
                                {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                                {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},	//Accept: image/gif, image/jpeg, text/*, */*
                                {HTTPCli_FIELD_NAME_CONTENT_LENGTH, "0"},
                                {NULL, NULL}
                            };
    bool        moreFlags;
    
    
    /* Set request header fields to be send for HTTP request. */
    HTTPCli_setRequestFields(httpClient, fields);

    /* Send GET method request. */
    /* Here we are setting moreFlags = 0 as there are no more header fields need to send
       at later stage. Please refer HTTP Library API documentaion @ HTTPCli_sendRequest
       for more information.
    */
    moreFlags = 0;

    /*
     * Here we are setting moreFlags = 1 as there are some more header fields need to send
     * other than setted in previous call HTTPCli_setRequestFields() at later stage.
     * Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest for more information.
     */
    //moreFlags = 1;
    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, GET_REQUEST_URI, moreFlags);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP GET request.\n\r");
        return lRetVal;
    }


    //##################################################################################################################
    //bool lastFlag = 1;
    //char tmpBuf[4];
    //sprintf((char *)tmpBuf, "%d", (sizeof(POST_DATA)-1));  // POST data body

    /*
     * Here we are setting lastFlag = 1 as it is last header field.
     * Please refer HTTP Library API documentaion @ref HTTPCli_sendField for more information.
    */
    //lastFlag = 1;
    //lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (const char *)tmpBuf, lastFlag);
    //if(lRetVal < 0)
    //{
    //	UART_PRINT("Failed to send HTTP POST request header.\n\r");
    //	return lRetVal;
    //}

     /* Send POST data/body */
     //lRetVal = HTTPCli_sendRequestBody(httpClient, POST_DATA, (sizeof(POST_DATA)-1));
     //if(lRetVal < 0)
     //{
    //	 UART_PRINT("Failed to send HTTP POST request body.\n\r");
    //	 return lRetVal;
     //}

//##################################################################################################################

     lRetVal = readResponse(httpClient);	//Currently returns all the text from get.html with text formatting (prints data to a serial port)



   // lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, GET_REQUEST_URI_JSON, moreFlags);
   //    if(lRetVal < 0)
   //     {
   //         UART_PRINT("Failed to send HTTP GET request.\n\r");
   //         return lRetVal;
   //     }


    //    lRetVal = readResponse(httpClient);


    return lRetVal;
}

/*!
    \brief This function read respose from server and dump on console
    \param[in]      httpClient - HTTP Client object
    \return         0 on success else -ve
    \note
    \warning
*/
static int readPageResponse(HTTPCli_Handle httpClient)//Read data.txt page
{

	long lRetVal = 0;
	int bytesRead = 0;
	int id = 0;
	unsigned long len = 0;
	int json = 0;
	char *dataBuffer=NULL;
	bool moreFlags = 1;
	//char * str1;
	//char * str2;
	const char *ids[4] = {
	                        HTTPCli_FIELD_NAME_CONTENT_LENGTH,
			                HTTPCli_FIELD_NAME_CONNECTION,
			                HTTPCli_FIELD_NAME_CONTENT_TYPE,
			                NULL
	                     };

	/* Read HTTP POST request status code. Server status */
	lRetVal = HTTPCli_getResponseStatus(httpClient);
	if(lRetVal > 0)
	{
		UART_PRINT("Read Page Response\n\r");
		switch(lRetVal)
		{
		case 200:
		{
			UART_PRINT("HTTP Status 200\n\r");

			HTTPCli_setResponseFields(httpClient, (const char **)ids);

			while((id = HTTPCli_getResponseField(httpClient, (char *)g_buff, sizeof(g_buff), &moreFlags))
					!= HTTPCli_FIELD_ID_END)
			{
				//(char *)g_buff - value Field value string.

				switch(id)
				{
				case 0: /* HTTPCli_FIELD_NAME_CONTENT_LENGTH, case 0 server sent number of bytes in the response*/
				{
					len = strtoul((char *)g_buff, NULL, 0);
					//UART_PRINT("\n\rHTTPCli_FIELD_NAME_CONTENT_LENGTH\n\r");
					UART_PRINT("\n\rNumber of characters received (action_page.php): %s", len);// Print number of characters that will be sent by the server
				}
				break;
				case 1: /* HTTPCli_FIELD_NAME_CONNECTION */
				{
					//UART_PRINT("\n\rHTTPCli_FIELD_NAME_CONNECTION\n\r");
					UART_PRINT("\n\n\rConnection name: %s", g_buff);
				}
				break;
				case 2: /* HTTPCli_FIELD_NAME_CONTENT_TYPE. Content of the response */
				{

					//UART_PRINT("\n\r HTTPCli_FIELD_NAME_CONTENT_TYPE\n\r");
					UART_PRINT("\n\r Content type : %s", g_buff);// Print Content of the response
					if(!strncmp((const char *)g_buff, "application/json",
							sizeof("application/json")))
					{
						json = 1;
						UART_PRINT("\n\r g_buff == application/json \n\r");
					}
					else
					{
						json = 0;
					}
				}
				break;
				default:
				{
					UART_PRINT("\n\r Wrong filter id \n\r");
					lRetVal = -1;
					goto end;
				}
				}
			}
			bytesRead = 0;
			if(len > sizeof(g_buff))
			{
				dataBuffer = (char *) malloc(len);
				if(dataBuffer)
				{
					UART_PRINT("\n\r Failed to allocate memory \n\r");
					lRetVal = -1;
					goto end;
				}
			}
			else
			{
				dataBuffer = (char *)g_buff;
			}

			bytesRead = HTTPCli_readResponseBody(httpClient, (char *)dataBuffer, len, &moreFlags);
			UART_PRINT("\n\r ***** Received data from data.txt: %s", dataBuffer); //Print data.txt response:
																				  //roman chak,chakr1@yahoo.com,hello,hello roman,
			//UART_PRINT("\n\n\rPartial page:\n");
			//printf( "%.100s", &dataBuffer[ 7722 ] );
			//char *s;
			char * pch;
			unsigned char i;
			//char *token;
			//const char * strstr ( const char * str1, const char * str2 );
			//		char * strstr (       char * str1, const char * str2 );
			//Returns a pointer to the first occurrence of str2 in str1, or a null pointer if str2 is not part of str1.
			//str1 = strstr(dataBuffer, "****** Data page - data.txt *******");
			//UART_PRINT(str1);
			/*
			 *
			 	 int main()
				{
    				char str[] = "Geeks-for-Geeks";

    				// Returns first token
    				char *token = strtok(str, "-");

    				// Keep printing tokens while one of the
    				// delimiters present in str[].
    				while (token != NULL)
    				{
        				printf("%s\n", token);
        				token = strtok(NULL, "-");
    				}

    				return 0;
}
			 */
			pch = strtok (dataBuffer," ,.-");// Returns the first token. dataBuffer is the input string shown in the above example.
			UART_PRINT ("%s\n", pch);
			const char *r_str = "Roman";
			int ret;
			ret = strncmp((const char *)pch, r_str, 5);// compare the first token to "hello"
			if (ret == 0){
				//GPIO_IF_Set(SH_GPIO_9,uiGPIOPort,pucGPIOPin,1);//Turn ON red LED 08/18/2017
		        //GPIO_IF_GetPortNPin(SH_GPIO_9,&uiGPIOPort,&pucGPIOPin);// Computes port and pin number from the GPIO number
		    	//GPIO_IF_Set(SH_GPIO_9,uiGPIOPort,pucGPIOPin,1);//Turn OFF red LED 08/18/2017
				for (i=0; i<10; i++){
					GPIO_IF_LedOn(MCU_RED_LED_GPIO);
					MAP_UtilsDelay(10000000);//0.625 sec delay
					GPIO_IF_LedOff(MCU_RED_LED_GPIO);
					MAP_UtilsDelay(10000000);//0.625 sec delay
				}
			}
			while (pch != NULL)
			{
				pch = strtok (NULL, " ,.-");
				UART_PRINT ("%s\n", pch);//print sequence received from the server
			}
			UART_PRINT("*******************************************");

			/*
			if (str1 != NULL)                     // if successful then str1 now points at the first character 'R'
			{
				//strncpy ( str2, str1, sizeof(str2) );
				//strncpy ( str2, str1, 13 );//partial copy (only 13 chars):
				UART_PRINT("\n\n\rFound string at index = %d \n", str1);//&s[i] - &dataBuffer[0]
				UART_PRINT(str1);
				UART_PRINT("\n\n\r");
				UART_PRINT(str2);
				UART_PRINT("\n\n\r");
				GPIO_IF_GetPortNPin(SH_GPIO_9,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
				GPIO_IF_Set(SH_GPIO_9,uiGPIOPort,pucGPIOPin,1);//Turn ON red LED 08/18/2017

			}                                  // index of "checked" in buff can be found by pointer subtraction
			else
			{
				UART_PRINT("\n\n\rNo data received from web page\n");  // strstr returns NULL if search string not found
				GPIO_IF_GetPortNPin(SH_GPIO_9,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
				GPIO_IF_Set(SH_GPIO_9,uiGPIOPort,pucGPIOPin,0);//Turn OFF red LED 08/18/2017

			}
*/
			//--------------------------------------------------------------------------------------------------


			if(bytesRead < 0)
			{
				UART_PRINT("\n\r Failed to received response body\n\r");
				lRetVal = bytesRead;
				goto end;
			}
			else if( bytesRead < len || moreFlags)
			{
				UART_PRINT("\n\r Mismatch in content length and received data length\n\r");
				goto end;
			}
			dataBuffer[bytesRead] = '\0';

			if(json)
			{
				/* Parse JSON data */
				lRetVal = ParseJSONData(dataBuffer);
				if(lRetVal < 0)
				{
					goto end;
				}
			}
			else
			{
				/* treating data as a plain text */
			}

		}
		break;

		case 404:
			UART_PRINT("File not found. \r\n");
			/* Handle response body as per requirement.
                  Note:
                    Developers are advised to take appropriate action for HTTP
                    return status code else flush the response body.
                    In this example we are flushing response body in default
                    case for all other than 200 HTTP Status code.
			 */

			break;
		default:
			/* Note:
              Need to flush received buffer explicitly as library will not do
              for next request.Apllication is responsible for reading all the
              data.
			 */
			FlushHTTPResponse(httpClient);
			break;
		}
	}
	else
	{
		UART_PRINT("Failed to receive data from server.\r\n");
		goto end;
	}

	lRetVal = 0;

end:
    if(len > sizeof(g_buff) && (dataBuffer != NULL))
	{
	    free(dataBuffer);
    }
    return lRetVal;
}

static int HTTPGetPageMethod(HTTPCli_Handle httpClient)//Read data.txt page
{

    long lRetVal = 0;

    HTTPCli_Field fields[4] = {
                                {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                                {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},	//Accept: image/gif, image/jpeg, text/*, */*
                                {HTTPCli_FIELD_NAME_CONTENT_LENGTH, "0"},
                                {NULL, NULL}
                            };
    bool        moreFlags;

    /* Set request header fields to be send for HTTP request. GET_REQUEST_URI*/
    HTTPCli_setRequestFields(httpClient, fields);

    moreFlags = 0;

    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, data_txt, moreFlags);//data_txt = data.txt
    //lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, GET_action_page, moreFlags);
    //lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, GET_REQUEST_URI, moreFlags);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP GET request.\n\r");
        return lRetVal;
    }

     lRetVal = readPageResponse(httpClient);	//Currently returns all the text from data.html with text formatting (prints data to a serial port)
     //lRetVal = readResponse(httpClient);
    return lRetVal;
}

//*****************************************************************************
//
//! Function to connect to AP
//!
//! \param  none
//!
//! \return Error-code or SUCCESS
//!
//*****************************************************************************
static long ConnectToAP()
{
    long lRetVal = -1;
    
    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its desired state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
        {
            UART_PRINT("Failed to configure the device in its default state, "
                            "Error-code: %d\n\r", DEVICE_NOT_IN_STATION_MODE);
        }

        return -1;
    }

    UART_PRINT("Device is configured in default state \n\r");

    //
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal)
    {
        ASSERT_ON_ERROR(DEVICE_START_FAILED);
    }

    UART_PRINT("Device started as STATION \n\r");

    // Connecting to WLAN AP - Set with static parameters defined at the top
    // After this call we will be connected and have IP address
    lRetVal = WlanConnect();

    UART_PRINT("Connected to the AP: %s\r\n", SSID_NAME);
    return 0;
}

//*****************************************************************************
//
//! Function to connect to HTTP server
//!
//! \param  httpClient - Pointer to HTTP Client instance
//!
//! \return Error-code or SUCCESS
//!
//*****************************************************************************
static int ConnectToHTTPServer(HTTPCli_Handle httpClient)
{
    long lRetVal = -1;
    struct sockaddr_in addr;
  

#ifdef USE_PROXY
    struct sockaddr_in paddr;
    paddr.sin_family = AF_INET;
    paddr.sin_port = htons(PROXY_PORT);
    paddr.sin_addr.s_addr = sl_Htonl(PROXY_IP);
    HTTPCli_setProxy((struct sockaddr *)&paddr);
#endif
    
    /* Resolve HOST NAME/IP */
    lRetVal = sl_NetAppDnsGetHostByName((signed char *)HOST_NAME,
                                          strlen((const char *)HOST_NAME),
                                          &g_ulDestinationIP,SL_AF_INET);
    if(lRetVal < 0)
    {
        ASSERT_ON_ERROR(GET_HOST_IP_FAILED);
    }

    /* Set up the input parameters for HTTP Connection */
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HOST_PORT);
    addr.sin_addr.s_addr = sl_Htonl(g_ulDestinationIP);

    /* Testing HTTPCli open call: handle, address params only */
    HTTPCli_construct(httpClient);
    lRetVal = HTTPCli_connect(httpClient, (struct sockaddr *)&addr, 0, NULL);
    if (lRetVal < 0)
    {
        UART_PRINT("Connection to server failed. error(%d)\n\r", lRetVal);
        ASSERT_ON_ERROR(SERVER_CONNECTION_FAILED);
    }    
    else
    {
    	UART_PRINT("Connection to server created successfully\r\n");
    	UART_PRINT(HOST_NAME);
    }

    return 0;
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t      CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

// Temperature
//###################################################################################

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************
//static int GetRegisterValue_temp(unsigned char ucRegAddr,
//                            unsigned short *pusRegValue);
//static double ComputeTemperature(double dVobject, double dTAmbient);

extern int GetRegisterValue_temp_(unsigned char ucRegAddr,
                            unsigned short *pusRegValue);
extern double ComputeTemperature_(double dVobject, double dTAmbient);

//****************************************************************************
//
//! Compute the temperature value from the sensor voltage and die temp.
//!
//! \param dVobject is the sensor voltage value
//! \param dTAmbient is the local die temperature
//!
//! This function
//!    1. Computes the temperature from the VObject and TAmbient values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
double ComputeTemperature(double dVobject, double dTAmbient)
{
    //
    // This algo is obtained from
    // http://processors.wiki.ti.com/index.php/SensorTag_User_Guide
    // #IR_Temperature_Sensor
    //
    double Tdie2 = dTAmbient + 273.15;
    const double S0 = 6.4E-14;            // Calibration factor
    const double a1 = 1.75E-3;
    const double a2 = -1.678E-5;
    const double b0 = -2.94E-5;
    const double b1 = -5.7E-7;
    const double b2 = 4.63E-9;
    const double c2 = 13.4;
    const double Tref = 298.15;
    double S = S0*(1+a1*(Tdie2 - Tref)+a2*pow((Tdie2 - Tref),2));
    double Vos = b0 + b1*(Tdie2 - Tref) + b2*pow((Tdie2 - Tref),2);
    double fObj = (dVobject - Vos) + c2*pow((dVobject - Vos),2);
    double tObj = pow(pow(Tdie2,4) + (fObj/S),.25);
    tObj = (tObj - 273.15);
    return tObj;
}

//######################################################################################



int main()
{
    long lRetVal = -1;
    //unsigned int uiGPIOPort;//02/17/2017
    //unsigned char pucGPIOPin;//02/17/2017
    unsigned char delay_cntr;
    unsigned int  uiChannel;
    unsigned int  uiIndex=0;
    unsigned long ulSample;
    unsigned long ADCsum;
    float  smpl_nbbr = 0;


    float sensorTemp;
    int cx;
    const char *strPtr;
    const char *strPtr1;
    HTTPCli_Struct httpClient;

    //
    // Board Initialization
    //
    BoardInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    //************************************************************************

    //
    // Display banner
    //
    DisplayBanner(APP_NAME);

   //Read GPIO3: pin58 - Light Sensor
   GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
   Lght = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);


   //Read GPIO22: SW2
   GPIO_IF_GetPortNPin(SH_GPIO_22,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
   ucPinValue = GPIO_IF_Get(SH_GPIO_22,uiGPIOPort,pucGPIOPin);	// Read pin status of GPIO22

   GPIO_IF_GetPortNPin(SH_GPIO_9,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
   GPIO_IF_Set(SH_GPIO_9,uiGPIOPort,pucGPIOPin,0);//Turn OFF red LED 08/18/2017
    //#################GET PASSWORD FROM CONSOLE############################
#ifdef cred
    if(ucPinValue == 1){//If SW2 is pressed
    iStringLength = 0;

    UART_PRINT("Enter password:\n\r");
    cCharacter = UartGetChar(); // Get the first character
    while(cCharacter != '\r' && cCharacter != '\n' && (iStringLength <= MAX_STRING_LENGTH -1)){
    	g_iCounter++;
    	if(cCharacter == '\r' || cCharacter == '\n' || (iStringLength >= MAX_STRING_LENGTH -1))
    	{
    		if(iStringLength >= MAX_STRING_LENGTH - 1)
    		{
    			UartPutChar(cCharacter);
    			cPassword[iStringLength] = cCharacter; //password
    			iStringLength++;
    		}

    		iStringLength++;
    		cPassword[iStringLength] = '\0';

    		iStringLength = 0;
    		//
    		// Echoes the input string
    		//
    		Report("\n\r Password: %s", cPassword);
    		UART_PRINT("\n\r");
    	}
    	else
    	{
    		UartPutChar(cCharacter);
    		cPassword[iStringLength] = cCharacter;
    		iStringLength++;
    	}
    	cCharacter = UartGetChar();
    }

    //#####################GET SSID FROM CONSOLE############################

    iStringLength = 0;
    UART_PRINT("Enter SSID name:\n\r");
    cCharacter = UartGetChar();
    while(cCharacter != '\r' && cCharacter != '\n' && (iStringLength <= MAX_STRING_LENGTH -1)){

    	g_iCounter++;
    	if(cCharacter == '\r' || cCharacter == '\n' || (iStringLength >= MAX_STRING_LENGTH -1))
    	{
    		if(iStringLength >= MAX_STRING_LENGTH - 1)
    		{
    			UartPutChar(cCharacter);
    			cSSID_NAME[iStringLength] = cCharacter; //password
    			iStringLength++;
    		}

    		iStringLength++;
    		cSSID_NAME[iStringLength] = '\0';
    		iStringLength = 0;
    		//
    		// Echoes the input string
    		//
    		Report("\n\r SSID: %s", cSSID_NAME);
    		UART_PRINT("\n\r");
    	}
    	else
    	{
    		UartPutChar(cCharacter);
    		cSSID_NAME[iStringLength] = cCharacter;
    		iStringLength++;
    	}
    	cCharacter = UartGetChar();
    }
    }
    else{
    	//No case
    }
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    InitializeAppVariables();

    lRetVal = ConnectToAP();
    if(lRetVal < 0)
    {
    	LOOP_FOREVER();
    }

    lRetVal = ConnectToHTTPServer(&httpClient);
    if(lRetVal < 0)
    {
        LOOP_FOREVER();
    }
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//---------------------------------------------------------------------------------------------
    //
    // I2C Init	I2C Init I2C Init I2C Init I2C Init I2C Init I2C Init
    //
    lRetVal = I2C_IF_Open(I2C_MASTER_MODE_FST);
    if(lRetVal < 0)
    {
    	ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //Init Temprature Sensor Init Temprature Sensor Init Temprature Sensor
    lRetVal = TMP006DrvOpen_();
    if(lRetVal < 0)
    {
    	ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //Init Accelerometer Sensor Init Accelerometer Sensor Init Accelerometer Sensor
    lRetVal = BMA222Open_();
    if(lRetVal < 0)
    {
    	ERR_PRINT(lRetVal);
    	LOOP_FOREVER();
    }
//------------------------------------------------------------------------------------------------
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    UART_PRINT("\n\r");
    UART_PRINT("HTTP Delete Begin:\n\r");
    lRetVal = HTTPDeleteMethod(&httpClient);

    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Delete failed.\n\r");
    }
    UART_PRINT("HTTP Delete End.\n\r");

    UART_PRINT("\n\r");
    UART_PRINT("HTTP Put Begin:\n\r");
    lRetVal = HTTPPutMethod(&httpClient);
    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Put failed.\n\r");
    }
    UART_PRINT("HTTP Put End.\n\r");

    UART_PRINT("\n\r");
    UART_PRINT("HTTP Get Method Begin:\n\r");
    lRetVal = HTTPGetMethod(&httpClient);
    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Post Get failed.\n\r");
    }
    UART_PRINT("\n\rHTTP Get End.\n\r");
    UART_PRINT("\n\r");
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    AccSample_(); // Just do a single reading for now. TODO: Make Async.
    SetAccAvg_(); // g_accXAvg, g_accYAvg, g_accZAvg, g_accTotalAvg

    TMP006DrvGetTemp_(&sensorTemp);

    //Read GPIO3: pin58 - Light Sensor
    GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
    Lght = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);

//ADC ####ADC####ADC####ADC####ADC####ADC####ADC##############################################################
    //
    // Pinmux for the selected ADC input pin
    //
    MAP_PinTypeADC(PIN_60,PIN_MODE_255);
    uiChannel = ADC_CH_3;
    //
    // Configure ADC timer which is used to timestamp the ADC data samples
    //
    MAP_ADCTimerConfig(ADC_BASE,2^17);

    //
    // Enable ADC timer which is used to timestamp the ADC data samples
    //
    MAP_ADCTimerEnable(ADC_BASE);

    //
    // Enable ADC module
    //
    MAP_ADCEnable(ADC_BASE);

    //
    // Enable ADC channel
    //
    MAP_ADCChannelEnable(ADC_BASE, uiChannel);


    //"acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85"//55 characters
    cx = snprintf(buf, 119, "acc=%.0f & accX=%.0f & accY=%.0f & accZ=%.0f & sensortemp=%.2f & SN=%.0f",
    		g_accTotalAvg,
    		g_accXAvg,
    		g_accYAvg,
    		g_accZAvg,
    		sensorTemp,
			smpl_nbbr);//cx is indice of the last buf[cx]

    if (cx>=0 && cx<119)	{// check returned value
    	if (Lght == 0){
        	strPtr1 = "& Light=Light is  ON";//74
        }
        else{
        	strPtr1 = "& Light=Light is OFF";
        }

        strcpy ( &buf[(cx - 1)], strPtr1 );
        strPtr = " & loc=Los Angeles \0";	// Your location. (95)
        strcpy ( &buf[(76)], strPtr );//"acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85 & loc=Los Angeles \0"
    }

    UART_PRINT(buf);//Print the above buffer
    UART_PRINT("\n\r");
    //Create Web page and post data #################################################################################
    lRetVal = HTTPPostMethod_data(&httpClient);//Make the web page and write data
    UART_PRINT("\n\r");
    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Post with Temperature and Accelerometer Data failed.\n\r");
    }//Web page is created and data is transferred


    //################################################################################################################


    //char *cString;
    while(1){
/*
    	//#define SEC_TO_LOOP(x)        ((80000000/5)*x)
    	for (delay_cntr = 0; delay_cntr < 5; delay_cntr++){
    		MAP_UtilsDelay(40000000);//2.5 sec delay
    	}
*/
    	//GPIO_IF_GetPortNPin(SH_GPIO_9,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
    	//GPIO_IF_Set(SH_GPIO_9,uiGPIOPort,pucGPIOPin,0);//Turn OFF red LED 08/18/2017
    	//GPIO_IF_GetPortNPin(10,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
    	//GPIO_IF_Set(10,uiGPIOPort,pucGPIOPin,0);//Turn OFF orange LED 08/18/2017
    	//GPIO_IF_GetPortNPin(11,&uiGPIOPort,&pucGPIOPin);	// Computes port and pin number from the GPIO number
    	//GPIO_IF_Set(11,uiGPIOPort,pucGPIOPin,0);//Turn OFF green LED 08/18/2017
    	//MAP_GPIOPinWrite(GPIOA3_BASE,0x10,0x10);
    	GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    	//GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO); //Set as I2C
    	//GPIO_IF_LedOff(MCU_GREEN_LED_GPIO); //Set as I2C
    	//MAP_GPIOPinWrite(GPIOA3_BASE,0x10,0x10);

    	//#########################################################################################
    	//Take readings from sensors
    	AccSample_(); // Just do a single reading for now. TODO: Make Async.
    	SetAccAvg_(); // g_accXAvg, g_accYAvg, g_accZAvg, g_accTotalAvg
    	TMP006DrvGetTemp_(&sensorTemp);
    	//Read GPIO3: pin58 - Light Sensor
    	GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
    	Lght = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);
    	//Add sample number
    	smpl_nbbr ++;

    	//itoa(smpl_nbbr, cString);//itoa(short cNum, char *cString);
    	//#########################################################################################

    	//"acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85"//55 characters
    	cx = snprintf(buf, 119, "acc=%.0f & accX=%.0f & accY=%.0f & accZ=%.0f & sensortemp=%.2f & SN=%.0f",
    	    	 g_accTotalAvg,
    	    	 g_accXAvg,
    	    	 g_accYAvg,
    	    	 g_accZAvg,
    	    	 sensorTemp,
				 smpl_nbbr);//cx is indice of the last buf[cx]. buff has all the data to be transferred
    	/*
    	 * Send:
    	 * acc=XX
    	 * accX=XX
    	 * accY=XX
    	 * accZ=XX
    	 * sensortemp=XX.XX
    	 */
    	if (cx>=0 && cx<119)	{// check returned value, the last array indice

    		if (Lght == 0){
    			strPtr1 = "& Light=Light is  ON";//74
    		}
    		else{
    			strPtr1 = "& Light=Light is OFF";
    		}

    		strcpy ( &buf[(cx - 1)], strPtr1 );//Copy the above string to the send buffer

    		strPtr = " & loc=Los Angeles \0";	// Your location. (95)

    		strcpy ( &buf[(86)], strPtr );//"acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85 & loc=Los Angeles \0"
    	}


    	UART_PRINT(buf);//Print the above buffer
    	UART_PRINT("\n\r");
    	//Create Web page and Post data ########################################################################################
    	lRetVal = HTTPPostMethod_data(&httpClient);

    	if(lRetVal < 0)
    	{
    		MAP_PRCMMCUReset(1);
    	}
//********************************************************** If Got Disconnected - Reconnect ***********************************
/*
    	if(lRetVal < 0){//If got disconnected

    		UART_PRINT("HTTP Post data to data.html page failed.\n\r");
    		UART_PRINT("Reconnecting...\n\r");

    	    InitializeAppVariables();

    	    lRetVal = ConnectToAP();
    	    if(lRetVal < 0)
    	    {
    	    	LOOP_FOREVER();
    	    }

    	    lRetVal = ConnectToHTTPServer(&httpClient);
    	    if(lRetVal < 0)
    	    {
    	        LOOP_FOREVER();
    	    }

    	}
*/
//***************************************************************************************************************************
    	lRetVal = HTTPGetPageMethod(&httpClient);//Read data from the action_page.php

    	if(lRetVal < 0)
    	{
    		MAP_PRCMMCUReset(1);
    	}

//*******************************************************************************************************************
//ADC  ##########################################################################################################
    	while(uiIndex < NO_OF_SAMPLES + 4)//Collect samples
    	{
    		if(MAP_ADCFIFOLvlGet(ADC_BASE, uiChannel))//If a sample is ready
    		{
    			ulSample = MAP_ADCFIFORead(ADC_BASE, uiChannel);//Read the sample
    			pulAdcSamples[uiIndex++] = ulSample;//Load the sample into an array
    		}

    	}

    	//MAP_ADCChannelDisable(ADC_BASE, uiChannel);//

    	uiIndex = 0;
    	ADCsum = 0;
    	while(uiIndex < NO_OF_SAMPLES + 4)//sum of all samples
    	{
    		//UART_PRINT("\n\rVoltage is %f\n\r",(((float)((pulAdcSamples[4+uiIndex] >> 2 ) & 0x0FFF))*1.4)/4096);
    		ADCsum = ADCsum + pulAdcSamples[uiIndex];
    		uiIndex++;
    	}
    	ADCsum = ADCsum / (NO_OF_SAMPLES + 4);//ADC average value

    	UART_PRINT("\n\rVoltage is %f\n\r",((ADCsum >> 2 ) & 0x0FFF)*1.4/4096);
    	//UART_PRINT("\n\rVoltage is %f\n\r",((pulAdcSamples[4] >> 2 ) & 0x0FFF)*1.4/4096);
    	UART_PRINT("\n\r");


    }

    // Stop the CC3200 device

    //LOOP_FOREVER();
}
