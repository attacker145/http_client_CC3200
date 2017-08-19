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

// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>

// JSON Parser
#include "jsmn.h"

#include <math.h>

#define cred

#define APPLICATION_VERSION "1.1.1"
#define APP_NAME            "HTTP Client"

//#define POST_REQUEST_URI 	"/post"
//#define POST_REQUEST_URI 	"/post.html"
#define POST_REQUEST_URI 	"/post.php"

							//{ "key": "value",\n\ "key": "value" \n\  }
//#define POST_DATA           "{\n\"name\":\"xyz\",\n\"address\":\n{\n\"plot#\":12,\n\"street\":\"abc\",\n\"city\":\"ijk\"\n},\n\"age\":30\n}"
#define POST_DATA "say=Hi&to=Mom"


#define DELETE_REQUEST_URI 	"/delete"


#define PUT_REQUEST_URI 	"/put"
#define PUT_DATA            "PUT request."

#define GET_REQUEST_URI_JSON 	"/get"

//#define GET_REQUEST_URI 	"/get.html"							//*****************************************************
#define GET_REQUEST_URI 	"/get.php?id=goodbye&mode=run"		//the text that follows the ? is the query string
//#define GET_REQUEST_URI 	"/get.php"

//#define HOST_NAME       	"httpbin.org" //"<host name>"
#define HOST_NAME       	"cnktechlabs.com" //"<host name>" 	********************************************************
#define HOST_PORT           80

#define PROXY_IP       	    <proxy_ip>
#define PROXY_PORT          <proxy_port>

#define READ_SIZE           1450
#define MAX_BUFF_SIZE       1460


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

volatile static float g_accXIntervalSum = 0;
volatile static float g_accYIntervalSum = 0;
volatile static float g_accZIntervalSum = 0;
volatile static long long g_accSampleCount = 0;

volatile static float g_accTotalAvg;
volatile static float g_accXAvg;
volatile static float g_accYAvg;
volatile static float g_accZAvg;
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
char buf[100];
unsigned char g_ucUARTRecvBuffer1[80];
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



#ifdef cred
    secParams.Key = (signed char *)cPassword;
    secParams.KeyLen = strlen(cPassword);
    secParams.Type = SECURITY_TYPE;
#else
    secParams.Key = (signed char *)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;
#endif

#ifdef cred
    lRetVal = sl_WlanConnect((signed char *)cSSID_NAME, strlen((const char *)cSSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);
#else
    lRetVal = sl_WlanConnect((signed char *)SSID_NAME, strlen((const char *)SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);
#endif

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

	/* Read HTTP POST request status code */
	lRetVal = HTTPCli_getResponseStatus(httpClient);
	if(lRetVal > 0)
	{
		switch(lRetVal)
		{
		case 200:
		{
			UART_PRINT("HTTP Status 200\n\r");
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
					UART_PRINT("\n\rHTTPCli_FIELD_NAME_CONTENT_LENGTH\n\r");
					UART_PRINT("\n\rLength: %s", g_buff);
				}
				break;
				case 1: /* HTTPCli_FIELD_NAME_CONNECTION */
				{
					UART_PRINT("\n\rHTTPCli_FIELD_NAME_CONNECTION\n\r");
					UART_PRINT("\n\n\rConnection name: %s", g_buff);
				}
				break;
				case 2: /* HTTPCli_FIELD_NAME_CONTENT_TYPE */
				{

					UART_PRINT("\n\r HTTPCli_FIELD_NAME_CONTENT_TYPE\n\r");
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

			 */
			bytesRead = HTTPCli_readResponseBody(httpClient, (char *)dataBuffer, len, &moreFlags);
			UART_PRINT("\n\n\r Received response body: \n\r %s", dataBuffer);//Print web-site response

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
static int HTTPPostMethod(HTTPCli_Handle httpClient)
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
    HTTPCli_setRequestFields(httpClient, fields);

    /* Send POST method request. */
    /* Here we are setting moreFlags = 1 as there are some more header fields need to send
       other than setted in previous call HTTPCli_setRequestFields() at later stage.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest for more information.
    */
    moreFlags = 1;
    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_POST, POST_REQUEST_URI, moreFlags);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP POST request header.\n\r");
        return lRetVal;
    }

    sprintf((char *)tmpBuf, "%d", (sizeof(POST_DATA)-1));  // POST data body

    /* Here we are setting lastFlag = 1 as it is last header field.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendField for more information.
    */
    lastFlag = 1;
    lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (const char *)tmpBuf, lastFlag);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP POST request header.\n\r");
        return lRetVal;
    }


    /* Send POST data/body */
    lRetVal = HTTPCli_sendRequestBody(httpClient, POST_DATA, (sizeof(POST_DATA)-1));
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
    HTTPCli_setRequestFields(httpClient, fields);

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
    sprintf((char *)tmpBuf, "%d", (sizeof(buf)-1));  // Size of POST data body 99
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

//****************************************************************************
//
//! Returns the value in the specified register
//!
//! \param ucRegAddr is the offset register address
//! \param pucRegValue is the pointer to the register value store
//!
//! This function
//!    1. Returns the value in the specified register
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
GetRegisterValue(unsigned char ucRegAddr, unsigned char *pucRegValue)
{
    //
    // Invoke the readfrom  API to get the required byte
    //
    if(I2C_IF_ReadFrom(BMA222_DEV_ADDR, &ucRegAddr, 1,
                   pucRegValue, 1) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Sets the value in the specified register
//!
//! \param ucRegAddr is the offset register address
//! \param ucRegValue is the register value to be set
//!
//! This function
//!    1. Returns the value in the specified register
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
SetRegisterValue(unsigned char ucRegAddr, unsigned char ucRegValue)
{
    unsigned char ucData[2];
    //
    // Select the register to be written followed by the value.
    //
    ucData[0] = ucRegAddr;
    ucData[1] = ucRegValue;
    //
    // Initiate the I2C write
    //
    if(I2C_IF_Write(BMA222_DEV_ADDR,ucData,2,1) == 0)	// DevAddr Received by I2C:
    {
        return SUCCESS;
    }
    else
    {
        DBG_PRINT("I2C write failed\n\r");
    }

    return FAILURE;
}

//****************************************************************************
//
//! Reads a block of continuous data
//!
//! \param ucRegAddr is the start offset register address
//! \param pucBlkData is the pointer to the data value store
//! \param ucBlkDataSz is the size of data to be read
//!
//! This function
//!    1. Returns the data values in the specified store
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BlockRead(unsigned char ucRegAddr,
          unsigned char *pucBlkData,
          unsigned char ucBlkDataSz)
{
    //
    // Invoke the readfrom I2C API to get the required bytes
    //
    if(I2C_IF_ReadFrom(BMA222_DEV_ADDR, &ucRegAddr, 1,
                   pucBlkData, ucBlkDataSz) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Initialize the BMA222 accelerometer device with defaults
//!
//! \param None
//!
//! This function
//!    1. Reads the CHIP ID.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BMA222Open()
{
    unsigned char ucRegVal;
    //
    // Read the CHIP ID NUM
    //
    RET_IF_ERR(GetRegisterValue(BMA222_CHID_ID_NUM, &ucRegVal));
    DBG_PRINT("CHIP ID: 0x%x\n\r", ucRegVal);

    return SUCCESS;
}

//****************************************************************************
//
//! Place the BMA222 accelerometer device to standby
//!
//! \param None
//!
//! This function
//!    1. Sets the device to standby mode.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BMA222Close()
{
    return SUCCESS;
}

//****************************************************************************
//
//! Get the accelerometer data readings
//!
//! \param pfAccX pointer to the AccX store
//! \param pfAccY pointer to the AccY store
//! \param pfAccZ pointer to the AccZ store
//!
//! This function
//!    1. Reads the data registers over I2C.
//!    2. Applies the range conversion to the raw values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BMA222Read(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ)
{
    char cAccX = 0;
    char cAccY = 0;
    char cAccZ = 0;
    //
    // Read the acclerometer output registers LSB and MSB
    //
    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_X, (unsigned char *)&cAccX,
                     sizeof(cAccX)));

    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_Y, (unsigned char *)&cAccY,
             sizeof(cAccY)));

    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_Z, (unsigned char *)&cAccZ,
             sizeof(cAccZ)));

    *pcAccX = cAccX;
    *pcAccY = cAccY;
    *pcAccZ = cAccZ;

    return SUCCESS;
}

//****************************************************************************
//
//! Get the raw accelerometer data register readings
//!
//! \param psAccX pointer to the raw AccX store
//! \param psAccY pointer to the raw AccY store
//! \param psAccZ pointer to the raw AccZ store
//!
//! This function
//!    1. Reads the data registers over I2C.
//!    2. Returns the accelerometer readings
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BMA222ReadNew(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ)
{
    char cAccX[6];

    //
    // Read the acclerometer output registers LSB and MSB
    //
    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_X_NEW, (unsigned char *)cAccX,6));

     //
    // Check whether new Sensor Data is available
    //
    if((cAccX[0] & 0x1) && (cAccX[2] & 0x1) && (cAccX[4] & 0x1))
    {
        *pcAccX = cAccX[1];
        *pcAccY = cAccX[3];
        *pcAccZ = cAccX[5];
        return SUCCESS;
    }

    //New Sensor Data Not Available
    return FAILURE;

}

//*****************************************************************************
//
//! AccSample
//!
//!    @brief  Read Accelerometer Data from Sensor to Globals
//!
//!
//!     @return none
//!
//!
//
//*****************************************************************************
void AccSample()
{
    signed char accX,accY,accZ;
    int iRet = -1;
    //unsigned long critKey;

    //critKey = osi_EnterCritical();

    iRet = BMA222ReadNew(&accX, &accY, &accZ);
    if(iRet)
    {
        //In case of error/ No New Data return
        return;
    }


    g_accXIntervalSum += accX;
    g_accYIntervalSum += accY;
    g_accZIntervalSum += accZ;

    g_accSampleCount++;
    //osi_ExitCritical(critKey);
}

//*****************************************************************************
//
//! ReadAccSensor
//!
//!    @brief  Calculate Averages of Accelerometer Globals
//!
//!
//!     @return none
//!
//!
//
//*****************************************************************************
void SetAccAvg()
{
	//unsigned long critKey;

    //critKey = osi_EnterCritical();
    g_accXAvg = g_accXIntervalSum / g_accSampleCount;
    g_accYAvg = g_accYIntervalSum / g_accSampleCount;
    g_accZAvg = g_accZIntervalSum / g_accSampleCount;
    g_accTotalAvg = (g_accZIntervalSum + g_accYIntervalSum + g_accXIntervalSum ) /
		(g_accSampleCount * 3);

    g_accXIntervalSum = 0;
    g_accYIntervalSum = 0;
    g_accZIntervalSum = 0;
    g_accSampleCount = 0;
    //osi_ExitCritical(critKey);

}


















// Temperature
//###################################################################################

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************
static int GetRegisterValue_temp(unsigned char ucRegAddr,
                            unsigned short *pusRegValue);
static double ComputeTemperature(double dVobject, double dTAmbient);


//****************************************************************************
//
//! Returns the value in the specified register
//!
//! \param ucRegAddr is the offset register address
//! \param pusRegValue is the pointer to the register value store
//!
//! This function
//!    1. Returns the value in the specified register
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
GetRegisterValue_temp(unsigned char ucRegAddr, unsigned short *pusRegValue)
{
    unsigned char ucRegData[2];
    //
    // Invoke the readfrom I2C API to get the required byte
    //
    if(I2C_IF_ReadFrom(TMP006_DEV_ADDR, &ucRegAddr, 1,
                   &ucRegData[0], 2) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return FAILURE;
    }

    *pusRegValue = (unsigned short)(ucRegData[0] << 8) | ucRegData[1];

    return SUCCESS;
}

//****************************************************************************
//
//! Initialize the temperature sensor
//!
//! \param None
//!
//! This function
//!    1. Get the device manufacturer and version
//!    2. Add any initialization here
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
TMP006DrvOpen()
{
    unsigned short usManufacID, usDevID, usConfigReg;

    //
    // Get the manufacturer ID
    //
    RET_IF_ERR(GetRegisterValue_temp(TMP006_MANUFAC_ID_REG_ADDR, &usManufacID));
    DBG_PRINT("Manufacturer ID: 0x%x\n\r", usManufacID);
    if(usManufacID != TMP006_MANUFAC_ID)
    {
        DBG_PRINT("Error in Manufacturer ID\n\r");
        return FAILURE;
    }

    //
    // Get the device ID
    //
    RET_IF_ERR(GetRegisterValue_temp(TMP006_DEVICE_ID_REG_ADDR, &usDevID));
    DBG_PRINT("Device ID: 0x%x\n\r", usDevID);
    if(usDevID != TMP006_DEVICE_ID)
    {
        DBG_PRINT("Error in Device ID\n");
        return FAILURE;
    }

    //
    // Get the configuration register value
    //
    RET_IF_ERR(GetRegisterValue_temp(TMP006_CONFIG_REG_ADDR, &usConfigReg));
    DBG_PRINT("Configuration register value: 0x%x\n\r", usConfigReg);

    return SUCCESS;
}
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

//****************************************************************************
//
//! Get the temperature value
//!
//! \param pfCurrTemp is the pointer to the temperature value store
//!
//! This function
//!    1. Get the sensor voltage reg and ambient temp reg values
//!    2. Compute the temperature from the read values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
TMP006DrvGetTemp(float *pfCurrTemp)
{
    unsigned short usVObjectRaw, usTAmbientRaw;
    double dVObject, dTAmbient;
    //
    // Get the sensor voltage register value
    //
    RET_IF_ERR(GetRegisterValue_temp(TMP006_VOBJECT_REG_ADDR, &usVObjectRaw));
    //
    // Get the ambient temperature register value
    //
    RET_IF_ERR(GetRegisterValue_temp(TMP006_TAMBIENT_REG_ADDR, &usTAmbientRaw));
    //
    // Apply the format conversion
    //
    dVObject = ((short)usVObjectRaw) * 156.25e-9;
    dTAmbient = ((short)usTAmbientRaw) / 128;

    *pfCurrTemp = ComputeTemperature(dVObject, dTAmbient);

    //
    // Convert to Farenheit
    //
    //*pfCurrTemp = ((*pfCurrTemp * 9) / 5) + 32;

    return SUCCESS;
}
//######################################################################################



int main()
{
    long lRetVal = -1;

    HTTPCli_Struct httpClient;
    //char c = 0;


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


    //c = MAP_UARTCharGetNonBlocking(CONSOLE);        // Get a single character
    //if (c != 0){
    	//UART_PRINT("\n\r\rExecuting UptimeTask Enter a string and press enter\n\r\r");
    	//GETChar(&passwrd[0]);
    //}


    //#################GET PASSWORD FROM CONSOLE############################
#ifdef cred

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
#endif


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
    lRetVal = TMP006DrvOpen();
    if(lRetVal < 0)
    {
    	ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //Init Accelerometer Sensor Init Accelerometer Sensor Init Accelerometer Sensor
    lRetVal = BMA222Open();
    if(lRetVal < 0)
    {
    	ERR_PRINT(lRetVal);
    	LOOP_FOREVER();
    }

    float sensorTemp;

    AccSample(); // Just do a single reading for now. TODO: Make Async.
    SetAccAvg(); // g_accXAvg, g_accYAvg, g_accZAvg, g_accTotalAvg


    TMP006DrvGetTemp(&sensorTemp);
    //int snprintf ( char * s, size_t n, const char * format, ... );
    //snprintf(buf, 1023, "acc=%.0f & accX=%.0f & accY=%.0f & accZ=%.0f & sensortemp=%.2f",
    //                g_accTotalAvg,
    //                g_accXAvg,
    //                g_accYAvg,
    //                g_accZAvg,
    //                sensorTemp );
    snprintf(buf, 99, "acc=%.0f & accX=%.0f & accY=%.0f & accZ=%.0f & sensortemp=%.2f",
                        g_accTotalAvg,
                        g_accXAvg,
                        g_accYAvg,
                        g_accZAvg,
                        sensorTemp );
    UART_PRINT(buf);

    //UART_PRINT("\n\r");
    //UART_PRINT("HTTP Post Begin:\n\r");
    //lRetVal = HTTPPostMethod(&httpClient);
    //if(lRetVal < 0)
    //{
    //	UART_PRINT("HTTP Post failed.\n\r");
    //}
    //UART_PRINT("HTTP Post End:\n\r");

    UART_PRINT("\n\r");
    UART_PRINT("HTTP Delete Begin:\n\r");
    lRetVal = HTTPDeleteMethod(&httpClient);

    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Delete failed.\n\r");
    }
    UART_PRINT("HTTP Delete End:\n\r");


    UART_PRINT("\n\r");
    UART_PRINT("HTTP Put Begin:\n\r");
    lRetVal = HTTPPutMethod(&httpClient);
    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Put failed.\n\r");
    }
    UART_PRINT("HTTP Put End:\n\r");

    UART_PRINT("\n\r");
    UART_PRINT("HTTP Get Begin:\n\r");
    lRetVal = HTTPGetMethod(&httpClient);
    if(lRetVal < 0)
    {
    	UART_PRINT("HTTP Post Get failed.\n\r");
    }
    UART_PRINT("HTTP Get End:\n\r");
    UART_PRINT("\n\r");



    //UART_PRINT("\n\r");
    //UART_PRINT("HTTP Post with Temperature and Accelerometer Data Begin:\n\r");
    //lRetVal = HTTPPostMethod_data(&httpClient);
    //if(lRetVal < 0)
    //{
    //	UART_PRINT("HTTP Post with Temperature and Accelerometer Data failed.\n\r");
    //}
    //    UART_PRINT("HTTP Post with Temperature and Accelerometer Data End:\n\r");
    //MAP_GPIOPinWrite(GPIOA2_BASE,0x40,0x40);
    //MAP_UtilsDelay(delay);
    //MAP_GPIOPinWrite(GPIOA3_BASE,0x10,0);
    //MAP_UtilsDelay(delay);
    //MAP_GPIOPinWrite(GPIOA2_BASE,0x40,0);

    //char c = 0;
    int cx;
    const char *strPtr;
    while(1){

    	MAP_UtilsDelay(40000000);

    	//c = MAP_UARTCharGetNonBlocking(CONSOLE);        // Get a single character
    	//if (c != 0){
    	//	g_UartHaveCmd = GETChar(&g_ucUARTRecvBuffer1[0]);
    	//}


    	AccSample(); // Just do a single reading for now. TODO: Make Async.
    	SetAccAvg(); // g_accXAvg, g_accYAvg, g_accZAvg, g_accTotalAvg

    	TMP006DrvGetTemp(&sensorTemp);

    	//int snprintf ( char * s, size_t n, const char * format, ... );
    	//snprintf(buf, 1023, "acc=%.0f & accX=%.0f & accY=%.0f & accZ=%.0f & sensortemp=%.2f",
    	//        g_accTotalAvg,
    	//        g_accXAvg,
    	//        g_accYAvg,
    	//        g_accZAvg,
    	//        sensorTemp );
    	//"acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85"
    	cx = snprintf(buf, 99, "acc=%.0f & accX=%.0f & accY=%.0f & accZ=%.0f & sensortemp=%.2f",
    	    	 g_accTotalAvg,
    	    	 g_accXAvg,
    	    	 g_accYAvg,
    	    	 g_accZAvg,
    	    	 sensorTemp );//cx is indice of the last buf[cx]
    	if (cx>=0 && cx<99)	{// check returned value
    		//bufPtr = &buf[cx];//bufPtr now points to buf[cx]
    		//snprintf ( buf+cx, 99-cx, ", and the half of that is %d.", 60/2/2 );
    		strPtr = " & loc=Los Angeles \0";
    		//char * strcpy ( char * destination, const char * source );
    		//snprintf ends string with '\0' since the string needs to continue '\0' has to be replaced with a character
    		//that is why cx - 1
    		strcpy ( &buf[(cx - 1)], strPtr );//"acc=26 & accX=13 & accY=-1 & accZ=67 & sensortemp=23.85 & loc=Los Angeles \0"
    	}

    	UART_PRINT(buf);
    	UART_PRINT("\n\r");

    	MAP_UtilsDelay(40000000);
    	lRetVal = HTTPPostMethod_data(&httpClient);
    	UART_PRINT("\n\r");
    	if(lRetVal < 0)
    	{
    		UART_PRINT("HTTP Post with Temperature and Accelerometer Data failed.\n\r");
    	 }
    	    //UART_PRINT("HTTP Post with Temperature and Accelerometer Data End:\n\r");

    }

    // Stop the CC3200 device

    //LOOP_FOREVER();
}

