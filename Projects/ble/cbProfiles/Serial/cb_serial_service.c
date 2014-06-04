/*---------------------------------------------------------------------------
* Copyright (c) 2000, 2001 connectBlue AB, Sweden.
* Any reproduction without written permission is prohibited by law.
*
* Component   : Serial Port Service 
* File        : cb_serial_service.c
*
* Description : Implementation of Serial Port Service service.
*               During connection establishment with directed advertisements
*               callbacks from the stack may in an unpredicted order. State
*               transitions related to this are commented with "Directed adv"
*               TBD mention security
*-------------------------------------------------------------------------*/

#include "bcomdef.h"
#include "OSAL.h"
#ifndef cbSPS_INDICATIONS
#include "OSAL_Timers.h"
#endif
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "hal_assert.h"
#include "cb_log.h"

#include "cb_assert.h"
#include "cb_serial_service.h"
#include "peripheral.h"

#ifdef cbSPS_READ_SECURITY_MODE
#include "cb_gap.h"
#include "cb_sec.h"
#endif

/*===========================================================================
* DEFINES
*=========================================================================*/
#define cbSPS_DEBUG

#define ATTRIBUTE128(uuid, pProps, pValue) { {ATT_UUID_SIZE, uuid}, pProps, 0, (uint8*)pValue}
#define ATTRIBUTE16(uuid, pProps, pValue) { {ATT_BT_UUID_SIZE, uuid}, pProps, 0, (uint8*)pValue}

#ifndef cbSPS_MAX_CALLBACKS
#define cbSPS_MAX_CALLBACKS (5)
#endif

#define cbSPS_MAX_LINKS                               (1)
#define cbSPS_INVALID_ID                              (0xFF)

#define cbSPS_POLL_TX_EVENT                           (1 << 0)
#define cbSPS_ACL_DISCONNECT_TIMEOUT_EVENT            (1 << 1)
#define cbSPS_TX_POLL_TIMEOUT_IN_MS                   (10)      



/*===========================================================================
* TYPES
*=========================================================================*/
typedef enum
{
    SPS_S_NOT_VALID = 0,
    SPS_S_IDLE,
    SPS_S_ACL_CONNECTED,
    SPS_S_CONNECTED,
    SPS_S_TX_IDLE,
    SPS_S_TX_WAIT,
    SPS_S_WAIT_DISCONNECT,
    SPS_S_WAIT_ACL_DISCONNECT,

    SPS_S_RX_READY

} cbSPS_State;

typedef struct  
{
    uint8         taskId;
    cbSPS_State   state;
    cbSPS_State   txState;
    cbSPS_State   rxState;
    bool          enabled;
    bool          creditsFlowControlUsed;
    int8          txCredits; // Number of packets that can be sent
    int8          rxCredits; // Number of packets that remote side can send
    uint16        connHandle;
    bool          secureConnection; // Encryption required 
    bool          notifyDisconnect;

    uint16        remainingBufSize;

    uint8         *pPendingTxBuf;
    uint8         pendingTxBufSize;

    uint16        creditsCccValue;
    uint16        fifoCccValue; 

#ifdef cbSPS_DEBUG
    uint32        dbgTxCount;
    uint32        dbgRxCount;
    uint32        dbgTxCreditsCount;
    uint32        dbgRxCreditsCount;
#endif
} cbSPS_Class;

/*===========================================================================
* DECLARATIONS
*=========================================================================*/
// Operations registered to BLE stack
static void handleConnStatusCB( uint16 connHandle, uint8 changeType );
static uint8 readAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 *pValue, uint8 len, uint16 offset );
//static bStatus_t authorizeAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 opcode );

// Operations handles incoming write operations
static void creditsReceiveHandler(uint16 connHandle, int8 credits);
static void fifoReceiveHandler(uint16 connHandle, uint8 *pBuf, uint8 size);

// Operations that sends indications to remote device
static bStatus_t writeFifo(uint16 connHandle, uint8 *pBuf, uint8 size);
static bStatus_t writeCredits(uint16 connHandle, uint8 credits);

// Operations used to call a set of registered callbacks
static void connectEvtCallback(uint16 connHandle);
static void disconnectEvtCallback(uint16 connHandle);
static void dataEvtCallback(uint16 connHandle, uint8 *pBuf, uint8 size);
static void dataCnfCallback(uint16 connHandle);

static void pollTx(void);
static void resetLink(void);
static void initConnection(uint16 connHandle, bool flowControlUsed);



/*===========================================================================
* DEFINITIONS
*=========================================================================*/
// Filename used by cb_ASSERT macro
static const char *file = "SPS";

CONST uint8 cbSPS_servUUID[ATT_UUID_SIZE] = {cbSPS_SERIAL_SERVICE_UUID};
CONST uint8 cbSPS_modeUUID[ATT_UUID_SIZE] = {cbSPS_MODE_UUID};
CONST uint8 cbSPS_fifoUUID[ATT_UUID_SIZE] = { cbSPS_FIFO_UUID };
CONST uint8 cbSPS_creditsUUID[ATT_UUID_SIZE] = { cbSPS_CREDITS_UUID };

CONST gattAttrType_t cbSPS_serviceUUID = { ATT_UUID_SIZE, cbSPS_servUUID };

// Characteristic properties
static uint8 fifoCharProps =    GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP | GATT_PROP_NOTIFY /*| GATT_PROP_INDICATE*/; 
static uint8 creditsCharProps = GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP | GATT_PROP_NOTIFY /*| GATT_PROP_INDICATE*/; 

// Characteristic configurations
static gattCharCfg_t creditsCCC[GATT_MAX_NUM_CONN];
static gattCharCfg_t fifoCCC[GATT_MAX_NUM_CONN]; 

//Characteristic data
static uint8 fifo[1]; // Note that no data is ever stored here. Size set to 1 to save memory
static uint8 credits;

// Attribute handles that are cached for faster access
static uint16 attrHandleFifo = 0;
static uint16 attrHandleCredits = 0;
static uint16 attrHandleFifoCCC = 0;
static uint16 attrHandleCreditsCCC = 0;

// Attribute table
static gattAttribute_t spsAttrTbl[] = 
{
    // Serial Port Service
    ATTRIBUTE16( primaryServiceUUID, GATT_PERMIT_READ, &cbSPS_serviceUUID),

    // Fifo Characteristic
    ATTRIBUTE16(characterUUID     , GATT_PERMIT_READ, &fifoCharProps),
    ATTRIBUTE128(cbSPS_fifoUUID   , GATT_PERMIT_WRITE , fifo),
    ATTRIBUTE16(clientCharCfgUUID , GATT_PERMIT_READ | GATT_PERMIT_WRITE , fifoCCC),

    // Credits Characteristic
    ATTRIBUTE16(characterUUID     , GATT_PERMIT_READ, &creditsCharProps),
    ATTRIBUTE128(cbSPS_creditsUUID, GATT_PERMIT_WRITE , &credits),
    ATTRIBUTE16(clientCharCfgUUID , GATT_PERMIT_READ | GATT_PERMIT_WRITE , creditsCCC),  
};

CONST gattServiceCBs_t serialCBs =
{
    readAttrCB,       // Read callback function pointer
    writeAttrCB,      // Write callback function pointer
    NULL,             // authorizeAttrCB,  Authorization callback function pointer
};

static cbSPS_Callbacks *spsCallbacks[cbSPS_MAX_CALLBACKS] = {NULL, NULL, NULL, NULL, NULL};
static cbSPS_Class sps;

/*===========================================================================
* FUNCTIONS
*=========================================================================*/

void cbSPS_init(uint8 taskId)
{
    sps.taskId = taskId;  
    sps.state = SPS_S_IDLE;
    sps.txState = SPS_S_NOT_VALID;
    sps.rxState = SPS_S_NOT_VALID;
    sps.enabled = FALSE;
    sps.creditsFlowControlUsed = FALSE;
    sps.secureConnection = FALSE;
    resetLink();

    sps.fifoCccValue = 0;
    sps.creditsCccValue = 0;

#ifdef cbSPS_DEBUG
    sps.dbgTxCount = 0;
    sps.dbgRxCount = 0;
    sps.dbgTxCreditsCount = 0;  
    sps.dbgRxCreditsCount = 0;
#endif
}

/*---------------------------------------------------------------------------
* Register service to GATT
*-------------------------------------------------------------------------*/
void cbSPS_addService(void)
{
    uint8 status;
    gattAttribute_t *pAttr;

    linkDB_Register( handleConnStatusCB );    
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, fifoCCC );
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, creditsCCC );  

    status = GATTServApp_RegisterService( spsAttrTbl, GATT_NUM_ATTRS( spsAttrTbl ), &serialCBs );
    cb_ASSERT(status == SUCCESS);

    //Init attribute handles needed for indications and notifications

    pAttr = GATTServApp_FindAttr(spsAttrTbl, GATT_NUM_ATTRS( spsAttrTbl ), fifo );
    cb_ASSERT(pAttr != NULL);
    attrHandleFifo = pAttr->handle;

    pAttr = GATTServApp_FindAttr(spsAttrTbl, GATT_NUM_ATTRS( spsAttrTbl ), (uint8*)&credits );
    cb_ASSERT(pAttr != NULL);
    attrHandleCredits = pAttr->handle;

    pAttr = GATTServApp_FindAttr(spsAttrTbl, GATT_NUM_ATTRS( spsAttrTbl ), (uint8*)fifoCCC );
    cb_ASSERT(pAttr != NULL);
    attrHandleFifoCCC = pAttr->handle;

    pAttr = GATTServApp_FindAttr(spsAttrTbl, GATT_NUM_ATTRS( spsAttrTbl ), (uint8*)creditsCCC );
    cb_ASSERT(pAttr != NULL);
    attrHandleCreditsCCC = pAttr->handle;
    
#ifdef cbSPS_READ_SECURITY_MODE
    // In the serial port application the security mode is read from cbSEC
    {
        cbSEC_SecurityMode securityMode;
        status = cbSEC_getSecurityMode(&securityMode);
        cb_ASSERT(status == SUCCESS);

        if (securityMode == cbSEC_SEC_MODE_AUTOACCEPT)
        {
            cbSPS_setSecurity(FALSE, FALSE);
        }
        else if (securityMode == cbSEC_SEC_MODE_JUST_WORKS)
        {
            // Encryption required
            cbSPS_setSecurity(TRUE, FALSE);
        }
        else
        {
            cbSPS_setSecurity(TRUE, TRUE);
        }
    }
#endif
}

/*---------------------------------------------------------------------------
* Security settings
* - encryption, set to to TRUE to enforce that the link is encrypted. This 
*              will trig a an unauthenticated bonding. (Just Works)
* - authentication, set to TRUE to enforce that an authenticated and encrypted
*              link is used. This will trig an authentication bonding.
*-------------------------------------------------------------------------*/
void cbSPS_setSecurity(bool encryption, bool authentication)
{
    gattAttribute_t *pAttr;

    // List of attributes for which security config applies. Some of the attributes are always readable.
    uint8 *attrValuePointer[6] =  {fifo, &credits, (uint8*)&fifoCCC, (uint8*)creditsCCC};

    sps.secureConnection = encryption;

    for(uint8 i = 0; i < 4; i++)
    {
        pAttr = GATTServApp_FindAttr(spsAttrTbl, GATT_NUM_ATTRS( spsAttrTbl ), attrValuePointer[i] );
        cb_ASSERT(pAttr != NULL);

        if(authentication == TRUE)
        {
            // Only authenticated read / writes allowed
            if((pAttr->permissions & (GATT_PERMIT_READ | GATT_PERMIT_AUTHEN_READ)) != 0)
            {
                pAttr->permissions |= (GATT_PERMIT_AUTHEN_READ);
            }

            if((pAttr->permissions & (GATT_PERMIT_WRITE | GATT_PERMIT_AUTHEN_WRITE)) != 0)
            {

                pAttr->permissions |= (GATT_PERMIT_AUTHEN_WRITE);
            }
        }
        else
        {
            // Authenticated and unauthenticated read / writes allowed
            if((pAttr->permissions & (GATT_PERMIT_READ | GATT_PERMIT_AUTHEN_READ)) != 0)
            {
                pAttr->permissions &= ~GATT_PERMIT_AUTHEN_READ;        
            }

            if((pAttr->permissions & (GATT_PERMIT_WRITE | GATT_PERMIT_AUTHEN_WRITE)) != 0)
            {

                pAttr->permissions &= ~(GATT_PERMIT_AUTHEN_WRITE);
            }
        }    
    }
}

/*---------------------------------------------------------------------------
* Register callback functions
*-------------------------------------------------------------------------*/
extern void cbSPS_register(cbSPS_Callbacks *pCallbacks)
{
    uint8 i;
    bool  found = FALSE;

    cb_ASSERT(pCallbacks != NULL);

    for (i = 0; ((i < cbSPS_MAX_CALLBACKS) && (found == FALSE)); i++)
    {
        if (spsCallbacks[i] == NULL)
        {
            spsCallbacks[i] = pCallbacks;
            found = TRUE;
        }
    }
    cb_ASSERT(found == TRUE);
}

/*---------------------------------------------------------------------------
* Unregister callback functions
*-------------------------------------------------------------------------*/
extern void cbSPS_unregister(cbSPS_Callbacks *pCallbacks)
{
    uint8 i;
    bool  found = FALSE;

    cb_ASSERT(pCallbacks != NULL);

    for (i = 0; ((i < cbSPS_MAX_CALLBACKS) && (found == FALSE)); i++)
    {
        if (spsCallbacks[i] == pCallbacks)
        {
            spsCallbacks[i] = NULL;
            found = TRUE;
        }
    }

    cb_ASSERT(found == TRUE);
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
uint16 cbSPS_processEvent(uint8 taskId, uint16 events)
{
    if ((events & cbSPS_POLL_TX_EVENT) != 0)
    {
        // Transmit pending fifo data or credits to remote side
        pollTx();
        return (events ^ cbSPS_POLL_TX_EVENT);
    }

    if ((events & cbSPS_ACL_DISCONNECT_TIMEOUT_EVENT) != 0)
    {
        if (sps.state == SPS_S_WAIT_DISCONNECT)
        {
            // Disconnect
            // cbGAP not part of demo application
            GAPRole_TerminateConnection();
            sps.state = SPS_S_WAIT_ACL_DISCONNECT;
        }

        return (events ^ cbSPS_ACL_DISCONNECT_TIMEOUT_EVENT);
    }

    return 0;
}

/*---------------------------------------------------------------------------
* Write fifo data. If notifications or indications have been enabled
* then fifo data will be sent to remote device.
*-------------------------------------------------------------------------*/
uint8 cbSPS_reqData(uint16 connHandle, uint8 *pBuf, uint8 size)
{
    bStatus_t status = FAILURE;

    cb_ASSERT(size != 0);
    cb_ASSERT(pBuf != NULL);
    cb_ASSERT(sps.pPendingTxBuf == NULL);
    cb_ASSERT(sps.pendingTxBufSize == 0);

    if (sps.state == SPS_S_CONNECTED)
    {
        switch (sps.txState)
        {
        case SPS_S_TX_IDLE:
        case SPS_S_TX_WAIT:
            sps.pPendingTxBuf = pBuf;
            sps.pendingTxBufSize = size;
            status = SUCCESS;
            osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT); 
            break;

        default:
            cb_EXIT(sps.state);
            break;
        }    
    }

    return status;
}

/*---------------------------------------------------------------------------
* Write credits. If notifications or indications have been enabled
* then credits will be sent to remote device.
*-------------------------------------------------------------------------*/
uint8 cbSPS_setRemainingBufSize(uint16 connHandle, uint16 size)
{
    bStatus_t status = FAILURE;

    if (sps.state == SPS_S_CONNECTED)
    {
        sps.remainingBufSize = size;
        osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT); 
        status = SUCCESS;
    }

    return status;
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void cbSPS_enable(void)
{
    sps.enabled = TRUE;
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void cbSPS_disable(void)
{
    sps.enabled = FALSE;

    switch (sps.state)
    {
    case SPS_S_CONNECTED:
        GAPRole_TerminateConnection();        
        sps.state = SPS_S_WAIT_ACL_DISCONNECT;        
        break;

    case SPS_S_WAIT_DISCONNECT:
    case SPS_S_WAIT_ACL_DISCONNECT:
        // Disconnect
        // cbGAP not part of demo application
        GAPRole_TerminateConnection();        
        sps.state = SPS_S_WAIT_ACL_DISCONNECT;
        break;
    default:
        break;
    }
}

/*===========================================================================
* STATIC FUNCTIONS
*=========================================================================*/

/*---------------------------------------------------------------------------
* Read callback
* None of the characteristics are readable.
*-------------------------------------------------------------------------*/
static bStatus_t readAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;
    *pLen = 0; 

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if ( offset > 0 )
    {
        return ( ATT_ERR_ATTR_NOT_LONG );
    }

    if ((sps.secureConnection == TRUE) &&
        (linkDB_Encrypted(connHandle) == FALSE))
    {
        return ATT_ERR_INSUFFICIENT_AUTHEN;
        //return ATT_ERR_INSUFFICIENT_ENCRYPT; // Does not trig a bonding
    }    


    status = ATT_ERR_INVALID_HANDLE;

    return ( status );
}

/*---------------------------------------------------------------------------
* Write callback
*-------------------------------------------------------------------------*/
static bStatus_t writeAttrCB( uint16 connHandle, gattAttribute_t *pAttr, uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;      

    if ((sps.secureConnection == TRUE) &&
        (linkDB_Encrypted(connHandle) == FALSE))
    {
        return ATT_ERR_INSUFFICIENT_AUTHEN;
        //return ATT_ERR_INSUFFICIENT_ENCRYPT; // Does not trig a bonding
    }

    if ( pAttr->type.len == ATT_UUID_SIZE )
    {   
        // Make sure it is not a blob operation
        if ( offset != 0 )
        {
            status = ATT_ERR_ATTR_NOT_LONG;
        }
        else if (osal_memcmp(pAttr->type.uuid, cbSPS_fifoUUID, ATT_UUID_SIZE) == TRUE)
        {
            fifoReceiveHandler(connHandle, pValue, len);
        }
        else if (osal_memcmp(pAttr->type.uuid, cbSPS_creditsUUID, ATT_UUID_SIZE) == TRUE)
        {
            if (len == 1)
            {
                if ((sps.creditsCccValue & GATT_CLIENT_CFG_NOTIFY) != 0)
                {
                    creditsReceiveHandler(connHandle, pValue[0]);
                }
                else
                {
                    // This may happen if bond is lost on peripheral side
                    status = 0xFD; // "improper configuration" not part of att.h
                }
            }
            else
            {
                status = ATT_ERR_INVALID_VALUE_SIZE;
            }    
        }
        else
        {
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    else if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {    
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

        switch (uuid)
        {
        case GATT_CLIENT_CHAR_CFG_UUID:
            cb_ASSERT(offset == 0);

            status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len, offset, GATT_CLIENT_CFG_NOTIFY);            

            if (status == SUCCESS)
            {
                sps.fifoCccValue = GATTServApp_ReadCharCfg( connHandle, fifoCCC ); 
                sps.creditsCccValue = GATTServApp_ReadCharCfg( connHandle, creditsCCC ); 

                // Update the Client Char Configuration for bonded devices.
                // Ignore return values, Failure is returned if the devices are not bonded
                if (pAttr->handle == attrHandleFifoCCC)
                {
                    GAPBondMgr_UpdateCharCfg(connHandle, attrHandleFifoCCC, sps.fifoCccValue);
                }
                
                if (pAttr->handle == attrHandleCreditsCCC)
                {
                    GAPBondMgr_UpdateCharCfg(connHandle, attrHandleCreditsCCC, sps.creditsCccValue);
                }                                                                                 

                // Connections are only accepted when not in AT mode
                if (sps.enabled == TRUE)         
                {   
                    if (attrHandleFifoCCC == pAttr->handle)
                    {               
                        // Check if "no flow control" mode shall be used. 
                        // This mode is triggered by enabling fifo notifications without
                        // enabling notifications for the credits.
                        if (((sps.state == SPS_S_ACL_CONNECTED) || (sps.state == SPS_S_WAIT_DISCONNECT) ||
                            (sps.state == SPS_S_IDLE)) && // Directed Adv
                            ((sps.fifoCccValue & GATT_CLIENT_CFG_NOTIFY) != 0) && 
                             (sps.creditsCccValue == 0))
                        {  
                            // Notifications enabled, connection without credits flow control initiated
                            initConnection(connHandle, FALSE);
                            connectEvtCallback(connHandle);
                        }
                        else if ((sps.creditsFlowControlUsed == FALSE) &&
                                 (sps.state == SPS_S_CONNECTED) &&
                                 ((sps.fifoCccValue & GATT_CLIENT_CFG_NOTIFY) == 0))
                        {
                            // Notifications disabled when connected, disconnection without credits flow control
                            sps.state = SPS_S_ACL_CONNECTED;
                            disconnectEvtCallback(sps.connHandle);                         
                        }
                    }
                }                
            }
            else
            {
                status = ATT_ERR_WRITE_NOT_PERMITTED;
            }
            break;

        default:
            // Should never get here!
            status = ATT_ERR_ATTR_NOT_FOUND;
            break;
        }     
    }  

    return ( status );
}


/*---------------------------------------------------------------------------
* Connection status callback
*-------------------------------------------------------------------------*/
static void handleConnStatusCB( uint16 connHandle, uint8 changeType )
{        
#ifdef cbSPS_BOND_MANAGER_CCC_WORKARAOUND
    uint16 fifoCccBondMan;
    uint16 creditsCccBondMan;
    uint8 statusFifo, statusCredits;
#endif
    
    // Make sure this is not loopback connection
    if(connHandle != LOOPBACK_CONNHANDLE)
    {    
        switch (changeType)
        {
        case LINKDB_STATUS_UPDATE_NEW:
            // ACL connection established
            cb_ASSERT((sps.state == SPS_S_IDLE));

#ifndef cbSPS_BOND_MANAGER_CCC_WORKARAOUND
            sps.fifoCccValue = GATTServApp_ReadCharCfg( connHandle, fifoCCC ); 
            sps.creditsCccValue = GATTServApp_ReadCharCfg( connHandle, creditsCCC );
#else
            // Workaround for race condition between CCC notifications 
            // from the bond manager and rx first credits when bonded
            statusFifo = GAPBondMgr_ReadCharCfg(connHandle, attrHandleFifoCCC, &fifoCccBondMan);
            statusCredits = GAPBondMgr_ReadCharCfg(connHandle, attrHandleCreditsCCC, &creditsCccBondMan);
            
            if (statusFifo == SUCCESS)
            {
              sps.fifoCccValue = fifoCccBondMan;     
            }          
            
            if (statusCredits == SUCCESS)
            {
              sps.creditsCccValue = creditsCccBondMan;     
            } 
#endif
                                        
            if ((sps.enabled == TRUE) &&
                ((sps.fifoCccValue & GATT_CLIENT_CFG_NOTIFY) != 0) && 
                ((sps.creditsCccValue & GATT_CLIENT_CFG_NOTIFY) == 0))
            {
                // Notifications enabled, connection without credits flow control initiated
                initConnection(connHandle, FALSE);
                connectEvtCallback(connHandle);                
            }
            else
            {
                sps.state = SPS_S_ACL_CONNECTED;
            }
            sps.connHandle = connHandle;
            break;

        case LINKDB_STATUS_UPDATE_REMOVED:
            // ACL connection terminated
            switch (sps.state)
            {        
            case SPS_S_CONNECTED:
            case SPS_S_WAIT_DISCONNECT:
            case SPS_S_WAIT_ACL_DISCONNECT:
                sps.state = SPS_S_IDLE;
                sps.txState = SPS_S_NOT_VALID;
                sps.rxState = SPS_S_NOT_VALID;    
                resetLink();
                disconnectEvtCallback(connHandle);
                
                GATTServApp_InitCharCfg( INVALID_CONNHANDLE, fifoCCC );
                GATTServApp_InitCharCfg( INVALID_CONNHANDLE, creditsCCC );
                sps.fifoCccValue = 0;
                sps.creditsCccValue = 0;
                break;

            case SPS_S_ACL_CONNECTED:              
              sps.state = SPS_S_IDLE;
              break;
              
            case SPS_S_IDLE:              
              // Ignore Directed Adv
              break;              

            default:
                cb_ASSERT(FALSE);
                break;
            }
            break;

        case LINKDB_STATUS_UPDATE_STATEFLAGS:       
            if (!linkDB_Up(connHandle))
            {
                cb_ASSERT(FALSE); // TBD - Can this happen? If so find out how and why.
            }
            break;

        default:
            break;
        }
    }
}

/*---------------------------------------------------------------------------
* This operation sends credits or data. If fifo data is successfully
* written to a lower layer then the data cnf callback is called immidiately
* allowing a higher layers to start a new write. If data can not be written
* to lower layer then a new poll is trigged after a timeout.
*-------------------------------------------------------------------------*/
static void pollTx(void)
{
    bStatus_t status;
    uint8 newCredits;

    switch (sps.state)
    {

    case SPS_S_CONNECTED:
        switch (sps.txState)
        {
        case SPS_S_TX_IDLE:
        case SPS_S_TX_WAIT:
            {
                if ((sps.creditsFlowControlUsed == TRUE) &&
                    (sps.rxCredits == 0) &&
                    (sps.remainingBufSize > cbSPS_FIFO_SIZE))
                {
                    newCredits = (sps.remainingBufSize / cbSPS_FIFO_SIZE) - sps.rxCredits;          

                    status = writeCredits(sps.connHandle, newCredits);

                    if (status == SUCCESS)
                    {
                        sps.remainingBufSize = 0;
                        sps.rxCredits += newCredits;

#ifdef cbSPS_DEBUG
                        sps.dbgRxCreditsCount += newCredits;       
#endif

                        // Trig another poll to send pending fifo data as well
                        if (sps.pPendingTxBuf != NULL)
                        {
                            osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT);        
                        }
                    }
                    else
                    {
                        sps.txState = SPS_S_TX_WAIT;
                        osal_start_timerEx(sps.taskId, cbSPS_POLL_TX_EVENT, cbSPS_TX_POLL_TIMEOUT_IN_MS);
                        //osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT);
                    }
                }
                else if ((sps.pPendingTxBuf != NULL) &&
                         ((sps.creditsFlowControlUsed == FALSE) || 
                          (sps.txCredits > 0)))
                {
                    status = writeFifo(sps.connHandle, sps.pPendingTxBuf, sps.pendingTxBufSize);

                    if (status == SUCCESS)
                    {

#ifdef cbSPS_DEBUG
                        sps.dbgTxCount += sps.pendingTxBufSize;
#endif
                        sps.txCredits--;
                        sps.pPendingTxBuf = NULL;
                        sps.pendingTxBufSize = 0;
                        sps.txState = SPS_S_TX_IDLE;

                        dataCnfCallback(sps.connHandle);
                    }
                    else
                    {
                        sps.txState = SPS_S_TX_WAIT;
                        //osal_start_timerEx(sps.taskId, cbSPS_POLL_TX_EVENT, cbSPS_TX_POLL_TIMEOUT_IN_MS);
                        osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT);
                    }
                }
            }
            break;

        default:
            cb_ASSERT(FALSE);
            break;
        }
        break;

    case SPS_S_WAIT_DISCONNECT:
        // Transmit disconnect confirm to remote side
        status = writeCredits(sps.connHandle, -1);

        if (status == SUCCESS)
        {
            sps.state = SPS_S_ACL_CONNECTED;           
        }
        else
        {
            // The radio tx queue is full. Try again later.
            osal_start_timerEx(sps.taskId, cbSPS_POLL_TX_EVENT, cbSPS_TX_POLL_TIMEOUT_IN_MS);
        }
        break;
    }
}

/*---------------------------------------------------------------------------
* Reset link variables
*-------------------------------------------------------------------------*/
static void resetLink(void)
{
    sps.txCredits = 0;
    sps.rxCredits = 0;
    sps.connHandle = INVALID_CONNHANDLE;
    sps.remainingBufSize = 0;
    sps.pPendingTxBuf = NULL;
    sps.pendingTxBufSize = 0;
}

static void initConnection(uint16 connHandle, bool flowControlUsed)
{
    sps.connHandle = connHandle;
    sps.state = SPS_S_CONNECTED;
    sps.txState = SPS_S_TX_IDLE;
    sps.rxState = SPS_S_RX_READY;
    sps.creditsFlowControlUsed = flowControlUsed;
    sps.notifyDisconnect = true;
}

/*---------------------------------------------------------------------------
* Handle received credits
*-------------------------------------------------------------------------*/
static void creditsReceiveHandler(uint16 connHandle, int8 credits)
{
    uint8 status;
    cb_ASSERT(credits != 0);

    switch(sps.state)
    {        
    case SPS_S_IDLE: // Directed adv
    case SPS_S_ACL_CONNECTED:      
        if (sps.enabled == TRUE) 
        { 
            if (((sps.fifoCccValue & GATT_CLIENT_CFG_NOTIFY) != 0) &&
                ((sps.creditsCccValue & GATT_CLIENT_CFG_NOTIFY) != 0) &&
                (credits > 0))
            {
                sps.txCredits += credits;
#ifdef cbSPS_DEBUG
                sps.dbgTxCreditsCount += credits;       
#endif     
                initConnection(connHandle, TRUE);        
                connectEvtCallback(connHandle);                
            }
            else
            {
                //Ignore the "connection request" if the client configuration is not correct
            }
        }
        else
        {
            // Reject connection by sending credits = -1
            sps.state = SPS_S_WAIT_DISCONNECT;
            osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT); 
        }
        break;

    case SPS_S_CONNECTED:
        {
            switch (sps.txState)
            {
            case SPS_S_TX_IDLE:
            case SPS_S_TX_WAIT:

                if (credits > 0)
                {
                    sps.txCredits += credits;
#ifdef cbSPS_DEBUG
                    sps.dbgTxCreditsCount += credits;       
#endif        
                    osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT); 
                }
                else if (credits == -1)
                {
                    // The central has initiated a disconnection. 
                    sps.state = SPS_S_WAIT_DISCONNECT;
                    sps.txState = SPS_S_NOT_VALID;
                    sps.rxState = SPS_S_NOT_VALID;                   

                    sps.txCredits = 0;
                    sps.rxCredits = 0;

                    sps.remainingBufSize = 0;
                    sps.pPendingTxBuf = NULL;
                    sps.pendingTxBufSize = 0;

                    // The call below will trig disconnect confirmation (credits = -1)
                    osal_set_event(sps.taskId, cbSPS_POLL_TX_EVENT); 

                    disconnectEvtCallback(connHandle);
                }
                break;

            default:
                cb_ASSERT(FALSE);
                break;
            }
        }
        break;

    case SPS_S_WAIT_ACL_DISCONNECT:
    case SPS_S_WAIT_DISCONNECT:      
        // Reconnection on SPS level without disconnecting the ACL link      
        if (sps.enabled == TRUE) 
        {
            if (credits > 0)
            {
                sps.txCredits += credits;
#ifdef cbSPS_DEBUG
                sps.dbgTxCreditsCount += credits;       
#endif     
                osal_stop_timerEx(sps.taskId, cbSPS_ACL_DISCONNECT_TIMEOUT_EVENT);
                initConnection(connHandle, TRUE);
                connectEvtCallback(connHandle);                
            }          
        }
        else
        {
            // Reject connection by sending credits = -1
            status = writeCredits(sps.connHandle, -1);
            cb_ASSERT(status == SUCCESS);
        }
        break;

    default:
        cb_EXIT(sps.state);
        break;
    }
}

/*---------------------------------------------------------------------------
* Handle incoming fifo data
*-------------------------------------------------------------------------*/
static void fifoReceiveHandler(uint16 connHandle, uint8 *pBuf, uint8 size)
{
    if (sps.state == SPS_S_CONNECTED)
    {      
        switch (sps.rxState)
        {
        case SPS_S_RX_READY:
            sps.rxCredits--;
#ifdef cbSPS_DEBUG
            sps.dbgRxCount += size;
#endif
            dataEvtCallback(connHandle, pBuf, size);
            break;

        default:
            cb_ASSERT(FALSE);
            break;
        }
    }
}

/*---------------------------------------------------------------------------
* Send indication with fifo attribute data to remote side
*-------------------------------------------------------------------------*/
#if 0
static uint8 previousByte = 0;
static uint8 previousStart = 0;
#endif

static bStatus_t writeFifo(uint16 connHandle, uint8 *pBuf, uint8 size)
{
    bStatus_t status = FAILURE;
    attHandleValueNoti_t attribute;

    cb_ASSERT(size <= cbSPS_FIFO_SIZE);

    if (sps.fifoCccValue & GATT_CLIENT_CFG_NOTIFY)
    {    
        cb_ASSERT(attrHandleFifo != 0);

        attribute.handle = attrHandleFifo;
        attribute.len = size;
        osal_memcpy(attribute.value, pBuf, size);

#ifdef cbSPS_DEBUG
        sps.dbgTxCount += size;
#endif

        status = GATT_Notification(connHandle, &attribute, FALSE);
    }

    return status;
}

/*---------------------------------------------------------------------------
* Send indication with credits attribute data to remote side
*-------------------------------------------------------------------------*/
static bStatus_t writeCredits(uint16 connHandle, uint8 credits)
{
    bStatus_t status = FAILURE;
    attHandleValueNoti_t attribute;

    if((sps.creditsCccValue & GATT_CLIENT_CFG_NOTIFY) != 0)
    {
        cb_ASSERT(attrHandleCredits != 0);

        attribute.handle = attrHandleCredits;
        attribute.len = 1;
        attribute.value[0] = credits;

        status = GATT_Notification(connHandle , &attribute, FALSE);
    }

    return status;
}
/*---------------------------------------------------------------------------
* Notify all registered users
*-------------------------------------------------------------------------*/
static void connectEvtCallback(uint16 connHandle)
{
    uint8 i;
    for(i = 0; (i < cbSPS_MAX_CALLBACKS); i++)
    {
        if ((spsCallbacks[i] != NULL) &&  
            (spsCallbacks[i]->connectEventCallback != NULL))
        {
            spsCallbacks[i]->connectEventCallback(connHandle);
        }
    }
}

/*---------------------------------------------------------------------------
* Notify all registered users
*-------------------------------------------------------------------------*/
static void disconnectEvtCallback(uint16 connHandle)
{
    uint8 i;    
    if (sps.notifyDisconnect == TRUE)
    {
        sps.notifyDisconnect = FALSE;

        for(i = 0; (i < cbSPS_MAX_CALLBACKS); i++)
        {
            if((spsCallbacks[i] != NULL) && 
                (spsCallbacks[i]->disconnectEventCallback != NULL))
            {
                spsCallbacks[i]->disconnectEventCallback(connHandle);
            }
        }
    }
}

/*---------------------------------------------------------------------------
* Notify all registered users
*-------------------------------------------------------------------------*/
static void dataEvtCallback(uint16 connHandle, uint8 *pBuf, uint8 size)
{
    uint8 i;
    for(i = 0; (i < cbSPS_MAX_CALLBACKS); i++)
    {
        if((spsCallbacks[i] != NULL) &&  
            (spsCallbacks[i]->dataEventCallback != NULL))
        {
            spsCallbacks[i]->dataEventCallback(connHandle, pBuf, size);
        }
    }
}

/*---------------------------------------------------------------------------
* Notify all registered users
*-------------------------------------------------------------------------*/
static void dataCnfCallback(uint16 connHandle)
{
    uint8 i;
    for(i = 0; (i < cbSPS_MAX_CALLBACKS); i++)
    {
        if((spsCallbacks[i] != NULL) && 
            (spsCallbacks[i]->dataCnfCallback != NULL))
        {
            spsCallbacks[i]->dataCnfCallback(connHandle);
        }
    }
}


/*********************************************************************
*********************************************************************/
