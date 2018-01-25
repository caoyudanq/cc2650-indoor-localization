/*
 *
 * */
#include <xdc/runtime/System.h>
#include <string.h>

#include "bcomdef.h"
#include "osal.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "beacons_profile.h"
#include "simple_peripheral_observer.h"

// BEACONS DISCOVERY SERVICE UUIDs
static const uint8_t beaconsDiscoServUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_DISCO_SERVICE_UUID)
};

static const uint8_t beaconsDiscoScanUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_DISCO_SCAN_UUID)
};

// BEACONS LIST SERVICE UUIDs
static const uint8_t beaconsListServUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_LIST_SERVICE_UUID)
};

static const uint8_t beaconsListGetRecordUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_LIST_GET_RECORD_UUID)
};

static const uint8_t beaconsListTotalCountUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_LIST_TOTAL_COUNT_UUID)
};

static const uint8_t beaconsListMacAddrUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_LIST_MAC_ADDR_UUID)
};

static const uint8_t beaconsListRssiUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_LIST_RSSI_UUID)
};

static const uint8_t beaconsListAgeUUID[ATT_UUID_SIZE] =
{
     TI_BASE_UUID_128(BEACONS_LIST_AGE_UUID)
};


static beaconsProfileCBs_t *beaconsProfile_AppCBs = NULL;

static const gattAttrType_t beaconsDiscoService = {ATT_UUID_SIZE, beaconsDiscoServUUID};
static uint8 beaconsDiscoScanChar = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 beaconsDiscoScanValue = 0;
static uint8 beaconsDiscoScanCharDesc[] = "Start scan - value > 1";

static beaconRecord beacons[DEFAULT_MAX_SCAN_RES];

static const gattAttrType_t beaconsListService = {ATT_UUID_SIZE, beaconsListServUUID};

static uint8 beaconsListGetRecordChar = GATT_PROP_WRITE;
static uint8 beaconsListGetRecordValue = 0;
static uint8 beaconsListGetRecordCharDesc[] = "Index of record";

static uint8 beaconsListTotalCountChar = GATT_PROP_READ;
static uint8 beaconsListTotalCountValue = 0;
static uint8 beaconsListTotalCountCharDesc[] = "Total count of records";

static uint8 beaconsListMacAddrChar = GATT_PROP_READ;
static uint8 beaconsListMacAddrValue[B_ADDR_LEN] = {0, 0, 0, 0, 0, 0};
static uint8 beaconsListMacAddrCharDesc[] = "MAC address";

static uint8 beaconsListRssiChar = GATT_PROP_READ;
static int8  beaconsListRssiValue = 0;
static uint8 beaconsListRssiCharDesc[] = "RSSI";


static bStatus_t beaconsProfileReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                          uint16_t maxLen, uint8_t method);

static bStatus_t beaconsProfileWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset,
                                           uint8_t method);


/*********************************************************************
 * PROFILE CALLBACKS
 */

// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
const gattServiceCBs_t beaconsProfileCBs =
{
  beaconsProfileReadAttrCB,  // Read callback function pointer.
  beaconsProfileWriteAttrCB, // Write callback function pointer.
  NULL            // Authorization callback function pointer.
};

static gattAttribute_t beaconsDiscoServiceAttrTbl[] =
{
     {
          {ATT_BT_UUID_SIZE, primaryServiceUUID},
          GATT_PERMIT_READ,
          0,
          (uint8 *)&beaconsDiscoService
     },
     {
          {ATT_BT_UUID_SIZE, characterUUID},
          GATT_PERMIT_READ,
          0,
          &beaconsDiscoScanChar
     },
     {
          {ATT_UUID_SIZE, beaconsDiscoScanUUID},
          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0,
          &beaconsDiscoScanValue
     },
     {
          {ATT_BT_UUID_SIZE, charUserDescUUID},
          GATT_PERMIT_READ,
          0,
          beaconsDiscoScanCharDesc
     }
};

static gattAttribute_t beaconsListServiceAttTbl[] =
{
     {
           {ATT_BT_UUID_SIZE, primaryServiceUUID},
           GATT_PERMIT_READ,
           0,
           (uint8 *)&beaconsListService
      },
      {
           {ATT_BT_UUID_SIZE, characterUUID},
           GATT_PERMIT_READ,
           0,
           &beaconsListGetRecordChar
      },
      {
           {ATT_UUID_SIZE, beaconsListGetRecordUUID},
           GATT_PERMIT_WRITE,
           0,
           &beaconsListGetRecordValue
      },
      {
           {ATT_BT_UUID_SIZE, charUserDescUUID},
           GATT_PERMIT_READ,
           0,
           beaconsListGetRecordCharDesc
      },
      {
           {ATT_BT_UUID_SIZE, characterUUID},
           GATT_PERMIT_READ,
           0,
           &beaconsListTotalCountChar
      },
      {
           {ATT_UUID_SIZE, beaconsListTotalCountUUID},
           GATT_PERMIT_READ,
           0,
           &beaconsListTotalCountValue
      },
      {
           {ATT_BT_UUID_SIZE, charUserDescUUID},
           GATT_PERMIT_READ,
           0,
           beaconsListTotalCountCharDesc
      },
      {
           {ATT_BT_UUID_SIZE, characterUUID},
           GATT_PERMIT_READ,
           0,
           &beaconsListMacAddrChar
      },
      {
           {ATT_UUID_SIZE, beaconsListMacAddrUUID},
           GATT_PERMIT_READ,
           0,
           beaconsListMacAddrValue
      },
      {
           {ATT_BT_UUID_SIZE, charUserDescUUID},
           GATT_PERMIT_READ,
           0,
           beaconsListMacAddrCharDesc
      },
      {
           {ATT_BT_UUID_SIZE, characterUUID},
           GATT_PERMIT_READ,
           0,
           &beaconsListRssiChar
      },
      {
           {ATT_UUID_SIZE, beaconsListRssiUUID},
           GATT_PERMIT_READ,
           0,
           &beaconsListRssiValue
      },
      {
           {ATT_BT_UUID_SIZE, charUserDescUUID},
           GATT_PERMIT_READ,
           0,
           beaconsListRssiCharDesc
      }
};

bStatus_t BeaconsProfileAddService(void)
{
    bStatus_t scanService = GATTServApp_RegisterService(beaconsDiscoServiceAttrTbl, GATT_NUM_ATTRS(beaconsDiscoServiceAttrTbl),
                                                        GATT_MAX_ENCRYPT_KEY_SIZE, &beaconsProfileCBs);

    bStatus_t listService = GATTServApp_RegisterService(beaconsListServiceAttTbl, GATT_NUM_ATTRS(beaconsListServiceAttTbl),
                                                        GATT_MAX_ENCRYPT_KEY_SIZE, &beaconsProfileCBs);

    if(scanService == SUCCESS && listService == SUCCESS)
        return SUCCESS;
    else
        return FAILURE;
}

bStatus_t BeaconsProfile_RegisterAppCBs(beaconsProfileCBs_t *appCallbacks)
{
    if(appCallbacks)
    {
        beaconsProfile_AppCBs = appCallbacks;

        return SUCCESS;
    }
    else
    {
        return bleAlreadyInRequestedMode;
    }
}

bStatus_t BeaconsProfile_SetParameter(uint8 param, uint8 len, void *value)
{
    bStatus_t status = SUCCESS;

    switch(param)
    {
        case BEACONS_DISCO_SCAN:
            if(len == sizeof(uint8))
            {
                beaconsDiscoScanValue = *((uint8 *) value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;
        case BEACONS_LIST_GET_RECORD:
            if(len == sizeof(uint8))
            {
                beaconsListGetRecordValue = *((uint8 *) value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;
        case BEACONS_LIST_TOTAL_COUNT:
            if(len == sizeof(uint8))
            {
                beaconsListTotalCountValue = *((uint8 *) value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;
        case BEACONS_LIST_ALL_RECORDS:
            if(len == (sizeof(beaconRecord) * DEFAULT_MAX_SCAN_RES))
            {
                memcpy(beacons, value, sizeof(beaconRecord) * DEFAULT_MAX_SCAN_RES);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;
        default:
            status = INVALIDPARAMETER;
            break;
    }

    return status;
}

bStatus_t BeaconsProfile_GetParameter(uint8 param, void *value)
{
    bStatus_t status = SUCCESS;

    switch(param)
    {
        case BEACONS_DISCO_SCAN:
            *((uint8 *) value) = beaconsDiscoScanValue;
            break;
        case BEACONS_LIST_GET_RECORD:
            *((uint8 *) value) = beaconsListGetRecordValue;
            break;
        case BEACONS_LIST_TOTAL_COUNT:
            *((uint8 *) value) = beaconsListTotalCountValue;
            break;
        case BEACONS_LIST_ALL_RECORDS:
            memcpy(value, beacons, sizeof(beaconRecord) * DEFAULT_MAX_SCAN_RES);
            break;
        default:
            status = INVALIDPARAMETER;
            break;
    }

    return status;
}

static bStatus_t beaconsProfileReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                          uint16_t maxLen, uint8_t method)
{
    bStatus_t status = SUCCESS;

    if(offset > 0)
        return ATT_ERR_ATTR_NOT_LONG;

    if(pAttr->type.len == ATT_UUID_SIZE)
    {
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);

        switch(uuid)
        {
            case BEACONS_DISCO_SCAN_UUID:
            case BEACONS_LIST_TOTAL_COUNT_UUID:
                *pLen = 1;
                pValue[0] = *pAttr->pValue;
                break;
            case BEACONS_LIST_MAC_ADDR_UUID:
                *pLen = B_ADDR_LEN;
                if(beaconsListTotalCountValue > 0)
                {
                    memcpy(pValue, beacons[beaconsListGetRecordValue].macAddr, B_ADDR_LEN);
                    /*uint8 i;
                    for(i = 0; i < B_ADDR_LEN; i++)
                    {
                        pValue[i] = beacons[beaconsListGetRecordValue].macAddr[i];
                    }*/
                }
                else
                    memcpy(pValue, pAttr->pValue, B_ADDR_LEN);
                break;
            case BEACONS_LIST_RSSI_UUID:
                *pLen = 1;
                //pValue[0] = *pAttr->pValue;
                if(beaconsListTotalCountValue > 0)
                    pValue[0] = beacons[beaconsListGetRecordValue].rssi;
                else
                    pValue[0] = *pAttr->pValue;
                break;
            default:
                *pLen = 0;
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
    }

    return status;
}

static bStatus_t beaconsProfileWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset,
                                           uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8 notifyApp = 0xFF;

    if(pAttr->type.len == ATT_UUID_SIZE)
    {
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);

        switch(uuid)
        {
            case BEACONS_DISCO_SCAN_UUID:
                if(offset == 0)
                {
                    if(len != 1)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                if(status == SUCCESS && pValue[0] > 0)
                {
                    uint8 *pCurValue = (uint8 *)pAttr->pValue;
                    *pCurValue = pValue[0];

                    notifyApp = BEACONS_DISCO_SCAN;
                }
                break;
            case BEACONS_LIST_GET_RECORD_UUID:
                if(offset == 0)
                {
                    if(len != 1)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                if(status == SUCCESS)
                {
                    if(pValue[0] < beaconsListTotalCountValue)
                    {
                        uint8 *pCurValue = (uint8 *)pAttr->pValue;
                        *pCurValue = pValue[0];
                    }
                }
                break;
            default:
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        status = ATT_ERR_INVALID_HANDLE;
    }

    if(notifyApp != 0xFF && beaconsProfile_AppCBs && beaconsProfile_AppCBs->pfnBeaconsProfileChange)
    {
        beaconsProfile_AppCBs->pfnBeaconsProfileChange(notifyApp);
    }

    return status;
}


