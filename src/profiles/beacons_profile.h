/*
 *
 * */

#define BEACONS_DISCO_SERVICE_UUID          0xA000
#define BEACONS_DISCO_SCAN_UUID             0xA100

#define BEACONS_LIST_SERVICE_UUID           0xB000
#define BEACONS_LIST_GET_RECORD_UUID        0xB100
#define BEACONS_LIST_TOTAL_COUNT_UUID       0xB200
#define BEACONS_LIST_MAC_ADDR_UUID          0xB300
#define BEACONS_LIST_RSSI_UUID              0xB400
#define BEACONS_LIST_AGE_UUID               0xB500
#define BEACONS_LIST_FLAG_OF_MAC_UUID       0xB600

#define BEACONS_DISCO_SCAN                  0
#define BEACONS_LIST_GET_RECORD             1
#define BEACONS_LIST_TOTAL_COUNT            2
#define BEACONS_LIST_ALL_RECORDS            3
#define BEACONS_LIST_MAC_ADDR               4

#define BEACONS_TOTAL_COUNT_LENGTH          2
#define BEACONS_AGE_OF_RECORD_LENGTH        4
#define BEACONS_MAC_ADDR_LENGTH             256

#define MAC_ADDR_NOT_FOUND                  -1


typedef void (*beaconsProfileChange_t)(uint8 paramID);

typedef struct
{
    beaconsProfileChange_t      pfnBeaconsProfileChange;

} beaconsProfileCBs_t;

typedef struct
{
    uint8 indexOfMacAddr;
    uint8 rssi;
    uint32_t discoTime;

} beaconRecord;

typedef struct
{
    uint8 macAddr[B_ADDR_LEN];

} macAddr;


extern bStatus_t BeaconsProfileAddService(void);
extern bStatus_t BeaconsProfile_SetParameter(uint8 param, uint16 len, void *value);
extern void* BeaconsProfile_GetParameter(uint8 param);
extern bStatus_t BeaconsProfile_RegisterAppCBs(beaconsProfileCBs_t *appCalbacks);
extern void BeaconsProfile_AddBeaconRecord(uint8 macAddr[B_ADDR_LEN], int8 rssi, uint32_t timestamp);
extern void BeaconsProfile_ClearMemory(void);
