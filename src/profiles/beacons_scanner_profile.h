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
#define BEACONS_LIST_AGE_OF_SCAN_UUID       0xB700

#define BEACONS_DISCO_SCAN                  0
#define BEACONS_LIST_GET_RECORD             1
#define BEACONS_LIST_TOTAL_COUNT            2
#define BEACONS_LIST_ALL_RECORDS            3
#define BEACONS_LIST_MAC_ADDR               4
#define BEACONS_LIST_FLAG_OF_MAC            5
#define BEACONS_LIST_AGE_OF_SCAN            6

#define BEACONS_SCAN_LENGTH                 2
#define BEACONS_TOTAL_COUNT_LENGTH          2
#define BEACONS_AGE_OF_RECORD_LENGTH        2
#define BEACONS_AGE_OF_SCAN_LENGTH          2

#define BEACONS_MAC_ADDR_LENGTH             256
#define BEACONS_RECORDS_LENGTH              1700

#define MAC_ADDR_NOT_FOUND                  -1


typedef void (*beaconsScannerProfileChange_t)(uint8 paramID);

typedef struct
{
    beaconsScannerProfileChange_t      pfnBeaconsScannerProfileChange;

} beaconsScannerProfileCBs_t;

typedef struct
{
    uint8 indexOfMacAddr;
    uint8 rssi;
    uint16 discoTime;

} beaconRecord;

typedef struct
{
    uint8 macAddr[B_ADDR_LEN];

} macAddr;


extern bStatus_t BeaconsScannerProfile_AddService(void);
extern bStatus_t BeaconsScannerProfile_SetParameter(uint8 param, uint16 len, void *value);
extern void* BeaconsScannerProfile_GetParameter(uint8 param);
extern bStatus_t BeaconsScannerProfile_RegisterAppCBs(beaconsScannerProfileCBs_t *appCalbacks);
extern void BeaconsScannerProfile_AddBeaconRecord(uint8 macAddr[B_ADDR_LEN], int8 rssi, uint32_t timestamp);
extern void BeaconsScannerProfile_ResetCounters(void);
