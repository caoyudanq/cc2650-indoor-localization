/*
 *
 * */

#include "comdef.h"
#include "tools.h"


/*
 * @fn      Tools_ArrayToBytes
 *
 * @brief   Function merges data from array to bytes.
 *
 * @param   array - pointer to an array with data
 * @param   length - length of data in bytes
 * @param   bytes - pointer to bytes where data will be stored after merge
 *
 * @return none
 *
 * */
void Tools_ArrayToBytes(uint8* array, uint8 length, void* bytes)
{

    switch(length)
    {
        case 2:
            {
                *((uint16 *) bytes) = (array[0] << 8) + array[1];
            }
            break;
        case 4:
            {
                *((uint32 *) bytes) = (array[0] << 24) + (array[1] << 16) + (array[2] << 8) + array[3];
            }
            break;
    }
}

/*
 * @fn      Tools_BytesToArray
 *
 * @brief   Function splits bytes to an array.
 *
 * @param   data - pointer to bytes which should be split
 * @param   length - length of data in bytes
 * @param   array - pointer to an array where data will be splitted
 *
 * @return none
 *
 * */
void Tools_BytesToArray(void* data, uint8 length, uint8* array)
{
    //if((length != 2 || length != 4) && length != LENGTH_OF_ARRAY(array))
       // return;

    switch(length)
    {
        case 2:
            {
                uint16 *pData = (uint16 *) data;

                array[0] = (*pData >> 8) & 0xFF;
                array[1] = *pData & 0xFF;
            }
            break;
        case 4:
            {
                uint32 *pData = (uint32 *) data;

                array[0] = (*pData >> 24) & 0xFF;
                array[1] = (*pData >> 16) & 0xFF;
                array[2] = (*pData >> 8) & 0xFF;
                array[3] = *pData & 0xFF;
            }
            break;
    }
}
