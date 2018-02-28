/*
 *
 * */

#define LENGTH_OF_ARRAY(arr)        (sizeof(arr) / sizeof(arr[0]))



extern void Tools_ArrayToBytes(uint8* array, uint8 length, void* bytes);

extern void Tools_BytesToArray(void* data, uint8 length, uint8* array);
