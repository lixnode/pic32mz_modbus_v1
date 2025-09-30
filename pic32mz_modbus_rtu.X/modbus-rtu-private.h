/* 
 * File:   modbus-rtu-private.h
 * Author: thanho
 *
 * Created on September 28, 2025, 11:33 AM
 */

#ifndef MODBUS_RTU_PRIVATE_H
#define	MODBUS_RTU_PRIVATE_H


#include <stdint.h>


#define _MODBUS_RTU_HEADER_LENGTH     1
#define _MODBUS_RTU_PRESET_REQ_LENGTH 6
#define _MODBUS_RTU_PRESET_RSP_LENGTH 2

#define _MODBUS_RTU_CHECKSUM_LENGTH 2


typedef struct _modbus_rtu 
{
    char *device;
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int baud;
    /* Data bit */
    uint8_t data_bit;
    /* Stop bit */
    uint8_t stop_bit;
    /* Parity: 'N', 'O', 'E' */
    char parity;
    /* RS232, RS485 modes */
    int serial_mode;   
    int onebyte_time;
    /* To handle many slaves on the same link */
    int confirmation_to_ignore;
} modbus_rtu_t;


#endif	/* MODBUS_RTU_PRIVATE_H */

