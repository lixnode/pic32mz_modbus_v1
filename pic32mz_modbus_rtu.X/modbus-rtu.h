/* 
 * File:   modbus-rtu.h
 * Author: thanho
 *
 * Created on September 28, 2025, 11:33 AM
 */

#ifndef MODBUS_RTU_H
#define	MODBUS_RTU_H

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 */
#define MODBUS_RTU_MAX_ADU_LENGTH 256

#define MODBUS_RTU_RS232 0
#define MODBUS_RTU_RS485 1

#define MODBUS_INFORMATIVE_RX_TIMEOUT       1
#define MODBUS_DEFAULT_SLAVE_ID             0x01
#define HAVE_DECL_TIOCSRS485
#define MODBUS_RTU_HAL_UART2                   "UART2"


MODBUS_API modbus_t *modbus_new_rtu(const char *device, int baud, char parity, int data_bit, int stop_bit);


#endif	/* MODBUS_RTU_H */

