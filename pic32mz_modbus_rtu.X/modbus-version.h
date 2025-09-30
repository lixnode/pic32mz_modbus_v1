/* 
 * File:   modbus-version.h
 * Author: thanho
 *
 * Created on September 30, 2025, 11:07 PM
 */

#ifndef MODBUS_VERSION_H
#define	MODBUS_VERSION_H


#define LIBMODBUS_VERSION_MAJOR (0)
#define LIBMODBUS_VERSION_MINOR (0)
#define LIBMODBUS_VERSION_MICRO (1)

/* The full version as a string (for display/logging) */
#define LIBMODBUS_VERSION_STRING "0.0.1"

/* Numerically encoded version, e.g. v3.1.4 ? 0x030104 */
#define LIBMODBUS_VERSION_HEX                                                                      \
    ((LIBMODBUS_VERSION_MAJOR << 16) | (LIBMODBUS_VERSION_MINOR << 8) |                            \
     (LIBMODBUS_VERSION_MICRO << 0))

/* Check if current version >= (major.minor.micro) */
#define LIBMODBUS_VERSION_CHECK(major, minor, micro)                                               \
    (LIBMODBUS_VERSION_MAJOR > (major) ||                                                          \
     (LIBMODBUS_VERSION_MAJOR == (major) && LIBMODBUS_VERSION_MINOR > (minor)) ||                  \
     (LIBMODBUS_VERSION_MAJOR == (major) && LIBMODBUS_VERSION_MINOR == (minor) &&                  \
      LIBMODBUS_VERSION_MICRO >= (micro)))


#endif	/* MODBUS_VERSION_H */

