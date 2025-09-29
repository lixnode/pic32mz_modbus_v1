/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

#include "../pic32mz_modbus_rtu.X/modbus.h"
#include "../pic32mz_modbus_rtu.X/config.h"


int main ( void )
{
    int rc;
    uint8_t req[MODBUS_MAX_ADU_LENGTH];
    
    modbus_t *ctx = NULL;
    modbus_mapping_t *mb_mapping = NULL;
    
    /* Initialize all modules */
    SYS_Initialize ( NULL );
      
    ctx = modbus_new_rtu(MODBUS_RTU_DEVICE, 115200, 'N', 8, 1);
    modbus_set_slave(ctx, MODBUS_DEFAULT_SLAVE_ID);
    mb_mapping =
        modbus_mapping_new(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);
    if (mb_mapping == NULL) {
        printf("Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        return -1;
    }
    
    printf("System initialize OK\r\n");
    while ( true )
    {
        rc = modbus_available(ctx);
        if (rc > 0)
        {
            memset(req, 0, MODBUS_MAX_ADU_LENGTH);
            rc = modbus_receive(ctx, req);
            if (rc > 0)
            {
                rc = modbus_reply(ctx, req, rc, mb_mapping);
            }
        }
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

