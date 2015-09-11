#ifndef INC_ADIS_HD_OPERATIONS_H_
#define INC_ADIS_HD_OPERATIONS_H_

#define ADIS_HD_BUFFER_SIZE__ 26 // Buffer size is number of registers + 2 normally
#define ADIS_HD_NUMREGISTERS__ 13 // how many registers are we going to read
#define PROD_HD_ID__ 16488 // Product ID of sensor

#include "inc/INIT.h"

// Register addresses of sensors
typedef enum {

    SYS_E_FLAG = 0x0800, //
    DIAG_STAT = 0x0A00, // we take DIAG_STAT here

    BAROM_OUT = 0x3000,
	TEMP_OUT = 0x0E00,
    X_GYRO_OUT = 0x1200,
    Y_GYRO_OUT = 0x1600,
    Z_GYRO_OUT = 0x1A00,
    X_ACCL_OUT = 0x1E00,
    Y_ACCL_OUT = 0x2200,
    Z_ACCL_OUT = 0x2600,
	X_MAGN_OUT = 0x2800,
	Y_MAGN_OUT = 0x2A00,
	Z_MAGN_OUT = 0x2C00,

    PROD_ID = 0x7E00,

} adis_hd_commands;



// Forward declaration of functions
void adis_hd_init(uint8_t adis_hd_commands[]);
void adis_hd_16bit_commands_to_char(uint16_t adis_hd_commands[], uint8_t char_commands[], size_t adis_hd_commands_size);
void adis_hd_read_spi(BlackLib::BlackSPI* adis_spi, uint8_t* hd_commands, uint8_t* hd_data, bool isOpened_spi);
void adis_hd_extract_message(uint8_t* hd_data, double* hd_output);
void adis_hd_display(double adis_hd_data[]);

#endif /* INC_ADIS_HD_OPERATIONS*/
