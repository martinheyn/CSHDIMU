/*
 * ADIS_convert.cpp
 *
 *  Created on: Aug 18, 2015
 *      Author: developer
 */

#include <inc/ADIS_HD_operations.h>
#include "inc/INIT.h"

void adis_hd_init(uint8_t adis_hd_commands[]){

	uint16_t adis_hd_dataread_cmds[] = {
		PROD_ID,
		DIAG_STAT,
		BAROM_OUT,

		X_ACCL_OUT,
		Y_ACCL_OUT,
		Z_ACCL_OUT,

		X_GYRO_OUT,
		Y_GYRO_OUT,
		Z_GYRO_OUT,

		X_MAGN_OUT,
		Y_MAGN_OUT,
		Z_MAGN_OUT,

		TEMP_OUT,

		PROD_ID // each entry requires 2 x 8 bit (1 byte) => length of data buffer must be 11 x 2 byte = 22!!!!!
	};

	adis_hd_16bit_commands_to_char(adis_hd_dataread_cmds, adis_hd_commands, 14); // Convert commands to weird ADIS style

}

void adis_hd_16bit_commands_to_char(uint16_t adis_hd_commands[], uint8_t char_hd_commands[], size_t adis_hd_commands_size) {

    uint i;

    for(i=0;i<adis_hd_commands_size;i++) {
        char_hd_commands[2*i] = (adis_hd_commands[i] >> 8) & 0xFF;
        char_hd_commands[2*i+1] = adis_hd_commands[i] & 0xFF;
    }

}

void adis_hd_read_spi(BlackLib::BlackSPI* adis_spi, uint8_t* hd_commands, uint8_t* hd_data, bool isOpened_spi){

	//isOpened_spi = adis_spi->open(BlackLib::NonBlock);

	if ( !isOpened_spi)
	{
		cout << "I cannot open the SPI device. Help!" << endl;
		exit(1);
	}
	/*else
	{
	    cout << "Device Path   : " << adis_spi->getPortName() << endl;
	    cout << "Max Speed(Hz) : " << adis_spi->getMaximumSpeed() << endl;
	    cout << "Bits Per Word : " << (int)adis_spi->getBitsPerWord() << endl;
	    cout << "Mode          : " << (int)adis_spi->getMode() << endl;
	}*/

	adis_spi->transfer(hd_commands, hd_data, ADIS_HD_BUFFER_SIZE__, 0);

}

void adis_hd_extract_message(uint8_t* hd_data, double* hd_output) {

	// Conversion of measurements
	double accl_res_conv_ = 0.8;
	double gyro_res_conv_ = 0.02;
	//double std_gravity_ = 9.80665;
	//double millig_to_accl_ = std_gravity_/1000;
	double magn_res_conv_ = 0.1;
	double baro_res_conv_ = 40;
	double temp_res_conv_ = 0.00565;
	double temp_zero_ = 25.0;

	int16_t prod_id = 0;
	int16_t diag = 0;
	int16_t baro_int = 0;
	int16_t accl_x_int, accl_y_int, accl_z_int = 0;
	int16_t gyro_x_int, gyro_y_int, gyro_z_int = 0;
	int16_t magn_x_int, magn_y_int, magn_z_int = 0;
	int16_t temp_int = 0;

	//double temp_x, temp_y, temp_z, gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z;
	uint16_t adis_hd_dataread_cmds[] = {
			PROD_ID,
			DIAG_STAT,

			X_ACCL_OUT,
			Y_ACCL_OUT,
			Z_ACCL_OUT,

			X_GYRO_OUT,
			Y_GYRO_OUT,
			Z_GYRO_OUT,

			X_MAGN_OUT,
			Y_MAGN_OUT,
			Z_MAGN_OUT,

			BAROM_OUT,
			TEMP_OUT,

			PROD_ID // each entry requires 2 x 8 bit (1 byte) => length of data buffer must be 11 x 2 byte = 22!!!!!
		};
	prod_id = hd_data[3]; // the first 16 bits are random numbers!!!! AAAAAAAAAAAAAAAAH JUST WHO DOES THAT???
	prod_id = prod_id | (hd_data[2] << 8);  // And to make things easy you have to read the second set of numbers first...
	hd_output[0] = prod_id;

	diag = hd_data[5];
	diag = diag | (hd_data[4] << 8);
	hd_output[1] = diag;

	baro_int = hd_data[7];
	baro_int = baro_int | (hd_data[6] << 8);
	hd_output[2] = diag;

	accl_x_int = hd_data[9];
	accl_x_int = accl_x_int | (hd_data[8] << 8);
	hd_output[3] = accl_x_int * accl_res_conv_;

	accl_y_int = hd_data[11];
	accl_y_int = accl_y_int | (hd_data[10] << 8);
	hd_output[4] = accl_y_int * accl_res_conv_;

	accl_z_int = hd_data[13];
	accl_z_int = accl_z_int | (hd_data[12] << 8);
	hd_output[5] = accl_z_int * accl_res_conv_;

	gyro_x_int = hd_data[15];
	gyro_x_int = gyro_x_int | (hd_data[14] << 8);
	hd_output[6] = gyro_x_int * gyro_res_conv_;

	gyro_y_int = hd_data[17];
	gyro_y_int = gyro_y_int | (hd_data[16] << 8);
	hd_output[7] = gyro_y_int * gyro_res_conv_;

	gyro_z_int = hd_data[19];
	gyro_z_int = gyro_z_int | (hd_data[18] << 8);
	hd_output[8] = gyro_z_int * gyro_res_conv_;

	magn_x_int = hd_data[21];
	magn_x_int = magn_x_int | (hd_data[20] << 8);
	hd_output[9] = magn_x_int * magn_res_conv_;

	magn_y_int = hd_data[23];
	magn_y_int = magn_y_int | (hd_data[22] << 8);
	hd_output[10] = magn_y_int * magn_res_conv_;

	magn_z_int = hd_data[25];
	magn_z_int = magn_z_int | (hd_data[24] << 8);
	hd_output[11] = magn_z_int * magn_res_conv_;

	temp_int = hd_data[27];
	temp_int = temp_int | (hd_data[26] << 8);
	hd_output[12] = temp_zero_ + (temp_int * temp_res_conv_);

}

void adis_hd_display(double adis_data[]){

	cout << " prod_id: " << adis_data[0] << endl;
	cout << " system_stat: " << adis_data[1] << endl;
	cout << "barometer: " << adis_data[2] << endl;
	cout << " accl_x: " << adis_data[3] << endl;
	cout << " accl_y: " << adis_data[4] << endl;
	cout << " accl_z: " << adis_data[5] << endl;
	cout << " gyro_x: " << adis_data[6] << endl;
	cout << " gyro_y: " << adis_data[7] << endl;
	cout << " gyro_z: " << adis_data[8] << endl;
	cout << " magn_x: " << adis_data[9] << endl;
	cout << " magn_y: " << adis_data[10] << endl;
	cout << " magn_z: " << adis_data[11] << endl;
	cout << endl;
}


