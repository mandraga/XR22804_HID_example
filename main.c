/**
 * @file main.c
 * @author Patrick Areny (patrick.areny@delair.aero)
 * @brief This program shows how to use the HID interface with the Exar XR22804 chip with Linux. EDGE gpios and PWM and the I2C bus.
 *        The customer support was mute, so I had to figure it out from the datasheet and the HID documentation.
 *        It uses the Hidapi library.
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * @licence: Apache
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "hidapi/hidapi.h"

#define MAX_STR 512

#define STARTBIT 1
#define STOPBIT  2
#define ACKBIT   4

#define BUFFLENGTH 64

#define DEBUG_FRAMES

/**
 * @brief Generic HID output report
 * 
 * @param handle 
 * @param data 
 * @return int 
 */
int output_report(hid_device *handle, unsigned char *pdata_wr, unsigned char *pdata_rd, size_t wrsize, size_t rdsize)
{
	int res;
	size_t size = 32 + 5; // Always write 32 bytes, the bytes outside of WrSize and RdSize will be ignored.
	unsigned char input[64];
	int timeout_milliseconds = 1000;

	if (wrsize != 0)
	{
		wprintf(L"sending a 32 bytes HID report:\n");
		for (int i = 0; i < size; i++)
		{
			if (i == 5)
				wprintf(L"data:\n");
			wprintf(L"%02d 0x%02x %c", i, pdata_wr[i], pdata_wr[i]);
			if (i >= 5 + wrsize)
				wprintf(L" -\n");
			else
				wprintf(L"\n");
		}
		res = hid_write(handle, pdata_wr, size);
	}
	if (rdsize != 0)
	{
		wprintf(L"reading a 32 bytes HID response report:\n");
		memset(input, 0, sizeof(input));
		size = 32 + 4; // Input report size
		res = hid_read_timeout(handle, input, size, timeout_milliseconds);
		if (res == size && input[2] == rdsize)
		{
			memcpy(pdata_rd, &input[4], rdsize);
		}
		else
		{
			wprintf(L"read of a 32 bytes HID report failed!\n");
		}
	}
	return res;
}

/**
 * @brief @brief Perform a write or read on the chip I2C bus
 * 
 * @param handle 
 * @param pdata    W
 * @param pdata_in R
 * @param wrsize
 * @param rdsize 
 * @return int 
 */
int HID_EXAR_I2C_out(hid_device *handle, unsigned char *pdata, unsigned char *pdata_in, size_t wrsize, size_t rdsize)
{
	const unsigned char i2c_device_address = 0xA0;
	unsigned char buf[BUFFLENGTH];
	int res;

	memset(buf, 0, sizeof(buf));
	buf[0] = 0x0;       // Report ID read or write
	buf[1] = STARTBIT | STOPBIT | 0xE0;  // Flags + sequence number
    buf[2] = wrsize;  // > 0? 32 : 0;  // Bytes to write
    buf[3] = rdsize;  // Bytes to read
	buf[4] = i2c_device_address;
#ifdef DEBUG_FRAMES
	wprintf(L"writing %d bytes, reading %d bytes\n", wrsize, rdsize);
#endif
	memcpy(&buf[5], pdata, wrsize); // Data
	res = output_report(handle, buf, pdata_in, wrsize, rdsize);
	return res;
}

/**
 * @brief Write the 24C01 EEPROM using the EXAR HID interface
 *        p32 of the datasheet
 * 
 * @param address    I2C address
 * @param pdata      data to be written
 * @param wrsize     writen bytes
 * @param rdsize     read bytes
 * @return int -1 if failure, number of written bytes
 */
int eeprom_page_write(hid_device *handle, unsigned short eeprom_address, unsigned char *pdata, size_t wrsize, size_t rdsize)
{
	unsigned char buf[BUFFLENGTH];
	unsigned char rd_buf[BUFFLENGTH];
	int res;

	memset(buf, 0, sizeof(buf));
	buf[0] = (eeprom_address >> 8) & 0x0F; // epprom address HI
	buf[1] = (eeprom_address >> 0) & 0xFF; // epprom address LO
	memcpy(&buf[2], pdata, wrsize); // Data
	res = HID_EXAR_I2C_out(handle, buf, rd_buf, wrsize + 2, rdsize);
	return res;
}

/**
 * @brief Reads the 24C01 EEPROM using the EXAR HID interface
 *        Always reading the 32bytes of a page
 *        p32 of the datasheet
 * 
 * @param handle 
 * @param eeprom_address 
 * @param pdata_in 
 * @return int 
 */
int eeprom_page_read(hid_device *handle, unsigned short eeprom_address, unsigned char *pdata_in)
{
	int res;
	size_t wrsize, rdsize;
	unsigned char data_out[BUFFLENGTH];

#ifdef DEBUG_FRAMES
	wprintf(L"R/W Start address @ 0x%X\n", eeprom_address);
#endif //DEBUG_FRAMES
	wrsize = 2;
	memset(data_out, 0, sizeof(data_out) / sizeof(unsigned char));
	data_out[0] = (eeprom_address >> 8) & 0x0F; // epprom address HI only the 4 lower bits are usefull
	data_out[1] = (eeprom_address >> 0) & 0xFF; // epprom address LO
	// Read the return data
	memset(pdata_in, 0, 64);
	rdsize = 32;
	res = HID_EXAR_I2C_out(handle, data_out, pdata_in, wrsize, rdsize);
#ifdef DEBUG_FRAMES
	for (int i = 0; i < 32; i++)
	{
		wprintf(L"r 0x%02x 0x%02x %c\n", eeprom_address + i, pdata_in[i], pdata_in[i]);
	}
#endif //DEBUG_FRAMES
	return res;
}

void open_I2C_HID()
{
	wchar_t wstr[MAX_STR];
	// Exar XR22804
	unsigned short VID = 0x04E2;
	// I2C bus device
	unsigned short I2C_PID = 0x1100;
	int res;
	hid_device *handle;

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(VID, I2C_PID, NULL);
	if (handle == NULL)
	{
        wprintf(L"Error on hid interface: NULL handle\n");
        return ;
	}
	// Read the Manufacturer String
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	wprintf(L"Manufacturer String: %ls\n", wstr);

	//*****************************************************************************************************************************************************************/
	//
	// Do a write on the I2C bus
	//
	unsigned char data[1024];
	size_t rdsize = 0;
	size_t wrsize = 0;
	unsigned short eeprom_address = 0;
	//
	// Do an EEPROM Write on the I2C bus
	//
//#define WRITEEEPROM
#ifdef WRITEEEPROM
#define WRITE_NUMERS
#ifdef WRITE_NUMERS
	wprintf(L"Writing 1 2 3 4...\n");
	for (int i = 0; i < 32; i++)
	{
		data[i] = i;
	}
	wrsize = 16; // Maximum length
	res = eeprom_page_write(handle, eeprom_address, data, wrsize, rdsize);
#else
	sprintf(data, "-> Test write on the EEPROM.");
	wrsize = strlen(data);
	res = eeprom_page_write(handle, eeprom_address, data, wrsize, rdsize);
#endif // WRITE_NUMERS
	wprintf(L"eeprom_page_write returned %d\n", res);
#endif // WRITEEEPROM
	//
	// Do an EEPROM Read on the I2C bus
	//
#define READ_EEPROM
#ifdef READ_EEPROM
	eeprom_address = 0;
	res = eeprom_page_read(handle, eeprom_address, data);
	wprintf(L"eeprom_page_read returned %d\n", res);
	/*
	for (int i = 0; i < 32; i++)
	{
		wprintf(L"D%d=0x%x\n", i, data[i]);
	}
	*/
#endif //READ_EEPROM
}

/**
 * @brief Must be run as root
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[])
{
	int res;
	unsigned char buf[BUFFLENGTH];
	struct hid_device_info HID_API_EXPORT* plist;
	// Exar XR22804
	unsigned short VID = 0x04E2;

	// Initialize the hidapi library
	res = hid_init();
    if (res != -0)
    {
        wprintf(L"Could not open the hid interface.\n");
        return 1;
    }

	// Writes and reads to/from an EEPROM
	open_I2C_HID();
	return 0;
}

