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
int output_report(hid_device *handle, unsigned char *pdata)
{
	unsigned char buf[BUFFLENGTH];
	int res;
	const size_t size = 32 + 5;

#ifdef DEBUG_FRAMES
	wprintf(L"sending:\n");
	memcpy(&buf[0], pdata, size);
	for (int i = 0; i < size; i++)
	{
		wprintf(L"%d 0x%02x\n", i, buf[i]);
	}
#endif
	res = hid_write(handle, buf, size);
	return res;
}

/**
 * @brief Perform a write or read on the chip I2C bus
 * 
 * @param handle 
 * @param pdata 
 * @param size 
 * @return int 
 */
int HID_EXAR_I2C_out(hid_device *handle, unsigned char *pdata, size_t wrsize, size_t rdsize)
{
	const unsigned char i2c_device_address = 0xA0;
	unsigned char buf[BUFFLENGTH];
	int res;

	memset(buf, 0, sizeof(buf));
	buf[0] = 0x0;       // Report ID read or write
	buf[1] = STARTBIT | STOPBIT | 0xE0;  // Flags + sequence number
    buf[2] = wrsize;// > 0? 32 : 0;  // Bytes to write
    buf[3] = rdsize > 0? 32 : 0;  // Bytes to read
	buf[4] = i2c_device_address;
#ifdef DEBUG_FRAMES
	wprintf(L"writing %d reading %d\n", wrsize, rdsize);
#endif
	memcpy(&buf[5], pdata, wrsize); // Data
	res = output_report(handle, buf);
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
	int res;

	memset(buf, 0, sizeof(buf));
	buf[0] = (eeprom_address >> 8) & 0xFF; // epprom address HI
	buf[1] = (eeprom_address >> 0) & 0xFF; // epprom address LO
	memcpy(&buf[2], pdata, wrsize); // Data
	res = HID_EXAR_I2C_out(handle, buf, wrsize + 2, rdsize);
	return res;
}

/**
 * @brief Read the 24C01 EEPROM using the EXAR HID interface
 *        Always reading the 32bytes of a page
 *        p32 of the datasheet
 * 
 * @param address    I2C address
 * @param pdata      data to be written
 * @return int -1 if failure, number of written bytes
 */
int eeprom_page_read(hid_device *handle, unsigned short eeprom_address, unsigned char *pdata)
{
	unsigned char buf[BUFFLENGTH];
	int res;
	size_t wrsize, rdsize;

	wprintf(L"Write the start address\n");
	wrsize = 0;
	rdsize = 0;
	res = eeprom_page_write(handle, eeprom_address, buf, wrsize, rdsize);
	wprintf(L"Write the start address returned %d\n", res);
	if (res >= 0)
	{
		// Read the return data
		memset(buf, 0, sizeof(buf));
		rdsize = 32;
		res = HID_EXAR_I2C_out(handle, buf, wrsize, rdsize);
		for (int i = 0; i < 32; i++)
		{
			wprintf(L"%d 0x%02x\n", i, buf[i]);
		}
	}
	return res;
}

/**
 * @brief Reads the 24C01 EEPROM using the EXAR HID interface
 *        p32 of the datasheet
 * 
 * @param address 
 * @param data 
 * @return int -1 if failure, number of received bytes otherwise
 */
int eeprom_read_response(hid_device *handle, unsigned char *pdata, size_t len)
{
	unsigned char statusflags;
	unsigned char wrbytes;
	unsigned char rdbytes;
	unsigned char buf[BUFFLENGTH];
	int res;

	res = hid_read_timeout(handle, pdata, len, 500);
	if (res > 4)
	{
		statusflags = buf[0];
		wrbytes = buf[1];
		rdbytes = buf[2];
		wprintf(L"Status 0x%02X Written %d read %d\n", statusflags, wrbytes, rdbytes);
	}
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
#ifdef WRITE_NUMERS
	wprintf(L"Writing 1 2 3 4...\n");
	for (int i = 0; i < 32; i++)
	{
		data[i] = i;
	}
	wrsize = 30; // Maximum length
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
	eeprom_address = 0;
	res = eeprom_page_read(handle, eeprom_address, data);
	wprintf(L"eeprom_page_read returned %d\n", res);
#ifdef WRITE_NUMERS
	rdsize = 32;
	res = eeprom_read_response(handle, data, rdsize);
	for (int i = 0; i < 32; i++)
	{
		wprintf(L"D%d=0x%x\n", i, data[i]);
	}
#else
	rdsize = 32;
	res = eeprom_read_response(handle, data, rdsize);
	data[42] = 0;
	wprintf(L"eeprom_read_response returned %d \"%s\"\n", res, data);
	for (int i = 0; i < 32; i++)
	{
		wprintf(L"D%d=0x%x\n", i, data[i]);
	}
#endif
}

/**
 * @brief EDGE interface
 * 
 * @param address 
 * @param data 
 * @return int 
 */
int hid_edge_write_register(hid_device *handle, u_int16_t address, u_int16_t data)
{
	unsigned char buf[BUFFLENGTH];
	int res;

	wprintf(L"Write address 0x%04x\n", address);
	buf[0] = 0x3C;                  // Write HID register
	buf[1] = (address & 0xFF);      // Write address LSB
    buf[2] = (address >> 8) & 0xFF; // Write address MSB
	buf[3] = (data & 0xFF);         // Write data LSB
    buf[4] = (data >> 8) & 0xFF;    // Write data MSB
	res = hid_send_feature_report(handle, buf, 5);
	wprintf(L"wr result res=%d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3], buf[4]);
	return res;
}

/**
 * @brief EDGE read address
 * 
 * @return int 
 */
int hid_edge_set_read_address(hid_device *handle, u_int16_t address)
{
	unsigned char buf[BUFFLENGTH];
	int res;
	
	wprintf(L"set read address 0x%04x\n", address);
	buf[0] = 0x4B;                  // Set address for HID register read
	buf[1] = (address & 0xFF);      // Write address LSB
    buf[2] = (address >> 8) & 0xFF; // Write address MSB
	buf[3] = 0;
	//wprintf(L"set read @ wr 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3]);
	res = hid_send_feature_report(handle, buf, 3);
	wprintf(L"set read @ result res=%d 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3]);
	return res;
}

/**
 * @brief EDGE read register @address
 * 
 * @return int 
 */
int hid_edge_read_register(hid_device *handle)
{
	unsigned char buf[BUFFLENGTH];
	int res;
	int milliseconds = 100;

	int  nonblock = 1;
	//hid_set_nonblocking(handle, nonblock);
	wprintf(L"read\n");
	buf[0] = 0x5A;    // Read HID register
	buf[1] = 0x00;
    buf[2] = 0x00;
	//res = hid_send_feature_report(handle, buf, 3);
	//wprintf(L"wr result res=%d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3], buf[4]);
	res = hid_get_feature_report(handle, buf, 3);
	wprintf(L"-> read result res=%d 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2]);
	return res;
}

#define EDGE_FUNC_SEL_ADDR 0x3C0
#define EDGE_DIR_ADDR 0x3C1
#define EDGE_SET_ADDR 0x3C2
#define EDGE_CLEAR_ADDR 0x3C3
#define EDGE_PWM0_CTRL_ADDR 0x3D8
#define EDGE_PWM0_HIGH_ADDR 0x3D9
#define EDGE_PWM0_LOW_ADDR  0x3DA
#define EDGE_PWM1_CTRL_ADDR 0x3DB
#define EDGE_PWM1_HIGH_ADDR 0x3DC
#define EDGE_PWM1_LOW_ADDR  0x3DD

int read_reg(hid_device *handle, u_int16_t address)
{
	hid_edge_set_read_address(handle, address);
	hid_edge_read_register(handle);
}

int initPWM(hid_device *handle)
{
	u_int16_t pwm_mode;

	// Enable GPIO1 and 2 Edge
	hid_edge_write_register(handle, EDGE_FUNC_SEL_ADDR, 0b000000011);
	// Output direction
	hid_edge_write_register(handle, EDGE_DIR_ADDR, 0b000000011);
	// Set to Zero
	hid_edge_write_register(handle, EDGE_CLEAR_ADDR, 0b000000011);
	sleep(1);
	// Set to one
	hid_edge_write_register(handle, EDGE_SET_ADDR,   0b000000011);
	//return 0;
	sleep(1);
	//pwm_mode = 0b0000 0001 1010 0000;
	pwm_mode = 0x1A0;
	// 1400 2000
	// PWM0 freq 1500hz, 266.667ns steps
	hid_edge_write_register(handle, EDGE_PWM0_HIGH_ADDR, 1250);
	hid_edge_write_register(handle, EDGE_PWM0_LOW_ADDR,  1250);
	// Use E0 as PWM0 output in free run mode
	hid_edge_write_register(handle, EDGE_PWM0_CTRL_ADDR, pwm_mode);
	pwm_mode = 0x1A1;
	// PWM1 freq 1500hz, 266.667ns steps
	hid_edge_write_register(handle, EDGE_PWM1_HIGH_ADDR, 1250);
	hid_edge_write_register(handle, EDGE_PWM1_LOW_ADDR,  1250);
	// Use E1 as PWM1 output in free run mode
	hid_edge_write_register(handle, EDGE_PWM1_CTRL_ADDR, pwm_mode);
	return 0;
}

void open_EDGE_HID()
{
	// Exar XR22804
	unsigned short VID = 0x04E2;
	unsigned short EDGE_PID = 0x1200;
	wchar_t wstr[MAX_STR];
	int res;
	hid_device *handle;

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(VID, EDGE_PID, NULL);
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
	// Do a register read
	//
	wprintf(L"Init pwm\n");
	initPWM(handle);
	return ;
	wprintf(L"Reading\n");
	read_reg(handle, 0x3C0);
	exit(0);

	for (int addr = 0x3C0; addr < 0x3DD; addr++)
	{
		read_reg(handle, addr);
	}
	//exit(0);
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
	// List the HID devices
	plist = hid_enumerate(VID, 0);
	while (plist != NULL)
	{
		wprintf(L"%ls, %ls, 0x%x 0x%x\n", plist->manufacturer_string, plist->product_string, plist->vendor_id, plist->product_id);
		plist = plist->next;
	}
	// Toggles E0 and E1 and sets the PWM output at 1500hz on them.
	//open_EDGE_HID();

	// Writes and reads to/from an EEPROM
	open_I2C_HID();
	return 0;
}

