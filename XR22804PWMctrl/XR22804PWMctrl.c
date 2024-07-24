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
#include <hidapi/hidapi.h>

#define MAX_STR 512
#define BUFFLENGTH 64

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

	buf[0] = 0x3C;                  // Write HID register
	buf[1] = (address & 0xFF);      // Write address LSB
    buf[2] = (address >> 8) & 0xFF; // Write address MSB
	buf[3] = (data & 0xFF);         // Write data LSB
    buf[4] = (data >> 8) & 0xFF;    // Write data MSB
	res = hid_send_feature_report(handle, buf, 5);
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
	
	buf[0] = 0x4B;                  // Set address for HID register read
	buf[1] = (address & 0xFF);      // Write address LSB
    buf[2] = (address >> 8) & 0xFF; // Write address MSB
	buf[3] = 0;
	//wprintf(L"set read @ wr 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3]);
	res = hid_send_feature_report(handle, buf, 3);
	//wprintf(L"set read @ result res=%d 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3]);
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
	//wprintf(L"read\n");
	buf[0] = 0x5A;    // Read HID register
	buf[1] = 0x00;
    buf[2] = 0x00;
	//res = hid_send_feature_report(handle, buf, 3);
	//wprintf(L"wr result res=%d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2], buf[3], buf[4]);
	res = hid_get_feature_report(handle, buf, 3);
	//wprintf(L"-> read result res=%d 0x%02x 0x%02x 0x%02x\n", res, buf[0], buf[1], buf[2]);
	return res;
}

int read_reg(hid_device *handle, u_int16_t address)
{
	hid_edge_set_read_address(handle, address);
	hid_edge_read_register(handle);
}

/**
 * @brief Configures a GPIO as output and sets his value to 0 or 1
 * 
 * @param handle 
 * @param pwmChannel 
 * @param benabled 
 * @return int 
 */
int initPWM(hid_device *handle, int pwmChannel, int enabled, int value)
{
    u_int16_t pwm_mask;
    u_int16_t clear_mask, set_mask;

    if (pwmChannel > 1)
    {
        return 1;
    }
    pwm_mask = (enabled? 1 : 0) << (pwmChannel);
    set_mask = clear_mask = 0;
    if (value)
    {
        set_mask = 1 << pwmChannel;
    }
    else
    {
        clear_mask = 1 << pwmChannel;
    }
	// Enable GPIO Edge function
	hid_edge_write_register(handle, EDGE_FUNC_SEL_ADDR, pwm_mask);
	// Output direction
	hid_edge_write_register(handle, EDGE_DIR_ADDR, pwm_mask);
	// Set to One
	hid_edge_write_register(handle, EDGE_SET_ADDR, set_mask);
	// Clear to Zero
	hid_edge_write_register(handle, EDGE_CLEAR_ADDR, clear_mask);
	return 0;
}

/**
 * @brief Converts a frequency to a period
 * 
 * @param pwmFreq 
 * @return int 
 */
double freq2time(double pwmFreq, double duty_cycle)
{
    double period_ns;
    const double mintime_ns = 266.667L;
    double res;

    period_ns =  duty_cycle * 1000000000.0L / (pwmFreq);
    res = (period_ns / mintime_ns);
    return res;
}

/**
 * @brief Starts the PWM
 * 
 * @param handle 
 * @param pwm1Freq Frequency to emit < 1010 if not used
 * @param pwm2Freq 
 * @return int
 */
int programPWM(hid_device *handle, int FreqHertz, int channel, double duty_cycle)
{
    u_int16_t pwm_mode;
    int high_period;
    int low_period;

    // freq 1500hz, 266.667ns steps
    high_period = freq2time((double)FreqHertz, duty_cycle);
    low_period = freq2time((double)FreqHertz, 1.0 - duty_cycle);
    if (channel == 0)
    {
        pwm_mode = 0x1A0;
        //wprintf(L"Prog channel 0 with freq hz: %d to freq is %d\n", FreqHertz, freq);
        hid_edge_write_register(handle, EDGE_PWM0_HIGH_ADDR, high_period);
        hid_edge_write_register(handle, EDGE_PWM0_LOW_ADDR,  low_period);
        // Use E0 as PWM0 output in free run mode
        hid_edge_write_register(handle, EDGE_PWM0_CTRL_ADDR, pwm_mode);
    }
    if (channel == 1)
    {
        pwm_mode = 0x1A1;
        // PWM1 freq 1500hz, 266.667ns steps
        hid_edge_write_register(handle, EDGE_PWM1_HIGH_ADDR, high_period);
        hid_edge_write_register(handle, EDGE_PWM1_LOW_ADDR,  low_period);
        // Use E1 as PWM1 output in free run mode
        hid_edge_write_register(handle, EDGE_PWM1_CTRL_ADDR, pwm_mode);
    }
    return 0;
}

hid_device *open_EDGE_HID()
{
	// Exar XR22804
	const unsigned short VID = 0x04E2;
	const unsigned short EDGE_PID = 0x1200;
	wchar_t wstr[MAX_STR];
    hid_device *handle;
	int res;

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(VID, EDGE_PID, NULL);
	if (handle == NULL)
	{
        wprintf(L"Error on hid interface: NULL handle\n");
	}
    else
    {
        // Read the Manufacturer String
        res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
        wprintf(L"Manufacturer String: %ls\n", wstr);
    }
    return handle;
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
    char *pend;

    if (argc < 2)
    {
        wprintf(L"Usage:\nXR22804PWMctrl -p GPIOCHANNEL ENABLED VALUE\nor \"XR22804PWMctrl -f FREQUENCY GPIOPORT DUTYCLE_0_TO_100\", frequency in Hertz\n");
        wprintf(L"example to set the first PWM as output to zero: XR22804PWMctrl -p 0 1 0\nto output a 1500hz frequency (1010hz to 3Mhz): XR22804PWMctrl -f 1500 1 50\n");
        wprintf(L"GPIOCHANNEL is 0 or 1.\n");
    }
    else
    {
        // Initialize the hidapi library
        res = hid_init();
        if (res != -0)
        {
            wprintf(L"Could not open the hid interface.\n");
            return 1;
        }
        hid_device *handle = open_EDGE_HID();
        if (handle == NULL)
        {
            wprintf(L"Device handle error.\n", argc);
        }
        else if (strcmp(argv[1], "-p") == 0 && (argc == 5) && handle)
        {
            int channel = strtol(argv[2], &pend, 10);
            int enabled = strcmp(argv[3], "1") == 0;
            int value = strcmp(argv[4], "1") == 0;
            initPWM(handle, channel, enabled, value);
        }
        else if (strcmp(argv[1], "-f") == 0 && (argc >= 5) && handle)
        {
            int freq = strtol(argv[2], &pend, 10);
            int channel = strtol(argv[3], &pend, 10);
            double dutycyle = (double)(strtol(argv[4], &pend, 10)) / 100.0;
            wprintf(L"Programming frequency %d at duty cycle %f on channel %d.\n", freq, dutycyle, channel);
            if (channel >= 0 && channel <= 1)
            {
                programPWM(handle, freq, channel, dutycyle);
            }
        }
        else
        {
            wprintf(L"Argument error (argc=%d).\n", argc);
        }
    }
	return 0;
}
