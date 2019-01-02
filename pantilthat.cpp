/*  
    C++ interface for controlling a Pimoroni Pan-Tilt HAT device on a Raspberry Pi 3 B+.
    Class built upon SMBus and i2C standard APIs.

    Works with reference PIM183 (https://shop.pimoroni.com/products/pan-tilt-hat?variant=22408353287) ONLY.
    Available in France at https://www.kubii.fr/cartes-extension-cameras-raspberry-pi/1873-pan-tilt-hat-kit-kubii-3272496007277.html

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_cp.sh for building details.

    Copyright (C) 2019 Regis Corbel (regis dot corbel at wanadoo dot fr)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include <sys/wait.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include "smbus.h"
#include "pantilthat.h"

using namespace std;

namespace pantilthatns {

    // Constructor
    pantilthat::pantilthat()
    {
    }
    // Destructor
    pantilthat::~pantilthat()
    {
	if (_i2c != 0)
	{
            _enable_servos[0] = 0;
            _enable_servos[1] = 0;
            set_config();
	    close(_i2c);
	}
    }

    // Initial setup. Must be called first.
    bool pantilthat::setup()
    {
	if (_is_setup) return true;
	// i2c access
	if (_i2c == 0)
	{
	    if ((_i2c = open("/dev/i2c-1", O_RDWR)) < 0)
	    {
		cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to open the i2c bus." << endl;
		return false;
	    }
	    if (ioctl(_i2c, I2C_SLAVE, _i2c_address) < 0)
	    {
		cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to access the i2c slave at 0x15." << endl;
		return false;
	    }
	    // Check functionalities
	    unsigned long funcs;
	    if (ioctl(_i2c, I2C_FUNCS, &funcs) < 0)
	    {
		cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
			<< ": could not get the adapter functionality matrix (errno " 
			<< strerror(errno) << ")."
			<< endl;
		return false;
	    }
	    if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE)) {
		cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
			<< MISSING_FUNC_1
			<< "SMBus receive byte" << MISSING_FUNC_2 << endl;
		return false;
	    }
	    if (!(funcs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
                cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
                        << MISSING_FUNC_1
                        << "SMBus send byte" << MISSING_FUNC_2 << endl;
                return false;
	    }
	    if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
                cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
                        << MISSING_FUNC_1
                        << "SMBus read byte" << MISSING_FUNC_2 << endl;
                return false;
	    }
	    if (!(funcs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
                cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
                        << MISSING_FUNC_1
                        << "SMBus read word" << MISSING_FUNC_2 << endl;
                return false;
	    }
	}

	clear();
	set_config();
	_is_setup = true;
	return true;
    }

    // Set the idle timeout for the servos
    void pantilthat::set_idle_timeout(char value)
    {
        /* Configure the time, in seconds, after which the servos will be automatically disabled.
        param value: Timeout in seconds */
        _idle_timeout = value;
    }

    // Generate config value for PanTilt HAT and write to device.
    void pantilthat::set_config()
    {
        int config = 0;
        config |= _enable_servos[0];
        config |= (_enable_servos[1] << 1);
        config |= (_enable_lights << 2);
        config |= (_light_mode    << 3);
        config |= (_light_on      << 4);
        if (!i2c_write_byte(REG_CONFIG, config))
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to write config." << endl;
    }

    // Check bounds of an int value.
    bool pantilthat::check_int_range(int value, int value_min, int value_max)
    {
        return ((value >= value_min) && (value <= value_max));
    }

    // Converts pulse time in microseconds to degrees
    int pantilthat::servo_us_to_degrees(int us, int us_min, int us_max)
    {
        // param us: Pulse time in microseconds
        // param us_min: Minimum possible pulse time in microseconds
        // param us_max: Maximum possible pulse time in microseconds
        if (!check_int_range(us, us_min, us_max))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": pulse time out of range." << endl;
	    return 0;
	}
        int servo_range = (us_max - us_min);
        float angle = (float(us - us_min) / float(servo_range)) * 180.0;
        return int(round(angle)) - 90;
    }

    // Converts degrees into a servo pulse time in microseconds
    int pantilthat::servo_degrees_to_us(int angle, int us_min, int us_max)
    {
        // param angle: angle in degrees from -90 to 90
        // param us_min: Minimum possible pulse time in microseconds
        // param us_max: Maximum possible pulse time in microseconds  
	if (!check_int_range(angle, -90, 90))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": angle out of range." << endl;
	    return 0;
	}
        angle += 90;
        int servo_range = (us_max - us_min);
        float us = (servo_range / 180.0) * angle;
        return us_min + int(us);
    }

    // Get the min and max range values for a servo
    bool pantilthat::servo_range(int servo_index, int * min, int * max)
    {
	if ((servo_index != 1) && (servo_index != 2))
	    return false;
	if ((min == NULL) || (max == NULL))
	    return false;
	*min = _servos[servo_index -1].valmin;
	*max = _servos[servo_index -1].valmax;
	return true;
    }

    // Write a block of data to the i2c device
    bool pantilthat::i2c_write_block(int daddr, unsigned char * data, int len)
    {
        for (int i = 0; i < _i2c_retries; i++)
        {
	    if (i2c_smbus_write_i2c_block_data(_i2c, daddr, len, data) == len) return true;
            lsleep(_i2c_retry_time);
        }
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to write block to the i2c device." << endl;
        return false;
    }

    // Write one word to the i2c device
    bool pantilthat::i2c_write_word(int reg, uint16_t w)
    {
	if (!check_int_range(w, 0, 0xFFFF)) return false;
        for (int i = 0; i < _i2c_retries; i++)
        {
	    if (i2c_smbus_write_word_data(_i2c, reg, w) >= 0) return true;
            lsleep(_i2c_retry_time);
        }
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to write word to the i2c device." << endl;
        return false;
    }

    // Write one byte to the i2c device
    bool pantilthat::i2c_write_byte(int reg, unsigned char c)
    {
	if (!check_int_range(c, 0, 0xFF)) return false;
        for (int i = 0; i < _i2c_retries; i++)
        {
            if (i2c_smbus_write_byte_data(_i2c, reg, c) >= 0) return true;
            lsleep(_i2c_retry_time);
        }
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to write byte to the i2c device." << endl;
        return false;
    }

    // Read one byte from the i2c device
    bool pantilthat::i2c_read_byte(int reg, unsigned char * c)
    {
	int res = -1;
        for (int i = 0; i < _i2c_retries; i++)
	{
	    if ((res = i2c_smbus_read_byte_data(_i2c, reg)) >= 0)
	    {
		*c = (res & 0xFF);
		return true;
	    }
            lsleep(_i2c_retry_time);
	}
	cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to read byte from the i2c device." << endl;
	return false;
    }

    // Read one word from the i2c device
    bool pantilthat::i2c_read_word(int reg, uint16_t * w)
    {
	int res = -1;
        for (int i = 0; i < _i2c_retries; i++)
        {
	    if ((res = i2c_smbus_read_word_data(_i2c, reg)) >= 0)
	    {
		*w = (res & 0xFFFF);
		return true;
	    }
            lsleep(_i2c_retry_time);
	}
	cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to read word from the i2c device." << endl;
	return false;
    }

    // Clear the pixel buffer
    void pantilthat::clear()
    {
	memset(_pixels, 0, 96);
    }

    // Set the light mode for attached lights.
    void pantilthat::light_mode(unsigned int mode)
    {
        /* PanTiltHAT can drive either WS2812 or SK6812 pixels,
        or provide a PWM dimming signal for regular LEDs.
        * PWM - PWM-dimmable LEDs
        * WS2812 - 24 WS2812 or 18 SK6812 pixels*/
        setup();
        _light_mode = mode;
        set_config();
    }

    // Set the light type for attached lights.
    void pantilthat::light_type(unsigned int set_type)
    {
        /*Set the type of lighting strip connected:
        * RGB - WS2812 pixels with RGB pixel order
        * RGB - WS2812 pixels with GRB pixel order
        * RGBW - SK6812 pixels with RGBW pixel order
        * GRBW - SK6812 pixels with GRBW pixel order*/
	setup();
        _light_type = set_type;
	set_config();
    }

    // Returns the supported number of pixels depending on light mode.
    int pantilthat::num_pixels()
    {
        /* RGBW or GRBW support 18 pixels
        RGB supports 24 pixels */
	if ((_light_type == RGBW) || (_light_type == GRBW))
	    return 18;
	return 24;
    }

    // Set the brightness of the connected LED ring.
    void pantilthat::set_brightness(char brightn)
    {
        /*This only applies if light_mode has been set to PWM.
        It will be ignored otherwise.
        param brightness: Brightness from 0 to 255*/
        if (!check_int_range(brightn, 0, 255))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": brightness out of range." << endl;
	    return;
	}
        if (_light_mode == PWM)
	{
	    setup();
            // The brightness value is taken from the first register of the WS2812 chain
            i2c_write_byte(REG_WS2812, brightn);
	}
    }

    // Set all pixels in the buffer.
    void pantilthat::set_all(char red, char green, char blue, int white)
    {
        /*param red: Amount of red, from 0 to 255
        param green: Amount of green, from 0 to 255
        param blue: Amount of blue, from 0 to 255
        param white: Optional amount of white for RGBW and GRBW strips*/
	for (int index = 0; index < num_pixels(); index++)
	    set_pixel(index, red, green, blue, white);
    }

    // Set a single pixel in the buffer for GRBW lighting stick
    void pantilthat::set_pixel_rgbw(int index, char red, char green, char blue, int white)
    {
        /*index: Index of pixel from 0 to 17
        red: Amount of red, from 0 to 255
        green: Amount of green, from 0 to 255
        blue: Amount of blue, from 0 to 255
        white: Amount of white, from 0 to 255*/
        set_pixel(index, red, green, blue, white);
    }

    // Set a single pixel in the buffer.
    bool pantilthat::set_pixel(int index, char red, char green, char blue, int white)
    {
	/*
	    index: Index of pixel from 0 to 23
	    red: Amount of red, from 0 to 255
	    green: Amount of green, from 0 to 255
	    blue: Amount of blue, from 0 to 255
	    white: Optional amount of white for RGBW and GRBW strips
	*/

	// Params validation
	if (
		(!check_int_range(index, 0, num_pixels() - 1)) ||
		(!check_int_range(red, 0, 255)) ||
		(!check_int_range(green, 0, 255)) ||
		(!check_int_range(blue, 0, 255)) ||
		((white != -1) && (!check_int_range((char)(white & 0xFF), 0, 255)))
	)
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": index or colors out of range." << endl;
	    return false;
	}

        if ((_light_type == RGBW) || (_light_type == GRBW))
	{
            index *= 4;
            if (_light_type == RGBW)
	    {
                _pixels[index] = red;
                _pixels[index+1] = green;
                _pixels[index+2] = blue;
	    }
            if (_light_type == GRBW)
	    {
                _pixels[index] = green;
                _pixels[index+1] = red;
                _pixels[index+2] = blue;
	    }
            if (white != -1)
                _pixels[index+3] = (char)(white & 0xFF);
	}
        else
	{
            index *= 3;
            if (_light_type == RGB)
	    {
                _pixels[index] = red;
                _pixels[index+1] = green;
                _pixels[index+2] = blue;
	    }
            if (_light_type == GRB)
	    {
                _pixels[index] = green;
                _pixels[index+1] = red;
                _pixels[index+2] = blue;
	    }
	}
	return true;
    } // pantilthat::set_pixel()

    // Display the buffer on the connected WS2812 strip.
    void pantilthat::show()
    {
        setup();
	// TO BE DONE
        /*i2c_write_block(self.REG_WS2812, self._pixels[:32]);
        i2c_write_block(self.REG_WS2812 + 32, self._pixels[32:64]);
        i2c_write_block(self.REG_WS2812 + 64, self._pixels[64:]);
        i2c_write_byte(self.REG_UPDATE, 1);*/
    }

    // Enable or disable a servo
    bool pantilthat::servo_enable(char index, unsigned char state)
    {
        /*Disabling a servo turns off the drive signal.

        It's good practise to do this if you don't want
        the Pan/Tilt to point in a certain direction and
        instead want to save power.

        index: Servo index: either 1 or 2
        state: Servo state: 1 = on, 0 = off*/

        setup();
        if ((index < 1) || (index > 2) ||
	    (state > 1))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": invalid parameters." << endl;
	    return false;
	}
	_enable_servos[index - 1] = state;
        set_config();
	return true;
    }

    // Is a servo enabled ?
    bool pantilthat::is_servo_enabled(char index, unsigned char  * state)
    {
	setup();
        if ((index < 1) || (index > 2) || (state == NULL))
        {
            cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": invalid parameters." << endl;
            return false;
        }
	*state = _enable_servos[index - 1];
	return true;
    }

    // Set the minimum high pulse for a servo in microseconds.
    bool pantilthat::servo_pulse_min(char index, unsigned int value)
    {
        // param value: Value in microseconds
        if ((index < 1) || (index > 2))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": servo index must be 1 or 2." << endl;
	    return false;
	}
	if (value >= servominmaxvalues[(index == 1 ? 0 : 2)])
	{
            _servos[index - 1].valmin = value;
	    return true;
	}
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
            << ": invalid pulse min value." << endl;
        return false;
    }

    // Set the maximum high pulse for a servo in microseconds.
    bool pantilthat::servo_pulse_max(char index, unsigned int value)
    {
        // param value: Value in microseconds
        if ((index < 1) || (index > 2))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": servo index must be 1 or 2." << endl;
	    return false;
	}
	if (value <= servominmaxvalues[(index == 1 ? 1 : 3)])
	{
            _servos[index - 1].valmax = value;
	    return true;
	}
	cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
	    << ": invalid pulse max value." << endl;
	return false;
    }

    // Get position of servo 1 or 2 in degrees
    int pantilthat::get_servo(char index)
    {
        if ((index < 1) || (index > 2))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": servo index must be 1 or 2." << endl;
	    return 0;
	}
	setup();
	int us_min = 0, us_max = 0;
	if (!servo_range(index, &us_min, &us_max))
	    return 0;
	uint16_t us = 0;
	i2c_read_word((index == 1 ? REG_SERVO1 : REG_SERVO2), &us);
       	return (servo_us_to_degrees(us, us_min, us_max));
    }

    // Set position of servo 1 or 2 in degrees.
    void pantilthat::set_servo(char index, int angle)
    {
	// param index : servo 1 or servo 2
        // param angle : angle in degrees from -90 to 90
        if ((index < 1) || (index > 2) ||
	    (angle < -90) || (angle > 90))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": invalid parameters." << endl;
	    return;
	}
        setup();
	if (_enable_servos[index - 1] == 0)
	{
	    _enable_servos[index - 1] = 1;
	    set_config();
	}
	int us_min, us_max;
	servo_range(index, &us_min, &us_max);
        int us = servo_degrees_to_us(angle, us_min, us_max);
        if (!i2c_write_word((index == 1 ? REG_SERVO1 : REG_SERVO2), us))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << "i2c_write_word() failed." << endl;
	    return;
	}
        if (_idle_timeout > 0)
	{
	    // Make this thread too slow
	    //lsleep(_idle_timeout);
	    // Apparently prevents the servos from running
	    //servo_stop(index);
	}
    }

    // Stop a servo
    // Commented out since it prevents the servos from running.
    // May be useful someday.
    /*bool pantilthat::servo_stop(char index)
    {
        if ((index < 1) || (index > 2))
	{
	    cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": servo index must be 1 or 2." << endl;
	    return false;
	}
	_enable_servos[index - 1] = 0;
        set_config();
	return true;
    }*/

    // Suspend this thread for ms milliseconds.
    int pantilthat::lsleep(long int ms)
    {
        int result = 0;
        struct timespec ts_sleep, ts_remaining;
	ts_remaining.tv_sec = (time_t)(ms / 1000);
	ts_remaining.tv_nsec = (long)(ms % 1000) * 1000000;
        do
        {
	    ts_sleep = ts_remaining;
            result = nanosleep(&ts_sleep, &ts_remaining);
        }
        while ((errno == EINTR) && (result == -1));
        if (result == -1)
        {
            cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << " nanosleep() failed" << endl;
        }
        return result;
    }

} //namespace pantilthatns
