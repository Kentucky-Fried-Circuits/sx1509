/******************************************************************************
sx1509.cpp ported by Brian Alano, Solar Stik, Inc. from 

SparkFunSX1509.cpp 
SparkFun SX1509 I/O Expander Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: September 21, 2015
https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

Here you'll find the ESP-IDF code used to interface with the SX1509 I2C
16 I/O expander. There are functions (many not implemented) to take advantage of everything the
SX1509 provides - input/output setting, writing pins high/low, reading 
the input value of pins, LED driver utilities (blink, breath, pwm), and
keypad engine utilites.

Development environment specifics:
	IDE: Espressif ESP-IDE
	Hardware Platform: ESP32

(C) 2022 Solar Stik, Inc. Creative Commons Attribution 2.0 (CC-BY 2.0) License
******************************************************************************/

#include "sx1509.hpp"
#include "sx1509_registers.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG // this works! It even overrides the CONFIG_LOG_MAXIMUM_LEVEL_DEBUG
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

SX1509::SX1509()
{
	_clkX = 0;
}

esp_err_t SX1509::begin(i2c_dev_t *dev, uint16_t server_addr, gpio_num_t resetPin, gpio_num_t interruptPin, gpio_num_t oscillatorPin) 
{
    // TODO CHECK_ARG(dev && (
    //         (server_addr [is acceptable]))

	// We assume these are already defined in dev. FIXME there's no reason to split this up.
    // dev->port = ;
    // dev->cfg.sda_io_num = ;
    // dev->cfg.scl_io_num = ;
    // dev->cfg.master.clk_speed = ;
	_dev = dev;
    dev->addr = server_addr;
	//dev->timeout_ticks = pdMS_TO_TICKS(10); // FIXME i2cdev.h says these ticks are against an 80 KHz(?) clock, not the FreeRTOS ticks.
	ESP_RETURN_ON_ERROR(i2c_dev_create_mutex(_dev), LIBTAG, "create_mutex failed");
	ESP_RETURN_ON_ERROR(init(), LIBTAG, "init() failed");

	// Store the received parameters into member variables
	_server_addr = server_addr; // TODO don't store this separate from _dev
	pinReset = resetPin;
	pinInterrupt = interruptPin;
	pinOscillator = oscillatorPin;

    return ESP_OK;
}

esp_err_t SX1509::end() 
{
	//FIXME this should be released by the program that installed the driver
	ESP_LOGD(LIBTAG, "end()");
	return i2c_driver_delete(I2C_NUM_0); // FIXME should not be hard-coded
	//FIXME release the mutex
}

// test communication. ESP_OK if good, ESP_ERR?
esp_err_t SX1509::init()
{
	// If the reset pin is connected
	if (pinReset != GPIO_NUM_MAX)
	{
		ESP_LOGD(LIBTAG, "Hardware reset");
		reset(1);
	} else {
		ESP_LOGD(LIBTAG, "Software reset");
		reset(0);
	}
	// are you there? 
	ESP_RETURN_ON_ERROR(i2c_dev_probe(_dev, I2C_DEV_WRITE), LIBTAG, "Failed to contact server");
	ESP_LOGD(LIBTAG, "Got ACK from %x", _dev->addr);
	return ESP_OK;
}

esp_err_t SX1509::reset(bool hardware)
{
	esp_err_t ret = ESP_OK;
	// if hardware bool is set
	if (hardware)
	{
		// Check if bit 2 of REG_MISC is set
		// if so nReset will not issue a POR, we'll need to clear that bit first
		uint8_t regMisc;
		ret = readByte(REG_MISC, &regMisc);
		ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "readByte(REG_MISC) returned 0x%x", ret);
		ESP_LOGD(LIBTAG, "read REG_MISC: %x", regMisc);
		if (regMisc & (1 << 2))
		{
			ESP_LOGD(LIBTAG, "clearing REG_MISC bit 2");
			regMisc &= ~(1 << 2);
			ret = writeByte(REG_MISC, regMisc);
			ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "writeByte(REG_MISC) returned 0x%x", ret);
		}
		ESP_LOGD(LIBTAG, "read REG_MISC: %x", regMisc);
		// Reset the SX1509, the pin is active low
		gpio_set_direction(pinReset, GPIO_MODE_OUTPUT);
		gpio_set_level(pinReset, LOW);  // pull reset pin low
		vTaskDelay(pdMS_TO_TICKS(1));					  // Wait for the pin to settle
		gpio_set_level(pinReset, HIGH); // pull reset pin back high
	}
	else
	{
		// Software reset command sequence:
		ret = writeByte(REG_RESET, 0x12);
		ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "writeByte(REG_RESET, 0x12) returned 0x%x", ret);
		ret = writeByte(REG_RESET, 0x34);
		ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "writeByte(REG_RESET, 0x34) returned 0x%x", ret);
	}
	return ret;
}

esp_err_t SX1509::pinMode(uint8_t pin, uint8_t inOut, uint8_t initialLevel)
{
	esp_err_t ret = ESP_OK;
	uint8_t modeBit;
	if ((inOut == OUTPUT) || (inOut == ANALOG_OUTPUT))
	{
		uint16_t tempRegData;
		ret = readWord(REG_DATA_B, &tempRegData);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_DATA_B) returned 0x%x", ret);
		if (initialLevel == LOW)
		{
			tempRegData &= ~(1 << pin);
			ret = writeWord(REG_DATA_B, tempRegData);
			ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeword(REG_DATA_B, %i) returned 0x%x", tempRegData, ret);
		}
		modeBit = 0;
	}
	else
	{
		modeBit = 1;
	}
	// ESP_LOGD(LIBTAG, "pin %i modeBit is %i", pin, modeBit);

	uint16_t workingDirectionReg;
	uint16_t savedDirectionReg;

	ret = readWord(REG_DIR_B, &workingDirectionReg);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_DIR_B) returned 0x%x", ret);
	ESP_LOGD(LIBTAG, "old dir flags: 0x%04x", workingDirectionReg);
	if (modeBit)
		workingDirectionReg |= (1 << pin);
	else
		workingDirectionReg &= ~(1 << pin);

	ret = writeWord(REG_DIR_B, workingDirectionReg);
	ESP_LOGD(LIBTAG, "setting pin %i dir to 0x%x: writeWord(REG_DIR_B, 0x%x) returned 0x%x", pin, inOut, workingDirectionReg, ret);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_DIR_B, 0x%x) returned 0x%x", workingDirectionReg, ret);
	// verify 
	ret = readWord(REG_DIR_B, &savedDirectionReg);
	ESP_RETURN_ON_FALSE((savedDirectionReg == workingDirectionReg), ESP_FAIL, LIBTAG, "setdir pin %i failed. 0x%x should be 0x%x", pin, savedDirectionReg, workingDirectionReg);

	// If INPUT_PULLUP was called, set up the pullup too:
	if (inOut == INPUT_PULLUP)
	{
		ret = writePin(pin, HIGH); //TEST
		ESP_LOGD(LIBTAG, "pulling up pin: writePin(%i, %i) returned 0x%x", pin, HIGH, ret);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writePin(%i, %i) returned 0x%x", pin, HIGH, ret);
	}
	if (inOut == ANALOG_OUTPUT)
	{
		ret = ledDriverInit(pin); //TEST
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "ledDriverInit(%i) returned 0x%x", pin, ret);
	}
	return ret;
}

//TEST
esp_err_t SX1509::writePin(uint8_t pin, uint8_t highLow)
{
	esp_err_t ret = ESP_OK;
	uint16_t tempRegDir;
	ret = readWord(REG_DIR_B, &tempRegDir);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_DIR_B) returned 0x%x", ret);
	if ((0xFFFF ^ tempRegDir) & (1 << pin)) // If the pin is an output, write high/low
	{
		uint16_t tempRegData;
		ret = readWord(REG_DATA_B, &tempRegData);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_DATA_B) returned 0x%x", ret);
		if (highLow)
			tempRegData |= (1 << pin);
		else
			tempRegData &= ~(1 << pin);
		return writeWord(REG_DATA_B, tempRegData);
	}
	else // Otherwise the pin is an input, pull-up/down
	{
		uint16_t tempPullUp;
		ret = readWord(REG_PULL_UP_B, &tempPullUp);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readword(REG_PULL_UP_B) returned 0x%x", ret);
		uint16_t tempPullDown;
		ret = readWord(REG_PULL_DOWN_B, &tempPullDown);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readword(REG_PULL_DOWN_B) returned 0x%x", ret);

		if (highLow) // if HIGH, do pull-up, disable pull-down
		{
			tempPullUp |= (1 << pin);
			tempPullDown &= ~(1 << pin);
		}
		else // If LOW do pull-down, disable pull-up
		{
			tempPullDown |= (1 << pin);
			tempPullUp &= ~(1 << pin);
		}
		ret = writeWord(REG_PULL_UP_B, tempPullUp);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_PULL_UP_B) returned 0x%x", ret);
		ret = writeWord(REG_PULL_DOWN_B, tempPullDown); 
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_PULL_DOWN_B) returned 0x%x", ret);
		return ret; 
	}
}

esp_err_t SX1509::digitalWrite(uint8_t pin, uint8_t highLow)
{
	return writePin(pin, highLow);
}

uint8_t SX1509::readPin(uint8_t pin)
{
	//esp_err_t ret = ESP_OK;
	uint16_t tempRegDir;
	//ret = readWord(REG_DIR_B, &tempRegDir);
	readWord(REG_DIR_B, &tempRegDir);

	if (tempRegDir & (1 << pin)) // If the pin is an input
	{
		uint16_t tempRegData = 0;
		//ret = readWord(REG_DATA_B, &tempRegData);
		readWord(REG_DATA_B, &tempRegData);
		if (tempRegData & (1 << pin))
			return HIGH;
	}
	else
	{
		ESP_LOGE(LIBTAG, "Pin %d not INPUT, REG_DIR_B: %x", pin, tempRegDir);
	}

	return LOW;
}

bool SX1509::readPin(const uint8_t pin, bool *value)
{
	uint16_t tempRegDir;
	if (readWord(REG_DIR_B, &tempRegDir))
	{
		if (tempRegDir & (1 << pin))
		{ // If the pin is an input
			uint16_t tempRegData;
			if (readWord(REG_DATA_B, &tempRegData))
			{
				*value = (tempRegData & (1 << pin)) != 0;
				return true;
			};
		}
		else
		{
			*value = false;
			return true;
		}
	}
	return false;
}

uint8_t SX1509::digitalRead(uint8_t pin)
{
	return readPin(pin);
}

bool SX1509::digitalRead(uint8_t pin, bool *value)
{
	return readPin(pin, value);
}

// ORing the ret's together out of laziness. If more than one type of error, a munged error code is guaranteed.
esp_err_t SX1509::ledDriverInit(uint8_t pin, uint8_t freq /*= 1*/, bool log /*= false*/)
{
	esp_err_t ret = ESP_OK;
	uint16_t tempWord;
	uint8_t tempByte;

	// Disable input buffer
	// Writing a 1 to the pin bit will disable that pins input buffer
	ret = readWord(REG_INPUT_DISABLE_B, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_INPUT_DISABLE_B) returned 0x%x", ret); 
	tempWord |= (1 << pin);
	ret = writeWord(REG_INPUT_DISABLE_B, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_INPUT_DISABLE_B, %x) returned 0x%x", tempWord, ret); 
	
	// Disable pull-up
	// Writing a 0 to the pin bit will disable that pull-up resistor
	ret = readWord(REG_PULL_UP_B, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_PULL_UP_B) returned 0x%x", ret); 
	tempWord &= ~(1 << pin);
	ret = writeWord(REG_PULL_UP_B, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_PULL_UP_B, %x) returned 0x%x", tempWord, ret); 

	// Set direction to output (REG_DIR_B)
	ret = readWord(REG_DIR_B, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_DIR_B) returned 0x%x", ret); 
	tempWord &= ~(1 << pin); // 0=output
	ret = writeWord(REG_DIR_B, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_DIR_B, %x) returned 0x%x", tempWord, ret); 

	// Enable oscillator (REG_CLOCK)
	ret = readByte(REG_CLOCK, &tempByte);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_CLOCK) returned 0x%x", ret); 
	tempByte |= (1 << 6);  // Internal 2MHz oscillator part 1 (set bit 6)
	tempByte &= ~(1 << 5); // Internal 2MHz oscillator part 2 (clear bit 5)
	ret = writeByte(REG_CLOCK, tempByte);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_CLOCK, %x) returned 0x%x", tempByte, ret); 

	// Configure LED driver clock and mode (REG_MISC)
	ret = readByte(REG_MISC, &tempByte);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_MISC) returned 0x%x", ret); 
	if (log)
	{
		tempByte |= (1 << 7); // set logarithmic mode bank B
		tempByte |= (1 << 3); // set logarithmic mode bank A
	}
	else
	{
		tempByte &= ~(1 << 7); // set linear mode bank B
		tempByte &= ~(1 << 3); // set linear mode bank A
	}

	// Use configClock to setup the clock divder
	if (_clkX == 0) // Make clckX non-zero
	{
		// _clkX = 2000000.0 / (1 << (1 - 1)); // Update private clock variable
		_clkX = 2000000.0;

		// uint8_t freq = (1 & 0x07) << 4; // freq should only be 3 bits from 6:4
		// tempByte |= freq;
	}

	freq = (freq & 0x7) << 4;	// mask only 3 bits and shift to bit position 6:4 
	tempByte |= freq;

	ret = writeByte(REG_MISC, tempByte);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_MISC, %x) returned 0x%x", tempWord, ret); 

	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	ret = readWord(REG_LED_DRIVER_ENABLE_B, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_LED_DRIVER_ENABLE_B) returned 0x%x", ret); 
	tempWord |= (1 << pin);
	ret = writeWord(REG_LED_DRIVER_ENABLE_B, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_LED_DRIVER_ENABLE_B, %x) returned 0x%x", tempWord, ret); 

	// Set REG_DATA bit low ~ LED driver started
	ret = readWord(REG_DATA_B, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_DATA_B) returned 0x%x", ret); 
	tempWord &= ~(1 << pin);
	ret = writeWord(REG_DATA_B, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_DATA_B, %x) returned 0x%x", tempWord, ret); 
	return ret;
}

void SX1509::pwm(uint8_t pin, uint8_t iOn)
{
	// Write the on intensity of pin
	// Linear mode: Ion = iOn
	// Log mode: Ion = f(iOn)
	writeByte(REG_I_ON[pin], iOn);
}

void SX1509::analogWrite(uint8_t pin, uint8_t iOn)
{
	pwm(pin, iOn);
}

esp_err_t SX1509::blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity)
{
	ESP_LOGD(LIBTAG, "blink() tOn:%lu, toff:%lu", tOn, tOff);
	uint8_t onReg = calculateLEDTRegister(tOn);
	uint8_t offReg = calculateLEDTRegister(tOff);
	// onReg = 18;
	// offReg = 2;
	ESP_LOGD(LIBTAG, "onReg:0x%x, offReg:0x%x", onReg, offReg);
	return setupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
}

esp_err_t SX1509::breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log)
{
	offInt = constrain(offInt, 0, 7);

	uint8_t onReg = calculateLEDTRegister(tOn);
	uint8_t offReg = calculateLEDTRegister(tOff);

	uint8_t riseTime = calculateSlopeRegister(rise, onInt, offInt);
	uint8_t fallTime = calculateSlopeRegister(fall, onInt, offInt);

	return setupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime, log);
}

esp_err_t SX1509::setupBlink(uint8_t pin, uint8_t onReg, uint8_t offReg, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall, bool log)
{
	esp_err_t ret = ESP_OK;
	ret = ledDriverInit(pin, log);
    ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "ledDriverInit(%i, %i) returned 0x%x", pin, log, ret); 

	// Keep parameters within their limits:
	onReg &= 0x1F;  // tOn should be a 5-bit value
	offReg &= 0x1F; // tOff should be a 5-bit value
	offIntensity &= 0x07;
	// Write the time on
	// 1-15:  TON = 64 * tOn * (255/ClkX)
	// 16-31: TON = 512 * tOn * (255/ClkX)
	ret = writeByte(REG_T_ON[pin], onReg);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeByte(%x[%i], %i) returned 0x%x", REG_T_ON[pin], pin, onReg, ret); 

	// Write the time/intensity off register
	// 1-15:  TOFF = 64 * tOff * (255/ClkX)
	// 16-31: TOFF = 512 * tOff * (255/ClkX)
	// linear Mode - IOff = 4 * offIntensity
	// log mode - Ioff = f(4 * offIntensity)
	uint8_t value = (offReg << 3) | offIntensity;
	ret = writeByte(REG_OFF[pin], value);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeByte(%x[%i], %i) returned 0x%x", REG_OFF[pin], pin, value, ret); 

	// Write the on intensity:
	ret = writeByte(REG_I_ON[pin], onIntensity);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeByte(%x[%i], %i) returned 0x%x", REG_T_ON[pin], pin, onIntensity, ret); 

	// Prepare tRise and tFall
	tRise &= 0x1F; // tRise is a 5-bit value
	tFall &= 0x1F; // tFall is a 5-bit value

	// Write regTRise
	// 0: Off
	// 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	// 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	if (REG_T_RISE[pin] != 0xFF)
	{
		ret = writeByte(REG_T_RISE[pin], tRise);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeByte(%x[%i], %i) returned 0x%x", REG_T_RISE[pin], pin, tRise, ret); 
	}
	// Write regTFall
	// 0: off
	// 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	// 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	if (REG_T_FALL[pin] != 0xFF)
	{
		ret = writeByte(REG_T_FALL[pin], tFall);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeByte(%x[%i], %i) returned 0x%x", REG_T_FALL[pin], pin, tRise, ret); 
	}
	return ret;
}


// void SX1509::keypad(uint8_t rows, uint8_t columns, uint16_t sleepTime, uint8_t scanTime, uint8_t debounceTime)
// {
// 	uint16_t tempWord;
// 	uint8_t tempByte;

// 	// If clock hasn't been set up, set it to internal 2MHz
// 	if (_clkX == 0)
// 		clock(INTERNAL_CLOCK_2MHZ);

// 	// Set regDir 0:7 outputs, 8:15 inputs:
// 	tempWord = readWord(REG_DIR_B);
// 	for (uint8_t i = 0; i < rows; i++)
// 		tempWord &= ~(1 << i);
// 	for (uint8_t i = 8; i < (columns * 2); i++)
// 		tempWord |= (1 << i);
// 	writeWord(REG_DIR_B, tempWord);

// 	// Set regOpenDrain on 0:7:
// 	tempByte = readByte(REG_OPEN_DRAIN_A);
// 	for (uint8_t i = 0; i < rows; i++)
// 		tempByte |= (1 << i);
// 	writeByte(REG_OPEN_DRAIN_A, tempByte);

// 	// Set regPullUp on 8:15:
// 	tempByte = readByte(REG_PULL_UP_B);
// 	for (uint8_t i = 0; i < columns; i++)
// 		tempByte |= (1 << i);
// 	writeByte(REG_PULL_UP_B, tempByte);

// 	// Debounce Time must be less than scan time
// 	debounceTime = constrain(debounceTime, 1, 64);
// 	scanTime = constrain(scanTime, 1, 128);
// 	if (debounceTime >= scanTime)
// 	{
// 		debounceTime = scanTime >> 1; // Force debounceTime to be less than scanTime
// 	}
// 	debounceKeypad(debounceTime, rows, columns);

// 	// Calculate scanTimeBits, based on scanTime
// 	uint8_t scanTimeBits = 0;
// 	for (uint8_t i = 7; i > 0; i--)
// 	{
// 		if (scanTime & (1 << i))
// 		{
// 			scanTimeBits = i;
// 			break;
// 		}
// 	}

// 	// Calculate sleepTimeBits, based on sleepTime
// 	uint8_t sleepTimeBits = 0;
// 	if (sleepTime != 0)
// 	{
// 		for (uint8_t i = 7; i > 0; i--)
// 		{
// 			if (sleepTime & ((uint16_t)1 << (i + 6)))
// 			{
// 				sleepTimeBits = i;
// 				break;
// 			}
// 		}
// 		// If sleepTime was non-zero, but less than 128,
// 		// assume we wanted to turn sleep on, set it to minimum:
// 		if (sleepTimeBits == 0)
// 			sleepTimeBits = 1;
// 	}

// 	// RegKeyConfig1 sets the auto sleep time and scan time per row
// 	sleepTimeBits = (sleepTimeBits & 0b111) << 4;
// 	scanTimeBits &= 0b111; // Scan time is bits 2:0
// 	tempByte = sleepTime | scanTimeBits;
// 	writeByte(REG_KEY_CONFIG_1, tempByte);

// 	// RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
// 	rows = (rows - 1) & 0b111;		 // 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
// 	columns = (columns - 1) & 0b111; // 0b000 = 1 column, ob111 = 8 columns, etc.
// 	writeByte(REG_KEY_CONFIG_2, (rows << 3) | columns);
// }

// uint16_t SX1509::readKeypad()
// {
// 	return readKeyData();
// }

// uint16_t SX1509::readKeyData()
// {
// 	return (0xFFFF ^ readWord(REG_KEY_DATA_1));
// }

// uint8_t SX1509::getRow(uint16_t keyData)
// {
// 	uint8_t rowData = uint8_t(keyData & 0x00FF);

// 	for (uint8_t i = 0; i < 8; i++)
// 	{
// 		if (rowData & (1 << i))
// 			return i;
// 	}
// 	return 0;
// }

// uint8_t SX1509::getCol(uint16_t keyData)
// {
// 	uint8_t colData = uint8_t((keyData & 0xFF00) >> 8);

// 	for (uint8_t i = 0; i < 8; i++)
// 	{
// 		if (colData & (1 << i))
// 			return i;
// 	}
// 	return 0;
// }

// void SX1509::sync(void)
// {
// 	// First check if nReset functionality is set
// 	uint8_t regMisc = readByte(REG_MISC);
// 	if (!(regMisc & 0x04))
// 	{
// 		regMisc |= (1 << 2);
// 		writeByte(REG_MISC, regMisc);
// 	}

// 	// Toggle nReset pin to sync LED timers
// 	::pinMode(pinReset, OUTPUT);	  // set reset pin as output
// 	::digitalWrite(pinReset, LOW);  // pull reset pin low
// 	delay(1);					  // Wait for the pin to settle
// 	::digitalWrite(pinReset, HIGH); // pull reset pin back high

// 	// Return nReset to POR functionality
// 	writeByte(REG_MISC, (regMisc & ~(1 << 2)));
// }

// void SX1509::debounceConfig(uint8_t configValue)
// {
// 	// First make sure clock is configured
// 	uint8_t tempByte = readByte(REG_MISC);
// 	if ((tempByte & 0x70) == 0)
// 	{
// 		tempByte |= (1 << 4); // Just default to no divider if not set
// 		writeByte(REG_MISC, tempByte);
// 	}
// 	tempByte = readByte(REG_CLOCK);
// 	if ((tempByte & 0x60) == 0)
// 	{
// 		tempByte |= (1 << 6); // default to internal osc.
// 		writeByte(REG_CLOCK, tempByte);
// 	}

// 	configValue &= 0b111; // 3-bit value
// 	writeByte(REG_DEBOUNCE_CONFIG, configValue);
// }

// void SX1509::debounceTime(uint8_t time)
// {
// 	if (_clkX == 0)					   // If clock hasn't been set up.
// 		clock(INTERNAL_CLOCK_2MHZ, 1); // Set clock to 2MHz.

// 	// Debounce time-to-byte map: (assuming fOsc = 2MHz)
// 	// 0: 0.5ms		1: 1ms
// 	// 2: 2ms		3: 4ms
// 	// 4: 8ms		5: 16ms
// 	// 6: 32ms		7: 64ms
// 	// 2^(n-1)
// 	uint8_t configValue = 0;
// 	// We'll check for the highest set bit position,
// 	// and use that for debounceConfig
// 	for (int8_t i = 7; i >= 0; i--)
// 	{
// 		if (time & (1 << i))
// 		{
// 			configValue = i + 1;
// 			break;
// 		}
// 	}
// 	configValue = constrain(configValue, 0, 7);

// 	debounceConfig(configValue);
// }

// void SX1509::debounceEnable(uint8_t pin)
// {
// 	uint16_t debounceEnable = readWord(REG_DEBOUNCE_ENABLE_B);
// 	debounceEnable |= (1 << pin);
// 	writeWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
// }

// void SX1509::debouncePin(uint8_t pin)
// {
// 	debounceEnable(pin);
// }

// void SX1509::debounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols)
// {
// 	// Set up debounce time:
// 	debounceTime(time);

// 	// Set up debounce pins:
// 	for (uint8_t i = 0; i < numRows; i++)
// 		debouncePin(i);
// 	for (uint8_t i = 0; i < (8 + numCols); i++)
// 		debouncePin(i);
// }

//TEST
esp_err_t SX1509::enableInterrupt(uint8_t pin, uint8_t riseFall)
{
	// Set REG_INTERRUPT_MASK
	esp_err_t ret = ESP_OK;
	uint16_t tempWord = 0;
	ret = readWord(REG_INTERRUPT_MASK_B, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_INTERRUPT_MASK_B) returned 0x%x", ret);
	tempWord &= ~(1 << pin); // 0 = event on IO will trigger interrupt
	ret = writeWord(REG_INTERRUPT_MASK_B, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_INTERRUPT_MASK_B, %x) returned 0x%x", tempWord, ret);
	
	uint8_t sensitivity = 0;
	switch (riseFall)
	{
	case CHANGE:
		sensitivity = 0b11;
		break;
	case FALLING:
		sensitivity = 0b10;
		break;
	case RISING:
		sensitivity = 0b01;
		break;
	}

	// Set REG_SENSE_XXX
	// Sensitivity is set as follows:
	// 00: None
	// 01: Rising
	// 10: Falling
	// 11: Both
	uint8_t pinMask = (pin & 0x07) * 2;
	uint8_t senseRegister;

	// Need to select between two words. One for bank A, one for B.
	if (pin >= 8)
		senseRegister = REG_SENSE_HIGH_B;
	else
		senseRegister = REG_SENSE_HIGH_A;

	ret = readWord(senseRegister, &tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(%x) returned 0x%x", senseRegister, ret);
	tempWord &= ~(0b11 << pinMask);		  // Mask out the bits we want to write
	tempWord |= (sensitivity << pinMask); // Add our new bits
	ret = writeWord(senseRegister, tempWord);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(%x, %x) returned 0x%x", senseRegister, tempWord, ret);
	return ret;
}

//TEST
esp_err_t SX1509::interruptSource( uint16_t *intSource, bool clear /* =true*/)
{
	esp_err_t ret = ESP_OK;
	ret = readWord(REG_INTERRUPT_SOURCE_B, intSource);
	ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "readWord(REG_INTERRUPT_SOURCE_B) returned 0x%x", ret);
	if (clear) 
	{	
		ret = writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF);
		ESP_RETURN_ON_FALSE((ret == ESP_OK), ret, LIBTAG, "writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF) returned 0x%x", ret);
	}
	return ret;
}

//TEST
bool SX1509::checkInterrupt(uint8_t pin)
{
	uint16_t intSource = 0;
	interruptSource(&intSource, false);
	if (intSource & (1 << pin))
		return true;

	return false;
}

esp_err_t SX1509::clock(uint8_t oscSource, uint8_t oscDivider, uint8_t oscPinFunction, uint8_t oscFreqOut)
{
	return configClock(oscSource, oscPinFunction, oscFreqOut, oscDivider);
}

esp_err_t SX1509::configClock(uint8_t oscSource /*= 2*/, uint8_t oscPinFunction /*= 0*/, uint8_t oscFreqOut /*= 0*/, uint8_t oscDivider /*= 1*/)
{
	esp_err_t ret = ESP_OK;

	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency souce
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 ouptut
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	oscSource = (oscSource & 0b11) << 5;		// 2-bit value, bits 6:5
	oscPinFunction = (oscPinFunction & 1) << 4; // 1-bit value bit 4
	oscFreqOut = (oscFreqOut & 0b1111);			// 4-bit value, bits 3:0
	uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
	ret = writeByte(REG_CLOCK, regClock);
	ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "reset(): writeByte(REG_CLOCK, %x) returned 0x%x", regClock, ret);

	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
	oscDivider = constrain(oscDivider, 1, 7);
	_clkX = 2000000.0 / (1 << (oscDivider - 1)); // Update private clock variable
	oscDivider = (oscDivider & 0b111) << 4;		 // 3-bit value, bits 6:4

	uint8_t regMisc;
	ret = readByte(REG_MISC, &regMisc);
	ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "reset(): readByte(REG_MISC, %x) returned 0x%x", regMisc, ret);
	regMisc &= ~(0b111 << 4);
	regMisc |= oscDivider;
	ret = writeByte(REG_MISC, regMisc);
	ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "reset(): writeByte(REG_MISC, %x) returned 0x%x", regMisc, ret);

	return ret;
}

uint8_t SX1509::calculateLEDTRegister(unsigned long ms)
{
	uint8_t regOn1, regOn2;
	float timeOn1, timeOn2;

	if (_clkX == 0)
		return 0;

	regOn1 = (float)(ms / 1000.0) / (64.0 * 255.0 / (float)_clkX);
	regOn2 = regOn1 / 8;
	regOn1 = constrain(regOn1, 1, 15);
	regOn2 = constrain(regOn2, 16, 31);

	timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
	timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;
	ESP_LOGD(LIBTAG, "calcLEDTReg() ms:%lu, 1:timeOn:%f, regOn:0x%x, 2:%f, 0x%x", ms, timeOn1, regOn1, timeOn2, regOn2);
	if (abs(timeOn1 - ms) < abs(timeOn2 - ms)) // return the regOn that's closest to the target value
		return regOn1;
	else
		return regOn2;
}

uint8_t SX1509::calculateSlopeRegister(uint8_t ms, uint8_t onIntensity, uint8_t offIntensity)
{
	uint16_t regSlope1, regSlope2;
	float regTime1, regTime2;

	if (_clkX == 0)
		return 0;

	float tFactor = ((float)onIntensity - (4.0 * (float)offIntensity)) * 255.0 / (float)_clkX;
	float timeS = float(ms) / 1000.0;

	regSlope1 = timeS / tFactor;
	regSlope2 = regSlope1 / 16;

	regSlope1 = constrain(regSlope1, 1, 15);
	regSlope2 = constrain(regSlope2, 16, 31);

	regTime1 = regSlope1 * tFactor * 1000.0;
	regTime2 = 16 * regTime1;

	if (abs(regTime1 - ms) < abs(regTime2 - ms))
		return regSlope1;
	else
		return regSlope2;
}

// readByte(uint8_t registerAddress)
//	This function reads a single byte located at the registerAddress register.
//	- deviceAddress should already be set by the constructor.
//	- Return value is ESP_OK if all is well, or an error code from i2c_master_write_read_device otherwise.
// Value is returned in read_buffer.
esp_err_t SX1509::readByte(uint8_t registerAddress, uint8_t* read_buffer)
{
	return i2c_dev_read_reg(_dev, registerAddress, read_buffer, 1); // TEST

	// return i2c_master_write_read_device(I2C_NUM_0, _server_addr,
	// 	&registerAddress, 1,
	// 	read_buffer, 1,
	// 	10 / portTICK_PERIOD_MS);
}

// // readWord(uint8_t registerAddress)
// //	This function will read a two-byte word beginning at registerAddress
// //	- A 16-bit uint16_t will be returned.
// //		- The msb of the return value will contain the value read from registerAddress
// //		- The lsb of the return value will contain the value read from registerAddress + 1
// uint16_t SX1509::readWord(uint8_t registerAddress)
//...
// 	return readValue;
// }

// bool SX1509::readByte(uint8_t registerAddress, uint8_t *value)
// {
// 	return readBytes(registerAddress, value, 1);
// }

// readWord(uint8_t registerAddress)
//	This function will read a two-byte word beginning at registerAddress
//	- A 16-bit uint16_t will be set in value.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
//	- Return boolean true if succesfull
// TEST
esp_err_t SX1509::readWord(uint8_t registerAddress, uint16_t *value)
{
	uint8_t dest[2] = {0, 0};
	esp_err_t ret = ESP_OK;
	// ESP_LOGD(LIBTAG, "readWord(%x)", registerAddress);
	ret = readBytes(registerAddress, dest, 2);
	// ret = readByte(registerAddress, &dest[0]); // FIXME DEBUG
	// ret = readByte(registerAddress+1, &dest[1]);
	ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, LIBTAG, "readBytes(%x, dest, 2) returned 0x%x", registerAddress, ret);
//	value[0] = dest[1];
//	value[1] = dest[0];
	*value = (uint16_t)dest[0] << 8 | dest[1]; // FIXME dest[0] is always 0, and dest[1] has what should be in dest[0]
	// ESP_LOGD(LIBTAG, "d0 d1:%x %x val:%x", dest[0], dest[1], *value);
	return ret;
}

// readBytes(uint8_t firstRegisterAddress, uint8_t * destination, uint8_t length)
//	This function reads a series of bytes incrementing from a given address
//	- firstRegisterAddress is the first address to be read
//	- destination is an array of bytes where the read values will be stored into
//	- length is the number of bytes to be read
//	- Return esp_err_t
//FIXME I2C_NUM_0 should not be hard-coded
esp_err_t SX1509::readBytes(uint8_t firstRegisterAddress, uint8_t* destination, uint8_t length)
{
	return i2c_dev_read_reg(_dev, firstRegisterAddress, destination, length); // TEST
	// return i2c_master_write_read_device(I2C_NUM_0, _server_addr,
	// 	&firstRegisterAddress, 1,
	// 	destination, length,
	// 	length * 10 / portTICK_PERIOD_MS); // TODO timing is a guess, and probably too long
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//	This function writes a single byte to a single register on the SX509.
//	- writeValue is written to registerAddress
//	- deviceAddres should already be set from the constructor
//	- Return value: ESP_OK if succeeded
esp_err_t SX1509::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
	return i2c_dev_write_reg(_dev, registerAddress, &writeValue, 1);
	
	// const uint8_t write_buffer[] = {registerAddress, writeValue};
	// return i2c_master_write_to_device(I2C_NUM_0, _server_addr,
    //                                  write_buffer, 2,
    //                                  10 / portTICK_PERIOD_MS);
}

// writeWord(uint8_t registerAddress, uint16_t writeValue)
//	This function writes a two-byte word to registerAddress and registerAddress + 1
//	- the upper byte of writeValue is written to registerAddress
//		- the lower byte of writeValue is written to registerAddress + 1
//	- Return value: true if succeeded, false if failed
// TEST
esp_err_t SX1509::writeWord(uint8_t registerAddress, uint16_t writeValue)
{
	const uint8_t write_buffer[] = { 
		((uint8_t)((writeValue & 0xFF00) >> 8)),
		((uint8_t)(writeValue & 0x00FF))
		};
	return i2c_dev_write_reg(_dev, registerAddress, &write_buffer, 2);
    // // ESP_LOGD(LIBTAG, "reg:0x%x val:0x%x, bytes:0x%x, 0x%x)", write_buffer[0], writeValue, write_buffer[1], write_buffer[2]);
	// return i2c_master_write_to_device(I2C_NUM_0, _server_addr,
    //                                  write_buffer, 3,
    //                                  2* 10 / portTICK_PERIOD_MS);
}

// // writeBytes(uint8_t firstRegisterAddress, uint8_t * writeArray, uint8_t length)
// //	This function writes an array of bytes, beggining at a specific adddress
// //	- firstRegisterAddress is the initial register to be written.
// //		- All writes following will be at incremental register addresses.
// //	- writeArray should be an array of byte values to be written.
// //	- length should be the number of bytes to be written.
// //	- Return value: true if succeeded, false if failed
// bool SX1509::writeBytes(uint8_t firstRegisterAddress, uint8_t *writeArray, uint8_t length)
// {
// ...
// 	return result && (endResult == I2C_ERROR_OK);
// }
