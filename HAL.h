/*
 * HAL.h
 *
 * Created: 30.09.2020 22:45:17
 *  Author: Gurev
 */ 


#ifndef HAL_H_
#define HAL_H_

//Настроки и основные константы
#define MAX_CS_PIN 0
#define SCREEN_UPDATE_DELAY 5
#define BUTTONS_READ_DELAY 50
#define BUTTONS_LONG_PRESS_DELAY 20
#define POINT_FLASH_DELAY 50
#define SCREEN_BLINK_DELAY 500
#define IDLE_DELAY 5000/BUTTONS_READ_DELAY
#define TEMP_READ_DELAY 250

#define HYSTERESIS_MAX 3
#define HYSTERESIS_MIN 3

#define FILTER_FACTOR 6

//Флаги состояния.
#define NORMAL_MODE 1
#define TEMPERATURE_SET_MODE 2
#define ENGENEER_MODE 4
#define PID_CALIBRATION_MODE 8
#define HYSTERESIS_CONFIG_MODE 16
#define HEAT_METHOD_SELECT_MOD 32
#define SCREEN_BLINK_MODE 128
#define SCREEN_OFF_STATE 256
#define OPEN_THERMOCOUPLE 512


//SPI PINS
#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define DD_SS   PB4
#define DD_MOSI PB5
#define DD_MISO PB6
#define DD_SCK  PB7


//Описание нажатых кнопок. Т.к. они подтянуты к +5, то при не нажатых все биты будут 1
#define BTN1 0x07
#define BTN2 0x0B
#define BTN3 0x0D
#define BTN4 0x0E
#define BTN_MASK 0x0f


//EEPROM ADRESSES
#define DEFAULT_TEMP_HBYTE      0x00
#define DEFAULT_TEMP_LBYTE      0x01

#endif /* HAL_H_ */