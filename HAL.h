/*
 * HAL.h
 *
 * Created: 30.09.2020 22:45:17
 *  Author: Gurev
 */ 


#ifndef HAL_H_
#define HAL_H_

//#include "stdint.h"
#include "pid.h"

//Настроки и основные константы
#define MAX_CS_PIN 0
#define SCREEN_UPDATE_DELAY 5
#define BUTTONS_READ_DELAY 50
#define BUTTONS_LONG_PRESS_DELAY 20
#define POINT_FLASH_DELAY 50
#define SCREEN_BLINK_DELAY 200
#define IDLE_DELAY 5000/BUTTONS_READ_DELAY
#define TEMP_READ_DELAY 250
#define PID_TIME_DELAY 150
#define PID_PWM_DELAY  MAX_PID_VAL
#define PID_MIN_PULSE_WIDTH 20

#define HYSTERESIS_MAX 3
#define HYSTERESIS_MIN 3

#define MAX_TEMP 3996       //Максимальная температура * 4 (3996/4=999)
#define MIN_TEMP 0

#define HOLD_TIMER_MAX 20   //Таймер удержания кнопки (в тактак чтения BUTTONS_READ_DELAY)

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
#define DEFAULT_TEMP_HBYTE          0x00
#define DEFAULT_TEMP_LBYTE          0x01
#define PID_P_K_HBYTE               0x10
#define PID_P_K_LBYTE               0x11
#define PID_I_K_HBYTE               0x12
#define PID_I_K_LBYTE               0x13
#define PID_D_K_HBYTE               0x14
#define PID_D_K_LBYTE               0x15

#define TEMPERATURE_CONTROL_MODE    0x02

//DEFAULT VALUES
#define DEFAULT_SEL_TEMP            128
#define PID_P_K_DEFAULT             256
#define PID_I_K_DEFAULT             0
#define PID_D_K_DEFAULT             0

#endif /* HAL_H_ */