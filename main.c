/*
 * Thermocontroller.c
 *
 * Created: 30.09.2020 10:04:18
 * Author : Gurev
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <string.h>
#include "HAL.h"


//Константы. Надписи.
const uint8_t digit[10] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f}; //0,1,2,3,4,5,6,7,8,9
const uint8_t Err[3] = {0x79,0x50,0x50};
const uint8_t PID[3] = {0x73,0x06,0x5e};
const uint8_t HYS[3] = {0x76,0x6e,0x6d};
const uint8_t En[3] =  {0x79,0x54,0x00};			//Надпись En
const uint8_t OPN[3] = {0x3f,0x73,0x54};
const uint8_t DEF[3] = {0x5e,0x79,0x71};
const uint8_t MOD[3] = {0x15,0x5c,0x5e};
const uint8_t CAL[3] = {0x39,0x77,0x38};
const uint8_t REL[3] = {0x50,0x79,0x38};
const uint8_t SET[3] = {0x6d,0x79,0x78};
const uint8_t MAX[3] = {0x15,0x77,0x76};
const uint8_t MIN[3] = {0x15,0x06,0x54};
const uint8_t PID_P[3] = {0x08,0x73,0x08};
const uint8_t PID_I[3] = {0x08,0x06,0x08};
const uint8_t PID_D[3] = {0x08,0x5e,0x08};	
//Глобальные переменные
uint8_t screen[3]={0x3f,0x3f,0x3f};					//Переменный массив содержит текущие показания дисплея.
uint16_t temp_arr[FILTER_FACTOR];					//Массив измеренных температур (скорее всего заменим на расчет бегущего среднего)
uint16_t flags;										//Флаги.
uint16_t set_temp = 0;								//Текущая установленная температура
uint16_t sel_temp = 0;								//Выбранная, но не установленная температура
uint16_t curr_temp = 0;								//Текущая измеренная температура
uint16_t menuNum = 0;								//Номер текущего меню (Инженерный режим)
uint8_t lastAct = 0;								//Хоранит значение предыдущего действия (для использования в пределах)

struct PID_DATA pidData;							//! Структура с параметрами PID регулятора.
int16_t inputValue = 0;								//Значение, просчитанное PID регулятором.
int16_t pK = 0;										//P-Коэффициент
int16_t iK = 0;										//I-Коэффициент
int16_t dK = 0;										//D-Коэффициент

/*! \brief Простой менеджер задач на базе массива.
*/
uint32_t timerManager[7] = {SCREEN_UPDATE_DELAY,
							POINT_FLASH_DELAY,
							BUTTONS_READ_DELAY,
							SCREEN_BLINK_DELAY,
							TEMP_READ_DELAY,
							PID_TIME_DELAY,
							PID_PWM_DELAY}; 


//Прототипы используемых в программе функций
void tick(void);			//Тик таймера. Настроен на 1мс по привычке.
void prnt(void);			//Вывод на экранзначения из массива screen[]. 
void btnsread(void);		//Чтение нажатых кнопочек.
void read_temp (void);		//Чтение текущей температуры из MAX6675
void convert_temp(uint16_t temp);
void SPI_init(void);		//Инициализация аппаратного SPI
int16_t max6675_read(void); //Читаем температуру.
void EEPROM_write(unsigned int uiAddress, unsigned char ucData); //Запись в EEPROM
unsigned char EEPROM_read(unsigned int uiAddress);				 //Чтение из EEPROM
void pSelect(void);
void iSelect(void);
void dSelect(void);


inline void RunTimer (void)
{
	TCCR2 = 1<<WGM21|2<<CS20; 				// Настройка счетчика с предделителем=8
	TCNT2 = 0;								// Обнуляем регистры
	OCR2  = 125; 							// Установка значения таймера 1 тик = 1 мс.
	TIMSK |= 1<<OCIE2;						// Включаем прерывание.
	sei();									// Включаем прерывания глобально
}
void convert_temp(uint16_t temp){
	if(flags&OPEN_THERMOCOUPLE){
		memccpy(screen,OPN,0,3);
		}else{
		temp>>=2;
		screen[0] = digit[temp/100]; temp %= 100;
		if (screen[0]==digit[0]) screen[0]=0x00;
		screen[1] = digit[temp/10]; temp %= 10;
		if (screen[0]==0x00&&screen[1]==digit[0])screen[1]=0x00;
		screen[2] = digit[temp];
	}
}
//Функции PID регулятора
int16_t Get_Reference(void);
int16_t Get_Measurement(void);
void Set_Input(int16_t inputValue);


ISR(TIMER2_COMP_vect)
{
	tick();
}
int main(void)
{
	_delay_ms(20);			//Задержка на 20 мс. Просто дать всему включиться.
	//Настройка портов
	DDRA = 0xff;			//Выход на сегменты
	DDRB = 0x00;			//Настройка кнопок на вход
	DDRC = 0x0f;			//����� �� ������ � SSR
	DDRD = 0xff;			//��������� MAX6675
	PORTA = 0xff;			//�������� ��� ����������
	PORTB = 0x5f;			//Подтягиваем все кнопки к +5В, также SS для MAX6675 К +5В.
	//Прогоняем тестовый экран. Для проверки динамической индикации.
	PORTC = 1;_delay_ms(250);
	PORTC = 2;_delay_ms(250);
	PORTC = 4;_delay_ms(250);
	PORTC = 8;_delay_ms(250);
	PORTC = 0;_delay_ms(250);
	
	//Вычитываем данные настроек из EEPROM
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		sel_temp = EEPROM_read(DEFAULT_TEMP_HBYTE);		//Температура по-умолчанию старший байт
		sel_temp <<=8;
		sel_temp |=EEPROM_read(DEFAULT_TEMP_LBYTE);		//Температура по-умолчанию младший байт
		if(sel_temp>MAX_TEMP)sel_temp=DEFAULT_SEL_TEMP;		//Защита от чистого EEPROM. С новья будет на 500.
		pK = EEPROM_read(PID_P_K_HBYTE);
		pK <<=8;
		pK|= EEPROM_read(PID_P_K_LBYTE);
		if ((pK>=MAX_TEMP)||(pK<0)) pK = PID_P_K_DEFAULT;
		iK = EEPROM_read(PID_I_K_HBYTE);
		iK <<=8;
		iK|= EEPROM_read(PID_I_K_LBYTE);
		if ((iK>=MAX_TEMP)||(iK<0)) iK = PID_I_K_DEFAULT;
		dK = EEPROM_read(PID_D_K_HBYTE);
		dK <<=8;
		dK|= EEPROM_read(PID_D_K_LBYTE);
		if ((dK>=MAX_TEMP)||(dK<0)) dK = PID_D_K_DEFAULT;
	}


	//Инициализация аппаратного SPI
	SPI_init();		
	//Инициализация PID алгоритма
	pid_Init(pK, iK, dK, &pidData);

	PORTC = 7;_delay_ms(1000);
	PORTC = 0;
	PORTA = 0x7f;
	//Если зажать левую кнопку (ESC) - то будет загрузка в инженерное меню.
	if((PINB&BTN_MASK)==BTN4){
		flags=ENGENEER_MODE;
		menuNum = 1;
		while((PINB&BTN_MASK)==BTN4){}			//Ждем пока кнопку отпустят.
	}else{
		flags=NORMAL_MODE;
	}
	RunTimer();					//Запускаем таймер.

//------------------------------------------------------------------	
//Процедуры инженерного меню.
//------------------------------------------------------------------
	while (flags==ENGENEER_MODE)
	{
		//------------------------------------------------------------------
		if (timerManager[0]>=SCREEN_UPDATE_DELAY){
			prnt();
			timerManager[0]=0;
		}
		if (timerManager[1]>100){
			switch (menuNum){
				//case 1:memccpy(screen,DEF,0,3);break;
				case 1:memccpy(screen,MOD,0,3);break;
				case 2:memccpy(screen,PID,0,3);break;
				case 3:memccpy(screen,HYS,0,3);break;
				case 4:memccpy(screen,CAL,0,3);break;
				case 10:memccpy(screen,PID,0,3);break;
				case 11:memccpy(screen,HYS,0,3);break;
				case 12:memccpy(screen,REL,0,3);break;
				case 20:memccpy(screen,SET,0,3);break;
//				case 21:memccpy(screen,CAL,0,3);break;
				case 30:memccpy(screen,MAX,0,3);break;
				case 31:memccpy(screen,MIN,0,3);break;
				case 40:convert_temp(0);break;
				case 41:convert_temp(500);break;
				case 200:memccpy(screen,PID_P,0,3);break;
				case 2000:pSelect();break;
				case 201:memccpy(screen,PID_I,0,3);break;
				case 2010:iSelect();break;
				case 202:memccpy(screen,PID_D,0,3);break;
				case 2020:dSelect();break;
				default:
					if(lastAct==1)menuNum/=10;
					if(lastAct==2)menuNum--;
					if(lastAct==3)menuNum++;
			}
			//convert_temp(menuNum);
			PORTA ^= (1<<7);	//Индикация инженерного меню. Моргаем точками разрядов.
			timerManager[1]=0;
		}
		
		if(timerManager[2]>=BUTTONS_READ_DELAY){
			static uint8_t btnStatus;
			if (btnStatus!=(PINB&BTN_MASK)) {
				btnStatus = (PINB&BTN_MASK);
				switch (btnStatus){
					case BTN1:menuNum*=10;lastAct=1;break;
					case BTN2:menuNum++;lastAct=2; break;
					case BTN3:menuNum--;lastAct=3; break;
					case BTN4:menuNum/=10;lastAct=4; break;
				}
				while ((PINB&BTN_MASK)!=BTN_MASK){PORTA=0x00;}	//Пока кнопку не отпустят - стопорим дальнейшее выполнение.
			}
			if (menuNum<=0 && lastAct==4){ //Продолжаем загрузку в нормальном режиме.
				flags=NORMAL_MODE;
				PORTA &= ~(1<<7);
			}
			timerManager[2]=0;
		}
	}
//------------------------------------------------------------------	
//Основное рабочее тело программы.
//------------------------------------------------------------------
    while (1) 
    {
//------------------------------------------------------------------
		if (timerManager[0]>=SCREEN_UPDATE_DELAY){
			prnt();
			timerManager[0]=0;
		}
//------------------------------------------------------------------
		/*if (timerManager[1]>=POINT_FLASH_DELAY){
			if (set_temp){
				if (curr_temp>(set_temp+HYSTERESIS_MAX)){
					PORTA &= ~(1<<7);
					PORTC &= ~(1<<3);
				}
				if (curr_temp<(set_temp-HYSTERESIS_MIN)){
					PORTA |= (1<<7);
					PORTC |= (1<<3);
				}
			}
			timerManager[1]=0;
		}*/
//------------------------------------------------------------------
		if(timerManager[2]>=BUTTONS_READ_DELAY){
			btnsread();
			timerManager[2]=0;
		}
//------------------------------------------------------------------		
		if (timerManager[3]>=SCREEN_BLINK_DELAY)
		{
			if (flags&TEMPERATURE_SET_MODE)	
			{
				flags ^= SCREEN_OFF_STATE;
				if (flags&SCREEN_OFF_STATE)
				{
					screen[0]=0x00;screen[1]=0x00;screen[2]=0x00;
				}else{
					convert_temp(sel_temp);
				}
			}else
			{
				flags &= ~(SCREEN_OFF_STATE);
				convert_temp(curr_temp);
			}
			timerManager[3]=0;
		}
//------------------------------------------------------------------
		if (timerManager[4]>=TEMP_READ_DELAY){
			read_temp();
			timerManager[4]=0;
		}
//------------------------------------------------------------------
		if (timerManager[5]>=PID_TIME_DELAY)
		{
			if (set_temp>0)
			{
				//referenceValue = Get_Reference();
      			//measurementValue = Get_Measurement();

      			inputValue = (pid_Controller(set_temp, curr_temp, &pidData));
				if (inputValue<0) inputValue=0;
      			//Set_Input(inputValue);
			}
			timerManager[5]=0;
		}
//----Программный ШИМ---------------------
		if (timerManager[6]>=PID_PWM_DELAY)
		{
			timerManager[6]=0;
		}
		//if (set_temp>0)
		//{
			if ((timerManager[6]< (uint16_t)inputValue)||((timerManager[6]<PID_MIN_PULSE_WIDTH)&&(inputValue>0)))
			{
				PORTA |= (1<<7);
				PORTC |= (1<<3);
			}else
			{
				PORTA &= ~(1<<7);
				PORTC &= ~(1<<3);
			}
		//}
		

    }
	wdt_reset();
}
void tick(void){
	for (uint8_t i=0;i<7;i++)
	{
		timerManager[i]++;
	}
}
void prnt(void){
	static uint8_t digit_num;
		switch (digit_num){
			case 0:
				PORTC = ((PORTC&0x08)|0x01); PORTA=((PORTA&0x80)|screen[0]);
				digit_num=1;
				break;
			case 1:
				PORTC = ((PORTC&0x08)|0x02); PORTA=((PORTA&0x80)|screen[1]);
				digit_num=2;
				break;
			case 2:
				PORTC = ((PORTC&0x08)|0x04); PORTA=((PORTA&0x80)|screen[2]);
				digit_num=0;
				break;
			default:
				digit_num=0;	
	}
}
void btnsread(void)
{

	static uint8_t hold_timer;				//Количество циклов удержания
	uint8_t step=4;							//Шаг приращения температуры с учетом сдвига влево (1<<2=4)
	static uint8_t btn_prev = BTN_MASK;		//Предыдущие нажатые кнопки.
	uint8_t btn_curr = PINB&BTN_MASK;		//Текущие нажатые кнопки.
	
	//Секция проверки кнопок. Настроено срабатывание после отпускания, либо после достижения порога таймера.
	if (btn_prev==BTN_MASK)					//Если на предыдущем шаге не было нажато кнопок
	{
		btn_prev = btn_curr;				//Записываем текущие показания
		hold_timer = 0;						//Сбрасываем таймер
		return;								//Выходим
	}
	if(btn_curr==btn_prev)					//Если показания не поменялись
	{
		hold_timer++;						//Приращиваем таймер пока не переполнится
		if (hold_timer<HOLD_TIMER_MAX)		//Проверяем переполнение таймера
		{
			return;							//Не переполнился, выходим.
		}else{
			hold_timer = HOLD_TIMER_MAX-1;	//Переполнился. Даем малую задержку, чтобы следующее срабатывание было очень скоро.
		}
	}
//Ниже идет отработка кнопок. Если программа сюда дошла, значит была нажата, а потом отпущена кнопка.
//Кнопки работают по отпусканию, а не по нажатию. Это исключает вариации "Двойного" нажатия
	if (flags&NORMAL_MODE)
	{
		switch (btn_prev)
		{
		case BTN1:
			flags &=~(NORMAL_MODE);				//Выключаем нормальный режим
			flags |=TEMPERATURE_SET_MODE;		//Включаем режим установки температуры
			break;
		}
	}else if (flags&TEMPERATURE_SET_MODE)
	{
		switch (btn_prev)
		{
			case BTN1:								//Кнопка Enter (Сохранение настройки)
					flags &=~(TEMPERATURE_SET_MODE);	//Выключаем режим выбора температуры
					flags |=NORMAL_MODE;				//Включаем обычный режим
					set_temp = sel_temp;				//Задаем температуру для нагрева.
					pid_Reset_Integrator(&pidData);
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE){	//Записываем в EEPROM новую температуру
						uint16_t eeprom_temp = EEPROM_read(DEFAULT_TEMP_HBYTE);		//Читаем температуру из EEPROM
						eeprom_temp <<=8;
						eeprom_temp |=EEPROM_read(DEFAULT_TEMP_LBYTE);		
						if (eeprom_temp!=sel_temp){		//Если значение установленной температуры отличается от того, что есть в EEPROM - перезаписываем.
							EEPROM_write(DEFAULT_TEMP_HBYTE, (unsigned char)(sel_temp>>8));
							EEPROM_write(DEFAULT_TEMP_LBYTE,(unsigned char)sel_temp);	
						}
					}
				break;
			case BTN2:								//Кнопка +
				if (sel_temp<=(uint16_t)(MAX_TEMP-step))sel_temp+=step;
				break;
			case BTN3:								//Кнопка -
				if (sel_temp>=(uint16_t)(MIN_TEMP+step))sel_temp-=step;
				break;
			case BTN4:								//Кнопка ESC (Выход без сохранения)
				flags &=~(TEMPERATURE_SET_MODE);	//Выключаем режим выбора температуры
				flags |=NORMAL_MODE;				//Включаем обычный режим
			default:
				break;
		}
	}
	btn_prev = btn_curr;
}
void read_temp (void){
	static int16_t filtered_data;
	int16_t raw_data = max6675_read();
	//Проверка на отключенную термопару.
	if ((raw_data>>2)&1) {
		flags |= OPEN_THERMOCOUPLE;
		curr_temp = 0;
	}else{
		flags &= ~(OPEN_THERMOCOUPLE);
		filtered_data= (((filtered_data*(FILTER_FACTOR-1))+(raw_data>>3))/FILTER_FACTOR); //Усредняем
		curr_temp=filtered_data;
	}
	PORTD |= 1<<0;	
}
void SPI_init(void){
    DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);	//Настройка портов MOSI и SCK
	PORTD = (1<<0);						//Настройка пина SS. ХЗ зачем я его сделал отдельно от основного SPI.
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR0);//Настройка регистра. Включить SPI, мастер, на частоте F_CPU/2
}
//Процедура возвращает два байта из микросхемы MAX6675
int16_t max6675_read(void){
    volatile uint8_t raw_byte=0;      	//данные из SPDR.
    uint16_t full_data=0;    			//Переменная для обоих байтов
	PORTD &= ~(1<<0);					//Прижимаем SS. Начало приема.
	SPDR = 0x00;						//Отправляем 8 бит, в ответ прилетит 8 от слейва
	while(!(SPSR & (1<<SPIF)));			//Ждем окончания
	raw_byte = SPDR;					//Считываем данные из регистра
	full_data = raw_byte;				//Пишем старший байт
	full_data <<=8;						//Двигаем его влево
	SPDR = 0x00;						//Отправляем еще 8 бит
	while(!(SPSR & (1<<SPIF)));			//Ждем окончания
	raw_byte = SPDR;					//Считываем данные из регистра
	full_data |= raw_byte;				//Пишем младший байт.
	PORTD |= (1<<0);					//Отпускаем SS. Конец приема.
    return full_data;    				//Возвращаем прочитанное значение.
}
void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
}
unsigned char EEPROM_read(unsigned int uiAddress){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}
/*! \brief Read reference value.
 *
 * This function must return the reference value.
 * May be constant or varying
 */
int16_t Get_Reference(void)
{
  return set_temp;
}
/*! \brief Read system process value
 *
 * This function must return the measured data
 */
int16_t Get_Measurement(void)
{
  return curr_temp;
}
/*! \brief Выбор пропорционального коэффициента
 *
 */
void pSelect(void){
	PORTA &= ~(1<<7);		//Выключаем точки
	uint8_t hold_timer=0;				//Количество циклов удержания
	uint8_t btn_prev = BTN_MASK;		//Предыдущие нажатые кнопки.
	int16_t pK_local = pK;
	while (1)
	{
		while (1)
		{
			if (timerManager[0]>=SCREEN_UPDATE_DELAY)
			{
				convert_temp(pK_local<<2);
				prnt();
				timerManager[0]=0;
			}
			if(timerManager[2]>=BUTTONS_READ_DELAY)
			{
				timerManager[2]=0;
				uint8_t step=1;							//Шаг приращения температуры с учетом сдвига влево (1<<2=4)
				uint8_t btn_curr = PINB&BTN_MASK;		//Текущие нажатые кнопки.
				//Секция проверки кнопок. Настроено срабатывание после отпускания, либо после достижения порога таймера.
				if (btn_prev==BTN_MASK)					//Если на предыдущем шаге не было нажато кнопок
				{
					btn_prev = btn_curr;				//Записываем текущие показания
					hold_timer = 0;						//Сбрасываем таймер
					break;								//Выходим
				}
				if(btn_curr==btn_prev)					//Если показания не поменялись
				{
					hold_timer++;						//Приращиваем таймер пока не переполнится
					if (hold_timer<HOLD_TIMER_MAX)		//Проверяем переполнение таймера
					{
						break;							//Не переполнился, выходим.
					}else{
						hold_timer = HOLD_TIMER_MAX-1;	//Переполнился. Даем малую задержку, чтобы следующее срабатывание было очень скоро.
					}
				}
			//Ниже идет отработка кнопок. Если программа сюда дошла, значит была нажата, а потом отпущена кнопка.
			//Кнопки работают по отпусканию, а не по нажатию. Это исключает вариации "Двойного" нажатия
					switch (btn_prev)
					{
						case BTN1:								//Кнопка Enter (Сохранение настройки)
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE){	
									int16_t eeprom_temp = EEPROM_read(PID_P_K_HBYTE);		
									eeprom_temp <<=8;
									eeprom_temp |=EEPROM_read(PID_P_K_LBYTE);		
									if (eeprom_temp!=pK_local){		
										EEPROM_write(PID_P_K_HBYTE, (unsigned char)(pK_local>>8));
										EEPROM_write(PID_P_K_LBYTE,(unsigned char)pK_local);	
									}
								}
								pK=pK_local;
								menuNum/=10;
								return;
							break;
						case BTN2:								//Кнопка +
							pK_local+=step;
							if (pK_local>MAX_TEMP) pK_local=MAX_TEMP;
							break;
						case BTN3:								//Кнопка -
							pK_local-=step;
							if (pK_local<0) pK_local=0;
							break;
						case BTN4:								//Кнопка ESC (Выход без сохранения)
							menuNum/=10;
							return;
						default:
							break;
					}
				btn_prev = btn_curr;
			}
		}
	}
}
void iSelect(void){
	PORTA &= ~(1<<7);		//Выключаем точки
	uint8_t hold_timer=0;				//Количество циклов удержания
	uint8_t btn_prev = BTN_MASK;		//Предыдущие нажатые кнопки.
	int16_t iK_local = iK;
	while (1)
	{
		while (1)
		{
			if (timerManager[0]>=SCREEN_UPDATE_DELAY)
			{
				convert_temp(iK_local<<2);
				prnt();
				timerManager[0]=0;
			}
			if(timerManager[2]>=BUTTONS_READ_DELAY)
			{
				timerManager[2]=0;
				uint8_t step=1;							//Шаг приращения температуры с учетом сдвига влево (1<<2=4)
				uint8_t btn_curr = PINB&BTN_MASK;		//Текущие нажатые кнопки.
				//Секция проверки кнопок. Настроено срабатывание после отпускания, либо после достижения порога таймера.
				if (btn_prev==BTN_MASK)					//Если на предыдущем шаге не было нажато кнопок
				{
					btn_prev = btn_curr;				//Записываем текущие показания
					hold_timer = 0;						//Сбрасываем таймер
					break;								//Выходим
				}
				if(btn_curr==btn_prev)					//Если показания не поменялись
				{
					hold_timer++;						//Приращиваем таймер пока не переполнится
					if (hold_timer<HOLD_TIMER_MAX)		//Проверяем переполнение таймера
					{
						break;							//Не переполнился, выходим.
					}else{
						hold_timer = HOLD_TIMER_MAX-1;	//Переполнился. Даем малую задержку, чтобы следующее срабатывание было очень скоро.
					}
				}
			//Ниже идет отработка кнопок. Если программа сюда дошла, значит была нажата, а потом отпущена кнопка.
			//Кнопки работают по отпусканию, а не по нажатию. Это исключает вариации "Двойного" нажатия
					switch (btn_prev)
					{
						case BTN1:								//Кнопка Enter (Сохранение настройки)
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE){	
									int16_t eeprom_temp = EEPROM_read(PID_I_K_HBYTE);		
									eeprom_temp <<=8;
									eeprom_temp |=EEPROM_read(PID_I_K_LBYTE);		
									if (eeprom_temp!=iK_local){		
										EEPROM_write(PID_I_K_HBYTE, (unsigned char)(iK_local>>8));
										EEPROM_write(PID_I_K_LBYTE,(unsigned char)iK_local);	
									}
								}
								iK=iK_local;
								menuNum/=10;
								return;
							break;
						case BTN2:								//Кнопка +
							iK_local+=step;
							if (iK_local>MAX_TEMP) iK_local=MAX_TEMP;
							break;
						case BTN3:								//Кнопка -
							iK_local-=step;
							if (iK_local<0) iK_local=0;
							break;
						case BTN4:								//Кнопка ESC (Выход без сохранения)
							menuNum/=10;
							return;
						default:
							break;
					}
				btn_prev = btn_curr;
			}
		}
	}
}
void dSelect(void){
	PORTA &= ~(1<<7);		//Выключаем точки
	uint8_t hold_timer=0;				//Количество циклов удержания
	uint8_t btn_prev = BTN_MASK;		//Предыдущие нажатые кнопки.
	int16_t dK_local = dK;
	while (1)
	{
		while (1)
		{
			if (timerManager[0]>=SCREEN_UPDATE_DELAY)
			{
				convert_temp(dK_local<<2);
				prnt();
				timerManager[0]=0;
			}
			if(timerManager[2]>=BUTTONS_READ_DELAY)
			{
				timerManager[2]=0;
				uint8_t step=1;							//Шаг приращения температуры с учетом сдвига влево (1<<2=4)
				uint8_t btn_curr = PINB&BTN_MASK;		//Текущие нажатые кнопки.
				//Секция проверки кнопок. Настроено срабатывание после отпускания, либо после достижения порога таймера.
				if (btn_prev==BTN_MASK)					//Если на предыдущем шаге не было нажато кнопок
				{
					btn_prev = btn_curr;				//Записываем текущие показания
					hold_timer = 0;						//Сбрасываем таймер
					break;								//Выходим
				}
				if(btn_curr==btn_prev)					//Если показания не поменялись
				{
					hold_timer++;						//Приращиваем таймер пока не переполнится
					if (hold_timer<HOLD_TIMER_MAX)		//Проверяем переполнение таймера
					{
						break;							//Не переполнился, выходим.
					}else{
						hold_timer = HOLD_TIMER_MAX-1;	//Переполнился. Даем малую задержку, чтобы следующее срабатывание было очень скоро.
					}
				}
			//Ниже идет отработка кнопок. Если программа сюда дошла, значит была нажата, а потом отпущена кнопка.
			//Кнопки работают по отпусканию, а не по нажатию. Это исключает вариации "Двойного" нажатия
					switch (btn_prev)
					{
						case BTN1:								//Кнопка Enter (Сохранение настройки)
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE){	
									int16_t eeprom_temp = EEPROM_read(PID_D_K_HBYTE);		
									eeprom_temp <<=8;
									eeprom_temp |=EEPROM_read(PID_D_K_LBYTE);		
									if (eeprom_temp!=dK_local){		
										EEPROM_write(PID_D_K_HBYTE, (unsigned char)(dK_local>>8));
										EEPROM_write(PID_D_K_LBYTE,(unsigned char)dK_local);	
									}
								}
								dK=dK_local;
								menuNum/=10;
								return;
							break;
						case BTN2:								//Кнопка +
							dK_local+=step;
							if (dK_local>MAX_TEMP) dK_local=MAX_TEMP;
							break;
						case BTN3:								//Кнопка -
							dK_local-=step;
							if (dK_local<0) dK_local=0;
							break;
						case BTN4:								//Кнопка ESC (Выход без сохранения)
							menuNum/=10;
							return;
						default:
							break;
					}
				btn_prev = btn_curr;
			}
		}
	}
}