
/*
	Arduino/AVR based H-shifter controller

	BUTTONS 1-6 MAPPED TO PB0-PB4
	BUTTONS 6-12 MAPPED TO PD2-PD7
	AXIS X MAPPED TO PC0
	AXIS Y MAPPED TO PC1

	CALIBRATION:
	- DISCONNECT SHFITER FROM PC
	- DISENGAGE HANDBRAKE
	- HOLD DOWN 1ST AND 2ND GEAR BUTTONS
	- RECONNECT SHIFTER
	- FULLY DISENGAGE HANDBRAKE
    - PRESS 1ST GEAR BUTTON
	- FULLY ENGAGE HANDBRAKE
    - PRESS 1ST GEAR BUTTON
*/

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


#define LED_ON (PORTB |= (1 << 5))
#define LED_OFF (PORTB &= ~(1 << 5))

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

uint8_t enable_axis;
/* Handbrake and extra axis calibration data storage stuff */
uint8_t calXmax;
uint8_t calXmin;
uint8_t calYmax;
uint8_t calYmin;
#define GET_AXIS_MODE eeprom_read_byte((void *)8)
#define SET_AXIS_MODE(val) eeprom_write_byte((void *)8, val)
#define CAL_EEPROM_ADDR (void*)16
#define CAL_KEY_1 (!((PIND >> 2) & 1))
#define CAL_KEY_2 (!((PIND >> 4) & 1))

void uartInit(void) {
    /* Enable transmission circuit */
    UCSR0B = (1 << TXEN0);
    /* 8 bit character sizes, 2 stop bits */
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);
    /* Baud rate settings */
    UBRR0H = (BAUD_PRESCALE >> 8);
    UBRR0L = BAUD_PRESCALE;
}

void uartPutChar(uint8_t ch) {
    /* Wait until TX buffer is empty before transmission */
    while(!(UCSR0A & (1 << UDRE0)));
    UDR0 = ch;
}

#ifdef SERIAL_DEBUG
void uartPutStr(char *str) {
    uartPutChar(*str);
    while(*str++ != '\0') {
        uartPutChar(*str);
    }
}

void uartPutCharHex(uint8_t ch) {
    const char LUT[16] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    uartPutChar('0');
    uint8_t tmp = (ch >> 4);
    uartPutChar('x');
    uartPutChar(LUT[tmp]);
    uartPutChar(LUT[ch & 0x0F]);
}
#endif

void adcInit(void) {
    /* AVCC as analog ref */
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    /* Enable and 128 prescaler */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/* Normalize value between min-max */
/* Probably should replace this with a faster formula */
#define NORMALIZE_X(val) val //((val - calXmin)/(calXmax - calXmin)) * 255
#define NORMALIZE_Y(val) val //((val - calYmin)/(calYmax - calYmin)) * 255

uint8_t (*adcGetX)(void);
uint8_t (*adcGetY)(void);

uint8_t adcGetNull(void) {
    return 0;
}

/* Returns 8-bit value of X axis */
uint8_t adcGetXaxis(void) {
    /* ADC on channel 0 */
    ADMUX = (ADMUX & 0xF8);
    /* Start conversion */
    ADCSRA |= (1 << ADSC);
    /* Wait until conversion complete */
    while(ADCSRA & (1 << ADSC)) {};
	return NORMALIZE_X(ADCH);
}

/* Returns 8-bit value of Y axis */
uint8_t adcGetYaxis(void) {
    /* ADC on channel 1 */
    ADMUX = (ADMUX & 0xF8) | 1;
    /* Start conversion */
    ADCSRA |= (1 << ADSC);
    /* Wait until conversion complete */
    while(ADCSRA & (1 << ADSC)) {};
	return NORMALIZE_Y(ADCH);
}

/* Load calibration data into globals */
void calLoadData(void) {
    /* Load values from EEPROM */
    calXmax = eeprom_read_byte(CAL_EEPROM_ADDR);
    calXmin = eeprom_read_byte(CAL_EEPROM_ADDR + sizeof(uint8_t));
    calYmax = eeprom_read_byte(CAL_EEPROM_ADDR + (2 * sizeof(uint8_t)));
    calYmin = eeprom_read_byte(CAL_EEPROM_ADDR + (3 * sizeof(uint8_t)));

    /* If min 0xFF then probably uncalibrated */
    /* or something went really wrong */
    if(calXmin == 0xFF && calYmin == 0xFF) {
        calXmax = 0xFF;
        calXmin = 0;
        calYmax = 0xFF;
        calYmin = 0;
    }
}

/* Save calibration data into eeprom */
void calSaveData(void) {
    eeprom_write_byte(CAL_EEPROM_ADDR, calXmax);
    eeprom_write_byte((CAL_EEPROM_ADDR + sizeof(uint8_t)), calXmin);
    eeprom_write_byte((CAL_EEPROM_ADDR + (2 * sizeof(uint8_t))), calYmax);
    eeprom_write_byte((CAL_EEPROM_ADDR + (3 * sizeof(uint8_t))), calYmin);
}

/* Get value of axis and wait for confirm on gear 1 */
uint8_t calGetAxis(uint8_t ch) {
    /* Wait for calibration key to begin */
    while(CAL_KEY_1);
    while(!CAL_KEY_1);
    _delay_ms(400);
    /* ADC on channel ch */
    ADMUX = (ADMUX & 0xF8) | ch;
    /* Start conversion */
    ADCSRA |= (1 << ADSC);
    /* Wait until conversion complete */
    while(ADCSRA & (1 << ADSC)) {};
	return ADCH;
}

/* Do calibration routine for axes */
void calCalibrate(void) {
    calXmin = calGetAxis(0);
    calXmax = calGetAxis(0);
    calYmin = calGetAxis(1);
    calYmax = calGetAxis(1);

    #ifdef SERIAL_DEBUG
    uartPutStr("calXmin: ");
    uartPutCharHex(calXmin);
    uartPutChar('\n');
    uartPutStr("calXmax: ");
    uartPutCharHex(calXmax);
    uartPutChar('\n');
    uartPutStr("calYmin: ");
    uartPutCharHex(calYmin);
    uartPutChar('\n');
    uartPutStr("calYmax: ");
    uartPutCharHex(calYmax);
    uartPutChar('\n');
    #endif // SERIAL_DEBUG

    while(CAL_KEY_1);
    while(!CAL_KEY_1);
    #ifndef SERIAL_DEBUG
    calSaveData();
    #endif // SERIAL_DEBUG
}

uint16_t getButtonStates(void) {
	uint8_t a = PINB & 0b00011111;
	uint8_t b = PIND & 0b11111100;
	return ~((a << 6) | (b >> 2)) & 0x07FF;
}

int main(void) {
    /* Initialize our chosen pins as inputs */
	DDRB = (uint8_t)~0b00011111;
	DDRC = (uint8_t)~0b00000011;
	DDRD = (uint8_t)~0b11111101;
	/* Enable pull-ups */
	PORTB = (uint8_t)0b00011111;
	PORTD = (uint8_t)0b11111101;
	LED_OFF;
	/* Initialize peripherals */
	uartInit();
	adcInit();

    enable_axis = GET_AXIS_MODE;

    /* Check for calibration key combo */
    if(CAL_KEY_1 && CAL_KEY_2) {
        /* Do calibration */
        while(CAL_KEY_1 && CAL_KEY_2) { LED_OFF; _delay_ms(100); LED_ON; };
        if(CAL_KEY_1) {
            enable_axis = 1;
            calCalibrate();
        } else {
            enable_axis = 0;
        }
        SET_AXIS_MODE(enable_axis);
        LED_OFF;
    }
    if(enable_axis) {
        /* Load saved calibration data */
        calLoadData();
        /* Set function pointers to axis funcs */
        adcGetX = &adcGetXaxis;
        adcGetY = &adcGetYaxis;
    } else {
        /* Set null function pointers for axis */
        adcGetX = &adcGetNull;
        adcGetY = &adcGetNull;
    }

	for(;;) {
		/* Transmit axis values */
        #ifdef SERIAL_DEBUG
        LED_OFF;
		uartPutChar('X');
		uartPutChar(':');
		uartPutCharHex(adcGetX());
		uartPutChar('\n');
		uartPutChar('Y');
		uartPutChar(':');
		uartPutCharHex(adcGetY());
        uartPutChar('\n');
        #else
        uartPutChar(adcGetX());
        uartPutChar(adcGetY());
        #endif

		/* Transmit button values */
		uint16_t buttonVal = getButtonStates();
		if(buttonVal != 0) {
            LED_ON;
		} else {
            LED_OFF;
		}
		#ifdef SERIAL_DEBUG
		uartPutCharHex((uint8_t)buttonVal);
		uartPutChar('\n');
		uartPutCharHex((uint8_t)(buttonVal >> 8));
		uartPutChar('\n');
		#else
		uartPutChar((uint8_t)buttonVal);
		uartPutChar((uint8_t)(buttonVal >> 8));
		#endif
        }
}
