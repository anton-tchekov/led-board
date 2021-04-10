#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

#define ARRLEN(x)             (sizeof(x) / sizeof(*x))

/* RC5 */
#define RC5_IN                PIND
#define RC5_PIN              7
#define RC5_TIME      1.778e-3 /* 1.778 _ms */
#define RC5_PULSE_MIN         (uint8_t)(F_CPU / 512 * RC5_TIME * 0.4 + 0.5)
#define RC5_PULSE_1_2         (uint8_t)(F_CPU / 512 * RC5_TIME * 0.8 + 0.5)
#define RC5_PULSE_MAX         (uint8_t)(F_CPU / 512 * RC5_TIME * 1.2 + 0.5)

static volatile uint16_t rc5_data;

/* UART */
#define UART_BAUD         9600
#define UART_PRESCALER        (uint16_t)(F_CPU / UART_BAUD / 16 - 0.5)

void uart_tx(char c)
{
	while(!(UCSR0A & (1 << UDRE0))) ;
	UDR0 = c;
}

void uart_tx_s(const char *s)
{
	register char c;
	while((c = *s++))
	{
		uart_tx(c);
	}
}

void uart_tx_P(const char *s)
{
	register char c;
	while((c = pgm_read_byte(s++)))
	{
		uart_tx(c);
	}
}

int main(void)
{
	char s[8];

	uint16_t i;

	/* RC5 Timer */
	TCCR2B = (1 << CS22) | (1 << CS21);
	TIMSK2 = (1 << TOIE2);

	/* UART Receiver */
	UBRR0 = UART_PRESCALER;
	UCSR0A = 0;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	sei();
	for(;;)
	{
		cli();
		i = rc5_data;
		rc5_data = 0;
		sei();

		if(i)
		{
			i = (i & 0x3F) | (~i >> 7 & 0x40);
			uart_tx_P(PSTR("KEY: "));
			uart_tx_s(itoa(i, s, 10));
			uart_tx_s("\r\n");
		}
	}

	return 0;
}

/* RC5 */
ISR(TIMER2_OVF_vect)
{
	static uint16_t rc5_tmp;
	static uint8_t rc5_bit, rc5_time;
	TCNT2 = -2;

	if(++rc5_time > RC5_PULSE_MAX)
	{
		if(!(rc5_tmp & 0x4000) && rc5_tmp & 0x2000)
		{
			rc5_data = rc5_tmp;
		}

		rc5_tmp = 0;
	}

	if((rc5_bit ^ RC5_IN) & (1 << RC5_PIN))
	{
		rc5_bit = ~rc5_bit;
		if(rc5_time < RC5_PULSE_MIN)
		{
			rc5_tmp = 0;
		}

		if(!rc5_tmp || rc5_time > RC5_PULSE_1_2)
		{
			if(!(rc5_tmp & 0x4000))
			{
				rc5_tmp <<= 1;
			}

			if(!(rc5_bit & (1 << RC5_PIN)))
			{
				rc5_tmp |= 1;
			}

			rc5_time = 0;
		}
	}
}

