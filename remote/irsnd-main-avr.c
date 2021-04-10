#include "irsnd.h"

#define BTN_IN                 PINB
#define BTN_COUNT             7
#define BTN_DELAY              0xFF

int main(void)
{
	static uint8_t btn_pins[BTN_COUNT] = { 0, 1, 3, 4, 5, 6, 7 };
	static uint8_t btn_codes[BTN_COUNT] = { 1, 3, 7, 2, 6, 8, 4 };
	static uint8_t btn_timer[BTN_COUNT];

	IRMP_DATA irmp_data;
	irsnd_init();
	irmp_data.protocol = IRMP_RC5_PROTOCOL;
	irmp_data.address = 0x00FF;
	irmp_data.flags = 0;
	OCR1A = (F_CPU / F_INTERRUPTS) - 1;
	TCCR1B = (1 << WGM12) | (1 << CS10);
	TIMSK = (1 << OCIE1A);
	PORTB = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) |
		(1 << 5) | (1 << 6) | (1 << 7);
	sei();
	for(;;)
	{
		uint8_t i;
		for(i = 0; i < BTN_COUNT; ++i)
		{
			if(BTN_IN & (1 << btn_pins[i]))
			{
				if(btn_timer[i])
				{
					--btn_timer[i];
				}
			}
			else
			{
				if(btn_timer[i] == 0)
				{
					btn_timer[i] = BTN_DELAY;
					irmp_data.command = btn_codes[i];
					irsnd_send_data(&irmp_data, TRUE);
				}
			}
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	irsnd_ISR();
}

