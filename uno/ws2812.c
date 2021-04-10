#define WS2812_OUT            PORTB
#define WS2812_DIR            DDRB

#define LED_SIZE            16
#define LED_PIXELS            (LED_SIZE * LED_SIZE)
#define LED_BYTES             (3 * LED_PIXELS)

#define W_ZERO_PULSE    350
#define W_ONE_PULSE     900
#define W_TOTAL_PERIOD 1250
#define W_FIXED_LOW       2
#define W_FIXED_HIGH      4
#define W_FIXED_TOTAL     8

#define W_ZERO_CYCLE \
	(((F_CPU / 1000) * W_ZERO_PULSE) / 1000000)

#define W_ONE_CYCLE \
	(((F_CPU / 1000) * W_ONE_PULSE + 500000) / 1000000)

#define W_TOTAL_CYCLE \
	(((F_CPU / 1000) * W_TOTAL_PERIOD + 500000) / 1000000)

#define W1 \
	(W_ZERO_CYCLE - W_FIXED_LOW)

#define W2 \
	(W_ONE_CYCLE - W_FIXED_HIGH - W1)

#define W3 \
	(W_TOTAL_CYCLE - W_FIXED_TOTAL - W1 - W2)

#if (W1 > 0)
#define W1_NOPS W1
#else
#define W1_NOPS  0
#endif

#define W_LOW_TIME \
	(((W1_NOPS + W_FIXED_LOW) * 1000000) / (F_CPU / 1000))

#if (W_LOW_TIME > 550)
#error "F_CPU to low"
#elif (W_LOW_TIME > 450)
#warning "Critical timing"
#endif

#if (W2 > 0)
#define W2_NOPS W2
#else
#define W2_NOPS  0
#endif

#if (W3 > 0)
#define W3_NOPS W3
#else
#define W3_NOPS  0
#endif

#define W_NOP1  "nop      \n\t"
#define W_NOP2  "rjmp .+0 \n\t"
#define W_NOP4  W_NOP2 W_NOP2
#define W_NOP8  W_NOP4 W_NOP4
#define W_NOP16 W_NOP8 W_NOP8

typedef struct COLOR { uint8_t R, G, B; } color_t;
static uint8_t _pixels[LED_BYTES];

static void led_update(void);
static void led_pixel(uint8_t x, uint8_t y, color_t *c);
static void led_clear(color_t *c);
static void ws2812(uint8_t *pixels, uint16_t count, uint8_t pin);

static void led_update(void)
{
	ws2812(_pixels, 128 * 3, 1);
	ws2812(_pixels + 128 * 3, 128 * 3, 2);
}

static void led_pixel(uint8_t x, uint8_t y, color_t *c)
{
	if(x < LED_SIZE && y < LED_SIZE)
	{
		uint16_t i;
		y = LED_SIZE - 1 - y;
		i = 3 * ((x % 2) ? ((LED_SIZE * x) + ((LED_SIZE - 1) - y)) :
			(LED_SIZE * x + y));
		_pixels[i] = c->G;
		_pixels[++i] = c->R;
		_pixels[++i] = c->B;
	}
}

static void led_clear(color_t *c)
{
	uint16_t i;
	for(i = 0; i < 3 * LED_PIXELS; ++i)
	{
		_pixels[i] = c->G;
		_pixels[++i] = c->R;
		_pixels[++i] = c->B;
	}
}

static void ws2812(uint8_t *pixels, uint16_t count, uint8_t pin)
{
	uint8_t b, c, h, l, s;
	h = (1 << pin);
	WS2812_DIR |= h;
	l = ~h & WS2812_OUT;
	h |= WS2812_OUT;
	s = SREG;
	asm volatile ("cli");
	while(count--)
	{
		b = *pixels++;
		asm volatile
		(
			"       ldi   %0,8  \n\t"
			"loop%=:            \n\t"
			"       out   %2,%3 \n\t"
#if (W1_NOPS & 1)
W_NOP1
#endif
#if (W1_NOPS & 2)
W_NOP2
#endif
#if (W1_NOPS & 4)
W_NOP4
#endif
#if (W1_NOPS & 8)
W_NOP8
#endif
#if (W1_NOPS & 16)
W_NOP16
#endif
			"       sbrs  %1,7  \n\t"
			"       out   %2,%4 \n\t"
			"       lsl   %1    \n\t"
#if (W2_NOPS & 1)
W_NOP1
#endif
#if (W2_NOPS & 2)
W_NOP2
#endif
#if (W2_NOPS & 4)
W_NOP4
#endif
#if (W2_NOPS & 8)
W_NOP8
#endif
#if (W2_NOPS & 16)
W_NOP16
#endif
			"       out   %2,%4 \n\t"
#if (W3_NOPS & 1)
W_NOP1
#endif
#if (W3_NOPS & 2)
W_NOP2
#endif
#if (W3_NOPS & 4)
W_NOP4
#endif
#if (W3_NOPS & 8)
W_NOP8
#endif
#if (W3_NOPS & 16)
W_NOP16
#endif
			"       dec   %0    \n\t"
			"       brne  loop%=\n\t"
			:	"=&d" (c)
			:	"r" (b),
				"I" (_SFR_IO_ADDR(WS2812_OUT)),
				"r" (h),
				"r" (l)
		);
	}

	SREG = s;
}

