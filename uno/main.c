#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "ws2812.c"

#define ARRLEN(x)             (sizeof(x) / sizeof(*x))

/* RC5 */
#define RC5_IN                PIND
#define RC5_PIN              7
#define RC5_TIME      1.778e-3 /* 1.778 _ms */
#define RC5_PULSE_MIN         (uint8_t)(F_CPU / 512 * RC5_TIME * 0.4 + 0.5)
#define RC5_PULSE_1_2         (uint8_t)(F_CPU / 512 * RC5_TIME * 0.8 + 0.5)
#define RC5_PULSE_MAX         (uint8_t)(F_CPU / 512 * RC5_TIME * 1.2 + 0.5)

static volatile uint16_t rc5_data;


/* Remote Control Buttons */
#define BTN_MODE_SMILEY      1
#define BTN_MODE_SNAKE       3
#define BTN_MODE_TETRIS      7

#define BTN_UP_PRESSED       2
#define BTN_RIGHT_PRESSED    6
#define BTN_DOWN_PRESSED     8
#define BTN_LEFT_PRESSED     4


/* UART */
#define UART_BAUD         9600
#define UART_PRESCALER        (uint16_t)(F_CPU / UART_BAUD / 16 - 0.5)

int16_t uart_rx(void);


/* Timer */
static volatile uint32_t _ms;


/* Mode */
enum { MODE_SMILEY, MODE_SNAKE, MODE_TETRIS } static _mode = MODE_SMILEY;


/* LED Board */
static color_t black = { 0, 0, 0 };


/* Smiley */
#define IMG_COUNT                   5
#define IMG_BYTES                  32
#define IMG_RANGE                    (255 / IMG_COUNT)

static void led_image(const uint8_t *i, color_t *fg, color_t *bg);
static void img_value(uint8_t v);

static uint8_t _avg = 255;
static uint32_t _sum;
static uint16_t _count;

static const uint8_t img[IMG_COUNT * IMG_BYTES] PROGMEM =
{
	0x00, 0x00, 0x00, 0x38, 0x0C, 0x3C, 0x1E, 0x0E, 0x1E, 0x06, 0x0C, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x0C, 0x06, 0x1E, 0x06, 0x1E, 0x0E, 0x0C, 0x3C, 0x00, 0x38, 0x00, 0x00, /* :(( */
	0x00, 0x00, 0x00, 0x18, 0x0C, 0x1C, 0x1E, 0x0C, 0x1E, 0x0C, 0x0C, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x0C, 0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x0C, 0x1C, 0x00, 0x18, 0x00, 0x00, /* :(  */
	0x00, 0x00, 0x00, 0x0C, 0x0C, 0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x0C, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x0C, 0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x0C, 0x0C, 0x00, 0x0C, 0x00, 0x00, /* :|  */
	0x00, 0x00, 0x00, 0x0C, 0x0C, 0x1C, 0x1E, 0x18, 0x1E, 0x18, 0x0C, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x0C, 0x18, 0x1E, 0x18, 0x1E, 0x18, 0x0C, 0x1C, 0x00, 0x0C, 0x00, 0x00, /* :)  */
	0x00, 0x00, 0x00, 0x0E, 0x0C, 0x1E, 0x1E, 0x38, 0x1E, 0x30, 0x0C, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x0C, 0x30, 0x1E, 0x30, 0x1E, 0x38, 0x0C, 0x1E, 0x00, 0x0E, 0x00, 0x00, /* :)) */
};


/* Snake */
#define SNAKE_INITIAL_LEN    4
#define SNAKE_MAX_LEN       16
#define SNAKE_MS_UPDATE    240

static void draw_snake(void);
static void draw_food(void);
static void random_food(void);
static void snake_init(void);
static uint8_t snake_update(void);
static void snake_advance(void);

enum
{
	PAUSE,
	UP,
	DOWN,
	LEFT,
	RIGHT
} _dir;

struct
{
	int8_t x, y;
	uint8_t color;
} _food;

struct
{
	struct { int8_t x, y; } blocks[SNAKE_MAX_LEN];
	uint8_t len;
	uint8_t color;
} static _snake;

static color_t _snake_colors[7] =
{
	{ 255,   0,   0 }, /* Red */
	{   0, 255,   0 }, /* Green */
	{   0,   0, 255 }, /* Blue */
	{ 255, 255,   0 }, /* Yellow */
	{ 255,   0, 255 }, /* Purple */
	{   0, 255, 255 }, /* Cyan */
	{ 255, 128,   0 }, /* Orange */
};

static uint16_t _snake_update_ticks;


/* Tetris */
#define FALL_SPEED_DEFAULT 350
#define ROTATE_RIGHT         1
#define ROTATE_LEFT          1

static void piece_undraw(void);
static void piece_draw(void);
static void piece_rotate_left(void);
static void piece_rotate_right(void);
static void piece_move_left(void);
static void piece_move_right(void);
static void piece_next(void);
static uint8_t piece_valid(void);
static void piece_to_field(void);

static int8_t field_get(int8_t x, int8_t y);
static void field_clear(void);
static void field_rows(void);

static uint8_t _field[LED_PIXELS];
static volatile uint16_t _tetris_update_ticks;

struct
{
	int8_t x, y, rotation;
	enum { I, J, L, O, S, T, Z } type;
} _piece;

struct
{
	uint16_t blocks[4];
	color_t color;
} _pieces[] =
{
	{
		/* I */
		{ 0x0F00, 0x2222, 0x00F0, 0x4444 },
		{ 0x00, 0xFF, 0xFF } /* Cyan */
	},
	{
		/* J */
		{ 0x44C0, 0x8E00, 0x6440, 0x0E20 },
		{ 0x00, 0x00, 0xFF } /* Blue */
	},
	{
		/* L */
		{ 0x4460, 0x0E80, 0xC440, 0x2E00 },
		{ 0xFF, 0x7F, 0x00 } /* Orange */
	},
	{
		/* O */
		{ 0xCC00, 0xCC00, 0xCC00, 0xCC00 },
		{ 0xFF, 0xFF, 0x00 } /* Yellow */
	},
	{
		/* S */
		{ 0x06C0, 0x8C40, 0x6C00, 0x4620 },
		{ 0x00, 0xFF, 0x00 } /* Green */
	},
	{
		/* T */
		{ 0x0E40, 0x4C40, 0x4E00, 0x4640 },
		{ 0xFF, 0x00, 0xFF } /* Purple */
	},
	{
		/* Z */
		{ 0x0C60, 0x4C80, 0xC600, 0x2640 },
		{ 0xFF, 0x00, 0x00 } /* Red */
	}
};


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
	char buf[3], *p = buf;
	char s[8];
	int16_t c;

	uint32_t ticks = 0;
	uint16_t i;

	led_clear(&black);
	led_update();

	/* MS Timer */
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 250;
	TIMSK0 = (1 << OCIE0A);

	/* RC5 Timer */
	TCCR2B = (1 << CS22) | (1 << CS21);
	TIMSK2 = (1 << TOIE2);

	/* UART Receiver */
	UBRR0 = UART_PRESCALER;
	UCSR0A = 0;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	sei();
	img_value(_avg);

	for(;;)
	{
		if((c = uart_rx()) >= 0)
		{
			if(c == '\n')
			{
				uint8_t val;
				*p = '\0';
				val = strtol(buf, NULL, 16);
				p = buf;
				_sum += val;
				++_count;
				_avg = _sum / _count;
				if(_mode == MODE_SMILEY)
				{
					img_value(_avg);
				}
			}
			else if(p < buf + ARRLEN(buf))
			{
				*p++ = c;
			}
		}

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

			switch(i)
			{
			case BTN_MODE_SMILEY:
				_mode = MODE_SMILEY;
				img_value(_avg);
				break;

			case BTN_MODE_SNAKE:
				_mode = MODE_SNAKE;
				snake_init();
				led_update();
				break;

			case BTN_MODE_TETRIS:
				_mode = MODE_TETRIS;
				_tetris_update_ticks = FALL_SPEED_DEFAULT;
				field_clear();
				led_clear(&black);
				piece_next();
				break;

			default:
				if(_mode == MODE_SNAKE)
				{
					switch(i)
					{
					case BTN_UP_PRESSED:
						_dir = UP;
						break;

					case BTN_RIGHT_PRESSED:
						_dir = RIGHT;
						break;

					case BTN_DOWN_PRESSED:
						_dir = DOWN;
						break;

					case BTN_LEFT_PRESSED:
						_dir = LEFT;
						break;
					}
				}
				else if(_mode == MODE_TETRIS)
				{
					switch(i)
					{
					case BTN_UP_PRESSED:
						piece_undraw();
						piece_rotate_right();
						piece_draw();
						led_update();
						break;

					case BTN_RIGHT_PRESSED:
						piece_undraw();
						piece_move_right();
						piece_draw();
						led_update();
						break;

					case BTN_DOWN_PRESSED:
						piece_undraw();
						piece_rotate_left();
						piece_draw();
						led_update();
						break;

					case BTN_LEFT_PRESSED:
						piece_undraw();
						piece_move_left();
						piece_draw();
						led_update();
						break;
					}
				}
				break;
			}
		}


		if(_mode == MODE_SNAKE)
		{
			if(_ms >= ticks + _snake_update_ticks)
			{
				ticks = _ms;
				if(snake_update())
				{
					snake_init();
				}

				led_update();
			}
		}
		else if(_mode == MODE_TETRIS)
		{
			if(_ms >= ticks + _tetris_update_ticks)
			{
				ticks = _ms;
				piece_undraw();
				++_piece.y;
				if(!piece_valid())
				{
					if(_piece.y <= 0)
					{
						_tetris_update_ticks = FALL_SPEED_DEFAULT;
						piece_next();
						field_clear();
					}
					else
					{
						--_piece.y;
						piece_to_field();
						field_rows();
						piece_next();
					}
				}

				piece_draw();
				led_update();
			}
		}
	}

	return 0;
}


/* UART */
int16_t uart_rx(void)
{
	if(!(UCSR0A & (1 << RXC0)))
	{
		return -1;
	}

	return UDR0;
}


/* MS Timer */
ISR(TIMER0_COMPA_vect)
{
	++_ms;
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


/* Smiley */
static void led_image(const uint8_t *i, color_t *fg, color_t *bg)
{
	uint8_t x, y;
	led_clear(bg);
	for(y = 0; y < LED_SIZE; ++y)
	{
		for(x = 0; x < 8; ++x)
		{
			if((pgm_read_byte(i + 2 * y) >> x) & 1)
			{
				led_pixel(y, x, fg);
			}

			if((pgm_read_byte(i + 2 * y + 1) >> x) & 1)
			{
				led_pixel(y, x + 8, fg);
			}
		}
	}

	led_update();
}

static void img_value(uint8_t v)
{
	color_t fg;
	uint8_t i, m = 0;
	for(i = 0; i < IMG_COUNT; ++i)
	{
		if(v >= m && v <= m + IMG_RANGE)
		{
			break;
		}

		m += IMG_RANGE;
	}

	if(i >= IMG_COUNT)
	{
		i = IMG_COUNT - 1;
	}

	fg.B = 0;
	if(v < 128)
	{
		fg.R = 255;
		fg.G = v;
	}
	else
	{
		fg.R = 255 - v;
		fg.G = 255;
	}

	led_image(img + i * IMG_BYTES, &fg, &black);
}


/* Snake */
static void draw_snake(void)
{
	uint8_t i;
	for(i = 0; i < _snake.len; ++i)
	{
		led_pixel(_snake.blocks[i].x, _snake.blocks[i].y,
			_snake_colors + _snake.color);
	}
}

static void draw_food(void)
{
	led_pixel(_food.x, _food.y, _snake_colors + _food.color);
}

static void random_food(void)
{
	uint8_t i;
	_food.x = -1;
	while(_food.x == -1)
	{
		_food.x = 1 + (rand() % (LED_SIZE - 2));
		_food.y = 1 + (rand() % (LED_SIZE - 2));
		_food.color = rand() % 7;
		if(_food.color == _snake.color)
		{
			_food.x = -1;
			continue;
		}

		for(i = 0; i < _snake.len; ++i)
		{
			if(_food.x == _snake.blocks[i].x ||
				_food.y == _snake.blocks[i].y)
			{
				_food.x = -1;
				break;
			}
		}
	}
}

static void snake_init(void)
{
	uint8_t i;
	_dir = 0;
	_snake_update_ticks = SNAKE_MS_UPDATE;
	led_clear(&black);
	_snake.len = SNAKE_INITIAL_LEN;
	for(i = 0; i < SNAKE_INITIAL_LEN; ++i)
	{
		_snake.blocks[i].x = i;
		_snake.blocks[i].y = 0;
	}

	_snake.color = rand() % 7;
	random_food();
	draw_snake();
	draw_food();
}

static uint8_t snake_update(void)
{
	if(_dir)
	{
		uint8_t i;
		/* Undraw Snake */
		for(i = 0; i < _snake.len; ++i)
		{
			led_pixel(_snake.blocks[i].x, _snake.blocks[i].y, &black);
		}

		/* Undraw Food */
		led_pixel(_food.x, _food.y, &black);

		for(i = 0; i < _snake.len - 1; ++i)
		{
			_snake.blocks[i].x = _snake.blocks[i + 1].x;
			_snake.blocks[i].y = _snake.blocks[i + 1].y;
		}

		snake_advance();

		if(_snake.blocks[_snake.len - 1].x < 0 ||
			_snake.blocks[_snake.len - 1].x >= LED_SIZE ||
			_snake.blocks[_snake.len - 1].y < 0 ||
			_snake.blocks[_snake.len - 1].y >= LED_SIZE)
		{
			return 1;
		}

		for(i = 0; i < _snake.len - 1; ++i)
		{
			if(_snake.blocks[_snake.len - 1].x == _snake.blocks[i].x &&
				_snake.blocks[_snake.len - 1].y == _snake.blocks[i].y)
			{
				return 1;
			}
		}

		if(_food.x == _snake.blocks[_snake.len - 1].x &&
			_food.y == _snake.blocks[_snake.len - 1].y)
		{
			if(_snake.len + 1 < SNAKE_MAX_LEN)
			{
				++_snake.len;
				_snake.blocks[_snake.len - 1].x =
					_snake.blocks[_snake.len - 2].x;

				_snake.blocks[_snake.len - 1].y =
					_snake.blocks[_snake.len - 2].y;

				snake_advance();
			}

			if(_snake_update_ticks > 20)
			{
				--_snake_update_ticks;
			}

			_snake.color = _food.color;
			random_food();
		}

		draw_snake();
		draw_food();
	}

	return 0;
}

static void snake_advance(void)
{
	switch(_dir)
	{
	case UP:
		--_snake.blocks[_snake.len - 1].y;
		break;

	case DOWN:
		++_snake.blocks[_snake.len - 1].y;
		break;

	case LEFT:
		--_snake.blocks[_snake.len - 1].x;
		break;

	case RIGHT:
		++_snake.blocks[_snake.len - 1].x;
		break;

	default:
		break;
	}
}


/* Tetris */
static void piece_undraw(void)
{
	int8_t row, col;
	uint16_t bit, blocks;
	col = row = 0;
	blocks = _pieces[_piece.type].blocks[_piece.rotation];
	for(bit = 0x8000; bit > 0; bit >>= 1)
	{
		if(blocks & bit)
		{
			led_pixel(_piece.x + col, _piece.y + row, &black);
		}

		if(++col == 4)
		{
			col = 0;
			++row;
		}
	}
}

static void piece_draw(void)
{
	int8_t row, col;
	uint16_t bit, blocks;
	col = row = 0;
	blocks = _pieces[_piece.type].blocks[_piece.rotation];
	for(bit = 0x8000; bit > 0; bit >>= 1)
	{
		if(blocks & bit)
		{
			led_pixel(_piece.x + col, _piece.y + row,
				&_pieces[_piece.type].color);
		}

		if(++col == 4)
		{
			col = 0;
			++row;
		}
	}
}

#if defined(ROTATE_RIGHT) && ROTATE_RIGHT
static void piece_rotate_right(void)
{
	if(++_piece.rotation == 4)
	{
		_piece.rotation = 0;
	}

	if(!piece_valid())
	{
		if(--_piece.rotation == -1)
		{
			_piece.rotation = 3;
		}
	}
}
#endif

#if defined(ROTATE_LEFT) && ROTATE_LEFT
static void piece_rotate_left(void)
{
	if(--_piece.rotation == -1)
	{
		_piece.rotation = 3;
	}

	if(!piece_valid())
	{
		if(++_piece.rotation == 4)
		{
			_piece.rotation = 0;
		}
	}
}
#endif

static void piece_move_left(void)
{
	--_piece.x;
	if(!piece_valid())
	{
		++_piece.x;
	}
}

static void piece_move_right(void)
{
	++_piece.x;
	if(!piece_valid())
	{
		--_piece.x;
	}
}

static void piece_next(void)
{
	static uint8_t bag[7];
	static uint8_t idx = 7;
	uint8_t i, j, v, unique;
	if(idx == 7)
	{
		for(i = 0; i < 7; ++i)
		{
			unique = 0;
			while(!unique)
			{
				unique = 1;
				v = rand() % 7;
				for(j = 0; j < i; ++j)
				{
					if(bag[j] == v)
					{
						unique = 0;
						break;
					}
				}
			}

			bag[i] = v;
		}

		idx = 0;
	}

	_piece.x = LED_SIZE / 2 - 2;
	_piece.y = -4;
	_piece.rotation = 0;
	_piece.type = bag[idx++];
}

static uint8_t piece_valid(void)
{
	int8_t row, col;
	uint16_t bit, blocks;
	row = col = 0;
	blocks = _pieces[_piece.type].blocks[_piece.rotation];
	for(bit = 0x8000; bit > 0; bit >>= 1)
	{
		if((blocks & bit) && field_get(_piece.x + col, _piece.y + row))
		{
			return 0;
		}

		if(++col == 4)
		{
			col = 0;
			++row;
		}
	}

	return 1;
}

static void piece_to_field(void)
{
	int8_t row, col;
	uint16_t bit, blocks;
	row = col = 0;
	blocks = _pieces[_piece.type].blocks[_piece.rotation];
	for(bit = 0x8000; bit > 0; bit >>= 1)
	{
		if(blocks & bit)
		{
			int8_t x, y;
			x = _piece.x + col;
			y = _piece.y + row;
			_field[y * LED_SIZE + x] = _piece.type + 1;
			led_pixel(x, y, &_pieces[_piece.type].color);
		}

		if(++col == 4)
		{
			col = 0;
			++row;
		}
	}
}

static int8_t field_get(int8_t x, int8_t y)
{
	if(x >= 0 && x < LED_SIZE && y < LED_SIZE)
	{
		if(y < 0)
		{
			return 0;
		}
		else
		{
			return _field[y * LED_SIZE + x];
		}
	}

	return -1;
}

static void field_clear(void)
{
	uint16_t i;
	for(i = 0; i < LED_SIZE * LED_SIZE; ++i)
	{
		_field[i] = 0;
	}

	led_clear(&black);
	led_update();
}

static void field_rows(void)
{
	int8_t x, y, i, j, v;
	--_piece.y;
	v = 0;
	for(y = 0; y < LED_SIZE; ++y)
	{
		for(x = 0; x < LED_SIZE; ++x)
		{
			if(!_field[y * LED_SIZE + x])
			{
				break;
			}
		}

		if(x == LED_SIZE)
		{
			v = 1;
			for(j = y; j > 0; --j)
			{
				for(i = 0; i < LED_SIZE; ++i)
				{
					_field[j * LED_SIZE + i] =
						_field[(j - 1) * LED_SIZE + i];
				}
			}
		}
	}

	/* Redraw */
	if(v)
	{
		led_clear(&black);
		for(y = 0; y < LED_SIZE; ++y)
		{
			for(x = 0; x < LED_SIZE; ++x)
			{
				if((v = _field[y * LED_SIZE + x]))
				{
					--v;
					led_pixel(x, y, &_pieces[v].color);
				}
			}
		}
	}
}

