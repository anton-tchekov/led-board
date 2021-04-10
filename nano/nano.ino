/* server ip: 192.168.4.1 */
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <SoftwareSerial.h>

#define ARRLEN(A)             (sizeof(A) / sizeof(*A))

#define SSID                  "LEDBOARD"
#define SERVER_PORT           "80" /* HTTP */
#define BAUDRATE_USB      9600
#define BAUDRATE_WLAN    19200

#define ESP8266_RX_PIN      11
#define ESP8266_TX_PIN      12
#define LED_WLAN_PIN        13

#define RESPONSE_LEN         3

static const char _str_response[] PROGMEM = "ack";
static char _str_ok[] = "OK";
static char _cmd_buf[128];

SoftwareSerial _esp8266(ESP8266_RX_PIN, ESP8266_TX_PIN);

void setup(void);
void loop(void);
static void send_response(const char *content, uint16_t len);
static void serial_print_p(const char *s);
static uint8_t esp8266_command(char *s, char *e);
static void panic(void);

void setup(void)
{
	pinMode(LED_WLAN_PIN, OUTPUT);
	Serial.begin(BAUDRATE_USB);
	while(!Serial);

	_esp8266.begin(BAUDRATE_WLAN);
	while(!_esp8266);

	_esp8266.setTimeout(5000);
	if(esp8266_command("AT+RST", "ready"))
	{
		panic();
	}

	_esp8266.setTimeout(1000);
	if(esp8266_command("AT+CWMODE=2", _str_ok))
	{
		panic();
	}

	_esp8266.setTimeout(20000);
	if(esp8266_command("AT+CWSAP=\"" SSID "\",\"\",5,0", _str_ok))
	{
		panic();
	}

	_esp8266.setTimeout(1000);
	if(esp8266_command("AT+CIPMODE=0", _str_ok) ||
		esp8266_command("AT+CIPMUX=0", _str_ok) ||
		esp8266_command("AT+CIPMUX=1", _str_ok) ||
		esp8266_command("AT+CIPSERVER=1," SERVER_PORT, _str_ok))
	{
		panic();
	}

	digitalWrite(LED_WLAN_PIN, HIGH);
}

void loop(void)
{
	if(_esp8266.available() && _esp8266.find("+IPD,"))
	{
		uint8_t i;
		int16_t c = -1;
		char cid[8], conv[8];
		char *p = cid;
		while(!isdigit(_esp8266.peek())) ;
		while(isdigit(_esp8266.peek()))
		{
			*p++ = _esp8266.read();
		}

		*p = '\0';
		for(i = 0; (c = _esp8266.read()) != '\n'; )
		{
			if(c != -1 && i < ARRLEN(_cmd_buf))
			{
				_cmd_buf[i++] = (char)c;
			}
		}

		_cmd_buf[i] = '\0';
		if((p = strstr(_cmd_buf, "input?")))
		{
			uint16_t n;
			char *q;
			/* strlen("input?") = 6 */
			for(p += 6, q = p; *q != ' '; ++q) ;
			*q = '\0';
			n = atoi(p);
			if(n < 256)
			{
				itoa(n, conv, 16);
				if(strlen(conv) == 1)
				{
					Serial.write('0');
				}

				Serial.write(conv);
				Serial.write('\n');
			}

			send_response(cid, _str_response, RESPONSE_LEN);
		}
	}
}

static void send_response(char *cid, const char *content, uint16_t len)
{
	char buf[8];
	uint16_t i;
	strcpy(_cmd_buf, "AT+CIPSEND=");
	strcat(_cmd_buf, cid);
	strcat(_cmd_buf, ",");
	strcat(_cmd_buf, itoa(len, buf, 10));
	if(esp8266_command(_cmd_buf, ">"))
	{
		return;
	}

	for(i = 0; i < len; ++i)
	{
		_esp8266.write(pgm_read_byte(content + i));
	}

	if(!_esp8266.find("SEND OK"))
	{
		return;
	}

	/*strcpy(_cmd_buf, "AT+CIPCLOSE=");
	strcat(_cmd_buf, cid);
	esp8266_command(_cmd_buf, _str_ok);*/
}

static void panic(void)
{
	for(;;)
	{
		digitalWrite(LED_WLAN_PIN, HIGH);
		delay(500);
		digitalWrite(LED_WLAN_PIN, LOW);
		delay(500);
	}
}

static uint8_t esp8266_command(char *s, char *e)
{
	_esp8266.println(s);
	return !_esp8266.findUntil(e, "ERROR");
}

