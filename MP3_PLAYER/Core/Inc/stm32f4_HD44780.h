/*	Opis wyprowadzen wyswietlacza, wraz z podlaczeniem do STM32F4-Discovery
 *
 *	LCD		STM32F4						DESCRIPTION
 *
 *	GND		GND								Masa
 *	VCC		+5V								Zasilanie wyswietlacza
 *	V0		Potentiometer			Sterowanie kontrastem, podlaczenie do potencjometru
 *	RS		PB2								Wybór rejestru instrukcji wyswietlacza, (stan niski) albo rejestru danych(wysoki)
 *	RW		GND								Odczyt (Stan niski)/Zapis(Stan wysoki)
 *	E			PB7								Odblokowanie wyswietlacza (Enable Pin)
 *	D0		-									Data 0 – nie podlaczone
 *	D1		-									Data 1 - nie podlaczone
 *	D2		-									Data 2 - nie podlaczone
 *	D3		-									Data 3 - nie podlaczone
 *	D4		PC12							Data 4
 *	D5		PC13							Data 5
 *	D6		PC14							Data 6
 *	D7		PC15							Data 7 (D0-D7 Magistrala danych)
 *	A			+3V3							Zasilanie podswietlenia 3,3V lub 5V
 *	K			GND								Masa podswietlenia
 */
#ifndef HD44780_H
#define HD44780_H 100

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//#include "defines.h"
#include "stm32f4_delay.h"

//RS - Register select pin
#ifndef HD44780_RS_PIN
#define HD44780_RS_RCC				RCC_AHB1Periph_GPIOB
#define HD44780_RS_PORT				GPIOB
#define HD44780_RS_PIN				GPIO_Pin_2
#endif
//E - Enable pin
#ifndef HD44780_E_PIN
#define HD44780_E_RCC				RCC_AHB1Periph_GPIOB
#define HD44780_E_PORT				GPIOB
#define HD44780_E_PIN				GPIO_Pin_7
#endif

//D4 - Data 4 pin
#ifndef HD44780_D4_PIN
#define HD44780_D4_RCC				RCC_AHB1Periph_GPIOC
#define HD44780_D4_PORT				GPIOC
#define HD44780_D4_PIN				GPIO_Pin_12
#endif
//D5 - Data 5 pin
#ifndef HD44780_D5_PIN
#define HD44780_D5_RCC				RCC_AHB1Periph_GPIOC
#define HD44780_D5_PORT				GPIOC
#define HD44780_D5_PIN				GPIO_Pin_13
#endif
//D6 - Data 6 pin
#ifndef HD44780_D6_PIN
#define HD44780_D6_RCC				RCC_AHB1Periph_GPIOC
#define HD44780_D6_PORT				GPIOC
#define HD44780_D6_PIN				GPIO_Pin_14
#endif
//D7 - Data 7 pin
#ifndef HD44780_D7_PIN
#define HD44780_D7_RCC				RCC_AHB1Periph_GPIOC
#define HD44780_D7_PORT				GPIOC
#define HD44780_D7_PIN				GPIO_Pin_15
#endif

#define HD44780_RS_LOW				GPIO_WriteBit(HD44780_RS_PORT, HD44780_RS_PIN, Bit_RESET)
#define HD44780_RS_HIGH				GPIO_WriteBit(HD44780_RS_PORT, HD44780_RS_PIN, Bit_SET)
#define HD44780_E_LOW				GPIO_WriteBit(HD44780_E_PORT, HD44780_E_PIN, Bit_RESET)
#define HD44780_E_HIGH				GPIO_WriteBit(HD44780_E_PORT, HD44780_E_PIN, Bit_SET)

#define HD44780_E_BLINK				HD44780_E_HIGH; HD44780_Delay(20); HD44780_E_LOW; HD44780_Delay(20)
#define HD44780_Delay(x)			Delay(x)

//Komendy
#define HD44780_CLEARDISPLAY		0x01
#define HD44780_RETURNHOME			0x02
#define HD44780_ENTRYMODESET		0x04
#define HD44780_DISPLAYCONTROL		0x08
#define HD44780_CURSORSHIFT			0x10
#define HD44780_FUNCTIONSET			0x20
#define HD44780_SETCGRAMADDR		0x40
#define HD44780_SETDDRAMADDR		0x80

//Flagi dla wyœwietlania danych przychodz¹cych
#define HD44780_ENTRYRIGHT			0x00
#define HD44780_ENTRYLEFT			0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

//Flagi do wyœwietlania w³¹czenia/wy³¹czenia wyœwietlacza
#define HD44780_DISPLAYON			0x04
#define HD44780_CURSORON			0x02
#define HD44780_BLINKON				0x01

//Flagi dla przesuniêc kursora
#define HD44780_DISPLAYMOVE			0x08
#define HD44780_CURSORMOVE			0x00
#define HD44780_MOVERIGHT			0x04
#define HD44780_MOVELEFT			0x00

//Flagi dla ustawienia poszczegolnych trybów pracy wyœwietlacza
#define HD44780_8BITMODE			0x10
#define HD44780_4BITMODE			0x00
#define HD44780_2LINE				0x08
#define HD44780_1LINE				0x00
#define HD44780_5x10DOTS			0x04
#define HD44780_5x8DOTS				0x00

/*	Wewnetrzna struktura dla LCD	*/
typedef struct
{
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t currentX;
	uint8_t currentY;
} HD44780_Options_t;

/*	Inicjalizacja LCD
		uint8_t cols: szerokosc, os x
		uint8_t rows: wysokosc, os y			*/
extern void HD44780_Init(uint8_t cols, uint8_t rows);

/*	Wlacza wyswietlacz */
extern void HD44780_DisplayOn(void);

/*	Wylacza wyswietlacz */
extern void HD44780_DisplayOff(void);

/*	Czysci wyswietlacz */
extern void HD44780_Clear(void);

/*	Wystawia ciag znakow na wyswietlacz
		uint8_t x: x location
		uint8_t y: y location
		char* str: wzkaznik do stringa				*/
extern void HD44780_Puts(uint8_t x, uint8_t y, char* str);

/*	Wlacza miganie kursora */
extern void HD44780_BlinkOn(void);

/*	Wylacza miganie kursora	*/
extern void HD44780_BlinkOff(void);

/*	Pokazuje kursor kursor	*/
extern void HD44780_CursorOn(void);

/*	Chowa kursor	*/
extern void HD44780_CursorOff(void);

/*	Przesuwa wyswietlanie na lewo	*/
extern void HD44780_ScrollLeft(void);

/*	Przesuwa wyswietlanie a prawo */
extern void HD44780_ScrollRight(void);

/*	Tworzy tablice ze znakiem, dopuszczalne jest 8 znaków wlasnych w lokalizacji o 0-7 (uint8_t location)
		(uint8_t *data) jest to wskaznik do zapisanej tablicy */
extern void HD44780_CreateChar(uint8_t location, uint8_t* data);

/*	Wyswietla tablice znaków na wyswietlaczu
		uint8_t location - lokalizacja zapisanej tablicy 0-7	*/
extern void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

/* Inicjalizuje piny wyswietlacza	*/
extern void HD44780_InitPins(void);

/*	Wysyla komendy do wyswietlacza	*/
extern void HD44780_Cmd(uint8_t cmd);

/*	Wysyla 4 bitowe komendy do wyswietlacza	*/
extern void HD44780_Cmd4bit(uint8_t cmd);

/*	Wysyla dane do wyswietlacza */
extern void HD44780_Data(uint8_t data);

/*	Ustawia kursor w pozycji x oraz y na wyswietlaczu
		uint8_t col - pozycja x
		uint8_t row - pozycja y	*/
extern void HD44780_CursorSet(uint8_t col, uint8_t row);

#endif
