#include "stm32f4_HD44780.h"

HD44780_Options_t HD44780_Opts;

//Funkcja inicjalizacja wyswietlacza
void HD44780_Init(uint8_t cols, uint8_t rows) 
{
	//Inicjalizacja opoznienia
	DELAY_Init();
	
	//Inicjalizacja pinow, wyprowadzen mikrokontrolera
	HD44780_InitPins();
	
	//Opóznienie minimum 40ms
	Delay(45000);
	
	//Przypisanie zmiennej HD44780_Opts.Rows zmiennej rows czyli rzedy
	HD44780_Opts.Rows = rows;
	//Przypisanie zmiennej HD44780_Opts.Cols zmiennej cols czyli kolumny
	HD44780_Opts.Cols = cols;
	
	//Przypisanie startowej pozycji x na 0
	HD44780_Opts.currentX = 0;
	//Przypisanie startowej pozycji y na 0
	HD44780_Opts.currentY = 0;
	
	//Ustawienie funkcji wyswietlacza
	//Tryb 4bitowy 0x00 | 5x8 0x00 | 1Line 0x00
	HD44780_Opts.DisplayFunction = HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_1LINE;
	if (rows > 1) 
	{
		//Wybranie dwóch linii 0x08
		HD44780_Opts.DisplayFunction |= HD44780_2LINE;
	}
	
	//Wyslanie komendy 0011, po niej nalezy odczekac przynajmniej 4,1 ms
	HD44780_Cmd4bit(0x03);
	Delay(4500);
	
	//Druga próba
	HD44780_Cmd4bit(0x03);
	Delay(200);
	
	//Trzecia próba
	HD44780_Cmd4bit(0x03);
	
	//Wybieramy 4 bitowy interfejs, wyslanie komendy 0010
	//Ustawienie 4 bitowego interfejsu
	HD44780_Cmd4bit(0x02);
	Delay(100);
	
	//Ustawienie lini, sposobu wyswietlania
	HD44780_Cmd(HD44780_FUNCTIONSET | HD44780_Opts.DisplayFunction);

	//Wlacz wyswietlacz 0x04
	HD44780_Opts.DisplayControl = HD44780_DISPLAYON;
	HD44780_DisplayOn();

	//Czyszczenie wyswietlacza
	HD44780_Clear();

	//Ustawienie domyslnej czcionki
	HD44780_Opts.DisplayMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
	HD44780_Cmd(HD44780_ENTRYMODESET | HD44780_Opts.DisplayMode);

	Delay(4500);
}

//Inicjalizacja pinów wyswietlacza
void HD44780_InitPins(void) 
{
	GPIO_InitTypeDef GPIOINIT;

	//Wlaczenie zegarów dla poszczególnych pinów
	RCC_AHB1PeriphClockCmd(HD44780_RS_RCC | HD44780_E_RCC, ENABLE);
	RCC_AHB1PeriphClockCmd(HD44780_D4_RCC | HD44780_D5_RCC | HD44780_D6_RCC | HD44780_D7_RCC, ENABLE);
	
	//RS
	GPIOINIT.GPIO_Pin = HD44780_RS_PIN;
	//Pin jako wyjœcie
	GPIOINIT.GPIO_Mode = GPIO_Mode_OUT;
	//Typ pushPull
	GPIOINIT.GPIO_OType = GPIO_OType_PP;
	//bez podci¹gniêcia
	GPIOINIT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//100MHz taktowanie
	GPIOINIT.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(HD44780_RS_PORT, &GPIOINIT);
	//Po inicjalizacji stan niski na wszystkie linie w celu zresetowania uk³adu
	GPIO_WriteBit(HD44780_RS_PORT, HD44780_RS_PIN, 0x00);
	
	//E
	GPIOINIT.GPIO_Pin = HD44780_E_PIN;
	GPIOINIT.GPIO_Mode = GPIO_Mode_OUT;
	GPIOINIT.GPIO_OType = GPIO_OType_PP;
	GPIOINIT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOINIT.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(HD44780_E_PORT, &GPIOINIT);
	GPIO_WriteBit(HD44780_E_PORT, HD44780_E_PIN, 0x00);
	
	//D4
	GPIOINIT.GPIO_Pin = HD44780_D4_PIN;
	GPIOINIT.GPIO_Mode = GPIO_Mode_OUT;
	GPIOINIT.GPIO_OType = GPIO_OType_PP;
	GPIOINIT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOINIT.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(HD44780_D4_PORT, &GPIOINIT);
	GPIO_WriteBit(HD44780_D4_PORT, HD44780_D4_PIN, 0x00);
	
	//D5
	GPIOINIT.GPIO_Pin = HD44780_D5_PIN;
	GPIOINIT.GPIO_Mode = GPIO_Mode_OUT;
	GPIOINIT.GPIO_OType = GPIO_OType_PP;
	GPIOINIT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOINIT.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(HD44780_D5_PORT, &GPIOINIT);
	GPIO_WriteBit(HD44780_D5_PORT, HD44780_D5_PIN, 0x00);
	
	//D6
	GPIOINIT.GPIO_Pin = HD44780_D6_PIN;
	GPIOINIT.GPIO_Mode = GPIO_Mode_OUT;
	GPIOINIT.GPIO_OType = GPIO_OType_PP;
	GPIOINIT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOINIT.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(HD44780_D6_PORT, &GPIOINIT);
	GPIO_WriteBit(HD44780_D6_PORT, HD44780_D6_PIN, 0x00);
	
	//D7
	GPIOINIT.GPIO_Pin = HD44780_D7_PIN;
	GPIOINIT.GPIO_Mode = GPIO_Mode_OUT;
	GPIOINIT.GPIO_OType = GPIO_OType_PP;
	GPIOINIT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOINIT.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(HD44780_D7_PORT, &GPIOINIT);
	GPIO_WriteBit(HD44780_D7_PORT, HD44780_D7_PIN, 0x00);
}
/*	Czyszczenie wyswietlacza */
void HD44780_Clear(void)
{
	HD44780_Cmd(HD44780_CLEARDISPLAY);
	HD44780_Delay(3000);
}
//Wysy³a komendy do wyœwietlacza
void HD44780_Cmd(uint8_t cmd) 
{
	//Sygna³ niski na liniê RS
	GPIO_WriteBit(HD44780_RS_PORT, HD44780_RS_PIN, 0x00);
	
	//Wys³anie starszej czêœci danych
	HD44780_Cmd4bit(cmd >> 4);
	//Wys³anie m³odszej czêœci
	HD44780_Cmd4bit(cmd & 0x0F);
}
//Wys³anie danych do wyœwietlacza
void HD44780_Data(uint8_t data) 
{
	GPIO_WriteBit(HD44780_RS_PORT, HD44780_RS_PIN, Bit_SET);
	
	//Wys³anie starszej czêœci danych
	HD44780_Cmd4bit(data >> 4);
	//Wys³anie m³odszej czêœci
	HD44780_Cmd4bit(data & 0x0F);
}
/*	Wysyla 4 bitowe komendy do wyswietlacza */
void HD44780_Cmd4bit(uint8_t cmd) 
{
	//Wpisanie danych do wyœwietlacza, jeœli wpisywane dane s¹ ró¿ne od zera wtedy stan wysoki
	//Jeœli nie s¹ wtedy stan niski
	GPIO_WriteBit(HD44780_D7_PORT, HD44780_D7_PIN, (cmd & 0x08) != 0 ? Bit_SET : Bit_RESET);
	GPIO_WriteBit(HD44780_D6_PORT, HD44780_D6_PIN, (cmd & 0x04) != 0 ? Bit_SET : Bit_RESET);
	GPIO_WriteBit(HD44780_D5_PORT, HD44780_D5_PIN, (cmd & 0x02) != 0 ? Bit_SET : Bit_RESET);
	GPIO_WriteBit(HD44780_D4_PORT, HD44780_D4_PIN, (cmd & 0x01) != 0 ? Bit_SET : Bit_RESET);
	//Wystawienie sygna³u wysokiego na pin E
	GPIO_WriteBit(HD44780_E_PORT, HD44780_E_PIN, Bit_SET);
	//Odczekanie 20us
	Delay(20);
	//Wystawienie stanu niskiego na pin E
	GPIO_WriteBit(HD44780_E_PORT, HD44780_E_PIN, Bit_RESET);
	//Odczekanie 20us
	Delay(20);
}
/*	Ustawia kursor na wybranej pozycji */
void HD44780_CursorSet(uint8_t col, uint8_t row) 
	{
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	
	//Go to the beginning
	if (row >= HD44780_Opts.Rows) 
	{
		row = 0;
	}
	
	HD44780_Opts.currentX = col;
	HD44780_Opts.currentY = row;
	
	HD44780_Cmd(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}
/*	Wystawia string na zadana pozycje*/
void HD44780_Puts(uint8_t x, uint8_t y, char* str) 
{
	HD44780_CursorSet(x, y);
	while (*str) 
	{
		if (HD44780_Opts.currentX >= HD44780_Opts.Cols) 
		{
			HD44780_Opts.currentX = 0;
			HD44780_Opts.currentY++;
			HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		}
		if (*str == '\n') 
		{
			HD44780_Opts.currentY++;
			HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		} 
		else if (*str == '\r') 
		{
			HD44780_CursorSet(0, HD44780_Opts.currentY);
		} 
		else 
		{
			HD44780_Data(*str);
			HD44780_Opts.currentX++;
		}
		str++;
	}
}
/*	Wlacza wyswietlacz */
void HD44780_DisplayOn(void) 
{
	HD44780_Opts.DisplayControl |= HD44780_DISPLAYON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}
/*	Wylacza wyswietlacz */
void HD44780_DisplayOff(void) 
{
	HD44780_Opts.DisplayControl &= ~HD44780_DISPLAYON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}
/*	Wlacza miganie wskaznikiem */
void HD44780_BlinkOn(void) 
{
	HD44780_Opts.DisplayControl |= HD44780_BLINKON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}
/*	Wylacza miganie */
void HD44780_BlinkOff(void) 
{
	HD44780_Opts.DisplayControl &= ~HD44780_BLINKON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}
/*	Wlacza kursor	*/
void HD44780_CursorOn(void) 
{
	HD44780_Opts.DisplayControl |= HD44780_CURSORON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}
/*	Wylacza kursor	*/
void HD44780_CursorOff(void) 
{
	HD44780_Opts.DisplayControl &= ~HD44780_CURSORON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}
/* Przewiniecie w lewo	*/
void HD44780_ScrollLeft(void) 
{
	HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}
/*	Przewiniecie w prawo	*/
void HD44780_ScrollRight(void) 
{
	HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}
/*	Zapisuje dane do okreslonego miejsca na wyswietlaczu, od 0 - 7*/
void HD44780_CreateChar(uint8_t location, uint8_t *data) 
{
	uint8_t i;
	location &= 0x07;	//8 Free locations for custom chars
	HD44780_Cmd(HD44780_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) 
	{
		HD44780_Data(data[i]);
	}
}
/*	Wyswietla przeslana tablice znaków	*/
void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location) 
{
	HD44780_CursorSet(x, y);
	HD44780_Data(location);
}

