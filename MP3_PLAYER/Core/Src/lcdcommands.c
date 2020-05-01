#include "lcdcommands.h"

void lcd_clear_1()
{
	lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("                    ");
}
void lcd_clear_2()
{
	lcd_send_cmd(0x80 | 0x40);
	lcd_send_string("                    ");
}
void lcd_clear_3()
{
	lcd_send_cmd(0x80 | 0x14);
	lcd_send_string("                    ");
}
void lcd_clear_4()
{
	lcd_send_cmd(0x80 | 0x54);
	lcd_send_string("                    ");
}
void lcd_cls()
{
	lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("                    ");
	lcd_send_cmd(0x80 | 0x40);
	lcd_send_string("                    ");
	lcd_send_cmd(0x80 | 0x14);
	lcd_send_string("                    ");
	lcd_send_cmd(0x80 | 0x54);
	lcd_send_string("                    ");
}
