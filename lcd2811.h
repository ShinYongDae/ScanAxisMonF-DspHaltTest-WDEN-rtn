/*************************************************/
/*		  lcd28x.h 	RealSYS                	 	 */
/*************************************************/
#ifndef __LCD28X_INCLUDED_
#define __LCD28X_INCLUDED_

#include "..\def28x.h"

#define	OUTDIR_LCD2811;		EALLOW;	\
							GpioMuxRegs.GPBMUX.all &= 0xff0a;	\
       						GpioMuxRegs.GPBDIR.all |= 0x00f5;	\
							EDIS;       
#define	OUT_LCD2811(d)		(GpioDataRegs.GPBDAT.all=(GpioDataRegs.GPBDAT.all & 0xff00)|d)

#define	CON			0
#define	DAT			1

#define	LCD_PORT(x)	OUT_LCD2811(x)

char	asc_to_hex(char asc);
char	hex_to_asc(char hex);
WORD	hex2_to_decimal(BYTE b);
LONG	hex4_to_decimal(WORD w);
void	delay_us(unsigned int d);
void	delay_ms(unsigned int d);

void	init_lcd(void);
void 	LCD_out(char data);
void 	LCD_out2(char rs, char data);
void	lcd_clear(void);
void	lcd_clear_line(char line);
void	lcd_gotoxy(int x, int y);
void	lcd_cursor_on(void);
void	lcd_cursor_off(void);
void	lcd_putc(char c);
void	lcd_puts(char *str);

void	lcd_hex2(char c);
void	lcd_hex4(WORD w);
void	lcd_hex8(LONG l);
void	lcd_decimal_byte(BYTE b);
void	lcd_decimal_word(WORD w);  
void	lcd_decimal_int(WORD w);

#endif
