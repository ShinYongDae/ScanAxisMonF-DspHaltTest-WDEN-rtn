/* lcd2811.c */
#include "..\def28x.h"
#include "..\lcd2811.h"

/*********** LCD interface (4 bit) ************
				   7  6  5  4  3  2  1  0
	Port: Px	=  D7 D6 D5 D4 x  E  0  RS      
	RS = 0(Control), 1(Data)	

Support LCD
// 16 char x 2
// 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F	-- 1st
// 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F	-- 2nd

// 20char x 4
// 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13	-- 1st
// 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 53	-- 2nd
// 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20 21 22 23 24 25 26 27	-- 3rd
// 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F 60 61 62 63 64 65 66 67	-- 4th
***********************************************/

/**************** Data conversion function ************************/
char	asc_to_hex(char asc){
    if((asc >= '0') && (asc <= '9'))    return (asc - '0');
    else if((asc >= 'A') && (asc <= 'F'))   return (asc - 'A' + 0x0a);
    else if((asc >= 'a') && (asc <= 'f'))    return (asc - 'a' + 0x0a);
    else return 0xff;
}       

char	hex_to_asc(char hex){
char	da;
	da = hex & 0x0f;
    if((da >= 0) && (da <= 9))    return  ('0' + da);
    else    return  ('A' + da - 0x0a);
}

WORD	hex2_to_decimal(BYTE b){
WORD	d100,d10,d1;
WORD	dec=0;
	d100 = b/100;
	d10 = (b - d100*100)/10;
	d1 = (b - d100*100 - d10*10); 
	dec = (d100<<8) | (d10<<4) | d1;
	return dec;
}

LONG	hex4_to_decimal(WORD w){
LONG	d10000,d1000,d100,d10,d1;
LONG	dec=0;
	d10000 = w/10000;
	d1000 = (w - d10000 * 10000)/1000;
	d100 = (w - d10000 * 10000 - d1000 * 1000)/100;
	d10 = (w - d10000 * 10000 - d1000 * 1000 - d100 * 100)/10;
	d1 = w - d10000 * 10000 - d1000 * 1000 - d100 * 100 - d10 * 10;
	dec = (d10000<<16) | (d1000<<12) | (d100<<8) | (d10<<4) | d1;
	return dec;
}
//-------------- LCD ---------------
void LCD_out(char data ){
char d;
	d = data << 4;
	LCD_PORT(d);			// RS=0, E=0
	LCD_PORT(d | 0x04);		// RS=0, E=1
	delay_us(1);
	LCD_PORT(d);			// RS=0, E=0
	delay_us(1);
}

void LCD_out2(char rs, char data){
char r;
char d;
	if(rs)	r = 0x01;
	else	r = 0x00;
	delay_us(1);
	d = (data & 0xf0) | r;	// upper 4 bit
	LCD_PORT(d);			// E = 0
	delay_us(1);
	LCD_PORT(d | 0x04);		// E = 1
	delay_us(1);
	LCD_PORT(d);			// E = 0
	delay_us(1);

	d = (data<<4) | r;		// lower 4 bit
	LCD_PORT(d);			// E = 0
	LCD_PORT(d | 0x04);		// E = 1
	delay_us(1);
	LCD_PORT(d);			// E = 0
	delay_us(1);
}

void	lcd_gotoxy(int x, int y){
int	pos;
	switch(y){
		case 0:	// 1st line
			pos = 0x80 | (x & 0x7f);
			break;
		case 1:	// 2nd line
			pos = 0x80 | ((0x40 + x) & 0x7f);
			break;
		case 2:	// 3rd line
			pos = 0x80 | ((0x14 + x) & 0x7f);
			break;
		case 3:	// 4th line
			pos = 0x80 | ((0x54 + x) & 0x7f);
			break;
		default:	break;
	}
	LCD_out2(CON,pos);
	delay_us(40);
}          

void	lcd_clear(void){
	LCD_out2(CON, 0x01);			/* clear display */
	delay_ms(2);
}             

void	lcd_clear_line(char line){	/* 0=1st, 1=2nd, 2=3rd, 3=4th line */
int	i,len=16;
	lcd_gotoxy(0,line);
	if(line >= 2) len=20;
	for(i=0;i<len;i++){
		LCD_out2(DAT, ' ');
		delay_us(40);
	}
}             
                                                                       
void	lcd_cursor_on(void){                                           
								/*        0 0 0 0 1    D            C             B        */
	LCD_out2(CON, 0x0d);		/* display on_off : Disp=ON(1), Cursor=OFF(0), Blink=ON(1) */
	delay_us(40);
}

void	lcd_cursor_off(void){
								/*        0 0 0 0 1    D            C             B        */
	LCD_out2(CON, 0x0c);		/* display on_off : Disp=ON(1), Cursor=OFF(0), Blink=OFF(0) */
	delay_us(40);
}   

void	lcd_putc(char c){
	LCD_out2(DAT, c);
	delay_us(40);
}

void	lcd_puts(char *str){
char	c;
	while(1){
		c = *str++;
		if(!c) break;
		LCD_out2(DAT, c);
		delay_us(40);
	}
} 

void	lcd_hex2(char c){
	lcd_putc(hex_to_asc(c>>4));
	lcd_putc(hex_to_asc(c));
}	

void	lcd_hex4(WORD w){
	lcd_putc(hex_to_asc(w>>12));
	lcd_putc(hex_to_asc(w>>8));
	lcd_putc(hex_to_asc(w>>4));
	lcd_putc(hex_to_asc(w));
}
void	lcd_hex8(LONG l){
	lcd_putc(hex_to_asc(l>>28));
	lcd_putc(hex_to_asc(l>>24));
	lcd_putc(hex_to_asc(l>>20));
	lcd_putc(hex_to_asc(l>>16));
	lcd_putc(hex_to_asc(l>>12));
	lcd_putc(hex_to_asc(l>>8));
	lcd_putc(hex_to_asc(l>>4));
	lcd_putc(hex_to_asc(l));
}

void	lcd_decimal_byte(BYTE b){
WORD	w;
	w = hex2_to_decimal(b);
	lcd_putc(hex_to_asc(w>>8));
	lcd_putc(hex_to_asc(w>>4));
	lcd_putc(hex_to_asc(w));
}

void	lcd_decimal_word(WORD w){
LONG	l;
	l = hex4_to_decimal(w);
	lcd_putc(hex_to_asc(l>>16));
	lcd_putc(hex_to_asc(l>>12));
	lcd_putc(hex_to_asc(l>>8));
	lcd_putc(hex_to_asc(l>>4));
	lcd_putc(hex_to_asc(l));
}

void	lcd_decimal_int(WORD w){
LONG	l;
int		f=0;
char	b;
	l = hex4_to_decimal(w);
	b = hex_to_asc(l>>16);
	if((b=='0') && (f==0)) lcd_putc(' ');
	else{
		lcd_putc(b);	f = 1;
	}
	b = hex_to_asc(l>>12);
	if((b=='0') && (f==0)) lcd_putc(' ');
	else{
		lcd_putc(b);	f = 1;
	}
	b = hex_to_asc(l>>8);
	if((b=='0') && (f==0)) lcd_putc(' ');
	else{
		lcd_putc(b);	f = 1;
	}
	b = hex_to_asc(l>>4);
	if((b=='0')&&(f==0)) lcd_putc(' ');
	else{
		lcd_putc(b);	f = 1;
	}
	lcd_putc(hex_to_asc(l));
}

void	init_lcd(void){

	OUTDIR_LCD2811;
	
/* GPBMUX: GPIO_B function 0=IOP,1=FUN	I(0)/O(1)
	bit15	0:	C6TRIP,PB15		;IOP		0		K7
	bit14	0:	C5TRIP,PB14		;IOP		0		K6
	bit13	0:	C4TRIP,PB13		;IOP		0		K5
	bit12	0:	TCLKINB,PB12	;IOP		0		K4
	bit11	0:	TDIRB,PB11		;IOP		0		K3
	bit10	0:	CAP6_QEPI2,PB10	;IOP		0		K2
	bit9	0:	CAP5_QEP4,PB9	;IOP		0		K1
	bit8	0:	CAP4_QEP3,PB8	;IOP		0		K0
	bit7	0:	T4PWM_T4CMP,PB7	;IOP		1		LCD7
	bit6	0:	T3PWM_T3CMP,PB6	;IOP		1		LCD6
	bit5	0:	PWM12,PB5		;IOP		1		LCD5
	bit4	0:	PWM11,PB4		;IOP		1		LCD4
	bit3	0:	PWM10,PB3		;IOP		1	
	bit2	0:	PWM9,PB2		;IOP		1		LCD2
	bit1	0:	PWM8,PB1		;IOP		1	
	bit0	0:	PWM7,PB0		;IOP		1		LCD0
*/
	
	LCD_PORT(0);   				/* 0x00 out */

	delay_ms(15);				/* wait 15ms */
	LCD_out(0x03);				/* function set 1 */
	delay_ms(4);
	LCD_out(0x03);				/* function set 2 */
	delay_ms(4);
	LCD_out(0x03);				/* function set 3 */
	delay_ms(4);
	LCD_out(0x02);				/* function set */
	delay_ms(4);

	LCD_out2(CON, 0x28);		/* function set */
	delay_us(40);
	LCD_out2(CON, 0x0c);		/* display on : Disp=ON, Cursor=OFF, Cursor_Blink=OFF */
	delay_us(40);
	LCD_out2(CON, 0x06);		/* entry mode */
	delay_us(40);
	LCD_out2(CON, 0x01);		/* clear display */
	delay_ms(2);
}
