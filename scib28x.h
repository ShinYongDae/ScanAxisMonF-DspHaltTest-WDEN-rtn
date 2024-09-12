// scib28x.h	By RealSYS
#ifndef __SCIB28X_INCLUDED_
#define __SCIB28X_INCLUDED_
#include "def28x.h"

void	sci_tx_start(void);

void 	init_sci(long bps);
char	asc2hex(char asc);
char	hex2asc(char hex);

void	send_char(char ch);
void    send_str(char *p);
void	send_hex4(WORD d);

void	sci_putc(char d);
void	sci_puts(char *p);
void	sci_hex2(BYTE d);
void	sci_hex4(WORD d);
void	sci_hex8(LONG d);

void	sci_decimal_byte(BYTE b);
void	sci_decimal_word(WORD w);
void	sci_decimal_int(WORD w);

void	sci_tx_isr(void);
void 	sci_rx_isr(void);

#endif
