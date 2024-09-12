/*************************************************/
/*		  delay28x.h 	RealSYS                	 	 */
/*************************************************/
#ifndef __DELAY28X_INCLUDED_
#define __DELAY28X_INCLUDED_

#include "..\def28x.h"

#define DELAY_1us	asm(" RPT #146 || NOP ");	/* 6.667ns * 150 = 1usec */

void	delay_us(unsigned int d);
void	delay_ms(unsigned int d);

#endif
