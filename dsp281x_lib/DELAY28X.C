// delay28x.c

#include "..\delay28x.h"

void	delay_us(unsigned int d){
	while(d--) DELAY_1us;
}
void	delay_ms(unsigned int d){
	while(d--) delay_us(992);
}

