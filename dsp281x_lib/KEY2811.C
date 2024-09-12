/* key2811.c */
#include "..\def28x.h"

#define	IN_KEY2811	(GpioDataRegs.GPBDAT.all >> 8)
enum 	key_code	{K1CODE,K2CODE,K3CODE,K4CODE,K5CODE,K6CODE,K7CODE,K8CODE};
#define	KEY_MASK	0xff
#define	NOKEY_CODE	0x0f  
#define	CODE_MASK	0x0f
#define	KEY_PRESSED	0x80
#define	KEY_CONT	0x40

BYTE	key_code=NOKEY_CODE;
BYTE	prev_key_code=NOKEY_CODE;	
int		key_count=0;

void	key_check(void){     
BYTE	tmp_key_code;
BYTE	key; 
	key = IN_KEY2811 & KEY_MASK;
	if(key == KEY_MASK){
		prev_key_code = key_code = NOKEY_CODE;
		return;
	}
	if((key & BIT0) == 0)		tmp_key_code = K1CODE;
	else if((key & BIT1) == 0)	tmp_key_code = K2CODE;
	else if((key & BIT2) == 0)	tmp_key_code = K3CODE;
	else if((key & BIT3) == 0)	tmp_key_code = K4CODE;
	else if((key & BIT4) == 0)	tmp_key_code = K5CODE;
	else if((key & BIT5) == 0)	tmp_key_code = K6CODE;
	else if((key & BIT6) == 0)	tmp_key_code = K7CODE;
	else if((key & BIT7) == 0)	tmp_key_code = K8CODE;
		
	if(tmp_key_code != (prev_key_code & CODE_MASK)){
		prev_key_code = tmp_key_code;
		key_code = KEY_PRESSED | tmp_key_code;
		key_count = 0;
	}	            
	else{
		if(key_count < 100) key_count++;
		else key_code |= KEY_CONT;
	}            
}

void	init_key(void){
// KEY: PB8~PB15
	EALLOW;
	GpioMuxRegs.GPBMUX.all &= 0x00ff;		// PB8~PB15=IOP
    GpioMuxRegs.GPBDIR.all &= 0x00ff;		// PB8~PB15=input(0) dir
	EDIS;       
}
