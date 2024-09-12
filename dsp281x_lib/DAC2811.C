/* dac2811.c */
#include "..\def28x.h"
#define	HI_LOAD		GpioDataRegs.GPESET.bit.GPIOE2=1
#define	LO_LOAD		GpioDataRegs.GPECLEAR.bit.GPIOE2=1

void init_dac(void){
	EALLOW;
    GpioMuxRegs.GPFMUX.bit.SPICLKA_GPIOF2 = 1;
    GpioMuxRegs.GPFMUX.bit.SPISIMOA_GPIOF0 = 1;
    GpioMuxRegs.GPFMUX.bit.SPISTEA_GPIOF3 = 1;
    GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2 = 0; 	// IOP

    GpioMuxRegs.GPEDIR.bit.GPIOE2=1;	// DIR: 1=output,0=input

// SPI init for DAC (DAC7612)
	SpiaRegs.SPICCR.bit.SPISWRESET=0;	// SPI SW RESET = 0
	SpiaRegs.SPICTL.all = 0x06;			// Master mode,without delay
	SpiaRegs.SPIBRR = 2;				// 0~2=LSPCLK(37.5MHz)/4=9.375Mbps, 3~127=LSPCLK/(SPIBRR+1)
										// 7: 37.5MHz/8= 4.6875MHz
	SpiaRegs.SPICCR.all = 0x4d;			// CLOCK_POLARITY(1)=falling, 14bit length
	SpiaRegs.SPICCR.bit.SPISWRESET=1;	// SPI SW RESET = 1
	HI_LOAD;							// /LOAD = 1;

	EDIS; 
}

void	out_dac(WORD ch,WORD dat){
int	tmp;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);	// wait if TX_BUF_FULL
	
	if(ch&1) SpiaRegs.SPITXBUF = (0x3000 | (dat & 0xfff))<<2;	// DAC_B
	else	 SpiaRegs.SPITXBUF = (0x2000 | (dat & 0xfff))<<2;	// DAC_A

	while(!SpiaRegs.SPISTS.bit.INT_FLAG);
	
	LO_LOAD;
	tmp = SpiaRegs.SPIRXBUF;
	HI_LOAD;
}
