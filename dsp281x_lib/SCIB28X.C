// filename: scib28x.c
// By RealSYS
#include "DSP281x_Device.h"

typedef	unsigned char	BYTE;
typedef	unsigned short	WORD;
typedef	unsigned long	LONG;

interrupt void scib_tx_isr(void);
interrupt void scib_rx_isr(void);

#define	CPUCLK		150000000L
#define	LSPCLK		(CPUCLK/4)

/*
BPS		CPUCLK		LSPCLK		BRR_VAL		BRR_VAL	calc. BPS	error
4800	150000000	37500000	975.5625	976 	4797.851 	-0.045 
9600	150000000	37500000	487.28125	487 	9605.533 	0.058 
19200	150000000	37500000	243.140625	243 	19211.066 	0.058 
38400	150000000	37500000	121.0703125	121 	38422.131 	0.058 
57600	150000000	37500000	80.38020833	80 		57870.370 	0.469 
115200	150000000	37500000	39.69010417	40 		114329.268 	-0.756 
*/
#define	SCI_TX_START	{	if(ScibRegs.SCICTL2.bit.TXRDY){						\
								ScibRegs.SCICTL2.bit.TXINTENA=1;				\
								ScibRegs.SCITXBUF = sci_tx_buf[sci_tx_pos++];	\
								if(sci_tx_pos >= SCI_TX_BUF_SIZE) sci_tx_pos=0;}\
						}
#define	SCI_TX_STOP		ScibRegs.SCICTL2.bit.TXINTENA=0

#define	SCI_TX_BUF_SIZE	100
#define	SCI_RX_BUF_SIZE	100


typedef union 
	{
		WORD	w[2];
		LONG	l;
	}ltype;
	
typedef union 
	{
		WORD	w[16];
		LONG	l[8];
	}htype;

#define CR  			0x0d
#define	ASC_HEX_ERROR	0xff

/* Variables for Serial Communication  */
BYTE	sci_tx_buf[SCI_TX_BUF_SIZE+1];
BYTE	sci_tx_pos=0,sci_tx_end=0;

BYTE	sci_rx_buf[SCI_RX_BUF_SIZE];
BYTE	sci_rx_pos=0,sci_rx_end=0;

void	sci_tx_start(void){
	SCI_TX_START;
}

/************ Polling send ***************/
void send_char(char c){
    while(!ScibRegs.SCICTL2.bit.TXRDY);
    ScibRegs.SCITXBUF=c;
}    

void    send_str(char *p){
char	rd;
    while(rd = *p++) send_char(rd);
}

void	send_hex4(WORD d){
	send_char(hex_to_asc(d >> 12));
	send_char(hex_to_asc(d >> 8));
	send_char(hex_to_asc(d >> 4));
	send_char(hex_to_asc(d));
}

/************ Interrupt send ***************/

void	sci_putc(char d){
	sci_tx_buf[sci_tx_end++] = d;
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	SCI_TX_START;
}

void	sci_puts(char *p){
char    rd;
	while(rd = *p++){             
		sci_tx_buf[sci_tx_end++] = rd;
		if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	}
	SCI_TX_START;
}

void	sci_hex2(BYTE d){
	sci_tx_buf[sci_tx_end++] = hex_to_asc(d >> 4);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(d);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	SCI_TX_START;
}

void	sci_hex4(WORD d){
	sci_tx_buf[sci_tx_end++] = hex_to_asc(d >> 12);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(d >> 8);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(d >> 4);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(d);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	SCI_TX_START;
}

void	sci_hex8(LONG d){
ltype	l;
	l.l = d;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[1]>>12);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[1]>>8);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[1]>>4);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[1]);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[0]>>12);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[0]>>8);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[0]>>4);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	sci_tx_buf[sci_tx_end++] = hex_to_asc(l.w[0]);
	if(sci_tx_end >= SCI_TX_BUF_SIZE)	sci_tx_end = 0;
	SCI_TX_START;
}

void	sci_decimal_byte(BYTE b){
WORD	w;
	w = hex2_to_decimal(b);
	sci_putc(hex_to_asc(w>>8));
	sci_putc(hex_to_asc(w>>4));
	sci_putc(hex_to_asc(w));
	SCI_TX_START;
}

void	sci_decimal_word(WORD w){
LONG	l;
	l = hex4_to_decimal(w);
	sci_putc(hex_to_asc(l>>16));
	sci_putc(hex_to_asc(l>>12));
	sci_putc(hex_to_asc(l>>8));
	sci_putc(hex_to_asc(l>>4));
	sci_putc(hex_to_asc(l));
	SCI_TX_START;
}

void	sci_decimal_int(WORD w){
LONG	l;
int		f=0;
char	b;
	l = hex4_to_decimal(w);
	b = hex_to_asc(l>>16);
	if((b=='0') && (f==0)) sci_putc(' ');
	else{
		sci_putc(b);	f = 1;
	}
	b = hex_to_asc(l>>12);
	if((b=='0') && (f==0)) sci_putc(' ');
	else{
		sci_putc(b);	f = 1;
	}
	b = hex_to_asc(l>>8);
	if((b=='0') && (f==0)) sci_putc(' ');
	else{
		sci_putc(b);	f = 1;
	}
	b = hex_to_asc(l>>4);
	if((b=='0')&&(f==0)) sci_putc(' ');
	else{
		sci_putc(b);	f = 1;
	}
	sci_putc(hex_to_asc(l));
	SCI_TX_START;
}
int	sci_rx_ready(void){
	if(sci_rx_pos == sci_rx_end) return 0;
	return 1;
}

int	sci_tx_ready(void){
	if(sci_tx_pos != sci_tx_end) return 0;
	return 1;
}

int	sci_getc(void){
BYTE	rxd;
	if(sci_rx_pos == sci_rx_end) return -1;
	rxd = sci_rx_buf[sci_rx_end++];
	if(sci_rx_end >= SCI_RX_BUF_SIZE) sci_rx_end = 0;
	return rxd;
}

interrupt void scib_tx_isr(void){
	if(sci_tx_pos != sci_tx_end){
		ScibRegs.SCITXBUF = sci_tx_buf[sci_tx_pos++];
		if(sci_tx_pos >= SCI_TX_BUF_SIZE)	sci_tx_pos = 0;
	}
	else{                              
		SCI_TX_STOP;
	}
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	
}

interrupt void scib_rx_isr(void){
	sci_rx_buf[sci_rx_pos++] = ScibRegs.SCIRXBUF.all;
	if(sci_rx_pos >= SCI_RX_BUF_SIZE) sci_rx_pos = 0;

	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	
}
void init_sci(long bps){
WORD	brr_val;

	EALLOW;
    GpioMuxRegs.GPGMUX.all=0x0030;
/* GPGMUX: GPIO_G function 0=IOP,1=FUN
	bit5	1:	SCIRXDB,PG5		;FUN
	bit4	1:	SCITXDB,PG4		;FUN		
*/
    GpioMuxRegs.GPGDIR.all=0x10;	// DIR: 1=output,0=input
	//	5(RXDB),4(TXDB),3,2,1,0
	//     0       1    0 0 0 0	= 0x10
	EDIS; 

 	ScibRegs.SCIFFTX.all = 0xa000;		// FIFO reset
 	ScibRegs.SCIFFCT.all = 0x4000;		// Clear ABD(Auto baud bit)
 	
 	ScibRegs.SCICCR.all =0x0007;   		// 1 stop bit,  No loopback 
                                   		// No parity,8 char bits,
                                   		// async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  		// enable TX, RX, internal SCICLK, 
                                   		// Disable RX ERR, SLEEP, TXWAKE

	ScibRegs.SCICTL2.bit.RXBKINTENA =1;	// RX/BK INT ENA=1,
	ScibRegs.SCICTL2.bit.TXINTENA =1;	// TX INT ENA=1,

	brr_val = LSPCLK / (8 * bps) - 1;
    ScibRegs.SCIHBAUD = brr_val >> 8;	// High Value
    ScibRegs.SCILBAUD = brr_val & 0xff;	// Low Value

	ScibRegs.SCICTL1.all =0x0023;		// Relinquish SCI from Reset 

	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.RXBINT = &scib_rx_isr;
	PieVectTable.TXBINT = &scib_tx_isr;
	EDIS;       // This is needed to disable write to EALLOW protected registers

    // Enable CPU INT9 for SCI-B
	IER |= M_INT9;
	
    // Enable SCI-B RX INT in the PIE: Group 9 interrupt 3
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;

    // Enable SCI-B TX INT in the PIE: Group 9 interrupt 4
	PieCtrlRegs.PIEIER9.bit.INTx4 = 1;
}
