/************************************************/
/*	debug_serial.c     By RealSYS  				*/
/************************************************/
#include "DSP281x_Device.h"
interrupt void scia_tx_isr(void);
interrupt void scia_rx_isr(void);

#define	CPUCLK		150000000L
#define	LSPCLK		(CPUCLK/4)
#define	BAUDRATE	38400L
#define	BRR_VAL		(LSPCLK/(8*BAUDRATE)-1)

/*
BPS		CPUCLK		LSPCLK		BRR_VAL		BRR_VAL	calc. BPS	error
4800	150000000	37500000	975.5625	976 	4797.851 	-0.045 
9600	150000000	37500000	487.28125	487 	9605.533 	0.058 
19200	150000000	37500000	243.140625	243 	19211.066 	0.058 
38400	150000000	37500000	121.0703125	121 	38422.131 	0.058 
57600	150000000	37500000	80.38020833	80 		57870.370 	0.469 
115200	150000000	37500000	39.69010417	39 		117187.500 	1.725 
*/

typedef	unsigned char	BYTE;
typedef	unsigned short	WORD;
typedef	unsigned long	LONG;

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

#define	DEBUG_BUF_SIZE	50

/* Variables for Serial Communication Debug */
LONG	debug_rd_addr=0,debug_rd_addr_tmp=0;
LONG	debug_wr_addr;
htype	debug_wr_data;
WORD    debug_sum=0,debug_sum_temp=0;               
BYTE	debug_state=0,debug_cnt=0,debug_wr_size=0;
BYTE	debug_tx_buf[DEBUG_BUF_SIZE+1];
BYTE	debug_tx_pos=0,debug_tx_end=0;
BYTE	debug_wr_sum_flag=1;

#define	DEBUG_TX_START	{  if(SciaRegs.SCICTL2.bit.TXRDY){							\
							SciaRegs.SCICTL2.bit.TXINTENA=1;						\
							SciaRegs.SCITXBUF = debug_tx_buf[debug_tx_pos++];		\
							if(debug_tx_pos >= DEBUG_BUF_SIZE) debug_tx_pos=0;}		\
						}

#define	DEBUG_TX_STOP	SciaRegs.SCICTL2.bit.TXINTENA=0

#define CR  			0x0d
#define	ASC_HEX_ERROR	0xff

// Transmit a character from the SCI'
//void send_char(char c){
//    while(!SciaRegs.SCICTL2.bit.TXRDY);
//    SciaRegs.SCITXBUF=c;
//}    


BYTE	asc2hex(BYTE asc){
    if((asc >= '0') && (asc <= '9'))    return (asc - '0');
    else if((asc >= 'A') && (asc <= 'F'))   return (asc - 'A' + 0x0a);
    else if((asc >= 'a') && (asc <= 'f'))    return (asc - 'a' + 0x0a);
    else return ASC_HEX_ERROR;
}
BYTE	hex2asc(BYTE hex){
int	da;
	da = hex & 0x0f;
    if((da >= 0) && (da <= 9))    return  ('0' + da);
    else    return  ('A' + da - 0x0a);
}

void	write_txbuf_string(char *p){
char    rd;
	while(rd = *p++){             
		debug_tx_buf[debug_tx_end++] = rd;
		if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	}
}

void	write_txbuf_asc(BYTE d){
	debug_tx_buf[debug_tx_end++] = d;
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
}

void	write_txbuf_byte(BYTE d){
	debug_tx_buf[debug_tx_end++] = hex2asc(d >> 4);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(d);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
}

void	write_txbuf_word(WORD d){
	debug_tx_buf[debug_tx_end++] = hex2asc(d >> 12);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(d >> 8);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(d >> 4);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(d);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
}

void	write_txbuf_long(LONG d){
ltype	l;
	l.l = d;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[1]>>12);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[1]>>8);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[1]>>4);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[1]);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[0]>>12);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[0]>>8);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[0]>>4);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
	debug_tx_buf[debug_tx_end++] = hex2asc(l.w[0]);
	if(debug_tx_end >= DEBUG_BUF_SIZE)	debug_tx_end = 0;
}

void state0_proc(BYTE rxd){                              
ltype	d;
int		i;
	switch(rxd){
		case '!':
			write_txbuf_asc('!');		/* for BPS & Exist check */
			DEBUG_TX_START;
			break;                             
		case '@': 
			debug_cnt=0;
			debug_rd_addr_tmp=0;
			debug_state=1;
			break;
		case 'R': 
		case 'r':
			d.w[0] = *(WORD *)(debug_rd_addr);
			write_txbuf_word(d.w[0]);
			write_txbuf_asc(CR);
			if(rxd =='r') debug_rd_addr++;
			DEBUG_TX_START;
			break;
		case 'L': 
		case 'l': 
			d.l = *(LONG *)(debug_rd_addr);
			write_txbuf_long(d.l);
			write_txbuf_asc(CR);
			if(rxd == 'l') debug_rd_addr += 2;
			DEBUG_TX_START;
			break;
		case 'O':
		case 'o':
			for(i=0; i<8; i+=2){
				d.l = *(LONG *)(debug_rd_addr + i);
				write_txbuf_long(d.l);
			}
			write_txbuf_asc(CR);	 
			if(rxd == 'o') debug_rd_addr += 8;
			DEBUG_TX_START;
			break;
		case '?':
			write_txbuf_long(debug_rd_addr);
			write_txbuf_string("_DSP28x");
			write_txbuf_asc(CR);
			DEBUG_TX_START;
			break;
		case 'W':
			debug_sum_temp=0;         
			debug_sum='W';
			debug_wr_sum_flag=1;
			debug_state=3;               	
		    debug_wr_data.l[0]=0;
		    debug_wr_data.l[1]=0;
		    debug_wr_data.l[2]=0;
		    debug_wr_data.l[3]=0;
		    debug_wr_addr=0;
		    debug_cnt=0;
			break;                 
		case 'w':
			debug_sum_temp=0;         
			debug_sum='w';
			debug_wr_sum_flag=0;
			debug_state=3;               	
		    debug_wr_data.l[0]=0;
		    debug_wr_data.l[1]=0;
		    debug_wr_data.l[2]=0;
		    debug_wr_data.l[3]=0;
		    debug_wr_addr=0;
		    debug_cnt=0;
			break;                 
			
		default: 
			break;
	}
}     

void state1_proc(BYTE rxd){                           
BYTE temp ;
	temp = asc2hex(rxd);
	if( temp == ASC_HEX_ERROR ){
		 debug_state=0;	
		 return;
	}
	debug_rd_addr_tmp =(debug_rd_addr_tmp<<4) + (temp & 0xf);
	debug_cnt++;    
	if(debug_cnt >= 6) debug_state=2;
}

void state2_proc(BYTE rxd){
	debug_state=0;
	switch(rxd){
		case CR:
			debug_rd_addr=debug_rd_addr_tmp;
			write_txbuf_asc('!');
			DEBUG_TX_START;
			break;
		case 'R':
		case 'r':
		case 'L':
		case 'l':
		case 'O':
		case 'o':
			debug_rd_addr=debug_rd_addr_tmp;
			state0_proc(rxd);
			break;
		default:
			write_txbuf_string("X");
			DEBUG_TX_START;
			break;
	}

}		
			
void state3_proc(BYTE rxd){                     
	if((rxd == '1') || (rxd == '2')){
		 debug_wr_size = asc2hex(rxd)*4;
		 debug_state=4;
	}
	else debug_state=0;
	
	debug_cnt = 0;                             
	debug_sum += rxd;
}                                      

void state4_proc(BYTE rxd){
BYTE temp ;
	temp = asc2hex(rxd);
	if( temp == ASC_HEX_ERROR){
		 debug_state=0;	
		 return;
	}
	
	debug_wr_addr = (debug_wr_addr<<4) + (temp & 0xf);
	debug_cnt++;    
	if(debug_cnt >= 6){
		debug_state = 5;
		debug_cnt = 0;
	}
	debug_sum += rxd;
}
	
void state5_proc(BYTE rxd){
BYTE hex,i;
	hex = asc2hex(rxd);
	if(hex == ASC_HEX_ERROR){
		 debug_state=0;	
		 return;
	}                                                 
	
	i = debug_cnt >> 3;
	debug_cnt++;
	debug_wr_data.l[i] = (debug_wr_data.l[i] << 4) + hex;
	if(debug_cnt >= debug_wr_size){
		debug_cnt = 0;
		debug_state=6;
	}
	debug_sum += rxd;
}	
	            
void state6_proc(BYTE rxd){
BYTE temp ;
	temp = asc2hex(rxd);
	if( temp == ASC_HEX_ERROR ){
		 debug_state=0;	
		 return;
	}
	
	debug_sum_temp = (debug_sum_temp << 4) + temp;
	debug_cnt++;
	if(debug_cnt >= 2)	debug_state=7;
}                         

void state7_proc(BYTE rxd){
	if(rxd != CR)	write_txbuf_string("X");
	else if((debug_sum & 0xff) != (debug_sum_temp & 0xff)){
		if(debug_wr_sum_flag){
			write_txbuf_string("X");
			DEBUG_TX_START;
		    debug_state=0;
		    return;
		}
		
	}

   	if(debug_wr_size == 4){
   		*(WORD *)(debug_wr_addr) = debug_wr_data.l[0];
   	}
   	else if(debug_wr_size == 8){
   		*(LONG *)(debug_wr_addr) = debug_wr_data.l[0];
   	}
   	write_txbuf_asc('!');
	DEBUG_TX_START;

    debug_state=0;
}

interrupt void scia_tx_isr(void){
	if(debug_tx_pos != debug_tx_end){
		SciaRegs.SCITXBUF = debug_tx_buf[debug_tx_pos++];
		if(debug_tx_pos >= DEBUG_BUF_SIZE)	debug_tx_pos = 0;
	}
	else{                              
		DEBUG_TX_STOP;
	}

	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	
}
	
interrupt void scia_rx_isr(void){
BYTE	rxd;            
	rxd = SciaRegs.SCIRXBUF.all;
	switch(debug_state){
		case 0:		state0_proc(rxd);	break;
		case 1:		state1_proc(rxd);	break;
		case 2:		state2_proc(rxd);	break;
		case 3:		state3_proc(rxd);	break;
		case 4:		state4_proc(rxd);	break;
		case 5:		state5_proc(rxd);	break;
		case 6:		state6_proc(rxd);	break;
		case 7:		state7_proc(rxd);	break;
		default:	debug_state = 0;	break;
	}
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	
}

void sci_debug_init(void){
// SCI(UART) init.

    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
	
 	SciaRegs.SCIFFTX.all = 0xa000;		// FIFO reset
 	SciaRegs.SCIFFCT.all = 0x4000;		// Clear ABD(Auto baud bit)
 	
 	SciaRegs.SCICCR.all =0x0007;   		// 1 stop bit,  No loopback 
                                   		// No parity,8 char bits,
                                   		// async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  		// enable TX, RX, internal SCICLK, 
                                   		// Disable RX ERR, SLEEP, TXWAKE

	SciaRegs.SCICTL2.bit.RXBKINTENA =1;	// RX/BK INT ENA=1,
	SciaRegs.SCICTL2.bit.TXINTENA =1;	// TX INT ENA=1,

    SciaRegs.SCIHBAUD = BRR_VAL >> 8;	// High Value
    SciaRegs.SCILBAUD = BRR_VAL & 0xff;	// Low Value

	SciaRegs.SCICTL1.all =0x0023;		// Relinquish SCI from Reset 


	// User specific functions, Reassign vectors (optional), Enable Interrupts:
	
    // Initialize SCI_A RX interrupt
    // Reassign SCI_A RX ISR. 
    // Reassign the PIE vector for RXAINT to point to a different ISR then
    // the shell routine found in DSP28_DefaultIsr.c.
    // This is done if the user does not want to use the shell ISR routine
    // but instead wants to use their own ISR.  This step is optional:
    
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.RXAINT = &scia_rx_isr;
	PieVectTable.TXAINT = &scia_tx_isr;

    GpioMuxRegs.GPFMUX.bit.SCITXDA_GPIOF4=1;
    GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5=1;
	
	EDIS;       // This is needed to disable write to EALLOW protected registers


    // Enable CPU INT9 for SCI-A
	IER |= M_INT9;
	
    // Enable SCI-A RX INT in the PIE: Group 9 interrupt 1
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;

    // Enable SCI-A TX INT in the PIE: Group 9 interrupt 2
	PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
}
