/* eeprom.c */
#include "..\def28x.h"
#include "..\delay28x.h"

#define	OUTDIR_EEP2811;		EALLOW;	\
       						GpioMuxRegs.GPBDIR.bit.GPIOB3 =1;	\
							EDIS;       
#define	INDIR_EEP2811;		EALLOW;	\
       						GpioMuxRegs.GPBDIR.bit.GPIOB3 =0;	\
							EDIS;       

#define	SDA_OUT_MODE;	OUTDIR_EEP2811;
#define	SDA_IN_MODE;	INDIR_EEP2811;

#define SDA_HIGH	(GpioDataRegs.GPBDAT.bit.GPIOB3 = 1)
#define SDA_LOW		(GpioDataRegs.GPBDAT.bit.GPIOB3 = 0)
#define SCL_HIGH	(GpioDataRegs.GPBDAT.bit.GPIOB1 = 1)
#define SCL_LOW		(GpioDataRegs.GPBDAT.bit.GPIOB1 = 0)
#define	CHK_SDA		GpioDataRegs.GPBDAT.bit.GPIOB3

/********************* Serial EEPROM (24C01 02 04 08 16) *********************/
/*      EEPROM delay    1.2 ~ 4.7us  delay_e()=5usec
	AT24Cxx : 100kHz(at < 2.7V), 400kHz(at 5V) */

#define	delay_e()		delay_us(5)

BYTE	EEP_ERR=0;
char	eep_buf[16];

void    eep_start(){
	SDA_HIGH;       SCL_HIGH;       delay_e();
	SDA_LOW;        delay_e();
}
void    eep_stop(){
	SDA_LOW;        SCL_HIGH;       delay_e();
	SDA_HIGH;       delay_e();
}
void    eep_data_stb(){
        SCL_LOW;	delay_e();    SCL_HIGH;   delay_e();    SCL_LOW;    delay_e();
}

/*  io: 1(read),0(write)    page: 0 ~ 7 (24c16) */
void    eep_dev_addr(BYTE io,BYTE page){
		SDA_HIGH;   eep_data_stb();     /*  dev id: 1 */
        SDA_LOW;    eep_data_stb();     /*  dev id: 0 */
        SDA_HIGH;   eep_data_stb();     /*  dev id: 1 */
        SDA_LOW;    eep_data_stb();     /*  dev id: 0 */

        if(page & 0x04) SDA_HIGH;       /* A2: 1 */
        else            SDA_LOW;        /* A2: 0 */
        eep_data_stb();

        if(page & 0x02) SDA_HIGH;       /* A1: 1 */
        else            SDA_LOW;        /* A1: 0 */
        eep_data_stb();

        if(page & 0x01) SDA_HIGH;       /* A0: 1 */
        else            SDA_LOW;        /* A0: 0 */
        eep_data_stb();

        if(io == 0)     SDA_LOW;        /*  READ    */
        else            SDA_HIGH;       /*  WRITE   */
        eep_data_stb();
}
void    eep_addr_wr(BYTE addr){    /*  MSB first   */
BYTE    i;
        for(i=0;i<8;i++){
            if(addr & (0x80>>i))  SDA_HIGH;
            else    SDA_LOW;
            eep_data_stb();
        }
}
void    eep_data_wr(BYTE dat){  /*  MSB first   */
BYTE    i;
        for(i=0;i<8;i++){
            if(dat & (0x80>>i))   SDA_HIGH;
            else    SDA_LOW;
            eep_data_stb();
        }
}
BYTE    eep_data_rd(){
BYTE    i,temp=0x00;
        SDA_IN_MODE;           /*  for input data  */
        
        for(i=0;i<8;i++){
            SCL_LOW;    delay_e();    SCL_HIGH;
            if(CHK_SDA)    temp |= (0x80>>i);
            delay_e();
        }
        SCL_LOW;        delay_e();

		SDA_OUT_MODE;
	    return  temp;
}
void    eep_ack(){
        SDA_IN_MODE;           /*  for ack input */
        
        SCL_LOW;        delay_e();
        SCL_HIGH;       delay_e();
        if(CHK_SDA)    EEP_ERR = 1;
        else    EEP_ERR = 0;
        SCL_LOW;        delay_e();

		SDA_OUT_MODE;
}

int	eep_write_byte(BYTE page,BYTE addr,BYTE wd){    /*  EEP byte write operation    */
		SDA_OUT_MODE;
        eep_start();
        eep_dev_addr(0,page);     /*  write   */
        eep_ack();
        if(EEP_ERR == 1)        return -1;
        eep_addr_wr(addr);
        eep_ack();
        if(EEP_ERR == 1)        return -1;
        eep_data_wr(wd);
        eep_ack();
        if(EEP_ERR == 1)        return -1;
        eep_stop();

		delay_ms(15);
        return 0;
}

int	eep_read_byte(BYTE page,BYTE addr){        /* EEP byte read operation */
BYTE	rd;
		SDA_OUT_MODE;
        eep_start();
        eep_dev_addr(0,page);     /*  dummy write */
        eep_ack();
        if(EEP_ERR == 1) return -1;
        eep_addr_wr(addr);
        eep_ack();
        if(EEP_ERR == 1) return -1;
        eep_start();
        eep_dev_addr(1,page);     /*  read    */
        eep_ack();
        if(EEP_ERR == 1) return -1;
        rd = eep_data_rd();
        SDA_HIGH;       eep_data_stb();
        eep_stop();
		return rd;
}

int    eep_write_block(BYTE page, BYTE addr, char *p){      /*  EEP block write operation    */
BYTE    i;
		SDA_OUT_MODE;
        eep_start();
        eep_dev_addr(0,page);     /*  write   */
        eep_ack();
        if(EEP_ERR == 1) return -1;
        eep_addr_wr(addr);
        eep_ack();
        if(EEP_ERR == 1) return -1;
        for(i=0;i<16;i++){
                eep_data_wr(p[i]);
                eep_ack();
                if(EEP_ERR == 1) return -1;
        }
        eep_stop();

		delay_ms(15);
		return 0;
}

int    eep_read_block(BYTE page, BYTE addr, char *p){       /* EEP block read operation */
BYTE    i;
		SDA_OUT_MODE;
        eep_start();
        eep_dev_addr(0,page);     /*  dummy write */
        eep_ack();
        if(EEP_ERR == 1) return -1;

        eep_addr_wr(addr);
        eep_ack();
        if(EEP_ERR == 1) return -1;

        eep_start();
        eep_dev_addr(1,page);
        eep_ack();
        if(EEP_ERR == 1) return -1;

        for(i=0;i<16;i++){
			p[i] = eep_data_rd();
			if(i == 15) SDA_HIGH;
			else	SDA_LOW;
			eep_data_stb();
        }
        eep_stop();
		return 0;
}

void	init_eep(void){
	EALLOW;
	GpioMuxRegs.GPBMUX.bit.PWM8_GPIOB1 = 0;		// PB1(SCL) = IOP
	GpioMuxRegs.GPBMUX.bit.PWM10_GPIOB3 = 0;	// PB3(SDA) = IOP
	GpioMuxRegs.GPBDIR.bit.GPIOB1 = 1;			// PB1(SCL) = out dir
	GpioMuxRegs.GPBDIR.bit.GPIOB3 = 1;			// PB1(SDA) = out dir
	GpioDataRegs.GPBSET.bit.GPIOB1 = 1;			// SCL = high(1) out
	GpioDataRegs.GPBSET.bit.GPIOB3 = 1;			// SDA = high(1) out
	EDIS;       
}
