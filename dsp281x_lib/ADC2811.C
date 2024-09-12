/* adc2811.c */
#include "..\def28x.h"

#define AD_START		AdcRegs.ADCTRL2.bit.SOC_SEQ1=1
#define IS_AD_BUSY		AdcRegs.ADCST.bit.SEQ1_BSY
#define AD_INT_CLR		AdcRegs.ADCST.bit.INT_SEQ1_CLR=1

Uint16	ad_data[16];

interrupt void adc_isr(void){
//	ON_LED;
	ad_data[0] = AdcRegs.ADCRESULT0>>4;
	ad_data[1] = AdcRegs.ADCRESULT1>>4;
	ad_data[2] = AdcRegs.ADCRESULT2>>4;
	ad_data[3] = AdcRegs.ADCRESULT3>>4;
	ad_data[4] = AdcRegs.ADCRESULT4>>4;
	ad_data[5] = AdcRegs.ADCRESULT5>>4;
	ad_data[6] = AdcRegs.ADCRESULT6>>4;
	ad_data[7] = AdcRegs.ADCRESULT7>>4;
	ad_data[8] = AdcRegs.ADCRESULT8>>4;
	ad_data[9] = AdcRegs.ADCRESULT9>>4;
	ad_data[10] = AdcRegs.ADCRESULT10>>4;
	ad_data[11] = AdcRegs.ADCRESULT11>>4;
	ad_data[12] = AdcRegs.ADCRESULT12>>4;
	ad_data[13] = AdcRegs.ADCRESULT13>>4;
	ad_data[14] = AdcRegs.ADCRESULT14>>4;
	ad_data[15] = AdcRegs.ADCRESULT15>>4;
	
	AD_INT_CLR;
	AD_START;

	// Acknowledge this interrupt to recieve more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;	

//	OFF_LED;
}

/****** init_adc ******/
// ch_num= 1~16
void init_adc(int ch_num){
	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x3;	// Power up bandgap/reference circuitry
	delay_ms(5);                    		// Delay before powering up rest of ADC
	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;		// Power up rest of ADC
	delay_us(20);                   		// Delay after powering up ADC

	AdcRegs.ADCTRL1.bit.ACQ_PS = 2;
	AdcRegs.ADCTRL1.bit.CPS = 1;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 4;  		// ADC core clock divider, HSPCLK(75MHz)/(2*4) = 9.3728MHz

	if((ch_num>=1) && (ch_num<=16)) AdcRegs.ADCMAXCONV.all = ch_num-1;	// 0 ~ 15
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;  		// Cascaded mode,SEQ1 and SEQ2 = single 16-state
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;  		// start-stop mode
	
	AdcRegs.ADCCHSELSEQ1.all = 0x3210;
	AdcRegs.ADCCHSELSEQ2.all = 0x7654;
	AdcRegs.ADCCHSELSEQ3.all = 0xBA98;
	AdcRegs.ADCCHSELSEQ4.all = 0xFEDC;

	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;

	EALLOW;		// This is needed to write to EALLOW protected registers
	PieVectTable.ADCINT = &adc_isr;			// for ADC
	EDIS;       // This is needed to disable write to EALLOW protected registers
	
// Interrupt Enable Register
	IER |= M_INT1;	// for ADCINT

//	PIE : Peripheral Interrupts setting
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;		// for ADCINT

	AD_START;
}
