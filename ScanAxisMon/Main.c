//#include "MyDefine.h"


#include "..\def28x.h"
//#include "..\lcd2811.h"
//#include "..\key2811.h"
#include "..\delay28x.h"
//#include "..\dac2811.h"
#include "..\adc2811.h"
#include "..\eep2811.h"
#include "..\scib28x.h"
#include <math.h>

#define	PI		3.1415927
#define	ABS(x)	((x) >= 0 ? (x) : -(x))

#define	DelayDisp 		30			// [mSec]
#define WatchDogComm	10000		// [雀]


// PC Comm Command
#define IsConnected		0
#define ReadArrayData	1
#define ReadData		2
#define WriteArrayData	3
#define WriteData		4
#define Reset			5
#define Auto			6
#define Close			7
#define Open			8
#define Query			9
#define Enc				10
#define Conf			11
#define Test			12


#define WinkNum			1			// [雀]
#define WinkDelay		5			// (*DelayDisp) [mSec]

#define CMD_NONE		0x7FFF
#define CMD_ERROR		0x7FFE

#define StopConfirmCnt	5			// [雀]

// for switch ~ case ...

#define GO_C		10
#define GO_R		20
#define GO_RA		30
#define GO_W		40
#define GO_WA		50
#define GO_E		60
#define GO_RST		70
#define GO_AT		80
#define GO_CL		90
#define GO_OP		100
#define GO_Q		110
#define GO_T		120
#define GO_TEST		130


#define DAC_1V		204.8
//#define DAC_LIMIT	2.8
//#define DAC_MAX		3.0
#define DEG_MAX		20.0
//#define DAC_1DEG	(DAC_1V*DAC_MAX/DEG_MAX)

/******** for RomData **************************************************************
Head information : Select mode [1~9], Spare, Spare, Spare, Spare
RomData[0][0] :
mode : 1 ~ 9 [ea]
RomData[mode][0] : 饭捞廉 瞒窜悼累 矫埃 ( 30 ~ 10,000 ) [mSec] 	default(30)
RomData[mode][1] : 悼累 颇屈 (  1 - S 目宏 , 2 - T 目宏 ) 		default(1)
RomData[mode][2] : 悼累 裹困 Min ( -20 deg ~ +20 deg ) [degree]	default(-20)
RomData[mode][3] : 悼累 裹困 Max ( -20 deg ~ +20 deg ) [degree]	default(20)
RomData[mode][4] : 浚内歹 眉磐傅 力芭 妻胶蔼 [pulse]			default(100)
RomData[mode][5] : 浚内歹 Direction [-1(开), +1(沥)]			default(1)
RomData[mode][6] : Reverse Open/Close [-1(开), +1(沥)] 			default(1)
RomData[mode][7] : 悼累 裹困 +Offset [%] 						default(0)
RomData[mode][8] : +/- 悼累 傈拘 [mV] 							default(4800)
************************************************************************************/
//#define	CutOffTimeMin		20
#define	CutOffTimeMin		5
#define CutOffTimeMax		1000
#define	CutOffTimeDefault	40
#define	Scurve				1
#define	Tcurve				2
#define AngleMin			-20
#define AngleMax			20
#define EncFilterDefault	2
#define	EncDir				1
#define	ShutterDir			1
#define AngleMinOffset		0
#define AngleMaxOffset		0
#define	MaxMode				1	// 0 ~ 9
#define	MaxFunc				9	// 0 ~ 8
									
#define	TimerPeriod			500 //500
									
//#pragma DATA_SECTION(FlashData, "Flash_Data")

#define	IN_IO2811	(GpioDataRegs.GPBDAT.all)
#define	IN_MASK		0xff

enum 	io_code	{IO_ENABLE=0,IN_PB1,IN_PB2,IN_PB3,IN_PB4,IN_PB5,IN_PB6,IN_PB7};
unsigned char io_code, g_IoIn, g_IoInF;
unsigned char g_bIoEnable;



#define	IN_KEY2811	(GpioDataRegs.GPBDAT.all >> 8)

enum 	key_code	{K1CODE,K2CODE,K3CODE,K4CODE,K5CODE,K6CODE,K7CODE,K8CODE};
#define	KEY_MASK	0xff
#define	NOKEY_CODE	0x0f  
#define	CODE_MASK	0x0f
#define	KEY_CONT	0x80
#define	KEY_UP		0x40
#define	KEY_DOWN_DO	0x20
#define	KEY_DOWN	0x10
BYTE	key_code=NOKEY_CODE;
BYTE	prev_key_code=NOKEY_CODE;	
int		key_count=0;

#define WatchDogKey	300		// *10[mSec]
enum 	key_mode	{KS_AUTO,KS_MNL,KS_CONF,KS_ENC};
enum 	key_func	{KS_FUNC0,KS_FUNC1,KS_FUNC2,KS_FUNC3,KS_FUNC4};
enum 	key_manual	{KS_CLOSE,KS_OPEN};
BYTE	key_mode=KS_AUTO;
BYTE	key_func=KS_FUNC0;
BYTE	key_manual=KS_CLOSE;
unsigned int KeyState=0x0000;
unsigned int KeyDisp=0x0000;
unsigned char g_bKeyDown, g_bKeyDownSt, g_cPushedKey;
unsigned int g_nWatchDogKey;

unsigned char g_bInitDone;
long g_lDiff;
char g_dir;

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);

long g_lCnt, g_lCntABS, g_lCntPrev, g_lCntPrev2;
long g_nEncDir;
unsigned char g_tCnt[3], g_dutyVoutS, g_bRun, g_bRunF;
int g_outV, g_outVprev;
unsigned int g_eCntJudge, g_eCnt, g_eCntPrev[10], g_eCntPrev2, g_eCntPrev3;
//long g_eCntJudge, g_eCnt, g_eCntPrev, g_eCntPrev2, g_eCntPrev3;
int g_nIdx, g_nStopCnt;

enum 	DispSeg	{D_BLANK,D_NOP,D_AUTO,D_HANDLE,D_CONF,D_ENCODER,D_OPEN,D_CLOSE,
					 D_VAL,D_FUNC0,D_FUNC1,D_FUNC2,D_FUNC3,D_FUNC4,D_FUNC5,D_FUNC6,D_FUNC7,D_FUNC8};
unsigned int DispSegment;


unsigned char g_bAuto, g_bClose, g_bOpen, g_bEnc, g_bConf, g_bRepeatTest;
unsigned char g_bAutoF, g_bCloseF, g_bOpenF, g_bEncF, g_bConfF;
unsigned char g_bAutoWinker, g_bCloseWinker, g_bOpenWinker, g_bEncWinker, g_bConfWinker;
unsigned char g_nAutoWinker, g_nCloseWinker, g_nOpenWinker, g_nEncWinker, g_nConfWinker;
unsigned char g_nWinkDelay;

unsigned char g_bDispAuto, g_bDispAutoF;
unsigned char g_bDispIoEnable, g_bDispIoEnableF;

void SetAuto();
void SetClose();
void SetOpen();
void SetEnc();
void SetConf();
void DispAuto();
void DispClose();
void DispOpen();
void DispEnc();
void DispConf();
void DoAuto();
void DoClose();
void DoOpen();

void InitIo();
void ProcessIo();
void DispIO();

void InitKey();
void CheckKey();
void ProcessKey();
void DispKeySts();

void DispAutoVal();
void DispEncVal();
void DispFuncVal(int nFunc);
void ResetPos();
void DispData();
void InitSegments();
void InitVal();
void SciSend(char *p);
void Int2Ascii(long nNum, char *ch);
									
void init_ev(void);
void init_gpio(void);

unsigned char getDigit(int digit);
void setDigit(int digit, unsigned char value, char dp);
void shutdown(char b);
void setScanLimit(int limit); 
void setIntensity(int intensity); 
void out_Max7219(unsigned char opcode, unsigned char data);
void init_Max7219();
void InitDsp();
void InitDac();
void OutDac();
void CheckDac();
void RepeatTest();


// for DAC =============================================
#define	HI_LOAD		GpioDataRegs.GPESET.bit.GPIOE2=1
#define	LO_LOAD		GpioDataRegs.GPECLEAR.bit.GPIOE2=1

// for IO =============================================
#define	LED_ON		GpioDataRegs.GPFCLEAR.bit.GPIOF14=1
#define	LED_OFF		GpioDataRegs.GPFSET.bit.GPIOF14=1
#define	LED_TOGGLE	GpioDataRegs.GPFTOGGLE.bit.GPIOF14=1

void InitDac(void)
{
	EALLOW;

    GpioMuxRegs.GPFMUX.bit.SPICLKA_GPIOF2 = 1;
    GpioMuxRegs.GPFMUX.bit.SPISIMOA_GPIOF0 = 1;
    GpioMuxRegs.GPFMUX.bit.SPISTEA_GPIOF3 = 1;
    GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2 = 0; 	// IOP

    GpioMuxRegs.GPEDIR.bit.GPIOE2=1;	// DIR: 1=output,0=input

// SPI init for DAC (DAC7612)
	SpiaRegs.SPICCR.bit.SPISWRESET=0;	// SPI SW RESET = 0
	SpiaRegs.SPICTL.all = 0x06;			// Master mode,without delay
	SpiaRegs.SPIBRR = 7;				// 0~2=LSPCLK(37.5MHz)/4=9.375Mbps, 3~127=LSPCLK/(SPIBRR+1)
										// 7: 37.5MHz/8= 4.6875MHz
	SpiaRegs.SPICCR.all = 0x4d;			// CLOCK_POLARITY(1)=falling, 14bit length
	SpiaRegs.SPICCR.bit.SPISWRESET=1;	// SPI SW RESET = 1
	HI_LOAD;							// /LOAD = 1;

	EDIS; 
}

void OutDac(WORD ch,WORD dat)
{
	unsigned int tmp;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);	// wait if TX_BUF_FULL
	
	if(ch&1) SpiaRegs.SPITXBUF = (0x3000 | (dat & 0xfff))<<2;	// DAC_B
	else	 SpiaRegs.SPITXBUF = (0x2000 | (dat & 0xfff))<<2;	// DAC_A

	while(!SpiaRegs.SPISTS.bit.INT_FLAG);
	
	LO_LOAD;
	tmp = SpiaRegs.SPIRXBUF;
	HI_LOAD;
}


// for EEPRom =============================================

int g_SetData[MaxMode][MaxFunc], g_BufRead[MaxFunc], g_BufWrite[MaxFunc];
unsigned int g_nStep, g_cDataIdx, g_cWatchDogComm;
int g_nMode, g_nFunc;
unsigned int g_pData[10] = {0};
int g_nData = CMD_NONE;
char g_cSerialInput;

int RomData[MaxMode][MaxFunc];


int EepRead(int nMode, int nFunc);
void EepWrite(int nMode, int nFunc, int nData);
void ReloadEep();

void InitCommPc();
int GetData();
void ExeCmd(int nCmd);
void CommPc();


/******** for Flash ************/
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
/*******************************/

#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs")
#pragma CODE_SECTION(CheckDac, "ramfuncs")
#pragma CODE_SECTION(CheckKey, "ramfuncs")

#pragma CODE_SECTION(EepRead, "ramfuncs")
#pragma CODE_SECTION(EepWrite, "ramfuncs")
#pragma CODE_SECTION(ReloadEep, "ramfuncs")

//#pragma CODE_SECTION(init_Max7219, "ramfuncs")
//#pragma CODE_SECTION(out_Max7219, "ramfuncs")

//#pragma CODE_SECTION(InitDac, "ramfuncs")
//#pragma CODE_SECTION(OutDac, "ramfuncs")

/*****************************************************************************/

/*****************************************************************************/

//Max7219
/* We keep track of the led-status for 8 LEDs in this array */
unsigned char Max7219_Status[8];
char Max7219_Dot[8];

/* This is the array we send the data with 2 commandbytes */
unsigned char Max7219_Data[2]; // Data+OpCode

#define	HI_CS_Max7219		GpioDataRegs.GPASET.bit.GPIOA15=1	// Unload
#define	LO_CS_Max7219		GpioDataRegs.GPACLEAR.bit.GPIOA15=1 // Load

//the opcodes the MAX7221 and MAX7219 understand
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

/*
 * Here are the segments to be switched on for characters and digits on
 * 7-Segment Displays
 *//*
static const byte charTable[128] = 
{
    B01111110,B00110000,B01101101,B01111001,B00110011,B01011011,B01011111,B01110000,
    B01111111,B01111011,B01110111,B00011111,B00001101,B00111101,B01001111,B01000111,
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
    B00000000,B00000000,B00000000,B00000000,B10000000,B00000001,B10000000,B00000000,
    B01111110,B00110000,B01101101,B01111001,B00110011,B01011011,B01011111,B01110000,
    B01111111,B01111011,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
    B00000000,B01110111,B00011111,B00001101,B00111101,B01001111,B01000111,B00000000,
    B00110111,B00000000,B00000000,B00000000,B00001110,B00000000,B00000000,B00000000,
    B01100111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00001000,
    B00000000,B01110111,B00011111,B00001101,B00111101,B01001111,B01000111,B00000000,
    B00110111,B00000000,B00000000,B00000000,B00001110,B00000000,B00000000,B00000000,
    B01100111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000
};*/

#define _OFF		16
#define _MINUS		17
#define _DONT_CARE	-1

static const unsigned char charTable[128] = 
{
	0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,
	0x7F,0x7B,0x77,0x1F,0x0D,0x3D,0x4F,0x47,
	0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,
	0x7F,0x7B,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x77,0x1F,0x0D,0x3D,0x4F,0x47,0x7B,
	0x37,0x30,0x38,0x0F,0x0E,0x55,0x15,0x1D,
	0x67,0x73,0x05,0x5B,0x0F,0x1C,0x3E,0x5C,
	0x37,0x33,0x6D,0x00,0x00,0x00,0x00,0x08,
	0x00,0x77,0x1F,0x0D,0x3D,0x4F,0x47,0x00,
	0x37,0x00,0x00,0x00,0x0E,0x00,0x00,0x00,
	0x67,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

void init_Max7219()
{
	EALLOW;

	GpioMuxRegs.GPFMUX.bit.SPICLKA_GPIOF2 = 1;
	GpioMuxRegs.GPFMUX.bit.SPISIMOA_GPIOF0 = 1;
	//ioMuxRegs.GPFMUX.bit.SPISTEA_GPIOF3 = 1;
	//GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2 = 0; 	// IOP

	SpiaRegs.SPICCR.bit.SPISWRESET=0;	// SPI SW RESET = 0
	SpiaRegs.SPICTL.all = 0x06;			// Master mode,without delay
	SpiaRegs.SPIBRR = 7;//2;//7				// 0~2=LSPCLK(37.5MHz)/4=9.375Mbps, 3~127=LSPCLK/(SPIBRR+1)
										// 7: 37.5MHz/8= 4.6875MHz
	SpiaRegs.SPICCR.all = 0x4f;			// CLOCK_POLARITY(1)=falling, 14bit length
	SpiaRegs.SPICCR.bit.SPISWRESET=1;	// SPI SW RESET = 1


	// SPI init for Max7219
	GpioMuxRegs.GPADIR.bit.GPIOA15=1;	// DIR: 1=output,0=input

	HI_CS_Max7219;							// /LOAD = 1;

	EDIS; 
}

void out_Max7219(unsigned char opcode, unsigned char data)
{
	unsigned int tmp;
	WORD dat_hi, dat_lo, dat_wd;

	if(opcode < 0 || opcode > 15)
		return;

	init_Max7219();

	Max7219_Data[0] = data;
	Max7219_Data[1] = opcode;

	dat_lo = 0x00FF & Max7219_Data[1];
	dat_hi = 0xFF00 & (dat_lo << 8);
	dat_lo = 0x00FF & Max7219_Data[0];
	dat_wd = dat_hi | dat_lo;


	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);	// wait if TX_BUF_FULL

	SpiaRegs.SPITXBUF = dat_wd;	

	while(!SpiaRegs.SPISTS.bit.INT_FLAG);		// wait for Tx to finish
	
	LO_CS_Max7219;
	tmp = SpiaRegs.SPIRXBUF;
	HI_CS_Max7219;
}

void shutdown(char b) 
{
	if(b)
		out_Max7219(OP_SHUTDOWN,0);
	else
		out_Max7219(OP_SHUTDOWN,1);
}
	
void setScanLimit(int limit) 
{
	if(limit>=0 || limit<8)
		out_Max7219(OP_SCANLIMIT,limit);
}

void setIntensity(int intensity) 
{
	if(intensity>=0 || intensity<16)	
		out_Max7219(OP_INTENSITY,intensity);    
}

void clearDisplay() 
{
	int i;
	for(i=0;i<8;i++) 
	{
		Max7219_Status[i]=0;
		Max7219_Dot[i]=0;
		//out_Max7219(i+1, Max7219_Status[i]);
	}
}

unsigned char getDigit(int digit)
{
	unsigned char v;

	if(digit<0 || digit>7) // || value>17)
		return _OFF;

	v = Max7219_Status[digit];

	return v;
}

void setDigit(int digit, unsigned char value, char dp) 
{
	unsigned char v;

	if(digit<0 || digit>7) // || value>17)
		return;

	v = charTable[value];
	Max7219_Status[digit] = value;
	if(dp != _DONT_CARE)
	{
		Max7219_Dot[digit] = dp;
		if(dp)
		{
			v |= 0x80;
		}
	}
	else
	{
		if(Max7219_Dot[digit])
		{
			v |= 0x80;
		}
	}

	out_Max7219(digit+1, v);

//	delay_us(30); // for 2812
}

//-----------------------------------------------------------------------

void InitIo(void)
{
	// ADCA0 ~ ADCA7 : Output , ADCB0 ~ ADCB7 : Input
	EALLOW;
	GpioMuxRegs.GPBMUX.all &= 0xff00;		// PB0~PB7=IOP
    GpioMuxRegs.GPBDIR.all &= 0xff00;		// PB8~PB15=input(0) dir
	EDIS;       
}


void ProcessIo()
{	
	// IO - Input

	if((g_IoIn & (0x01<<IO_ENABLE)) == 0 && (g_IoInF & (0x01<<IO_ENABLE)) != 0)
	{		
		g_IoInF &= ~(0x01<<IO_ENABLE);
		g_bIoEnable = 1;
	}
	else if((g_IoIn & (0x01<<IO_ENABLE)) != 0 && (g_IoInF & (0x01<<IO_ENABLE)) == 0)
	{
		g_IoInF |= (0x01<<IO_ENABLE);
		g_bIoEnable = 0;
	}

	//g_bIoEnable = 1; // for Test

	if((g_IoIn & (0x01<<IN_PB1)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB1);
	}
	else if((g_IoIn & (0x01<<IN_PB1)) != 0 && (g_IoInF & (0x01<<IN_PB1)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB1);
	}

	if((g_IoIn & (0x01<<IN_PB2)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB2);
	}
	else if((g_IoIn & (0x01<<IN_PB2)) != 0 && (g_IoInF & (0x01<<IN_PB2)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB2);
	}

	if((g_IoIn & (0x01<<IN_PB3)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB3);
	}
	else if((g_IoIn & (0x01<<IN_PB3)) != 0 && (g_IoInF & (0x01<<IN_PB3)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB3);
	}

	if((g_IoIn & (0x01<<IN_PB4)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB4);
	}
	else if((g_IoIn & (0x01<<IN_PB4)) != 0 && (g_IoInF & (0x01<<IN_PB4)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB4);
	}

	if((g_IoIn & (0x01<<IN_PB5)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB5);
	}
	else if((g_IoIn & (0x01<<IN_PB5)) != 0 && (g_IoInF & (0x01<<IN_PB5)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB5);
	}

	if((g_IoIn & (0x01<<IN_PB6)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB6);
	}
	else if((g_IoIn & (0x01<<IN_PB6)) != 0 && (g_IoInF & (0x01<<IN_PB6)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB6);
	}

	if((g_IoIn & (0x01<<IN_PB7)) == 0)	
	{		
		g_IoInF &= ~(0x01<<IN_PB7);
	}
	else if((g_IoIn & (0x01<<IN_PB7)) != 0 && (g_IoInF & (0x01<<IN_PB7)) == 0)
	{
		g_IoInF |= (0x01<<IN_PB7);
	}
}

//-----------------------------------------------------------------------
void CheckKey(void) // every time after 10 [mSec]
{     
	BYTE	tmp_key_code;
	BYTE	key; 

	if(++g_nWatchDogKey > WatchDogKey)
	{
		g_nWatchDogKey = 0;

		if( !(KeyState & 0x0002) && !(KeyState & 0x0001) ) 			// Auto
		{
			if(!g_bEnc && g_bKeyDownSt)
			{
				g_bAuto = 1;
				g_bKeyDownSt = 0;

				g_eCntPrev3 = g_eCnt+1; //DispEncVal();
				KeyDisp &= ~0x000C;
			}
		}
		else if( !(KeyState & 0x0002) && (KeyState & 0x0001) ) 		// Handle
		{
			NOP;
		}
		else if( (KeyState & 0x0002) && !(KeyState & 0x0001) ) 		// Config
		{
			NOP;
			/*KeyState &= ~0x0003;
			g_bAuto = 1;

			g_bKeyDownSt = 0;

			g_eCntPrev3 = g_eCnt+1; //DispEncVal();
			KeyDisp &= ~0x000C;*/
		}
		else if( (KeyState & 0x0002) && (KeyState & 0x0001) ) 		// Encoder
		{
			g_eCntPrev3 = g_eCnt+1; //DispEncVal();
			g_bKeyDownSt = 0;

			/*KeyState &= ~0x0003;
			g_bAuto = 1;

			g_bKeyDownSt = 0;

			g_eCntPrev3 = g_eCnt+1; //DispEncVal();
			KeyDisp &= ~0x000C;*/
		}
		else 										// Auto
		{
			KeyState &= ~0x0003;
			if(!g_bAuto && g_bKeyDownSt)
			{
				g_bAuto = 1;
				g_bKeyDownSt = 0;

				g_eCntPrev3 = g_eCnt+1; //DispEncVal();
				KeyDisp &= ~0x000C;
			}
		}
	}


	key = IN_KEY2811 & KEY_MASK;
//	if(key == KEY_MASK)
//	{
//		prev_key_code = key_code = NOKEY_CODE;
//		return;
//	}
	if((key & BIT0) == 0)		
		tmp_key_code = K1CODE;
	else if((key & BIT1) == 0)	
		tmp_key_code = K2CODE;
	else if((key & BIT2) == 0)	
		tmp_key_code = K3CODE;
	else if((key & BIT3) == 0)	
		tmp_key_code = K4CODE;
	else if((key & BIT4) == 0)	
		tmp_key_code = K5CODE;
	else if((key & BIT5) == 0)	
		tmp_key_code = K6CODE;
	else if((key & BIT6) == 0)	
		tmp_key_code = K7CODE;
	else if((key & BIT7) == 0)	
		tmp_key_code = K8CODE;
	else
		tmp_key_code = 0x0F;
		
	if(!(key_code & KEY_DOWN) && !g_bKeyDown && (tmp_key_code != 0x0F))
	{
		g_bKeyDown = 1;
		prev_key_code = tmp_key_code;
		key_code = (KEY_DOWN | tmp_key_code);
		key_code &= ~KEY_DOWN_DO;
		key_code &= ~KEY_UP;
		key_code &= ~KEY_CONT;
		key_count = 0;
	}	            
	else if((key_code & KEY_DOWN) && (tmp_key_code == (prev_key_code & CODE_MASK)))
	{
		if(key_count < 100) 	// after *10 [mSec]
			key_count++;
		else if((prev_key_code & CODE_MASK) == (key_code & CODE_MASK))
			key_code |= KEY_CONT;
	}            
	else if(!(key_code & KEY_UP) && ((key_code & KEY_DOWN) || (key_code & KEY_CONT)) && (tmp_key_code == 0x0F))
	{
		g_bKeyDown = 0;
		key_code |= KEY_UP;

		key_code &= ~KEY_DOWN_DO;
		key_code &= ~KEY_DOWN;
		key_code &= ~KEY_CONT;
	}

}

void InitKey(void)
{
// KEY: PB8~PB15
	EALLOW;
	GpioMuxRegs.GPBMUX.all &= 0x00ff;		// PB8~PB15=IOP
    GpioMuxRegs.GPBDIR.all &= 0x00ff;		// PB8~PB15=input(0) dir
	EDIS;       
}

void ProcessKey()
{	
	unsigned char value;
	unsigned char PushedKey = (key_code & CODE_MASK);
	switch(PushedKey)
	{
		case 0 :
			break;
		case 1 :
			break;
		case 2 :
			break;
		case 3 :
			break;
		case 4 :
			break;
		case 5 :
			break;
		case 6 :
			break;
		case 7 :
			g_cPushedKey = 7;

			if((key_code & KEY_DOWN) && (key_code & KEY_CONT))		// On Push Continuous
			{
				key_code &= ~KEY_DOWN;
				key_code &= ~KEY_DOWN_DO;
				key_code &= ~KEY_UP;
				//key_code &= ~KEY_CONT;
				key_code |= 0x0F;

				g_nWatchDogKey = 0;

				if( !(KeyState & 0x0001) && !(KeyState & 0x0002) ) 				// Auto Display [Auto/HAndLE/conF/EncodEr]
				{
					if( !(KeyDisp & 0x0008) && !(KeyDisp & 0x0004) )  	// Auto
					{
						// NOP
						g_bKeyDownSt = 1;

						key_code |= 0x07;

						if(DispSegment != D_BLANK)
						{
							DispSegment = D_BLANK;

							value = _OFF;
							setDigit(7, value, _DONT_CARE);
							value = _OFF;
							setDigit(6, value, _DONT_CARE);
							value = _OFF;
							setDigit(5, value, _DONT_CARE);
							value = _OFF;
							setDigit(4, value, _DONT_CARE);
							value = _OFF;
							setDigit(3, value, _DONT_CARE);
							value = _OFF;
							setDigit(2, value, _DONT_CARE);
							value = _OFF;
							setDigit(1, value, _DONT_CARE);
							value = _OFF;
							setDigit(0, value, _DONT_CARE);
						}

						SetAuto();
					}
					else if( !(KeyDisp & 0x0008) && (KeyDisp & 0x0004) ) 	// HAndLE
					{
						KeyDisp |= 0x0001;
						KeyDisp &= ~0x0002;

						KeyState |= 0x0001;
						KeyState &= ~0x0002;

						KeyDisp &= ~0x0010;
						KeyState &= ~0x0010;
						g_bKeyDownSt = 0;
						SetClose();

					}
					else if( (KeyDisp & 0x0008) && !(KeyDisp & 0x0004) )	// conF
					{
						KeyDisp |= 0x0002;
						KeyDisp &= ~0x0001;

						KeyState |= 0x0002;
						KeyState &= ~0x0001;

						KeyDisp &= ~0x01E0;
						KeyState &= ~0x01E0;
						g_bKeyDownSt = 0;
						SetConf();
					}
					else if( (KeyDisp & 0x0008) && (KeyDisp & 0x0004) )		// EncodEr
					{
						KeyDisp &= ~0x000C;

						KeyState |= 0x0003;

						g_bKeyDownSt = 0;
						SetEnc();
					}
				}
				else if( !(KeyState & 0x0002) && (KeyState & 0x0001) ) 	// Manual Display [oPEn/cLoSE]
				{
					KeyDisp &= ~0x0003;
					KeyState &= ~0x0003;

					g_bKeyDownSt = 1;

					key_code |= 0x07;

					if(DispSegment != D_BLANK)
					{
						DispSegment = D_BLANK;

						value = _OFF;
						setDigit(7, value, _DONT_CARE);
						value = _OFF;
						setDigit(6, value, _DONT_CARE);
						value = _OFF;
						setDigit(5, value, _DONT_CARE);
						value = _OFF;
						setDigit(4, value, _DONT_CARE);
						value = _OFF;
						setDigit(3, value, _DONT_CARE);
						value = _OFF;
						setDigit(2, value, _DONT_CARE);
						value = _OFF;
						setDigit(1, value, _DONT_CARE);
						value = _OFF;
						setDigit(0, value, _DONT_CARE);
					}

					SetAuto();
				}
				else if( (KeyState & 0x0002) && !(KeyState & 0x0001) ) 	// Config Display Func(0~4) [Value]
				{
					KeyDisp &= ~0x0003;
					KeyState &= ~0x0003;

					g_bKeyDownSt = 1;

					key_code |= 0x07;

					if(DispSegment != D_BLANK)
					{
						DispSegment = D_BLANK;

						value = _OFF;
						setDigit(7, value, _DONT_CARE);
						value = _OFF;
						setDigit(6, value, _DONT_CARE);
						value = _OFF;
						setDigit(5, value, _DONT_CARE);
						value = _OFF;
						setDigit(4, value, _DONT_CARE);
						value = _OFF;
						setDigit(3, value, _DONT_CARE);
						value = _OFF;
						setDigit(2, value, _DONT_CARE);
						value = _OFF;
						setDigit(1, value, _DONT_CARE);
						value = _OFF;
						setDigit(0, value, _DONT_CARE);
					}

					SetAuto();
				}
				else if( (KeyState & 0x0002) && (KeyState & 0x0001) )	// Encoder Display
				{
					if( !(KeyDisp & 0x0008) && !(KeyDisp & 0x0004) )  	// Auto
					{
						KeyDisp &= ~0x0003;
						KeyState &= ~0x0003;

						g_bKeyDownSt = 1;

						key_code |= 0x07;

						if(DispSegment != D_BLANK)
						{
							DispSegment = D_BLANK;

							value = _OFF;
							setDigit(7, value, _DONT_CARE);
							value = _OFF;
							setDigit(6, value, _DONT_CARE);
							value = _OFF;
							setDigit(5, value, _DONT_CARE);
							value = _OFF;
							setDigit(4, value, _DONT_CARE);
							value = _OFF;
							setDigit(3, value, _DONT_CARE);
							value = _OFF;
							setDigit(2, value, _DONT_CARE);
							value = _OFF;
							setDigit(1, value, _DONT_CARE);
							value = _OFF;
							setDigit(0, value, _DONT_CARE);
						}

						SetAuto();
					}
					else if( !(KeyDisp & 0x0008) && (KeyDisp & 0x0004) ) 	// HAndLE
					{
						KeyDisp |= 0x0001;
						KeyDisp &= ~0x0002;

						KeyState |= 0x0001;
						KeyState &= ~0x0002;

						KeyDisp &= ~0x0010;
						KeyState &= ~0x0010;
						g_bKeyDownSt = 0;
						SetClose();

					}
					else if( (KeyDisp & 0x0008) && !(KeyDisp & 0x0004) )	// conF
					{
						KeyDisp |= 0x0002;
						KeyDisp &= ~0x0001;

						KeyState |= 0x0002;
						KeyState &= ~0x0001;

						KeyDisp &= ~0x01E0;
						KeyState &= ~0x01E0;
						g_bKeyDownSt = 0;
						SetConf();
					}
					else if( (KeyDisp & 0x0008) && (KeyDisp & 0x0004) )		// EncodEr
					{
						KeyDisp &= ~0x000C;

						KeyState |= 0x0003;

						g_bKeyDownSt = 0;
						SetEnc();
					}
				}
			}
			else if((key_code & KEY_DOWN) && !(key_code & KEY_DOWN_DO))	// On Push down
			{
				key_code |= KEY_DOWN_DO;

				g_bKeyDownSt = 1;
				//g_bEnc = 0;
				g_nWatchDogKey = 0;
			}
			else if(key_code & KEY_UP) 			// On Push up
			{
				key_code &= ~KEY_DOWN;
				key_code &= ~KEY_DOWN_DO;
				key_code &= ~KEY_UP;
				key_code &= ~KEY_CONT;
				key_code |= 0x0F;

				g_nWatchDogKey = 0;
				if(g_bKeyDownSt)
					DispKeySts();
			}

			break;
		default:
			break;
	}

	if(g_cPushedKey == 7)
	{
		g_cPushedKey = 0xFF;

	}



	//key_code &= ~KEY_DOWN; 	/* Clear key flag */
}  

void DispKeySts()
{
	//enum 	key_mode	{KS_AUTO,KS_MNL,KS_CONF,KS_ENC};
	//enum 	key_func	{KS_FUNC0,KS_FUNC1,KS_FUNC2,KS_FUNC3,KS_FUNC4};
	//enum 	key_manual	{KS_CLOSE,KS_OPEN};

	unsigned char value;
	
	if( !(KeyState & 0x0002) && !(KeyState & 0x0001) ) 				// Display [Auto/HAndLE/conF/EncodEr]
	{
		if( !(KeyDisp & 0x0008) && !(KeyDisp & 0x0004) ) 			// Auto -> HAndLE
		{
			KeyDisp |= (0x0001 << 2);

			if(DispSegment != D_HANDLE)
			{
				DispSegment = D_HANDLE;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = 'H';
				setDigit(5, value, _DONT_CARE);
				value = 'A';
				setDigit(4, value, _DONT_CARE);
				value = 'N';
				setDigit(3, value, _DONT_CARE);
				value = 'D';
				setDigit(2, value, _DONT_CARE);
				value = 'L';
				setDigit(1, value, _DONT_CARE);
				value = 'E';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else if( !(KeyDisp & 0x0008) && (KeyDisp & 0x0004) ) 	// HAndLE -> conF
		{
			KeyDisp &= ~0x000C;
			KeyDisp |= (0x0002 << 2);

			if(DispSegment != D_CONF)
			{
				DispSegment = D_CONF;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'C';
				setDigit(3, value, _DONT_CARE);
				value = 'O';
				setDigit(2, value, _DONT_CARE);
				value = 'N';
				setDigit(1, value, _DONT_CARE);
				value = 'F';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else if( (KeyDisp & 0x0008) && !(KeyDisp & 0x0004) )	// conF -> EncodEr
		{
			KeyDisp &= ~0x000C;
			KeyDisp |= (0x0003 << 2);

			if(DispSegment != D_ENCODER)
			{
				DispSegment = D_ENCODER;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = 'E';
				setDigit(6, value, _DONT_CARE);
				value = 'N';
				setDigit(5, value, _DONT_CARE);
				value = 'C';
				setDigit(4, value, _DONT_CARE);
				value = 'O';
				setDigit(3, value, _DONT_CARE);
				value = 'D';
				setDigit(2, value, _DONT_CARE);
				value = 'E';
				setDigit(1, value, _DONT_CARE);
				value = 'R';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else if( (KeyDisp & 0x0008) && (KeyDisp & 0x0004) )	// EncodEr -> Auto
		{
			KeyDisp &= ~0x000C;

			if(DispSegment != D_AUTO)
			{
				DispSegment = D_AUTO;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'A';
				setDigit(3, value, _DONT_CARE);
				value = 'U';
				setDigit(2, value, _DONT_CARE);
				value = 'T';
				setDigit(1, value, _DONT_CARE);
				value = 'O';
				setDigit(0, value, _DONT_CARE);
			}
		}
	}
	else if( !(KeyState & 0x0002) && (KeyState & 0x0001) ) 	// Display [oPEn/cLoSE]
	{
		if( !(KeyDisp & 0x0010) ) 		// Close
		{
			KeyDisp |= 0x0010;
			KeyState |= 0x0010;

			SetOpen();
		}
		else if( (KeyDisp & 0x0010) ) 	// Open
		{
			KeyDisp &= ~0x0010;
			KeyState &= ~0x0010;

			SetClose();
		}
	}
	else if( (KeyState & 0x0002) && !(KeyState & 0x0001) ) 	// Display Func(0~8) [Value]
	{
		if( !(KeyDisp & 0x0100) && !(KeyDisp & 0x0080) && !(KeyDisp & 0x0040) && !(KeyDisp & 0x0020) ) 			// Func0 -> Func1
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x0020;
			//KeyDisp &= ~0x01C0;

			DispFuncVal(1);
		}
		else if( !(KeyDisp & 0x0100) && !(KeyDisp & 0x0080) && !(KeyDisp & 0x0040) && (KeyDisp & 0x0020) ) 			// Func1 -> Func2
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x0040;
			//KeyDisp &= ~0x01A0;

			DispFuncVal(2);
		}
		else if( !(KeyDisp & 0x0100) && !(KeyDisp & 0x0080) && (KeyDisp & 0x0040) && !(KeyDisp & 0x0020) ) 			// Func2 -> Func3
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x0060;
			//KeyDisp &= ~0x0180;

			DispFuncVal(3);
		}
		else if( !(KeyDisp & 0x0100) && !(KeyDisp & 0x0080) && (KeyDisp & 0x0040) && (KeyDisp & 0x0020) ) 				// Func3 -> Func4
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x0080;
			//KeyDisp &= ~0x0160;

			DispFuncVal(4);
		}
		else if( !(KeyDisp & 0x0100) && (KeyDisp & 0x0080) && !(KeyDisp & 0x0040) && !(KeyDisp & 0x0020) ) 			// Func4 -> Func5
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x00A0;
			//KeyDisp &= ~0x0140;

			DispFuncVal(5);
		}
		else if( !(KeyDisp & 0x0100) && (KeyDisp & 0x0080) && !(KeyDisp & 0x0040) && (KeyDisp & 0x0020) ) 				// Func5 -> Func6
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x00C0;
			//KeyDisp &= ~0x0140;

			DispFuncVal(6);
		}
		else if( !(KeyDisp & 0x0100) && (KeyDisp & 0x0080) && (KeyDisp & 0x0040) && !(KeyDisp & 0x0020) )	 			// Func6 -> Func7
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x00E0;
			//KeyDisp &= ~0x0120;

			DispFuncVal(7);
		}
		else if( !(KeyDisp & 0x0100) && (KeyDisp & 0x0080) && (KeyDisp & 0x0040) && (KeyDisp & 0x0020) )	 			// Func7 -> Func8
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x0100;
			//KeyDisp &= ~0x00E0;

			DispFuncVal(8);
		}
		else if( (KeyDisp & 0x0100) && !(KeyDisp & 0x0080) && !(KeyDisp & 0x0040) && !(KeyDisp & 0x0020) ) 				// Func8 -> Func0
		{
			KeyDisp &= ~0x01E0;
			KeyDisp |= 0x0000;

			DispFuncVal(0);
		}
	}
	else if( (KeyState & 0x0002) && (KeyState & 0x0001) )	// Display Enc
	{
		if( !(KeyDisp & 0x0008) && !(KeyDisp & 0x0004) ) 			// Auto -> HAndLE
		{
			KeyDisp |= (0x0001 << 2);

			if(DispSegment != D_HANDLE)
			{
				DispSegment = D_HANDLE;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = 'H';
				setDigit(5, value, _DONT_CARE);
				value = 'A';
				setDigit(4, value, _DONT_CARE);
				value = 'N';
				setDigit(3, value, _DONT_CARE);
				value = 'D';
				setDigit(2, value, _DONT_CARE);
				value = 'L';
				setDigit(1, value, _DONT_CARE);
				value = 'E';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else if( !(KeyDisp & 0x0008) && (KeyDisp & 0x0004) ) 	// HAndLE -> conF
		{
			KeyDisp &= ~0x000C;
			KeyDisp |= (0x0002 << 2);

			if(DispSegment != D_CONF)
			{
				DispSegment = D_CONF;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'C';
				setDigit(3, value, _DONT_CARE);
				value = 'O';
				setDigit(2, value, _DONT_CARE);
				value = 'N';
				setDigit(1, value, _DONT_CARE);
				value = 'F';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else if( (KeyDisp & 0x0008) && !(KeyDisp & 0x0004) )	// conF -> EncodEr
		{
			KeyDisp &= ~0x000C;
			KeyDisp |= (0x0003 << 2);

			if(DispSegment != D_ENCODER)
			{
				DispSegment = D_ENCODER;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = 'E';
				setDigit(6, value, _DONT_CARE);
				value = 'N';
				setDigit(5, value, _DONT_CARE);
				value = 'C';
				setDigit(4, value, _DONT_CARE);
				value = 'O';
				setDigit(3, value, _DONT_CARE);
				value = 'D';
				setDigit(2, value, _DONT_CARE);
				value = 'E';
				setDigit(1, value, _DONT_CARE);
				value = 'R';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else if( (KeyDisp & 0x0008) && (KeyDisp & 0x0004) )	// EncodEr -> Auto
		{
			KeyDisp &= ~0x000C;

			if(DispSegment != D_AUTO)
			{
				DispSegment = D_AUTO;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'A';
				setDigit(3, value, _DONT_CARE);
				value = 'U';
				setDigit(2, value, _DONT_CARE);
				value = 'T';
				setDigit(1, value, _DONT_CARE);
				value = 'O';
				setDigit(0, value, _DONT_CARE);
			}
		}
	}
}

void init_gpio(void)
{
	EALLOW;
	//GpioMuxRegs.GPAMUX.all=0x03ff;
	GpioMuxRegs.GPAMUX.all=0x0300;
/* GPAMUX: GPIO_A function 0=IOP,1=FUN	I(0)/O(1)
	bit15	0:	C3TRIP,PA15		;IOP		1
	bit14	0:	C2TRIP,PA14		;IOP		1
	bit13	0:	C1TRIP,PA13		;IOP		1
	bit12	0:	TCLKINA,PA12	;IOP		1
	bit11	0:	TDIRA,PA11		;IOP		1
	bit10	0:	CAP3_QEPI1,PA10	;IOP		1
	bit9	0:	CAP2_QEP2,PA9	;IOP		0
	bit8	0:	CAP1_QEP1,PA8	;IOP		0
	bit7	0:	T2PWM_T2CMP,PA7	;IOP		1
	bit6	0:	T1PWM_T1CMP,PA6	;IOP		1
	bit5	0:	PWM6,PA5		;IOP		1	
	bit4	0:	PWM5,PA4		;IOP		1	
	bit3	0:	PWM4,PA3		;IOP		1	
	bit2	0:	PWM3,PA2		;IOP		1	
	bit1	0:	PWM2,PA1		;IOP		1	
	bit0	0:	PWM1,PA0		;IOP		1
	*/
    GpioMuxRegs.GPADIR.all=0xfcff;	// PA DIR: 1=output,0=input
    GpioMuxRegs.GPAQUAL.all=0x0000;	// PA Input Qualification:
    /* 0x00=No, 0x01=SYSCLK/2, 0x02=SYSCLK/4, 0x04=SYSCLK/510 */
    
    GpioMuxRegs.GPBMUX.all=0x0000;   
/* GPBMUX: GPIO_B function 0=IOP,1=FUN	I(0)/O(1)
	bit15	0:	C6TRIP,PB15		;IOP		1
	bit14	0:	C5TRIP,PB14		;IOP		1
	bit13	0:	C4TRIP,PB13		;IOP		1
	bit12	0:	TCLKINB,PB12	;IOP		1
	bit11	0:	TDIRB,PB11		;IOP		1
	bit10	0:	CAP6_QEPI2,PB10	;IOP		1
	bit9	0:	CAP5_QEP4,PB9	;IOP		1
	bit8	0:	CAP4_QEP3,PB8	;IOP		1
	bit7	0:	T4PWM_T4CMP,PB7	;IOP		1
	bit6	0:	T3PWM_T3CMP,PB6	;IOP		1
	bit5	0:	PWM12,PB5		;IOP		1	
	bit4	0:	PWM11,PB4		;IOP		1	
	bit3	0:	PWM10,PB3		;IOP		1	
	bit2	0:	PWM9,PB2		;IOP		1	
	bit1	0:	PWM8,PB1		;IOP		1	
	bit0	0:	PWM7,PB0		;IOP		1
	*/
    GpioMuxRegs.GPBDIR.all=0x0000;	// PB DIR: 1=output,0=input
    GpioMuxRegs.GPBQUAL.all=0x0000;	// PB Input Qualification:
    /* 0x00=No, 0x01=SYSCLK/2, 0x02=SYSCLK/4, 0x04=SYSCLK/510 */
  
    GpioMuxRegs.GPDMUX.all=0x0021;
/* GPDMUX: GPIO_D function 0=IOP,1=FUN
	bit6	0:	T4CTRIP,PD6			;IOP
	bit5	1:	T3CTRIP_PDPINTB,PD5	;FUN
	bit4	0:	res
	bit3	0:	res
	bit2	0:	res
	bit1	0:	T2CTRIP,PD1			;IOP		
	bit0	1:	T1CTRIP_PDPINTA,PD0	;FUN
	*/
    GpioMuxRegs.GPDDIR.all=0x5e;	// DIR: 1=output,0=input
	//	6(PD6),5(PDPINTB),4,3,2,1(PD1),0(PDPINTA)
	//     1       0      1 1 1    1       0	= 0x5e
    GpioMuxRegs.GPDQUAL.all=0x0000;	// PD Input Qualification:
    /* 0x00=No, 0x01=SYSCLK/2, 0x02=SYSCLK/4, 0x04=SYSCLK/510 */
	
    GpioMuxRegs.GPEMUX.all=0x0000; 	
/* GPEMUX: GPIO_E function 0=IOP,1=FUN		I(0)/O(1)
	bit2	0:	XNMI_XINT13,PE2		;IOP		1
	bit1	0:	XINT2_ADCSOC,PE1	;IOP		1	
	bit0	0:	XINT1_XBIO,PE0		;IOP		1
	*/
    GpioMuxRegs.GPEDIR.all=0x07;	// DIR: 1=output,0=input
	GpioDataRegs.GPEDAT.all=0x07;	// all 1 out
   // GpioMuxRegs.GPEQUAL.all=0x0000;	// PE Input Qualification:
    /* 0x00=No, 0x01=SYSCLK/2, 0x02=SYSCLK/4, 0x04=SYSCLK/510 */

    GpioMuxRegs.GPFMUX.all=0x00f0;		 
/* GPFMUX: GPIO_F function 0=IOP,1=FUN		I(0)/O(1)
	bit14	0:	XF,PF14			;IOP			1(CPU_LED)
	bit13	0:	MDR,PF13		;IOP			1
	bit12	0:	MDX,PF12		;IOP			1
	
	bit11	0:	MFSR,PF11		;IOP			1
	bit10	0:	MFSX,PF10		;IOP			1
	bit9	0:	MCLKR,PF9		;IOP			1
	bit8	0:	MCLKX,PF8		;IOP			1
	
	bit7	1:	CANRX,PF7		;FUN			0
	bit6	1:	CANTX,PF6		;FUN			1
	bit5	1:	SCIRXDA,PF5		;FUN			0
	bit4	1:	SCITXDA,PF4		;FUN			1
	
	bit3	0:	SPISTE,PF3		;IOP			1
	bit2	0:	SPICLK,PF2		;IOP			1
	bit1	0:	SPISOMI,PF1		;IOP			1
	bit0	0:	SPISIMO,PF0		;IOP			1
	*/
    GpioMuxRegs.GPFDIR.all=0xff5f;	// DIR: 1=output,0=input

    GpioMuxRegs.GPGMUX.all=0x0030;
/* GPGMUX: GPIO_G function 0=IOP,1=FUN
	bit5	1:	SCIRXDB,PG5		;FUN
	bit4	1:	SCITXDB,PG4		;FUN		
	*/
    GpioMuxRegs.GPGDIR.all=0x10;	// DIR: 1=output,0=input
	//	5(RXDB),4(TXDB),3,2,1,0
	//     0       1    0 0 0 0	= 0x10
	
	EDIS; 
}

/*
void InitPwm(void)
{

asm( "EALLOW");	//Enable write operations in protected registers

// Independent Compare Output Enable	/

	EvaRegs.EXTCONA.bit.INDCOE= 1;

//
//	Write values in the two following registers until all the other
//	registers have been written, as their value depends on the other
//	registers. 							*
//

// Timer 1 Period Regiser	/

	EvaRegs.T1PR = 0xB71A;			//use this value for a frequency of 200 Hz assuming
								    //sysclkout=150 MHz and its pre-scaler=16
								    
// CMPR1 Register Configuration /

	EvaRegs.CMPR1= 0x493D;		//to get 60% of duty cycle this value equals the 1-0.6= 40% period
		
//

	EvaRegs.GPTCONA.all= 0x0050;
	
// bit 15 		0		Reserved 
// bit 14 		0		T2STAT GP timer 2 Status. 
// bit 13 		0		T1STAT GP timer 1 Status. 
// bit 12 		0		T2CTRIPE T2CTRIP Enable. 
// bit 11 		0		T1CTRIPE T1CTRIP Enable. 
// bit 10-9 	00		T2TOADC Start ADC with timer 2 event
// bit 8-7 		00		T1TOADC Start ADC with timer 1 event
// bit 6 		1		TCMPOE Timer compare output enable. 
// bit 5		0		T2CMPOE Timer 2 compare output enable. 
// bit 4 		1		T1CMPOE Timer 1 Compare Output Enable. 
// bit 3-2 		00		T2PIN Polarity of GP timer 2 compare output
// bit 1-0 		00		T1PIN Polarity of GP timer 1 compare output
	

// COMCONA Register Configuration/

	EvaRegs.COMCONA.all = 0x8020;
	
// bit 15 		1:		CENABLE: Compare Enable
// bit 14-13 	00:		CLD1, CLD0: CMPRx Reload Condition
// bit 12 		0:		SVENABLE: PWM Vector Enable
// bit 11-10 	00:		ACTRLD1,ACTRLD0: Action Control Regiser Reload Condition
// bit 9 		0:		FCMPOE: Full Compare Output Enable
// bit 8 		0:		PDPINTA: PDPINTA Status
// bit 7 		0:		FCMP3OE: Compare 3 Output Enable
// bit 6 		0:		FCMP2OE: Compare 2 Output Enable
// bit 5 		1:		FCMP1OE: Compare 1 Output Enable
// bit 4-3 		00:		Reserved
// bit 2 		0:		C3TRIPE: C3TRIP Enable 
// bit 1 		0:		C2TRIPE: C2TRIP Enable 
// bit 0 		0:		C1TRIPE: C1TRIP Enable 

// ACTRA Register Configuration/

	EvaRegs.ACTRA.all = 0x0002;
	
// bit 15 		0:		SVRDIR: PWM Vector rotate direction 
// bit 14-12 	000:	D2-D: PWM Vector basic bits
// bit 11-10 	00:		CMP6ACT1-0 Compare Action in pin 6, CMP6.
// bit 9-8 		00:		CMP5ACT1-0 Compare Action in pin 5, CMP5.
// bit 7-6 		00:		CMP4ACT1-0 Compare Action in pin 4, CMP4.
// bit 5-4 		00:		CMP3ACT1-0 Compare Action in pin 3, CMP3
// bit 3-2 		00:		CMP2ACT1-0 Compare Action in pin 2, CMP2
// bit 1-0 		10		CMP1ACT1-0 Compare Action in el pin 1, CMP1

// T1CON Register Configuration/

	EvaRegs.T1CON.all = 0x9442;
	
// bit 15:14 	10:		Emulation Control bits
// bit 13 		0:		Reserved
// bit 12-11 	10:		TMODE1-TMODE0 Counter Mode Selection
// bit 10-8 	100:	TPS2-TPS0 Pre-scaler: 100= HSPCLK/16
// bit 7 		0:		T2SWT1 Timer 2 trigger by timer 1
// bit 6 		1:		TENABLE Timer Enable
// bit 5-4 		00:		TCLKS(1,0) Clock Source Selection
// bit 3-2 		00:		TCLD(1,0) Timer Compare Register Reload Condition
// bit 1 		1:		TECMPR Compare Operations Enable
// bit 0		0:		SELT1PR: Period Register Selection
						

asm( "EDIS");	//Protected Registers Write Disabled

}


    void Initeva(void)   
04.    {              
05.    // Initialize EVA Timer 1:   
06.    // Setup Timer 1 Registers (EV A)   
07.    //EvaRegs.GPTCONA.bit.T2STAT = 1;      //定时器2递增计数   
08.    //EvaRegs.GPTCONA.bit.T1STAT = 1;      //定时器1递增计数   
09.    EvaRegs.T1CON.all=0;               //定时器控制寄存器   
10.    EvaRegs.T1CON.bit.FREE = 0;           //仿真停止,计数停止   
11.    EvaRegs.T1CON.bit.SOFT = 0;           //   
12.    EvaRegs.T1CON.bit.TMODE=2;            //连续递增   
13.    EvaRegs.T1CON.bit.TPS=7;              //预定标128   
14.    EvaRegs.T1CON.bit.T2SWT1=0;           //使用自己的TENABLE   
15.    EvaRegs.T1CON.bit.TENABLE=0;          //定时器禁止    
16.    EvaRegs.T1CON.bit.TCLKS10=0;          //内部时钟   
17.    EvaRegs.T1CON.bit.TCLD10=0;           //计数为0是装载比较寄存器   
18.    EvaRegs.T1CON.bit.TECMPR=0;           //比较禁止   
19.    EvaRegs.T1CON.bit.SET1PR=0;           //使用自己的周期寄存器   
20.                                    
21.    EvaRegs.T1PR = 0xFFFF;                 //周期寄存器的值,定时器开始启动     
22.    EvaRegs.T1CNT=0;                  
23.    EvaRegs.T2CON.all=0x1700;   
24.    EvaRegs.T2PR = 0xFFFF;    
25.    EvaRegs.T2CNT=0;   
26.       
27.    EvaRegs.CAPCON.all=0;           //捕捉控制寄存器    
28.    EvaRegs.CAPCON.bit.CAPQEPN=01;      //鼓懿痘竦ピ?1,2   
29.    EvaRegs.CAPCON.bit.CAP3EN=1;        //使能捕获单元3    
30.    EvaRegs.CAPCON.bit.CAP3TSEL=0;      //捕获单元3选择定时器2   
31.    EvaRegs.CAPCON.bit.CAP12TSEL=1;     //捕获单元1,2选择定时器1   
32.    EvaRegs.CAPCON.bit.CAP1EDGE=01;     //边沿检测,?仙?   
33.    EvaRegs.CAPCON.bit.CAP2EDGE=01;   
34.    EvaRegs.CAPCON.bit.CAP3EDGE=01;   
35.       
36.    EvaRegs.CAPFIFO.bit.CAP1FIFO=0;   //CAP1FIFO空    
37.    EvaRegs.CAPFIFO.bit.CAP2FIFO=0;   
38.    EvaRegs.CAPFIFO.bit.CAP3FIFO=0;   
39.       
40.    EvaRegs.EVAIFRC.bit.CAP1INT=1;     //清捕获中断1标志位   
41.    EvaRegs.EVAIMRC.bit.CAP1INT=1;     //捕获1中断使能   
42.    EvaRegs.EVAIFRC.bit.CAP2INT=1;     //清捕获中断2标志位   
43.    EvaRegs.EVAIMRC.bit.CAP2INT=1;   
44.    EvaRegs.EVAIFRC.bit.CAP3INT=1;     //清捕获中断3标志位   
45.    EvaRegs.EVAIMRC.bit.CAP3INT=1;   
46.   
47.   
48.    }   


*/

void init_ev(void)
{
	EALLOW;	// This is needed to write to EALLOW protected registers

// EVA Configure T1PWM, T2PWM, PWM1-PWM6 
// Step 1  Initalize the timers
	// Initalize EVA Timer1 
	EvaRegs.T1PR = 4095;       	// Timer1 period
									//use this value for a frequency of 2 KHz assuming
								    //sysclkout=150 MHz and its pre-scaler=16

	EvaRegs.T1CMPR = 0;     	// Timer1 compare
	EvaRegs.T1CNT = 0;      	// Timer1 counter
    // TMODE = continuous up/down
	// Timer enable
	// Timer compare enable
/*** T1CON Register Configuration*/
// bit 15:14 	10:		Emulation Control bits
// bit 13 		0:		Reserved
// bit 12-11 	10:		TMODE1-TMODE0 Counter Mode Selection
// bit 10-8 	100:	TPS2-TPS0 Pre-scaler: 100= HSPCLK/16
// bit 7 		0:		T2SWT1 Timer 2 trigger by timer 1
// bit 6 		1:		TENABLE Timer Enable
// bit 5-4 		00:		TCLKS(1,0) Clock Source Selection
// bit 3-2 		00:		TCLD(1,0) Timer Compare Register Reload Condition
// bit 1 		1:		TECMPR Compare Operations Enable
// bit 0		0:		SELT1PR: Period Register Selection

	EvaRegs.T1CON.all = 0x0842;   

// Step 2  Setup T1PWM and T2PWM
/*** GPTCONA Register Configuration*/
// bit 15 		0		Reserved 
// bit 14 		0		T2STAT GP timer 2 Status. 
// bit 13 		0		T1STAT GP timer 1 Status. 
// bit 12 		0		T2CTRIPE T2CTRIP Enable. 
// bit 11 		0		T1CTRIPE T1CTRIP Enable. 
// bit 10-9 	00		T2TOADC Start ADC with timer 2 event
// bit 8-7 		00		T1TOADC Start ADC with timer 1 event
// bit 6 		1		TCMPOE Timer compare output enable. 
// bit 5		0		T2CMPOE Timer 2 compare output enable. 
// bit 4 		1		T1CMPOE Timer 1 Compare Output Enable. 
// bit 3-2 		00		T2PIN Polarity of GP timer 2 compare output
// bit 1-0 		00		T1PIN Polarity of GP timer 1 compare output

	// Drive T1/T2 PWM by compare logic
	EvaRegs.GPTCONA.bit.TCMPOE = 1;
	// Polarity of GP Timer 1 Compare = Active low
	EvaRegs.GPTCONA.bit.T1PIN = 1;
	// Polarity of GP Timer 2 Compare = Active high
	EvaRegs.GPTCONA.bit.T2PIN = 2;

// Step 3 Enable compare for PWM1-PWM6
	EvaRegs.CMPR1 = ad_data[0]; 	// 0x493D;		//to get 60% of duty cycle this value equals the 1-0.6= 40% period
	EvaRegs.CMPR2 = ad_data[0];
	EvaRegs.CMPR3 = 0x800;
    

/** ACTRA Register Configuration*/
// bit 15 		0:		SVRDIR: PWM Vector rotate direction 
// bit 14-12 	000:	D2-D: PWM Vector basic bits
// bit 11-10 	00:		CMP6ACT1-0 Compare Action in pin 6, CMP6.
// bit 9-8 		00:		CMP5ACT1-0 Compare Action in pin 5, CMP5.
// bit 7-6 		00:		CMP4ACT1-0 Compare Action in pin 4, CMP4.
// bit 5-4 		00:		CMP3ACT1-0 Compare Action in pin 3, CMP3
// bit 3-2 		00:		CMP2ACT1-0 Compare Action in pin 2, CMP2
// bit 1-0 		10		CMP1ACT1-0 Compare Action in el pin 1, CMP1

    // Compare action control.  Action that takes place
    // on a cmpare event
    // output pin 1 CMPR1 - active high
    // output pin 2 CMPR1 - active low
    // output pin 3 CMPR2 - active high
    // output pin 4 CMPR2 - active low
    // output pin 5 CMPR3 - active high
    // output pin 6 CMPR3 - active low
    EvaRegs.ACTRA.all = 0x0999;		// change 0x0666 => 0x0999
	EvaRegs.DBTCONA.all = 0x09ec; 	// Deadband: enable


/*** COMCONA Register Configuration*/
// bit 15 		1:		CENABLE: Compare Enable
// bit 14-13 	00:		CLD1, CLD0: CMPRx Reload Condition
// bit 12 		0:		SVENABLE: PWM Vector Enable
// bit 11-10 	00:		ACTRLD1,ACTRLD0: Action Control Regiser Reload Condition
// bit 9 		0:		FCMPOE: Full Compare Output Enable
// bit 8 		0:		PDPINTA: PDPINTA Status
// bit 7 		0:		FCMP3OE: Compare 3 Output Enable
// bit 6 		0:		FCMP2OE: Compare 2 Output Enable
// bit 5 		1:		FCMP1OE: Compare 1 Output Enable
// bit 4-3 		00:		Reserved
// bit 2 		0:		C3TRIPE: C3TRIP Enable 
// bit 1 		0:		C2TRIPE: C2TRIP Enable 
// bit 0 		0:		C1TRIPE: C1TRIP Enable 

    EvaRegs.COMCONA.all = 0xA600;

	// Initalize EVA Timer2 for Encoder
	EvaRegs.T2PR = 0xffff;		// Timer2 period
	EvaRegs.T2CNT = 0x0000;		// Timer2 counter
	EvaRegs.T2CON.all = 0xd870;	// FREE=SOFT=1, Dir_UP_DN,x/1,QEP

	EDIS;       // This is needed to disable write to EALLOW protected registers
}

void Int2Ascii(long nNum, char *ch)
{
	char tmp0[20], *tmp;
	int i=0, j=0;
	tmp = tmp0;

	if(nNum < 0)
	{
		*ch = '-';
		ch++;
	}

	nNum = ABS(nNum);

	for(i=0; 1; i++)
	{
		if(i>19)
			break;

		if(nNum/10 < 1)
		{
			*ch = '0' + nNum;
			ch++;
			for(j=i-1; j>=0; j--)
			{
				*ch = *(tmp0+j);
				ch++;
			}
			*ch = '\0';
			break;
		}
		else
		{
			j = nNum % 10;
			*tmp = '0' + j;
			tmp++;
			nNum /= 10;
		}
	}
}

void SciSend(char *p)
{
	char    rd;
	while(rd = *p++)
	{   
		sci_putc(rd);          
	}
}

void CheckDac()
{
	char bNop;
	int dir, nOffset, nTop, nBottom;
	long lDiff, lDiff2, lDiff3, lDiff4, lCntDiff;
	float fRad;
	WORD wConvOut;

	int AngleOffset;


//	int nMode = RomData[0][0];
	int nMode = 0;
	int nTime = RomData[nMode][0]*1000/TimerPeriod; //  * 1 [mSec]
	int nShape = RomData[nMode][1];
	int nMin = RomData[nMode][2];
	int nMax = RomData[nMode][3];
	int nFiler = RomData[nMode][4];
	int nEncDir = RomData[nMode][5];
	int nShutterDir = RomData[nMode][6];
	int nAngleOffset = RomData[nMode][7];
	int DAC_MAX = RomData[nMode][8];				// [mV]

	float DAC_LIMIT = (float)DAC_MAX / 1000.0;		// [V]
	float DAC_RATIO = (float)DAC_LIMIT / 10.0;		// DAC output Max Voltage Ratio. (10V -> DAC_LIMIT [V])
	//float DAC_1DEG = (DAC_1V * DAC_RATIO / DEG_MAX);
	float DAC_1DEG = (2048 / DEG_MAX) * DAC_RATIO;
 
	g_nEncDir = nEncDir;
		
	if(nTime < CutOffTimeMin*1000/TimerPeriod)
		nTime = CutOffTimeMin*1000/TimerPeriod;
	else if(nTime > CutOffTimeMax*1000/TimerPeriod)
		nTime = CutOffTimeMax*1000/TimerPeriod;
/*
	if(nShape < Scurve)
		nShape = Scurve;
	else if(nShape > Tcurve)
		nShape = Tcurve;

	if(nMin < AngleMin)
		nMin = AngleMin;
	else if(nMin > nMax)
	{
		nMin = nMax;
		if(nMin > AngleMax)
			nMin = AngleMax;
	}

	if(nMax > AngleMax)
		nMax = AngleMax;
	else if(nMax < nMin)
	{
		nMax = nMin;
		if(nMax < AngleMin)
			nMax = AngleMin;
	}	

	if(nFiler < EncFilterDefault)
		nFiler = EncFilterDefault;

	if(nEncDir != -1 && nEncDir != 1)
		nEncDir = EncDir;

	if(nShutterDir != -1 && nShutterDir != 1)
		nShutterDir = ShutterDir;

	if(nAngleOffset > 100)
		nAngleOffset = 100;
	else if(nAngleOffset < 0)
		nAngleOffset = 0;
*/	

	AngleOffset = (int)((680 * nAngleOffset) / 100);

//	nTop = 2048 + 102.4 * nMax;
//	nBottom = 2048 + 102.4 * nMin;
//	nTop = 2048 + 25.55 * nMax - AngleOffset;
//	nBottom = 2048 + 25.55 * nMin + AngleOffset;
	nTop = 2048 + DAC_1DEG * (nMax - AngleOffset);
	nBottom = 2048 + DAC_1DEG * (nMin + AngleOffset);
	g_eCntJudge = nFiler;

	// 1msec period
	CpuTimer0.InterruptCount++;
	g_tCnt[0]++;

	dir = EvaRegs.GPTCONA.bit.T2STAT;	// get T2STAT(0=down,1=up)
	g_eCnt = EvaRegs.T2CNT;

	//lDiff = (long)(g_eCnt - g_eCntPrev);
	lDiff = g_lDiff = (long)(g_eCnt - g_eCntPrev[0]);
	lDiff2 = (long)(g_eCntPrev[0] - g_eCntPrev[1]);
	lDiff3 = (long)(g_eCntPrev[1] - g_eCntPrev[2]);
	lDiff4 = (long)(g_eCntPrev[2] - g_eCntPrev[3]);

	bNop=0;
	if(dir==1 && lDiff > 0)
		g_lCnt += lDiff;
	else if(dir!=1 && lDiff < 0)
		g_lCnt += lDiff;
	else 
	{
//		if((dir==1 && g_dir==1) && (lDiff < 0 && lDiff2 < 0))
//		if((dir==1) && (lDiff < 0 && lDiff2 > 0 && lDiff3 > 0 && lDiff4 > 0))
//		else if((dir!=1 && g_dir!=1) && (lDiff > 0 && lDiff2 > 0))
//		else if((dir!=1) && (lDiff > 0 && lDiff2 < 0 && lDiff3 < 0 && lDiff4 < 0))
		if((dir==1) && (lDiff < 0))
			g_lCnt += (lDiff+65536);
		else if((dir!=1) && (lDiff > 0))
			g_lCnt += (lDiff-65536);
		else
			bNop=1;
	}

	if(!bNop)
	{	
		g_eCntPrev[3] = g_eCntPrev[2];
		g_eCntPrev[2] = g_eCntPrev[1];
		g_eCntPrev[1] = g_eCntPrev[0];
	}

	g_eCntPrev[0] = g_eCnt;
	g_dir = dir;


	lCntDiff = g_lCnt - g_lCntPrev2;

	if(lCntDiff < -65000 || lCntDiff > 65000)
	{
		g_lCnt = g_lCntPrev2;
		return;				// modify 20220712
	}



	if(g_bAuto && (g_nAutoWinker > (WinkNum*2)) && g_bIoEnable)
	{
		if((g_tCnt[0]%10)==0)					// 1 msec
		{
			g_tCnt[0] = 0;
		
			//lDiff = (long)(g_eCnt - g_eCntPrev2);
			//g_eCntPrev2 = g_eCnt;
			lDiff = (long)(g_lCnt - g_lCntPrev2);

			lDiff = ABS(lDiff);

//			g_bRun = 1;
			if(lDiff > g_eCntJudge && !g_bRun && !g_bRunF)
			{
				g_bRun = 1;
				g_nStopCnt = 0;
			}
			else if(lDiff <= g_eCntJudge && g_bRun && g_bRunF)
			{
				if(g_nStopCnt >= StopConfirmCnt)
				{
					g_bRun = 0;
					g_nStopCnt = 0;
				}
				else
				{
					g_nStopCnt++;
				}
			}
		}
	}
	else if(g_bClose && (g_nCloseWinker > (WinkNum*2)))
	{
		if(g_bRun && g_bRunF) // if Open...
			g_bRun = 0;
	}
	else if(g_bOpen && (g_nOpenWinker > (WinkNum*2)))
	{
		if(!g_bRun && !g_bRunF) // if Close...
			g_bRun = 1;
	}
	else if(g_bAuto && !g_bIoEnable)
	{
		g_bRun = 0;
		g_nStopCnt = 0;
	}

	// Check Encoder Count...
	if(g_bRun && !g_bRunF)
	{
		// shutter Open

		if(nShape != Scurve)
		{
			if((nBottom+g_nIdx) < nTop)
			{
				nOffset = (nTop-nBottom) / nTime;
				g_nIdx += nOffset;			// Rising Time is 4096/100 mSec
				g_outV = nBottom + g_nIdx;
			}
			else
			{
				g_bRunF = 1;
				g_nIdx = 0;
			}
		}
		else
		{
			if((nBottom+g_nIdx) < nTop)
			{
				nOffset = (nTop-nBottom) / nTime;
				g_nIdx += nOffset;			// Rising Time is 4096/100 mSec

				fRad = 1.0*PI*((float)g_nIdx/(float)(nTop-nBottom));
				//fRad = 1.0*PI*((float)g_nIdx/(float)(nTop-1));
				if(fRad < PI/2.0)
						g_outV = nBottom + (float)(nTop-nBottom-1) * (1.0 - cos(fRad));
						//g_outV = nBottom + (float)(nTop-nBottom-1)/2.0 * (1.0 - cos(fRad));
				else
						g_outV = nBottom + (float)(nTop-nBottom-1) * (1.0 + sin(fRad-PI/2.0));
						//g_outV = nBottom + (float)(nTop-nBottom-1)/2.0 * (1.0 + sin(fRad-PI/2.0));
			}
			else
			{
				g_bRunF = 1;
				g_nIdx = 0;
			}
		}
	}
	else if(!g_bRun && g_bRunF)
	{
		// shutter Close

		if(nShape != Scurve)
		{
			if((nTop-g_nIdx) > nBottom)
			{
				nOffset = (nTop-nBottom) / nTime;
				g_nIdx += nOffset;			// Rising Time is 4096/100 mSec
				g_outV = (nTop-1) - g_nIdx;
			}
			else
			{
				g_bRunF = 0;
				g_nIdx = 0;
			}
		}
		else
		{
			if((nTop-g_nIdx) > nBottom)
			{
				nOffset = (nTop-nBottom) / nTime;
				g_nIdx += nOffset;			// Rising Time is 4096/100 mSec

				fRad = 1.0*PI*((float)g_nIdx/(float)(nTop-nBottom));
//				fRad = 1.0*PI*((float)g_nIdx/(float)(nTop-1));
				if(fRad < PI/2.0)
						g_outV = nBottom + (float)(nTop-nBottom-1) * (1.0 + cos(fRad));
						//g_outV = nBottom + (float)(nTop-nBottom-1)/2.0 * (1.0 + cos(fRad));
				else
						g_outV = nBottom + (float)(nTop-nBottom-1) * (1.0 - sin(fRad-PI/2.0));
						//g_outV = nBottom + (float)(nTop-nBottom-1)/2.0 * (1.0 - sin(fRad-PI/2.0));
			}
			else
			{
				g_bRunF = 0;
				g_nIdx = 0;
			}
		}
	}


	if(g_outVprev != g_outV && g_bInitDone)
	{
/*		if(g_outV < 0)
			g_outV = 0;
		if(g_outV > 4095)
			g_outV = 4095;*/


/*		if(g_outV < 1537)
			g_outV = 1537;
		if(g_outV > 2559)
			g_outV = 2559;*/
		if(g_outV < 2048-(DAC_LIMIT*DAC_1V))
			g_outV = 2048-(DAC_LIMIT*DAC_1V);
		if(g_outV > 2048+(DAC_LIMIT*DAC_1V))
			g_outV = 2048+(DAC_LIMIT*DAC_1V);

		g_outVprev = g_outV;

		wConvOut = (WORD)(nBottom+nTop-g_outV);
		InitDac();

		if(nShutterDir == -1)
			OutDac(0, (WORD)g_outV);
		else
			OutDac(0, wConvOut);
	}

	g_lCntPrev2 = g_lCnt;

}

interrupt void cpu_timer0_isr(void)
{
	g_IoIn = IN_IO2811 & IN_MASK;

	CheckDac();

	g_tCnt[1]++;
	if((g_tCnt[1]%10)==0)	// 10 msec
	{	
		g_tCnt[1] = 0;				
		CheckKey();

		g_tCnt[2]++;
		if((g_tCnt[2]%30)==0)
		{
			g_tCnt[2] = 0;
			KickDog();	// WatchDog Reset				
			LED_TOGGLE;	// watchdog of cpu_timer0_isr() 
		}
	}

	// Acknowledge this interrupt to recieve more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;	
}

void DispAutoVal()
{
	unsigned char value;
	int nTop, nBottom, nOffset;
	int AngleOffset;

	int nMin = RomData[0][2];
	int nMax = RomData[0][3];
	int nTime = RomData[0][0]*1000/TimerPeriod; //  * 1 [mSec]
	int nAngleOffset = RomData[0][7];
	int DAC_MAX = RomData[0][8];					// [mV]

	float DAC_LIMIT = (float)DAC_MAX / 1000.0;		// [V]
	float DAC_RATIO = (float)DAC_LIMIT / 10.0;		// DAC output Max Voltage Ratio. (10V -> DAC_LIMIT [V])
	//float DAC_1DEG = (DAC_1V * DAC_RATIO / DEG_MAX);
	float DAC_1DEG = (2048 / DEG_MAX) * DAC_RATIO;
  

	if(nTime < CutOffTimeMin*1000/TimerPeriod)
		nTime = CutOffTimeMin*1000/TimerPeriod;
	else if(nTime > CutOffTimeMax*1000/TimerPeriod)
		nTime = CutOffTimeMax*1000/TimerPeriod;


	if(nAngleOffset > 100)
		nAngleOffset = 100;
	else if(nAngleOffset < 0)
		nAngleOffset = 0;
	

	AngleOffset = (int)((680 * nAngleOffset) / 100);

		
	nTop = 2048 + DAC_1DEG * (nMax - AngleOffset);
	nBottom = 2048 + DAC_1DEG * (nMin + AngleOffset);
//	nTop = 2048 + 25.55 * nMax - AngleOffset;
//	nBottom = 2048 + 25.55 * nMin + AngleOffset;
//	nTop = 2048 + 102.4 * nMax;
//	nBottom = 2048 + 102.4 * nMin;
	nOffset = (nTop-nBottom) / nTime;

	if( /*(nBottom-nOffset*2) <= g_outV && */g_outV <= (nBottom+nOffset*2) )
	{
		// Display "CLOSE"
		if(DispSegment != D_CLOSE)
		{
			DispSegment = D_CLOSE;

			value = _OFF;
			setDigit(7, value, _DONT_CARE);
			value = _OFF;
			setDigit(6, value, _DONT_CARE);
			value = _OFF;
			setDigit(5, value, _DONT_CARE);
			value = 'C';
			setDigit(4, value, _DONT_CARE);
			value = 'L';
			setDigit(3, value, _DONT_CARE);
			value = 'O';
			setDigit(2, value, _DONT_CARE);
			value = 'S';
			setDigit(1, value, _DONT_CARE);
			value = 'E';
			setDigit(0, value, _DONT_CARE);
		}
	}
	else if( (nTop-nOffset*2) <= g_outV/* && g_outV <= (nTop+nOffset*2)*/ )
	{
		// Display "OPEN"
		if(DispSegment != D_OPEN)
		{
			DispSegment = D_OPEN;

			value = _OFF;
			setDigit(7, value, _DONT_CARE);
			value = _OFF;
			setDigit(6, value, _DONT_CARE);
			value = _OFF;
			setDigit(5, value, _DONT_CARE);
			value = _OFF;
			setDigit(4, value, _DONT_CARE);
			value = 'O';
			setDigit(3, value, _DONT_CARE);
			value = 'P';
			setDigit(2, value, _DONT_CARE);
			value = 'E';
			setDigit(1, value, _DONT_CARE);
			value = 'N';
			setDigit(0, value, _DONT_CARE);
		}
	}
	else
	{
		// Display "NOP"
		if(DispSegment != D_NOP)
		{
			DispSegment = D_NOP;

			value = _OFF;
			setDigit(7, value, _DONT_CARE);
			value = _OFF;
			setDigit(6, value, _DONT_CARE);
			value = _OFF;
			setDigit(5, value, _DONT_CARE);
			value = _OFF;
			setDigit(4, value, _DONT_CARE);
			value = _OFF;
			setDigit(3, value, _DONT_CARE);
			value = 'N';
			setDigit(2, value, _DONT_CARE);
			value = 'O';
			setDigit(1, value, _DONT_CARE);
			value = 'P';
			setDigit(0, value, _DONT_CARE);
		}
	}
}

void DispEncVal()
{
	unsigned char value[8];
	long lCntDisp;

	int nMode = 0;
	int nEncDir = RomData[nMode][5];
	g_nEncDir = nEncDir;

	if(nEncDir != -1 && nEncDir != 1)
		nEncDir = EncDir;

	lCntDisp = (long)nEncDir * g_lCnt;

//	if(!g_bAuto || (g_nAutoWinker <= (WinkNum*2)) || !g_bEnc)
//		return;

	DispSegment = D_VAL;

	g_lCntABS = ABS(g_lCnt);

	value[7] = ((g_lCntABS%100000000) / 10000000);
	if(value[7] == 0)
		setDigit(7, _OFF, _DONT_CARE); // 7Leds all off
	else
		setDigit(7, value[7], _DONT_CARE);

	value[6] = ((g_lCntABS%10000000) / 1000000);
	if(value[7] == 0 && value[6] == 0)
		setDigit(6, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(6, value[6], _DONT_CARE);

		if(lCntDisp < 0 && value[7] == 0)
			setDigit(7, _MINUS, _DONT_CARE); // '-'
	}

	value[5] = ((g_lCntABS%1000000) / 100000);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0)
		setDigit(5, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(5, value[5], _DONT_CARE);

		if(lCntDisp < 0 && value[7] == 0 && value[6] == 0)
			setDigit(6, _MINUS, _DONT_CARE); // '-'
	}

	value[4] = ((g_lCntABS%100000) / 10000);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0)
		setDigit(4, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(4, value[4], _DONT_CARE);

		if(lCntDisp < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0)
			setDigit(5, _MINUS, _DONT_CARE); // '-'
	}

	value[3] = ((g_lCntABS%10000) / 1000);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0)
		setDigit(3, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(3, value[3], _DONT_CARE);

		if(lCntDisp < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0)
			setDigit(4, _MINUS, _DONT_CARE); // '-'
	}

	value[2] = ((g_lCntABS%1000) / 100);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0 && value[2] == 0)
		setDigit(2, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(2, value[2], _DONT_CARE);

		if(lCntDisp < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
			&& value[3] == 0)
			setDigit(3, _MINUS, _DONT_CARE); // '-'
	}

	value[1] = ((g_lCntABS%100) / 10);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0 && value[2] == 0 && value[1] == 0)
		setDigit(1, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(1, value[1], _DONT_CARE);

		if(lCntDisp < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
			&& value[3] == 0 && value[2] == 0)
			setDigit(2, _MINUS, _DONT_CARE); // '-'
	}

	value[0] = (g_lCntABS%10);
	setDigit(0, value[0], _DONT_CARE);
	if(lCntDisp < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0 && value[2] == 0 && value[1] == 0)
		setDigit(1, _MINUS, _DONT_CARE); // '-'

}

void DispFuncVal(int nFunc)
{
	unsigned char value[8];
	int nFuncDataAbs;
	int nFuncData = RomData[0][nFunc];
/*
	switch(nFunc)
	{
	case 0:
		if(DispSegment != D_FUNC0)
			DispSegment = D_FUNC0;
		else
			return;
		break;

	case 1:
		if(DispSegment != D_FUNC1)
			DispSegment = D_FUNC1;
		else
			return;
		break;

	case 2:
		if(DispSegment != D_FUNC2)
			DispSegment = D_FUNC2;
		else
			return;
		break;

	case 3:
		if(DispSegment != D_FUNC3)
			DispSegment = D_FUNC3;
		else
			return;
		break;

	case 4:
		if(DispSegment != D_FUNC4)
			DispSegment = D_FUNC4;
		else
			return;
		break;

	case 5:
		if(DispSegment != D_FUNC5)
			DispSegment = D_FUNC5;
		else
			return;
		break;

	case 6:
		if(DispSegment != D_FUNC6)
			DispSegment = D_FUNC6;
		else
			return;
		break;

	case 7:
		if(DispSegment != D_FUNC7)
			DispSegment = D_FUNC7;
		else
			return;

	case 8:
		if(DispSegment != D_FUNC8)
			DispSegment = D_FUNC8;
		else
			return;

		break;
	}
*/
	nFuncDataAbs = ABS(nFuncData);

	value[7] = ((nFuncDataAbs%100000000) / 10000000);
	if(value[7] == 0)
		setDigit(7, _OFF, _DONT_CARE); // 7Leds all off
	else
		setDigit(7, value[7], _DONT_CARE);

	value[6] = ((nFuncDataAbs%10000000) / 1000000);
	if(value[7] == 0 && value[6] == 0)
		setDigit(6, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(6, value[6], _DONT_CARE);

		if(nFuncData < 0 && value[7] == 0)
			setDigit(7, _MINUS, _DONT_CARE); // '-'
	}

	value[5] = ((nFuncDataAbs%1000000) / 100000);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0)
		setDigit(5, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(5, value[5], _DONT_CARE);

		if(nFuncData < 0 && value[7] == 0 && value[6] == 0)
			setDigit(6, _MINUS, _DONT_CARE); // '-'
	}

	value[4] = ((nFuncDataAbs%100000) / 10000);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0)
		setDigit(4, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(4, value[4], _DONT_CARE);

		if(nFuncData < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0)
			setDigit(5, _MINUS, _DONT_CARE); // '-'
	}

	value[3] = ((nFuncDataAbs%10000) / 1000);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0)
		setDigit(3, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(3, value[3], _DONT_CARE);

		if(nFuncData < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0)
			setDigit(4, _MINUS, _DONT_CARE); // '-'
	}

	value[2] = ((nFuncDataAbs%1000) / 100);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0 && value[2] == 0)
		setDigit(2, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(2, value[2], _DONT_CARE);

		if(nFuncData < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
			&& value[3] == 0)
			setDigit(3, _MINUS, _DONT_CARE); // '-'
	}

	value[1] = ((nFuncDataAbs%100) / 10);
	if(value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0 && value[2] == 0 && value[1] == 0)
		setDigit(1, _OFF, _DONT_CARE); // 7Leds all off
	else
	{
		setDigit(1, value[1], _DONT_CARE);

		if(nFuncData < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
			&& value[3] == 0 && value[2] == 0)
			setDigit(2, _MINUS, _DONT_CARE); // '-'
	}

	value[0] = (nFuncDataAbs%10);
	setDigit(0, value[0], _DONT_CARE);
	if(nFuncData < 0 && value[7] == 0 && value[6] == 0 && value[5] == 0 && value[4] == 0
		&& value[3] == 0 && value[2] == 0 && value[1] == 0)
		setDigit(1, _MINUS, _DONT_CARE); // '-'

}

void InitVal()
{
	unsigned char nMode, i;

	g_IoIn = 0xFF;
	g_IoInF = 0xFF;
	g_bIoEnable = 0;
	g_bInitDone = 0;

	g_eCnt = 0;	

	for(i=0;i<10;i++)		// pulse
		g_eCntPrev[i] = 0; 		// pulse

	g_eCntPrev2 = 0; 		// pulse
	g_eCntPrev3 = -1; 		// pulse
	g_eCntJudge = 2;	 	// pulse

	g_lCnt = 0;				// pulse
	g_lCntPrev2 = 0;				// pulse
	g_lCntABS = 0; 			// pulse
	g_lCntPrev = -1; 		// pulse

	g_tCnt[0] = 0;				// 1msec period 
	g_tCnt[1] = 0;				// 1msec period 
	g_tCnt[2] = 0;				// 1msec period 
	g_dutyVoutS = 100;		// %
	g_bRun = 0;				// stop event
	g_bRunF = 1;			// stop state

	g_nIdx = 0;				// Vout Sampling Index
	g_outV = 0;				// 0~4096 [mV]
	g_outVprev = 1;				// 0~4096 [mV]

	g_nStopCnt = 0;			// StopConfirmCnt 鳖瘤 墨款飘 饶 Close

	g_bDispAuto = 0;
	g_bDispAutoF = 0;
	g_bDispIoEnable = 0;
	g_bDispIoEnableF = 0;

	g_bRepeatTest = 0;
	g_bAuto = 0;
	g_bClose = 0;
	g_bOpen = 0;
	g_bEnc = 0;
	g_bConf = 0;
	g_bAutoF = 0;
	g_bCloseF = 0;
	g_bOpenF = 0;
	g_bEncF = 0;
	g_bConfF = 0;
	g_bAutoWinker = 0;
	g_bCloseWinker = 0;
	g_bOpenWinker = 0;
	g_bEncWinker = 0;
	g_bConfWinker = 0;
	g_nAutoWinker = 0;
	g_nCloseWinker = 0;
	g_nOpenWinker = 0;
	g_nEncWinker = 0;
	g_nConfWinker = 0;
	g_nWinkDelay = 0;

	g_bKeyDown = 0;
	g_bKeyDownSt = 0;
	g_nWatchDogKey = 0;
	g_cPushedKey = 0xFF;
	
	DispSegment = D_BLANK;

}

void InitSegments()
{
	int i;
	unsigned char value[8];

	for(i=0;i<8;i++) 
	{
		Max7219_Status[i]=0x00;
		Max7219_Dot[i]=0;
	}

//	out_Max7219(OP_DISPLAYTEST,1);
	//scanlimit is set to max on startup
	setScanLimit(7);
	//decode is done in source
	out_Max7219(OP_DECODEMODE,0);
	clearDisplay();
	setIntensity(0);	// 15 = brightest
	//we go into shutdown-mode on startup
	shutdown(0);		// turns on display
}


void DispData()
{
	char sEnc[20];

	if(g_lCnt != g_lCntPrev)
	{
		g_lCntPrev = g_lCnt;

//		lcd_gotoxy(0,0);
//		lcd_puts("Encoder Value:");

//		lcd_gotoxy(0,1);
//		if(g_lCnt<0) 
//			lcd_putc('-');					// Right direction
//		else
//			lcd_putc(' ');					// Right direction
		
//		lcd_gotoxy(1,1);
//		lcd_decimal_int(ABS(g_lCnt));			// display encoder conut

		Int2Ascii(g_lCnt, sEnc);
		SciSend(":e,");
		SciSend(sEnc);
		sci_puts(";\r\n");
	}
}


// for EEPRom ========================================================

int EepRead(int nMode, int nFunc)
{
	int nRtn, nUp, nDn;
	BYTE nAddr;
	BYTE temp1, temp2;

	//return 0;
	
	nAddr = (BYTE)((nMode * MaxFunc + nFunc) * 2);
	nUp = eep_read_byte(0, nAddr++);
	nDn = eep_read_byte(0, nAddr);
	nRtn = ((nUp << 8) & 0xFF00) + (nDn & 0x00FF);
	//delay_us(5);
	return nRtn;
}

void EepWrite(int nMode, int nFunc, int nData)
{
	int nUp, nDn;
	BYTE nAddr;
	
	nAddr = (BYTE)((nMode * MaxFunc + nFunc) * 2);
	nUp = ((nData & 0xFF00) >> 8);
	nDn = (nData & 0x00FF);
	
	eep_write_byte(0, nAddr++, (BYTE)nUp);
	eep_write_byte(0, nAddr, (BYTE)nDn);
}


void ReloadEep()
{
	int nMode, nTemp;

	//return;

	DINT;

	//=========== for RomData ========================================
	//#define	CutOffTimeMin		5
	//#define CutOffTimeMax		1000
	//#define	CutOffTimeDefault	40
	//#define	Scurve				1
	//#define	Tcurve				2
	//#define AngleMin			-20
	//#define AngleMax			20
	//#define EncFilterDefault	100
	//#define	EncDir				1
	//#define	ShutterDir			1
	//==================================================================

	nMode = 0;		// Head information
	RomData[nMode][0] = EepRead(nMode, 0);
	RomData[nMode][1] = EepRead(nMode, 1);
	RomData[nMode][2] = EepRead(nMode, 2);
	RomData[nMode][3] = EepRead(nMode, 3);
	RomData[nMode][4] = EepRead(nMode, 4);
	RomData[nMode][5] = EepRead(nMode, 5);
	RomData[nMode][6] = EepRead(nMode, 6);
	RomData[nMode][7] = EepRead(nMode, 7);
	RomData[nMode][8] = EepRead(nMode, 8);

	EINT;
//	lcd_gotoxy(10,1);
//	lcd_puts("Read");			// display encoder conut
}


void InitCommPc()
{
	unsigned int i;

	g_nStep = 0;
	g_cDataIdx = 0;
	g_nData = CMD_NONE;
	g_cWatchDogComm = 0;
	g_nMode = 0;
	g_nFunc = 0;

	for( i=0; i<10; i++)
	{
		g_pData[i] = 0;

		if(i<5)
		{
			g_BufRead[i] = 0;
			g_BufWrite[i] = 0;
		}
	}
}

int GetData()
{
	int i, nReturnVal, nSign, nStIdx;
	nReturnVal= CMD_NONE;

	g_cWatchDogComm++;

	while(sci_rx_ready())
	{
		g_cSerialInput = (char)sci_getc();
		//g_cWatchDogComm++;

		if(g_cSerialInput >= '0' && g_cSerialInput <= '9')
		{
			g_pData[g_cDataIdx] = g_cSerialInput - '0';
			g_cDataIdx++;
		}
		else if(g_cSerialInput == '-' || g_cSerialInput == '+') //(g_cSerialInput == 59) // 59 is ';'
		{
			if(g_cSerialInput == '-')
			{
				g_pData[g_cDataIdx] = g_cSerialInput;
				g_cDataIdx++;
			}
		}
		else if(g_cSerialInput == ',' || g_cSerialInput == ';') //(g_cSerialInput == 59) // 59 is ';'
		{
			nReturnVal = 0;
			nSign = 1;
			nStIdx = 0;
			if(g_pData[0] == '-')
			{
				nSign = -1;
				nStIdx = 1;
	}

			for(i = nStIdx ; i < g_cDataIdx ; i++)
				nReturnVal += g_pData[i] * pow( 10, (g_cDataIdx-1-i) );	

			nReturnVal *= nSign;

			// Data啊 沥惑利栏肺 涝仿登菌栏搁,...
			g_nData = nReturnVal;
			return 0;
		}
		else
			return CMD_ERROR;
	}

	return nReturnVal;
}

void ExeCmd(int nCmd)
{
	char sVal[10]={0};
	char sSend[50]={0};

	switch(nCmd)
	{
	case IsConnected:
		SciSend(":y;");
		sci_puts("\r\n");
		Int2Ascii(g_lCnt, sSend);
		SciSend(":e,");
		SciSend(sSend);
		sci_puts(";\r\n");
		break;
	case ReadArrayData:
		RomData[g_nMode][0] = EepRead(g_nMode, 0);
		RomData[g_nMode][1] = EepRead(g_nMode, 1);
		RomData[g_nMode][2] = EepRead(g_nMode, 2);
		RomData[g_nMode][3] = EepRead(g_nMode, 3);
		RomData[g_nMode][4] = EepRead(g_nMode, 4);
		RomData[g_nMode][5] = EepRead(g_nMode, 5);
		RomData[g_nMode][6] = EepRead(g_nMode, 6);
		RomData[g_nMode][7] = EepRead(g_nMode, 7);
		RomData[g_nMode][8] = EepRead(g_nMode, 8);

		SciSend(":ra,");
		Int2Ascii(g_nMode, sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][0], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][1], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][2], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][3], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][4], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][5], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][6], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][7], sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][8], sVal);
		SciSend(sVal);
		sci_puts(";\r\n");

//		lcd_gotoxy(10,1);
//		lcd_puts("Read");			// display encoder conut

		break;
	case ReadData:
		RomData[g_nMode][g_nFunc] = EepRead(g_nMode, g_nFunc);
		SciSend(":r,");
		Int2Ascii(g_nMode, sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(g_nFunc, sVal);
		SciSend(sVal);
		SciSend(",");
		Int2Ascii(RomData[g_nMode][g_nFunc], sVal);
		SciSend(sVal);
		sci_puts(";\r\n");

//		lcd_gotoxy(10,1);
//		lcd_puts("Read");			// display encoder conut

		break;
	case WriteArrayData:
		EepWrite(g_nMode,0,RomData[g_nMode][0]);
		EepWrite(g_nMode,1,RomData[g_nMode][1]);
		EepWrite(g_nMode,2,RomData[g_nMode][2]);
		EepWrite(g_nMode,3,RomData[g_nMode][3]);
		EepWrite(g_nMode,4,RomData[g_nMode][4]);
		EepWrite(g_nMode,5,RomData[g_nMode][5]);
		EepWrite(g_nMode,6,RomData[g_nMode][6]);
		EepWrite(g_nMode,7,RomData[g_nMode][7]);
		EepWrite(g_nMode,8,RomData[g_nMode][8]);

		RomData[g_nMode][0] = EepRead(g_nMode, 0);
		RomData[g_nMode][1] = EepRead(g_nMode, 1);
		RomData[g_nMode][2] = EepRead(g_nMode, 2);
		RomData[g_nMode][3] = EepRead(g_nMode, 3);
		RomData[g_nMode][4] = EepRead(g_nMode, 4);
		RomData[g_nMode][5] = EepRead(g_nMode, 5);
		RomData[g_nMode][6] = EepRead(g_nMode, 6);
		RomData[g_nMode][7] = EepRead(g_nMode, 7);
		RomData[g_nMode][8] = EepRead(g_nMode, 8);

//		lcd_gotoxy(10,1);
//		lcd_puts("Write");			// display encoder conut

		break;
	case WriteData:
		EepWrite(g_nMode,g_nFunc,g_BufWrite[0]);

		RomData[g_nMode][g_nFunc] = EepRead(g_nMode, g_nFunc);

//		lcd_gotoxy(10,1);
//		lcd_puts("Write");			// display encoder conut

		break;

	case Reset:
		ResetPos();
		break;
	case Auto:
		SetAuto();
		break;
	case Close:
		SetClose();
		break;
	case Open:
		SetOpen();
		break;
	case Enc:
		SetEnc();
		break;
	case Conf:
		SetConf();
		break;

	case Query:
		if(g_bAuto)
			SciSend(":at;");
		else if(g_bClose)
			SciSend(":cl;");
		else if(g_bOpen)
			SciSend(":op;");

		sci_puts("\r\n");
		break;

	case Test:
		RepeatTest();
		break;

	}

}


void CommPc()
{
	int nGetData;
	if(g_cWatchDogComm > WatchDogComm)
		g_nStep=0;

	switch(g_nStep)
	{
	case 0:	
		InitCommPc();
		g_nStep++;
		break;
	case 1:	
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();	
			if(g_cSerialInput == ':') // Wait for ':'
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}
		}
		break;

	case 2:
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == 'A' || g_cSerialInput == 'a')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_AT; 						// in case 'AT'
			}
			else if(g_cSerialInput == 'C' || g_cSerialInput == 'c')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_C; 						// in case 'C'
			}
			else if(g_cSerialInput == 'O' || g_cSerialInput == 'o')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_OP; 						// in case 'OP'
			}
			else if(g_cSerialInput == 'R' || g_cSerialInput == 'r')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_R; 						// in case 'R'
			}
			else if(g_cSerialInput == 'W' || g_cSerialInput == 'w')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_W; 						// in case 'W'
			}
			else if(g_cSerialInput == 'Q' || g_cSerialInput == 'q')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_Q; 						// in case '?'
			}
			else if(g_cSerialInput == 'T' || g_cSerialInput == 't')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_T; 						// in case 'R'
			}

		}
		break;

	case GO_C:	//GO_C							// in case 'C'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;

			if(g_cSerialInput == 'L' || g_cSerialInput == 'l')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_CL;								// in case 'CL'
			}
			else if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_C+1://GO_C_1
		ExeCmd(IsConnected);
		g_nStep=0;
		break;


	case GO_R://GO_R
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{	
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;	
			if(g_cSerialInput == 'A' || g_cSerialInput == 'a')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_RA;								// in case 'RA'
			}
			else if(g_cSerialInput == 'S' || g_cSerialInput == 's')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_RST;								// in case 'RS'
			}
			else if(g_cSerialInput == ',')
			{
				g_cWatchDogComm = 0;
				g_nStep++;										// in case 'R'
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_R+1:	//GO_R_1												// in case 'R'
		nGetData = GetData();
		if(0 == nGetData)
		{
			//g_nMode = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_R+2:	//GO_R_2						
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_nFunc = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_R+3://GO_R_3
		ExeCmd(ReadData);
		g_nStep=0;
		break;

	case GO_RA:	//GO_RA							// in case 'RA'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == ',')
			{
				g_cWatchDogComm = 0;
				g_cDataIdx = 0;
				g_nData = CMD_NONE;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_RA+1:	//GO_RA_1									// in case 'RA'
		nGetData = GetData();
		if(0 == nGetData)
		{
			//g_nMode = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_RA+2:	//GO_RA_2						
		ExeCmd(ReadArrayData);
		g_nStep=0;
		break;



	case GO_W://GO_W
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{		
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;	
			if(g_cSerialInput == 'A' || g_cSerialInput == 'a')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_WA;								// in case 'WA'
			}
			else if(g_cSerialInput == ',')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_W+1:	//GO_W_1							
		nGetData = GetData();
		if(0 == nGetData)
		{
			//g_nMode = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_W+2:	//GO_W_2						
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_nFunc = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_W+3:	//GO_W_3						
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_BufWrite[0] = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_W+4://GO_W_4
		ExeCmd(WriteData);
		g_nStep=0;
		break;

	case GO_WA:	//GO_WA										// in case 'WA'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{	
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == ',')
			{
				g_cWatchDogComm = 0;
				g_cDataIdx = 0;
				g_nData = CMD_NONE;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_WA+1:// GO_WA_1
		nGetData = GetData();
		if(0 == nGetData)
		{
			//g_nMode = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_WA+2://GO_WA_2
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_BufWrite[0] = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_WA+3://GO_WA_3
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_BufWrite[1] = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_WA+4:// GO_WA_4
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_BufWrite[2] = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_WA+5://GO_WA_5
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_BufWrite[3] = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_WA+6://GO_WA_6
		nGetData = GetData();
		if(0 == nGetData)
		{
			g_BufWrite[4] = g_nData;

			g_cWatchDogComm = 0;
			g_cDataIdx = 0;
		 	g_nData = CMD_NONE;
			g_nStep++;
		}
		else if(CMD_ERROR == nGetData)
			g_nStep=0;

		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		break;

	case GO_WA+7://GO_WA_7
		ExeCmd(WriteArrayData);
		g_nStep=0;
		break;


	case GO_RST:	//GO_RST							// in case 'RS'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == 'T' || g_cSerialInput == 't')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_RST+1:		//GO_RST_1						// in case 'RST'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_RST+2://GO_RST_2
		ExeCmd(Reset);
		g_nStep=0;
		break;

	case GO_AT:	//GO_AT								// in case 'AT'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == 'T' || g_cSerialInput == 't')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_AT+1:	//GO_AT_1							
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;

			if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_AT+2://GO_AT_2
		ExeCmd(Auto);
		g_nStep=0;
		break;

	case GO_CL:		//GO_CL						// in case 'CL'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;

			if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_CL+1://GO_CL_1
		ExeCmd(Close);
		g_nStep=0;
		break;


	case GO_OP:	//GO_OP								// in case 'OP'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;
			if(g_cSerialInput == 'P' || g_cSerialInput == 'p')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_OP+1:	//GO_OP_1							
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;

			if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_OP+2: //GO_OP_2
		ExeCmd(Open);
		g_nStep=0;
		break;

	case GO_Q:	//GO_Q							// in case 'C'
		g_cWatchDogComm++;
			if(g_cWatchDogComm > WatchDogComm)
				g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			//g_cWatchDogComm++;

			if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}

			//if(g_cWatchDogComm > WatchDogComm)
			//	g_nStep=0;
		}
		break;

	case GO_Q+1: //GO_Q_1
		ExeCmd(Query);
		g_nStep=0;
		break;

	case GO_T://GO_T
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{	
			g_cSerialInput = (char)sci_getc();
			if(g_cSerialInput == 'E' || g_cSerialInput == 'e')
			{
				g_cWatchDogComm = 0;
				g_nStep = GO_TEST;								// in case 'TE'
			}
		}
		break;

	case GO_TEST:	//GO_TEST							// in case 'TE'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			if(g_cSerialInput == 'S' || g_cSerialInput == 's')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}
		}
		break;

	case GO_TEST+1:		//GO_TEST_1						// in case 'TES'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			if(g_cSerialInput == 'T' || g_cSerialInput == 't')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}
		}
		break;

	case GO_TEST+2:		//GO_TEST_2						// in case 'TEST'
		g_cWatchDogComm++;
		if(g_cWatchDogComm > WatchDogComm)
			g_nStep=0;
		while(sci_rx_ready())
		{
			g_cSerialInput = (char)sci_getc();
			if(g_cSerialInput == ';')
			{
				g_cWatchDogComm = 0;
				g_nStep++;
			}
		}
		break;

	case GO_TEST+3://GO_TEST_3
		ExeCmd(Test);
		g_nStep=0;
		break;

	}

}


void ResetPos()
{
	DINT;
	EvaRegs.T2CNT = 0x0000;		// Timer2 counter
	EINT;

	InitVal();
/*	InitCommPc();

	InitDsp();

	// Start My code...
	
	delay_us(10);	
	ReloadEep();

	SciSend("\r\nScan Axis Encoder Monitor\r\n");
	sci_puts("\r\n");

	SciSend(":at;");
	sci_puts("\r\n");

	InitSegments();
*/
	SetAuto();
	//DispAuto();

	DispData();
	//DispEncVal();

}


void SetAuto()
{
	g_bAuto = 1;
	g_bClose = 0;
	g_bOpen = 0;
	g_bEnc = 0;
	g_bConf = 0;
	g_bRepeatTest = 0;

	g_nAutoWinker = 0;
}

void SetClose()
{
	g_bAuto = 0;
	g_bClose = 1;
	g_bOpen = 0;
	g_bEnc = 0;
	g_bConf = 0;

	g_nCloseWinker = 0;
}

void SetOpen()
{
	g_bAuto = 0;
	g_bClose = 0;
	g_bOpen = 1;
	g_bEnc = 0;
	g_bConf = 0;

	g_nOpenWinker = 0;
}

void RepeatTest()
{
	g_bAuto = 0;
	g_bRepeatTest = 1;
}

void SetEnc()
{
	g_bAuto = 1;
	g_bClose = 0;
	g_bOpen = 0;
	g_bEnc = 1;
	g_bConf = 0;

	g_nEncWinker = 0;
}

void SetConf()
{
	g_bAuto = 1;
	g_bClose = 0;
	g_bOpen = 0;
	g_bEnc = 0;
	g_bConf = 1;

	g_nConfWinker = 0;
}

void DoAuto()
{
	SciSend(":at;");
	sci_puts("\r\n");
}

void DoClose()
{
	SciSend(":cl;");
	sci_puts("\r\n");
}

void DoOpen()
{
	SciSend(":op;");
	sci_puts("\r\n");
}

void DispAuto()
{
	unsigned char value;

	if(g_bAuto && !g_bAutoF)
	{
		g_bAutoF = 1;
		g_bAutoWinker = 1;
		//g_nAutoWinker = 0;

		DoAuto();
	}
	else if(!g_bAuto && g_bAutoF)
	{
		g_bAutoF = 0;
		g_bAutoWinker = 0;
	}

	if(g_bAutoWinker && g_nAutoWinker < (WinkNum*2))
	{
		if(!(g_nAutoWinker%2)) 	// Display On
		{
			if(DispSegment != D_AUTO)
			{
				DispSegment = D_AUTO;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'A';
				setDigit(3, value, _DONT_CARE);
				value = 'U';
				setDigit(2, value, _DONT_CARE);
				value = 'T';
				setDigit(1, value, _DONT_CARE);
				value = 'O';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else 					// Display Off
		{
			if(DispSegment != D_BLANK)
			{
				DispSegment = D_BLANK;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = _OFF;
				setDigit(3, value, _DONT_CARE);
				value = _OFF;
				setDigit(2, value, _DONT_CARE);
				value = _OFF;
				setDigit(1, value, _DONT_CARE);
				value = _OFF;
				setDigit(0, value, _DONT_CARE);
			}
		}

		g_nAutoWinker++;
		if(g_nAutoWinker == (WinkNum*2))
		{
			g_eCntPrev3 = g_eCnt+1; //DispEncVal();
			g_nAutoWinker++;
			g_bKeyDownSt = 0;
		}
	}
}

void DispClose()
{
	unsigned char value;

	if(g_bClose && !g_bCloseF)
	{
		g_bCloseF = 1;
		g_bCloseWinker = 1;
		//g_nCloseWinker = 0;

		DoClose();
	}
	else if(!g_bClose && g_bCloseF)
	{
		g_bCloseF = 0;
		g_bCloseWinker = 0;
	}

	if(g_bCloseWinker && g_nCloseWinker <= (WinkNum*2))
	{
		if(!(g_nCloseWinker%2)) 	// Display On
		{
			if(DispSegment != D_CLOSE)
			{
				DispSegment = D_CLOSE;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = 'C';
				setDigit(4, value, _DONT_CARE);
				value = 'L';
				setDigit(3, value, _DONT_CARE);
				value = 'O';
				setDigit(2, value, _DONT_CARE);
				value = 'S';
				setDigit(1, value, _DONT_CARE);
				value = 'E';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else 					// Display Off
		{
			if(DispSegment != D_BLANK)
			{
				DispSegment = D_BLANK;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = _OFF;
				setDigit(3, value, _DONT_CARE);
				value = _OFF;
				setDigit(2, value, _DONT_CARE);
				value = _OFF;
				setDigit(1, value, _DONT_CARE);
				value = _OFF;
				setDigit(0, value, _DONT_CARE);
			}
		}

		g_nCloseWinker++;
	}
}

void DispOpen()
{
	unsigned char value;

	if(g_bOpen && !g_bOpenF)
	{
		g_bOpenF = 1;
		g_bOpenWinker = 1;

		DoOpen();
	}
	else if(!g_bOpen && g_bOpenF)
	{
		g_bOpenF = 0;
		g_bOpenWinker = 0;
	}

	if(g_bOpenWinker && g_nOpenWinker <= (WinkNum*2))
	{
		if(!(g_nOpenWinker%2)) 	// Display On
		{
			if(DispSegment != D_OPEN)
			{
				DispSegment = D_OPEN;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'O';
				setDigit(3, value, _DONT_CARE);
				value = 'P';
				setDigit(2, value, _DONT_CARE);
				value = 'E';
				setDigit(1, value, _DONT_CARE);
				value = 'N';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else 					// Display Off
		{
			if(DispSegment != D_BLANK)
			{
				DispSegment = D_BLANK;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = _OFF;
				setDigit(3, value, _DONT_CARE);
				value = _OFF;
				setDigit(2, value, _DONT_CARE);
				value = _OFF;
				setDigit(1, value, _DONT_CARE);
				value = _OFF;
				setDigit(0, value, _DONT_CARE);
			}
		}

		g_nOpenWinker++;
	}
}

void DispEnc()
{
	unsigned char value;

	if(g_bEnc && !g_bEncF)
	{
		g_bEncF = 1;
		g_bEncWinker = 1;
	}
	else if(!g_bEnc && g_bEncF)
	{
		g_bEncF = 0;
		g_bEncWinker = 0;
	}

	if(g_bEncWinker && g_nEncWinker <= (WinkNum*2))
	{
		if(!(g_nEncWinker%2)) 	// Display On
		{
			if(DispSegment != D_ENCODER)
			{
				DispSegment = D_ENCODER;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = 'E';
				setDigit(6, value, _DONT_CARE);
				value = 'N';
				setDigit(5, value, _DONT_CARE);
				value = 'C';
				setDigit(4, value, _DONT_CARE);
				value = 'O';
				setDigit(3, value, _DONT_CARE);
				value = 'D';
				setDigit(2, value, _DONT_CARE);
				value = 'E';
				setDigit(1, value, _DONT_CARE);
				value = 'R';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else 					// Display Off
		{
			if(DispSegment != D_BLANK)
			{
				DispSegment = D_BLANK;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = _OFF;
				setDigit(3, value, _DONT_CARE);
				value = _OFF;
				setDigit(2, value, _DONT_CARE);
				value = _OFF;
				setDigit(1, value, _DONT_CARE);
				value = _OFF;
				setDigit(0, value, _DONT_CARE);
			}
		}

		g_nEncWinker++;
	}
}

void DispConf()
{
	unsigned char value;

	if(g_bConf && !g_bConfF)
	{
		g_bConfF = 1;
		g_bConfWinker = 1;
	}
	else if(!g_bConf && g_bConfF)
	{
		g_bConfF = 0;
		g_bConfWinker = 0;
	}

	if(g_bConfWinker && g_nConfWinker <= (WinkNum*2))
	{
		if(!(g_nConfWinker%2)) 	// Display On
		{
			if(DispSegment != D_CONF)
			{
				DispSegment = D_CONF;

				DispSegment = D_CONF;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = 'C';
				setDigit(3, value, _DONT_CARE);
				value = 'O';
				setDigit(2, value, _DONT_CARE);
				value = 'N';
				setDigit(1, value, _DONT_CARE);
				value = 'F';
				setDigit(0, value, _DONT_CARE);
			}
		}
		else 					// Display Off
		{
			if(DispSegment != D_BLANK)
			{
				DispSegment = D_BLANK;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = _OFF;
				setDigit(3, value, _DONT_CARE);
				value = _OFF;
				setDigit(2, value, _DONT_CARE);
				value = _OFF;
				setDigit(1, value, _DONT_CARE);
				value = _OFF;
				setDigit(0, value, _DONT_CARE);
			}
		}

		g_nConfWinker++;
	}
	else
	{
		if(g_nConfWinker == (WinkNum*2)+1)
		{
			if(DispSegment != D_BLANK)
			{
				DispSegment = D_BLANK;

				value = _OFF;
				setDigit(7, value, _DONT_CARE);
				value = _OFF;
				setDigit(6, value, _DONT_CARE);
				value = _OFF;
				setDigit(5, value, _DONT_CARE);
				value = _OFF;
				setDigit(4, value, _DONT_CARE);
				value = _OFF;
				setDigit(3, value, _DONT_CARE);
				value = _OFF;
				setDigit(2, value, _DONT_CARE);
				value = _OFF;
				setDigit(1, value, _DONT_CARE);
				value = _OFF;
				setDigit(0, value, _DONT_CARE);
			}

			KeyDisp &= ~0x01E0;

			DispFuncVal(0);

			g_nConfWinker++;
		}
	}
}



void InitDsp()
{
	// Initialize System Control registers, PLL, WatchDog, Clocks to default state:
	// This function is found in the DSP28_SysCtrl.c file.

	InitSysCtrl(0x0A); // 150MHz

	
/************************ for Flash ********************************/
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
/*******************************************************************/
	
	
	init_gpio();	

	// Disable and clear all CPU interrupts:
	DINT;
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize Pie Control Registers To Default State:
	InitPieCtrl();

	// Initialize the PIE Vector Table To a Known State:
	// This function populates the PIE vector table with pointers
	// to the shell ISR functions found in DSP28_DefaultIsr.c.
	InitPieVectTable();	
	
	EALLOW;	// This is needed to write to EALLOW protected registers

	PieVectTable.TINT0 = &cpu_timer0_isr;	// for TINT0
	
	EDIS;       // This is needed to disable write to EALLOW protected registers

	InitCpuTimers();
	//ConfigCpuTimer(&CpuTimer0, 150, 1000);	// 150MHz CPU Freq, 1 msec Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 150, TimerPeriod);	// 150MHz CPU Freq, TimerPeriod usec Period (in uSeconds)
	StartCpuTimer0();


	// Interrupt Enable Register
	IER |= M_INT1;	// for TINT0
	
	//	PIE : Peripheral Interrupts setting
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		// for TINT0


	InitIo();
	InitKey();
//	init_lcd();
	InitDac();
	init_eep();	// FOR 2811
//	Init_eep();	// FOR 2812

	init_Max7219();

//	sci_debug_init();	// for SCI debug
//	init_sci(9600);
	init_sci(38400);
//	init_sci(115200);

	init_ev();

	sci_debug_init();

	// Enable global Interrupts and higher priority real-time debug events:
	EINT;	// Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

}

void DispIO()
{
	unsigned char value;

	if(g_bIoEnable && !g_bDispIoEnable)
	{
		g_bDispIoEnable = 1;
		value = getDigit(7);
		setDigit(7, value, 1); //Max7219_Status[7]
	}
	else if(!g_bIoEnable && g_bDispIoEnable)
	{
		g_bDispIoEnable = 0;
		value = getDigit(7);
		setDigit(7, value, 0); //Max7219_Status[7]
	}

	if(g_bAuto && !g_bDispAuto)
	{
		g_bDispAuto = 1;
		value = getDigit(6);
		setDigit(6, value, 1); //Max7219_Status[6]
	}
	else if(!g_bAuto && g_bDispAuto)
	{
		g_bDispAuto = 0;
		value = getDigit(6);
		setDigit(6, value, 0); //Max7219_Status[6]
	}
/*
	if(g_bClose)
	{
		value = getDigit(5);
		setDigit(5, value, 1); //Max7219_Status[0]
	}
	else
	{
		value = getDigit(5);
		setDigit(5, value, 0); //Max7219_Status[0]
	}

	if(g_bOpen)
	{
		value = getDigit(4);
		setDigit(4, value, 1); //Max7219_Status[0]
	}
	else
	{
		value = getDigit(4);
		setDigit(4, value, 0); //Max7219_Status[0]
	}
	*/
}


void main(void)
{
	unsigned int nDelay, nToggle, nToggle2, nToggle3;
	unsigned char value;

	InitVal();
	InitCommPc();
	
	InitDsp();

	// Start My code...

	delay_us(10);	
	ReloadEep();

	SciSend("\r\nScan Axis Encoder Monitor\r\n");
	sci_puts("\r\n");

	SciSend(":at;");
	sci_puts("\r\n");

	InitSegments();

	SetAuto();

	EnableDog();

	nDelay = (992 * DelayDisp); // [1mSec] * 30

	g_bInitDone = 1;

	while(1)
	{
		if(!(nDelay % DelayDisp))
		{
			KickDog();
		}

		if(nDelay-- <= 0)
		{
			nDelay = (992 * DelayDisp); // [1mSec] * 30 for 2812

			if(WinkDelay < ++g_nWinkDelay)
			{
				g_nWinkDelay = 0;
				DispAuto();
			}

			if(g_nAutoWinker >= (WinkNum*2))
				break;
		}
	}


	nToggle = 0;
	nToggle2 = 0;
	nToggle3 = 0;
	nDelay = 992 * DelayDisp; // [1mSec] * 30

	while(1)
	{
		if(!(nDelay % DelayDisp))
		{
			KickDog();
		}

		ProcessIo();

		if((key_code & KEY_DOWN) || (key_code & KEY_UP))
			ProcessKey();

		if(!g_bKeyDownSt)
		{
			if(g_eCntPrev3 != g_eCnt)
			{
				g_eCntPrev3 = g_eCnt;

				if(g_bEnc && (g_nEncWinker > (WinkNum*2)))
					DispEncVal();
			}

			if(!g_bConf && !g_bEnc && g_bAuto && (g_nAutoWinker > (WinkNum*2)))
				DispAutoVal();
		}

		DispIO();

		CommPc();

		if(nDelay-- <= 0)
		{
			nDelay = 992 * DelayDisp; // [1mSec] * 30 for 2811

			//LED_TOGGLE;	// watchdog of main() 

			nToggle++;
			if(nToggle%2)
			{
				value = getDigit(0);
				setDigit(0, value, 1); //Max7219_Status[0]
				
				if(g_bRepeatTest && !g_bAuto)
				{
					nToggle2++;
					if(!(nToggle2%20))
					{					
						nToggle3++;
						if(nToggle3%2)
							SetOpen();
						else
							SetClose();					
					}
				}
			}
			else
			{
				value = getDigit(0);
				setDigit(0, value, 0); //Max7219_Status[0]
			}
				
			DispData();

			if(WinkDelay < ++g_nWinkDelay)
			{
				g_nWinkDelay = 0;
				DispAuto();
				DispClose();
				DispOpen();
				DispEnc();
				DispConf();
			}

		}
	}
}
