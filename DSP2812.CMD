/* dsp2812.cmd */

MEMORY
{
PAGE 0 :
   PRAMH0      : origin = 0x3F8000, length = 0x006000
   RESET       : origin = 0x3FFFC0, length = 0x000002   
   BOOTROM     : origin = 0x3FF000, length = 0x000fc0
      FLASH	   : origin = 0x3D8000, length = 0x01FF80
       
PAGE 1 :   
   RAMM0       : origin = 0x000000, length = 0x000400
   RAMM1       : origin = 0x000400, length = 0x000400 
   DRAMH0      : origin = 0x008000, length = 0x002000    
   DEV_EMU     : origin = 0x000880, length = 0x000180
   PIE_VECT    : origin = 0x000D00, length = 0x000100
   FLASH_REGS  : origin = 0x000A80, length = 0x000060
   CSM         : origin = 0x000AE0, length = 0x000010
   XINTF       : origin = 0x000B20, length = 0x000020
   CPU_TIMER0  : origin = 0x000C00, length = 0x000008
   PIE_CTRL    : origin = 0x000CE0, length = 0x000020
   ECANA       : origin = 0x006000, length = 0x000040
   ECANA_LAM   : origin = 0x006040, length = 0x000040
   ECANA_MOTS  : origin = 0x006080, length = 0x000040
   ECANA_MOTO  : origin = 0x0060C0, length = 0x000040
   ECANA_MBOX  : origin = 0x006100, length = 0x000100
   SYSTEM      : origin = 0x007010, length = 0x000020
   SPIA        : origin = 0x007040, length = 0x000010
   SCIA        : origin = 0x007050, length = 0x000010 
   XINTRUPT    : origin = 0x007070, length = 0x000010 
   GPIOMUX     : origin = 0x0070C0, length = 0x000020 
   GPIODAT     : origin = 0x0070E0, length = 0x000020 
   ADC         : origin = 0x007100, length = 0x000020  
   EVA         : origin = 0x007400, length = 0x000040
   EVB         : origin = 0x007500, length = 0x000040   
   SCIB        : origin = 0x007750, length = 0x000010   
   MCBSPA      : origin = 0x007800, length = 0x000040 
   CSM_PWL     : origin = 0x3F7FF8, length = 0x000008
}


    
SECTIONS
{
   codestart        : > PRAMH0,       PAGE = 0
   .text            : > PRAMH0,       PAGE = 0
   .cinit           : > PRAMH0,       PAGE = 0
   ramfuncs         : > PRAMH0,       PAGE = 0
   .reset           : > RESET,        PAGE = 0, TYPE = DSECT   
   IQmath           : > PRAMH0        PAGE = 0
   IQmathTables     : > BOOTROM       PAGE = 0, TYPE = NOLOAD
   .stack           : > RAMM1,        PAGE = 1

   .bss             : > DRAMH0,       PAGE = 1
   .ebss            : > DRAMH0,       PAGE = 1
   .const           : > DRAMH0,       PAGE = 1
   .econst          : > DRAMH0,       PAGE = 1      
   .sysmem          : > DRAMH0,       PAGE = 1

   PieVectTableFile  : > PIE_VECT,    PAGE = 1
/*** Peripheral Frame 0 Register Structures ***/
   DevEmuRegsFile    : > DEV_EMU,     PAGE = 1
   FlashRegsFile     : > FLASH_REGS,  PAGE = 1
   CsmRegsFile       : > CSM,         PAGE = 1
   XintfRegsFile     : > XINTF,       PAGE = 1
   CpuTimer0RegsFile : > CPU_TIMER0,  PAGE = 1  
   PieCtrlRegsFile   : > PIE_CTRL,    PAGE = 1      
/*** Peripheral Frame 1 Register Structures ***/
   SysCtrlRegsFile   : > SYSTEM,      PAGE = 1
   SpiaRegsFile      : > SPIA,        PAGE = 1
   SciaRegsFile      : > SCIA,        PAGE = 1
   XIntruptRegsFile  : > XINTRUPT,    PAGE = 1
   GpioMuxRegsFile   : > GPIOMUX,     PAGE = 1
   GpioDataRegsFile  : > GPIODAT      PAGE = 1
   AdcRegsFile       : > ADC,         PAGE = 1
   EvaRegsFile       : > EVA,         PAGE = 1
   EvbRegsFile       : > EVB,         PAGE = 1
   ScibRegsFile      : > SCIB,        PAGE = 1
   McbspaRegsFile    : > MCBSPA,      PAGE = 1
/*** Peripheral Frame 2 Register Structures ***/
   ECanaRegsFile     : > ECANA,       PAGE = 1
   ECanaLAMRegsFile  : > ECANA_LAM    PAGE = 1   
   ECanaMboxesFile   : > ECANA_MBOX   PAGE = 1
   ECanaMOTSRegsFile : > ECANA_MOTS   PAGE = 1
   ECanaMOTORegsFile : > ECANA_MOTO   PAGE = 1
/*** Code Security Module Register Structures ***/
   CsmPwlFile        : > CSM_PWL,     PAGE = 1
}



/* [EOF] dsp2811_ad_pwm.cmd */
