/* dsp2812f.cmd */
/* RealSYS 2004/11/29 */

MEMORY
{
PAGE 0:    /* Program Memory */
   RAML0       : origin = 0x008000, length = 0x001000     /* on-chip RAM block L0 */
   PRAMH0      : origin = 0x3F8000, length = 0x002000	  /* Program RAM Area */
   ZONE0       : origin = 0x002000, length = 0x002000     /* XINTF zone 0 */
   ZONE1       : origin = 0x004000, length = 0x002000     /* XINTF zone 1 */
   ZONE2       : origin = 0x080000, length = 0x080000     /* XINTF zone 2 */
   ZONE6       : origin = 0x100000, length = 0x080000     /* XINTF zone 6 */
   OTP         : origin = 0x3D7800, length = 0x000800     /* on-chip OTP */
/*   FLASH	   : origin = 0x3D8000, length = 0x01FF80 */
   FLASH	   : origin = 0x3D8000, length = 0x01FB80
/*   FLASHJ      : origin = 0x3D8000, length = 0x002000 */    /* on-chip FLASH */
/*   FLASHI      : origin = 0x3DA000, length = 0x002000 */    /* on-chip FLASH */
/*   FLASHH      : origin = 0x3DC000, length = 0x004000 */    /* on-chip FLASH */
/*   FLASHG      : origin = 0x3E0000, length = 0x004000 */    /* on-chip FLASH */
/*   FLASHF      : origin = 0x3E4000, length = 0x004000 */    /* on-chip FLASH */
/*   FLASHE      : origin = 0x3E8000, length = 0x004000 */    /* on-chip FLASH */
/*   FLASHD      : origin = 0x3EC000, length = 0x004000 */    /* on-chip FLASH */
/*   FLASHC      : origin = 0x3F0000, length = 0x004000 */    /* on-chip FLASH */
/*   FLASHB      : origin = 0x3F4000, length = 0x002000 */    /* on-chip FLASH */
/*   FLASHA      : origin = 0x3F6000, length = 0x001F80 */    /* on-chip FLASH */
   FLASH_DATA  : origin = 0x3F7B80, length = 0x000400

   CSM_RSVD	   : origin = 0x3F7F80, length = 0x000076     /* CSM Reserved locations in FLASHA */
   CSM_PWL     : origin = 0x3F7FF8, length = 0x000008     /* CSM password locations in FLASHA */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
/* ZONE7       : origin = 0x3FC000, length = 0x003FC0     /* XINTF zone 7 available if MP/MCn=1 */ 
   BOOTROM     : origin = 0x3FF000, length = 0x000FC0     /* Boot ROM available if MP/MCn=0 */
   RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM (MP/MCn=0) or XINTF zone 7 (MP/MCn=1) */
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM (MP/MCn=0) or XINTF zone 7 (MP/MCn=1) */
PAGE 1 :   /* Data Memory */
           /* Registers remain on PAGE1 */
   DRAMM       : origin = 0x000000, length = 0x000800     /* on-chip RAM block M0+M1 */
   RAML1       : origin = 0x009000, length = 0x001000     /* on-chip RAM block L1 */
   DEV_EMU     : origin = 0x000880, length = 0x000180     /* Device emulation registers */
   FLASH_REGS  : origin = 0x000A80, length = 0x000060     /* FLASH registers */
   CSM         : origin = 0x000AE0, length = 0x000010     /* Code security module registers */
   XINTF       : origin = 0x000B20, length = 0x000020     /* External interface registers */
   CPU_TIMER0  : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 and Timer2 are reserved for BIOS)*/
   PIE_CTRL    : origin = 0x000CE0, length = 0x000020     /* PIE control registers */
   PIE_VECT    : origin = 0x000D00, length = 0x000100     /* PIE vector table */
   ECANA       : origin = 0x006000, length = 0x000040     /* eCAN registers */
   ECANA_LAM   : origin = 0x006040, length = 0x000040     /* eCAN mailbox LAM */
   ECANA_MOTS  : origin = 0x006080, length = 0x000040     /* eCAN MOTS  */
   ECANA_MOTO  : origin = 0x0060C0, length = 0x000040     /* eCAN MOTO  */
   ECANA_MBOX  : origin = 0x006100, length = 0x000100     /* eCAN mailboxes */
   SYSTEM      : origin = 0x007010, length = 0x000020     /* System control registers */
   SPIA        : origin = 0x007040, length = 0x000010     /* SPI registers */
   SCIA        : origin = 0x007050, length = 0x000010     /* SCI-A registers */
   XINTRUPT    : origin = 0x007070, length = 0x000010     /* External interrupt registers */
   GPIOMUX     : origin = 0x0070C0, length = 0x000020     /* GPIO mux registers */
   GPIODAT     : origin = 0x0070E0, length = 0x000020     /* GPIO data registers */
   ADC         : origin = 0x007100, length = 0x000020     /* ADC registers */
   EVA         : origin = 0x007400, length = 0x000040     /* Event Manager A registers */
   EVB         : origin = 0x007500, length = 0x000040     /* Event Manager B registers */
   SCIB        : origin = 0x007750, length = 0x000010     /* SCI-B registers */
   MCBSPA      : origin = 0x007800, length = 0x000040     /* McBSP registers */
}

/* Allocate sections to memory blocks.
   Note:
         codestart user defined section in DSP28_CodeStartBranch.asm used to redirect code 
                   execution when booting to flash */ 
SECTIONS
{
   codestart           : > BEGIN,	PAGE = 0
   .text               : > FLASH,	PAGE = 0
   .cinit              : > FLASH,	PAGE = 0
   .pinit              : > FLASH,	PAGE = 0
   .switch             : > FLASH,	PAGE = 0
   .econst             : > FLASH,	PAGE = 0      
   ramfuncs            : LOAD = FLASH, 
                         RUN = RAML0, 
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         PAGE = 0

   Flash_Data		   : > FLASH_DATA,	  PAGE = 0

   IQmath              : > FLASH	 PAGE = 0
   IQmathTables        : > BOOTROM	 PAGE = 0, TYPE = NOLOAD
   .stack              : > DRAMM,	 PAGE = 1
   .ebss               : > RAML1,	 PAGE = 1
   .esysmem            : > RAML1,	 PAGE = 1
   csm_rsvd	           : > CSM_RSVD, PAGE = 0
   csmpasswds          : > CSM_PWL,  PAGE = 0 
   /* .reset indicates the start of _c_int00 for C Code.  
   /* When using the boot ROM this section and the CPU vector
   /* table is not needed.  Thus the default type is set to 
   /* DSECT */ 
   .reset              : > RESET,	PAGE = 0, TYPE = DSECT
   vectors             : > VECTORS,	PAGE = 0, TYPE = DSECT
/* ------------------------------------------------------------- */
/* The following allocations are required for the DSP28 Header file
   examples.  Each allocation maps a structure defined in the DSP28 
   header files to the memory location of those registers.  
*/
   PieVectTableFile    : > PIE_VECT,   PAGE = 1
   /* Allocate Peripheral Frame 0 Register Structures:   */
   DevEmuRegsFile      : > DEV_EMU     PAGE = 1
   FlashRegsFile       : > FLASH_REGS  PAGE = 1
   CsmRegsFile         : > CSM         PAGE = 1
   XintfRegsFile       : > XINTF       PAGE = 1
   CpuTimer0RegsFile   : > CPU_TIMER0  PAGE = 1      
   PieCtrlRegsFile     : > PIE_CTRL    PAGE = 1      
   PieVectTable        : > PIE_VECT    PAGE = 1
   /* Allocate Peripheral Frame 2 Register Structures:   */
   ECanaRegsFile       : > ECANA       PAGE = 1   
   ECanaLAMRegsFile    : > ECANA_LAM   PAGE = 1   
   ECanaMboxesFile     : > ECANA_MBOX  PAGE = 1
   ECanaMOTSRegsFile   : > ECANA_MOTS  PAGE = 1
   ECanaMOTORegsFile   : > ECANA_MOTO  PAGE = 1
   /* Allocate Peripheral Frame 1 Register Structures:   */
   SysCtrlRegsFile     : > SYSTEM      PAGE = 1
   SpiaRegsFile        : > SPIA        PAGE = 1
   SciaRegsFile        : > SCIA        PAGE = 1
   XIntruptRegsFile    : > XINTRUPT    PAGE = 1
   GpioMuxRegsFile     : > GPIOMUX     PAGE = 1
   GpioDataRegsFile    : > GPIODAT     PAGE = 1
   AdcRegsFile         : > ADC         PAGE = 1
   EvaRegsFile         : > EVA         PAGE = 1
   EvbRegsFile         : > EVB         PAGE = 1
   ScibRegsFile        : > SCIB        PAGE = 1
   McbspaRegsFile      : > MCBSPA      PAGE = 1
}
