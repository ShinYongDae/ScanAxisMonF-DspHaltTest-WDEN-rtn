
// dsp281x_init.h   
// By RealSYS   
   

#ifndef DSP281x_INIT_H
#define DSP281x_INIT_H


#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------   
//---------------------------------------------------------------------------   
// Example: KickDog:    
//---------------------------------------------------------------------------   
// This function resets the watchdog timer.   
// Enable this function for using KickDog in the application    
   
void KickDog(void);   
   
//---------------------------------------------------------------------------   
// Example: DisableDog:    
//---------------------------------------------------------------------------   
// This function disables the watchdog timer.   
   
void DisableDog(void);  
   
   
//---------------------------------------------------------------------------   
// Example: InitPll:    
//---------------------------------------------------------------------------   
// This function initializes the PLLCR register.   
   
void InitPll(Uint16 val);  
   
//--------------------------------------------------------------------------   
// Example: InitPeripheralClocks:    
//---------------------------------------------------------------------------   
// This function initializes the clocks to the peripheral modules.   
// First the high and low clock prescalers are set   
// Second the clocks are enabled to each peripheral.   
// To reduce power, leave clocks to unused peripherals disabled   
// Note: If a peripherals clock is not enabled then you cannot    
// read or write to the registers for that peripheral    
   
void InitPeripheralClocks(void);   
   
void InitSysCtrl(void);   
   
   
// InitPieCtrl:    
// This function initializes the PIE control registers to a known state.   
void InitPieCtrl(void);   
   
//---------------------------------------------------------------------------   
// EnableInterrupts:    
//---------------------------------------------------------------------------   
// This function enables the PIE module and CPU interrupts   
//   
void EnableInterrupts();   
/*   
const struct PIE_VECT_TABLE PieVectTableInit = {   
   
      PIE_RESERVED,  // Reserved space   
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
      PIE_RESERVED,      
   
   
// Non-Peripheral Interrupts   
      INT13_ISR,     // XINT13 or CPU-Timer 1   
      INT14_ISR,     // CPU-Timer2   
      DATALOG_ISR,   // Datalogging interrupt   
      RTOSINT_ISR,   // RTOS interrupt   
      EMUINT_ISR,    // Emulation interrupt   
      NMI_ISR,       // Non-maskable interrupt   
      ILLEGAL_ISR,   // Illegal operation TRAP   
      USER1_ISR,     // User Defined trap 1   
      USER2_ISR,     // User Defined trap 2   
      USER3_ISR,     // User Defined trap 3   
      USER4_ISR,     // User Defined trap 4   
      USER5_ISR,     // User Defined trap 5   
      USER6_ISR,     // User Defined trap 6   
      USER7_ISR,     // User Defined trap 7   
      USER8_ISR,     // User Defined trap 8   
      USER9_ISR,     // User Defined trap 9   
      USER10_ISR,    // User Defined trap 10   
      USER11_ISR,    // User Defined trap 11   
      USER12_ISR,     // User Defined trap 12   
   
// Group 1 PIE Vectors   
      PDPINTA_ISR,   // EV-A   
      PDPINTB_ISR,   // EV-B   
      rsvd_ISR,   
      XINT1_ISR,        
      XINT2_ISR,   
      ADCINT_ISR,    // ADC   
      TINT0_ISR,     // Timer 0   
      WAKEINT_ISR,   // WD   
   
// Group 2 PIE Vectors   
      CMP1INT_ISR,   // EV-A   
      CMP2INT_ISR,   // EV-A   
      CMP3INT_ISR,   // EV-A   
      T1PINT_ISR,    // EV-A   
      T1CINT_ISR,    // EV-A   
      T1UFINT_ISR,   // EV-A   
      T1OFINT_ISR,   // EV-A   
      rsvd_ISR,   
         
// Group 3 PIE Vectors   
      T2PINT_ISR,    // EV-A   
      T2CINT_ISR,    // EV-A   
      T2UFINT_ISR,   // EV-A   
      T2OFINT_ISR,   // EV-A   
      CAPINT1_ISR,   // EV-A   
      CAPINT2_ISR,   // EV-A   
      CAPINT3_ISR,   // EV-A   
      rsvd_ISR,   
         
// Group 4 PIE Vectors   
      CMP4INT_ISR,   // EV-B   
      CMP5INT_ISR,   // EV-B   
      CMP6INT_ISR,   // EV-B   
      T3PINT_ISR,    // EV-B   
      T3CINT_ISR,    // EV-B   
      T3UFINT_ISR,   // EV-B   
      T3OFINT_ISR,   // EV-B   
      rsvd_ISR,         
        
// Group 5 PIE Vectors   
      T4PINT_ISR,    // EV-B   
      T4CINT_ISR,    // EV-B   
      T4UFINT_ISR,   // EV-B   
      T4OFINT_ISR,   // EV-B   
      CAPINT4_ISR,   // EV-B   
      CAPINT5_ISR,   // EV-B   
      CAPINT6_ISR,   // EV-B   
      rsvd_ISR,         
   
// Group 6 PIE Vectors   
      SPIRXINTA_ISR,   // SPI-A   
      SPITXINTA_ISR,   // SPI-A   
      rsvd_ISR,   
      rsvd_ISR,   
      MRINTA_ISR,    // McBSP-A   
      MXINTA_ISR,    // McBSP-A   
      rsvd_ISR,   
      rsvd_ISR,   
         
// Group 7 PIE Vectors   
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
   
// Group 8 PIE Vectors   
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
         
// Group 9 PIE Vectors        
      SCIRXINTA_ISR, // SCI-A   
      SCITXINTA_ISR, // SCI-A   
      SCIRXINTB_ISR, // SCI-B   
      SCITXINTB_ISR, // SCI-B   
      ECAN0INTA_ISR, // eCAN   
      ECAN1INTA_ISR, // eCAN   
      rsvd_ISR,      
      rsvd_ISR,      
         
// Group 10 PIE Vectors   
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
               
// Group 11 PIE Vectors   
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
   
// Group 12 PIE Vectors   
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
      rsvd_ISR,      
};   
 */  
   
//---------------------------------------------------------------------------   
// InitPieVectTable:    
//---------------------------------------------------------------------------   
// This function initializes the PIE vector table to a known state.   
// This function must be executed after boot time.   
//   
   
void InitPieVectTable(void);   
   
// struct CPUTIMER_VARS CpuTimer0;   
   
// CpuTimer 1 and CpuTimer2 are reserved for DSP BIOS & other RTOS   
//struct CPUTIMER_VARS CpuTimer1;   
//struct CPUTIMER_VARS CpuTimer2;   
   
//---------------------------------------------------------------------------   
// InitCpuTimers:    
//---------------------------------------------------------------------------   
// This function initializes all three CPU timers to a known state.   
//   
void InitCpuTimers(void);   
      
///---------------------------------------------------------------------------   
// ConfigCpuTimer:    
//---------------------------------------------------------------------------   
// This function initializes the selected timer to the period specified   
// by the "Freq" and "Period" parameters. The "Freq" is entered as "MHz"   
// and the period in "uSeconds". The timer is held in the stopped state   
// after configuration.   
//   
void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period);   
//---------------------------------------------------------------------------   
// InitXINTF:    
//---------------------------------------------------------------------------   
// This function initializes the External Interface the default reset state.   
//   
// Do not modify the timings of the XINTF while running from the XINTF.  Doing   
// so can yield unpredictable results   
   
   
void InitXintf(void);   

 
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of DSP281x_INIT_H definition

//===========================================================================
// No more.
//===========================================================================



 
 