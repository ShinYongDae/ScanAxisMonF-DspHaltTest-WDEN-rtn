/* filename: def28x.h */
/* By RealSYS	2003/10/23 */

#ifndef	__DEF28X_H
#define	__DEF28X_H

#include "DSP281x_Device.h"

/* define constants */
#define 	LOW		0
#define 	LF		0x0A	// Line feed
#define 	CR		0x0D	// Carrage return

typedef			 char	BOOL;
typedef	unsigned char	BYTE;
typedef	unsigned int	WORD;
typedef	unsigned long	LONG;

#define	NOP		asm("	NOP")
#define	EI		asm("	clrc INTM")
#define	DI		asm("	setc INTM")

#define	HI_BYTE(val)		((val>>8)&0xff)
#define	LO_BYTE(val)		(val&0xff)

#define	BIT_MASK(x)			(1 << (x))
#define GetBit(val, bit)	(((val) & BIT_MASK(bit))>>(bit))
#define	SetBit(val, bit)	((val) |= BIT_MASK(bit))
#define	ClearBit(val, bit)	((val) &= ~BIT_MASK(bit))
#define	ToggleBit(val, bit)	((val) ^=BIT_MASK(bit))

#define	CHK_BIT(x, bit)		((x) & (bit))
#define	SET_BIT(x, bit)		((x) |= (bit))
#define	CLR_BIT(x, bit)		((x) &= ~(bit))
#define	HI_BIT(x, bit)		((x) |= (bit))
#define	LO_BIT(x, bit)		((x) &= ~(bit))
#define	TOGGLE_BIT(x, bit)	((x) ^= (bit))

/* for DSP2812 */
#define	XMEM(a)		*(WORD *)(a)
#define	OUT_LED(d)	*(BYTE *)0x80000 = d
#define	OUT_LCD(d)	*(BYTE *)0x80001 = d
#define	IN_KEY		*(BYTE *)0x80000

/***************/
#define	ON_LED	GpioDataRegs.GPFCLEAR.bit.GPIOF14=1
#define	OFF_LED	GpioDataRegs.GPFSET.bit.GPIOF14=1
#define	T_LED	GpioDataRegs.GPFTOGGLE.bit.GPIOF14=1

#define	H_PE0	GpioDataRegs.GPECLEAR.bit.GPIOE0=1
#define	L_PE0	GpioDataRegs.GPESET.bit.GPIOE0=1
#define	T_PE0	GpioDataRegs.GPETOGGLE.bit.GPIOE0=1

#define	H_PE1	GpioDataRegs.GPECLEAR.bit.GPIOE1=1
#define	L_PE1	GpioDataRegs.GPESET.bit.GPIOE1=1
#define	T_PE1	GpioDataRegs.GPETOGGLE.bit.GPIOE1=1

typedef union 
	{
		BYTE	b[4];
		WORD	w[2];
		LONG	l;
	}ltype;

#define	STD_TYPE	0
#define	EXT_TYPE	1

#endif
