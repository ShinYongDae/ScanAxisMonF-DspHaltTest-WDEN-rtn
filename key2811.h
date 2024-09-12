/*************************************************/
/*		  key2811.h 	RealSYS                	 	 */
/*************************************************/
#ifndef __KEY2811_INCLUDED_
#define __KEY2811_INCLUDED_

enum 	key_code	{K1CODE,K2CODE,K3CODE,K4CODE,K5CODE,K6CODE,K7CODE,K8CODE};
#define	KEY_MASK	0xff
#define	NOKEY_CODE	0x0f  
#define	CODE_MASK	0x0f
#define	KEY_PRESSED	0x80
#define	KEY_CONT	0x40

extern BYTE	key_code;

#endif
