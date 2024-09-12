/*************************************************/
/*		  eep2811.h 	RealSYS                	 	 */
/*************************************************/
#ifndef __EEP2811_INCLUDED_
#define __EEP2811_INCLUDED_

#include "..\def28x.h"

int		eep_write_byte(BYTE page,BYTE addr,BYTE wd);	/* EEP byte write operation    */
int	    eep_read_byte(BYTE page,BYTE addr);    			/* EEP byte read operation */
int	    eep_write_block(BYTE page,BYTE addr, char *p);	/* EEP block write operation    */
int	    eep_read_block(BYTE page,BYTE addr, char *p); 	/* EEP block read operation */

#endif
