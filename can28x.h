/*************************************************/
/*		  can28x.h 	RealSYS                	 	 */
/*************************************************/
#ifndef __CAN28X_INCLUDED_
#define __CAN28X_INCLUDED_
#include "..\def28x.h"

void init_can(int type,LONG rx_id);	
void can_send(int type,LONG tx_id,LONG txd0,LONG txd1);

#endif
