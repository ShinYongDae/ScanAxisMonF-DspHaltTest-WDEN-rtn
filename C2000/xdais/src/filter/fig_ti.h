/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fig_ti.h ========
 *  Vendor specific (TI) interface header for Filter Group algorithm
 */
#ifndef FIG_TI_
#define FIG_TI_

#include <ialg.h>
#include <ifig.h>

/*
 *  ======== FIG_TI_exit ========
 *  Required module finalization function
 */
extern Void FIG_TI_exit(Void);

/*
 *  ======== FIG_TI_init ========
 *  Required module initialization function
 */
extern Void FIG_TI_init(Void);

/*
 *  ======== FIG_TI_IALG ========
 *  TI's implementation of FIG's IALG interface
 */
extern IALG_Fxns FIG_TI_IALG;

/*
 *  ======== FIG_TI_IFIG ========
 *  TI's implementation of FIG's IFIG interface
 */
extern IFIG_Fxns FIG_TI_IFIG;

#endif  /* FIG_TI_ */
