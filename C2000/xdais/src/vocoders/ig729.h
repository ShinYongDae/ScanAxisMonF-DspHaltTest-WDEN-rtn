/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig729.h ========
 *  This header defines all types and constants common to all G729
 *  encoder and decoder modules.
 */
#ifndef IG729_
#define IG729_

/*
 *  ======== IG729_Cmd ========
 *  This structure defines the control commands for the G729 encoder
 *  and decoder instance objects. 
 */
typedef enum IG729_Cmd {
    IG729_GETSTATUS,
    IG729_SETSTATUS
} IG729_Cmd;

#endif  /* IG729_ */
