/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig728.h ========
 *  This header defines all types and constants common to all G728
 *  encoder and decoder modules.
 */
#ifndef IG728_
#define IG728_

/*
 *  ======== IG728_Cmd ========
 *  This structure defines the control commands for the G728 encoder
 *  and decoder instance objects.
 */
typedef enum IG728_Cmd {
    IG728_GETSTATUS,
    IG728_SETSTATUS
} IG728_Cmd;

/*
 *  ======== IG728_Mode ========
 *  This structure defines the possible working modes for the G728 encoder
 *  and decoder instance objects.
 */
typedef enum IG728_Mode {
    IG728_LINEAR,
    IG728_ULAW,
    IG728_ALAW
} IG728_Mode;


#endif  /* IG728_ */
