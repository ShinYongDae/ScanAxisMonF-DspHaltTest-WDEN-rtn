/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig726.h ========
 *  This header defines all types and constants common to all G726
 *  encoder and decoder modules.
 */
#ifndef IG726_
#define IG726_

/*
 *  ======== IG726_Cmd ========
 *  This structure defines the control commands for the G726 encoder
 *  and decoder instance objects.
 */
typedef enum IG726_Cmd {
    IG726_GETSTATUS,
    IG726_SETSTATUS,
    IG726_END_COMMANDS
} IG726_Cmd;

/*
 *  ======== IG726_Mode ========
 *  This structure defines the possible working modes for the G726 encoder
 *  and decoder instance objects.
 */
typedef enum IG726_Mode {
    IG726_LINEAR,
    IG726_ULAW,
    IG726_ALAW
} IG726_Mode;

/*
 *  ======== IG726_Rate ========
 *  This structure defines the possible working rates for the G726 encoder
 *  and decoder instance objects. 
 */
typedef enum IG726_Rate {
    IG726_16KBPS,       /* 16k bits per second */
    IG726_24KBPS,       /* 24k bits per second */
    IG726_32KBPS,       /* 32k bits per second */
    IG726_40KBPS        /* 40k bits per second */
} IG726_Rate;

#endif  /* IG726_ */
