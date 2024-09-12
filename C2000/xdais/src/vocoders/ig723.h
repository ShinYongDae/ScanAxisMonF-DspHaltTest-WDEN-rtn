/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig723.h ========
 *  This header defines all types and constants common to all G723
 *  encoder and decoder modules.
 */
#ifndef IG723_
#define IG723_

/*
 *  ======== IG723_Cmd ========
 *  This structure defines the control commands for the G723 encoder
 *  and decoder instance objects.
 */
typedef enum IG723_Cmd {
    IG723_GETSTATUS,
    IG723_SETSTATUS
} IG723_Cmd;

/*
 *  ======== IG723_Rate ========
 *  This structure defines the possible working rates for G723 encoder
 *  and decoder instance objects. 
 */
typedef enum IG723_Rate {
    IG723_5300BPS = 0,    /* 5.3k bits per second */
    IG723_6300BPS = 1     /* 6.3k bits per second */
} IG723_Rate;

#endif  /* IG723_ */
