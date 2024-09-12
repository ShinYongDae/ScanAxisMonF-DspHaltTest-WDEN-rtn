/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig726enc.c ========
 *  Default creation parameters for all implementations of the G726ENC
 *  module abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ig726.h>
#include <ig726enc.h>

/*
 *  ======== IG726ENC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G726ENC object.
 */
const IG726ENC_Params IG726ENC_PARAMS = {
    sizeof(IG726ENC_PARAMS),    /* Size of this structure */
    1,                          /* Sample by sample processing */
    IG726_ALAW,                 /* Input buffer format is A-law */
    IG726_16KBPS,               /* Working rate is 16kbps */
};

