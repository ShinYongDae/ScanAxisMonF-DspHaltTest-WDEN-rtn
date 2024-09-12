/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig726dec.c ========
 *  Default creation parameters for all implementations of the G726DEC
 *  module abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ig726dec.h>

/*
 *  ======== IG726DEC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G726DEC object.
 */
const IG726DEC_Params IG726DEC_PARAMS = {
    sizeof(IG726DEC_PARAMS),   /* Size of this structure */
    1,                         /* Sample by sample processing */
    IG726_ALAW,                /* Out buffer is A-law */
    IG726_16KBPS,              /* Working rate is 16kbps */
};

