/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig728dec.c ========
 *  Default creation parameters for all implementations of the G728DEC
 *  module abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ig728.h>
#include <ig728dec.h>

/*
 *  ======== IG728DEC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G728DEC object.
 */
const IG728DEC_Params IG728DEC_PARAMS = {
    sizeof(IG728DEC_PARAMS),   /* Size of this structure */
    IG728_ALAW,                /* Out buffer is A-law */
    XDAS_TRUE,                 /* Post Filter is tuned on */
    0,                         /* Inband synch is disabled */
};

