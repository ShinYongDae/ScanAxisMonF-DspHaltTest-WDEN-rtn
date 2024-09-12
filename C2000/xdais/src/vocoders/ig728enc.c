/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig728enc.c ========
 *  Default creation parameters for all implementations of the G728ENC
 *  module abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ig728.h>
#include <ig728enc.h>

/*
 *  ======== IG728ENC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G728ENC object.
 */
const IG728ENC_Params IG728ENC_PARAMS = {
    sizeof(IG728ENC_PARAMS),    /* Size of this structure */
    IG728_ULAW,                 /* Input buffer format is u-law */
    XDAS_TRUE,                  /* Perceptual weighting filter in turned on */
    0,                          /* Inband sync disabled */ 
};

