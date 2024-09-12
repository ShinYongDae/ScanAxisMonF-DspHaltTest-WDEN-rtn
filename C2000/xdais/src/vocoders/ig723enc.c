/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig723enc.c ========
 *  Default creation parameters for all implementations of the G723ENC
 *  module abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ig723.h>
#include <ig723enc.h>

/*
 *  ======== IG723ENC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G723ENC object.
 */
const IG723ENC_Params IG723ENC_PARAMS = {
    sizeof(IG723ENC_PARAMS),    /* Size of this structure */
    XDAS_TRUE,                  /* Annex A implementation */
    XDAS_TRUE,                  /* High Pass Filter is turned on */
    IG723_5300BPS,              /* Coding rate is 5.3kbps */
    XDAS_TRUE,                  /* VAD turned on (if Annex A) */
};

