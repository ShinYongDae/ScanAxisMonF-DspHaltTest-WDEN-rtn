/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig729enc.c ========
 *  Default creation parameters for all implementations of the G729ENC
 *  module abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ig729.h>
#include <ig729enc.h>

/*
 *  ======== IG729ENC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G729ENC object.
 */
const IG729ENC_Params IG729ENC_PARAMS = {
    sizeof(IG729ENC_PARAMS),    /* Size of this structure */
    XDAS_TRUE,                  /* Annex A implementation */
    XDAS_TRUE,                  /* Annex B implementation */
    XDAS_TRUE,                  /* Voice Activity Detector turned on */
};

