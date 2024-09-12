/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ilec.c ========
 *  Default creation parameters definition for all implementations of the
 *  LEC modules abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <ilec.h>

/*
 *  ======== ILEC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a LEC object.
 */
const ILEC_Params ILEC_PARAMS = {
    sizeof(ILEC_PARAMS),  /* Size of this structure */
    XDAS_TRUE,            /* Adaptation turned on */
    XDAS_TRUE,            /* Double talk detector turned on */
    40,                   /* 40 samples per frame, 5msec of data */
    XDAS_TRUE,            /* Non-linear processor turned on */
    240,                  /* Bulk delay of 240 samples, 30msec of data */ 
    256,                  /* Tail length (echo spread) of 256 samples, 32msec */
};

