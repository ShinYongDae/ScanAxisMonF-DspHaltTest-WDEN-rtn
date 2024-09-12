/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== icptd.c ========
 *  Definition of the default creation parameters for all implementations 
 *  of the CPTD modules abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <icptd.h>

/*
 *  ======== ICPTD_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a CPTD object.
 */
const ICPTD_Params ICPTD_PARAMS = {
    sizeof(ICPTD_PARAMS),  /* Size of this structure */
    80,                    /* 80 samples per frame, 10msec of data */
};

