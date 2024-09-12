/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== idtmf.c ========
 *  Default creation parameters definition for all implementations of the
 *  DTMF modules abstract interface. 
 */
#include <std.h>

#include <xdas.h>
#include <idtmf.h>

/*
 *  ======== IDTMF_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a DTMF object.
 */
const IDTMF_Params IDTMF_PARAMS = {
    sizeof(IDTMF_PARAMS),  /* Size of this structure */
    80,                    /* 80 samples per frame, 10msec of data */
};

