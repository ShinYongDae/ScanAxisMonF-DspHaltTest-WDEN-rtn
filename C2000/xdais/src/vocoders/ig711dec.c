/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig711dec.c ========
 *  Default instance creation parameters the G711 decoder module.
 */
#include <std.h>

#include <xdas.h>
#include <ig711.h>
#include <ig711dec.h>

/*
 *  ======== IG711DEC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G711DEC object.
 */
const IG711DEC_Params IG711DEC_PARAMS = {
    sizeof(IG711DEC_PARAMS),  /* Size of this structure */
    80,                       /* 80 samples per frame, 10msec of data */
    IG711_ALAW,               /* Input data format is A-law */
};

