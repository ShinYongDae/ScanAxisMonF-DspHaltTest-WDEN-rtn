/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig711enc.c ========
 *  Default instance creation parameters for a G711 encoder object.
 */
#include <std.h>

#include <xdas.h>
#include <ig711.h>
#include <ig711enc.h>

/*
 *  ======== IG711ENC_PARAMS ========
 *  This static initialization defines the default parameters used to
 *  create an instances of a G711ENC object.
 */
const IG711ENC_Params IG711ENC_PARAMS = {
    sizeof(IG711ENC_PARAMS),  /* Size of this structure */
    80,                       /* 80 samples per frame, 10msec of data */
    IG711_ALAW,               /* Encoded data is A-law format */
};

