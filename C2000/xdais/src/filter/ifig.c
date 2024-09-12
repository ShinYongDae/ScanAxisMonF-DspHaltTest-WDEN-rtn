/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ifig.c ========
 *  Filter Group - default instance creation parameters
 */
#include <std.h>
#include <ifig.h>

/*
 *  ======== IFIG_PARAMS ========
 */
const IFIG_Params IFIG_PARAMS = {
    sizeof(IFIG_Params),    /* sizeof this structure */
    NULL,                   /* coefficient array */
    0,                      /* length of coefficient array */
};

