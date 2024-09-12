/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== cptd.c ========
 *  CPTD detection Module - implements all functions and defines
 *  all constant structures common to all CPTD algorithm implementations.
 *
 */

#include <std.h>

#include <alg.h>
#include <xdas.h>
#include <cptd.h>

/*
 *  ======== CPTD_apply ========
 *  Apply a CPTD detector to the in buffer. Place the detected tones in the
 *  out buffer and return number of tones detected.
 */
XDAS_Int8 CPTD_apply(CPTD_Handle handle, XDAS_Int16 *in, CPTD_Tone *out)
{
    XDAS_Int8  numTones;

    /* Check for valid handle and fxn ptr */
    if (handle == NULL || handle->fxns->detect == NULL) {
        return (-1);    /* detection failed */
    }

    /* Activate instance object */
    ALG_activate((IALG_Handle)handle);

    /* Detect digits */
    numTones=handle->fxns->detect(handle, in, out); 
    
    /* Deactivate instance object */
    ALG_deactivate((IALG_Handle)handle);

    return(numTones);
}

/*
 *  ======== CPTD_control ========
 *  Function to either write to the read/write parameters in the status
 *  structure or to read all the parameters in the status structure.
 */
XDAS_Bool CPTD_control(CPTD_Handle handle, CPTD_Cmd cmd, CPTD_Status *status)
{
   /* Check for valid handle and fxn ptr and do control operation */ 
   if (handle && handle->fxns->control) {
        return(handle->fxns->control(handle, cmd, status));
    }

    return (XDAS_FALSE);
}

/*
 *  ======== CPTD_exit ========
 *  CPTD module finalization
 */
Void CPTD_exit(Void)
{
}

/*
 *  ======== CPTD_init ========
 *  CPTD module initialization
 */
Void CPTD_init(Void)
{
}

