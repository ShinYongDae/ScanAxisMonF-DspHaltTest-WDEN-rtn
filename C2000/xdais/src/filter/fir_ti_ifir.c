/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir_ti_ifir.c ========
 *  FIR Filter Module - TI implementation of a FIR filter algorithm
 */
#include <std.h>

#include <ifir.h>
#include <fir_ti.h>
#include <fir_ti_priv.h>

#include <xdas.h>
#include <string.h>     /* memcpy() declaration */

/*
 *  ======== dot ========
 */
static Int dot(Int *a, Int *b, Int n)
{
    XDAS_Int32 sum = 0;
    Int i;
    
    for (i = 0; i < n; i++) {
        sum += (XDAS_Int32)*a++ * (XDAS_Int32)*b++;
    }

    return (sum >>15);
}

/*
 *  ======== FIR_TI_filter ========
 */
Void FIR_TI_filter(IFIR_Handle handle, Int in[], Int out[])
{
    FIR_TI_Obj *fir = (Void *)handle;
    Int *src = fir->workBuf;
    Int *dst = out;
    Int i;
    
    /* copy input buffer into working buffer */
    memcpy((Void *)(fir->workBuf + fir->filterLenM1), (Void *)in,
        fir->frameLen * sizeof (Int));

    /* filter data */
    for (i = 0; i < fir->frameLen; i++) {
        *dst++ = dot(src++, fir->coeff, fir->filterLenM1 + 1);
    }

    /* shift filter history to beginning of working buffer for next frame */
    memcpy((Void *)fir->workBuf, (Void *)(fir->workBuf + fir->frameLen),
        fir->filterLenM1 * sizeof (Int));
}

