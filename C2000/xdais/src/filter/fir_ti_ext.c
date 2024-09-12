/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir_ti_ext.c ========
 */
#pragma CODE_SECTION(FIR_TI_create, ".text:create")
#pragma CODE_SECTION(FIR_TI_delete, ".text:delete")

#include <std.h>
#include <alg.h>
#include <ialg.h>
#include <fir.h>
#include <ifir.h>

#include <fir_ti.h>
#include <fir_ti_priv.h>

/*
 *  ======== FIR_TI_create ========
 */
FIR_TI_Handle FIR_TI_create(const FIR_Params *params)
{
    return ((Void *)ALG_create(&FIR_TI_IALG, NULL, (IALG_Params *)params));
}

/*
 *  ======== FIR_TI_delete ========
 */
Void FIR_TI_delete(FIR_TI_Handle handle)
{
    ALG_delete((ALG_Handle)handle);
}

/*
 *  ======== FIR_TI_nApply ========
 */
Void FIR_TI_nApply(FIR_TI_Handle handle, Int input[], Int n)
{
    Int *in;
    Int i;

    ALG_activate((ALG_Handle)handle);

    for (in = input, i = 0; i < n; i++) {
        FIR_TI_filter((IFIR_Handle)handle, in, in);
        in += handle->frameLen;
    }

    ALG_deactivate((ALG_Handle)handle);
}

