/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ifir.h ========
 *  This header defines all types, constants, and functions shared by all
 *  implementations of the FIR algorithm.
 */
#ifndef IFIR_
#define IFIR_

#include <ialg.h>

/*
 *  ======== IFIR_Obj ========
 *  Every implementation of IFIR *must* declare this structure as
 *  the first member of the implementation's object.
 */
typedef struct IFIR_Obj {
    struct IFIR_Fxns *fxns;
} IFIR_Obj;

/*
 *  ======== IFIR_Handle ========
 *  This type is a pointer to an implementation's instance object.
 */
typedef struct IFIR_Obj *IFIR_Handle;

/*
 *  ======== IFIR_Params ========
 *  This structure defines the parameters necessary to create an
 *  instance of a FIR object.
 *
 *  Every implementation of IFIR *must* declare this structure as
 *  the first member of the implementation's parameter structure.
 */
typedef struct IFIR_Params {
    Int size;           /* sizeof the whole parameter struct */
    Int *coeffPtr;      /* pointer to coefficients */
    Int filterLen;      /* length of filter */
    Int frameLen;       /* length of input (output) buffer */
} IFIR_Params;

/*
 *  ======== IFIR_PARAMS ========
 *  Default instance creation parameters (defined in ifir.c)
 */
extern IFIR_Params IFIR_PARAMS;

/*
 *  ======== IFIR_Fxns ========
 *  All implementation's of FIR must declare and statically 
 *  initialize a constant variable of this type.
 *
 *  By convention the name of the variable is FIR_<vendor>_IFIR, where
 *  <vendor> is the vendor name.
 */
typedef struct IFIR_Fxns {
    IALG_Fxns   ialg;
    Void        (*filter)(IFIR_Handle handle, Int in[], Int out[]);
} IFIR_Fxns;

#endif  /* IFIR_ */
