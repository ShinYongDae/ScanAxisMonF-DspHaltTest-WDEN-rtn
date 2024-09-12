/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== fir_ti.h ========
 *  Vendor specific (TI) interface header for FIR algorithm.
 *
 *  Applications that use this interface enjoy type safety and
 *  and minimal overhead at the expense of being tied to a
 *  particular FIR implementation.
 *
 *  This header only contains declarations that are specific
 *  to this implementation.  Thus, applications that do not
 *  want to be tied to a particular implementation should never
 *  include this header (i.e., it should never directly
 *  reference anything defined in this header.)
 */
#ifndef FIR_TI_
#define FIR_TI_

#include <ialg.h>
#include <ifir.h>

/*
 *  ======== FIR_TI_exit ========
 *  Required module finalization function
 */
extern Void FIR_TI_exit(Void);

/*
 *  ======== FIR_TI_init ========
 *  Required module initialization function
 */
extern Void FIR_TI_init(Void);

/*
 *  ======== FIR_TI_IALG ========
 *  TI's implementation of FIR's IALG interface
 */
extern IALG_Fxns FIR_TI_IALG; 

/*
 *  ======== FIR_TI_IFIR ========
 *  TI's implementation of FIR's IFIR interface
 */
extern IFIR_Fxns FIR_TI_IFIR; 

/*
 *  ======== Vendor specific methods  ========
 *  The remainder of this file illustrates how a vendor can
 *  extend an interface with custom operations.
 *
 *  The operations below simply provide a type safe interface 
 *  for the creation, deletion, and application of TI's FIR filters.
 *  However, other implementation specific operations can also
 *  be added.
 */

/*
 *  ======== FIR_TI_Handle ========
 */
typedef struct FIR_TI_Obj *FIR_TI_Handle;

/*
 *  ======== FIR_TI_Params ========
 *  We don't add any new parameters to the standard ones defined
 *  by IFIR.
 */
typedef IFIR_Params FIR_TI_Params;

/*
 *  ======== FIR_TI_PARAMS ========
 *  Define our defult parameters.
 */
#define FIR_TI_PARAMS   IFIR_PARAMS

/*
 *  ======== FIR_TI_create ========
 *  Create a FIR_TI instance object.
 */
extern FIR_TI_Handle FIR_TI_create(const FIR_TI_Params *params);

/*
 *  ======== FIR_TI_delete ========
 *  Delete a FIR_TI instance object.
 */
extern Void FIR_TI_delete(FIR_TI_Handle handle);

/*
 *  ======== FIR_TI_nApply ========
 *  Apply specified FIR filter to n input frames and overwrite
 *  input with the result.
 */
extern Void FIR_TI_nApply(FIR_TI_Handle handle, Int inout[], Int n);

#endif  /* FIR_TI_ */
