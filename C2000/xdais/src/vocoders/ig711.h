/*
 *  Copyright 2003 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) XDAS 2.51.00 11-29-2003 (xdas-2.50.00.9)
 */
/*
 *  ======== ig711.h ========
 *  This header defines all types and constants common to all G711
 *  encoder and decoder modules.
 */
#ifndef IG711_
#define IG711_

/*
 *  ======== IG711_Cmd ========
 *  This structure defines the control commands for G711 encoder and G711
 *  decoder instance objects.
 */
typedef enum IG711_Cmd {
    IG711_GETSTATUS,
    IG711_SETSTATUS
} IG711_Cmd;

/*
 *  ======== IG711_Mode ========
 *  This structure defines the possible working modes for G711 encoder and
 *  G711 decoder instance objects.
 */
typedef enum IG711_Mode {
    IG711_ULAW,
    IG711_ALAW
} IG711_Mode;

#endif  /* IG711_ */
