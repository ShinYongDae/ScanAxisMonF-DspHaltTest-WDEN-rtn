;//###########################################################################
;//
;// FILE:  DSP281x_CodeStartBranch.asm	
;//
;// TITLE: Branch for redirecting code execution after boot. 
;//
;//###########################################################################
;//
;//  Ver | dd mmm yyyy | Who  | Description of changes
;// =====|=============|======|===============================================
;//  1.00| 11 Sep 03   | L.H. | Updated based on D.A source to allow 
;//      |             |      | disabling the watchdog before branching to
;//      |             |      | the C init routine.  This is useful if the
;//      |             |      | watchdog is timing out before main() is reached.
;//########################################################################### 	


***********************************************************************

WD_DISABLE	.set	1		;set to 1 to disable WD, else set to 0

    .ref _c_int00

***********************************************************************
* Function: codestart section
*
* Description: Branch to code starting point
***********************************************************************

    .sect "codestart"

code_start:
    .if WD_DISABLE == 1
        LB wd_disable       ;Branch to watchdog disable code
    .else
		LB wd_enable       ;Branch to watchdog enable code
;       LB _c_int00         ;Branch to start of boot.asm in RTS library
;		LB copy_sections	;Branch to copy_sections
    .endif

;end codestart section


***********************************************************************
* Function: wd_disable
*
* Description: Disables the watchdog timer
***********************************************************************
    .if WD_DISABLE == 1

    .text
wd_disable:
    SETC OBJMODE        ;Set OBJMODE for 28x object code
    EALLOW              ;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6  ;Set data page for WDCR register
    MOV @7029h, #0068h  ;Set WDDIS bit in WDCR to disable WD
    EDIS                ;Disable EALLOW protected register access
    LB _c_int00         ;Branch to start of boot.asm in RTS library

    .endif

;end wd_disable



	.end
	


***********************************************************************
* Function: wd_enable
*
* Description: Enables the watchdog timer
***********************************************************************
    .if WD_DISABLE == 0

    .text
wd_enable:
    SETC OBJMODE        ;Set OBJMODE for 28x object code
    EALLOW              ;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6  ;Set data page for WDCR register
    MOV @7029h, #0028h  ;Set WDDIS bit in WDCR to enable WD
    EDIS                ;Disable EALLOW protected register access
    LB _c_int00         ;Branch to start of boot.asm in RTS library

    .endif

;end wd_disable



	.end

; end of file CodeStartBranch.asm