;****************************************************************************
;* BOOT.ASM
;*
;* THIS IS THE INITAL BOOT ROUTINE FOR C28xx C++ PROGRAMS.
;* IT MUST BE LINKED AND LOADED WITH ALL C++ PROGRAMS.
;*
;* THIS MODULE PERFORMS THE FOLLOWING ACTIONS:
;*   1) SET RESET VECTOR TO POINT AT _C_INT00
;*   2) ALLOCATES THE STACK AND INITIALIZES THE STACK POINTER
;*   3) SET UP PROPER STATUS
;*   4) PERFORMS AUTO-INITIALIZATION
;*   5) CALLS INITALIZATION ROUTINES FOR FILE SCOPE CONSTRUCTION
;*   6) CALLS THE FUNCTION MAIN TO START THE C++ PROGRAM
;*   7) CALLS THE STANDARD EXIT ROUTINE
;*
;* THIS MODULE DEFINES THE FOLLOWING GLOBAL SYMBOLS:
;*   1) __stack     STACK MEMORY AREA
;*   2) _c_int00    BOOT ROUTINE
;*
;****************************************************************************
CONST_COPY	.set 0
  ;      .if __TI_EABI__
      ;     .asg _args_main, __args_main
      ;     .asg exit, _exit
      ;     .asg copy_in, _copy_in
      ;     .asg _system_pre_init, __system_pre_init
      ;     .asg _system_post_cinit, __system_post_cinit
     ;      .global __TI_auto_init
     ;   .endif

	.global  _c_int00, cinit, binit, pinit
	.global  __args_main, _exit, _copy_in
	.global  __system_pre_init, __system_post_cinit
	.global  __stack
	.global  epwm1_isr
****************************************************************************
* Declare the stack.  Size is determined by the linker option -stack.  The *
* default value is 1K words.                                               *
****************************************************************************
__stack:	.usect	".stack",0



   .sect "codestart"

code_start:

     LB wd_disable       ;Branch to watchdog disable code
****************************************************************************
*  INITIALIZE RESET VECTOR TO POINT AT _c_int00                            *
****************************************************************************
	.sect .reset
	.long _c_int00

	.text

wd_disable:
    SETC OBJMODE        ;Set OBJMODE for 28x object code
    EALLOW              ;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6  ;Set data page for WDCR register
    MOV @7029h, #0068h  ;Set WDDIS bit in WDCR to disable WD
    EDIS                ;Disable EALLOW protected register access
    LB _c_int00

****************************************************************************
* FUNCTION DEF : _c_int00                                                  *
*                                                                          *
****************************************************************************

_c_int00:	.asmfunc stack_usage(0)
****************************************************************************
*  INITIALIZE STACK POINTER.                                               *
****************************************************************************
	MOV  	SP,#__stack		; set to beginning of stack space

****************************************************************************
* INITIALIZE STATUS BIT FIELDS *NOT* INITIALIZED AT RESET                  *
****************************************************************************
	SPM	0			; set product shift to 0

****************************************************************************
* SET C28x MODES                                                           *
****************************************************************************
    C28OBJ                          ; select C28x object mode
    C28ADDR                         ; clear the addressing mode
    C28MAP                          ; set block M0 and M1 mode

    .if .TMS320C2800_FPU32
		SETFLG		RNDF32=1      	; Enable rounding in FPU32 mode.
    .endif
****************************************************************************
* SETTING THESE STATUS BITS/REGISTER TO RESET VALUES.  IF YOU RUN          *
* _c_int00 FROM RESET, YOU CAN REMOVE THIS CODE                            *
****************************************************************************
	CLRC    PAGE0			; use stack addressing mode
	MOVW    DP,#0			; initialize DP to point at low 64K
	CLRC    OVM         	; turn off overflow mode

	ASP						; ensure SP is aligned

****************************************************************************
*  IF _system_pre_init is 0, bypass C/C++ autoinitialization               *
****************************************************************************
	;LCR		#__system_pre_init
	;CMP		AL, #0
	;B		BYPASS_AUTO_INIT, EQ ; Branch if _system_pre_init returns 0

 ;   .if     __TI_EABI__
  ;  	LCR #__TI_auto_init
  ;  .else

****************************************************************************
* SET THE EALLOW BIT BEFORE THE CINIT TABLE IS COPIED.                     *
****************************************************************************
	EALLOW

****************************************************************************
*  IF cinit IS NOT -1, PROCESS CINIT INITIALIZATION TABLE	           *
****************************************************************************
	MOV		AL,#cinit
	MOV		AH,#hi16(cinit)
	ADDB	ACC,#1
	B		DO_BINIT,EQ		; if cinit < 0 (-1) no init tables

****************************************************************************
*  PROCESS CINIT INITIALIZATION TABLE.  TABLE IS IN PROGRAM MEMORY IN THE  *
*  FOLLOWING FORMAT:                                                       *
*                                                                          *
*       .word  <length of init data in words>                              *
*       .word  or .long <address of variable to initialize>                *
*       .word  <init data>                                                 *
*       .word  ...                                                         *
*                                                                          *
*  If the variable's address is greater than 65535 (located in 'far'       *
*  memory), then the address field of the cinit record will be 32-bits     *
*  instead of the default 16-bits. The length value is negated to tag      *
*  cinit records for those variables located in far memory.                *
*                                                                          *
*  The init table is terminated with a zero length                         *
*                                                                          *
****************************************************************************
	MOVL	XAR7,#cinit		; point XAR7 at start of table
	CLRC    TC		        ; reset TC bit used as far flag
	B 		START, UNC		; jump to start processing
LOOP:
	MOVB    AH,#0		        ; zero out upper addr bits
	PREAD   AL,*XAR7		; load address of variable to be inited
	ADDB    XAR7,#1			; point to initialization data
	B		GET_DATA,NTC	        ; get data if variable is not far
	CLRC    TC		        ; reset TC bit used as far flag
	PREAD   AH,*XAR7	        ; otherwise, get hi bits of 22-bit addr
	ADDB    XAR7,#1
GET_DATA:
	MOVL	XAR6,ACC	        ; address
	RPT		AR1			; repeat length + 1 times
||	PREAD   *XAR6++,*XAR7		; copy data from table to memory

	MOVL	ACC,XAR7		; using ACC as temp, point XAR7 to
	ADD  	ACC,AR1			; next cinit record since PREAD
	ADDB	ACC,#1			; doesn't change value of XAR7.
	MOVL	XAR7,ACC
START:
	PREAD	AL,*XAR7		; load length
	B		GET_ADDR,GEQ	        ; a length < 0 denotes far data
	NEG     AL		        ; negate value to get real length
	SETC    TC		        ; flag that the address field is 32-bits
GET_ADDR:
	MOVZ	AR1,AL		        ; copy length value to loop register
	ADDB    XAR7,#1			; point to address field
	BANZ	LOOP,AR1--		; if (length-- != 0) continue

****************************************************************************
*  IF binit IS NOT -1, PROCESS CINIT INITIALIZATION TABLE	           *
****************************************************************************
DO_BINIT:
	MOV		AL,#binit
	MOV		AH,#hi16(binit)
	ADDB	ACC,#1
	B		DO_PINIT,EQ		; if binit < 0 (-1) no init tables
	MOVL	XAR4,#binit
	LCR		#_copy_in

****************************************************************************
*  IF pinit IS NOT -1, PROCESS CONSTRUCTOR ROUTINES                        *
****************************************************************************
DO_PINIT:

****************************************************************************
* CLEAR THE EALLOW BIT AFTER THE CINIT TABLE IS COPIED.                    *
****************************************************************************
	EDIS

****************************************************************************
* Call the startup hook function.                                          *
****************************************************************************
	;LCR     #__system_post_cinit

	MOV		AL,#pinit
	MOV		AH,#hi16(pinit)
	ADDB	ACC,#1
	B		DONE_INIT,EQ		; if pinit < 0 (-1) no pinit table

****************************************************************************
*  PROCESS PINIT SECTION. TABLE CONSISTS OF CONSTRUCTOR ROUTINE ADDRESSES  *
*  THAT POINT TO C++ INITIALIZATION FUNCTIONS. THESE ROUTINES MUST EXECUTE *
*  BEFORE MAIN IS CALLED                                                   *
****************************************************************************
	MOVL    XAR7,#pinit	        ; set up pointer to initialization table
	B		START_PINIT, UNC        ; jump to start processing

LOOP_PINIT:
	ADDB    XAR7,#1			; point to next table entry
	PUSH	XAR7			; save off table entry address
	MOVL	XAR7,ACC		; load constructor routine address
	LCR     *XAR7			; execute routine
	POP		XAR7			; reload table entry address

START_PINIT:
	PREAD   AL,*XAR7		; unpack constructor routine address
	ADDB    XAR7,#1
	PREAD   AH,*XAR7
	TEST	ACC				; test for table end condition (zero)
	B		LOOP_PINIT,NEQ	; process table entry if not zero


****************************************************************************
*  COPY CONSTANTS TO DATA MEMORY, IF NECESSARY                             *
****************************************************************************
DONE_INIT:
	.if CONST_COPY
        LCR	_const_init
	.endif

 ;       .endif  ; __TI_EABI__
BYPASS_AUTO_INIT:
****************************************************************************
*  CALL USER'S PROGRAM                                                     *
****************************************************************************
MAIN:




;Step 1. Initialize System Control:
;PLL, WatchDog, enable Peripheral Clocks
DisableDog:
		ADDB         SP, #2
		EALLOW
		MOVW         DP, #0x1c0
		MOV          AL, @0x29
		ANDB         AL, #0x7
		MOV          *-SP[1], AL
		MOV          AL, *-SP[1]
		ORB          AL, #0x68
		MOV          @0x29, AL
		EDIS
		SPM          #0
		SUBB         SP, #2
		EDIS

		EALLOW
		SPM          #0





GPIO_EnableUnbondedIOPullupsFor100Pin:
		EALLOW
		MOVL         XAR4, #0x3ffc1c
		MOVW         DP, #0x1f0
		MOVL         @0xc, XAR4
		MOV          @AL, #0x0e00
		MOV          @AH, #0xfc00
		MOVW         DP, #0x1f1
		MOVL         @0xc, ACC
		MOV          @AL, #0x43e7
		MOV          @AH, #0x1ef0
		MOVW         DP, #0x1f2
		MOVL         @0xc, ACC
		MOVW         DP, #0x1f3
		MOVB         ACC, #8
		MOVL         @0xc, ACC
		MOVW         DP, #0x1f4
		MOVB         ACC, #0
		MOVL         @0xc, ACC
		SETC         SXM
		MOVW         DP, #0x1f5
		MOV          ACC, #0xffff << 9
		MOVL         @0xc, ACC
		EDIS
		SPM          #0

    ;CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    ;CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    ;CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
    ;CpuSysRegs.PCLKCR13.bit.ADC_D = 1;
		EALLOW
		MOVW         DP, #0x174c
		OR           @0x3c, #0x0001
		OR           @0x3c, #0x0002
		OR           @0x3c, #0x0004
		OR           @0x3c, #0x0008

		MOVL         XAR4, #0x007bdd
		MOVW         DP, #0x1746
		MOVL         @0x36, XAR4
		MOVL         @0x38, XAR4
		MOVL         @0x3a, XAR4
		MOVL         @0x3c, XAR4


		MOVW         DP, #0x174c
		AND          @0x3c, #0xfffe
		AND          @0x3c, #0xfffd
		AND          @0x3c, #0xfffb
		AND          @0x3c, #0xfff7
		EDIS

SysXtalOscSel:
		EALLOW
		MOVW         DP, #0x1748
		AND          @0x8, #0xffef
		AND          AL, @0x8, #0xfffc
		ORB          AL, #0x1
		MOV          @0x8, AL
		EDIS
		SPM          #0

		RPT #200 || NOP

InitSysPll:
		EALLOW
;Bypass PLL and set dividers to /1
		MOVW         DP, #0x1748
		AND          @0xe, #0xfffd
		RPT          #20
		|| NOP
		AND          @0x22, #0xffc0


		MOV		AR0,#5
PLL_LOOP:
		MOVW         DP, #0x1748
		AND          @0xe, #0xfffe
		RPT          #20
		|| NOP
		MOV	ACC,#40		;Write multiplier, which automatically turns on the PLL
		MOVL         @0x14, ACC
;Wait for the SYSPLL lock counter
C$L14:
		MOV          AL, @0x16
		ANDB         AL, #0x1
		CMPB         AL, #0x1
		SB           C$L14, NEQ

		BANZ	PLL_LOOP,AR0--



;Set divider to produce slower output frequency to limit current increase
		MOV		AL,#0X0002
		MOV     @0x22, AL




		MOVW         DP, #0x1744
		OR           @0x2c, #0x0001
		MOVW         DP, #0x1748
		OR           @0xe, #0x0002
		RPT          #20
		|| NOP



;Set the divider to user value
		RPT          #200
		|| NOP
		MOVW    DP, #0x1748
		MOV		AL,#0X0001
		MOV     @0x22, AL
		EDIS

InitPeripheralClocks:
		EALLOW
		MOVW         DP, #0x174c
		OR           @0x22, #0x0001
		OR           @0x22, #0x0004
		OR           @0x22, #0x0008
		OR           @0x22, #0x0010
		OR           @0x22, #0x0020
		OR           @0x23, #0x0001
		OR           @0x23, #0x0004
		OR           @0x24, #0x0001
		OR           @0x24, #0x0002
		OR           @0x26, #0x0001
		OR           @0x26, #0x0002
		OR           @0x26, #0x0004
		OR           @0x26, #0x0008
		OR           @0x26, #0x0010
		OR           @0x26, #0x0020
		OR           @0x26, #0x0040
		OR           @0x26, #0x0080
		OR           @0x26, #0x0100
		OR           @0x26, #0x0200
		OR           @0x26, #0x0400
		OR           @0x26, #0x0800
		OR           @0x28, #0x0001
		OR           @0x28, #0x0002
		OR           @0x28, #0x0004
		OR           @0x28, #0x0008
		OR           @0x28, #0x0010
		OR           @0x28, #0x0020
		OR           @0x2a, #0x0001
		OR           @0x2a, #0x0002
		OR           @0x2a, #0x0004
		OR           @0x2e, #0x0001
		OR           @0x2e, #0x0002
		OR           @0x30, #0x0001
		OR           @0x30, #0x0002
		OR           @0x30, #0x0004
		OR           @0x30, #0x0008
		OR           @0x32, #0x0001
		OR           @0x32, #0x0002
		OR           @0x32, #0x0004
		OR           @0x34, #0x0001
		OR           @0x34, #0x0002
		OR           @0x36, #0x0001
		OR           @0x36, #0x0002
		OR           @0x38, #0x0001
		OR           @0x38, #0x0002
		OR           @0x39, #0x0001
		OR           @0x3a, #0x0001
		OR           @0x3c, #0x0001
		OR           @0x3c, #0x0002
		OR           @0x3c, #0x0004
		OR           @0x3c, #0x0008
		OR           @0x3e, #0x0001
		OR           @0x3e, #0x0002
		OR           @0x3e, #0x0004
		OR           @0x3e, #0x0008
		OR           @0x3e, #0x0010
		OR           @0x3e, #0x0020
		OR           @0x3e, #0x0040
		OR           @0x3e, #0x0080
		MOVW         DP, #0x174d
		OR           @0x3, #0x0001
		OR           @0x3, #0x0002
		OR           @0x3, #0x0004
		EDIS
		SPM          #0


;init the pins for the SCI-A port.
;gpio84 85
		EALLOW
		MOVW         DP, #0x1f2
		AND          AL, @0x8, #0xfcff
		OR           @AL, #0x0100
		MOV          @0x8, AL
		AND          AL, @0x8, #0xf3ff
		OR           @AL, #0x0400
		MOV          @0x8, AL
		AND          AL, @0x22, #0xfcff
		OR           @AL, #0x0100
		MOV          @0x22, AL
		AND          AL, @0x22, #0xf3ff
		OR           @AL, #0x0400
		MOV          @0x22, AL
		EDIS





;enable PWM1, PWM2 and PWM3


		MOVW         DP, #0x174c
		OR           @0x26, #0x0001


InitEPwm1Gpio:
		EALLOW
		MOVW         DP, #0x1f0
		OR           @0xc, #0x0001
		OR           @0xc, #0x0002
		AND          AL, @0x6, #0xfffc
		ORB          AL, #0x1
		MOV          @0x6, AL
		AND          AL, @0x6, #0xfff3
		ORB          AL, #0x4
		MOV          @0x6, AL
		EDIS
		SPM          #0

;Step 3. Clear all interrupts and initialize PIE vector table:
;Disable CPU interrupts

		SETC         INTM

InitPieCtrl:
		SETC         INTM
		MOVW         DP, #0x33
		AND          @0x20, #0xfffe
		MOV          @0x22, #0
		MOV          @0x24, #0
		MOV          @0x26, #0
		MOV          @0x28, #0
		MOV          @0x2a, #0
		MOV          @0x2c, #0
		MOV          @0x2e, #0
		MOV          @0x30, #0
		MOV          @0x32, #0
		MOV          @0x34, #0
		MOV          @0x36, #0
		MOV          @0x38, #0
		MOV          @0x23, #0
		MOV          @0x25, #0
		MOV          @0x27, #0
		MOV          @0x29, #0
		MOV          @0x2b, #0
		MOV          @0x2d, #0
		MOV          @0x2f, #0
		MOV          @0x31, #0
		MOV          @0x33, #0
		MOV          @0x35, #0
		MOV          @0x37, #0
		MOV          @0x39, #0
		SPM          #0

;Disable CPU interrupts and clear all CPU interrupt flags
		AND          IER, #0x0000
		AND          IFR, #0x0000

InitPieVectTable:
		ADDB         SP, #6
		MOVL         XAR4, #0x00a800
		MOVL         *-SP[2], XAR4
		MOVL         XAR4, #0x000d00
		MOVL         *-SP[4], XAR4
		MOVB         ACC, #6
		ADDL         ACC, *-SP[2]
		MOVL         *-SP[2], ACC
		MOVB         ACC, #6
		ADDL         ACC, *-SP[4]
		MOVL         *-SP[4], ACC
		EALLOW
		MOV          *-SP[5], #0
		MOV          AL, *-SP[5]
		CMPB         AL, #0xdd
		SB           C$L2, HIS
C$L1:
		MOVL         XAR5, *-SP[4]
		MOVL         XAR4, *-SP[2]
		MOVL         @ACC, XAR5
		MOVL         XAR6, *XAR4++
		ADDB         ACC, #2
		MOVL         *-SP[2], XAR4
		MOVL         *-SP[4], ACC
		MOVL         *+XAR5[0], XAR6
		INC          *-SP[5]
		MOV          AL, *-SP[5]
		CMPB         AL, #0xdd
		SB           C$L1, LO
C$L2:
		EDIS
		MOVW         DP, #0x33
		OR           @0x20, #0x0001
		SPM          #0
		SUBB         SP, #6




;Interrupts that are used in this example are re-mapped to
;ISR functions found within this file.
;PieVectTable.EPWM1_INT = &epwm1_isr;
		EALLOW
		MOVW         DP, #0x35
		MOVL         XAR4, #epwm1_isr
		MOVL         @0x20, XAR4
		;MOVL         XAR4, #epwm1_isr
		;MOVL         @0x22, XAR4
		;MOVL         XAR4, #epwm1_isr
		;MOVL         @0x24, XAR4
		EDIS

;initialize the ePWM

		EALLOW
		MOVW         DP, #0x174c
		AND          @0x23, #0xfffb ;CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
		EDIS
		SPM          #0


InitEPwm1:
		MOVW         DP, #0x101
		MOV          @0x23, #0x07d0
		MOV          @0x21, #0
		MOVW         DP, #0x100
		MOV          @0x4, #0
		MOVW         DP, #0x101
		MOVB         @0x2b, #0x32, UNC
		MOV          @0x2d, #0x079e
		MOVW         DP, #0x100
		AND          AL, @0x0, #0xfffc
		ORB          AL, #0x2
		MOV          @0x0, AL
		AND          @0x0, #0xfffb
		AND          @0x0, #0xfc7f
		AND          @0x0, #0xe3ff
		AND          @0x8, #0xffef
		AND          @0x8, #0xffbf
		AND          @0x8, #0xfffc
		AND          @0x8, #0xfff3
		MOVW         DP, #0x101
		AND          AL, @0x0, #0xffcf
		ORB          AL, #0x20
		MOV          @0x0, AL
		AND          AL, @0x0, #0xff3f
		ORB          AL, #0x40
		MOV          @0x0, AL
		AND          AL, @0x2, #0xfcff
		OR           @AL, #0x0200
		MOV          @0x2, AL
		AND          AL, @0x2, #0xf3ff
		OR           @AL, #0x0400
		MOV          @0x2, AL
		MOVW         DP, #0x102
		AND          AL, @0x24, #0xfff8
		ORB          AL, #0x1
		MOV          @0x24, AL
		OR           @0x24, #0x0008
		OR           @0x26, #0x0003
		MOVW         DP, #0x2a7
		MOVB         @0xc, #0x01, UNC
		MOV          @0xd, #0
		MOV          @0xe, #0
		MOVL         XAR4, #0x004000
		MOVL         @0xa, XAR4
		MOV          @0xf, #0x079e
		MOVB         @0x10, #0x32, UNC
		MOV          @0x11, #0x079e
		MOVB         @0x12, #0x32, UNC

		EALLOW
		MOVW         DP, #0x174c
		OR           @0x23, #0x0004 ;CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
		EDIS



;Enable CPU INT3 which is connected to EPWM1-3 INT:
		OR           IER, #0x0004

;Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
		MOVW         DP, #0x33
		OR           @0x26, #0x0001
		OR           @0x26, #0x0002
		OR           @0x26, #0x0004



scia_fifo_init:
		MOVW         DP, #0x1c8
		MOV          @0xa, #0xe040
		MOV          @0xb, #0x2044
		MOV          @0xc, #0
;  scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 9600,
;                       default, 1 STOP bit, no parity
scia_echoback_init:
		MOVW         DP, #0x1c8
		MOVB         @0x0, #0x07, UNC
		MOVB         @0x1, #0x03, UNC
		MOVB         @0x4, #0x03, UNC
		OR           @0x4, #0x0001
		OR           @0x4, #0x0002
		MOVB         @0x2, #0x02, UNC
		MOVB         @0x3, #0x8b, UNC
		MOVB         @0x1, #0x23, UNC


		CLRC         INTM
		CLRC         DBGM	;enable interrput

;----------------------------LED INIT----------------------------
LED_INIT:
		EALLOW
		MOVL	XAR4,#0x007C0A
		MOV		*XAR4,#0x3000
		MOVL	XAR4,#0x007F06
		OR		*XAR4,#0x1000
		EDIS


MIAN_LOOP:



;----------------------------LED END----------------------------
		MOV		AL,#0x0000
		MOV		AH,#0x0010
DELAY:
		SUB		ACC,#1
		BF		DELAY,GEQ

		MOVL	XAR4,#0x007F06
		OR		*XAR4,#0x2000		;blue led
;----------------------------LED END----------------------------


		LB	MIAN_LOOP



led_count .usect "ramgs0",2,0,0



epwm1_isr:
		ASP
		PUSH         RB
		PUSH         AR1H:AR0H
		MOVL         *SP++, XT
		MOVL         *SP++, XAR4
		MOVL         *SP++, XAR5
		MOVL         *SP++, XAR6
		MOVL         *SP++, XAR7
		MOV32        *SP++, STF
		MOV32        *SP++, R0H
		MOV32        *SP++, R1H
		MOV32        *SP++, R2H
		MOV32        *SP++, R3H
		SETFLG       RNDF32=1,RNDF64=1
		SPM          #0
		CLRC         OVM|PAGE0
		CLRC         AMODE
		MOVL         XAR4, #0x00a9ca




		MOVW	DP,#led_count
		MOVB	ACC,#1
		ADDL	@led_count,ACC
		MOV		AH,#0
		MOV		AL,#5000
		CMPL	ACC,@led_count
		BF		led_end,GEQ
		MOV		AH,#0
		MOV		AL,#0
		MOVL	@led_count,ACC
		MOVL	XAR4,#0x007F06
		OR		*XAR4,#0x1000		;red led


		MOV		AL,#0xAD
		LCR		scia_xmit


led_end:
		;LCR          update_compare


		MOVW         DP, #0x102
		OR           @0x2a, #0x0001
		MOVW         DP, #0x33
		MOVB         @0x21, #0x04, UNC
		MOV32        R3H, *--SP, UNCF
		MOV32        R2H, *--SP, UNCF
		MOV32        R1H, *--SP, UNCF
		MOV32        R0H, *--SP, UNCF
		MOV32        STF, *--SP
		MOVL         XAR7, *--SP
		MOVL         XAR6, *--SP
		MOVL         XAR5, *--SP
		MOVL         XAR4, *--SP
		MOVL         XT, *--SP
		POP          AR1H:AR0H
		SETC         INTM|DBGM
		POP          RB
		NASP
		IRET








scia_xmit:
		ADDB         SP, #2
		MOV          *-SP[1], AL
C$L3:
		MOVW         DP, #0x1c8
		AND          AL, @0xa, #0x1f00
		LSR          AL, 8
		SBF          C$L3, NEQ
		MOV          AL, *-SP[1]
		MOV          @0x9, AL
		SUBB         SP, #2
		LRETR








	.endasmfunc



	.if CONST_COPY

****************************************************************************
* FUNCTION DEF : __const_init                                              *
*                                                                          *
*  COPY .CONST AND .ECONST SECTION FROM PROGRAM TO DATA MEMORY             *
*                                                                          *
*   The function depends on the following variables                        *
*   defined in the linker command file                                     *
*                                                                          *
*   __c_load         ; global var containing start                         *
*                      of .const in program memory                         *
*   __const_run      ; global var containing run                           *
*                      address in data memory                              *
*   __const_length   ; global var length of .const                         *
*                      section                                             *
*                                                                          *
*                                                                          *
*   Similarly for constants to be placed into extended memory (far):       *
*                                                                          *
*   __ec_load         ; global var containing start                        *
*                      of .econst in program memory                        *
*   __econst_run      ; global var containing run                          *
*                      address in data memory                              *
*   __econst_length   ; global var length of .econst                       *
*                      section                                             *
*                                                                          *
****************************************************************************
        .global __const_length,  __c_load,  __const_run
        .global __econst_length, __ec_load, __econst_run

        .sect ".c_mark"              ; establish LOAD adress of
        .label __c_load              ; .const section

        .sect ".ec_mark"              ; establish LOAD adress of
        .label __ec_load              ; .econst section

        .text
_const_init:	.asmfunc stack_usage(2)

        MOV     AL,#__const_length
        B       __econst_init,EQ
        DEC     AL
        MOVL    XAR6,#__const_run
        MOVL    XAR7,#__c_load
        RPT     AL
||      PREAD   *XAR6++,*XAR7

__econst_init:
        MOVL    XAR6, #__econst_length
        MOVL    ACC,XAR6
        B       __end_const,EQ
        MOVL    XAR6,#__econst_run
        MOVL    XAR7,#__ec_load
        DEC     AL
        B       __econst_loop,GEQ
        DEC     AH
__econst_loop:
        MOV     AR0,AH
        RPT     AL
||      PREAD   *XAR6++,*XAR7
        MOV     AH,#0
        ADDL    ACC,XAR7
        ADDB    ACC, #1
        MOVL    XAR7,ACC
        MOV     AL, #0xffff
        MOV     AH,AR0
        SUBB    AH,#1
        B       __econst_loop,GEQ

******************************************************
*  AT END OF CONSTANT SECTIONS RETURN TO CALLER      *
******************************************************
__end_const:
        LRETR
	.endasmfunc
        .endif			;  if CONST_COPY
