	.section	__TEXT,__text,regular,pure_instructions
	.build_version macos, 15, 0	sdk_version 26, 0
	.globl	__Z19PROJ_TriangulateDLTRKNSt3__16vectorIN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEENS_9allocatorIS3_EEEES8_RKNS2_IdLi3ELi4ELi0ELi3ELi4EEESB_ ; -- Begin function _Z19PROJ_TriangulateDLTRKNSt3__16vectorIN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEENS_9allocatorIS3_EEEES8_RKNS2_IdLi3ELi4ELi0ELi3ELi4EEESB_
	.p2align	2
__Z19PROJ_TriangulateDLTRKNSt3__16vectorIN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEENS_9allocatorIS3_EEEES8_RKNS2_IdLi3ELi4ELi0ELi3ELi4EEESB_: ; @_Z19PROJ_TriangulateDLTRKNSt3__16vectorIN5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEENS_9allocatorIS3_EEEES8_RKNS2_IdLi3ELi4ELi0ELi3ELi4EEESB_
Lfunc_begin0:
	.cfi_startproc
	.cfi_personality 155, ___gxx_personality_v0
	.cfi_lsda 16, Lexception0
; %bb.0:
	stp	d15, d14, [sp, #-160]!          ; 16-byte Folded Spill
	stp	d13, d12, [sp, #16]             ; 16-byte Folded Spill
	stp	d11, d10, [sp, #32]             ; 16-byte Folded Spill
	stp	d9, d8, [sp, #48]               ; 16-byte Folded Spill
	stp	x28, x27, [sp, #64]             ; 16-byte Folded Spill
	stp	x26, x25, [sp, #80]             ; 16-byte Folded Spill
	stp	x24, x23, [sp, #96]             ; 16-byte Folded Spill
	stp	x22, x21, [sp, #112]            ; 16-byte Folded Spill
	stp	x20, x19, [sp, #128]            ; 16-byte Folded Spill
	stp	x29, x30, [sp, #144]            ; 16-byte Folded Spill
	add	x29, sp, #144
	sub	sp, sp, #736
	.cfi_def_cfa w29, 16
	.cfi_offset w30, -8
	.cfi_offset w29, -16
	.cfi_offset w19, -24
	.cfi_offset w20, -32
	.cfi_offset w21, -40
	.cfi_offset w22, -48
	.cfi_offset w23, -56
	.cfi_offset w24, -64
	.cfi_offset w25, -72
	.cfi_offset w26, -80
	.cfi_offset w27, -88
	.cfi_offset w28, -96
	.cfi_offset b8, -104
	.cfi_offset b9, -112
	.cfi_offset b10, -120
	.cfi_offset b11, -128
	.cfi_offset b12, -136
	.cfi_offset b13, -144
	.cfi_offset b14, -152
	.cfi_offset b15, -160
	mov	x20, x1
	mov	x21, x0
	mov	x19, x8
Lloh0:
	adrp	x8, ___stack_chk_guard@GOTPAGE
Lloh1:
	ldr	x8, [x8, ___stack_chk_guard@GOTPAGEOFF]
Lloh2:
	ldr	x8, [x8]
	stur	x8, [x29, #-152]
	ldp	d1, d0, [x2]
	stp	d0, d1, [sp, #112]              ; 16-byte Folded Spill
	ldp	d1, d0, [x2, #48]
	stp	d0, d1, [sp, #96]               ; 16-byte Folded Spill
	ldp	d13, d1, [x2, #16]
	ldp	d0, d15, [x2, #32]
	stp	d0, d1, [sp, #80]               ; 16-byte Folded Spill
	ldp	d9, d1, [x2, #64]
	ldp	d0, d11, [x2, #80]
	stp	d0, d1, [sp, #64]               ; 16-byte Folded Spill
	ldp	d1, d0, [x3]
	stp	d0, d1, [sp, #48]               ; 16-byte Folded Spill
	ldp	d1, d0, [x3, #48]
	stp	d0, d1, [sp, #32]               ; 16-byte Folded Spill
	ldp	d14, d1, [x3, #16]
	ldp	d0, d10, [x3, #32]
	stp	d0, d1, [sp, #16]               ; 16-byte Folded Spill
	ldp	d8, d1, [x3, #64]
	ldp	d0, d12, [x3, #80]
	stp	d0, d1, [sp]                    ; 16-byte Folded Spill
	ldp	x24, x25, [x0]
	stp	xzr, xzr, [x19]
	str	xzr, [x19, #16]
	subs	x8, x25, x24
	b.eq	LBB0_3
; %bb.1:
	asr	x26, x8, #4
	lsr	x9, x26, #59
	cbnz	x9, LBB0_11
; %bb.2:
	lsl	x23, x8, #1
	mov	x0, x23
	bl	__Znwm
	mov	x22, x0
	str	x0, [x19]
	add	x8, x0, x26, lsl #5
	str	x8, [x19, #16]
	mov	x1, x23
	bl	_bzero
	add	x8, x22, x23
	str	x8, [x19, #8]
	b	LBB0_4
LBB0_3:
	mov	x22, #0                         ; =0x0
LBB0_4:
	add	x8, sp, #256
	add	x9, x8, #281
	movi.2d	v0, #0000000000000000
	str	q0, [x9]
	add	x8, x8, #297
	stp	q0, q0, [sp, #512]
	stp	q0, q0, [sp, #480]
	stp	q0, q0, [sp, #448]
	stp	q0, q0, [sp, #416]
	stp	q0, q0, [sp, #384]
	stp	q0, q0, [sp, #352]
	stp	q0, q0, [sp, #320]
	stp	q0, q0, [sp, #288]
	stp	q0, q0, [sp, #256]
	mov	w9, #1                          ; =0x1
	strh	w9, [x8]
	mov	w8, #16                         ; =0x10
	str	w8, [sp, #556]
	str	xzr, [sp, #560]
	str	xzr, [sp, #576]
	cmp	x25, x24
	b.eq	LBB0_8
; %bb.5:
	mov	x23, #0                         ; =0x0
	mov	x25, #0                         ; =0x0
	add	x26, x22, #16
LBB0_6:                                 ; =>This Inner Loop Header: Depth=1
	add	x8, x24, x23
	ldr	d0, [x8]
	ldr	d1, [sp, #120]                  ; 8-byte Folded Reload
	fnmsub	d1, d13, d0, d1
	ldr	d2, [sp, #88]                   ; 8-byte Folded Reload
	fnmsub	d2, d15, d0, d2
	str	d1, [sp, #128]
	str	d2, [sp, #160]
	ldr	d1, [sp, #104]                  ; 8-byte Folded Reload
	fnmsub	d1, d9, d0, d1
	ldp	d3, d2, [sp, #72]               ; 16-byte Folded Reload
	fnmsub	d0, d11, d0, d3
	str	d1, [sp, #192]
	str	d0, [sp, #224]
	ldr	d0, [x8, #8]
	ldr	d1, [sp, #112]                  ; 8-byte Folded Reload
	fnmsub	d1, d13, d0, d1
	fnmsub	d2, d15, d0, d2
	str	d1, [sp, #136]
	str	d2, [sp, #168]
	ldr	d1, [sp, #96]                   ; 8-byte Folded Reload
	fnmsub	d1, d9, d0, d1
	ldr	d2, [sp, #64]                   ; 8-byte Folded Reload
	fnmsub	d0, d11, d0, d2
	str	d1, [sp, #200]
	str	d0, [sp, #232]
	ldr	x8, [x20]
	add	x8, x8, x23
	ldr	d0, [x8]
	ldr	d1, [sp, #56]                   ; 8-byte Folded Reload
	fnmsub	d1, d14, d0, d1
	ldr	d2, [sp, #24]                   ; 8-byte Folded Reload
	fnmsub	d2, d10, d0, d2
	str	d1, [sp, #144]
	str	d2, [sp, #176]
	ldr	d1, [sp, #40]                   ; 8-byte Folded Reload
	fnmsub	d1, d8, d0, d1
	ldp	d3, d2, [sp, #8]                ; 16-byte Folded Reload
	fnmsub	d0, d12, d0, d3
	str	d1, [sp, #208]
	str	d0, [sp, #240]
	ldr	d0, [x8, #8]
	ldr	d1, [sp, #48]                   ; 8-byte Folded Reload
	fnmsub	d1, d14, d0, d1
	fnmsub	d2, d10, d0, d2
	str	d1, [sp, #152]
	str	d2, [sp, #184]
	ldr	d1, [sp, #32]                   ; 8-byte Folded Reload
	fnmsub	d1, d8, d0, d1
	ldr	d2, [sp]                        ; 8-byte Folded Reload
	fnmsub	d0, d12, d0, d2
	str	d1, [sp, #216]
	str	d0, [sp, #248]
	ldr	w2, [sp, #556]
Ltmp0:
	add	x0, sp, #256
	add	x1, sp, #128
	bl	__ZN5Eigen9JacobiSVDINS_6MatrixIdLi4ELi4ELi0ELi4ELi4EEELi16EE12compute_implIS2_EERS3_RKNS_10MatrixBaseIT_EEj
Ltmp1:
; %bb.7:                                ;   in Loop: Header=BB0_6 Depth=1
	ldr	q0, [sp, #480]
	stur	q0, [x26, #-16]
	ldr	q0, [sp, #496]
	str	q0, [x26], #32
	add	x25, x25, #1
	ldp	x24, x8, [x21]
	sub	x8, x8, x24
	add	x23, x23, #16
	cmp	x25, x8, asr #4
	b.lo	LBB0_6
LBB0_8:
	ldur	x8, [x29, #-152]
Lloh3:
	adrp	x9, ___stack_chk_guard@GOTPAGE
Lloh4:
	ldr	x9, [x9, ___stack_chk_guard@GOTPAGEOFF]
Lloh5:
	ldr	x9, [x9]
	cmp	x9, x8
	b.ne	LBB0_10
; %bb.9:
	add	sp, sp, #736
	ldp	x29, x30, [sp, #144]            ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #128]            ; 16-byte Folded Reload
	ldp	x22, x21, [sp, #112]            ; 16-byte Folded Reload
	ldp	x24, x23, [sp, #96]             ; 16-byte Folded Reload
	ldp	x26, x25, [sp, #80]             ; 16-byte Folded Reload
	ldp	x28, x27, [sp, #64]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp, #48]               ; 16-byte Folded Reload
	ldp	d11, d10, [sp, #32]             ; 16-byte Folded Reload
	ldp	d13, d12, [sp, #16]             ; 16-byte Folded Reload
	ldp	d15, d14, [sp], #160            ; 16-byte Folded Reload
	ret
LBB0_10:
	bl	___stack_chk_fail
LBB0_11:
	bl	__ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev
LBB0_12:
Ltmp2:
	mov	x20, x0
	cbz	x22, LBB0_14
; %bb.13:
	str	x22, [x19, #8]
	mov	x0, x22
	bl	__ZdlPv
LBB0_14:
	mov	x0, x20
	bl	__Unwind_Resume
	.loh AdrpLdrGotLdr	Lloh0, Lloh1, Lloh2
	.loh AdrpLdrGotLdr	Lloh3, Lloh4, Lloh5
Lfunc_end0:
	.cfi_endproc
	.section	__TEXT,__gcc_except_tab
	.p2align	2, 0x0
GCC_except_table0:
Lexception0:
	.byte	255                             ; @LPStart Encoding = omit
	.byte	255                             ; @TType Encoding = omit
	.byte	1                               ; Call site Encoding = uleb128
	.uleb128 Lcst_end0-Lcst_begin0
Lcst_begin0:
	.uleb128 Lfunc_begin0-Lfunc_begin0      ; >> Call Site 1 <<
	.uleb128 Ltmp0-Lfunc_begin0             ;   Call between Lfunc_begin0 and Ltmp0
	.byte	0                               ;     has no landing pad
	.byte	0                               ;   On action: cleanup
	.uleb128 Ltmp0-Lfunc_begin0             ; >> Call Site 2 <<
	.uleb128 Ltmp1-Ltmp0                    ;   Call between Ltmp0 and Ltmp1
	.uleb128 Ltmp2-Lfunc_begin0             ;     jumps to Ltmp2
	.byte	0                               ;   On action: cleanup
	.uleb128 Ltmp1-Lfunc_begin0             ; >> Call Site 3 <<
	.uleb128 Lfunc_end0-Ltmp1               ;   Call between Ltmp1 and Lfunc_end0
	.byte	0                               ;     has no landing pad
	.byte	0                               ;   On action: cleanup
Lcst_end0:
	.p2align	2, 0x0
                                        ; -- End function
	.section	__TEXT,__text,regular,pure_instructions
	.private_extern	__ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev ; -- Begin function _ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev
	.globl	__ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev
	.weak_def_can_be_hidden	__ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev
	.p2align	2
__ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev: ; @_ZNSt3__16vectorIN5Eigen6MatrixIdLi4ELi1ELi0ELi4ELi1EEENS_9allocatorIS3_EEE20__throw_length_errorB8ne200100Ev
	.cfi_startproc
; %bb.0:
	stp	x29, x30, [sp, #-16]!           ; 16-byte Folded Spill
	mov	x29, sp
	.cfi_def_cfa w29, 16
	.cfi_offset w30, -8
	.cfi_offset w29, -16
Lloh6:
	adrp	x0, l_.str@PAGE
Lloh7:
	add	x0, x0, l_.str@PAGEOFF
	bl	__ZNSt3__120__throw_length_errorB8ne200100EPKc
	.loh AdrpAdd	Lloh6, Lloh7
	.cfi_endproc
                                        ; -- End function
	.private_extern	__ZNSt3__120__throw_length_errorB8ne200100EPKc ; -- Begin function _ZNSt3__120__throw_length_errorB8ne200100EPKc
	.globl	__ZNSt3__120__throw_length_errorB8ne200100EPKc
	.weak_def_can_be_hidden	__ZNSt3__120__throw_length_errorB8ne200100EPKc
	.p2align	2
__ZNSt3__120__throw_length_errorB8ne200100EPKc: ; @_ZNSt3__120__throw_length_errorB8ne200100EPKc
Lfunc_begin1:
	.cfi_startproc
	.cfi_personality 155, ___gxx_personality_v0
	.cfi_lsda 16, Lexception1
; %bb.0:
	stp	x20, x19, [sp, #-32]!           ; 16-byte Folded Spill
	stp	x29, x30, [sp, #16]             ; 16-byte Folded Spill
	add	x29, sp, #16
	.cfi_def_cfa w29, 16
	.cfi_offset w30, -8
	.cfi_offset w29, -16
	.cfi_offset w19, -24
	.cfi_offset w20, -32
	mov	x20, x0
	mov	w0, #16                         ; =0x10
	bl	___cxa_allocate_exception
	mov	x19, x0
Ltmp3:
	mov	x1, x20
	bl	__ZNSt12length_errorC1B8ne200100EPKc
Ltmp4:
; %bb.1:
Lloh8:
	adrp	x1, __ZTISt12length_error@GOTPAGE
Lloh9:
	ldr	x1, [x1, __ZTISt12length_error@GOTPAGEOFF]
Lloh10:
	adrp	x2, __ZNSt12length_errorD1Ev@GOTPAGE
Lloh11:
	ldr	x2, [x2, __ZNSt12length_errorD1Ev@GOTPAGEOFF]
	mov	x0, x19
	bl	___cxa_throw
LBB2_2:
Ltmp5:
	mov	x20, x0
	mov	x0, x19
	bl	___cxa_free_exception
	mov	x0, x20
	bl	__Unwind_Resume
	.loh AdrpLdrGot	Lloh10, Lloh11
	.loh AdrpLdrGot	Lloh8, Lloh9
Lfunc_end1:
	.cfi_endproc
	.section	__TEXT,__gcc_except_tab
	.p2align	2, 0x0
GCC_except_table2:
Lexception1:
	.byte	255                             ; @LPStart Encoding = omit
	.byte	255                             ; @TType Encoding = omit
	.byte	1                               ; Call site Encoding = uleb128
	.uleb128 Lcst_end1-Lcst_begin1
Lcst_begin1:
	.uleb128 Lfunc_begin1-Lfunc_begin1      ; >> Call Site 1 <<
	.uleb128 Ltmp3-Lfunc_begin1             ;   Call between Lfunc_begin1 and Ltmp3
	.byte	0                               ;     has no landing pad
	.byte	0                               ;   On action: cleanup
	.uleb128 Ltmp3-Lfunc_begin1             ; >> Call Site 2 <<
	.uleb128 Ltmp4-Ltmp3                    ;   Call between Ltmp3 and Ltmp4
	.uleb128 Ltmp5-Lfunc_begin1             ;     jumps to Ltmp5
	.byte	0                               ;   On action: cleanup
	.uleb128 Ltmp4-Lfunc_begin1             ; >> Call Site 3 <<
	.uleb128 Lfunc_end1-Ltmp4               ;   Call between Ltmp4 and Lfunc_end1
	.byte	0                               ;     has no landing pad
	.byte	0                               ;   On action: cleanup
Lcst_end1:
	.p2align	2, 0x0
                                        ; -- End function
	.section	__TEXT,__text,regular,pure_instructions
	.private_extern	__ZNSt12length_errorC1B8ne200100EPKc ; -- Begin function _ZNSt12length_errorC1B8ne200100EPKc
	.globl	__ZNSt12length_errorC1B8ne200100EPKc
	.weak_def_can_be_hidden	__ZNSt12length_errorC1B8ne200100EPKc
	.p2align	2
__ZNSt12length_errorC1B8ne200100EPKc:   ; @_ZNSt12length_errorC1B8ne200100EPKc
	.cfi_startproc
; %bb.0:
	stp	x29, x30, [sp, #-16]!           ; 16-byte Folded Spill
	mov	x29, sp
	.cfi_def_cfa w29, 16
	.cfi_offset w30, -8
	.cfi_offset w29, -16
	bl	__ZNSt11logic_errorC2EPKc
Lloh12:
	adrp	x8, __ZTVSt12length_error@GOTPAGE
Lloh13:
	ldr	x8, [x8, __ZTVSt12length_error@GOTPAGEOFF]
	add	x8, x8, #16
	str	x8, [x0]
	ldp	x29, x30, [sp], #16             ; 16-byte Folded Reload
	ret
	.loh AdrpLdrGot	Lloh12, Lloh13
	.cfi_endproc
                                        ; -- End function
	.globl	__ZN5Eigen9JacobiSVDINS_6MatrixIdLi4ELi4ELi0ELi4ELi4EEELi16EE12compute_implIS2_EERS3_RKNS_10MatrixBaseIT_EEj ; -- Begin function _ZN5Eigen9JacobiSVDINS_6MatrixIdLi4ELi4ELi0ELi4ELi4EEELi16EE12compute_implIS2_EERS3_RKNS_10MatrixBaseIT_EEj
	.weak_def_can_be_hidden	__ZN5Eigen9JacobiSVDINS_6MatrixIdLi4ELi4ELi0ELi4ELi4EEELi16EE12compute_implIS2_EERS3_RKNS_10MatrixBaseIT_EEj
	.p2align	2
__ZN5Eigen9JacobiSVDINS_6MatrixIdLi4ELi4ELi0ELi4ELi4EEELi16EE12compute_implIS2_EERS3_RKNS_10MatrixBaseIT_EEj: ; @_ZN5Eigen9JacobiSVDINS_6MatrixIdLi4ELi4ELi0ELi4ELi4EEELi16EE12compute_implIS2_EERS3_RKNS_10MatrixBaseIT_EEj
	.cfi_startproc
; %bb.0:
	ldrb	w8, [x0, #293]
	cmp	w8, #1
	b.ne	LBB4_2
; %bb.1:
	ldr	w8, [x0, #300]
	cmp	w8, w2
	b.eq	LBB4_3
LBB4_2:
	str	wzr, [x0, #288]
	mov	w8, #256                        ; =0x100
	strh	w8, [x0, #292]
	str	w2, [x0, #300]
	ubfx	w8, w2, #2, #1
	strb	w8, [x0, #295]
	ubfx	w8, w2, #3, #1
	strb	w8, [x0, #296]
	mov	w8, #1                          ; =0x1
	strb	w8, [x0, #297]
	ubfx	w8, w2, #5, #1
	strb	w8, [x0, #298]
LBB4_3:
	ldp	q1, q0, [x1]
	fabs.2d	v2, v1
	fabs.2d	v0, v0
	fmax.2d	v0, v2, v0
	ldp	q2, q3, [x1, #32]
	fabs.2d	v2, v2
	fabs.2d	v3, v3
	fmax.2d	v2, v2, v3
	fmax.2d	v0, v0, v2
	ldp	q2, q3, [x1, #64]
	fabs.2d	v2, v2
	fabs.2d	v3, v3
	fmax.2d	v2, v2, v3
	ldp	q3, q4, [x1, #96]
	fabs.2d	v3, v3
	fabs.2d	v4, v4
	fmax.2d	v3, v3, v4
	fmax.2d	v2, v2, v3
	fmax.2d	v0, v0, v2
	mov	d2, v0[1]
	fcmp	d0, d2
	fccmp	d2, d2, #1, pl
	fcsel	d2, d2, d0, vs
	fcmp	d0, d0
	fcsel	d0, d0, d2, vs
	fmov	x8, d0
	and	x8, x8, #0x7fffffffffffffff
	mov	x9, #9218868437227405312        ; =0x7ff0000000000000
	cmp	x8, x9
	b.lt	LBB4_5
; %bb.4:
	mov	w8, #1                          ; =0x1
	strb	w8, [x0, #292]
	mov	w8, #3                          ; =0x3
	str	w8, [x0, #288]
	str	xzr, [x0, #304]
	ret
LBB4_5:
	fcmp	d0, #0.0
	fmov	d2, #1.00000000
	fcsel	d0, d2, d0, eq
	dup.2d	v4, v0[0]
	fdiv.2d	v1, v1, v4
	str	q1, [x0, #336]
	ldr	q2, [x1, #16]
	fdiv.2d	v2, v2, v4
	str	q2, [x0, #352]
	ldr	q2, [x1, #32]
	fdiv.2d	v2, v2, v4
	str	q2, [x0, #368]
	ldr	q3, [x1, #48]
	fdiv.2d	v3, v3, v4
	str	q3, [x0, #384]
	ldr	q3, [x1, #64]
	fdiv.2d	v3, v3, v4
	str	q3, [x0, #400]
	ldr	q3, [x1, #80]
	fdiv.2d	v3, v3, v4
	str	q3, [x0, #416]
	ldr	q5, [x1, #96]
	fdiv.2d	v5, v5, v4
	str	q5, [x0, #432]
	ldr	q5, [x1, #112]
	fdiv.2d	v4, v5, v4
	str	q4, [x0, #448]
	ldrb	w8, [x0, #295]
	cmp	w8, #1
	b.eq	LBB4_35
; %bb.6:
	ldrb	w8, [x0, #296]
	cmp	w8, #1
	b.eq	LBB4_36
LBB4_7:
	ldrb	w8, [x0, #297]
	cmp	w8, #1
	b.eq	LBB4_37
LBB4_8:
	ldrb	w8, [x0, #298]
	cmp	w8, #1
	b.ne	LBB4_10
LBB4_9:
	movi.2d	v5, #0000000000000000
	stp	q5, q5, [x0, #224]
	stp	q5, q5, [x0, #192]
	stp	q5, q5, [x0, #160]
	stp	q5, q5, [x0, #128]
	mov	x8, #4607182418800017408        ; =0x3ff0000000000000
	str	x8, [x0, #128]
	str	x8, [x0, #168]
	str	x8, [x0, #208]
	str	x8, [x0, #248]
LBB4_10:
	add	x9, x0, #336
	zip1.2d	v1, v1, v3
	fabs.2d	v3, v1
	zip2.2d	v2, v2, v4
	fabs.2d	v4, v2
	facgt.2d	v1, v2, v1
	bsl.16b	v1, v4, v3
	mov	d2, v1[1]
	fcmp	d1, d2
	fcsel	d1, d2, d1, mi
	add	x8, x0, #128
	mov	w10, #1                         ; =0x1
	mov	x11, #4377498837804122112       ; =0x3cc0000000000000
	fmov	d2, x11
	mov	x11, #4503599627370496          ; =0x10000000000000
	fmov	d3, x11
	fmov	d4, #1.00000000
	mov	w11, #1                         ; =0x1
	mov	w5, #1                          ; =0x1
	b	LBB4_12
LBB4_11:                                ;   in Loop: Header=BB4_12 Depth=1
	add	x12, x11, #1
	cmp	x12, #4
	cset	w12, eq
	and	w13, w12, w5
	csinc	x11, x10, x11, eq
	orr	w5, w12, w5
	tbnz	w13, #0, LBB4_33
LBB4_12:                                ; =>This Loop Header: Depth=1
                                        ;     Child Loop BB4_16 Depth 2
	mov	x12, #0                         ; =0x0
	mov	x13, #0                         ; =0x0
	lsl	x1, x11, #5
	add	x14, x9, x1
	lsl	x2, x11, #3
	add	x15, x9, x2
	add	x16, x0, x1
	add	x17, x8, x1
	add	x1, x1, #336
	add	x2, x2, #336
	mov	w3, #336                        ; =0x150
	mov	w4, #432                        ; =0x1b0
	b	LBB4_16
LBB4_13:                                ;   in Loop: Header=BB4_16 Depth=2
	ldr	q7, [x17]
	ldr	q16, [x5, #128]
	fmul.2d	v17, v16, v6[0]
	fmla.2d	v17, v7, v5[0]
	str	q17, [x17]
	fmul.2d	v16, v16, v5[0]
	fmls.2d	v16, v7, v6[0]
	str	q16, [x5, #128]
	ldr	q7, [x17, #16]
	ldr	q16, [x5, #144]
	fmul.2d	v17, v16, v6[0]
	fmla.2d	v17, v7, v5[0]
	str	q17, [x17, #16]
	fmul.2d	v5, v16, v5[0]
	fmls.2d	v5, v7, v6[0]
	str	q5, [x5, #144]
LBB4_14:                                ;   in Loop: Header=BB4_16 Depth=2
	mov	w5, #0                          ; =0x0
	ldr	d5, [x14, x11, lsl #3]
	ldr	d6, [x0, x3]
	fabs	d5, d5
	fabs	d6, d6
	fcmp	d5, d6
	fcsel	d5, d6, d5, mi
	fcmp	d1, d5
	fcsel	d1, d5, d1, mi
LBB4_15:                                ;   in Loop: Header=BB4_16 Depth=2
	add	x13, x13, #1
	add	x1, x1, #8
	add	x12, x12, #32
	add	x4, x4, #8
	add	x2, x2, #32
	add	x3, x3, #40
	cmp	x11, x13
	b.eq	LBB4_11
LBB4_16:                                ;   Parent Loop BB4_12 Depth=1
                                        ; =>  This Inner Loop Header: Depth=2
	fmul	d5, d1, d2
	fmaxnm	d5, d5, d3
	ldr	d6, [x0, x2]
	fabs	d17, d6
	ldr	d19, [x0, x1]
	fabs	d7, d19
	fcmp	d17, d5
	fccmp	d7, d5, #0, le
	b.le	LBB4_15
; %bb.17:                               ;   in Loop: Header=BB4_16 Depth=2
	ldr	d18, [x14, x11, lsl #3]
	ldr	d5, [x0, x3]
	fabd	d7, d19, d6
	fcmp	d7, d3
	b.pl	LBB4_19
; %bb.18:                               ;   in Loop: Header=BB4_16 Depth=2
	fmov	d16, #1.00000000
	movi	d7, #0000000000000000
	fcmp	d16, d4
	b.eq	LBB4_20
	b	LBB4_23
LBB4_19:                                ;   in Loop: Header=BB4_16 Depth=2
	fsub	d7, d19, d6
	fadd	d16, d18, d5
	fdiv	d16, d16, d7
	fmadd	d7, d16, d16, d4
	fsqrt	d20, d7
	fdiv	d7, d4, d20
	fdiv	d16, d16, d20
	fcmp	d16, d4
	b.ne	LBB4_23
LBB4_20:                                ;   in Loop: Header=BB4_16 Depth=2
	fcmp	d7, #0.0
	b.ne	LBB4_23
; %bb.21:                               ;   in Loop: Header=BB4_16 Depth=2
	fadd	d19, d17, d17
	fcmp	d19, d3
	b.mi	LBB4_24
LBB4_22:                                ;   in Loop: Header=BB4_16 Depth=2
	fsub	d5, d18, d5
	fdiv	d5, d5, d19
	fmadd	d18, d5, d5, d4
	fsqrt	d18, d18
	fneg	d19, d18
	fcmp	d5, #0.0
	fcsel	d18, d18, d19, gt
	fadd	d5, d5, d18
	fdiv	d18, d4, d5
	fmadd	d5, d18, d18, d4
	fsqrt	d5, d5
	fdiv	d5, d4, d5
	fdiv	d6, d6, d17
	fneg	d17, d6
	fcmp	d18, #0.0
	fcsel	d6, d17, d6, gt
	fabs	d17, d18
	fmul	d6, d17, d6
	fmul	d6, d6, d5
	fmul	d17, d16, d5
	fmadd	d17, d7, d6, d17
	fmul	d16, d16, d6
	fnmsub	d7, d7, d5, d16
	fcmp	d17, d4
	b.eq	LBB4_25
	b	LBB4_26
LBB4_23:                                ;   in Loop: Header=BB4_16 Depth=2
	fmul	d17, d18, d16
	fmadd	d18, d19, d7, d17
	fmul	d17, d6, d16
	fmul	d6, d6, d7
	fmadd	d19, d5, d7, d17
	fnmsub	d5, d5, d16, d6
	fabs	d17, d19
	fmov	d6, d19
	fadd	d19, d17, d17
	fcmp	d19, d3
	b.pl	LBB4_22
LBB4_24:                                ;   in Loop: Header=BB4_16 Depth=2
	fmov	d5, #1.00000000
	movi	d6, #0000000000000000
	fmul	d17, d16, d5
	fmadd	d17, d7, d6, d17
	fmul	d16, d16, d6
	fnmsub	d7, d7, d5, d16
	fcmp	d17, d4
	b.ne	LBB4_26
LBB4_25:                                ;   in Loop: Header=BB4_16 Depth=2
	fcmp	d7, #0.0
	b.eq	LBB4_29
LBB4_26:                                ;   in Loop: Header=BB4_16 Depth=2
	add	x5, x0, x4
	ldr	d16, [x15]
	ldur	d18, [x5, #-96]
	fmul	d19, d7, d18
	fmadd	d19, d17, d16, d19
	str	d19, [x15]
	fmul	d16, d7, d16
	fnmsub	d16, d17, d18, d16
	stur	d16, [x5, #-96]
	ldr	d16, [x15, #32]
	ldur	d18, [x5, #-64]
	fmul	d19, d7, d18
	fmadd	d19, d17, d16, d19
	str	d19, [x15, #32]
	fmul	d16, d7, d16
	fnmsub	d16, d17, d18, d16
	stur	d16, [x5, #-64]
	ldr	d16, [x15, #64]
	ldur	d18, [x5, #-32]
	fmul	d19, d7, d18
	fmadd	d19, d17, d16, d19
	str	d19, [x15, #64]
	fmul	d16, d7, d16
	fnmsub	d16, d17, d18, d16
	stur	d16, [x5, #-32]
	ldr	d16, [x15, #96]
	ldr	d18, [x5]
	fmul	d19, d7, d18
	fmadd	d19, d17, d16, d19
	str	d19, [x15, #96]
	fmul	d16, d7, d16
	fnmsub	d16, d17, d18, d16
	str	d16, [x5]
	ldrb	w5, [x0, #295]
	tbnz	w5, #0, LBB4_28
; %bb.27:                               ;   in Loop: Header=BB4_16 Depth=2
	ldrb	w5, [x0, #296]
	cmp	w5, #1
	b.ne	LBB4_29
LBB4_28:                                ;   in Loop: Header=BB4_16 Depth=2
	add	x5, x0, x12
	ldr	q16, [x16]
	ldr	q18, [x5]
	fmul.2d	v19, v18, v7[0]
	fmla.2d	v19, v16, v17[0]
	str	q19, [x16]
	fmul.2d	v18, v18, v17[0]
	fmls.2d	v18, v16, v7[0]
	str	q18, [x5]
	ldr	q16, [x16, #16]
	ldr	q18, [x5, #16]
	fmul.2d	v19, v18, v7[0]
	fmla.2d	v19, v16, v17[0]
	str	q19, [x16, #16]
	fmul.2d	v17, v18, v17[0]
	fmls.2d	v17, v16, v7[0]
	str	q17, [x5, #16]
LBB4_29:                                ;   in Loop: Header=BB4_16 Depth=2
	fcmp	d6, #0.0
	b.ne	LBB4_31
; %bb.30:                               ;   in Loop: Header=BB4_16 Depth=2
	fcmp	d5, d4
	b.eq	LBB4_14
LBB4_31:                                ;   in Loop: Header=BB4_16 Depth=2
	add	x5, x0, x12
	fneg	d6, d6
	ldr	q7, [x14]
	ldr	q16, [x5, #336]
	fmul.2d	v17, v16, v6[0]
	fmla.2d	v17, v7, v5[0]
	str	q17, [x14]
	fmul.2d	v16, v16, v5[0]
	fmls.2d	v16, v7, v6[0]
	str	q16, [x5, #336]
	add	x5, x0, x12
	ldr	q7, [x14, #16]
	ldr	q16, [x5, #352]
	fmul.2d	v17, v16, v6[0]
	fmla.2d	v17, v7, v5[0]
	str	q17, [x14, #16]
	fmul.2d	v16, v16, v5[0]
	fmls.2d	v16, v7, v6[0]
	str	q16, [x5, #352]
	ldrb	w6, [x0, #297]
	tbnz	w6, #0, LBB4_13
; %bb.32:                               ;   in Loop: Header=BB4_16 Depth=2
	ldrb	w6, [x0, #298]
	cmp	w6, #1
	b.eq	LBB4_13
	b	LBB4_14
LBB4_33:
	ldr	d1, [x0, #336]
	fabs	d2, d1
	str	d2, [x0, #256]
	ldrb	w9, [x0, #295]
	tbz	w9, #0, LBB4_38
; %bb.34:
	mov	w10, #1                         ; =0x1
	fcmp	d1, #0.0
	b.mi	LBB4_39
	b	LBB4_41
LBB4_35:
	movi.2d	v5, #0000000000000000
	stp	q5, q5, [x0, #96]
	stp	q5, q5, [x0, #64]
	stp	q5, q5, [x0, #32]
	stp	q5, q5, [x0]
	mov	x8, #4607182418800017408        ; =0x3ff0000000000000
	str	x8, [x0]
	str	x8, [x0, #40]
	str	x8, [x0, #80]
	str	x8, [x0, #120]
	ldrb	w8, [x0, #296]
	cmp	w8, #1
	b.ne	LBB4_7
LBB4_36:
	movi.2d	v5, #0000000000000000
	stp	q5, q5, [x0, #96]
	stp	q5, q5, [x0, #64]
	stp	q5, q5, [x0, #32]
	stp	q5, q5, [x0]
	mov	x8, #4607182418800017408        ; =0x3ff0000000000000
	str	x8, [x0]
	str	x8, [x0, #40]
	str	x8, [x0, #80]
	str	x8, [x0, #120]
	ldrb	w8, [x0, #297]
	cmp	w8, #1
	b.ne	LBB4_8
LBB4_37:
	movi.2d	v5, #0000000000000000
	stp	q5, q5, [x0, #224]
	stp	q5, q5, [x0, #192]
	stp	q5, q5, [x0, #160]
	stp	q5, q5, [x0, #128]
	mov	x8, #4607182418800017408        ; =0x3ff0000000000000
	str	x8, [x0, #128]
	str	x8, [x0, #168]
	str	x8, [x0, #208]
	str	x8, [x0, #248]
	ldrb	w8, [x0, #298]
	cmp	w8, #1
	b.eq	LBB4_9
	b	LBB4_10
LBB4_38:
	ldrb	w10, [x0, #296]
	fcmp	d1, #0.0
	b.pl	LBB4_41
LBB4_39:
	tbz	w10, #0, LBB4_41
; %bb.40:
	ldp	q1, q2, [x0]
	fneg.2d	v1, v1
	fneg.2d	v2, v2
	stp	q1, q2, [x0]
LBB4_41:
	ldr	d1, [x0, #376]
	fabs	d2, d1
	str	d2, [x0, #264]
	tbz	w9, #0, LBB4_43
; %bb.42:
	mov	w10, #1                         ; =0x1
	fcmp	d1, #0.0
	b.mi	LBB4_44
	b	LBB4_46
LBB4_43:
	ldrb	w10, [x0, #296]
	fcmp	d1, #0.0
	b.pl	LBB4_46
LBB4_44:
	tbz	w10, #0, LBB4_46
; %bb.45:
	ldp	q1, q2, [x0, #32]
	fneg.2d	v1, v1
	fneg.2d	v2, v2
	stp	q1, q2, [x0, #32]
LBB4_46:
	ldr	d1, [x0, #416]
	fabs	d2, d1
	str	d2, [x0, #272]
	tbz	w9, #0, LBB4_48
; %bb.47:
	mov	w10, #1                         ; =0x1
	fcmp	d1, #0.0
	b.mi	LBB4_49
	b	LBB4_51
LBB4_48:
	ldrb	w10, [x0, #296]
	fcmp	d1, #0.0
	b.pl	LBB4_51
LBB4_49:
	tbz	w10, #0, LBB4_51
; %bb.50:
	ldp	q1, q2, [x0, #64]
	fneg.2d	v1, v1
	fneg.2d	v2, v2
	stp	q1, q2, [x0, #64]
LBB4_51:
	ldr	d1, [x0, #456]
	fabs	d2, d1
	str	d2, [x0, #280]
	tbz	w9, #0, LBB4_53
; %bb.52:
	mov	w9, #1                          ; =0x1
	fcmp	d1, #0.0
	b.mi	LBB4_54
	b	LBB4_56
LBB4_53:
	ldrb	w9, [x0, #296]
	fcmp	d1, #0.0
	b.pl	LBB4_56
LBB4_54:
	tbz	w9, #0, LBB4_56
; %bb.55:
	ldp	q1, q2, [x0, #96]
	fneg.2d	v1, v1
	fneg.2d	v2, v2
	stp	q1, q2, [x0, #96]
LBB4_56:
	ldp	q1, q3, [x0, #256]
	fmul.2d	v2, v1, v0[0]
	fmul.2d	v0, v3, v0[0]
	stp	q2, q0, [x0, #256]
	mov	w9, #4                          ; =0x4
	ldr	d0, [x0, #256]
	dup.2d	v3, v0[0]
	str	x9, [x0, #304]
	fcmgt.2d	v1, v2, v3
	faddp.2d	d1, v1
	fcmp	d1, #0.0
	cset	w11, ne
	fmov	d1, d0
	b.eq	LBB4_58
; %bb.57:
	fmaxp.2d	d1, v2
	dup.2d	v3, v1[0]
LBB4_58:
	add	x9, x0, #256
	ldr	q2, [x0, #272]
	fcmgt.2d	v3, v2, v3
	faddp.2d	d3, v3
	fcmp	d3, #0.0
	fmaxp.2d	d2, v2
	mov	w12, #1                         ; =0x1
	mov	w10, #2                         ; =0x2
	csel	x10, xzr, x10, eq
	fcsel	d1, d1, d2, eq
	csel	w11, w11, w12, eq
	cbz	w11, LBB4_61
; %bb.59:
	ldr	d1, [x9, x10, lsl #3]
	add	x11, x10, #1
	ldr	d2, [x9, x11, lsl #3]
	fcmp	d2, d1
	b.le	LBB4_61
; %bb.60:
	mov	x10, x11
	fmov	d1, d2
LBB4_61:
	fcmp	d1, #0.0
	b.ne	LBB4_63
; %bb.62:
	mov	x8, #0                          ; =0x0
	b	LBB4_101
LBB4_63:
	cbz	x10, LBB4_70
; %bb.64:
	ldr	d1, [x9, x10, lsl #3]
	str	d1, [x0, #256]
	str	d0, [x9, x10, lsl #3]
	ldrb	w11, [x0, #295]
	tbnz	w11, #0, LBB4_66
; %bb.65:
	ldrb	w11, [x0, #296]
	cmp	w11, #1
	b.ne	LBB4_67
LBB4_66:
	add	x11, x0, x10, lsl #5
	ldp	q0, q1, [x0]
	ldp	q2, q3, [x11]
	stp	q2, q3, [x0]
	stp	q0, q1, [x11]
LBB4_67:
	ldrb	w11, [x0, #297]
	tbnz	w11, #0, LBB4_69
; %bb.68:
	ldrb	w11, [x0, #298]
	cmp	w11, #1
	b.ne	LBB4_70
LBB4_69:
	add	x10, x8, x10, lsl #5
	ldp	q0, q1, [x0, #128]
	ldp	q2, q3, [x10]
	stp	q2, q3, [x0, #128]
	stp	q0, q1, [x10]
LBB4_70:
	ldr	d0, [x0, #264]
	dup.2d	v2, v0[0]
	add	x11, x0, #264
	ldr	q1, [x11]
	fcmgt.2d	v2, v1, v2
	faddp.2d	d2, v2
	fcmp	d2, #0.0
	fmov	d3, d0
	b.eq	LBB4_72
; %bb.71:
	fmaxp.2d	d3, v1
LBB4_72:
	ldr	d4, [x0, #280]
	fcmp	d4, d3
	cset	w10, gt
	fcsel	d1, d4, d3, gt
	fcmp	d2, #0.0
	fccmp	d4, d3, #0, ne
	ubfiz	x10, x10, #1, #32
	b.gt	LBB4_75
; %bb.73:
	ldr	d1, [x11, x10, lsl #3]
	add	x12, x10, #1
	ldr	d2, [x11, x12, lsl #3]
	fcmp	d2, d1
	b.le	LBB4_75
; %bb.74:
	mov	x10, x12
	fmov	d1, d2
LBB4_75:
	fcmp	d1, #0.0
	b.ne	LBB4_77
; %bb.76:
	mov	w8, #1                          ; =0x1
	b	LBB4_101
LBB4_77:
	cbz	x10, LBB4_84
; %bb.78:
	add	x10, x10, #1
	ldr	d1, [x9, x10, lsl #3]
	str	d1, [x0, #264]
	str	d0, [x9, x10, lsl #3]
	ldrb	w11, [x0, #295]
	tbnz	w11, #0, LBB4_80
; %bb.79:
	ldrb	w11, [x0, #296]
	cmp	w11, #1
	b.ne	LBB4_81
LBB4_80:
	add	x11, x0, x10, lsl #5
	ldp	q0, q1, [x0, #32]
	ldp	q2, q3, [x11]
	stp	q2, q3, [x0, #32]
	stp	q0, q1, [x11]
LBB4_81:
	ldrb	w11, [x0, #297]
	tbnz	w11, #0, LBB4_83
; %bb.82:
	ldrb	w11, [x0, #298]
	cmp	w11, #1
	b.ne	LBB4_84
LBB4_83:
	add	x10, x8, x10, lsl #5
	ldp	q0, q1, [x0, #160]
	ldp	q2, q3, [x10]
	stp	q2, q3, [x0, #160]
	stp	q0, q1, [x10]
LBB4_84:
	ldr	d0, [x0, #272]
	dup.2d	v1, v0[0]
	ldr	q3, [x0, #272]
	fcmgt.2d	v1, v3, v1
	faddp.2d	d2, v1
	fcmp	d2, #0.0
	fmov	d1, d0
	b.eq	LBB4_86
; %bb.85:
	fmaxp.2d	d1, v3
LBB4_86:
	fcmp	d2, #0.0
	b.eq	LBB4_89
; %bb.87:
	ldp	d1, d2, [x0, #272]
	fcmp	d2, d1
	b.le	LBB4_89
; %bb.88:
	mov	w11, #0                         ; =0x0
	mov	w10, #3                         ; =0x3
	fmov	d1, d2
	b	LBB4_90
LBB4_89:
	mov	w11, #1                         ; =0x1
	mov	w10, #2                         ; =0x2
LBB4_90:
	fcmp	d1, #0.0
	b.ne	LBB4_92
; %bb.91:
	mov	w8, #2                          ; =0x2
	b	LBB4_101
LBB4_92:
	tbnz	w11, #0, LBB4_99
; %bb.93:
	ldr	d1, [x9, x10, lsl #3]
	str	d1, [x0, #272]
	str	d0, [x9, x10, lsl #3]
	ldrb	w9, [x0, #295]
	tbnz	w9, #0, LBB4_95
; %bb.94:
	ldrb	w9, [x0, #296]
	cmp	w9, #1
	b.ne	LBB4_96
LBB4_95:
	add	x9, x0, x10, lsl #5
	ldp	q0, q1, [x0, #64]
	ldp	q2, q3, [x9]
	stp	q2, q3, [x0, #64]
	stp	q0, q1, [x9]
LBB4_96:
	ldrb	w9, [x0, #297]
	tbnz	w9, #0, LBB4_98
; %bb.97:
	ldrb	w9, [x0, #298]
	cmp	w9, #1
	b.ne	LBB4_99
LBB4_98:
	add	x8, x8, x10, lsl #5
	ldp	q0, q1, [x0, #192]
	ldp	q2, q3, [x8]
	stp	q2, q3, [x0, #192]
	stp	q0, q1, [x8]
LBB4_99:
	ldr	d0, [x0, #280]
	fcmp	d0, #0.0
	b.ne	LBB4_102
; %bb.100:
	mov	w8, #3                          ; =0x3
LBB4_101:
	str	x8, [x0, #304]
LBB4_102:
	mov	w8, #1                          ; =0x1
	strb	w8, [x0, #292]
	ret
	.cfi_endproc
                                        ; -- End function
	.section	__TEXT,__cstring,cstring_literals
l_.str:                                 ; @.str
	.asciz	"vector"

.subsections_via_symbols
