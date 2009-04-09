section .text
;align	16

global	_nasm_test_mov_disp
_nasm_test_mov_disp:
	mov al, [1]
	mov cl, [1]
	mov ax, [1]
	mov cx, [1]
	mov eax, [1]
	mov ecx, [1]
	mov [1], al
	mov [1], cl
	mov [1], ax
	mov [1], cx
	mov [1], eax
	mov [1], ecx

global	_nasm_test_avx_a
_nasm_test_avx_a:
	vaddpd xmm1, xmm2, xmm3
	vaddpd xmm1, xmm2, [edx]
	vaddpd ymm1, ymm2, ymm3
	vaddpd ymm1, ymm2, [edx]
	vaddps xmm1, xmm2, xmm3
	vaddps xmm1, xmm2, [edx]
	vaddps ymm1, ymm2, ymm3
	vaddps ymm1, ymm2, [edx]
	vaddsd xmm1, xmm2, [edx]
	vaddss xmm1, xmm2, [edx]
	vaddsubpd xmm1, xmm2, xmm3
	vaddsubpd xmm1, xmm2, [edx]
	vaddsubpd ymm1, ymm2, ymm3
	vaddsubpd ymm1, ymm2, [edx]
	vaddsubps xmm1, xmm2, xmm3
	vaddsubps xmm1, xmm2, [edx]
	vaddsubps ymm1, ymm2, ymm3
	vaddsubps ymm1, ymm2, [edx]
	;vaesenc xmm1, xmm2
	;vaesenc xmm1, [edx]
	;vaesenclast xmm1, xmm2
	;vaesenclast xmm1, [edx]
	;vaesdec xmm1, xmm2
	;vaesdec xmm1, [edx]
	;vaesdeclast xmm1, xmm2
	;vaesdeclast xmm1, [edx]
	;vaesimc xmm1, xmm2
	;vaesimc xmm1, [edx]
	;vaeskeygenassist xmm1, xmm2, 3
	;vaeskeygenassist xmm1, [edx], 3
	vandpd xmm1, xmm2, xmm3
	vandpd xmm1, xmm2, [edx]
	vandpd ymm1, ymm2, ymm3
	vandpd ymm1, ymm2, [edx]
	vandps xmm1, xmm2, xmm3
	vandps xmm1, xmm2, [edx]
	vandps ymm1, ymm2, ymm3
	vandps ymm1, ymm2, [edx]
	vandnpd xmm1, xmm2, xmm3
	vandnpd xmm1, xmm2, [edx]
	vandnpd ymm1, ymm2, ymm3
	vandnpd ymm1, ymm2, [edx]
	vandnps xmm1, xmm2, xmm3
	vandnps xmm1, xmm2, [edx]
	vandnps ymm1, ymm2, ymm3
	vandnps ymm1, ymm2, [edx]
