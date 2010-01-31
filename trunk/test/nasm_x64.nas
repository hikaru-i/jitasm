section .text
align	16

global	nasm_test_mov_disp
nasm_test_mov_disp:
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

	mov rax, [1]
	mov [1], rax
	mov rax, [qword 100000000h]
	mov [qword 100000000h], rax

global nasm_test_avx_a
nasm_test_avx_a:
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
	;aesenc xmm1, xmm2
	;aesenc xmm1, [edx]
	vaesenc xmm1, xmm2, xmm3
	vaesenc xmm1, xmm2, [edx]
	;aesenclast xmm1, xmm2
	;aesenclast xmm1, [edx]
	vaesenclast xmm1, xmm2, xmm3
	vaesenclast xmm1, xmm2, [edx]
	;aesdec xmm1, xmm2
	;aesdec xmm1, [edx]
	vaesdec xmm1, xmm2, xmm3
	vaesdec xmm1, xmm2, [edx]
	;aesdeclast xmm1, xmm2
	;aesdeclast xmm1, [edx]
	vaesdeclast xmm1, xmm2, xmm3
	vaesdeclast xmm1, xmm2, [edx]
	;aesimc xmm1, xmm2
	;aesimc xmm1, [edx]
	vaesimc xmm1, xmm2
	vaesimc xmm1, [edx]
	;aeskeygenassist xmm1, xmm2, 3
	;aeskeygenassist xmm1, [edx], 3
	vaeskeygenassist xmm1, xmm2, 3
	vaeskeygenassist xmm1, [edx], 3
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

	vaddpd xmm9, xmm10, xmm11
	vaddpd xmm9, xmm10, [r11]
	vaddpd ymm9, ymm10, ymm11
	vaddpd ymm9, ymm10, [r11]
	vaddps xmm9, xmm10, xmm11
	vaddps xmm9, xmm10, [r11]
	vaddps ymm9, ymm10, ymm11
	vaddps ymm9, ymm10, [r11]
	vaddsd xmm9, xmm10, [r11]
	vaddss xmm9, xmm10, [r11]
	vaddsubpd xmm9, xmm10, xmm11
	vaddsubpd xmm9, xmm10, [r11]
	vaddsubpd ymm9, ymm10, ymm11
	vaddsubpd ymm9, ymm10, [r11]
	vaddsubps xmm9, xmm10, xmm11
	vaddsubps xmm9, xmm10, [r11]
	vaddsubps ymm9, ymm10, ymm11
	vaddsubps ymm9, ymm10, [r11]
	vaesenc xmm9, xmm10, xmm11
	vaesenc xmm9, xmm10, [r11]
	vaesenclast xmm9, xmm10, xmm11
	vaesenclast xmm9, xmm10, [r11]
	vaesdec xmm9, xmm10, xmm11
	vaesdec xmm9, xmm10, [r11]
	vaesdeclast xmm9, xmm10, xmm11
	vaesdeclast xmm9, xmm10, [r11]
	vaesimc xmm9, xmm10
	vaesimc xmm9, [r11]
	vaeskeygenassist xmm9, xmm10, 3
	vaeskeygenassist xmm9, [r11], 3
	vandpd xmm9, xmm10, xmm11
	vandpd xmm9, xmm10, [r11]
	vandpd ymm9, ymm10, ymm11
	vandpd ymm9, ymm10, [r11]
	vandps xmm9, xmm10, xmm11
	vandps xmm9, xmm10, [r11]
	vandps ymm9, ymm10, ymm11
	vandps ymm9, ymm10, [r11]
	vandnpd xmm9, xmm10, xmm11
	vandnpd xmm9, xmm10, [r11]
	vandnpd ymm9, ymm10, ymm11
	vandnpd ymm9, ymm10, [r11]
	vandnps xmm9, xmm10, xmm11
	vandnps xmm9, xmm10, [r11]
	vandnps ymm9, ymm10, ymm11
	vandnps ymm9, ymm10, [r11]

global	nasm_test_avx_b
nasm_test_avx_b:
	vblendpd xmm1, xmm2, xmm2, 3
	vblendpd xmm1, xmm2, [edx], 3
	vblendpd ymm1, ymm2, ymm2, 3
	vblendpd ymm1, ymm2, [edx], 3
	vblendps xmm1, xmm2, xmm2, 3
	vblendps xmm1, xmm2, [edx], 3
	vblendps ymm1, ymm2, ymm2, 3
	vblendps ymm1, ymm2, [edx], 3
	vblendvpd xmm1, xmm2, xmm2, xmm3
	vblendvpd xmm1, xmm2, [edx], xmm3
	vblendvpd ymm1, ymm2, ymm2, ymm3
	vblendvpd ymm1, ymm2, [edx], ymm3
	vblendvps xmm1, xmm2, xmm2, xmm3
	vblendvps xmm1, xmm2, [edx], xmm3
	vblendvps ymm1, ymm2, ymm2, ymm3
	vblendvps ymm1, ymm2, [edx], ymm3
	vbroadcastss xmm1, [edx]
	vbroadcastss ymm1, [edx]
	vbroadcastsd ymm1, [edx]
	vbroadcastf128 ymm1, [edx]
	vcmppd xmm1, xmm2, xmm2, 3
	vcmppd xmm1, xmm2, [edx], 3
	vcmppd ymm1, ymm2, ymm2, 3
	vcmppd ymm1, ymm2, [edx], 3
	vcmpps xmm1, xmm2, xmm2, 3
	vcmpps xmm1, xmm2, [edx], 3
	vcmpps ymm1, ymm2, ymm2, 3
	vcmpps ymm1, ymm2, [edx], 3
	vcmpsd xmm1, xmm2, xmm2, 3
	vcmpsd xmm1, xmm2, [edx], 3
	vcmpss xmm1, xmm2, xmm2, 3
	vcmpss xmm1, xmm2, [edx], 3
	vcomisd xmm1, xmm2
	vcomisd xmm1, [edx]
	vcomiss xmm1, xmm2
	vcomiss xmm1, [edx]
	vcvtdq2pd xmm1, xmm2
	vcvtdq2pd xmm1, [edx]
	vcvtdq2pd ymm1, xmm2
	vcvtdq2pd ymm1, [edx]
	vcvtdq2ps xmm1, xmm2
	vcvtdq2ps xmm1, [edx]
	vcvtdq2ps ymm1, ymm2
	vcvtdq2ps ymm1, [edx]
	vcvtpd2dq xmm1, xmm2
	vcvtpd2dq xmm1, oword [edx]
	vcvtpd2dq xmm1, ymm2
	vcvtpd2dq xmm1, yword [edx]
	vcvtpd2ps xmm1, xmm2
	vcvtpd2ps xmm1, oword [edx]
	vcvtpd2ps xmm1, ymm2
	vcvtpd2ps xmm1, yword [edx]
	vcvtps2dq xmm1, xmm2
	vcvtps2dq xmm1, [edx]
	vcvtps2dq ymm1, ymm2
	vcvtps2dq ymm1, [edx]
	vcvtps2pd xmm1, xmm2
	vcvtps2pd xmm1, [edx]
	vcvtps2pd ymm1, xmm2
	vcvtps2pd ymm1, [edx]
	vcvtsd2si esp, xmm2
	vcvtsd2si esp, [edx]
	vcvtsd2si rsp, xmm2
	vcvtsd2si rsp, [edx]
	vcvtsd2ss xmm1, xmm2, xmm2
	vcvtsd2ss xmm1, xmm2, [edx]
	vcvtsi2sd xmm1, xmm2, ebx
	vcvtsi2sd xmm1, xmm2, dword [edx]
	vcvtsi2sd xmm1, xmm2, rbx
	vcvtsi2sd xmm1, xmm2, qword [edx]
	vcvtsi2ss xmm1, xmm2, ebx
	vcvtsi2ss xmm1, xmm2, dword [edx]
	vcvtsi2ss xmm1, xmm2, rbx
	vcvtsi2ss xmm1, xmm2, qword [edx]
	vcvtss2sd xmm1, xmm2, xmm2
	vcvtss2sd xmm1, xmm2, [edx]
	vcvtss2si ecx, xmm2
	vcvtss2si ecx, [edx]
	vcvtss2si rcx, xmm2
	vcvtss2si rcx, [edx]
	vcvttpd2dq xmm1, xmm2
	vcvttpd2dq xmm1, oword [edx]
	vcvttpd2dq xmm1, ymm2
	vcvttpd2dq xmm1, yword [edx]
	vcvttps2dq xmm1, xmm2
	vcvttps2dq xmm1, [edx]
	vcvttps2dq ymm1, ymm2
	vcvttps2dq ymm1, [edx]
	vcvttsd2si ecx, xmm2
	vcvttsd2si ecx, [edx]
	vcvttsd2si rcx, xmm2
	vcvttsd2si rcx, [edx]
	vcvttss2si ecx, xmm2
	vcvttss2si ecx, [edx]
	vcvttss2si rcx, xmm2
	vcvttss2si rcx, [edx]

global	nasm_test_avx_d
nasm_test_avx_d:
	vdivpd xmm1, xmm2, xmm3
	vdivpd xmm1, xmm2, oword [edx]
	vdivpd ymm1, ymm2, ymm3
	vdivpd ymm1, ymm2, yword [edx]
	vdivps xmm1, xmm2, xmm3
	vdivps xmm1, xmm2, oword [edx]
	vdivps ymm1, ymm2, ymm3
	vdivps ymm1, ymm2, yword [edx]
	vdivsd xmm1, xmm2, xmm3
	vdivsd xmm1, xmm2, [edx]
	vdivss xmm1, xmm2, xmm3
	vdivss xmm1, xmm2, [edx]
	vdppd xmm1, xmm2, xmm3, 5
	vdppd xmm1, xmm2, oword [edx], 5
	vdpps xmm1, xmm2, xmm3, 5
	vdpps xmm1, xmm2, oword [edx], 5
	vdpps ymm1, ymm2, ymm3, 5
	vdpps ymm1, ymm2, yword [edx], 5
	vextractf128 xmm1, xmm2, 1			; vextractf128 xmm1, ymm2, 1
	vextractf128 oword [edx], xmm2, 1	; vextractf128 oword [edx], ymm2, 1
	vextractps eax, xmm2, 5
	vextractps [eax], xmm2, 5
	vhaddpd xmm1, xmm2, xmm3
	vhaddpd xmm1, xmm2, oword [edx]
	vhaddpd ymm1, ymm2, ymm3
	vhaddpd ymm1, ymm2, yword [edx]
	vhaddps xmm1, xmm2, xmm3
	vhaddps xmm1, xmm2, oword [edx]
	vhaddps ymm1, ymm2, ymm3
	vhaddps ymm1, ymm2, yword [edx]
	vhsubpd xmm1, xmm2, xmm3
	vhsubpd xmm1, xmm2, oword [edx]
	vhsubpd ymm1, ymm2, ymm3
	vhsubpd ymm1, ymm2, yword [edx]
	vhsubps xmm1, xmm2, xmm3
	vhsubps xmm1, xmm2, oword [edx]
	vhsubps ymm1, ymm2, ymm3
	vhsubps ymm1, ymm2, yword [edx]
	vinsertf128 ymm1, ymm2, xmm3, 1
	vinsertf128 ymm1, ymm2, oword [edx], 1
	vinsertps xmm1, xmm2, xmm3, 1
	vinsertps xmm1, xmm2, [edx], 1
	vlddqu xmm1, oword [edx]
	vlddqu ymm1, yword [edx]
	vldmxcsr [edx]


global	nasm_test_avx_r
nasm_test_avx_r:
	vzeroall
	vzeroupper
