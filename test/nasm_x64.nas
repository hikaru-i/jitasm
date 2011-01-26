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
	vaddsd xmm1, xmm2, xmm4
	vaddsd xmm1, xmm2, [edx]
	vaddss xmm1, xmm2, xmm5
	vaddss xmm1, xmm2, [edx]
	vaddsubpd xmm1, xmm2, xmm3
	vaddsubpd xmm1, xmm2, [edx]
	vaddsubpd ymm1, ymm2, ymm3
	vaddsubpd ymm1, ymm2, [edx]
	vaddsubps xmm1, xmm2, xmm3
	vaddsubps xmm1, xmm2, [edx]
	vaddsubps ymm1, ymm2, ymm3
	vaddsubps ymm1, ymm2, [edx]
	aesenc xmm1, xmm2
	aesenc xmm1, [rdx]
	vaesenc xmm1, xmm2, xmm3
	vaesenc xmm1, xmm2, [edx]
	aesenclast xmm1, xmm2
	aesenclast xmm1, [rdx]
	vaesenclast xmm1, xmm2, xmm3
	vaesenclast xmm1, xmm2, [edx]
	aesdec xmm1, xmm2
	aesdec xmm1, [rdx]
	vaesdec xmm1, xmm2, xmm3
	vaesdec xmm1, xmm2, [edx]
	aesdeclast xmm1, xmm2
	aesdeclast xmm1, [rdx]
	vaesdeclast xmm1, xmm2, xmm3
	vaesdeclast xmm1, xmm2, [edx]
	aesimc xmm1, xmm2
	aesimc xmm1, [rdx]
	vaesimc xmm1, xmm2
	vaesimc xmm1, [edx]
	aeskeygenassist xmm1, xmm2, 3
	aeskeygenassist xmm1, [rdx], 3
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
	db 0x67 ;vlddqu ymm1, yword [edx]
	db 0xC5
	db 0xFF
	db 0xF0
	db 0x0A
	vldmxcsr [edx]
	vmaskmovdqu xmm1, xmm2
	vmaskmovps xmm1, xmm2, oword [edx]
	vmaskmovps ymm1, ymm2, yword [edx]
	vmaskmovpd xmm1, xmm2, oword [edx]
	vmaskmovpd ymm1, ymm2, yword [edx]
	vmaskmovps oword [edx], xmm2, xmm3
	db 0x67	;vmaskmovps yword [edx], ymm2, ymm3
	db 0xC4
	db 0xE2
	db 0x6D
	db 0x2E
	db 0x1A
	vmaskmovpd oword [edx], xmm2, xmm3
	vmaskmovpd yword [edx], ymm2, ymm3
	vmaxpd xmm1, xmm2, xmm3
	vmaxpd xmm1, xmm2, oword [edx]
	vmaxpd ymm1, ymm2, ymm3
	vmaxpd ymm1, ymm2, yword [edx]
	vmaxps xmm1, xmm2, xmm3
	vmaxps xmm1, xmm2, oword [edx]
	vmaxps ymm1, ymm2, ymm3
	vmaxps ymm1, ymm2, yword [edx]
	vmaxsd xmm1, xmm2, xmm3
	vmaxsd xmm1, xmm2, qword [edx]
	vmaxss xmm1, xmm2, xmm3
	vmaxss xmm1, xmm2, dword [edx]
	vminpd xmm1, xmm2, xmm3
	vminpd xmm1, xmm2, oword [edx]
	vminpd ymm1, ymm2, ymm3
	vminpd ymm1, ymm2, yword [edx]
	vminps xmm1, xmm2, xmm3
	vminps xmm1, xmm2, oword [edx]
	vminps ymm1, ymm2, ymm3
	vminps ymm1, ymm2, yword [edx]
	vminsd xmm1, xmm2, xmm3
	vminsd xmm1, xmm2, qword [edx]
	vminss xmm1, xmm2, xmm3
	vminss xmm1, xmm2, dword [edx]
	vmovapd xmm1, xmm2
	vmovapd xmm1, oword [edx]
	vmovapd oword [edx], xmm2
	vmovapd ymm1, ymm2
	vmovapd ymm1, yword [edx]
	vmovapd yword [edx], ymm2
	vmovaps xmm1, xmm2
	vmovaps xmm1, oword [edx]
	vmovaps oword [edx], xmm2
	vmovaps ymm1, ymm2
	vmovaps ymm1, yword [edx]
	vmovaps yword [edx], ymm2
	vmovd xmm3, edx
	vmovd xmm3, dword [edx]
	vmovd eax, xmm4
	vmovd dword [eax], xmm4
	vmovq xmm3, xmm4
	vmovq xmm3, qword [edx]
	vmovq qword [eax], xmm4
	vmovq xmm3, rdx
	vmovq rax, xmm4
	vmovddup xmm3, xmm4
	vmovddup xmm3, qword [edx]
	vmovddup ymm3, ymm4
	vmovddup ymm3, yword [edx]
	vmovdqa xmm3, xmm4
	vmovdqa xmm3, oword [edx]
	vmovdqa oword [eax], xmm4
	vmovdqa ymm3, ymm4
	vmovdqa ymm3, [edx]
	vmovdqa yword [eax], ymm4
	vmovdqu xmm3, xmm4
	vmovdqu xmm3, oword [edx]
	vmovdqu oword [eax], xmm4
	vmovdqu ymm3, ymm4
	vmovdqu ymm3, yword [edx]
	vmovdqu yword [eax], ymm4
	vmovhlps xmm3, xmm4, xmm5
	vmovhpd xmm3, xmm4, qword [edx]
	vmovhpd qword [eax], xmm4
	vmovhps xmm3, xmm4, qword [edx]
	vmovhps qword [eax], xmm4
	vmovlhps xmm3, xmm4, xmm5
	vmovlpd xmm3, xmm4, qword [edx]
	vmovlpd qword [eax], xmm4
	vmovlps xmm3, xmm4, qword [edx]
	vmovlps qword [eax], xmm4
	vmovmskpd eax, xmm5
	vmovmskpd eax, ymm5
	vmovmskpd rax, xmm5
	vmovmskpd rax, ymm5
	vmovmskps eax, xmm5
	vmovmskps eax, ymm5
	vmovmskps rax, xmm5
	vmovmskps rax, ymm5
	vmovntdq oword [eax], xmm5
	vmovntdq yword [eax], ymm5
	vmovntdqa xmm3, oword [edi]
	vmovntpd oword [eax], xmm5
	vmovntpd yword [eax], ymm5
	vmovntps oword [eax], xmm5
	db 0x67 ; vmovntps yword [eax], ymm5
	db 0xC5
	db 0xFC
	db 0x2B
	db 0x28
	vmovsd xmm3, xmm5, xmm6
	vmovsd xmm3, qword [edi]
	vmovsd qword [eax], xmm5
	vmovshdup xmm3, xmm5
	vmovshdup xmm3, oword [edi]
	vmovshdup ymm3, ymm5
	vmovshdup ymm3, yword [edi]
	vmovsldup xmm3, xmm5
	vmovsldup xmm3, oword [edi]
	vmovsldup ymm3, ymm5
	vmovsldup ymm3, yword [edi]
	vmovss xmm3, xmm5, xmm6
	vmovss xmm3, [edi]
	vmovss [eax], xmm5
	vmovupd xmm3, xmm5
	vmovupd xmm3, oword [edi]
	vmovupd oword [eax], xmm3
	vmovupd ymm3, ymm5
	vmovupd ymm3, yword [edi]
	vmovupd yword [eax], ymm3
	vmovups xmm3, xmm5
	vmovups xmm3, oword [edi]
	vmovups oword [eax], xmm3
	vmovups ymm3, ymm5
	vmovups ymm3, yword [edi]
	vmovups yword [eax], ymm3
	vmpsadbw xmm3, xmm5, xmm6, 2
	vmpsadbw xmm3, xmm5, oword [edi], 2
	vmulpd xmm3, xmm5, xmm6
	vmulpd xmm3, xmm5, oword [edi]
	vmulpd ymm3, ymm5, ymm6
	vmulpd ymm3, ymm5, yword [edi]
	vmulps xmm3, xmm5, xmm6
	vmulps xmm3, xmm5, oword [edi]
	vmulps ymm3, ymm5, ymm6
	vmulps ymm3, ymm5, yword [edi]
	vmulsd xmm3, xmm5, xmm6
	vmulsd xmm3, xmm5, [edi]
	vmulss xmm3, xmm5, xmm6
	vmulss xmm3, xmm5, [edi]

global	nasm_test_avx_o
nasm_test_avx_o:
	vorpd xmm1, xmm2, xmm3
	vorpd xmm1, xmm2, oword [edx]
	vorpd ymm1, ymm2, ymm3
	vorpd ymm1, ymm2, yword [edx]
	vorps xmm1, xmm2, xmm3
	vorps xmm1, xmm2, oword [edx]
	vorps ymm1, ymm2, ymm3
	vorps ymm1, ymm2, yword [edx]
	vpabsb xmm1, xmm2
	vpabsb xmm1, oword [edx]
	vpabsw xmm1, xmm2
	vpabsw xmm1, oword [edx]
	vpabsd xmm1, xmm2
	vpabsd xmm1, oword [edx]
	vpacksswb xmm1, xmm2, xmm3
	vpacksswb xmm1, xmm2, oword [edx]
	vpackssdw xmm1, xmm2, xmm3
	vpackssdw xmm1, xmm2, oword [edx]
	vpackuswb xmm1, xmm2, xmm3
	vpackuswb xmm1, xmm2, oword [edx]
	vpackusdw xmm1, xmm2, xmm3
	vpackusdw xmm1, xmm2, oword [edx]
	vpaddb xmm1, xmm2, xmm3
	vpaddb xmm1, xmm2, oword [edx]
	vpaddw xmm1, xmm2, xmm3
	vpaddw xmm1, xmm2, oword [edx]
	vpaddd xmm1, xmm2, xmm3
	vpaddd xmm1, xmm2, oword [edx]
	vpaddq xmm1, xmm2, xmm3
	vpaddq xmm1, xmm2, oword [edx]
	vpaddsb xmm1, xmm2, xmm3
	vpaddsb xmm1, xmm2, oword [edx]
	vpaddsw xmm1, xmm2, xmm3
	vpaddsw xmm1, xmm2, oword [edx]
	vpaddusb xmm1, xmm2, xmm3
	vpaddusb xmm1, xmm2, oword [edx]
	vpaddusw xmm1, xmm2, xmm3
	vpaddusw xmm1, xmm2, oword [edx]
	vpalignr xmm1, xmm2, xmm3, 1
	vpalignr xmm1, xmm2, oword [edx], 1
	vpand xmm1, xmm2, xmm3
	vpand xmm1, xmm2, oword [edx]
	vpandn xmm1, xmm2, xmm3
	vpandn xmm1, xmm2, oword [edx]
	vpavgb xmm1, xmm2, xmm3
	vpavgb xmm1, xmm2, oword [edx]
	vpavgw xmm1, xmm2, xmm3
	vpavgw xmm1, xmm2, oword [edx]
	vpblendvb xmm1, xmm2, oword [edx], xmm4
	vpblendvb xmm1, xmm2, xmm3, xmm4
	vpblendw xmm1, xmm2, oword [edx], 5
	vpblendw xmm1, xmm2, xmm3, 5
	pclmulqdq xmm1, oword [rdx], 1
	pclmulqdq xmm1, xmm2, 1
	vpclmulqdq xmm1, xmm2, oword [edx], 1
	vpclmulqdq xmm1, xmm2, xmm3, 1
	vpcmpeqb xmm0, xmm1, xmm2
	vpcmpeqb xmm0, xmm1, oword [esi]
	vpcmpeqw xmm0, xmm1, xmm2
	vpcmpeqw xmm0, xmm1, oword [esi]
	vpcmpeqd xmm0, xmm1, xmm2
	vpcmpeqd xmm0, xmm1, oword [esi]
	db 0xC4	;vpcmpeqq xmm0, xmm1, xmm2
	db 0xE2
	db 0x71
	db 0x29
	db 0xC2
	db 0x67	;vpcmpeqq xmm0, xmm1, oword [esi]
	db 0xC4
	db 0xE2
	db 0x71
	db 0x29
	db 0x06
	vpcmpgtb xmm0, xmm1, xmm2
	vpcmpgtb xmm0, xmm1, oword [esi]
	vpcmpgtw xmm0, xmm1, xmm2
	vpcmpgtw xmm0, xmm1, oword [esi]
	vpcmpgtd xmm0, xmm1, xmm2
	vpcmpgtd xmm0, xmm1, oword [esi]
	db 0xC4	;vpcmpgtq xmm0, xmm1, xmm2
	db 0xE2
	db 0x71
	db 0x37
	db 0xC2
	db 0x67	;vpcmpgtq xmm0, xmm1, oword [esi]
	db 0xC4
	db 0xE2
	db 0x71
	db 0x37
	db 0x06
	vpcmpestri xmm2, xmm1, 0
	vpcmpestri xmm2, oword [esi], 0
	vpcmpestrm xmm2, xmm1, 1
	vpcmpestrm xmm2, oword [esi], 1
	vpcmpistri xmm2, xmm1, 0
	vpcmpistri xmm2, oword [esi], 0
	vpcmpistrm xmm2, xmm1, 1
	vpcmpistrm xmm2, oword [esi], 1
	vpermilpd xmm1, xmm2, xmm3
	vpermilpd xmm1, xmm2, oword [rsi]
	vpermilpd ymm4, ymm5, ymm6
	vpermilpd ymm4, ymm5, yword [rsp]
	vpermilpd xmm1, xmm2, 1
	vpermilpd xmm1, oword [rsi], 2
	vpermilpd ymm4, ymm5, 3
	vpermilpd ymm4, yword [rsp], 4
	vpermilps xmm1, xmm2, xmm3
	vpermilps xmm1, xmm2, oword [esi]
	vpermilps ymm4, ymm5, ymm6
	vpermilps ymm4, ymm5, yword [rsp]
	vpermilps xmm1, xmm2, 3
	vpermilps xmm1, oword [esp], 2
	vpermilps ymm4, ymm5, 1
	vpermilps ymm4, yword [rsp], 1
	vperm2f128 ymm4, ymm5, ymm6, 1
	vperm2f128 ymm4, ymm5, yword [rsp], 2
	vpextrb ecx, xmm7, 13
	vpextrb byte[rsi], xmm7, 5
	vpextrw edx, xmm7, 6
	db 0xC4 ;vpextrw word[rsp], xmm7, 4
	db 0xE3 ;nasm-Bugs-3143040
	db 0x79
	db 0x15
	db 0x3C
	db 0x24
	db 0x04
	vpextrd eax, xmm7, 3
	vpextrd dword[esp], xmm7, 2
	vpextrb r9, xmm8, 12
	vpextrw rax, xmm9, 7
	vpextrd r10d, xmm3, 3
	vpextrq rcx, xmm10, 0
	vpextrq qword[r13], xmm2, 1
	vphaddw xmm7, xmm6, xmm5
	vphaddw xmm7, xmm6, oword[rsp]
	vphaddd xmm7, xmm6, xmm5
	vphaddd xmm7, xmm6, oword[rsp]
	vphaddsw xmm7, xmm6, xmm5
	vphaddsw xmm7, xmm6, oword[rsp]
	vphminposuw xmm7, xmm5
	vphminposuw xmm7, oword[rsp]
	vphsubw xmm7, xmm6, xmm5
	vphsubw xmm7, xmm6, oword[rsp]
	vphsubd xmm7, xmm6, xmm5
	vphsubd xmm7, xmm6, oword[rsp]
	vphsubsw xmm7, xmm6, xmm5
	vphsubsw xmm7, xmm6, oword[rsp]
	vpinsrb xmm2, xmm0, ebx, 15
	vpinsrb xmm2, xmm0, byte [rsi], 7
	vpinsrw xmm2, xmm0, ecx, 6
	vpinsrw xmm2, xmm0, word [rdi], 5
	vpinsrd xmm2, xmm1, eax, 3
	vpinsrd xmm2, xmm0, dword [esp], 2
	vpinsrb xmm7, xmm0, r9d, 13
	vpinsrw xmm9, xmm0, r10d, 4
	vpinsrd xmm10, xmm0, r11d, 1
	vpinsrq xmm11, xmm0, rcx, 1
	vpinsrq xmm12, xmm0, qword [r12], 0
	vpmaddwd xmm2, xmm1, xmm6
	vpmaddwd xmm2, xmm1, oword [esi]
	vpmaddubsw xmm2, xmm1, xmm5
	vpmaddubsw xmm2, xmm1, oword[rsi]
	vpmaxsb xmm6, xmm5, xmm4
	vpmaxsb xmm6, xmm5, oword [rsi]
	vpmaxsw xmm6, xmm5, xmm4
	vpmaxsw xmm6, xmm5, oword [rsi]
	vpmaxsd xmm6, xmm5, xmm4
	vpmaxsd xmm6, xmm5, oword [rsi]
	vpmaxub xmm6, xmm5, xmm4
	vpmaxub xmm6, xmm5, oword [rsi]
	vpmaxuw xmm6, xmm5, xmm4
	vpmaxuw xmm6, xmm5, oword [rsi]
	vpmaxud xmm6, xmm5, xmm4
	vpmaxud xmm6, xmm5, oword [rsi]
	vpminsb xmm6, xmm5, xmm4
	vpminsb xmm6, xmm5, oword [rsi]
	vpminsw xmm6, xmm5, xmm4
	vpminsw xmm6, xmm5, oword [rsi]
	vpminsd xmm6, xmm5, xmm4
	vpminsd xmm6, xmm5, oword [rsi]
	vpminub xmm6, xmm5, xmm4
	vpminub xmm6, xmm5, oword [rsi]
	vpminuw xmm6, xmm5, xmm4
	vpminuw xmm6, xmm5, oword [rsi]
	vpminud xmm6, xmm5, xmm4
	vpminud xmm6, xmm5, oword [rsi]
	vpmovmskb ecx, xmm5
	vpmovmskb rax, xmm5
	vpmovsxbw xmm1, xmm2
	vpmovsxbw xmm1, qword [esp]
	vpmovsxbd xmm1, xmm2
	vpmovsxbd xmm1, dword [esp]
	vpmovsxbq xmm1, xmm2
	vpmovsxbq xmm1, word [esp]
	vpmovsxwd xmm1, xmm2
	vpmovsxwd xmm1, qword [esp]
	vpmovsxwq xmm1, xmm2
	vpmovsxwq xmm1, dword [esp]
	vpmovsxdq xmm1, xmm2
	vpmovsxdq xmm1, qword [esp]
	vpmovzxbw xmm1, xmm2
	vpmovzxbw xmm1, qword [esp]
	vpmovzxbd xmm1, xmm2
	vpmovzxbd xmm1, dword [esp]
	vpmovzxbq xmm1, xmm2
	vpmovzxbq xmm1, word [esp]
	vpmovzxwd xmm1, xmm2
	vpmovzxwd xmm1, qword [esp]
	vpmovzxwq xmm1, xmm2
	vpmovzxwq xmm1, dword [esp]
	vpmovzxdq xmm1, xmm2
	vpmovzxdq xmm1, qword [esp]
	vpmulhuw xmm2, xmm3, xmm4
	vpmulhuw xmm2, xmm3, [rsi]
	vpmulhrsw xmm2, xmm3, xmm4
	vpmulhrsw xmm2, xmm3, [rsi]
	vpmulhw xmm2, xmm3, xmm4
	vpmulhw xmm2, xmm3, [rsi]
	vpmullw xmm2, xmm3, xmm4
	vpmullw xmm2, xmm3, [rsi]
	vpmulld xmm2, xmm3, xmm4
	vpmulld xmm2, xmm3, [rsi]
	vpmuludq xmm2, xmm3, xmm4
	vpmuludq xmm2, xmm3, [rsi]
	vpmuldq xmm2, xmm3, xmm4
	vpmuldq xmm2, xmm3, [rsi]
	vpor xmm2, xmm3, xmm4
	vpor xmm2, xmm3, [rsi]
	vpsadbw xmm2, xmm3, xmm4
	vpsadbw xmm2, xmm3, [rsi]
	vpshufb xmm2, xmm3, xmm4
	vpshufb xmm2, xmm3, [rsi]
	vpshufd xmm2, xmm3, 1
	vpshufd xmm2, [rsi], 2
	vpshufhw xmm2, xmm3, 3
	vpshufhw xmm2, [rsi], 4
	vpshuflw xmm2, xmm3, 5
	vpshuflw xmm2, [rsi], 6
	vpsignb xmm5, xmm0, xmm1
	vpsignb xmm5, xmm0, [rsi]
	vpsignw xmm5, xmm0, xmm1
	vpsignw xmm5, xmm0, [rsi]
	vpsignd xmm5, xmm0, xmm1
	vpsignd xmm5, xmm0, [rsi]
	vpsllw xmm7, xmm5, xmm2
	vpsllw xmm7, xmm5, [rsi]
	vpsllw xmm7, xmm5, 1
	vpslld xmm7, xmm5, xmm2
	vpslld xmm7, xmm5, [rsi]
	vpslld xmm7, xmm5, 1
	vpsllq xmm7, xmm5, xmm2
	vpsllq xmm7, xmm5, [rsi]
	vpsllq xmm7, xmm5, 1
	vpslldq xmm7, xmm5, 1
	vpsraw xmm7, xmm5, xmm2
	vpsraw xmm7, xmm5, [rsi]
	vpsraw xmm7, xmm5, 1
	vpsrad xmm7, xmm5, xmm2
	vpsrad xmm7, xmm5, [rsi]
	vpsrad xmm7, xmm5, 1
	vpsrlw xmm7, xmm5, xmm2
	vpsrlw xmm7, xmm5, [rsi]
	vpsrlw xmm7, xmm5, 1
	vpsrld xmm7, xmm5, xmm2
	vpsrld xmm7, xmm5, [rsi]
	vpsrld xmm7, xmm5, 1
	vpsrlq xmm7, xmm5, xmm2
	vpsrlq xmm7, xmm5, [rsi]
	vpsrlq xmm7, xmm5, 1
	vpsrldq xmm7, xmm5, 1
	vptest xmm1, xmm0
	vptest xmm1, [rsi]
	vptest ymm1, ymm0
	vptest ymm1, [rsi]
	vtestps xmm1, xmm0
	vtestps xmm1, [rsi]
	vtestps ymm1, ymm0
	vtestps ymm1, [rsi]
	vtestpd xmm1, xmm0
	vtestpd xmm1, [rsi]
	vtestpd ymm1, ymm0
	vtestpd ymm1, [rsi]
	vpsubb xmm2, xmm3, xmm7
	vpsubb xmm2, xmm3, [rsi]
	vpsubw xmm2, xmm3, xmm7
	vpsubw xmm2, xmm3, [rsi]
	vpsubd xmm2, xmm3, xmm7
	vpsubd xmm2, xmm3, [rsi]
	vpsubq xmm2, xmm3, xmm7
	vpsubq xmm2, xmm3, [rsi]
	vpsubsb xmm2, xmm3, xmm7
	vpsubsb xmm2, xmm3, [rsi]
	vpsubsw xmm2, xmm3, xmm7
	vpsubsw xmm2, xmm3, [rsi]
	vpsubusb xmm2, xmm3, xmm7
	vpsubusb xmm2, xmm3, [rsi]
	vpsubusw xmm2, xmm3, xmm7
	vpsubusw xmm2, xmm3, [rsi]
	vpunpckhbw xmm1, xmm2, xmm3
	vpunpckhbw xmm1, xmm2, [rsi]
	vpunpckhwd xmm1, xmm2, xmm3
	vpunpckhwd xmm1, xmm2, [rsi]
	vpunpckhdq xmm1, xmm2, xmm3
	vpunpckhdq xmm1, xmm2, [rsi]
	vpunpckhqdq xmm1, xmm2, xmm3
	vpunpckhqdq xmm1, xmm2, [rsi]
	vpunpcklbw xmm1, xmm2, xmm3
	vpunpcklbw xmm1, xmm2, [rsi]
	vpunpcklwd xmm1, xmm2, xmm3
	vpunpcklwd xmm1, xmm2, [rsi]
	vpunpckldq xmm1, xmm2, xmm3
	vpunpckldq xmm1, xmm2, [rsi]
	vpunpcklqdq xmm1, xmm2, xmm3
	vpunpcklqdq xmm1, xmm2, [rsi]
	vpxor xmm1, xmm2, xmm3
	vpxor xmm1, xmm2, [rsi]

global	nasm_test_avx_r
nasm_test_avx_r:
	vrcpps xmm5, xmm0
	vrcpps xmm5, [rsi]
	vrcpps ymm4, ymm0
	vrcpps ymm4, [rsi]
	vrcpss xmm5, xmm3, xmm0
	vrcpss xmm5, xmm3, [rsi]
	vrsqrtps xmm5, xmm0
	vrsqrtps xmm5, [rsi]
	vrsqrtps ymm4, ymm0
	vrsqrtps ymm4, [rsi]
	vrsqrtss xmm5, xmm3, xmm0
	vrsqrtss xmm5, xmm3, [rsi]
	vroundpd xmm1, xmm3, 1
	vroundpd xmm1, [rsi], 2
	vroundpd ymm1, ymm2, 1
	vroundpd ymm1, [rsi], 3
	vroundps xmm1, xmm3, 0
	vroundps xmm1, [rsi], 1
	vroundps ymm1, ymm2, 2
	vroundps ymm1, [rsi], 0
	vroundsd xmm1, xmm2, xmm3, 1
	vroundsd xmm1, xmm2, [rsi], 2
	vroundss xmm1, xmm2, xmm3, 3
	vroundss xmm1, xmm2, [rdx], 1
	vshufpd xmm1, xmm3, xmm4, 1
	vshufpd xmm1, xmm3, [esi], 2
	vshufpd ymm2, ymm0, ymm5, 3
	vshufpd ymm2, ymm0, [esi], 4
	vshufps xmm1, xmm3, xmm5, 5
	vshufps xmm1, xmm3, [esi], 6
	vshufps ymm2, ymm0, ymm6, 7
	vshufps ymm2, ymm0, [esi], 8
	vsqrtpd xmm7, xmm3
	vsqrtpd xmm7, [edx]
	vsqrtpd ymm7, ymm3
	vsqrtpd ymm7, [edx]
	vsqrtps xmm7, xmm3
	vsqrtps xmm7, [edx]
	vsqrtps ymm7, ymm3
	vsqrtps ymm7, [edx]
	vsqrtsd xmm7, xmm2, xmm4
	vsqrtsd xmm7, xmm2, [edx]
	vsqrtss xmm7, xmm2, xmm5
	vsqrtss xmm7, xmm2, [edx]
	vstmxcsr dword [esi]
	vsubpd xmm1, xmm2, xmm3
	vsubpd xmm1, xmm2, [edx]
	vsubpd ymm1, ymm2, ymm3
	vsubpd ymm1, ymm2, [edx]
	vsubps xmm1, xmm2, xmm3
	vsubps xmm1, xmm2, [edx]
	vsubps ymm1, ymm2, ymm3
	vsubps ymm1, ymm2, [edx]
	vsubsd xmm1, xmm2, xmm4
	vsubsd xmm1, xmm2, [edx]
	vsubss xmm1, xmm2, xmm5
	vsubss xmm1, xmm2, [edx]
	vucomisd xmm4, xmm6
	vucomisd xmm3, qword [ebp]
	vucomiss xmm0, xmm7
	vucomiss xmm1, dword [ebx]
	vunpckhpd xmm1, xmm2, xmm3
	vunpckhpd xmm1, xmm2, oword [edi]
	vunpckhpd ymm1, ymm2, ymm4
	vunpckhpd ymm1, ymm2, yword [esi]
	vunpckhps xmm1, xmm2, xmm3
	vunpckhps xmm1, xmm2, oword [edi]
	vunpckhps ymm1, ymm2, ymm4
	vunpckhps ymm1, ymm2, yword [esi]
	vunpcklpd xmm1, xmm2, xmm3
	vunpcklpd xmm1, xmm2, oword [edi]
	vunpcklpd ymm1, ymm2, ymm4
	vunpcklpd ymm1, ymm2, yword [esi]
	vunpcklps xmm1, xmm2, xmm3
	vunpcklps xmm1, xmm2, oword [edi]
	vunpcklps ymm1, ymm2, ymm4
	vunpcklps ymm1, ymm2, yword [esi]
	vxorpd xmm1, xmm2, xmm3
	vxorpd xmm1, xmm2, oword [edi]
	vxorpd ymm1, ymm2, ymm4
	vxorpd ymm1, ymm2, yword [esi]
	vxorps xmm1, xmm2, xmm3
	vxorps xmm1, xmm2, oword [edi]
	vxorps ymm1, ymm2, ymm4
	vxorps ymm1, ymm2, yword [esi]
	vzeroall
	vzeroupper
