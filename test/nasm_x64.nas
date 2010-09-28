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
	vlddqu ymm1, yword [edx]
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
	vmovd xmm1, edx
	vmovd xmm1, [edx]
	vmovd edx, xmm2
	vmovd [edx], xmm2
	vmovq xmm1, rdx
	vmovq xmm1, [rdx]
	vmovq rdx, xmm2
	vmovq [rdx], xmm2

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
	vpcmpestri xmm2, xmm1, 0
	vpcmpestri xmm2, oword [esi], 0
	vpcmpestrm xmm2, xmm1, 1
	vpcmpestrm xmm2, oword [esi], 1
	vpcmpistri xmm2, xmm1, 0
	vpcmpistri xmm2, oword [esi], 0
	vpcmpistrm xmm2, xmm1, 1
	vpcmpistrm xmm2, oword [esi], 1
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

global	nasm_test_avx_r
nasm_test_avx_r:
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
