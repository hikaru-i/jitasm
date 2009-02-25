.686
.model flat, c
.xmm

.code

hoge PROC

	test al, 1
	test cl, 1
	test ax, 1
	test cx, 1
	test eax, 1
	test ecx, 1
	test al, cl
	test ax, cx
	test eax, ecx
	test byte ptr[eax], 1
	test word ptr[eax], 1
	test dword ptr[eax], 1
	test byte ptr[eax], cl
	test word ptr[eax], cx
	test dword ptr[eax], ecx

	add al, 1
	add ax, 1
	add eax, 1
	add ax, 100h
	add eax, 10000h
	add eax, ecx
	add ecx, eax
	add eax, dword ptr[ecx]
	add dword ptr[eax], ecx
	or eax, ecx
	adc eax, ecx
	sbb eax, ecx
	and eax, ecx
	sub eax, ecx
	xor eax, ecx
	cmp eax, ecx

	inc al
	inc ax
	inc eax
	inc word ptr[eax]
	inc dword ptr[eax]
	dec al
	dec ax
	dec eax
	dec word ptr[eax]
	dec dword ptr[eax]

	push ax
	push eax
	push 1
	push 100h
	push word ptr[eax]
	push dword ptr[eax]
	pop ax
	pop eax
	pop word ptr[eax]
	pop dword ptr[eax]

	lea ax, word ptr[eax]
	lea eax, dword ptr[eax]
	lea eax, dword ptr[eax + 1]
	lea eax, dword ptr[ecx * 4]
	lea eax, dword ptr[ecx * 4 + 1]
	lea eax, dword ptr[eax + ecx]
	lea eax, dword ptr[eax + ecx * 4]
	lea eax, dword ptr[eax + ecx * 4 + 1]

	lea	ax, word ptr[eax]
	lea	eax, dword ptr[eax]
	lea	eax, dword ptr[eax + ecx * 4 + 100h]

	mov al, cl
	mov byte ptr[eax], cl
	mov al, byte ptr[ecx]
	mov al, -1
	mov ax, cx
	mov word ptr[eax], cx
	mov ax, word ptr[ecx]
	mov ax, -1
	mov eax, ecx
	mov dword ptr[eax], ecx
	mov eax, dword ptr[ecx]
	mov eax, -1

	movzx ax, cl
	movzx ax, byte ptr[ecx]
	movzx eax, ch
	movzx eax, byte ptr[ecx]
	movzx eax, cx
	movzx eax, word ptr[ecx]
	xchg al, cl
	xchg cl, al
	xchg al, byte ptr[ecx]
	xchg byte ptr[eax], cl
	xchg ax, cx
	xchg cx, ax
	xchg ax, word ptr[ecx]
	xchg word ptr[eax], cx
	xchg eax, ecx
	xchg ecx, eax
	xchg eax, dword ptr[ecx]
	xchg dword ptr[eax], ecx

	movdqa xmm0, xmm1
	movdqa xmm0, xmmword ptr[ecx]
	movdqa xmmword ptr[eax], xmm1

	movdqu xmm0, xmm1
	movdqu xmm0, xmmword ptr[ecx]
	movdqu xmmword ptr[eax], xmm1

	pabsb mm0, mm1
	pabsb mm0, qword ptr[ecx]
	pabsb xmm0, xmm1
	pabsb xmm0, xmmword ptr[ecx]
	pabsw mm0, mm1
	pabsw mm0, qword ptr[ecx]
	pabsw xmm0, xmm1
	pabsw xmm0, xmmword ptr[ecx]
	pabsd mm0, mm1
	pabsd mm0, qword ptr[ecx]
	pabsd xmm0, xmm1
	pabsd xmm0, xmmword ptr[ecx]

	packsswb mm0, mm1
	packsswb mm0, qword ptr[ecx]
	packsswb xmm0, xmm1
	packsswb xmm0, xmmword ptr[ecx]
	packssdw mm0, mm1
	packssdw mm0, qword ptr[ecx]
	packssdw xmm0, xmm1
	packssdw xmm0, xmmword ptr[ecx]
	packuswb mm0, mm1
	packuswb mm0, qword ptr[ecx]
	packuswb xmm0, xmm1
	packuswb xmm0, xmmword ptr[ecx]
	packusdw xmm0, xmm1
	packusdw xmm0, xmmword ptr[ecx]

	paddb mm0, mm1
	paddb mm0, qword ptr[ecx]
	paddb xmm0, xmm1
	paddb xmm0, xmmword ptr[ecx]
	paddw mm0, mm1
	paddw mm0, qword ptr[ecx]
	paddw xmm0, xmm1
	paddw xmm0, xmmword ptr[ecx]
	paddd mm0, mm1
	paddd mm0, qword ptr[ecx]
	paddd xmm0, xmm1
	paddd xmm0, xmmword ptr[ecx]

	pxor mm0, mm1
	pxor mm0, qword ptr[ecx]
	pxor xmm0, xmm1
	pxor xmm0, xmmword ptr[ecx]

hoge ENDP

hoge2	proc C USES esi edi ebx, pDst: PTR, pSrc: PTR, nLen: SDWORD
	LOCAL a:SDWORD
	mov esi, pSrc
	mov edi, pDst
	mov esi, [pSrc]
	mov edi, [pDst]
	ret
hoge2	endp

masm_test_shift	proc
	sal al, 1
	sal al, 2
	sal ax, 1
	sal ax, 2
	sal eax, 1
	sal eax, 2
	sal byte ptr[eax], 1
	sal byte ptr[eax], 2
	sal word ptr[eax], 1
	sal word ptr[eax], 2
	sal dword ptr[eax], 1
	sal dword ptr[eax], 2
	sar al, 1
	sar al, 2
	shl al, 1
	shl al, 2
	shr al, 1
	shr al, 2
masm_test_shift endp

end
