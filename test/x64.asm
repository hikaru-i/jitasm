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
	test rax, 1
	test rax, r8
	test qword ptr[eax], 1
	test qword ptr[eax], r8
	test r8, 1
	test r8, rax
	test qword ptr[r8], r9
	test qword ptr[r8], r9

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
	add rax, 1
	add r8, 1
	add rax, 100h
	add rax, 10000h
	add rax, r8
	add r8, rax
	add rax, qword ptr[r8]
	add qword ptr[rax], r8

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
	push rax
	push 1
	push 100h
	push word ptr[eax]
	push qword ptr[eax]
	push qword ptr[rax]
	pop ax
	pop rax
	pop word ptr[eax]
	pop qword ptr[eax]
	pop qword ptr[rax]

	lea ax, word ptr[eax]
	lea eax, dword ptr[eax]
	lea eax, dword ptr[eax + 1]
	lea eax, dword ptr[ecx * 4]
	lea eax, dword ptr[ecx * 4 + 1]
	lea eax, dword ptr[eax + ecx]
	lea eax, dword ptr[eax + ecx * 4]
	lea eax, dword ptr[eax + ecx * 4 + 1]
	lea ax, word ptr[rax]
	lea eax, dword ptr[rax]
	lea eax, dword ptr[rax + 1]
	lea eax, dword ptr[rcx * 4]
	lea eax, dword ptr[rcx * 4 + 1]
	lea eax, dword ptr[rax + rcx]
	lea eax, dword ptr[rax + rcx * 4]
	lea eax, dword ptr[rax + rcx * 4 + 1]

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
	mov rax, rcx
	mov qword ptr[rax], rcx
	mov rax, qword ptr[rcx]
	mov rax, 1
	mov rax, -1
	mov rax, 10000000h
	mov rax, 80000000h
	mov rax, 100000000h
	mov rax, 800000000h

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
	movzx rax, cl
	movzx rax, byte ptr[rcx]
	movzx rax, cx
	movzx rax, word ptr[rcx]
	movzx r8, cl
	movzx r8, byte ptr[rcx]
	movzx r8, cx
	movzx r8, word ptr[rcx]
	xchg rax, r8
	xchg rax, qword ptr[r8]
	xchg qword ptr[rax], r8
	xchg r8, rax
	xchg r8, qword ptr[rax]
	xchg qword ptr[r8], rax

	movdqa xmm0, xmm1
	movdqa xmm8, xmm1
	movdqa xmm0, xmm9
	movdqa xmm0, xmmword ptr[ecx]
	movdqa xmm0, xmmword ptr[rcx]
	movdqa xmm8, xmmword ptr[ecx]
	movdqa xmm8, xmmword ptr[rcx]
	movdqa xmmword ptr[eax], xmm1
	movdqa xmmword ptr[rax], xmm1
	movdqa xmmword ptr[eax], xmm9
	movdqa xmmword ptr[rax], xmm9

	movdqu xmm0, xmm1
	movdqu xmm8, xmm1
	movdqu xmm0, xmm9
	movdqu xmm0, xmmword ptr[ecx]
	movdqu xmm0, xmmword ptr[rcx]
	movdqu xmm8, xmmword ptr[ecx]
	movdqu xmm8, xmmword ptr[rcx]
	movdqu xmmword ptr[eax], xmm1
	movdqu xmmword ptr[rax], xmm1
	movdqu xmmword ptr[eax], xmm9
	movdqu xmmword ptr[rax], xmm9

	pabsb xmm0, xmm1
	pabsb xmm0, xmmword ptr[rcx]
	pabsw xmm0, xmm1
	pabsw xmm0, xmmword ptr[rcx]
	pabsd xmm0, xmm1
	pabsd xmm0, xmmword ptr[rcx]

	packsswb xmm0, xmm1
	packsswb xmm0, xmmword ptr[rcx]
	packssdw xmm0, xmm1
	packssdw xmm0, xmmword ptr[rcx]
	packuswb xmm0, xmm1
	packuswb xmm0, xmmword ptr[rcx]
	packusdw xmm0, xmm1
	packusdw xmm0, xmmword ptr[rcx]

	paddb xmm0, xmm1
	paddb xmm0, xmmword ptr[rcx]
	paddw xmm0, xmm1
	paddw xmm0, xmmword ptr[rcx]
	paddd xmm0, xmm1
	paddd xmm0, xmmword ptr[rcx]

	pxor xmm0, xmm1
	pxor xmm8, xmm1
	pxor xmm0, xmm9
	pxor xmm0, xmmword ptr[ecx]
	pxor xmm0, xmmword ptr[rcx]
	pxor xmm8, xmmword ptr[ecx]
	pxor xmm8, xmmword ptr[rcx]

	xchg rax, rcx
	xchg rcx, rax
	xchg r8, rcx
	xchg rcx, r8

hoge ENDP

hoge2	proc USES r12 rsi rdi rbx, pDst: PTR, pSrc: PTR, nLen: SDWORD
	LOCAL a:SDWORD
	mov rsi, pSrc
	mov rdi, pDst
	mov rsi, qword ptr pSrc
	mov rdi, qword ptr pDst
	ret
hoge2	endp

masm_test_shift	proc
	sal al, 1
	sal al, 2
	sal al, -1
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
	sal r8b, 1
	sal r8b, 2
	sal r8w, 1
	sal r8w, 2
	sal r8d, 1
	sal r8d, 2
	sal r8, 1
	sal r8, 2
	sal qword ptr[rax], 1
	sal qword ptr[rax], 2
masm_test_shift	endp

end
