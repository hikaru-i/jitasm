.code

hoge proc
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
hoge endp

hoge2 proc USES r12 rsi rdi rbx, pDst: PTR, pSrc: PTR, nLen: SDWORD
	LOCAL a:SDWORD
	mov rsi, pSrc
	mov rdi, pDst
	mov rsi, qword ptr pSrc
	mov rdi, qword ptr pDst
	ret
hoge2 endp

;----------------------------------------
; SAL
;----------------------------------------
masm_test_sal proc
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
masm_test_sal endp

;----------------------------------------
; SAR
;----------------------------------------
masm_test_sar proc
	sar al, 1
	sar al, 2
	sar al, -1
	sar ax, 1
	sar ax, 2
	sar eax, 1
	sar eax, 2
	sar byte ptr[eax], 1
	sar byte ptr[eax], 2
	sar word ptr[eax], 1
	sar word ptr[eax], 2
	sar dword ptr[eax], 1
	sar dword ptr[eax], 2

	sar r8b, 1
	sar r8b, 2
	sar r8w, 1
	sar r8w, 2
	sar r8d, 1
	sar r8d, 2
	sar r8, 1
	sar r8, 2
	sar qword ptr[rax], 1
	sar qword ptr[rax], 2
masm_test_sar endp

;----------------------------------------
; SHL
;----------------------------------------
masm_test_shl proc
	shl al, 1
	shl al, 2
	shl al, -1
	shl ax, 1
	shl ax, 2
	shl eax, 1
	shl eax, 2
	shl byte ptr[eax], 1
	shl byte ptr[eax], 2
	shl word ptr[eax], 1
	shl word ptr[eax], 2
	shl dword ptr[eax], 1
	shl dword ptr[eax], 2

	shl r8b, 1
	shl r8b, 2
	shl r8w, 1
	shl r8w, 2
	shl r8d, 1
	shl r8d, 2
	shl r8, 1
	shl r8, 2
	shl qword ptr[rax], 1
	shl qword ptr[rax], 2
masm_test_shl endp

;----------------------------------------
; SHR
;----------------------------------------
masm_test_shr proc
	shr al, 1
	shr al, 2
	shr al, -1
	shr ax, 1
	shr ax, 2
	shr eax, 1
	shr eax, 2
	shr byte ptr[eax], 1
	shr byte ptr[eax], 2
	shr word ptr[eax], 1
	shr word ptr[eax], 2
	shr dword ptr[eax], 1
	shr dword ptr[eax], 2

	shr r8b, 1
	shr r8b, 2
	shr r8w, 1
	shr r8w, 2
	shr r8d, 1
	shr r8d, 2
	shr r8, 1
	shr r8, 2
	shr qword ptr[rax], 1
	shr qword ptr[rax], 2
masm_test_shr endp

;----------------------------------------
; RCL
;----------------------------------------
masm_test_rcl proc
	rcl al, 1
	rcl al, 2
	rcl al, -1
	rcl ax, 1
	rcl ax, 2
	rcl eax, 1
	rcl eax, 2
	rcl byte ptr[eax], 1
	rcl byte ptr[eax], 2
	rcl word ptr[eax], 1
	rcl word ptr[eax], 2
	rcl dword ptr[eax], 1
	rcl dword ptr[eax], 2

	rcl r8b, 1
	rcl r8b, 2
	rcl r8w, 1
	rcl r8w, 2
	rcl r8d, 1
	rcl r8d, 2
	rcl r8, 1
	rcl r8, 2
	rcl qword ptr[rax], 1
	rcl qword ptr[rax], 2
masm_test_rcl endp

;----------------------------------------
; RCR
;----------------------------------------
masm_test_rcr proc
	rcr al, 1
	rcr al, 2
	rcr al, -1
	rcr ax, 1
	rcr ax, 2
	rcr eax, 1
	rcr eax, 2
	rcr byte ptr[eax], 1
	rcr byte ptr[eax], 2
	rcr word ptr[eax], 1
	rcr word ptr[eax], 2
	rcr dword ptr[eax], 1
	rcr dword ptr[eax], 2

	rcr r8b, 1
	rcr r8b, 2
	rcr r8w, 1
	rcr r8w, 2
	rcr r8d, 1
	rcr r8d, 2
	rcr r8, 1
	rcr r8, 2
	rcr qword ptr[rax], 1
	rcr qword ptr[rax], 2
masm_test_rcr endp

;----------------------------------------
; ROL
;----------------------------------------
masm_test_rol proc
	rol al, 1
	rol al, 2
	rol al, -1
	rol ax, 1
	rol ax, 2
	rol eax, 1
	rol eax, 2
	rol byte ptr[eax], 1
	rol byte ptr[eax], 2
	rol word ptr[eax], 1
	rol word ptr[eax], 2
	rol dword ptr[eax], 1
	rol dword ptr[eax], 2

	rol r8b, 1
	rol r8b, 2
	rol r8w, 1
	rol r8w, 2
	rol r8d, 1
	rol r8d, 2
	rol r8, 1
	rol r8, 2
	rol qword ptr[rax], 1
	rol qword ptr[rax], 2
masm_test_rol endp

;----------------------------------------
; ROR
;----------------------------------------
masm_test_ror proc
	ror al, 1
	ror al, 2
	ror al, -1
	ror ax, 1
	ror ax, 2
	ror eax, 1
	ror eax, 2
	ror byte ptr[eax], 1
	ror byte ptr[eax], 2
	ror word ptr[eax], 1
	ror word ptr[eax], 2
	ror dword ptr[eax], 1
	ror dword ptr[eax], 2

	ror r8b, 1
	ror r8b, 2
	ror r8w, 1
	ror r8w, 2
	ror r8d, 1
	ror r8d, 2
	ror r8, 1
	ror r8, 2
	ror qword ptr[rax], 1
	ror qword ptr[rax], 2
masm_test_ror endp

;----------------------------------------
; INC/DEC
;----------------------------------------
masm_test_inc_dec proc
	inc al
	inc ax
	inc eax
	inc byte ptr[eax]
	inc word ptr[eax]
	inc dword ptr[eax]
	dec al
	dec ax
	dec eax
	dec byte ptr[eax]
	dec word ptr[eax]
	dec dword ptr[eax]
	inc r8b
	inc r8w
	inc r8d
	inc rax
	inc r8
	inc qword ptr[rax]
	dec r8b
	dec r8w
	dec r8d
	dec rax
	dec r8
	dec qword ptr[rax]
masm_test_inc_dec endp

;----------------------------------------
; PUSH/POP
;----------------------------------------
masm_test_push_pop proc
	push ax
	push word ptr[eax]
	push 1h
	push 100h
	push 10000h
	push -1h
	push -100h
	push -10000h
	pop ax
	pop word ptr[eax]
	push rax
	push r8
	push qword ptr[rax]
	push qword ptr[r8]
	pop rax
	pop r8
	pop qword ptr[rax]
	pop qword ptr[r8]
masm_test_push_pop endp

;----------------------------------------
; ADD
;----------------------------------------
masm_test_add proc
	add al, 1h
	add al, -1h
	add ax, 1h
	add ax, 100h
	add ax, -1h
	add ax, -100h
	add eax, 1h
	add eax, 100h
	add eax, 10000h
	add eax, -1h
	add eax, -100h
	add eax, -10000h
	add cl, 1h
	add cl, -1h
	add cx, 1h
	add cx, 100h
	add cx, -1h
	add cx, -100h
	add ecx, 1h
	add ecx, 100h
	add ecx, 10000h
	add ecx, -1h
	add ecx, -100h
	add ecx, -10000h
	add byte ptr[eax], 1
	add word ptr[eax], 1
	add dword ptr[eax], 1
	add al, byte ptr[eax]
	add ax, word ptr[eax]
	add eax, dword ptr[eax]
	add byte ptr[eax], al
	add word ptr[eax], ax
	add dword ptr[eax], eax

	add rax, 1h
	add rax, 100h
	add rax, 10000h
	add rax, -1h
	add rax, -100h
	add rax, -10000h
	add r8, 1h
	add r8, 100h
	add r8, 10000h
	add r8, -1h
	add r8, -100h
	add r8, -10000h
	add qword ptr[rax], 1
	add rax, qword ptr[rax]
	add qword ptr[rax], rax
masm_test_add endp

;----------------------------------------
; OR
;----------------------------------------
masm_test_or proc
	or al, 1h
	or al, -1h
	or ax, 1h
	or ax, 100h
	or ax, -1h
	or ax, -100h
	or eax, 1h
	or eax, 100h
	or eax, 10000h
	or eax, -1h
	or eax, -100h
	or eax, -10000h
	or cl, 1h
	or cl, -1h
	or cx, 1h
	or cx, 100h
	or cx, -1h
	or cx, -100h
	or ecx, 1h
	or ecx, 100h
	or ecx, 10000h
	or ecx, -1h
	or ecx, -100h
	or ecx, -10000h
	or byte ptr[eax], 1
	or word ptr[eax], 1
	or dword ptr[eax], 1
	or al, byte ptr[eax]
	or ax, word ptr[eax]
	or eax, dword ptr[eax]
	or byte ptr[eax], al
	or word ptr[eax], ax
	or dword ptr[eax], eax

	or rax, 1h
	or rax, 100h
	or rax, 10000h
	or rax, -1h
	or rax, -100h
	or rax, -10000h
	or r8, 1h
	or r8, 100h
	or r8, 10000h
	or r8, -1h
	or r8, -100h
	or r8, -10000h
	or qword ptr[rax], 1
	or rax, qword ptr[rax]
	or qword ptr[rax], rax
masm_test_or endp

;----------------------------------------
; ADC
;----------------------------------------
masm_test_adc proc
	adc al, 1h
	adc al, -1h
	adc ax, 1h
	adc ax, 100h
	adc ax, -1h
	adc ax, -100h
	adc eax, 1h
	adc eax, 100h
	adc eax, 10000h
	adc eax, -1h
	adc eax, -100h
	adc eax, -10000h
	adc cl, 1h
	adc cl, -1h
	adc cx, 1h
	adc cx, 100h
	adc cx, -1h
	adc cx, -100h
	adc ecx, 1h
	adc ecx, 100h
	adc ecx, 10000h
	adc ecx, -1h
	adc ecx, -100h
	adc ecx, -10000h
	adc byte ptr[eax], 1
	adc word ptr[eax], 1
	adc dword ptr[eax], 1
	adc al, byte ptr[eax]
	adc ax, word ptr[eax]
	adc eax, dword ptr[eax]
	adc byte ptr[eax], al
	adc word ptr[eax], ax
	adc dword ptr[eax], eax

	adc rax, 1h
	adc rax, 100h
	adc rax, 10000h
	adc rax, -1h
	adc rax, -100h
	adc rax, -10000h
	adc r8, 1h
	adc r8, 100h
	adc r8, 10000h
	adc r8, -1h
	adc r8, -100h
	adc r8, -10000h
	adc qword ptr[rax], 1
	adc rax, qword ptr[rax]
	adc qword ptr[rax], rax
masm_test_adc endp

;----------------------------------------
; SBB
;----------------------------------------
masm_test_sbb proc
	sbb al, 1h
	sbb al, -1h
	sbb ax, 1h
	sbb ax, 100h
	sbb ax, -1h
	sbb ax, -100h
	sbb eax, 1h
	sbb eax, 100h
	sbb eax, 10000h
	sbb eax, -1h
	sbb eax, -100h
	sbb eax, -10000h
	sbb cl, 1h
	sbb cl, -1h
	sbb cx, 1h
	sbb cx, 100h
	sbb cx, -1h
	sbb cx, -100h
	sbb ecx, 1h
	sbb ecx, 100h
	sbb ecx, 10000h
	sbb ecx, -1h
	sbb ecx, -100h
	sbb ecx, -10000h
	sbb byte ptr[eax], 1
	sbb word ptr[eax], 1
	sbb dword ptr[eax], 1
	sbb al, byte ptr[eax]
	sbb ax, word ptr[eax]
	sbb eax, dword ptr[eax]
	sbb byte ptr[eax], al
	sbb word ptr[eax], ax
	sbb dword ptr[eax], eax

	sbb rax, 1h
	sbb rax, 100h
	sbb rax, 10000h
	sbb rax, -1h
	sbb rax, -100h
	sbb rax, -10000h
	sbb r8, 1h
	sbb r8, 100h
	sbb r8, 10000h
	sbb r8, -1h
	sbb r8, -100h
	sbb r8, -10000h
	sbb qword ptr[rax], 1
	sbb rax, qword ptr[rax]
	sbb qword ptr[rax], rax
masm_test_sbb endp

;----------------------------------------
; AND
;----------------------------------------
masm_test_and proc
	and al, 1h
	and al, -1h
	and ax, 1h
	and ax, 100h
	and ax, -1h
	and ax, -100h
	and eax, 1h
	and eax, 100h
	and eax, 10000h
	and eax, -1h
	and eax, -100h
	and eax, -10000h
	and cl, 1h
	and cl, -1h
	and cx, 1h
	and cx, 100h
	and cx, -1h
	and cx, -100h
	and ecx, 1h
	and ecx, 100h
	and ecx, 10000h
	and ecx, -1h
	and ecx, -100h
	and ecx, -10000h
	and byte ptr[eax], 1
	and word ptr[eax], 1
	and dword ptr[eax], 1
	and al, byte ptr[eax]
	and ax, word ptr[eax]
	and eax, dword ptr[eax]
	and byte ptr[eax], al
	and word ptr[eax], ax
	and dword ptr[eax], eax

	and rax, 1h
	and rax, 100h
	and rax, 10000h
	and rax, -1h
	and rax, -100h
	and rax, -10000h
	and r8, 1h
	and r8, 100h
	and r8, 10000h
	and r8, -1h
	and r8, -100h
	and r8, -10000h
	and qword ptr[rax], 1
	and rax, qword ptr[rax]
	and qword ptr[rax], rax
masm_test_and endp

;----------------------------------------
; SUB
;----------------------------------------
masm_test_sub proc
	sub al, 1h
	sub al, -1h
	sub ax, 1h
	sub ax, 100h
	sub ax, -1h
	sub ax, -100h
	sub eax, 1h
	sub eax, 100h
	sub eax, 10000h
	sub eax, -1h
	sub eax, -100h
	sub eax, -10000h
	sub cl, 1h
	sub cl, -1h
	sub cx, 1h
	sub cx, 100h
	sub cx, -1h
	sub cx, -100h
	sub ecx, 1h
	sub ecx, 100h
	sub ecx, 10000h
	sub ecx, -1h
	sub ecx, -100h
	sub ecx, -10000h
	sub byte ptr[eax], 1
	sub word ptr[eax], 1
	sub dword ptr[eax], 1
	sub al, byte ptr[eax]
	sub ax, word ptr[eax]
	sub eax, dword ptr[eax]
	sub byte ptr[eax], al
	sub word ptr[eax], ax
	sub dword ptr[eax], eax

	sub rax, 1h
	sub rax, 100h
	sub rax, 10000h
	sub rax, -1h
	sub rax, -100h
	sub rax, -10000h
	sub r8, 1h
	sub r8, 100h
	sub r8, 10000h
	sub r8, -1h
	sub r8, -100h
	sub r8, -10000h
	sub qword ptr[rax], 1
	sub rax, qword ptr[rax]
	sub qword ptr[rax], rax
masm_test_sub endp

;----------------------------------------
; XOR
;----------------------------------------
masm_test_xor proc
	xor al, 1h
	xor al, -1h
	xor ax, 1h
	xor ax, 100h
	xor ax, -1h
	xor ax, -100h
	xor eax, 1h
	xor eax, 100h
	xor eax, 10000h
	xor eax, -1h
	xor eax, -100h
	xor eax, -10000h
	xor cl, 1h
	xor cl, -1h
	xor cx, 1h
	xor cx, 100h
	xor cx, -1h
	xor cx, -100h
	xor ecx, 1h
	xor ecx, 100h
	xor ecx, 10000h
	xor ecx, -1h
	xor ecx, -100h
	xor ecx, -10000h
	xor byte ptr[eax], 1
	xor word ptr[eax], 1
	xor dword ptr[eax], 1
	xor al, byte ptr[eax]
	xor ax, word ptr[eax]
	xor eax, dword ptr[eax]
	xor byte ptr[eax], al
	xor word ptr[eax], ax
	xor dword ptr[eax], eax

	xor rax, 1h
	xor rax, 100h
	xor rax, 10000h
	xor rax, -1h
	xor rax, -100h
	xor rax, -10000h
	xor r8, 1h
	xor r8, 100h
	xor r8, 10000h
	xor r8, -1h
	xor r8, -100h
	xor r8, -10000h
	xor qword ptr[rax], 1
	xor rax, qword ptr[rax]
	xor qword ptr[rax], rax
masm_test_xor endp

;----------------------------------------
; CMP
;----------------------------------------
masm_test_cmp proc
	cmp al, 1h
	cmp al, -1h
	cmp ax, 1h
	cmp ax, 100h
	cmp ax, -1h
	cmp ax, -100h
	cmp eax, 1h
	cmp eax, 100h
	cmp eax, 10000h
	cmp eax, -1h
	cmp eax, -100h
	cmp eax, -10000h
	cmp cl, 1h
	cmp cl, -1h
	cmp cx, 1h
	cmp cx, 100h
	cmp cx, -1h
	cmp cx, -100h
	cmp ecx, 1h
	cmp ecx, 100h
	cmp ecx, 10000h
	cmp ecx, -1h
	cmp ecx, -100h
	cmp ecx, -10000h
	cmp byte ptr[eax], 1
	cmp word ptr[eax], 1
	cmp dword ptr[eax], 1
	cmp al, byte ptr[eax]
	cmp ax, word ptr[eax]
	cmp eax, dword ptr[eax]
	cmp byte ptr[eax], al
	cmp word ptr[eax], ax
	cmp dword ptr[eax], eax

	cmp rax, 1h
	cmp rax, 100h
	cmp rax, 10000h
	cmp rax, -1h
	cmp rax, -100h
	cmp rax, -10000h
	cmp r8, 1h
	cmp r8, 100h
	cmp r8, 10000h
	cmp r8, -1h
	cmp r8, -100h
	cmp r8, -10000h
	cmp qword ptr[rax], 1
	cmp rax, qword ptr[rax]
	cmp qword ptr[rax], rax
masm_test_cmp endp

;----------------------------------------
; XCHG
;----------------------------------------
masm_test_xchg proc
	xchg al, cl
	xchg cl, al
	xchg ax, cx
	xchg cx, ax
	xchg eax, ecx
	xchg ecx, eax
	xchg ecx, edx
	xchg edx, ecx
	xchg al, byte ptr[ecx]
	xchg byte ptr[ecx], al
	xchg cl, byte ptr[eax]
	xchg byte ptr[eax], cl
	xchg ax, word ptr[ecx]
	xchg word ptr[ecx], ax
	xchg cx, word ptr[eax]
	xchg word ptr[eax], cx
	xchg eax, dword ptr[ecx]
	xchg dword ptr[ecx], eax
	xchg ecx, dword ptr[eax]
	xchg dword ptr[eax], ecx

	xchg rax, r8
	xchg r8, rax
	xchg rax, qword ptr[r8]
	xchg qword ptr[r8], rax
	xchg r8, qword ptr[rax]
	xchg qword ptr[rax], r8
masm_test_xchg endp

;----------------------------------------
; TEST
;----------------------------------------
masm_test_test proc
	test al, 1
	test ax, 1
	test eax, 1
	test al, -1
	test ax, -1
	test eax, -1
	test cl, 1
	test cx, 1
	test ecx, 1
	test cl, -1
	test cx, -1
	test ecx, -1
	test al, cl
	test ax, cx
	test eax, ecx
	test cl, al
	test cx, ax
	test ecx, eax
	test byte ptr[eax], 1
	test word ptr[eax], 1
	test dword ptr[eax], 1
	test byte ptr[eax], cl
	test word ptr[eax], cx
	test dword ptr[eax], ecx

	test rax, 1
	test rax, -1
	test r8, 1
	test r8, -1
	test rax, r8
	test r8, rax
	test qword ptr[eax], 1
	test qword ptr[eax], r8
masm_test_test endp

;----------------------------------------
; MOV/MOVZX
;----------------------------------------
masm_test_mov proc
	mov al, cl
	mov ax, cx
	mov eax, ecx
	mov byte ptr[eax], cl
	mov word ptr[eax], cx
	mov dword ptr[eax], ecx
	mov al, byte ptr[ecx]
	mov ax, word ptr[ecx]
	mov eax, dword ptr[ecx]
	mov al, 1
	mov al, -1
	mov ax, 1
	mov ax, -1
	mov eax, 1
	mov eax, -1
	mov byte ptr[eax], 1
	mov word ptr[eax], 1
	mov dword ptr[eax], 1
	movzx ax, cl
	movzx eax, cl
	movzx eax, cx
	movzx ax, byte ptr[ecx]
	movzx eax, byte ptr[ecx]
	movzx eax, word ptr[ecx]

	mov rax, r8
	mov qword ptr[rax], r8
	mov rax, qword ptr[r8]
	mov rax, 1
	mov rax, -1
	mov qword ptr[rax], 1
	movzx rax, cl
	movzx rax, cx
	movzx rax, byte ptr[rcx]
	movzx rax, word ptr[rcx]
masm_test_mov endp

;----------------------------------------
; LEA
;----------------------------------------
masm_test_lea proc
	lea eax, dword ptr[eax]
	lea eax, dword ptr[esp]
	lea eax, dword ptr[ebp]
	lea eax, dword ptr[eax + ecx]
	lea eax, dword ptr[ecx + eax]
	lea eax, dword ptr[esp + ecx]
	lea eax, dword ptr[ecx + esp]
	lea eax, dword ptr[ebp + ecx]
	lea eax, dword ptr[ecx + ebp]
	lea eax, dword ptr[esp + ebp]
	lea eax, dword ptr[ebp + esp]
	lea eax, dword ptr[eax + 1h]
	lea eax, dword ptr[esp + 1h]
	lea eax, dword ptr[ebp + 1h]
	lea eax, dword ptr[eax + 100h]
	lea eax, dword ptr[esp + 100h]
	lea eax, dword ptr[ebp + 100h]
	lea eax, dword ptr[eax + 10000h]
	lea eax, dword ptr[esp + 10000h]
	lea eax, dword ptr[ebp + 10000h]
	lea eax, dword ptr[eax * 2]
	lea eax, dword ptr[ebp * 2]
	lea eax, dword ptr[eax * 4]
	lea eax, dword ptr[ebp * 4]
	lea eax, dword ptr[eax * 8]
	lea eax, dword ptr[ebp * 8]
	lea eax, dword ptr[eax * 2 + 1h]
	lea eax, dword ptr[ebp * 2 + 1h]
	lea eax, dword ptr[eax * 2 + 100h]
	lea eax, dword ptr[ebp * 2 + 100h]
	lea eax, dword ptr[eax * 2 + 10000h]
	lea eax, dword ptr[ebp * 2 + 10000h]
	lea eax, dword ptr[eax + ecx * 2]
	lea eax, dword ptr[ecx + eax * 2]
	lea eax, dword ptr[esp + ecx * 2]
	lea eax, dword ptr[ebp + ecx * 2]
	lea eax, dword ptr[ecx + ebp * 2]
	lea eax, dword ptr[esp + ebp * 2]
	lea eax, dword ptr[eax + ecx + 1h]
	lea eax, dword ptr[ecx + eax + 1h]
	lea eax, dword ptr[esp + ecx + 1h]
	lea eax, dword ptr[ecx + esp + 1h]
	lea eax, dword ptr[ebp + ecx + 1h]
	lea eax, dword ptr[ecx + ebp + 1h]
	lea eax, dword ptr[esp + ebp + 1h]
	lea eax, dword ptr[ebp + esp + 1h]
	lea eax, dword ptr[eax + ecx + 100h]
	lea eax, dword ptr[ecx + eax + 100h]
	lea eax, dword ptr[esp + ecx + 100h]
	lea eax, dword ptr[ecx + esp + 100h]
	lea eax, dword ptr[ebp + ecx + 100h]
	lea eax, dword ptr[ecx + ebp + 100h]
	lea eax, dword ptr[esp + ebp + 100h]
	lea eax, dword ptr[ebp + esp + 100h]
	lea eax, dword ptr[eax + ecx + 10000h]
	lea eax, dword ptr[ecx + eax + 10000h]
	lea eax, dword ptr[esp + ecx + 10000h]
	lea eax, dword ptr[ecx + esp + 10000h]
	lea eax, dword ptr[ebp + ecx + 10000h]
	lea eax, dword ptr[ecx + ebp + 10000h]
	lea eax, dword ptr[esp + ebp + 10000h]
	lea eax, dword ptr[ebp + esp + 10000h]
	lea eax, dword ptr[eax + ecx * 2 + 1h]
	lea eax, dword ptr[ecx + eax * 2 + 1h]
	lea eax, dword ptr[esp + ecx * 2 + 1h]
	lea eax, dword ptr[ebp + ecx * 2 + 1h]
	lea eax, dword ptr[ecx + ebp * 2 + 1h]
	lea eax, dword ptr[esp + ebp * 2 + 1h]
	lea eax, dword ptr[eax + ecx * 2 + 100h]
	lea eax, dword ptr[ecx + eax * 2 + 100h]
	lea eax, dword ptr[esp + ecx * 2 + 100h]
	lea eax, dword ptr[ebp + ecx * 2 + 100h]
	lea eax, dword ptr[ecx + ebp * 2 + 100h]
	lea eax, dword ptr[esp + ebp * 2 + 100h]
	lea eax, dword ptr[eax + ecx * 2 + 10000h]
	lea eax, dword ptr[ecx + eax * 2 + 10000h]
	lea eax, dword ptr[esp + ecx * 2 + 10000h]
	lea eax, dword ptr[ebp + ecx * 2 + 10000h]
	lea eax, dword ptr[ecx + ebp * 2 + 10000h]
	lea eax, dword ptr[esp + ebp * 2 + 10000h]
	lea eax, dword ptr[eax - 1h]
	lea eax, dword ptr[eax - 100h]
	lea eax, dword ptr[eax - 10000h]

	lea rax, qword ptr[rax]
	lea rax, qword ptr[rsp]
	lea rax, qword ptr[rbp]
	lea rax, qword ptr[rax + rcx]
	lea rax, qword ptr[rcx + rax]
	lea rax, qword ptr[rsp + rcx]
	lea rax, qword ptr[rcx + rsp]
	lea rax, qword ptr[rbp + rcx]
	lea rax, qword ptr[rcx + rbp]
	lea rax, qword ptr[rsp + rbp]
	lea rax, qword ptr[rbp + rsp]
	lea rax, qword ptr[rax + 1h]
	lea rax, qword ptr[rsp + 1h]
	lea rax, qword ptr[rbp + 1h]
	lea rax, qword ptr[rax + 100h]
	lea rax, qword ptr[rsp + 100h]
	lea rax, qword ptr[rbp + 100h]
	lea rax, qword ptr[rax + 10000h]
	lea rax, qword ptr[rsp + 10000h]
	lea rax, qword ptr[rbp + 10000h]
	lea rax, qword ptr[rax * 2]
	lea rax, qword ptr[rbp * 2]
	lea rax, qword ptr[rax * 4]
	lea rax, qword ptr[rbp * 4]
	lea rax, qword ptr[rax * 8]
	lea rax, qword ptr[rbp * 8]
	lea rax, qword ptr[rax * 2 + 1h]
	lea rax, qword ptr[rbp * 2 + 1h]
	lea rax, qword ptr[rax * 2 + 100h]
	lea rax, qword ptr[rbp * 2 + 100h]
	lea rax, qword ptr[rax * 2 + 10000h]
	lea rax, qword ptr[rbp * 2 + 10000h]
	lea rax, qword ptr[rax + rcx * 2]
	lea rax, qword ptr[rcx + rax * 2]
	lea rax, qword ptr[rsp + rcx * 2]
	lea rax, qword ptr[rbp + rcx * 2]
	lea rax, qword ptr[rcx + rbp * 2]
	lea rax, qword ptr[rsp + rbp * 2]
	lea rax, qword ptr[rax + rcx + 1h]
	lea rax, qword ptr[rcx + rax + 1h]
	lea rax, qword ptr[rsp + rcx + 1h]
	lea rax, qword ptr[rcx + rsp + 1h]
	lea rax, qword ptr[rbp + rcx + 1h]
	lea rax, qword ptr[rcx + rbp + 1h]
	lea rax, qword ptr[rsp + rbp + 1h]
	lea rax, qword ptr[rbp + rsp + 1h]
	lea rax, qword ptr[rax + rcx + 100h]
	lea rax, qword ptr[rcx + rax + 100h]
	lea rax, qword ptr[rsp + rcx + 100h]
	lea rax, qword ptr[rbp + rcx + 100h]
	lea rax, qword ptr[rcx + rbp + 100h]
	lea rax, qword ptr[rsp + rbp + 100h]
	lea rax, qword ptr[rbp + rsp + 100h]
	lea rax, qword ptr[rax + rcx + 10000h]
	lea rax, qword ptr[rcx + rax + 10000h]
	lea rax, qword ptr[rsp + rcx + 10000h]
	lea rax, qword ptr[rbp + rcx + 10000h]
	lea rax, qword ptr[rcx + rbp + 10000h]
	lea rax, qword ptr[rsp + rbp + 10000h]
	lea rax, qword ptr[rbp + rsp + 10000h]
	lea rax, qword ptr[rax + rcx * 2 + 1h]
	lea rax, qword ptr[rcx + rax * 2 + 1h]
	lea rax, qword ptr[rsp + rcx * 2 + 1h]
	lea rax, qword ptr[rbp + rcx * 2 + 1h]
	lea rax, qword ptr[rcx + rbp * 2 + 1h]
	lea rax, qword ptr[rsp + rbp * 2 + 1h]
	lea rax, qword ptr[rax + rcx * 2 + 100h]
	lea rax, qword ptr[rcx + rax * 2 + 100h]
	lea rax, qword ptr[rsp + rcx * 2 + 100h]
	lea rax, qword ptr[rbp + rcx * 2 + 100h]
	lea rax, qword ptr[rcx + rbp * 2 + 100h]
	lea rax, qword ptr[rsp + rbp * 2 + 100h]
	lea rax, qword ptr[rax + rcx * 2 + 10000h]
	lea rax, qword ptr[rcx + rax * 2 + 10000h]
	lea rax, qword ptr[rsp + rcx * 2 + 10000h]
	lea rax, qword ptr[rbp + rcx * 2 + 10000h]
	lea rax, qword ptr[rcx + rbp * 2 + 10000h]
	lea rax, qword ptr[rsp + rbp * 2 + 10000h]
masm_test_lea endp

;----------------------------------------
; FLD
;----------------------------------------
masm_test_fld proc
	fld real4 ptr[esp]
	fld real8 ptr[esp]
	fld real10 ptr[esp]
	fld st(0)
	fld st(7)

	fld real4 ptr[rsp]
	fld real8 ptr[rsp]
	fld real10 ptr[rsp]
masm_test_fld endp

;----------------------------------------
; JMP
;----------------------------------------
masm_test_jmp proc
	; jmp short
	nop
	jmp L1
	ja L1
	jae L1
	jb L1
	jbe L1
	jc L1
	jrcxz L1
	jecxz L1
	je L1
	jg L1
	jge L1
	jl L1
	jle L1
	jna L1
	jnae L1
	jnb L1
	jnbe L1
	jnc L1
	jne L1
	jng L1
	jnge L1
	jnl L1
	jnle L1
	jno L1
	jnp L1
	jns L1
	jnz L1
	jo L1
	jp L1
	jpe L1
	jpo L1
	js L1
	jz L1
L1:
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[rsp + rcx + 100h]	; 10 bytes
	; jmp near
	jmp L1
	ja L1
	jae L1
	jb L1
	jbe L1
	jc L1
	je L1
	jg L1
	jge L1
	jl L1
	jle L1
	jna L1
	jnae L1
	jnb L1
	jnbe L1
	jnc L1
	jne L1
	jng L1
	jnge L1
	jnl L1
	jnle L1
	jno L1
	jnp L1
	jns L1
	jnz L1
	jo L1
	jp L1
	jpe L1
	jpo L1
	js L1
	jz L1
masm_test_jmp endp

;----------------------------------------
; MOVSB/MOVSW/MOVSD/MOVSQ
;----------------------------------------
masm_test_movs proc
	movsb 
	movsw 
	movsd 
	rep movsb 
	rep movsw 
	rep movsd 

	movsq 
	rep movsq 
masm_test_movs endp

;----------------------------------------
; NEG/NOT
;----------------------------------------
masm_test_neg_not proc
	neg al
	neg ax
	neg eax
	neg byte ptr[esp]
	neg word ptr[esp]
	neg dword ptr[esp]
	not al
	not ax
	not eax
	not byte ptr[esp]
	not word ptr[esp]
	not dword ptr[esp]

	neg r8w
	neg rax
	neg r8
	neg qword ptr[rsp]
	not r8w
	not rax
	not r8
	not qword ptr[rsp]
masm_test_neg_not endp

;----------------------------------------
; DIV/IDIV/MUL
;----------------------------------------
masm_test_div_idiv_mul proc
	div al
	div ax
	div eax
	div byte ptr[esp]
	div word ptr[esp]
	div dword ptr[esp]
	idiv al
	idiv ax
	idiv eax
	idiv byte ptr[esp]
	idiv word ptr[esp]
	idiv dword ptr[esp]
	mul al
	mul ax
	mul eax
	mul byte ptr[esp]
	mul word ptr[esp]
	mul dword ptr[esp]

	div r8w
	div rax
	div r8
	div qword ptr[rsp]
	idiv r8w
	idiv rax
	idiv r8
	idiv qword ptr[rsp]
	mul r8w
	mul rax
	mul r8
	mul qword ptr[rsp]
masm_test_div_idiv_mul endp

;----------------------------------------
; IMUL
;----------------------------------------
masm_test_imul proc
	imul al
	imul ax
	imul eax
	imul byte ptr[esp]
	imul word ptr[esp]
	imul dword ptr[esp]
	imul ax, ax
	imul ax, word ptr[esp]
	imul eax, eax
	imul eax, dword ptr[esp]
	imul ax, ax, 1h
	imul ax, ax, -1h
	imul ax, ax, 100h
	imul eax, eax, 1h
	imul eax, eax, -1h
	imul eax, eax, 100h

	imul r8w
	imul rax
	imul r8
	imul qword ptr[rsp]
	imul rax, rax
	imul rax, qword ptr[rsp]
	imul rax, rax, 1h
	imul rax, rax, -1h
	imul rax, rax, 100h
masm_test_imul endp

;----------------------------------------
; FST/FSTP
;----------------------------------------
masm_test_fst proc
	fst real4 ptr[esp]
	fst real8 ptr[esp]
	fst st(0)
	fst st(7)
	fstp real4 ptr[esp]
	fstp real8 ptr[esp]
	fstp real10 ptr[esp]
	fstp st(0)
	fstp st(7)

	fst real4 ptr[rsp]
	fst real8 ptr[rsp]
	fstp real4 ptr[rsp]
	fstp real8 ptr[rsp]
	fstp real10 ptr[rsp]
masm_test_fst endp

;----------------------------------------
; SSE2 A~
;----------------------------------------
masm_test_sse2_a proc
	addpd xmm0, xmmword ptr[esp]
	addpd xmm0, xmm0
	addsd xmm0, qword ptr[esp]
	addsd xmm0, xmm0
	andpd xmm0, xmmword ptr[esp]
	andpd xmm0, xmm0
	andnpd xmm0, xmmword ptr[esp]
	andnpd xmm0, xmm0
	addpd xmm8, xmmword ptr[rsp]
	addpd xmm8, xmmword ptr[r8]
	addpd xmm8, xmm8
	addsd xmm8, qword ptr[rsp]
	addsd xmm8, qword ptr[r8]
	addsd xmm8, xmm8
	andpd xmm8, xmmword ptr[rsp]
	andpd xmm8, xmmword ptr[r8]
	andpd xmm8, xmm8
	andnpd xmm8, xmmword ptr[rsp]
	andnpd xmm8, xmmword ptr[r8]
	andnpd xmm8, xmm8
	clflush byte ptr[esp]
	clflush byte ptr[r8]

	cmpeqpd xmm0, xmmword ptr[esp]
	cmpeqpd xmm0, xmm0
	cmpltpd xmm0, xmmword ptr[esp]
	cmpltpd xmm0, xmm0
	cmplepd xmm0, xmmword ptr[esp]
	cmplepd xmm0, xmm0
	cmpunordpd xmm0, xmmword ptr[esp]
	cmpunordpd xmm0, xmm0
	cmpneqpd xmm0, xmmword ptr[esp]
	cmpneqpd xmm0, xmm0
	cmpnltpd xmm0, xmmword ptr[esp]
	cmpnltpd xmm0, xmm0
	cmpnlepd xmm0, xmmword ptr[esp]
	cmpnlepd xmm0, xmm0
	cmpordpd xmm0, xmmword ptr[esp]
	cmpordpd xmm0, xmm0

	cmpeqsd xmm0, qword ptr[esp]
	cmpeqsd xmm0, xmm0
	cmpltsd xmm0, qword ptr[esp]
	cmpltsd xmm0, xmm0
	cmplesd xmm0, qword ptr[esp]
	cmplesd xmm0, xmm0
	cmpunordsd xmm0, qword ptr[esp]
	cmpunordsd xmm0, xmm0
	cmpneqsd xmm0, qword ptr[esp]
	cmpneqsd xmm0, xmm0
	cmpnltsd xmm0, qword ptr[esp]
	cmpnltsd xmm0, xmm0
	cmpnlesd xmm0, qword ptr[esp]
	cmpnlesd xmm0, xmm0
	cmpordsd xmm0, qword ptr[esp]
	cmpordsd xmm0, xmm0

	comisd xmm0, qword ptr[esp]
	comisd xmm0, xmm0

	cmpeqpd xmm8, xmmword ptr[rsp]
	cmpeqpd xmm8, xmm8
	cmpltpd xmm8, xmmword ptr[r8]
	cmpltpd xmm8, xmm8
	cmplepd xmm8, xmmword ptr[rsp]
	cmplepd xmm8, xmm8
	cmpunordpd xmm8, xmmword ptr[r8]
	cmpunordpd xmm8, xmm8
	cmpneqpd xmm8, xmmword ptr[rsp]
	cmpneqpd xmm8, xmm8
	cmpnltpd xmm8, xmmword ptr[r8]
	cmpnltpd xmm8, xmm8
	cmpnlepd xmm8, xmmword ptr[rsp]
	cmpnlepd xmm8, xmm8
	cmpordpd xmm8, xmmword ptr[r8]
	cmpordpd xmm8, xmm8

	cmpeqsd xmm8, qword ptr[rsp]
	cmpeqsd xmm8, xmm8
	cmpltsd xmm8, qword ptr[r8]
	cmpltsd xmm8, xmm8
	cmplesd xmm8, qword ptr[rsp]
	cmplesd xmm8, xmm8
	cmpunordsd xmm8, qword ptr[r8]
	cmpunordsd xmm8, xmm8
	cmpneqsd xmm8, qword ptr[rsp]
	cmpneqsd xmm8, xmm8
	cmpnltsd xmm8, qword ptr[r8]
	cmpnltsd xmm8, xmm8
	cmpnlesd xmm8, qword ptr[rsp]
	cmpnlesd xmm8, xmm8
	cmpordsd xmm8, qword ptr[r8]
	cmpordsd xmm8, xmm8

	comisd xmm8, qword ptr[rsp]
	comisd xmm8, qword ptr[r8]
	comisd xmm8, xmm8

	cvtdq2pd xmm0, xmm1
	cvtdq2pd xmm0, qword ptr[esp]
	cvtpd2dq xmm0, xmm1
	cvtpd2dq xmm0, xmmword ptr[esp]
	cvtpd2pi mm0, xmm1
	cvtpd2pi mm0, xmmword ptr[esp]
	cvtpd2ps xmm0, xmm1
	cvtpd2ps xmm0, xmmword ptr[esp]
	cvtpi2pd xmm0, mm1
	cvtpi2pd xmm0, qword ptr[esp]
	cvtps2dq xmm0, xmm1
	cvtps2dq xmm0, xmmword ptr[esp]
	cvtdq2ps xmm0, xmm1
	cvtdq2ps xmm0, xmmword ptr[esp]
	cvtps2pd xmm0, xmm1
	cvtps2pd xmm0, qword ptr[esp]
	cvtsd2ss xmm0, xmm1
	cvtsd2ss xmm0, qword ptr[esp]
	cvtsi2sd xmm0, eax;
	cvtsi2sd xmm0, dword ptr[esp]
	cvtss2sd xmm0, xmm1
	cvtss2sd xmm0, dword ptr[esp]
	cvttpd2dq xmm0, xmm1
	cvttpd2dq xmm0, xmmword ptr[esp]
	cvttpd2pi mm0, xmm1
	cvttpd2pi mm0, xmmword ptr[esp]
	cvttps2dq xmm0, xmm1
	cvttps2dq xmm0, xmmword ptr[esp]
	cvttsd2si eax, xmm1
	cvttsd2si eax, qword ptr[esp]

	cvtdq2pd xmm8, xmm9
	cvtdq2pd xmm8, qword ptr[r8]
	cvtpd2dq xmm8, xmm9
	cvtpd2dq xmm8, xmmword ptr[r8]
	cvtpd2pi mm0, xmm9
	cvtpd2pi mm0, xmmword ptr[r8]
	cvtpd2ps xmm8, xmm9
	cvtpd2ps xmm8, xmmword ptr[r8]
	cvtpi2pd xmm8, mm1
	cvtpi2pd xmm8, qword ptr[r8]
	cvtps2dq xmm8, xmm9
	cvtps2dq xmm8, xmmword ptr[r8]
	cvtdq2ps xmm8, xmm9
	cvtdq2ps xmm8, xmmword ptr[r8]
	cvtps2pd xmm8, xmm9
	cvtps2pd xmm8, qword ptr[r8]
	cvtsd2ss xmm8, xmm9
	cvtsd2ss xmm8, qword ptr[r8]
	cvtsi2sd xmm8, r8;
	cvtsi2sd xmm8, dword ptr[r8]
	cvtss2sd xmm8, xmm9
	cvtss2sd xmm8, dword ptr[r8]
	cvttpd2dq xmm8, xmm9
	cvttpd2dq xmm8, xmmword ptr[r8]
	cvttpd2pi mm0, xmm9
	cvttpd2pi mm0, xmmword ptr[r8]
	cvttps2dq xmm8, xmm9
	cvttps2dq xmm8, xmmword ptr[r8]
	cvttsd2si rax, xmm1
	cvttsd2si rax, qword ptr[r8]
	cvttsd2si r8, xmm1
	cvttsd2si r8, qword ptr[r8]
masm_test_sse2_a endp

;----------------------------------------
; SSE2 D~
;----------------------------------------
masm_test_sse2_d proc
	divpd xmm0, xmm1
	divpd xmm0, xmmword ptr[esp]
	divsd xmm0, xmm1
	divsd xmm0, qword ptr[esp]
	maskmovdqu xmm0, xmm1
	maxpd xmm0, xmm1
	maxpd xmm0, xmmword ptr[esp]
	maxsd xmm0, xmm1
	maxsd xmm0, qword ptr[esp]
	minpd xmm0, xmm1
	minpd xmm0, xmmword ptr[esp]
	minsd xmm0, xmm1
	minsd xmm0, qword ptr[esp]
	divpd xmm8, xmm9
	divpd xmm8, xmmword ptr[r8]
	divsd xmm8, xmm9
	divsd xmm8, qword ptr[r8]
	maskmovdqu xmm8, xmm9
	maxpd xmm8, xmm9
	maxpd xmm8, xmmword ptr[r8]
	maxsd xmm8, xmm9
	maxsd xmm8, qword ptr[r8]
	minpd xmm8, xmm9
	minpd xmm8, xmmword ptr[r8]
	minsd xmm8, xmm9
	minsd xmm8, qword ptr[r8]

	movapd xmm0, xmm1
	movapd xmm0, xmmword ptr[esp]
	movapd xmmword ptr[esp], xmm0
	movdqa xmm0, xmm1
	movdqa xmm0, xmmword ptr[esp]
	movdqa xmmword ptr[esp], xmm0
	movdqu xmm0, xmm1
	movdqu xmm0, xmmword ptr[esp]
	movdqu xmmword ptr[esp], xmm0
	movdq2q mm0, xmm1
	movhpd qword ptr[esp], xmm1
	movhpd xmm0, qword ptr[esp]
	movlpd qword ptr[esp], xmm1
	movlpd xmm0, qword ptr[esp]
	movmskpd eax, xmm1
	movntdq xmmword ptr[esp], xmm1
	movnti dword ptr[esp], eax
	movntpd xmmword ptr[esp], xmm1
	movq2dq xmm0, mm1

	movapd xmm8, xmm9
	movapd xmm8, xmmword ptr[r8]
	movapd xmmword ptr[r8], xmm8
	movdqa xmm8, xmm9
	movdqa xmm8, xmmword ptr[r8]
	movdqa xmmword ptr[r8], xmm8
	movdqu xmm8, xmm9
	movdqu xmm8, xmmword ptr[r8]
	movdqu xmmword ptr[r8], xmm8
	movdq2q mm0, xmm9
	movhpd qword ptr[r8], xmm9
	movhpd xmm8, qword ptr[r8]
	movlpd qword ptr[r8], xmm9
	movlpd xmm8, qword ptr[r8]
	movmskpd eax, xmm9
	movmskpd rax, xmm9
	movntdq xmmword ptr[r8], xmm9
	movnti dword ptr[r8], ecx
	movnti qword ptr[r8], rcx
	movntpd xmmword ptr[r8], xmm9
	movq2dq xmm8, mm1
masm_test_sse2_d endp

;----------------------------------------
; MOVD/MOVQ
;----------------------------------------
masm_test_movd_movq proc
	movd mm0, dword ptr[eax]
	movd mm0, eax
	movq mm0, qword ptr[eax]
	movd dword ptr[eax], mm0
	movd eax, mm0
	movq qword ptr[eax], mm0
	movd xmm0, dword ptr[eax]
	movd xmm0, eax
	movq xmm0, qword ptr[eax]
	movd dword ptr[eax], xmm0
	movd eax, xmm0
	movq qword ptr[eax], xmm0
	movq mm0, mm0
	movq mm0, qword ptr[eax]
	movq qword ptr[eax], mm0
	movq xmm0, xmm0
	movq xmm0, qword ptr[eax]
	movq qword ptr[eax], xmm0

	movd mm0, rax				; movq
	movd rax, mm0				; movq
	movd xmm0, rax				; movq
	movd rax, xmm0				; movq
	movd mm0, dword ptr[rax]
	movq mm0, qword ptr[rax]
	movd dword ptr[rax], mm0
	movq qword ptr[rax], mm0
	movd xmm0, dword ptr[rax]
	movq xmm0, qword ptr[rax]
	movd dword ptr[rax], xmm0
	movq qword ptr[rax], xmm0
	movq mm0, qword ptr[rax]
	movq qword ptr[rax], mm0
	movq xmm0, qword ptr[rax]
	movq qword ptr[rax], xmm0
	; test REX
	movd mm0, r8				; movq
	movd r8, mm0				; movq
	movd xmm0, rax				; movq
	movd xmm0, r8				; movq
	movd xmm8, rax				; movq
	movd xmm8, r8				; movq
	movd rax, xmm0				; movq
	movd rax, xmm8				; movq
	movd r8, xmm0				; movq
	movd r8, xmm8				; movq
	movd mm0, dword ptr[r8]
	movq mm0, qword ptr[r8]
	movd dword ptr[r8], mm0
	movq qword ptr[r8], mm0
	movd xmm0, dword ptr[rax]
	movd xmm0, dword ptr[r8]
	movd xmm1, dword ptr[rax]
	movd xmm1, dword ptr[r8]
	movq xmm0, qword ptr[rax]
	movq xmm0, qword ptr[r8]
	movq xmm1, qword ptr[rax]
	movq xmm1, qword ptr[r8]
	movd dword ptr[rax], xmm0
	movd dword ptr[rax], xmm1
	movd dword ptr[r8], xmm0
	movd dword ptr[r8], xmm1
	movq qword ptr[rax], xmm0
	movq qword ptr[rax], xmm1
	movq qword ptr[r8], xmm0
	movq qword ptr[r8], xmm1
masm_test_movd_movq endp

;----------------------------------------
; MOVSD/MOVSS
;----------------------------------------
masm_test_movsd_movss proc
	movsd xmm0, xmm1
	movsd xmm0, qword ptr[esp]
	movsd qword ptr[esp], xmm0
	movss xmm0, xmm1
	movss xmm0, dword ptr[esp]
	movss dword ptr[esp], xmm0

	movsd xmm8, xmm1
	movsd xmm8, qword ptr[rsp]
	movsd qword ptr[r8], xmm8
	movss xmm8, xmm1
	movss xmm8, dword ptr[rsp]
	movss dword ptr[r8], xmm8
masm_test_movsd_movss endp

;----------------------------------------
; function0_cdecl<char>
;----------------------------------------
masm_test_function_return_char proc
	push rbp
	mov rbp, rsp
	push rsi
	movzx esi, cl
	mov al, cl
	pop rsi
	leave
	ret
masm_test_function_return_char endp

;----------------------------------------
; function0_cdecl<short>
;----------------------------------------
masm_test_function_return_short proc
	push rbp
	mov rbp, rsp
	mov ax, word ptr[rsi]
	leave
	ret
masm_test_function_return_short endp

;----------------------------------------
; function0_cdecl<int> (return immediate)
;----------------------------------------
masm_test_function_return_int_imm proc
	push rbp
	mov rbp, rsp
	mov eax, 16
	leave
	ret
masm_test_function_return_int_imm endp

;----------------------------------------
; function0_cdecl<int> (return eax)
;----------------------------------------
masm_test_function_return_int_eax proc
	push rbp
	mov rbp, rsp
	leave
	ret
masm_test_function_return_int_eax endp

;----------------------------------------
; function0_cdecl<float> (return immediate)
;----------------------------------------
masm_test_function_return_float_imm proc
	push rbp
	mov rbp, rsp
	mov dword ptr[rsp - 4], 41300000h
	movss xmm0, dword ptr[rsp - 4]
	leave
	ret
masm_test_function_return_float_imm endp

;----------------------------------------
; function0_cdecl<float> (return xmm)
;----------------------------------------
masm_test_function_return_float_xmm proc
	push rbp
	mov rbp, rsp
	sub rsp, 16
	movdqa xmmword ptr[rsp], xmm7
	movss xmm7, dword ptr[rsp]
	movss xmm0, xmm7
	movdqa xmm7, xmmword ptr[rsp]
	add rsp, 16
	leave
	ret
masm_test_function_return_float_xmm endp

;----------------------------------------
; function1_cdecl<float> (return ptr)
;----------------------------------------
masm_test_function_return_float_ptr proc
	push rbp
	mov rbp, rsp
	movss dword ptr[rbp + 16], xmm0
	movss xmm0, dword ptr[rbp + 16]
	leave
	ret
masm_test_function_return_float_ptr endp

;----------------------------------------
; function1_cdecl<float> (return st(0))
;----------------------------------------
masm_test_function_return_float_st0 proc
	push rbp
	mov rbp, rsp
	movss dword ptr[rbp + 16], xmm0
	fld real4 ptr[rbp + 16]
	fstp real4 ptr[rsp - 4]
	movss xmm0, dword ptr[rsp - 4]
	leave
	ret
masm_test_function_return_float_st0 endp

;----------------------------------------
; function0_cdecl<double> (return immediate)
;----------------------------------------
masm_test_function_return_double_imm proc
	push rbp
	mov rbp, rsp
	mov dword ptr[rsp - 8], 0
	mov dword ptr[rsp - 4], 40260000h
	movsd xmm0, qword ptr[rsp - 8]
	leave
	ret
masm_test_function_return_double_imm endp

;----------------------------------------
; function0_cdecl<double> (return xmm)
;----------------------------------------
masm_test_function_return_double_xmm proc
	push rbp
	mov rbp, rsp
	sub rsp, 16
	movdqa xmmword ptr[rsp], xmm7
	movsd xmm7, qword ptr[rsp]
	movsd xmm0, xmm7
	movdqa xmm7, xmmword ptr[rsp]
	add rsp, 16
	leave
	ret
masm_test_function_return_double_xmm endp

;----------------------------------------
; function1_cdecl<double> (return ptr)
;----------------------------------------
masm_test_function_return_double_ptr proc
	push rbp
	mov rbp, rsp
	movsd qword ptr[rbp + 16], xmm0
	movsd xmm0, qword ptr[rbp + 16]
	leave
	ret
masm_test_function_return_double_ptr endp

;----------------------------------------
; function1_cdecl<double> (return st(0))
;----------------------------------------
masm_test_function_return_double_st0 proc
	push rbp
	mov rbp, rsp
	movsd qword ptr[rbp + 16], xmm0
	fld real8 ptr[rbp + 16]
	fstp real8 ptr[rsp - 8]
	movsd xmm0, qword ptr[rsp - 8]
	leave
	ret
masm_test_function_return_double_st0 endp

end
