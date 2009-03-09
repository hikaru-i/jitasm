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
; function0_cdecl<int>
;----------------------------------------
masm_test_function0_cdecl proc
	push rbp
	mov rbp, rsp
	push rbx
	push rdi
	push rsi
	push r12;
	push r13;
	push r14;
	push r15;
	mov eax, 16
	pop r15;
	pop r14;
	pop r13;
	pop r12;
	pop rsi
	pop rdi
	pop rbx
	leave
	ret
masm_test_function0_cdecl endp

end
