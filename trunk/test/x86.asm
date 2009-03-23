.686
.model flat, c
.xmm

.code

hoge proc
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
hoge endp

hoge2 proc C USES esi edi ebx, pDst: PTR, pSrc: PTR, nLen: SDWORD
	LOCAL a:SDWORD
	mov esi, pSrc
	mov edi, pDst
	mov esi, [pSrc]
	mov edi, [pDst]
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
	push eax
	push dword ptr[eax]
	pop eax
	pop dword ptr[eax]
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
	jcxz L1
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
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
	pabsb xmm0, xmmword ptr[esp + ecx + 100h]	; 10 bytes
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
	clflush byte ptr[esp]

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
	movupd xmm0, xmm1
	movupd xmm0, xmmword ptr[esp]
	movupd xmmword ptr[esp], xmm0
	mulpd xmm0, xmm1
	mulpd xmm0, xmmword ptr[esp]
	mulsd xmm0, xmm1
	mulsd xmm0, qword ptr[esp]
	orpd xmm0, xmm1
	orpd xmm0, xmmword ptr[esp]
masm_test_sse2_d endp

;----------------------------------------
; SSE2 P~
;----------------------------------------
masm_test_sse2_p proc
	packsswb xmm0, xmm1
	packsswb xmm0, xmmword ptr[esp]
	packssdw xmm0, xmm1
	packssdw xmm0, xmmword ptr[esp]
	packuswb xmm0, xmm1
	packuswb xmm0, xmmword ptr[esp]
	paddb xmm0, xmm1
	paddb xmm0, xmmword ptr[esp]
	paddw xmm0, xmm1
	paddw xmm0, xmmword ptr[esp]
	paddd xmm0, xmm1
	paddd xmm0, xmmword ptr[esp]
	paddq mm0, mm1
	paddq mm0, qword ptr[esp]
	paddq xmm0, xmm1
	paddq xmm0, xmmword ptr[esp]
	paddsb xmm0, xmm1
	paddsb xmm0, xmmword ptr[esp]
	paddsw xmm0, xmm1
	paddsw xmm0, xmmword ptr[esp]
	paddusb xmm0, xmm1
	paddusb xmm0, xmmword ptr[esp]
	paddusw xmm0, xmm1
	paddusw xmm0, xmmword ptr[esp]
	pand xmm0, xmm1
	pand xmm0, xmmword ptr[esp]
	pandn xmm0, xmm1
	pandn xmm0, xmmword ptr[esp]
	pause 
	pavgb xmm0, xmm1
	pavgb xmm0, xmmword ptr[esp]
	pavgw xmm0, xmm1
	pavgw xmm0, xmmword ptr[esp]
	pcmpeqb xmm0, xmm1
	pcmpeqb xmm0, xmmword ptr[esp]
	pcmpeqw xmm0, xmm1
	pcmpeqw xmm0, xmmword ptr[esp]
	pcmpeqd xmm0, xmm1
	pcmpeqd xmm0, xmmword ptr[esp]
	pcmpgtb xmm0, xmm1
	pcmpgtb xmm0, xmmword ptr[esp]
	pcmpgtw xmm0, xmm1
	pcmpgtw xmm0, xmmword ptr[esp]
	pcmpgtd xmm0, xmm1
	pcmpgtd xmm0, xmmword ptr[esp]
;	pextrw eax, xmm1, 1
	pinsrw xmm0, ecx, 3
	pinsrw xmm0, word ptr[esp], 4
	pmaddwd xmm0, xmm1
	pmaddwd xmm0, xmmword ptr[esp]
	pmaxsw xmm0, xmm1
	pmaxsw xmm0, xmmword ptr[esp]
	pmaxub xmm0, xmm1
	pmaxub xmm0, xmmword ptr[esp]
	pminsw xmm0, xmm1
	pminsw xmm0, xmmword ptr[esp]
	pminub xmm0, xmm1
	pminub xmm0, xmmword ptr[esp]
	pmovmskb eax, xmm1
	pmulhuw xmm0, xmm1
	pmulhuw xmm0, xmmword ptr[esp]
	pmulhw xmm0, xmm1
	pmulhw xmm0, xmmword ptr[esp]
	pmullw xmm0, xmm1
	pmullw xmm0, xmmword ptr[esp]
	pmuludq mm0, mm1
	pmuludq mm0, qword ptr[esp]
	pmuludq xmm0, xmm1
	pmuludq xmm0, xmmword ptr[esp]
	por xmm0, xmm1
	por xmm0, xmmword ptr[esp]
	psadbw xmm0, xmm1
	psadbw xmm0, xmmword ptr[esp]
	pshufd xmm0, xmm1, 1Bh
	pshufd xmm0, xmmword ptr[esp], 0Bh
	pshufhw xmm0, xmm1, 1Ah
	pshufhw xmm0, xmmword ptr[esp], 18h
	pshuflw xmm0, xmm1, 14h
	pshuflw xmm0, xmmword ptr[esp], 12h
;	psllw xmm0, xmm1
;	psllw xmm0, xmmword ptr[ebp]
;	psllw xmm0, 2
;	pslld xmm0, xmm1
;	pslld xmm0, xmmword ptr[ebp]
;	pslld xmm0, 2
;	psllq xmm0, xmm1
;	psllq xmm0, xmmword ptr[ebp]
;	psllq xmm0, 2
;	pslldq xmm0, 2
;	psraw xmm0, xmm1
;	psraw xmm0, xmmword ptr[ebp]
;	psraw xmm0, 2
;	psrad xmm0, xmm1
;	psrad xmm0, xmmword ptr[ebp]
;	psrad xmm0, 2
;	psrlw xmm0, xmm1
;	psrlw xmm0, xmmword ptr[ebp]
;	psrlw xmm0, 2
;	psrld xmm0, xmm1
;	psrld xmm0, xmmword ptr[ebp]
;	psrld xmm0, 2
;	psrlq xmm0, xmm1
;	psrlq xmm0, xmmword ptr[ebp]
;	psrlq xmm0, 2
;	psrldq xmm0, 2
	psubb xmm0, xmm1
	psubb xmm0, xmmword ptr[esp]
	psubw xmm0, xmm1
	psubw xmm0, xmmword ptr[esp]
	psubd xmm0, xmm1
	psubd xmm0, xmmword ptr[esp]
	psubq mm0, mm1
	psubq mm0, qword ptr[esp]
	psubq xmm0, xmm1
	psubq xmm0, xmmword ptr[esp]
	psubsb xmm0, xmm1
	psubsb xmm0, xmmword ptr[esp]
	psubsw xmm0, xmm1
	psubsw xmm0, xmmword ptr[esp]
	psubusb xmm0, xmm1
	psubusb xmm0, xmmword ptr[esp]
	psubusw xmm0, xmm1
	psubusw xmm0, xmmword ptr[esp]
	punpckhbw xmm0, xmm1
	punpckhbw xmm0, xmmword ptr[esp]
	punpckhwd xmm0, xmm1
	punpckhwd xmm0, xmmword ptr[esp]
	punpckhdq xmm0, xmm1
	punpckhdq xmm0, xmmword ptr[esp]
	punpckhqdq xmm0, xmm1
	punpckhqdq xmm0, xmmword ptr[esp]
	punpcklbw xmm0, xmm1
	punpcklbw xmm0, xmmword ptr[esp]
	punpcklwd xmm0, xmm1
	punpcklwd xmm0, xmmword ptr[esp]
	punpckldq xmm0, xmm1
	punpckldq xmm0, xmmword ptr[esp]
	punpcklqdq xmm0, xmm1
	punpcklqdq xmm0, xmmword ptr[esp]
	pxor xmm0, xmm1
	pxor xmm0, xmmword ptr[esp]
masm_test_sse2_p endp

;----------------------------------------
; SSE2 S~
;----------------------------------------
masm_test_sse2_s proc
	shufpd xmm0, xmm1, 2
	shufpd xmm0, xmmword ptr[esp], 1
	sqrtpd xmm0, xmm1
	sqrtpd xmm0, xmmword ptr[esp]
	sqrtsd xmm0, xmm1
	sqrtsd xmm0, qword ptr[ebp]
	subpd xmm0, xmm1
	subpd xmm0, xmmword ptr[esp]
	subsd xmm0, xmm1
	subsd xmm0, qword ptr[ebp]
	ucomisd xmm0, xmm1
	ucomisd xmm0, qword ptr[ebp]
	unpckhpd xmm0, xmm1
	unpckhpd xmm0, xmmword ptr[esp]
	unpcklpd xmm0, xmm1
	unpcklpd xmm0, xmmword ptr[esp]
	xorpd xmm0, xmm1
	xorpd xmm0, xmmword ptr[esp]
masm_test_sse2_s endp

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
	movq mm0, mm1
	movq mm0, qword ptr[eax]
	movq qword ptr[eax], mm0
	movq xmm0, xmm0
	movq xmm0, qword ptr[eax]
	movq qword ptr[eax], xmm0
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
masm_test_movsd_movss endp

;----------------------------------------
; function0_cdecl<char>
;----------------------------------------
masm_test_function_return_char proc
	push ebp
	mov ebp, esp
	push esi
	movzx esi, cl
	mov al, cl
	pop esi
	leave
	ret
masm_test_function_return_char endp

;----------------------------------------
; function0_cdecl<short>
;----------------------------------------
masm_test_function_return_short proc
	push ebp
	mov ebp, esp
	mov ax, word ptr[esi]
	leave
	ret
masm_test_function_return_short endp

;----------------------------------------
; function0_cdecl<int> (return immediate)
;----------------------------------------
masm_test_function_return_int_imm proc
	push ebp
	mov ebp, esp
	mov eax, 16
	leave
	ret
masm_test_function_return_int_imm endp

;----------------------------------------
; function0_cdecl<int> (return eax)
;----------------------------------------
masm_test_function_return_int_eax proc
	push ebp
	mov ebp, esp
	leave
	ret
masm_test_function_return_int_eax endp

;----------------------------------------
; function0_cdecl<float> (return immediate)
;----------------------------------------
masm_test_function_return_float_imm proc
	push ebp
	mov ebp, esp
	mov dword ptr[esp - 4], 41300000h
	fld real4 ptr[esp - 4]
	leave
	ret
masm_test_function_return_float_imm endp

;----------------------------------------
; function0_cdecl<float> (return xmm)
;----------------------------------------
masm_test_function_return_float_xmm proc
	push ebp
	mov ebp, esp
	movss xmm7, dword ptr[esp]
	movss dword ptr[esp - 4], xmm7
	fld real4 ptr[esp - 4]
	leave
	ret
masm_test_function_return_float_xmm endp

;----------------------------------------
; function1_cdecl<float, float> (return ptr)
;----------------------------------------
masm_test_function_return_float_ptr proc
	push ebp
	mov ebp, esp
	fld real4 ptr[ebp + 8]
	leave
	ret
masm_test_function_return_float_ptr endp

;----------------------------------------
; function1_cdecl<float, float> (return st(0))
;----------------------------------------
masm_test_function_return_float_st0 proc
	push ebp
	mov ebp, esp
	fld real4 ptr[ebp + 8]
	leave
	ret
masm_test_function_return_float_st0 endp

;----------------------------------------
; function0_cdecl<double> (return immediate)
;----------------------------------------
masm_test_function_return_double_imm proc
	push ebp
	mov ebp, esp
	mov dword ptr[esp - 8], 0
	mov dword ptr[esp - 4], 40260000h
	fld real8 ptr[esp - 8]
	leave
	ret
masm_test_function_return_double_imm endp

;----------------------------------------
; function0_cdecl<double> (return xmm)
;----------------------------------------
masm_test_function_return_double_xmm proc
	push ebp
	mov ebp, esp
	movsd xmm7, qword ptr[esp]
	movsd qword ptr[esp - 8], xmm7
	fld real8 ptr[esp - 8]
	leave
	ret
masm_test_function_return_double_xmm endp

;----------------------------------------
; function1_cdecl<double, double> (return ptr)
;----------------------------------------
masm_test_function_return_double_ptr proc
	push ebp
	mov ebp, esp
	fld real8 ptr[ebp + 8]
	leave
	ret
masm_test_function_return_double_ptr endp

;----------------------------------------
; function1_cdecl<double, double> (return st(0))
;----------------------------------------
masm_test_function_return_double_st0 proc
	push ebp
	mov ebp, esp
	fld real8 ptr[ebp + 8]
	leave
	ret
masm_test_function_return_double_st0 endp

;----------------------------------------
; function1_cdecl<__m64, int> (return mm1)
;----------------------------------------
masm_test_function_return_m64_mm1 proc
	push ebp
	mov ebp, esp
	movd mm1, dword ptr[ebp + 8]
	punpckldq mm1, mm1
	paddw mm1, mm1
	movq mm0, mm1
	leave
	ret
masm_test_function_return_m64_mm1 endp

;----------------------------------------
; function1_cdecl<__m64> (return ptr)
;----------------------------------------
masm_test_function_return_m64_ptr proc
	push ebp
	mov ebp, esp
	movq mm0, qword ptr[esp - 8]
	leave
	ret
masm_test_function_return_m64_ptr endp

;----------------------------------------
; function0_cdecl<__m128> (return xmm1)
;----------------------------------------
masm_test_function_return_m128_xmm1 proc
	push ebp
	mov ebp, esp
	pxor xmm1, xmm1
	movaps xmm0, xmm1
	leave
	ret
masm_test_function_return_m128_xmm1 endp

;----------------------------------------
; function0_cdecl<__m128> (return ptr)
;----------------------------------------
masm_test_function_return_m128_ptr proc
	push ebp
	mov ebp, esp
	movaps xmm0, xmmword ptr[esp - 16]
	leave
	ret
masm_test_function_return_m128_ptr endp

;----------------------------------------
; function0_cdecl<__m128d> (return xmm1)
;----------------------------------------
masm_test_function_return_m128d_xmm1 proc
	push ebp
	mov ebp, esp
	pxor xmm1, xmm1
	movapd xmm0, xmm1
	leave
	ret
masm_test_function_return_m128d_xmm1 endp

;----------------------------------------
; function0_cdecl<__m128d> (return ptr)
;----------------------------------------
masm_test_function_return_m128d_ptr proc
	push ebp
	mov ebp, esp
	movapd xmm0, xmmword ptr[esp - 16]
	leave
	ret
masm_test_function_return_m128d_ptr endp

;----------------------------------------
; function0_cdecl<__m128i> (return xmm1)
;----------------------------------------
masm_test_function_return_m128i_xmm1 proc
	push ebp
	mov ebp, esp
	pxor xmm1, xmm1
	movdqa xmm0, xmm1
	leave
	ret
masm_test_function_return_m128i_xmm1 endp

;----------------------------------------
; function0_cdecl<__m128i> (return ptr)
;----------------------------------------
masm_test_function_return_m128i_ptr proc
	push ebp
	mov ebp, esp
	movdqa xmm0, xmmword ptr[esp - 16]
	leave
	ret
masm_test_function_return_m128i_ptr endp

end
