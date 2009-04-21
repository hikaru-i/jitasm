.686
.model flat, c
.xmm

.code

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
	; jump short
	loop L1
	loope L1
	loopne L1
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
	; jump near
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
	;movsq 
	;rep movsq 
	lodsb 
	lodsw 
	lodsd 
	rep lodsb 
	rep lodsw 
	rep lodsd 
	;lodsq 
	;rep lodsq 
	stosb 
	stosw 
	stosd 
	rep stosb 
	rep stosw 
	rep stosd 
	;stosq 
	;rep stosq 
	cmpsb 
	cmpsw 
	cmpsd 
	;cmpsq 
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
; setcc
;----------------------------------------
masm_test_setcc proc
	seta bl
	seta byte ptr[esp]
	setae bl
	setae byte ptr[esp]
	setb bl
	setb byte ptr[esp]
	setbe bl
	setbe byte ptr[esp]
	setc bl
	setc byte ptr[esp]
	sete bl
	sete byte ptr[esp]
	setg bl
	setg byte ptr[esp]
	setge bl
	setge byte ptr[esp]
	setl bl
	setl byte ptr[esp]
	setle bl
	setle byte ptr[esp]
	setna bl
	setna byte ptr[esp]
	setnae bl
	setnae byte ptr[esp]
	setnb bl
	setnb byte ptr[esp]
	setnbe bl
	setnbe byte ptr[esp]
	setnc bl
	setnc byte ptr[esp]
	setne bl
	setne byte ptr[esp]
	setng bl
	setng byte ptr[esp]
	setnge bl
	setnge byte ptr[esp]
	setnl bl
	setnl byte ptr[esp]
	setnle bl
	setnle byte ptr[esp]
	setno bl
	setno byte ptr[esp]
	setnp bl
	setnp byte ptr[esp]
	setns bl
	setns byte ptr[esp]
	setnz bl
	setnz byte ptr[esp]
	seto bl
	seto byte ptr[esp]
	setp bl
	setp byte ptr[esp]
	setpe bl
	setpe byte ptr[esp]
	setpo bl
	setpo byte ptr[esp]
	sets bl
	sets byte ptr[esp]
	setz bl
	setz byte ptr[esp]
masm_test_setcc endp

;----------------------------------------
; cmovcc
;----------------------------------------
masm_test_cmovcc proc
	cmova bx, dx
	cmova bx, word ptr[esp]
	cmovae bx, dx
	cmovae bx, word ptr[esp]
	cmovb bx, dx
	cmovb bx, word ptr[esp]
	cmovbe bx, dx
	cmovbe bx, word ptr[esp]
	cmovc bx, dx
	cmovc bx, word ptr[esp]
	cmove bx, dx
	cmove bx, word ptr[esp]
	cmovg bx, dx
	cmovg bx, word ptr[esp]
	cmovge bx, dx
	cmovge bx, word ptr[esp]
	cmovl bx, dx
	cmovl bx, word ptr[esp]
	cmovle bx, dx
	cmovle bx, word ptr[esp]
	cmovna bx, dx
	cmovna bx, word ptr[esp]
	cmovnae bx, dx
	cmovnae bx, word ptr[esp]
	cmovnb bx, dx
	cmovnb bx, word ptr[esp]
	cmovnbe bx, dx
	cmovnbe bx, word ptr[esp]
	cmovnc bx, dx
	cmovnc bx, word ptr[esp]
	cmovne bx, dx
	cmovne bx, word ptr[esp]
	cmovng bx, dx
	cmovng bx, word ptr[esp]
	cmovnge bx, dx
	cmovnge bx, word ptr[esp]
	cmovnl bx, dx
	cmovnl bx, word ptr[esp]
	cmovnle bx, dx
	cmovnle bx, word ptr[esp]
	cmovno bx, dx
	cmovno bx, word ptr[esp]
	cmovnp bx, dx
	cmovnp bx, word ptr[esp]
	cmovns bx, dx
	cmovns bx, word ptr[esp]
	cmovnz bx, dx
	cmovnz bx, word ptr[esp]
	cmovo bx, dx
	cmovo bx, word ptr[esp]
	cmovp bx, dx
	cmovp bx, word ptr[esp]
	cmovpe bx, dx
	cmovpe bx, word ptr[esp]
	cmovpo bx, dx
	cmovpo bx, word ptr[esp]
	cmovs bx, dx
	cmovs bx, word ptr[esp]
	cmovz bx, dx
	cmovz bx, word ptr[esp]
	cmova ebx, edx
	cmova ebx, dword ptr[esp]
	cmovae ebx, edx
	cmovae ebx, dword ptr[esp]
	cmovb ebx, edx
	cmovb ebx, dword ptr[esp]
	cmovbe ebx, edx
	cmovbe ebx, dword ptr[esp]
	cmovc ebx, edx
	cmovc ebx, dword ptr[esp]
	cmove ebx, edx
	cmove ebx, dword ptr[esp]
	cmovg ebx, edx
	cmovg ebx, dword ptr[esp]
	cmovge ebx, edx
	cmovge ebx, dword ptr[esp]
	cmovl ebx, edx
	cmovl ebx, dword ptr[esp]
	cmovle ebx, edx
	cmovle ebx, dword ptr[esp]
	cmovna ebx, edx
	cmovna ebx, dword ptr[esp]
	cmovnae ebx, edx
	cmovnae ebx, dword ptr[esp]
	cmovnb ebx, edx
	cmovnb ebx, dword ptr[esp]
	cmovnbe ebx, edx
	cmovnbe ebx, dword ptr[esp]
	cmovnc ebx, edx
	cmovnc ebx, dword ptr[esp]
	cmovne ebx, edx
	cmovne ebx, dword ptr[esp]
	cmovng ebx, edx
	cmovng ebx, dword ptr[esp]
	cmovnge ebx, edx
	cmovnge ebx, dword ptr[esp]
	cmovnl ebx, edx
	cmovnl ebx, dword ptr[esp]
	cmovnle ebx, edx
	cmovnle ebx, dword ptr[esp]
	cmovno ebx, edx
	cmovno ebx, dword ptr[esp]
	cmovnp ebx, edx
	cmovnp ebx, dword ptr[esp]
	cmovns ebx, edx
	cmovns ebx, dword ptr[esp]
	cmovnz ebx, edx
	cmovnz ebx, dword ptr[esp]
	cmovo ebx, edx
	cmovo ebx, dword ptr[esp]
	cmovp ebx, edx
	cmovp ebx, dword ptr[esp]
	cmovpe ebx, edx
	cmovpe ebx, dword ptr[esp]
	cmovpo ebx, edx
	cmovpo ebx, dword ptr[esp]
	cmovs ebx, edx
	cmovs ebx, dword ptr[esp]
	cmovz ebx, edx
	cmovz ebx, dword ptr[esp]
masm_test_cmovcc endp

;----------------------------------------
; General-Purpose Instructions B~
;----------------------------------------
masm_test_gpi_b proc
	bsf bx, dx
	bsf bx, word ptr[esp]
	bsf ebx, edx
	bsf ebx, dword ptr[esp]
	bsr bx, dx
	bsr bx, word ptr[esp]
	bsr ebx, edx
	bsr ebx, dword ptr[esp]
	bswap ebx
	bt bx, dx
	bt word ptr[eax], dx
	bt ebx, edx
	bt dword ptr[ecx], edx
	bt bx, 55h
	bt word ptr[eax], 55h
	bt ebx, 55h
	bt dword ptr[ecx], 55h
	btc bx, dx
	btc word ptr[eax], dx
	btc ebx, edx
	btc dword ptr[ecx], edx
	btc bx, 55h
	btc word ptr[eax], 55h
	btc ebx, 55h
	btc dword ptr[ecx], 55h
	btr bx, dx
	btr word ptr[eax], dx
	btr ebx, edx
	btr dword ptr[ecx], edx
	btr bx, 55h
	btr word ptr[eax], 55h
	btr ebx, 55h
	btr dword ptr[ecx], 55h
	bts bx, dx
	bts word ptr[eax], dx
	bts ebx, edx
	bts dword ptr[ecx], edx
	bts bx, 55h
	bts word ptr[eax], 55h
	bts ebx, 55h
	bts dword ptr[ecx], 55h
	call bx
	call ebx
	cbw 
	cwde 
	clc 
	cld 
	cli 
	cmc 
	cmpxchg bl, dl
	cmpxchg byte ptr[esp], dl
	cmpxchg bx, dx
	cmpxchg word ptr[eax], dx
	cmpxchg ebx, edx
	cmpxchg dword ptr[ecx], edx
	cmpxchg8b qword ptr[ecx]
	cpuid 
	cwd 
	cdq 
masm_test_gpi_b endp

;----------------------------------------
; General-Purpose Instructions E~
;----------------------------------------
masm_test_gpi_e proc
	enter 100h, 0
	enter 100h, 1
	enter 100h, 2
	hlt
	in al, 0AAh
	in ax, 0AAh
	in eax, 0AAh
	in al, dx
	in ax, dx
	in eax, dx
	insb
	insw
	insd
	rep insb
	rep insw
	rep insd
	int 3
	int 1
	into
	invd
	invlpg dword ptr[esp]
	iret
	iretd
	;iretq
	lar bx, dx
	lar bx, word ptr[esp]
	lar ebx, edx
	lar ebx, word ptr[esp]
	;lar rbx, rdx
	;lar rbx, word ptr[esp]
	leave 
	db 0Fh	;lldt cx
	db 0h
	db 0D1h
	db 0Fh	;lldt word ptr[ecx]
	db 00h
	db 11h
	db 0Fh	;lmsw cx
	db 01h
	db 0F1h
	db 0Fh	;lmsw word ptr[ecx]
	db 01h
	db 31h
	;movbe bx, word ptr[esp]
	;movbe ebx, dword ptr[esp]
	;movbe word ptr[esp], bx
	;movbe dword ptr[esp], ebx
	;movbe rbx, qword ptr[esp]
	;movbe qword ptr[esp], rbx
	movsx bx, dl
	movsx bx, byte ptr[esp]
	movsx ebx, dl
	movsx ebx, byte ptr[esp]
	movsx ebx, dx
	movsx ebx, word ptr[esp]
	;movsx rbx, dl
	;movsx rbx, byte ptr[esp]
	;movsx rbx, dx
	;movsx rbx, word ptr[esp]
	;movsxd rbx, edx
	;movsxd rbx, dword ptr[esp]
	nop
	out 0AAh, al
	out 0AAh, ax
	out 0AAh, eax
	out dx, al
	out dx, ax
	out dx, eax
	outsb 
	outsw 
	outsd 
	rep outsb 
	rep outsw 
	rep outsd 
	popf
	popfd
	;popfq
	pushf
	pushfd
	;pushfq
	db 0Fh	;rdmsr
	db 32h
	rdpmc
	rdtsc
	ret
	ret 1
	ret -1
	rsm
	scasb 
	scasw 
	scasd 
	;scasq 
	sgdt dword ptr[esp]
	shld bx, dx, 1
	shld word ptr[esp], dx, 1
	shld bx, dx, cl
	shld word ptr[esp], dx, cl
	shld ebx, edx, 1
	shld dword ptr[esp], edx, 1
	shld ebx, edx, cl
	shld dword ptr[esp], edx, cl
	;shld rbx, rdx, 1
	;shld qword ptr[esp], rdx, 1
	;shld rbx, rdx, cl
	;shld qword ptr[esp], rdx, cl
	shrd bx, dx, 1
	shrd word ptr[esp], dx, 1
	shrd bx, dx, cl
	shrd word ptr[esp], dx, cl
	shrd ebx, edx, 1
	shrd dword ptr[esp], edx, 1
	shrd ebx, edx, cl
	shrd dword ptr[esp], edx, cl
	;shrd rbx, rdx, 1
	;shrd qword ptr[esp], rdx, 1
	;shrd rbx, rdx, cl
	;shrd qword ptr[esp], rdx, cl
	sidt dword ptr[esp]
	sldt bx
	sldt word ptr[esp]
	;sldt r8
	smsw bx
	smsw word ptr[esp]
	;smsw r8
	stc 
	std 
	sti 
	sysenter
	db 0Fh	;sysexit
	db 35h
	;swapgs
	;syscall
	;sysret
	ud2 
	verr bx
	verr word ptr[esp]
	verw bx
	verw word ptr[esp]
	wait 
	xadd bl, dl
	xadd byte ptr[esp], dl
	xadd bx, dx
	xadd word ptr[esp], dx
	xadd ebx, edx
	xadd dword ptr[esp], edx
	;xadd rbx, rdx
	;xadd qword ptr[esp], rdx
	db 0Fh	;wbinvd
	db 09h
	db 0Fh	;wrmsr
	db 30h
	xlatb
masm_test_gpi_e endp

;----------------------------------------
; FPU
;----------------------------------------
masm_test_fpu proc
	f2xm1 
	fabs 
	fadd st(0), st(2)
	fadd st(1), st(0)
	fadd real4 ptr[ebx]
	fadd real8 ptr[edx]
	faddp 
	faddp st(1), st(0)
	fiadd word ptr[esp]
	fiadd real4 ptr[ebx]
	fbld real10 ptr[edi]
	fbstp real10 ptr[edi]
	fchs 
	fclex 
	fnclex 
	fcmovb st(0), st(2)
	fcmovbe st(0), st(2)
	fcmove st(0), st(2)
	fcmovnb st(0), st(2)
	fcmovnbe st(0), st(2)
	fcmovne st(0), st(2)
	fcmovnu st(0), st(2)
	fcmovu st(0), st(2)
	fcom 
	fcom st(1)
	fcom real4 ptr[ebx]
	fcom real8 ptr[edx]
	fcomp 
	fcomp st(1)
	fcomp real4 ptr[ebx]
	fcomp real8 ptr[edx]
	fcompp 
	fcomi st(0), st(2)
	fcomip st(0), st(2)
	fcos 
	fdecstp 
	fdiv st(0), st(2)
	fdiv st(1), st(0)
	fdiv real4 ptr[ebx]
	fdiv real8 ptr[edx]
	fdivp 
	fdivp st(1), st(0)
	fidiv word ptr[esp]
	fidiv real4 ptr[ebx]
	fdivr st(0), st(2)
	fdivr st(1), st(0)
	fdivr real4 ptr[ebx]
	fdivr real8 ptr[edx]
	fdivrp 
	fdivrp st(1), st(0)
	fidivr word ptr[esp]
	fidivr real4 ptr[ebx]
	ffree st(1)
	ficom word ptr[esp]
	ficom real4 ptr[ebx]
	ficomp word ptr[esp]
	ficomp real4 ptr[ebx]
	fild word ptr[esp]
	fild real4 ptr[ebx]
	fild real8 ptr[edx]
	fincstp 
	finit 
	fninit 
	fist word ptr[esp]
	fist real4 ptr[ebx]
	fistp word ptr[esp]
	fistp real4 ptr[ebx]
	fistp real8 ptr[edx]
	fisttp word ptr[esp]
	fisttp real4 ptr[ebx]
	fisttp real8 ptr[edx]
	fld real4 ptr[ecx]
	fld real8 ptr[esi]
	fld real10 ptr[esp]
	fld st(2)
	fld1 
	fldcw word ptr[ebp]
	fldenv [ebp]
	fldl2e 
	fldl2t 
	fldlg2 
	fldln2 
	fldpi 
	fldz 
	fmul st(0), st(2)
	fmul st(1), st(0)
	fmul real4 ptr[ebx]
	fmul real8 ptr[edx]
	fmulp 
	fmulp st(1), st(0)
	fimul word ptr[esp]
	fimul real4 ptr[ebx]
	fnop 
	fpatan 
	fprem 
	fprem1 
	fptan 
	frndint 
	frstor [ebp]
	fsave [edi]
	fnsave [edi]
	fscale 
	fsin 
	fsincos 
	fsqrt 
	fst real4 ptr[ebx]
	fst real8 ptr[edx]
	fst st(1)
	fstp st(1)
	fstp real4 ptr[ebx]
	fstp real8 ptr[edx]
	fstp real10 ptr[edi]
	fstcw word ptr[esp]
	fnstcw word ptr[esp]
	fstenv [ebp]
	fnstenv [ebp]
	fstsw word ptr[esp]
	fstsw ax
	fnstsw word ptr[esp]
	fnstsw ax
	fsub st(0), st(2)
	fsub st(1), st(0)
	fsub real4 ptr[ebx]
	fsub real8 ptr[edx]
	fsubp 
	fsubp st(1), st(0)
	fisub word ptr[esp]
	fisub real4 ptr[ebx]
	fsubr st(0), st(2)
	fsubr st(1), st(0)
	fsubr real4 ptr[ebx]
	fsubr real8 ptr[edx]
	fsubrp 
	fsubrp st(1), st(0)
	fisubr word ptr[esp]
	fisubr real4 ptr[ebx]
	ftst 
	fucom 
	fucom st(1)
	fucomp 
	fucomp st(1)
	fucompp 
	fucomi st(0), st(2)
	fucomip st(0), st(2)
	fwait 
	fxam 
	fxch 
	fxch st(1)
	fxrstor [esp]
	fxsave [esp]
	fxtract 
	fyl2x 
	fyl2xp1 
masm_test_fpu endp

;----------------------------------------
; MMX
;----------------------------------------
masm_test_mmx proc
	emms
	packsswb mm1, mm2
	packsswb mm1, qword ptr[ebp]
	packssdw mm1, mm2
	packssdw mm1, qword ptr[ebp]
	packuswb mm1, mm2
	packuswb mm1, qword ptr[ebp]
	paddb mm1, mm2
	paddb mm1, qword ptr[ebp]
	paddw mm1, mm2
	paddw mm1, qword ptr[ebp]
	paddd mm1, mm2
	paddd mm1, qword ptr[ebp]
	paddsb mm1, mm2
	paddsb mm1, qword ptr[ebp]
	paddsw mm1, mm2
	paddsw mm1, qword ptr[ebp]
	paddusb mm1, mm2
	paddusb mm1, qword ptr[ebp]
	paddusw mm1, mm2
	paddusw mm1, qword ptr[ebp]
	pand mm1, mm2
	pand mm1, qword ptr[ebp]
	pandn mm1, mm2
	pandn mm1, qword ptr[ebp]
	pcmpeqb mm1, mm2
	pcmpeqb mm1, qword ptr[ebp]
	pcmpeqw mm1, mm2
	pcmpeqw mm1, qword ptr[ebp]
	pcmpeqd mm1, mm2
	pcmpeqd mm1, qword ptr[ebp]
	pcmpgtb mm1, mm2
	pcmpgtb mm1, qword ptr[ebp]
	pcmpgtw mm1, mm2
	pcmpgtw mm1, qword ptr[ebp]
	pcmpgtd mm1, mm2
	pcmpgtd mm1, qword ptr[ebp]
	pmaddwd mm1, mm2
	pmaddwd mm1, qword ptr[ebp]
	pmulhw mm1, mm2
	pmulhw mm1, qword ptr[ebp]
	pmullw mm1, mm2
	pmullw mm1, qword ptr[ebp]
	por mm1, mm2
	por mm1, qword ptr[ebp]
	psllw mm1, mm3
	psllw mm1, qword ptr[ebp + ecx]
	psllw mm1, 2
	pslld mm1, mm3
	pslld mm1, qword ptr[ebp + ecx]
	pslld mm1, 2
	psllq mm1, mm3
	psllq mm1, qword ptr[ebp + ecx]
	psllq mm1, 2
	psraw mm1, mm3
	psraw mm1, qword ptr[ebp + ecx]
	psraw mm1, 2
	psrad mm1, mm3
	psrad mm1, qword ptr[ebp + ecx]
	psrad mm1, 2
	psrlw mm1, mm3
	psrlw mm1, qword ptr[ebp + ecx]
	psrlw mm1, 2
	psrld mm1, mm3
	psrld mm1, qword ptr[ebp + ecx]
	psrld mm1, 2
	psrlq mm1, mm3
	psrlq mm1, qword ptr[ebp + ecx]
	psrlq mm1, 2
	psubb mm1, mm2
	psubb mm1, qword ptr[ebp]
	psubw mm1, mm2
	psubw mm1, qword ptr[ebp]
	psubd mm1, mm2
	psubd mm1, qword ptr[ebp]
	psubsb mm1, mm2
	psubsb mm1, qword ptr[ebp]
	psubsw mm1, mm2
	psubsw mm1, qword ptr[ebp]
	psubusb mm1, mm2
	psubusb mm1, qword ptr[ebp]
	psubusw mm1, mm2
	psubusw mm1, qword ptr[ebp]
	punpckhbw mm2, mm3
	punpckhbw mm2, qword ptr[esp]
	punpckhwd mm2, mm3
	punpckhwd mm2, qword ptr[esp]
	punpckhdq mm2, mm3
	punpckhdq mm2, qword ptr[esp]
	punpcklbw mm2, mm3
	punpcklbw mm2, dword ptr[esp]
	punpcklwd mm2, mm3
	punpcklwd mm2, dword ptr[esp]
	punpckldq mm2, mm3
	punpckldq mm2, dword ptr[esp]
	pxor mm2, mm3
	pxor mm2, qword ptr[esp]
masm_test_mmx endp

;----------------------------------------
; MMX2
;----------------------------------------
masm_test_mmx2 proc
	pavgb mm1, mm2
	pavgb mm1, qword ptr[ebp]
	pavgw mm1, mm2
	pavgw mm1, qword ptr[ebp]
	pextrw ecx, mm2, 1
	pinsrw mm1, ecx, 2
	pinsrw mm1, word ptr[esp], 1
	pmaxsw mm1, mm2
	pmaxsw mm1, qword ptr[ebp]
	pmaxub mm1, mm2
	pmaxub mm1, qword ptr[ebp]
	pminsw mm1, mm2
	pminsw mm1, qword ptr[ebp]
	pminub mm1, mm2
	pminub mm1, qword ptr[ebp]
	pmovmskb eax, mm2
	pmulhuw mm1, mm2
	pmulhuw mm1, qword ptr[ebp]
	psadbw mm1, mm2
	psadbw mm1, qword ptr[ebp]
	pshufw mm1, mm2, 10h
	pshufw mm1, qword ptr[ebp], 1
masm_test_mmx2 endp

;----------------------------------------
; SSE
;----------------------------------------
masm_test_sse proc
	addps xmm1, xmm2
	addps xmm1, xmmword ptr[ebp]
	addss xmm1, xmm2
	addss xmm1, dword ptr[esi]
	andps xmm1, xmm2
	andps xmm1, xmmword ptr[ebp]
	andnps xmm1, xmm2
	andnps xmm1, xmmword ptr[ebp]
	cmpeqps xmm1, xmm2
	cmpeqps xmm1, xmmword ptr[ebp]
	cmpltps xmm1, xmm2
	cmpltps xmm1, xmmword ptr[ebp]
	cmpleps xmm1, xmm2
	cmpleps xmm1, xmmword ptr[ebp]
	cmpunordps xmm1, xmm2
	cmpunordps xmm1, xmmword ptr[ebp]
	cmpneqps xmm1, xmm2
	cmpneqps xmm1, xmmword ptr[ebp]
	cmpnltps xmm1, xmm2
	cmpnltps xmm1, xmmword ptr[ebp]
	cmpnleps xmm1, xmm2
	cmpnleps xmm1, xmmword ptr[ebp]
	cmpordps xmm1, xmm2
	cmpordps xmm1, xmmword ptr[ebp]
	cmpeqss xmm1, xmm2
	cmpeqss xmm1, dword ptr[esi]
	cmpltss xmm1, xmm2
	cmpltss xmm1, dword ptr[esi]
	cmpless xmm1, xmm2
	cmpless xmm1, dword ptr[esi]
	cmpunordss xmm1, xmm2
	cmpunordss xmm1, dword ptr[esi]
	cmpneqss xmm1, xmm2
	cmpneqss xmm1, dword ptr[esi]
	cmpnltss xmm1, xmm2
	cmpnltss xmm1, dword ptr[esi]
	cmpnless xmm1, xmm2
	cmpnless xmm1, dword ptr[esi]
	cmpordss xmm1, xmm2
	cmpordss xmm1, dword ptr[esi]
	comiss xmm1, xmm2
	comiss xmm1, dword ptr[esi]
	cvtpi2ps xmm1, mm3
	cvtpi2ps xmm1, qword ptr[esi]
	cvtps2pi mm1, xmm2
	cvtps2pi mm1, qword ptr[esi]
	cvtsi2ss xmm1, ecx
	cvtsi2ss xmm1, dword ptr[esi]
	cvtss2si eax, xmm2
	cvtss2si ecx, dword ptr[esi]
	cvttps2pi mm1, xmm2
	cvttps2pi mm1, qword ptr[esi]
	cvttss2si eax, xmm2
	cvttss2si ecx, dword ptr[esi]
	divps xmm1, xmm2
	divps xmm1, xmmword ptr[ebp]
	divss xmm1, xmm2
	divss xmm1, dword ptr[esi]
	ldmxcsr dword ptr[esi]
	maskmovq mm1, mm2
	maxps xmm1, xmm2
	maxps xmm1, xmmword ptr[ebp]
	maxss xmm1, xmm2
	maxss xmm1, dword ptr[esi]
	minps xmm1, xmm2
	minps xmm1, xmmword ptr[ebp]
	minss xmm1, xmm2
	minss xmm1, dword ptr[esi]
	movaps xmm1, xmm2
	movaps xmm1, xmmword ptr[ebp]
	movaps xmmword ptr[esp], xmm2
	movhlps xmm1, xmm2
	movhps xmm1, qword ptr[esi]
	movhps qword ptr[edi], xmm2
	movlhps xmm1, xmm2
	movlps xmm1, qword ptr[esi]
	movlps qword ptr[edi], xmm2
	movmskps eax, xmm2
	movntps xmmword ptr[esp], xmm2
	movntq qword ptr[edi], mm2
	movss xmm1, xmm2
	movss xmm1, dword ptr[esi]
	movss dword ptr[ebp], xmm2
	movups xmm1, xmm2
	movups xmm1, xmmword ptr[ebp]
	movups xmmword ptr[esp], xmm2
	mulps xmm1, xmm2
	mulps xmm1, xmmword ptr[ebp]
	mulss xmm1, xmm2
	mulss xmm1, dword ptr[esi]
	orps xmm1, xmm2
	orps xmm1, xmmword ptr[ebp]
	prefetcht0 byte ptr[ebp]
	prefetcht1 byte ptr[ebp]
	prefetcht2 byte ptr[ebp]
	prefetchnta byte ptr[ebp]
	rcpps xmm1, xmm2
	rcpps xmm1, xmmword ptr[ebp]
	rcpss xmm1, xmm2
	rcpss xmm1, dword ptr[esi]
	rsqrtps xmm1, xmm2
	rsqrtps xmm1, xmmword ptr[ebp]
	rsqrtss xmm1, xmm2
	rsqrtss xmm1, dword ptr[esi]
	sfence 
	shufps xmm1, xmm2, 10h
	shufps xmm1, xmmword ptr[ebp], 20h
	sqrtps xmm1, xmm2
	sqrtps xmm1, xmmword ptr[ebp]
	sqrtss xmm1, xmm2
	sqrtss xmm1, dword ptr[esi]
	stmxcsr dword ptr[esi]
	subps xmm1, xmm2
	subps xmm1, xmmword ptr[ebp]
	subss xmm1, xmm2
	subss xmm1, dword ptr[esi]
	ucomiss xmm1, xmm2
	ucomiss xmm1, dword ptr[esi]
	unpckhps xmm1, xmm2
	unpckhps xmm1, xmmword ptr[ebp]
	unpcklps xmm1, xmm2
	unpcklps xmm1, xmmword ptr[ebp]
	xorps xmm1, xmm2
	xorps xmm1, xmmword ptr[ebp]
masm_test_sse endp

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
	cvtsd2si eax, xmm1
	cvtsd2si ecx, qword ptr[esp]
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
	lfence
	maskmovdqu xmm0, xmm1
	maxpd xmm0, xmm1
	maxpd xmm0, xmmword ptr[esp]
	maxsd xmm0, xmm1
	maxsd xmm0, qword ptr[esp]
	mfence
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
	pextrw eax, xmm1, 1
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
	psllw xmm0, xmm1
	psllw xmm0, xmmword ptr[ebp]
	psllw xmm0, 2
	pslld xmm0, xmm1
	pslld xmm0, xmmword ptr[ebp]
	pslld xmm0, 2
	psllq xmm0, xmm1
	psllq xmm0, xmmword ptr[ebp]
	psllq xmm0, 2
	pslldq xmm0, 2
	psraw xmm0, xmm1
	psraw xmm0, xmmword ptr[ebp]
	psraw xmm0, 2
	psrad xmm0, xmm1
	psrad xmm0, xmmword ptr[ebp]
	psrad xmm0, 2
	psrlw xmm0, xmm1
	psrlw xmm0, xmmword ptr[ebp]
	psrlw xmm0, 2
	psrld xmm0, xmm1
	psrld xmm0, xmmword ptr[ebp]
	psrld xmm0, 2
	psrlq xmm0, xmm1
	psrlq xmm0, xmmword ptr[ebp]
	psrlq xmm0, 2
	psrldq xmm0, 2
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
; SSE3
;----------------------------------------
masm_test_sse3 proc
	addsubps xmm0, xmm1
	addsubps xmm0, mmword ptr[ebp]		; it must be xmmword...
	addsubpd xmm0, xmm1
	addsubpd xmm0, xmmword ptr[ebp]
	fisttp word ptr[ebp]
	fisttp dword ptr[ebp]
	fisttp qword ptr[ebp]
	haddps xmm0, xmm1
	haddps xmm0, xmmword ptr[ebp]
	haddpd xmm0, xmm1
	haddpd xmm0, xmmword ptr[ebp]
	hsubps xmm0, xmm1
	hsubps xmm0, xmmword ptr[ebp]
	hsubpd xmm0, xmm1
	hsubpd xmm0, xmmword ptr[ebp]
	lddqu xmm0, xmmword ptr[ebp]
	movddup xmm0, xmm1
	movddup xmm0, qword ptr[ebp]
	movshdup xmm0, xmm1
	movshdup xmm0, xmmword ptr[ebp]
	movsldup xmm0, xmm1
	movsldup xmm0, xmmword ptr[ebp]
	monitor eax,ecx,edx
	mwait eax,ecx
masm_test_sse3 endp

;----------------------------------------
; SSSE3
;----------------------------------------
masm_test_ssse3 proc
	pabsb mm0, mm1
	pabsb mm0, qword ptr[esp]
	pabsb xmm0, xmm1
	pabsb xmm0, xmmword ptr[esp]
	pabsw mm0, mm1
	pabsw mm0, qword ptr[esp]
	pabsw xmm0, xmm1
	pabsw xmm0, xmmword ptr[esp]
	pabsd mm0, mm1
	pabsd mm0, qword ptr[esp]
	pabsd xmm0, xmm1
	pabsd xmm0, xmmword ptr[esp]
	palignr mm0, mm1, 2
	palignr mm0, qword ptr[esp], 3
	palignr xmm0, xmm1, 4
	palignr xmm0, xmmword ptr[esp], 5
	phaddw mm0, mm1
	phaddw mm0, qword ptr[esp]
	phaddw xmm0, xmm1
	phaddw xmm0, xmmword ptr[esp]
	phaddd mm0, mm1
	phaddd mm0, qword ptr[esp]
	phaddd xmm0, xmm1
	phaddd xmm0, xmmword ptr[esp]
	phaddsw mm0, mm1
	phaddsw mm0, qword ptr[esp]
	phaddsw xmm0, xmm1
	phaddsw xmm0, xmmword ptr[esp]
	phsubw mm0, mm1
	phsubw mm0, qword ptr[esp]
	phsubw xmm0, xmm1
	phsubw xmm0, xmmword ptr[esp]
	phsubd mm0, mm1
	phsubd mm0, qword ptr[esp]
	phsubd xmm0, xmm1
	phsubd xmm0, xmmword ptr[esp]
	phsubsw mm0, mm1
	phsubsw mm0, qword ptr[esp]
	phsubsw xmm0, xmm1
	phsubsw xmm0, xmmword ptr[esp]
	pmaddubsw mm0, mm1
	pmaddubsw mm0, qword ptr[esp]
	pmaddubsw xmm0, xmm1
	pmaddubsw xmm0, xmmword ptr[esp]
	pmulhrsw mm0, mm1
	pmulhrsw mm0, qword ptr[esp]
	pmulhrsw xmm0, xmm1
	pmulhrsw xmm0, xmmword ptr[esp]
	pshufb mm0, mm1
	pshufb mm0, qword ptr[esp]
	pshufb xmm0, xmm1
	pshufb xmm0, xmmword ptr[esp]
	psignb mm0, mm1
	psignb mm0, qword ptr[esp]
	psignb xmm0, xmm1
	psignb xmm0, xmmword ptr[esp]
	psignw mm0, mm1
	psignw mm0, qword ptr[esp]
	psignw xmm0, xmm1
	psignw xmm0, xmmword ptr[esp]
	psignd mm0, mm1
	psignd mm0, qword ptr[esp]
	psignd xmm0, xmm1
	psignd xmm0, xmmword ptr[esp]
masm_test_ssse3 endp

;----------------------------------------
; SSE4.1
;----------------------------------------
masm_test_sse4_1 proc
	blendps xmm1, xmm2, 1
	blendps xmm1, xmmword ptr[esp], 2
	blendpd xmm1, xmm2, 3
	blendpd xmm1, xmmword ptr[esp], 4
	blendvps xmm1, xmm2, xmm0
	blendvps xmm1, xmmword ptr[esp], xmm0
	blendvpd xmm1, xmm2, xmm0
	blendvpd xmm1, xmmword ptr[esp], xmm0
	dpps xmm1, xmm2, 1
	dpps xmm1, xmmword ptr[esp], 2
	dppd xmm1, xmm2, 0
	dppd xmm1, xmmword ptr[esp], 1
	extractps eax, xmm2, 1
	extractps dword ptr[esp], xmm2, 0
	insertps xmm1, xmm2, 68h
	insertps xmm1, dword ptr[esp], 0
	movntdqa xmm1, xmmword ptr[esp]
	mpsadbw xmm1, xmm2, 1
	mpsadbw xmm1, xmmword ptr[esp], 4
	packusdw xmm1, xmm2
	packusdw xmm1, xmmword ptr[esp]
	pblendvb xmm1, xmm2, xmm0
	pblendvb xmm1, xmmword ptr[esp], xmm0
	pblendw xmm1, xmm2, 1
	pblendw xmm1, xmmword ptr[esp], 2
	pcmpeqq xmm1, xmm2
	pcmpeqq xmm1, xmmword ptr[esp]
	pextrb eax, xmm2, 1
	pextrb byte ptr[esp], xmm2, 2
	pextrw word ptr[esp], xmm2, 1
	pextrd eax, xmm2, 3
	pextrd dword ptr[esp], xmm2, 2
	pinsrb xmm1, eax, 0
	pinsrb xmm1, byte ptr[esp], 2
	pinsrd xmm1, eax, 1
	pinsrd xmm1, dword ptr[esp], 0
	pmaxsb xmm1, xmm2
	pmaxsb xmm1, xmmword ptr[esp]
	pmaxsd xmm1, xmm2
	pmaxsd xmm1, xmmword ptr[esp]
	pmaxuw xmm1, xmm2
	pmaxuw xmm1, xmmword ptr[esp]
	pmaxud xmm1, xmm2
	pmaxud xmm1, xmmword ptr[esp]
	pminsb xmm1, xmm2
	pminsb xmm1, xmmword ptr[esp]
	pminsd xmm1, xmm2
	pminsd xmm1, xmmword ptr[esp]
	pminuw xmm1, xmm2
	pminuw xmm1, xmmword ptr[esp]
	pminud xmm1, xmm2
	pminud xmm1, xmmword ptr[esp]

	; ML 9.00 generates wrong opcode for pmovsx/pmovsz
	; http://connect.microsoft.com/VisualStudio/feedback/ViewFeedback.aspx?FeedbackID=377701
	pmovsxbd xmm1, xmm2				; pmovsxbw
	pmovsxbd xmm1, qword ptr[esp]	; pmovsxbw
	pmovsxbq xmm1, xmm2				; pmovsxbd
	pmovsxbq xmm1, dword ptr[esp]	; pmovsxbd
	pmovsxbw xmm1, xmm2				; pmovsxbq
	pmovsxbw xmm1, word ptr[esp]	; pmovsxbq
	pmovsxdq xmm1, xmm2				; pmovsxwd
	pmovsxdq xmm1, qword ptr[esp]	; pmovsxwd
	pmovsxwd xmm1, xmm2				; pmovsxwq
	pmovsxwd xmm1, dword ptr[esp]	; pmovsxwq
	pmovsxwq xmm1, xmm2				; pmovsxdq
	pmovsxwq xmm1, qword ptr[esp]	; pmovsxdq
	pmovzxbd xmm1, xmm2				; pmovzxbw
	pmovzxbd xmm1, qword ptr[esp]	; pmovzxbw
	pmovzxbq xmm1, xmm2				; pmovzxbd
	pmovzxbq xmm1, dword ptr[esp]	; pmovzxbd
	pmovzxbw xmm1, xmm2				; pmovzxbq
	pmovzxbw xmm1, word ptr[esp]	; pmovzxbq
	pmovzxdq xmm1, xmm2				; pmovzxwd
	pmovzxdq xmm1, qword ptr[esp]	; pmovzxwd
	pmovzxwd xmm1, xmm2				; pmovzxwq
	pmovzxwd xmm1, dword ptr[esp]	; pmovzxwq
	pmovzxwq xmm1, xmm2				; pmovzxdq
	pmovzxwq xmm1, qword ptr[esp]	; pmovzxdq

	pmuldq xmm1, xmm2
	pmuldq xmm1, xmmword ptr[esp]
	pmulld xmm1, xmm2
	pmulld xmm1, xmmword ptr[esp]
	ptest xmm1, xmm2
	ptest xmm1, xmmword ptr[esp]
	roundps xmm1, xmm2, 0
	roundps xmm1, xmmword ptr[esp], 1
	roundpd xmm1, xmm2, 2
	roundpd xmm1, xmmword ptr[esp], 3
	roundss xmm1, xmm2, 0
	roundss xmm1, dword ptr[esp], 1
	roundsd xmm1, xmm2, 2
	roundsd xmm1, qword ptr[esp], 3
masm_test_sse4_1 endp

;----------------------------------------
; SSE4.2
;----------------------------------------
masm_test_sse4_2 proc
	crc32 eax, bh
	crc32 eax, byte ptr[esi]
	crc32 eax, bx
	crc32 eax, word ptr[esi]
	crc32 eax, ecx
	crc32 eax, dword ptr[esi]
	pcmpestri xmm2, xmm1, 0
	pcmpestri xmm2, xmmword ptr[esi], 0
	pcmpestrm xmm2, xmm1, 1
	pcmpestrm xmm2, xmmword ptr[esi], 1
	pcmpistri xmm2, xmm1, 0
	pcmpistri xmm2, xmmword ptr[esi], 0
	pcmpistrm xmm2, xmm1, 1
	pcmpistrm xmm2, xmmword ptr[esi], 1
	pcmpgtq xmm0, xmm1
	pcmpgtq xmm0, xmmword ptr[esi]
	popcnt ax, cx
	popcnt bx, word ptr[esi]
	popcnt eax, ecx
	popcnt eax, dword ptr[esi]
masm_test_sse4_2 endp

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