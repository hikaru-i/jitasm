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

	mov r8b, bl
	mov r8w, bx
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
	movsq 
	rep movsq 
	lodsb 
	lodsw 
	lodsd 
	rep lodsb 
	rep lodsw 
	rep lodsd 
	lodsq 
	rep lodsq 
	stosb 
	stosw 
	stosd 
	rep stosb 
	rep stosw 
	rep stosd 
	stosq 
	rep stosq 
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

	cmova rbx, rdx
	cmova rbx, qword ptr[esp]
	cmovae rbx, rdx
	cmovae rbx, qword ptr[esp]
	cmovb rbx, rdx
	cmovb rbx, qword ptr[esp]
	cmovbe rbx, rdx
	cmovbe rbx, qword ptr[esp]
	cmovc rbx, rdx
	cmovc rbx, qword ptr[esp]
	cmove rbx, rdx
	cmove rbx, qword ptr[esp]
	cmovg rbx, rdx
	cmovg rbx, qword ptr[esp]
	cmovge rbx, rdx
	cmovge rbx, qword ptr[esp]
	cmovl rbx, rdx
	cmovl rbx, qword ptr[esp]
	cmovle rbx, rdx
	cmovle rbx, qword ptr[esp]
	cmovna rbx, rdx
	cmovna rbx, qword ptr[esp]
	cmovnae rbx, rdx
	cmovnae rbx, qword ptr[esp]
	cmovnb rbx, rdx
	cmovnb rbx, qword ptr[esp]
	cmovnbe rbx, rdx
	cmovnbe rbx, qword ptr[esp]
	cmovnc rbx, rdx
	cmovnc rbx, qword ptr[esp]
	cmovne rbx, rdx
	cmovne rbx, qword ptr[esp]
	cmovng rbx, rdx
	cmovng rbx, qword ptr[esp]
	cmovnge rbx, rdx
	cmovnge rbx, qword ptr[esp]
	cmovnl rbx, rdx
	cmovnl rbx, qword ptr[esp]
	cmovnle rbx, rdx
	cmovnle rbx, qword ptr[esp]
	cmovno rbx, rdx
	cmovno rbx, qword ptr[esp]
	cmovnp rbx, rdx
	cmovnp rbx, qword ptr[esp]
	cmovns rbx, rdx
	cmovns rbx, qword ptr[esp]
	cmovnz rbx, rdx
	cmovnz rbx, qword ptr[esp]
	cmovo rbx, rdx
	cmovo rbx, qword ptr[esp]
	cmovp rbx, rdx
	cmovp rbx, qword ptr[esp]
	cmovpe rbx, rdx
	cmovpe rbx, qword ptr[esp]
	cmovpo rbx, rdx
	cmovpo rbx, qword ptr[esp]
	cmovs rbx, rdx
	cmovs rbx, qword ptr[esp]
	cmovz rbx, rdx
	cmovz rbx, qword ptr[esp]
masm_test_cmovcc endp

;----------------------------------------
; General-Purpose Instructions B~
;----------------------------------------
masm_test_gpi_b proc
	bsf bx, dx
	bsf bx, word ptr[esp]
	bsf ebx, edx
	bsf ebx, dword ptr[esp]
	bsf rbx, r11
	bsf rbx, qword ptr[rsp]
	bsr bx, dx
	bsr bx, word ptr[esp]
	bsr ebx, edx
	bsr ebx, dword ptr[esp]
	bsr rbx, r11
	bsr rbx, qword ptr[rsp]
	bswap ebx
	bswap ebx
	bt bx, dx
	bt word ptr[eax], dx
	bt ebx, edx
	bt dword ptr[ecx], edx
	bt bx, 55h
	bt word ptr[eax], 55h
	bt ebx, 55h
	bt dword ptr[ecx], 55h
	bt rbx, r11
	bt qword ptr[ecx], r11
	bt rbx, 55h
	bt qword ptr[ecx], 55h
	btc bx, dx
	btc word ptr[eax], dx
	btc ebx, edx
	btc dword ptr[ecx], edx
	btc bx, 55h
	btc word ptr[eax], 55h
	btc ebx, 55h
	btc dword ptr[ecx], 55h
	btc rbx, r11
	btc qword ptr[ecx], r11
	btc rbx, 55h
	btc qword ptr[ecx], 55h
	btr bx, dx
	btr word ptr[eax], dx
	btr ebx, edx
	btr dword ptr[ecx], edx
	btr bx, 55h
	btr word ptr[eax], 55h
	btr ebx, 55h
	btr dword ptr[ecx], 55h
	btr rbx, r11
	btr qword ptr[ecx], r11
	btr rbx, 55h
	btr qword ptr[ecx], 55h
	bts bx, dx
	bts word ptr[eax], dx
	bts ebx, edx
	bts dword ptr[ecx], edx
	bts bx, 55h
	bts word ptr[eax], 55h
	bts ebx, 55h
	bts dword ptr[ecx], 55h
	bts rbx, r11
	bts qword ptr[ecx], r11
	bts rbx, 55h
	bts qword ptr[ecx], 55h
	call rbx
	cbw 
	cwde 
	cdqe 
	clc 
	cld 
	cli 
	clts 
	cmc 
	cmpxchg bl, dl
	cmpxchg byte ptr[esp], dl
	cmpxchg bx, dx
	cmpxchg word ptr[eax], dx
	cmpxchg ebx, edx
	cmpxchg dword ptr[ecx], edx
	cmpxchg rbx, r11
	cmpxchg qword ptr[ecx], r11
	cmpxchg8b qword ptr[ecx]
	cmpxchg16b xmmword ptr[esp]
	cpuid 
	cwd 
	cdq 
	cqo 
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
	;into
	invd
	invlpg dword ptr[esp]
	iret
	iretd
	iretq
	lar bx, dx
	lar bx, word ptr[esp]
	lar ebx, edx
	lar ebx, word ptr[esp]
	lar rbx, rdx
	lar rbx, word ptr[esp]
	leave 
	lldt cx
	lldt word ptr[ecx]
	lmsw cx
	lmsw word ptr[ecx]
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
	movsx rbx, dl
	movsx rbx, byte ptr[esp]
	movsx rbx, dx
	movsx rbx, word ptr[esp]
	movsxd rbx, edx
	movsxd rbx, dword ptr[esp]
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
	;popfd
	popfq
	pushf
	;pushfd
	pushfq
	rdmsr
	rdpmc
	rdtsc
	ret
	ret 1
	ret -1
	rsm
	scasb 
	scasw 
	scasd 
	scasq 
	sgdt dword ptr[esp]
	shld bx, dx, 1
	shld word ptr[esp], dx, 1
	shld bx, dx, cl
	shld word ptr[esp], dx, cl
	shld ebx, edx, 1
	shld dword ptr[esp], edx, 1
	shld ebx, edx, cl
	shld dword ptr[esp], edx, cl
	shld rbx, rdx, 1
	shld qword ptr[esp], rdx, 1
	shld rbx, rdx, cl
	shld qword ptr[esp], rdx, cl
	shrd bx, dx, 1
	shrd word ptr[esp], dx, 1
	shrd bx, dx, cl
	shrd word ptr[esp], dx, cl
	shrd ebx, edx, 1
	shrd dword ptr[esp], edx, 1
	shrd ebx, edx, cl
	shrd dword ptr[esp], edx, cl
	shrd rbx, rdx, 1
	shrd qword ptr[esp], rdx, 1
	shrd rbx, rdx, cl
	shrd qword ptr[esp], rdx, cl
	sidt dword ptr[esp]
	sldt bx
	sldt word ptr[esp]
	sldt r8
	smsw bx
	smsw word ptr[esp]
	smsw r8
	stc 
	std 
	sti 
	;sysenter
	;sysexit
	swapgs
	syscall
	sysret
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
	xadd rbx, rdx
	xadd qword ptr[esp], rdx
	wbinvd
	wrmsr
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
	db 67h	;fsave [edi]
	db 9Bh
	db 0DDh
	db 37h
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
	db 67h	;fstcw word ptr[esp]
	db 9Bh
	db 0D9h
	db 3Ch
	db 24h
	fnstcw word ptr[esp]
	db 67h	;fstenv [ebp]
	db 9Bh
	db 0D9h
	db 75h
	db 00h
	fnstenv [ebp]
	db 67h	;fstsw word ptr[esp]
	db 9Bh
	db 0DDh
	db 3Ch
	db 24h	
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

	packsswb mm1, qword ptr[r8]
	packssdw mm1, qword ptr[r8]
	packuswb mm1, qword ptr[r8]
	paddb mm1, qword ptr[r8]
	paddw mm1, qword ptr[r8]
	paddd mm1, qword ptr[r8]
	paddsb mm1, qword ptr[r8]
	paddsw mm1, qword ptr[r8]
	paddusb mm1, qword ptr[r8]
	paddusw mm1, qword ptr[r8]
	pand mm1, qword ptr[r8]
	pandn mm1, qword ptr[r8]
	pcmpeqb mm1, qword ptr[r8]
	pcmpeqw mm1, qword ptr[r8]
	pcmpeqd mm1, qword ptr[r8]
	pcmpgtb mm1, qword ptr[r8]
	pcmpgtw mm1, qword ptr[r8]
	pcmpgtd mm1, qword ptr[r8]
	pmaddwd mm1, qword ptr[r8]
	pmulhw mm1, qword ptr[r8]
	pmullw mm1, qword ptr[r8]
	por mm1, qword ptr[r8]
	psllw mm1, qword ptr[r8 + rcx]
	pslld mm1, qword ptr[r8 + rcx]
	psllq mm1, qword ptr[r8 + rcx]
	psraw mm1, qword ptr[r8 + rcx]
	psrad mm1, qword ptr[r8 + rcx]
	psrlw mm1, qword ptr[r8 + rcx]
	psrld mm1, qword ptr[r8 + rcx]
	psrlq mm1, qword ptr[r8 + rcx]
	psubb mm1, qword ptr[r8]
	psubw mm1, qword ptr[r8]
	psubd mm1, qword ptr[r8]
	psubsb mm1, qword ptr[r8]
	psubsw mm1, qword ptr[r8]
	psubusb mm1, qword ptr[r8]
	psubusw mm1, qword ptr[r8]
	punpckhbw mm2, qword ptr[rsp]
	punpckhwd mm2, qword ptr[rsp]
	punpckhdq mm2, qword ptr[rsp]
	punpcklbw mm2, dword ptr[rsp]
	punpcklwd mm2, dword ptr[rsp]
	punpckldq mm2, dword ptr[rsp]
	pxor mm2, qword ptr[rsp]
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

	pavgb mm1, qword ptr[rbp]
	pavgw mm1, qword ptr[rbp]
	pextrw rax, mm2, 2
	pextrw r9, mm2, 3
	pextrw r10d, mm2, 0
	pinsrw mm1, rcx, 3
	pinsrw mm1, r9, 0
	pinsrw mm1, r10d, 1
	pmaxsw mm1, qword ptr[rbp]
	pmaxub mm1, qword ptr[rbp]
	pminsw mm1, qword ptr[rbp]
	pminub mm1, qword ptr[rbp]
	pmovmskb rcx, mm2
	pmovmskb r9, mm2
	pmulhuw mm1, qword ptr[rbp]
	psadbw mm1, qword ptr[rbp]
	pshufw mm1, qword ptr[r9], 1
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

	addps xmm8, xmm9
	addps xmm8, xmmword ptr[rbp]
	addss xmm8, xmm9
	addss xmm8, dword ptr[r9]
	andps xmm8, xmm9
	andps xmm8, xmmword ptr[rbp]
	andnps xmm8, xmm9
	andnps xmm8, xmmword ptr[rbp]
	cmpeqps xmm8, xmm9
	cmpeqps xmm8, xmmword ptr[rbp]
	cmpltps xmm8, xmm9
	cmpltps xmm8, xmmword ptr[rbp]
	cmpleps xmm8, xmm9
	cmpleps xmm8, xmmword ptr[rbp]
	cmpunordps xmm8, xmm9
	cmpunordps xmm8, xmmword ptr[rbp]
	cmpneqps xmm8, xmm9
	cmpneqps xmm8, xmmword ptr[rbp]
	cmpnltps xmm8, xmm9
	cmpnltps xmm8, xmmword ptr[rbp]
	cmpnleps xmm8, xmm9
	cmpnleps xmm8, xmmword ptr[rbp]
	cmpordps xmm8, xmm9
	cmpordps xmm8, xmmword ptr[rbp]
	cmpeqss xmm8, xmm9
	cmpeqss xmm8, dword ptr[r9]
	cmpltss xmm8, xmm9
	cmpltss xmm8, dword ptr[r9]
	cmpless xmm8, xmm9
	cmpless xmm8, dword ptr[r9]
	cmpunordss xmm8, xmm9
	cmpunordss xmm8, dword ptr[r9]
	cmpneqss xmm8, xmm9
	cmpneqss xmm8, dword ptr[r9]
	cmpnltss xmm8, xmm9
	cmpnltss xmm8, dword ptr[r9]
	cmpnless xmm8, xmm9
	cmpnless xmm8, dword ptr[r9]
	cmpordss xmm8, xmm9
	cmpordss xmm8, dword ptr[r9]
	comiss xmm8, xmm9
	comiss xmm8, dword ptr[r9]
	cvtpi2ps xmm8, mm3
	cvtpi2ps xmm8, qword ptr[r9]
	cvtps2pi mm1, xmm9
	cvtps2pi mm1, qword ptr[r9]
	cvtsi2ss xmm8, ecx
	cvtsi2ss xmm8, dword ptr[r9]
	cvtss2si eax, xmm9
	cvtss2si ecx, dword ptr[r9]
	cvttps2pi mm1, xmm9
	cvttps2pi mm1, qword ptr[r9]
	cvttss2si eax, xmm9
	cvttss2si ecx, dword ptr[r9]
	cvtsi2ss xmm8, rax
	cvtsi2ss xmm8, r9
	cvtsi2ss xmm8, qword ptr[r9]
	cvtss2si r9d, xmm9
	cvtss2si rcx, xmm9
	cvtss2si r8, xmm9
	cvtss2si rax, dword ptr[r9]
	cvttss2si r9, xmm9
	cvttss2si r10, dword ptr[r9]
	divps xmm8, xmm9
	divps xmm8, xmmword ptr[rbp]
	divss xmm8, xmm9
	divss xmm8, dword ptr[r9]
	ldmxcsr dword ptr[r9]
	maskmovq mm1, mm2
	maxps xmm8, xmm9
	maxps xmm8, xmmword ptr[rbp]
	maxss xmm8, xmm9
	maxss xmm8, dword ptr[r9]
	minps xmm8, xmm9
	minps xmm8, xmmword ptr[rbp]
	minss xmm8, xmm9
	minss xmm8, dword ptr[r9]
	movaps xmm8, xmm9
	movaps xmm8, xmmword ptr[rbp]
	movaps xmmword ptr[rsp], xmm9
	movhlps xmm8, xmm9
	movhps xmm8, qword ptr[r9]
	movhps qword ptr[rdi], xmm9
	movlhps xmm8, xmm9
	movlps xmm8, qword ptr[r9]
	movlps qword ptr[rdi], xmm9
	movmskps eax, xmm9
	movmskps r9d, xmm9
	movmskps rax, xmm9
	movmskps r9, xmm3
	movntps xmmword ptr[rsp], xmm9
	movntq qword ptr[rdi], mm7
	movss xmm8, xmm9
	movss xmm8, dword ptr[r9]
	movss dword ptr[rbp], xmm9
	movups xmm8, xmm9
	movups xmm8, xmmword ptr[rbp]
	movups xmmword ptr[rsp], xmm9
	mulps xmm8, xmm9
	mulps xmm8, xmmword ptr[rbp]
	mulss xmm8, xmm9
	mulss xmm8, dword ptr[r9]
	orps xmm8, xmm9
	orps xmm8, xmmword ptr[rbp]
	prefetcht0 byte ptr[rbp]
	prefetcht1 byte ptr[rbp]
	prefetcht2 byte ptr[rbp]
	prefetchnta byte ptr[rbp]
	rcpps xmm8, xmm9
	rcpps xmm8, xmmword ptr[rbp]
	rcpss xmm8, xmm9
	rcpss xmm8, dword ptr[r9]
	rsqrtps xmm8, xmm9
	rsqrtps xmm8, xmmword ptr[rbp]
	rsqrtss xmm8, xmm9
	rsqrtss xmm8, dword ptr[r9]
	sfence 
	shufps xmm8, xmm9, 10h
	shufps xmm8, xmmword ptr[rbp], 20h
	sqrtps xmm8, xmm9
	sqrtps xmm8, xmmword ptr[rbp]
	sqrtss xmm8, xmm9
	sqrtss xmm8, dword ptr[r9]
	stmxcsr dword ptr[r9]
	subps xmm8, xmm9
	subps xmm8, xmmword ptr[rbp]
	subss xmm8, xmm9
	subss xmm8, dword ptr[r9]
	ucomiss xmm8, xmm9
	ucomiss xmm8, dword ptr[r9]
	unpckhps xmm8, xmm9
	unpckhps xmm8, xmmword ptr[rbp]
	unpcklps xmm8, xmm9
	unpcklps xmm8, xmmword ptr[rbp]
	xorps xmm8, xmm9
	xorps xmm8, xmmword ptr[rbp]
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
	cvtsd2si r8d, xmm9
	cvtsd2si r9d, qword ptr[r8]
	cvtsd2si r8, xmm9
	cvtsd2si r9, qword ptr[r8]
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
	movupd xmm0, xmm1
	movupd xmm0, xmmword ptr[esp]
	movupd xmmword ptr[esp], xmm0
	mulpd xmm0, xmm1
	mulpd xmm0, xmmword ptr[esp]
	mulsd xmm0, xmm1
	mulsd xmm0, qword ptr[esp]
	orpd xmm0, xmm1
	orpd xmm0, xmmword ptr[esp]

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
	movupd xmm8, xmm9
	movupd xmm8, xmmword ptr[r8]
	movupd xmmword ptr[r8], xmm8
	mulpd xmm8, xmm9
	mulpd xmm8, xmmword ptr[r8]
	mulsd xmm8, xmm9
	mulsd xmm8, qword ptr[r8]
	orpd xmm8, xmm9
	orpd xmm8, xmmword ptr[r8]
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

	packsswb xmm8, xmm9
	packsswb xmm8, xmmword ptr[r8]
	packssdw xmm8, xmm9
	packssdw xmm8, xmmword ptr[r8]
	packuswb xmm8, xmm9
	packuswb xmm8, xmmword ptr[r8]
	paddb xmm8, xmm9
	paddb xmm8, xmmword ptr[r8]
	paddw xmm8, xmm9
	paddw xmm8, xmmword ptr[r8]
	paddd xmm8, xmm9
	paddd xmm8, xmmword ptr[r8]
	paddq mm0, mm1
	paddq mm0, qword ptr[r8]
	paddq xmm8, xmm9
	paddq xmm8, xmmword ptr[r8]
	paddsb xmm8, xmm9
	paddsb xmm8, xmmword ptr[r8]
	paddsw xmm8, xmm9
	paddsw xmm8, xmmword ptr[r8]
	paddusb xmm8, xmm9
	paddusb xmm8, xmmword ptr[r8]
	paddusw xmm8, xmm9
	paddusw xmm8, xmmword ptr[r8]
	pand xmm8, xmm9
	pand xmm8, xmmword ptr[r8]
	pandn xmm8, xmm9
	pandn xmm8, xmmword ptr[r8]
	pause 
	pavgb xmm8, xmm9
	pavgb xmm8, xmmword ptr[r8]
	pavgw xmm8, xmm9
	pavgw xmm8, xmmword ptr[r8]
	pcmpeqb xmm8, xmm9
	pcmpeqb xmm8, xmmword ptr[r8]
	pcmpeqw xmm8, xmm9
	pcmpeqw xmm8, xmmword ptr[r8]
	pcmpeqd xmm8, xmm9
	pcmpeqd xmm8, xmmword ptr[r8]
	pcmpgtb xmm8, xmm9
	pcmpgtb xmm8, xmmword ptr[r8]
	pcmpgtw xmm8, xmm9
	pcmpgtw xmm8, xmmword ptr[r8]
	pcmpgtd xmm8, xmm9
	pcmpgtd xmm8, xmmword ptr[r8]
	pextrw eax, xmm9, 1
	pextrw rax, xmm9, 2
	pextrw r8, xmm9, 2
	pinsrw xmm8, ecx, 3
	pinsrw xmm8, word ptr[r8], 4
	pinsrw xmm8, rcx, 1
	pinsrw xmm8, r8, 1
	pmaddwd xmm8, xmm9
	pmaddwd xmm8, xmmword ptr[r8]
	pmaxsw xmm8, xmm9
	pmaxsw xmm8, xmmword ptr[r8]
	pmaxub xmm8, xmm9
	pmaxub xmm8, xmmword ptr[r8]
	pminsw xmm8, xmm9
	pminsw xmm8, xmmword ptr[r8]
	pminub xmm8, xmm9
	pminub xmm8, xmmword ptr[r8]
	pmovmskb eax, xmm9
	pmovmskb rax, xmm9
	pmovmskb r8, xmm9
	pmulhuw xmm8, xmm9
	pmulhuw xmm8, xmmword ptr[r8]
	pmulhw xmm8, xmm9
	pmulhw xmm8, xmmword ptr[r8]
	pmullw xmm8, xmm9
	pmullw xmm8, xmmword ptr[r8]
	pmuludq mm0, mm1
	pmuludq mm0, qword ptr[r8]
	pmuludq xmm8, xmm9
	pmuludq xmm8, xmmword ptr[r8]
	por xmm8, xmm9
	por xmm8, xmmword ptr[r8]
	psadbw xmm8, xmm9
	psadbw xmm8, xmmword ptr[r8]
	pshufd xmm8, xmm9, 1Bh
	pshufd xmm8, xmmword ptr[r8], 0Bh
	pshufhw xmm8, xmm9, 1Ah
	pshufhw xmm8, xmmword ptr[r8], 18h
	pshuflw xmm8, xmm9, 14h
	pshuflw xmm8, xmmword ptr[r8], 12h
	psllw xmm8, xmm9
	psllw xmm8, xmmword ptr[rbp]
	psllw xmm8, 2
	pslld xmm8, xmm9
	pslld xmm8, xmmword ptr[rbp]
	pslld xmm8, 2
	psllq xmm8, xmm9
	psllq xmm8, xmmword ptr[rbp]
	psllq xmm8, 2
	pslldq xmm8, 2
	psraw xmm8, xmm9
	psraw xmm8, xmmword ptr[rbp]
	psraw xmm8, 2
	psrad xmm8, xmm9
	psrad xmm8, xmmword ptr[rbp]
	psrad xmm8, 2
	psrlw xmm8, xmm9
	psrlw xmm8, xmmword ptr[rbp]
	psrlw xmm8, 2
	psrld xmm8, xmm9
	psrld xmm8, xmmword ptr[rbp]
	psrld xmm8, 2
	psrlq xmm8, xmm9
	psrlq xmm8, xmmword ptr[rbp]
	psrlq xmm8, 2
	psrldq xmm8, 2
	psubb xmm8, xmm9
	psubb xmm8, xmmword ptr[r8]
	psubw xmm8, xmm9
	psubw xmm8, xmmword ptr[r8]
	psubd xmm8, xmm9
	psubd xmm8, xmmword ptr[r8]
	psubq mm0, mm1
	psubq mm0, qword ptr[r8]
	psubq xmm8, xmm9
	psubq xmm8, xmmword ptr[r8]
	psubsb xmm8, xmm9
	psubsb xmm8, xmmword ptr[r8]
	psubsw xmm8, xmm9
	psubsw xmm8, xmmword ptr[r8]
	psubusb xmm8, xmm9
	psubusb xmm8, xmmword ptr[r8]
	psubusw xmm8, xmm9
	psubusw xmm8, xmmword ptr[r8]
	punpckhbw xmm8, xmm9
	punpckhbw xmm8, xmmword ptr[r8]
	punpckhwd xmm8, xmm9
	punpckhwd xmm8, xmmword ptr[r8]
	punpckhdq xmm8, xmm9
	punpckhdq xmm8, xmmword ptr[r8]
	punpckhqdq xmm8, xmm9
	punpckhqdq xmm8, xmmword ptr[r8]
	punpcklbw xmm8, xmm9
	punpcklbw xmm8, xmmword ptr[r8]
	punpcklwd xmm8, xmm9
	punpcklwd xmm8, xmmword ptr[r8]
	punpckldq xmm8, xmm9
	punpckldq xmm8, xmmword ptr[r8]
	punpcklqdq xmm8, xmm9
	punpcklqdq xmm8, xmmword ptr[r8]
	pxor xmm8, xmm9
	pxor xmm8, xmmword ptr[r8]
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

	shufpd xmm8, xmm9, 2
	shufpd xmm8, xmmword ptr[r8], 1
	sqrtpd xmm8, xmm9
	sqrtpd xmm8, xmmword ptr[r8]
	sqrtsd xmm8, xmm9
	sqrtsd xmm8, qword ptr[rbp]
	subpd xmm8, xmm9
	subpd xmm8, xmmword ptr[r8]
	subsd xmm8, xmm9
	subsd xmm8, qword ptr[rbp]
	ucomisd xmm8, xmm9
	ucomisd xmm8, qword ptr[rbp]
	unpckhpd xmm8, xmm9
	unpckhpd xmm8, xmmword ptr[r8]
	unpcklpd xmm8, xmm9
	unpcklpd xmm8, xmmword ptr[r8]
	xorpd xmm8, xmm9
	xorpd xmm8, xmmword ptr[r8]
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
	monitor rax,rcx,rdx
	mwait rax,rcx

	addsubps xmm9, xmm10
	addsubps xmm9, mmword ptr[r9]		; it must be xmmword...
	addsubpd xmm9, xmm10
	addsubpd xmm9, xmmword ptr[r9]
	fisttp word ptr[r9]
	fisttp dword ptr[r9]
	fisttp qword ptr[r9]
	haddps xmm9, xmm10
	haddps xmm9, xmmword ptr[r9]
	haddpd xmm9, xmm10
	haddpd xmm9, xmmword ptr[r9]
	hsubps xmm9, xmm10
	hsubps xmm9, xmmword ptr[r9]
	hsubpd xmm9, xmm10
	hsubpd xmm9, xmmword ptr[r9]
	lddqu xmm9, xmmword ptr[r9]
	movddup xmm9, xmm10
	movddup xmm9, qword ptr[rbp]
	movshdup xmm9, xmm10
	movshdup xmm9, xmmword ptr[r9]
	movsldup xmm9, xmm10
	movsldup xmm9, xmmword ptr[r9]
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

	blendps xmm8, xmm9, 1
	blendps xmm8, xmmword ptr[r8], 2
	blendpd xmm8, xmm9, 3
	blendpd xmm8, xmmword ptr[r8], 4
	blendvps xmm8, xmm9, xmm0
	blendvps xmm8, xmmword ptr[r8], xmm0
	blendvpd xmm8, xmm9, xmm0
	blendvpd xmm8, xmmword ptr[r8], xmm0
	dpps xmm8, xmm9, 1
	dpps xmm8, xmmword ptr[r8], 2
	dppd xmm8, xmm9, 0
	dppd xmm8, xmmword ptr[r8], 1
	extractps r9d, xmm10, 1
	extractps dword ptr[r8], xmm9, 0
	extractps rcx, xmm10, 2
	extractps r8, xmm9, 3
	insertps xmm8, xmm9, 68h
	insertps xmm8, dword ptr[r8], 0
	movntdqa xmm8, xmmword ptr[r8]
	mpsadbw xmm8, xmm9, 1
	mpsadbw xmm8, xmmword ptr[r8], 4
	packusdw xmm8, xmm9
	packusdw xmm8, xmmword ptr[r8]
	pblendvb xmm8, xmm9, xmm0
	pblendvb xmm8, xmmword ptr[r8], xmm0
	pblendw xmm8, xmm9, 3
	pblendw xmm8, xmmword ptr[r8], 4
	pcmpeqq xmm8, xmm9
	pcmpeqq xmm8, xmmword ptr[r8]
	pextrb r10d, xmm9, 1
	pextrb byte ptr[r8], xmm9, 2
	pextrw word ptr[r8], xmm9, 1
	pextrd r9d, xmm9, 3
	pextrd dword ptr[r8], xmm9, 2
	pextrb rax, xmm9, 2
	pextrq r9, xmm9, 1
	pextrq qword ptr[r8], xmm9, 1
	pinsrb xmm8, r8d, 0
	pinsrb xmm8, byte ptr[r8], 2
	pinsrd xmm8, r9d, 1
	pinsrd xmm8, dword ptr[r8], 0
	pinsrb xmm8, rax, 10
	pinsrq xmm8, r10, 1
	pinsrq xmm8, qword ptr[r8], 0
	pmaxsb xmm8, xmm9
	pmaxsb xmm8, xmmword ptr[r8]
	pmaxsd xmm8, xmm9
	pmaxsd xmm8, xmmword ptr[r8]
	pmaxuw xmm8, xmm9
	pmaxuw xmm8, xmmword ptr[r8]
	pmaxud xmm8, xmm9
	pmaxud xmm8, xmmword ptr[r8]
	pminsb xmm8, xmm9
	pminsb xmm8, xmmword ptr[r8]
	pminsd xmm8, xmm9
	pminsd xmm8, xmmword ptr[r8]
	pminuw xmm8, xmm9
	pminuw xmm8, xmmword ptr[r8]
	pminud xmm8, xmm9
	pminud xmm8, xmmword ptr[r8]
	; ML 9.00 generates wrong opcode for pmovsx/pmovsz
	; http://connect.microsoft.com/VisualStudio/feedback/ViewFeedback.aspx?FeedbackID=377701
	pmovsxbd xmm8, xmm9				; pmovsxbw
	pmovsxbd xmm8, qword ptr[r8]	; pmovsxbw
	pmovsxbq xmm8, xmm9				; pmovsxbd
	pmovsxbq xmm8, dword ptr[r8]	; pmovsxbd
	pmovsxbw xmm8, xmm9				; pmovsxbq
	pmovsxbw xmm8, word ptr[r8]		; pmovsxbq
	pmovsxdq xmm8, xmm9				; pmovsxwd
	pmovsxdq xmm8, qword ptr[r8]	; pmovsxwd
	pmovsxwd xmm8, xmm9				; pmovsxwq
	pmovsxwd xmm8, dword ptr[r8]	; pmovsxwq
	pmovsxwq xmm8, xmm9				; pmovsxdq
	pmovsxwq xmm8, qword ptr[r8]	; pmovsxdq
	pmovzxbd xmm8, xmm9				; pmovzxbw
	pmovzxbd xmm8, qword ptr[r8]	; pmovzxbw
	pmovzxbq xmm8, xmm9				; pmovzxbd
	pmovzxbq xmm8, dword ptr[r8]	; pmovzxbd
	pmovzxbw xmm8, xmm9				; pmovzxbq
	pmovzxbw xmm8, word ptr[r8]		; pmovzxbq
	pmovzxdq xmm8, xmm9				; pmovzxwd
	pmovzxdq xmm8, qword ptr[r8]	; pmovzxwd
	pmovzxwd xmm8, xmm9				; pmovzxwq
	pmovzxwd xmm8, dword ptr[r8]	; pmovzxwq
	pmovzxwq xmm8, xmm9				; pmovzxdq
	pmovzxwq xmm8, qword ptr[r8]	; pmovzxdq

	pmuldq xmm8, xmm9
	pmuldq xmm8, xmmword ptr[r8]
	pmulld xmm8, xmm9
	pmulld xmm8, xmmword ptr[r8]
	ptest xmm8, xmm9
	ptest xmm8, xmmword ptr[r8]
	roundps xmm8, xmm9, 0
	roundps xmm8, xmmword ptr[r8], 1
	roundpd xmm8, xmm9, 2
	roundpd xmm8, xmmword ptr[r8], 3
	roundss xmm8, xmm9, 0
	roundss xmm8, dword ptr[r8], 1
	roundsd xmm8, xmm9, 2
	roundsd xmm8, qword ptr[r8], 3
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

	crc32 eax, r9b
	crc32 r8d, byte ptr[rsi]
	crc32 eax, r9w
	crc32 r8d, word ptr[esp]
	crc32 eax, r9d
	crc32 r8d, dword ptr[rsi]
	crc32 rax, bl
	crc32 r8, byte ptr[rsi]
	crc32 r9, r10
	crc32 r9, qword ptr[rsi]
	pcmpestri xmm10, xmm9, 0
	pcmpestri xmm10, xmmword ptr[r8d], 0
	pcmpestrm xmm10, xmm9, 1
	pcmpestrm xmm10, xmmword ptr[esi], 1
	pcmpistri xmm10, xmm9, 0
	pcmpistri xmm10, xmmword ptr[esi], 0
	pcmpistrm xmm10, xmm9, 1
	pcmpistrm xmm10, xmmword ptr[esi], 1
	pcmpgtq xmm8, xmm9
	pcmpgtq xmm8, xmmword ptr[esi]
	popcnt r8w, cx
	popcnt r9w, word ptr[esp]
	popcnt r8d, r9d
	popcnt r8d, dword ptr[rsi]
	popcnt r9, rax
	popcnt r9, qword ptr[rsi]
masm_test_sse4_2 endp

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
; function1_cdecl<float, float> (return ptr)
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
; function1_cdecl<float, float> (return st(0))
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
; function1_cdecl<double, double> (return ptr)
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
; function1_cdecl<double, double> (return st(0))
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

;----------------------------------------
; function1_cdecl<__m64, int> (return mm1)
;----------------------------------------
masm_test_function_return_m64_mm1 proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	movd mm1, dword ptr[rbp + 16]
	punpckldq mm1, mm1
	paddw mm1, mm1
	movd rax, mm1
	leave
	ret
masm_test_function_return_m64_mm1 endp

;----------------------------------------
; function1_cdecl<__m64> (return ptr)
;----------------------------------------
masm_test_function_return_m64_ptr proc
	push rbp
	mov rbp, rsp
	mov rax, qword ptr[rsp - 8]
	leave
	ret
masm_test_function_return_m64_ptr endp

;----------------------------------------
; function0_cdecl<__m128> (return xmm1)
;----------------------------------------
masm_test_function_return_m128_xmm1 proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	pxor xmm1, xmm1
	mov rax, qword ptr[rbp + 16]
	movaps xmmword ptr[rax], xmm1
	leave
	ret
masm_test_function_return_m128_xmm1 endp

;----------------------------------------
; function0_cdecl<__m128> (return ptr)
;----------------------------------------
masm_test_function_return_m128_ptr proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	mov rax, qword ptr[rbp + 16]
	movaps xmm0, xmmword ptr[rsp - 16]
	movaps xmmword ptr[rax], xmm0
	leave
	ret
masm_test_function_return_m128_ptr endp

;----------------------------------------
; function0_cdecl<__m128d> (return xmm1)
;----------------------------------------
masm_test_function_return_m128d_xmm1 proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	pxor xmm1, xmm1
	mov rax, qword ptr[rbp + 16]
	movapd xmmword ptr[rax], xmm1
	leave
	ret
masm_test_function_return_m128d_xmm1 endp

;----------------------------------------
; function0_cdecl<__m128d> (return ptr)
;----------------------------------------
masm_test_function_return_m128d_ptr proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	mov rax, qword ptr[rbp + 16]
	movapd xmm0, xmmword ptr[rsp - 16]
	movapd xmmword ptr[rax], xmm0
	leave
	ret
masm_test_function_return_m128d_ptr endp

;----------------------------------------
; function0_cdecl<__m128i> (return xmm1)
;----------------------------------------
masm_test_function_return_m128i_xmm1 proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	pxor xmm1, xmm1
	mov rax, qword ptr[rbp + 16]
	movdqa xmmword ptr[rax], xmm1
	leave
	ret
masm_test_function_return_m128i_xmm1 endp

;----------------------------------------
; function0_cdecl<__m128i> (return ptr)
;----------------------------------------
masm_test_function_return_m128i_ptr proc
	push rbp
	mov rbp, rsp
	mov qword ptr[rbp + 16], rcx
	mov rax, qword ptr[rbp + 16]
	movdqa xmm0, xmmword ptr[rsp - 16]
	movdqa xmmword ptr[rax], xmm0
	leave
	ret
masm_test_function_return_m128i_ptr endp

end
