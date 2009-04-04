section .text

global	nasm_test_mov_disp
align	16
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