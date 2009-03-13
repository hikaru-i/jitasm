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

	mov rax, [1]
	mov rax, [qword 100000000h]
