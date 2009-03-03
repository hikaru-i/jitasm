#include <tchar.h>
#include <assert.h>
#include "jitasm.h"

#define ASSERT	assert

template<class Func>
void TEST(void (*func1)(), Func func2)
{
	func2.Assemble();

	unsigned char* p1 = (unsigned char*) func1;
	if (*p1 == 0xE9) p1 = (unsigned char*) (p1 + (unsigned long&) *(p1 + 1) + 5);	// Strip thunk
	unsigned char* p2 = (unsigned char*) func2.GetCode();

	size_t size = func2.GetCodeSize();
	for (size_t i = 0; i < size; i++) {
		ASSERT(p1[i] == p2[i]);
	}
}

struct test_mmx_sse2 : jitasm::function0<void>
{
	virtual void main()
	{
#ifdef JITASM64
		movdqa(xmm0, xmm1);
		movdqa(xmm8, xmm1);
		movdqa(xmm0, xmm9);
		movdqa(xmm0, xmmword_ptr[ecx]);
		movdqa(xmm0, xmmword_ptr[rcx]);
		movdqa(xmm8, xmmword_ptr[ecx]);
		movdqa(xmm8, xmmword_ptr[rcx]);
		movdqa(xmmword_ptr[eax], xmm1);
		movdqa(xmmword_ptr[rax], xmm1);
		movdqa(xmmword_ptr[eax], xmm9);
		movdqa(xmmword_ptr[rax], xmm9);

		movdqu(xmm0, xmm1);
		movdqu(xmm8, xmm1);
		movdqu(xmm0, xmm9);
		movdqu(xmm0, xmmword_ptr[ecx]);
		movdqu(xmm0, xmmword_ptr[rcx]);
		movdqu(xmm8, xmmword_ptr[ecx]);
		movdqu(xmm8, xmmword_ptr[rcx]);
		movdqu(xmmword_ptr[eax], xmm1);
		movdqu(xmmword_ptr[rax], xmm1);
		movdqu(xmmword_ptr[eax], xmm9);
		movdqu(xmmword_ptr[rax], xmm9);

		pxor(xmm0, xmm1);
		pxor(xmm8, xmm1);
		pxor(xmm0, xmm9);
		pxor(xmm0, xmmword_ptr[ecx]);
		pxor(xmm0, xmmword_ptr[rcx]);
		pxor(xmm8, xmmword_ptr[ecx]);
		pxor(xmm8, xmmword_ptr[rcx]);
#else
		movdqa(xmm0, xmm1);
		movdqa(xmm0, xmmword_ptr[ecx]);
		movdqa(xmmword_ptr[eax], xmm1);

		movdqu(xmm0, xmm1);
		movdqu(xmm0, xmmword_ptr[ecx]);
		movdqu(xmmword_ptr[eax], xmm1);

		pabsb(mm0, mm1);
		pabsb(mm0, mmword_ptr[ecx]);
		pabsb(xmm0, xmm1);
		pabsb(xmm0, xmmword_ptr[ecx]);
		pabsw(mm0, mm1);
		pabsw(mm0, mmword_ptr[ecx]);
		pabsw(xmm0, xmm1);
		pabsw(xmm0, xmmword_ptr[ecx]);
		pabsd(mm0, mm1);
		pabsd(mm0, mmword_ptr[ecx]);
		pabsd(xmm0, xmm1);
		pabsd(xmm0, xmmword_ptr[ecx]);

		packsswb(mm0, mm1);
		packsswb(mm0, mmword_ptr[ecx]);
		packsswb(xmm0, xmm1);
		packsswb(xmm0, xmmword_ptr[ecx]);
		packssdw(mm0, mm1);
		packssdw(mm0, mmword_ptr[ecx]);
		packssdw(xmm0, xmm1);
		packssdw(xmm0, xmmword_ptr[ecx]);
		packuswb(mm0, mm1);
		packuswb(mm0, mmword_ptr[ecx]);
		packuswb(xmm0, xmm1);
		packuswb(xmm0, xmmword_ptr[ecx]);
		packusdw(xmm0, xmm1);
		packusdw(xmm0, xmmword_ptr[ecx]);

		paddb(mm0, mm1);
		paddb(mm0, mmword_ptr[ecx]);
		paddb(xmm0, xmm1);
		paddb(xmm0, xmmword_ptr[ecx]);
		paddw(mm0, mm1);
		paddw(mm0, mmword_ptr[ecx]);
		paddw(xmm0, xmm1);
		paddw(xmm0, xmmword_ptr[ecx]);
		paddd(mm0, mm1);
		paddd(mm0, mmword_ptr[ecx]);
		paddd(xmm0, xmm1);
		paddd(xmm0, xmmword_ptr[ecx]);

		pxor(mm0, mm1);
		pxor(mm0, mmword_ptr[ecx]);
		pxor(xmm0, xmm1);
		pxor(xmm0, xmmword_ptr[ecx]);
#endif
	}
};

struct test_func : jitasm::function0<int>
{
	virtual Result main()
	{
		xor(zax, zax);
		mov(zcx, 100);
	label("loop_beg");
		cmp(zcx, 0);
		jle("loop_end");
		add(zax, zcx);
		sub(zcx, 1);
		jmp("loop_beg");
	label("loop_end");
#if 0
#ifndef JITASM64
		push(zax);
		push((intptr_t) "%d");
		mov(zcx, (intptr_t) printf);
		call(zcx);
		add(zsp, sizeof(intptr_t) * 2);
#else
		sub(zsp, 40);
		mov(zcx, (intptr_t) "%d");
		mov(zdx, zax);
		mov(zax, (intptr_t) printf);
		call(zax);
		add(zsp, 40);
#endif
#endif
		ret();
		return eax;
	}
};

//----------------------------------------
// SAL
//----------------------------------------
extern "C" void masm_test_sal();
struct test_sal : jitasm::function0<void>
{
	virtual void naked_main()
	{
		sal(al, 1);
		sal(al, 2);
		sal(al, -1);
		sal(ax, 1);
		sal(ax, 2);
		sal(eax, 1);
		sal(eax, 2);
		sal(byte_ptr[eax], 1);
		sal(byte_ptr[eax], 2);
		sal(word_ptr[eax], 1);
		sal(word_ptr[eax], 2);
		sal(dword_ptr[eax], 1);
		sal(dword_ptr[eax], 2);
#ifdef JITASM64
		sal(r8b, 1);
		sal(r8b, 2);
		sal(r8w, 1);
		sal(r8w, 2);
		sal(r8d, 1);
		sal(r8d, 2);
		sal(r8, 1);
		sal(r8, 2);
		sal(qword_ptr[rax], 1);
		sal(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// SAR
//----------------------------------------
extern "C" void masm_test_sar();
struct test_sar : jitasm::function0<void>
{
	virtual void naked_main()
	{
		sar(al, 1);
		sar(al, 2);
		sar(al, -1);
		sar(ax, 1);
		sar(ax, 2);
		sar(eax, 1);
		sar(eax, 2);
		sar(byte_ptr[eax], 1);
		sar(byte_ptr[eax], 2);
		sar(word_ptr[eax], 1);
		sar(word_ptr[eax], 2);
		sar(dword_ptr[eax], 1);
		sar(dword_ptr[eax], 2);
#ifdef JITASM64
		sar(r8b, 1);
		sar(r8b, 2);
		sar(r8w, 1);
		sar(r8w, 2);
		sar(r8d, 1);
		sar(r8d, 2);
		sar(r8, 1);
		sar(r8, 2);
		sar(qword_ptr[rax], 1);
		sar(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// SHL
//----------------------------------------
extern "C" void masm_test_shl();
struct test_shl : jitasm::function0<void>
{
	virtual void naked_main()
	{
		shl(al, 1);
		shl(al, 2);
		shl(al, -1);
		shl(ax, 1);
		shl(ax, 2);
		shl(eax, 1);
		shl(eax, 2);
		shl(byte_ptr[eax], 1);
		shl(byte_ptr[eax], 2);
		shl(word_ptr[eax], 1);
		shl(word_ptr[eax], 2);
		shl(dword_ptr[eax], 1);
		shl(dword_ptr[eax], 2);
#ifdef JITASM64
		shl(r8b, 1);
		shl(r8b, 2);
		shl(r8w, 1);
		shl(r8w, 2);
		shl(r8d, 1);
		shl(r8d, 2);
		shl(r8, 1);
		shl(r8, 2);
		shl(qword_ptr[rax], 1);
		shl(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// SHR
//----------------------------------------
extern "C" void masm_test_shr();
struct test_shr : jitasm::function0<void>
{
	virtual void naked_main()
	{
		shr(al, 1);
		shr(al, 2);
		shr(al, -1);
		shr(ax, 1);
		shr(ax, 2);
		shr(eax, 1);
		shr(eax, 2);
		shr(byte_ptr[eax], 1);
		shr(byte_ptr[eax], 2);
		shr(word_ptr[eax], 1);
		shr(word_ptr[eax], 2);
		shr(dword_ptr[eax], 1);
		shr(dword_ptr[eax], 2);
#ifdef JITASM64
		shr(r8b, 1);
		shr(r8b, 2);
		shr(r8w, 1);
		shr(r8w, 2);
		shr(r8d, 1);
		shr(r8d, 2);
		shr(r8, 1);
		shr(r8, 2);
		shr(qword_ptr[rax], 1);
		shr(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// INC/DEC
//----------------------------------------
extern "C" void masm_test_inc_dec();
struct test_inc_dec : jitasm::function0<void>
{
	virtual void naked_main()
	{
		inc(al);
		inc(ax);
		inc(eax);
		inc(byte_ptr[eax]);
		inc(word_ptr[eax]);
		inc(dword_ptr[eax]);
		dec(al);
		dec(ax);
		dec(eax);
		dec(byte_ptr[eax]);
		dec(word_ptr[eax]);
		dec(dword_ptr[eax]);
#ifdef JITASM64
		inc(r8b);
		inc(r8w);
		inc(r8d);
		inc(rax);
		inc(r8);
		inc(qword_ptr[rax]);
		dec(r8b);
		dec(r8w);
		dec(r8d);
		dec(rax);
		dec(r8);
		dec(qword_ptr[rax]);
#endif
	}
};

//----------------------------------------
// PUSH/POP
//----------------------------------------
extern "C" void masm_test_push_pop();
struct test_push_pop : jitasm::function0<void>
{
	virtual void naked_main()
	{
		push(ax);
		push(word_ptr[eax]);
		push(0x1);
		push(0x100);
		push(0x10000);
		push(-0x1);
		push(-0x100);
		push(-0x10000);
		pop(ax);
		pop(word_ptr[eax]);
#ifndef JITASM64
		push(eax);
		push(dword_ptr[eax]);
		pop(eax);
		pop(dword_ptr[eax]);
#else
		push(rax);
		push(r8);
		push(qword_ptr[rax]);
		push(qword_ptr[r8]);
		pop(rax);
		pop(r8);
		pop(qword_ptr[rax]);
		pop(qword_ptr[r8]);
#endif
	}
};

//----------------------------------------
// ADD
//----------------------------------------
extern "C" void masm_test_add();
struct test_add : jitasm::function0<void>
{
	virtual void naked_main()
	{
		add(al, 0x1);
		add(al, -0x1);
		add(ax, 0x1);
		add(ax, 0x100);
		add(ax, -0x1);
		add(ax, -0x100);
		add(eax, 0x1);
		add(eax, 0x100);
		add(eax, 0x10000);
		add(eax, -0x1);
		add(eax, -0x100);
		add(eax, -0x10000);
		add(cl, 0x1);
		add(cl, -0x1);
		add(cx, 0x1);
		add(cx, 0x100);
		add(cx, -0x1);
		add(cx, -0x100);
		add(ecx, 0x1);
		add(ecx, 0x100);
		add(ecx, 0x10000);
		add(ecx, -0x1);
		add(ecx, -0x100);
		add(ecx, -0x10000);
		add(byte_ptr[eax], 1);
		add(word_ptr[eax], 1);
		add(dword_ptr[eax], 1);
		add(al, byte_ptr[eax]);
		add(ax, word_ptr[eax]);
		add(eax, dword_ptr[eax]);
		add(byte_ptr[eax], al);
		add(word_ptr[eax], ax);
		add(dword_ptr[eax], eax);
#ifdef JITASM64
		add(rax, 0x1);
		add(rax, 0x100);
		add(rax, 0x10000);
		add(rax, -0x1);
		add(rax, -0x100);
		add(rax, -0x10000);
		add(r8, 0x1);
		add(r8, 0x100);
		add(r8, 0x10000);
		add(r8, -0x1);
		add(r8, -0x100);
		add(r8, -0x10000);
		add(qword_ptr[rax], 1);
		add(rax, qword_ptr[rax]);
		add(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// OR
//----------------------------------------
extern "C" void masm_test_or();
struct test_or : jitasm::function0<void>
{
	virtual void naked_main()
	{
		or(al, 0x1);
		or(al, -0x1);
		or(ax, 0x1);
		or(ax, 0x100);
		or(ax, -0x1);
		or(ax, -0x100);
		or(eax, 0x1);
		or(eax, 0x100);
		or(eax, 0x10000);
		or(eax, -0x1);
		or(eax, -0x100);
		or(eax, -0x10000);
		or(cl, 0x1);
		or(cl, -0x1);
		or(cx, 0x1);
		or(cx, 0x100);
		or(cx, -0x1);
		or(cx, -0x100);
		or(ecx, 0x1);
		or(ecx, 0x100);
		or(ecx, 0x10000);
		or(ecx, -0x1);
		or(ecx, -0x100);
		or(ecx, -0x10000);
		or(byte_ptr[eax], 1);
		or(word_ptr[eax], 1);
		or(dword_ptr[eax], 1);
		or(al, byte_ptr[eax]);
		or(ax, word_ptr[eax]);
		or(eax, dword_ptr[eax]);
		or(byte_ptr[eax], al);
		or(word_ptr[eax], ax);
		or(dword_ptr[eax], eax);
#ifdef JITASM64
		or(rax, 0x1);
		or(rax, 0x100);
		or(rax, 0x10000);
		or(rax, -0x1);
		or(rax, -0x100);
		or(rax, -0x10000);
		or(r8, 0x1);
		or(r8, 0x100);
		or(r8, 0x10000);
		or(r8, -0x1);
		or(r8, -0x100);
		or(r8, -0x10000);
		or(qword_ptr[rax], 1);
		or(rax, qword_ptr[rax]);
		or(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// ADC
//----------------------------------------
extern "C" void masm_test_adc();
struct test_adc : jitasm::function0<void>
{
	virtual void naked_main()
	{
		adc(al, 0x1);
		adc(al, -0x1);
		adc(ax, 0x1);
		adc(ax, 0x100);
		adc(ax, -0x1);
		adc(ax, -0x100);
		adc(eax, 0x1);
		adc(eax, 0x100);
		adc(eax, 0x10000);
		adc(eax, -0x1);
		adc(eax, -0x100);
		adc(eax, -0x10000);
		adc(cl, 0x1);
		adc(cl, -0x1);
		adc(cx, 0x1);
		adc(cx, 0x100);
		adc(cx, -0x1);
		adc(cx, -0x100);
		adc(ecx, 0x1);
		adc(ecx, 0x100);
		adc(ecx, 0x10000);
		adc(ecx, -0x1);
		adc(ecx, -0x100);
		adc(ecx, -0x10000);
		adc(byte_ptr[eax], 1);
		adc(word_ptr[eax], 1);
		adc(dword_ptr[eax], 1);
		adc(al, byte_ptr[eax]);
		adc(ax, word_ptr[eax]);
		adc(eax, dword_ptr[eax]);
		adc(byte_ptr[eax], al);
		adc(word_ptr[eax], ax);
		adc(dword_ptr[eax], eax);
#ifdef JITASM64
		adc(rax, 0x1);
		adc(rax, 0x100);
		adc(rax, 0x10000);
		adc(rax, -0x1);
		adc(rax, -0x100);
		adc(rax, -0x10000);
		adc(r8, 0x1);
		adc(r8, 0x100);
		adc(r8, 0x10000);
		adc(r8, -0x1);
		adc(r8, -0x100);
		adc(r8, -0x10000);
		adc(qword_ptr[rax], 1);
		adc(rax, qword_ptr[rax]);
		adc(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// SBB
//----------------------------------------
extern "C" void masm_test_sbb();
struct test_sbb : jitasm::function0<void>
{
	virtual void naked_main()
	{
		sbb(al, 0x1);
		sbb(al, -0x1);
		sbb(ax, 0x1);
		sbb(ax, 0x100);
		sbb(ax, -0x1);
		sbb(ax, -0x100);
		sbb(eax, 0x1);
		sbb(eax, 0x100);
		sbb(eax, 0x10000);
		sbb(eax, -0x1);
		sbb(eax, -0x100);
		sbb(eax, -0x10000);
		sbb(cl, 0x1);
		sbb(cl, -0x1);
		sbb(cx, 0x1);
		sbb(cx, 0x100);
		sbb(cx, -0x1);
		sbb(cx, -0x100);
		sbb(ecx, 0x1);
		sbb(ecx, 0x100);
		sbb(ecx, 0x10000);
		sbb(ecx, -0x1);
		sbb(ecx, -0x100);
		sbb(ecx, -0x10000);
		sbb(byte_ptr[eax], 1);
		sbb(word_ptr[eax], 1);
		sbb(dword_ptr[eax], 1);
		sbb(al, byte_ptr[eax]);
		sbb(ax, word_ptr[eax]);
		sbb(eax, dword_ptr[eax]);
		sbb(byte_ptr[eax], al);
		sbb(word_ptr[eax], ax);
		sbb(dword_ptr[eax], eax);
#ifdef JITASM64
		sbb(rax, 0x1);
		sbb(rax, 0x100);
		sbb(rax, 0x10000);
		sbb(rax, -0x1);
		sbb(rax, -0x100);
		sbb(rax, -0x10000);
		sbb(r8, 0x1);
		sbb(r8, 0x100);
		sbb(r8, 0x10000);
		sbb(r8, -0x1);
		sbb(r8, -0x100);
		sbb(r8, -0x10000);
		sbb(qword_ptr[rax], 1);
		sbb(rax, qword_ptr[rax]);
		sbb(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// AND
//----------------------------------------
extern "C" void masm_test_and();
struct test_and : jitasm::function0<void>
{
	virtual void naked_main()
	{
		and(al, 0x1);
		and(al, -0x1);
		and(ax, 0x1);
		and(ax, 0x100);
		and(ax, -0x1);
		and(ax, -0x100);
		and(eax, 0x1);
		and(eax, 0x100);
		and(eax, 0x10000);
		and(eax, -0x1);
		and(eax, -0x100);
		and(eax, -0x10000);
		and(cl, 0x1);
		and(cl, -0x1);
		and(cx, 0x1);
		and(cx, 0x100);
		and(cx, -0x1);
		and(cx, -0x100);
		and(ecx, 0x1);
		and(ecx, 0x100);
		and(ecx, 0x10000);
		and(ecx, -0x1);
		and(ecx, -0x100);
		and(ecx, -0x10000);
		and(byte_ptr[eax], 1);
		and(word_ptr[eax], 1);
		and(dword_ptr[eax], 1);
		and(al, byte_ptr[eax]);
		and(ax, word_ptr[eax]);
		and(eax, dword_ptr[eax]);
		and(byte_ptr[eax], al);
		and(word_ptr[eax], ax);
		and(dword_ptr[eax], eax);
#ifdef JITASM64
		and(rax, 0x1);
		and(rax, 0x100);
		and(rax, 0x10000);
		and(rax, -0x1);
		and(rax, -0x100);
		and(rax, -0x10000);
		and(r8, 0x1);
		and(r8, 0x100);
		and(r8, 0x10000);
		and(r8, -0x1);
		and(r8, -0x100);
		and(r8, -0x10000);
		and(qword_ptr[rax], 1);
		and(rax, qword_ptr[rax]);
		and(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// SUB
//----------------------------------------
extern "C" void masm_test_sub();
struct test_sub : jitasm::function0<void>
{
	virtual void naked_main()
	{
		sub(al, 0x1);
		sub(al, -0x1);
		sub(ax, 0x1);
		sub(ax, 0x100);
		sub(ax, -0x1);
		sub(ax, -0x100);
		sub(eax, 0x1);
		sub(eax, 0x100);
		sub(eax, 0x10000);
		sub(eax, -0x1);
		sub(eax, -0x100);
		sub(eax, -0x10000);
		sub(cl, 0x1);
		sub(cl, -0x1);
		sub(cx, 0x1);
		sub(cx, 0x100);
		sub(cx, -0x1);
		sub(cx, -0x100);
		sub(ecx, 0x1);
		sub(ecx, 0x100);
		sub(ecx, 0x10000);
		sub(ecx, -0x1);
		sub(ecx, -0x100);
		sub(ecx, -0x10000);
		sub(byte_ptr[eax], 1);
		sub(word_ptr[eax], 1);
		sub(dword_ptr[eax], 1);
		sub(al, byte_ptr[eax]);
		sub(ax, word_ptr[eax]);
		sub(eax, dword_ptr[eax]);
		sub(byte_ptr[eax], al);
		sub(word_ptr[eax], ax);
		sub(dword_ptr[eax], eax);
#ifdef JITASM64
		sub(rax, 0x1);
		sub(rax, 0x100);
		sub(rax, 0x10000);
		sub(rax, -0x1);
		sub(rax, -0x100);
		sub(rax, -0x10000);
		sub(r8, 0x1);
		sub(r8, 0x100);
		sub(r8, 0x10000);
		sub(r8, -0x1);
		sub(r8, -0x100);
		sub(r8, -0x10000);
		sub(qword_ptr[rax], 1);
		sub(rax, qword_ptr[rax]);
		sub(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// XOR
//----------------------------------------
extern "C" void masm_test_xor();
struct test_xor : jitasm::function0<void>
{
	virtual void naked_main()
	{
		xor(al, 0x1);
		xor(al, -0x1);
		xor(ax, 0x1);
		xor(ax, 0x100);
		xor(ax, -0x1);
		xor(ax, -0x100);
		xor(eax, 0x1);
		xor(eax, 0x100);
		xor(eax, 0x10000);
		xor(eax, -0x1);
		xor(eax, -0x100);
		xor(eax, -0x10000);
		xor(cl, 0x1);
		xor(cl, -0x1);
		xor(cx, 0x1);
		xor(cx, 0x100);
		xor(cx, -0x1);
		xor(cx, -0x100);
		xor(ecx, 0x1);
		xor(ecx, 0x100);
		xor(ecx, 0x10000);
		xor(ecx, -0x1);
		xor(ecx, -0x100);
		xor(ecx, -0x10000);
		xor(byte_ptr[eax], 1);
		xor(word_ptr[eax], 1);
		xor(dword_ptr[eax], 1);
		xor(al, byte_ptr[eax]);
		xor(ax, word_ptr[eax]);
		xor(eax, dword_ptr[eax]);
		xor(byte_ptr[eax], al);
		xor(word_ptr[eax], ax);
		xor(dword_ptr[eax], eax);
#ifdef JITASM64
		xor(rax, 0x1);
		xor(rax, 0x100);
		xor(rax, 0x10000);
		xor(rax, -0x1);
		xor(rax, -0x100);
		xor(rax, -0x10000);
		xor(r8, 0x1);
		xor(r8, 0x100);
		xor(r8, 0x10000);
		xor(r8, -0x1);
		xor(r8, -0x100);
		xor(r8, -0x10000);
		xor(qword_ptr[rax], 1);
		xor(rax, qword_ptr[rax]);
		xor(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// CMP
//----------------------------------------
extern "C" void masm_test_cmp();
struct test_cmp : jitasm::function0<void>
{
	virtual void naked_main()
	{
		cmp(al, 0x1);
		cmp(al, -0x1);
		cmp(ax, 0x1);
		cmp(ax, 0x100);
		cmp(ax, -0x1);
		cmp(ax, -0x100);
		cmp(eax, 0x1);
		cmp(eax, 0x100);
		cmp(eax, 0x10000);
		cmp(eax, -0x1);
		cmp(eax, -0x100);
		cmp(eax, -0x10000);
		cmp(cl, 0x1);
		cmp(cl, -0x1);
		cmp(cx, 0x1);
		cmp(cx, 0x100);
		cmp(cx, -0x1);
		cmp(cx, -0x100);
		cmp(ecx, 0x1);
		cmp(ecx, 0x100);
		cmp(ecx, 0x10000);
		cmp(ecx, -0x1);
		cmp(ecx, -0x100);
		cmp(ecx, -0x10000);
		cmp(byte_ptr[eax], 1);
		cmp(word_ptr[eax], 1);
		cmp(dword_ptr[eax], 1);
		cmp(al, byte_ptr[eax]);
		cmp(ax, word_ptr[eax]);
		cmp(eax, dword_ptr[eax]);
		cmp(byte_ptr[eax], al);
		cmp(word_ptr[eax], ax);
		cmp(dword_ptr[eax], eax);
#ifdef JITASM64
		cmp(rax, 0x1);
		cmp(rax, 0x100);
		cmp(rax, 0x10000);
		cmp(rax, -0x1);
		cmp(rax, -0x100);
		cmp(rax, -0x10000);
		cmp(r8, 0x1);
		cmp(r8, 0x100);
		cmp(r8, 0x10000);
		cmp(r8, -0x1);
		cmp(r8, -0x100);
		cmp(r8, -0x10000);
		cmp(qword_ptr[rax], 1);
		cmp(rax, qword_ptr[rax]);
		cmp(qword_ptr[rax], rax);
#endif
	}
};

//----------------------------------------
// XCHG
//----------------------------------------
extern "C" void masm_test_xchg();
struct test_xchg : jitasm::function0<void>
{
	virtual void naked_main()
	{
		xchg(al, cl);
		xchg(cl, al);
		xchg(ax, cx);
		xchg(cx, ax);
		xchg(eax, ecx);
		xchg(ecx, eax);
		xchg(ecx, edx);
		xchg(edx, ecx);
		xchg(al, byte_ptr[ecx]);
		xchg(byte_ptr[ecx], al);
		xchg(cl, byte_ptr[eax]);
		xchg(byte_ptr[eax], cl);
		xchg(ax, word_ptr[ecx]);
		xchg(word_ptr[ecx], ax);
		xchg(cx, word_ptr[eax]);
		xchg(word_ptr[eax], cx);
		xchg(eax, dword_ptr[ecx]);
		xchg(dword_ptr[ecx], eax);
		xchg(ecx, dword_ptr[eax]);
		xchg(dword_ptr[eax], ecx);
#ifdef JITASM64
		xchg(rax, r8);
		xchg(r8, rax);
		xchg(rax, qword_ptr[r8]);
		xchg(qword_ptr[r8], rax);
		xchg(r8, qword_ptr[rax]);
		xchg(qword_ptr[rax], r8);
#endif
	}
};

//----------------------------------------
// TEST
//----------------------------------------
extern "C" void masm_test_test();
struct test_test : jitasm::function0<void>
{
	virtual void naked_main()
	{
		test(al, 1);
		test(ax, 1);
		test(eax, 1);
		test(al, -1);
		test(ax, -1);
		test(eax, -1);
		test(cl, 1);
		test(cx, 1);
		test(ecx, 1);
		test(cl, -1);
		test(cx, -1);
		test(ecx, -1);
		test(al, cl);
		test(ax, cx);
		test(eax, ecx);
		test(cl, al);
		test(cx, ax);
		test(ecx, eax);
		test(byte_ptr[eax], 1);
		test(word_ptr[eax], 1);
		test(dword_ptr[eax], 1);
		test(byte_ptr[eax], cl);
		test(word_ptr[eax], cx);
		test(dword_ptr[eax], ecx);
#ifdef JITASM64
		test(rax, 1);
		test(rax, -1);
		test(r8, 1);
		test(r8, -1);
		test(rax, r8);
		test(r8, rax);
		test(qword_ptr[eax], 1);
		test(qword_ptr[eax], r8);
#endif
	}
};

//----------------------------------------
// MOV/MOVZX
//----------------------------------------
extern "C" void masm_test_mov();
struct test_mov : jitasm::function0<void>
{
	virtual void naked_main()
	{
		mov(al, cl);
		mov(ax, cx);
		mov(eax, ecx);
		mov(byte_ptr[eax], cl);
		mov(word_ptr[eax], cx);
		mov(dword_ptr[eax], ecx);
		mov(al, byte_ptr[ecx]);
		mov(ax, word_ptr[ecx]);
		mov(eax, dword_ptr[ecx]);
		mov(al, 1);
		mov(al, -1);
		mov(ax, 1);
		mov(ax, -1);
		mov(eax, 1);
		mov(eax, -1);
		mov(byte_ptr[eax], 1);
		mov(word_ptr[eax], 1);
		mov(dword_ptr[eax], 1);
		movzx(ax, cl);
		movzx(eax, cl);
		movzx(eax, cx);
		movzx(ax, byte_ptr[ecx]);
		movzx(eax, byte_ptr[ecx]);
		movzx(eax, word_ptr[ecx]);
#ifdef JITASM64
		mov(rax, r8);
		mov(qword_ptr[rax], r8);
		mov(rax, qword_ptr[r8]);
		mov(rax, 1);
		mov(rax, -1);
		mov(qword_ptr[rax], 1);
		movzx(rax, cl);
		movzx(rax, cx);
		movzx(rax, byte_ptr[rcx]);
		movzx(rax, word_ptr[rcx]);
#endif
	}
};

//----------------------------------------
// LEA
//----------------------------------------
extern "C" void masm_test_lea();
struct test_lea : jitasm::function0<void>
{
	virtual void naked_main()
	{
		lea(eax, dword_ptr[eax]);
		lea(eax, dword_ptr[esp]);
		lea(eax, dword_ptr[ebp]);
		lea(eax, dword_ptr[eax + ecx]);
		lea(eax, dword_ptr[ecx + eax]);
		lea(eax, dword_ptr[esp + ecx]);
		lea(eax, dword_ptr[ecx + esp]);
		lea(eax, dword_ptr[ebp + ecx]);
		lea(eax, dword_ptr[ecx + ebp]);
		lea(eax, dword_ptr[esp + ebp]);
		lea(eax, dword_ptr[ebp + esp]);
		lea(eax, dword_ptr[eax + 0x1]);
		lea(eax, dword_ptr[esp + 0x1]);
		lea(eax, dword_ptr[ebp + 0x1]);
		lea(eax, dword_ptr[eax + 0x100]);
		lea(eax, dword_ptr[esp + 0x100]);
		lea(eax, dword_ptr[ebp + 0x100]);
		lea(eax, dword_ptr[eax + 0x10000]);
		lea(eax, dword_ptr[esp + 0x10000]);
		lea(eax, dword_ptr[ebp + 0x10000]);
		lea(eax, dword_ptr[eax * 2]);
		lea(eax, dword_ptr[ebp * 2]);
		lea(eax, dword_ptr[eax * 4]);
		lea(eax, dword_ptr[ebp * 4]);
		lea(eax, dword_ptr[eax * 8]);
		lea(eax, dword_ptr[ebp * 8]);
		lea(eax, dword_ptr[eax * 2 + 0x1]);
		lea(eax, dword_ptr[ebp * 2 + 0x1]);
		lea(eax, dword_ptr[eax * 2 + 0x100]);
		lea(eax, dword_ptr[ebp * 2 + 0x100]);
		lea(eax, dword_ptr[eax * 2 + 0x10000]);
		lea(eax, dword_ptr[ebp * 2 + 0x10000]);
		lea(eax, dword_ptr[eax + ecx * 2]);
		lea(eax, dword_ptr[ecx + eax * 2]);
		lea(eax, dword_ptr[esp + ecx * 2]);
		lea(eax, dword_ptr[ebp + ecx * 2]);
		lea(eax, dword_ptr[ecx + ebp * 2]);
		lea(eax, dword_ptr[esp + ebp * 2]);
		lea(eax, dword_ptr[eax + ecx + 0x1]);
		lea(eax, dword_ptr[ecx + eax + 0x1]);
		lea(eax, dword_ptr[esp + ecx + 0x1]);
		lea(eax, dword_ptr[ecx + esp + 0x1]);
		lea(eax, dword_ptr[ebp + ecx + 0x1]);
		lea(eax, dword_ptr[ecx + ebp + 0x1]);
		lea(eax, dword_ptr[esp + ebp + 0x1]);
		lea(eax, dword_ptr[ebp + esp + 0x1]);
		lea(eax, dword_ptr[eax + ecx + 0x100]);
		lea(eax, dword_ptr[ecx + eax + 0x100]);
		lea(eax, dword_ptr[esp + ecx + 0x100]);
		lea(eax, dword_ptr[ecx + esp + 0x100]);
		lea(eax, dword_ptr[ebp + ecx + 0x100]);
		lea(eax, dword_ptr[ecx + ebp + 0x100]);
		lea(eax, dword_ptr[esp + ebp + 0x100]);
		lea(eax, dword_ptr[ebp + esp + 0x100]);
		lea(eax, dword_ptr[eax + ecx + 0x10000]);
		lea(eax, dword_ptr[ecx + eax + 0x10000]);
		lea(eax, dword_ptr[esp + ecx + 0x10000]);
		lea(eax, dword_ptr[ecx + esp + 0x10000]);
		lea(eax, dword_ptr[ebp + ecx + 0x10000]);
		lea(eax, dword_ptr[ecx + ebp + 0x10000]);
		lea(eax, dword_ptr[esp + ebp + 0x10000]);
		lea(eax, dword_ptr[ebp + esp + 0x10000]);
		lea(eax, dword_ptr[eax + ecx * 2 + 0x1]);
		lea(eax, dword_ptr[ecx + eax * 2 + 0x1]);
		lea(eax, dword_ptr[esp + ecx * 2 + 0x1]);
		lea(eax, dword_ptr[ebp + ecx * 2 + 0x1]);
		lea(eax, dword_ptr[ecx + ebp * 2 + 0x1]);
		lea(eax, dword_ptr[esp + ebp * 2 + 0x1]);
		lea(eax, dword_ptr[eax + ecx * 2 + 0x100]);
		lea(eax, dword_ptr[ecx + eax * 2 + 0x100]);
		lea(eax, dword_ptr[esp + ecx * 2 + 0x100]);
		lea(eax, dword_ptr[ebp + ecx * 2 + 0x100]);
		lea(eax, dword_ptr[ecx + ebp * 2 + 0x100]);
		lea(eax, dword_ptr[esp + ebp * 2 + 0x100]);
		lea(eax, dword_ptr[eax + ecx * 2 + 0x10000]);
		lea(eax, dword_ptr[ecx + eax * 2 + 0x10000]);
		lea(eax, dword_ptr[esp + ecx * 2 + 0x10000]);
		lea(eax, dword_ptr[ebp + ecx * 2 + 0x10000]);
		lea(eax, dword_ptr[ecx + ebp * 2 + 0x10000]);
		lea(eax, dword_ptr[esp + ebp * 2 + 0x10000]);
		lea(eax, dword_ptr[eax - 0x1]);
		lea(eax, dword_ptr[eax - 0x100]);
		lea(eax, dword_ptr[eax - 0x10000]);
#ifdef JITASM64
		lea(rax, qword_ptr[rax]);
		lea(rax, qword_ptr[rsp]);
		lea(rax, qword_ptr[rbp]);
		lea(rax, qword_ptr[rax + rcx]);
		lea(rax, qword_ptr[rcx + rax]);
		lea(rax, qword_ptr[rsp + rcx]);
		lea(rax, qword_ptr[rcx + rsp]);
		lea(rax, qword_ptr[rbp + rcx]);
		lea(rax, qword_ptr[rcx + rbp]);
		lea(rax, qword_ptr[rsp + rbp]);
		lea(rax, qword_ptr[rbp + rsp]);
		lea(rax, qword_ptr[rax + 0x1]);
		lea(rax, qword_ptr[rsp + 0x1]);
		lea(rax, qword_ptr[rbp + 0x1]);
		lea(rax, qword_ptr[rax + 0x100]);
		lea(rax, qword_ptr[rsp + 0x100]);
		lea(rax, qword_ptr[rbp + 0x100]);
		lea(rax, qword_ptr[rax + 0x10000]);
		lea(rax, qword_ptr[rsp + 0x10000]);
		lea(rax, qword_ptr[rbp + 0x10000]);
		lea(rax, qword_ptr[rax * 2]);
		lea(rax, qword_ptr[rbp * 2]);
		lea(rax, qword_ptr[rax * 4]);
		lea(rax, qword_ptr[rbp * 4]);
		lea(rax, qword_ptr[rax * 8]);
		lea(rax, qword_ptr[rbp * 8]);
		lea(rax, qword_ptr[rax * 2 + 0x1]);
		lea(rax, qword_ptr[rbp * 2 + 0x1]);
		lea(rax, qword_ptr[rax * 2 + 0x100]);
		lea(rax, qword_ptr[rbp * 2 + 0x100]);
		lea(rax, qword_ptr[rax * 2 + 0x10000]);
		lea(rax, qword_ptr[rbp * 2 + 0x10000]);
		lea(rax, qword_ptr[rax + rcx * 2]);
		lea(rax, qword_ptr[rcx + rax * 2]);
		lea(rax, qword_ptr[rsp + rcx * 2]);
		lea(rax, qword_ptr[rbp + rcx * 2]);
		lea(rax, qword_ptr[rcx + rbp * 2]);
		lea(rax, qword_ptr[rsp + rbp * 2]);
		lea(rax, qword_ptr[rax + rcx + 0x1]);
		lea(rax, qword_ptr[rcx + rax + 0x1]);
		lea(rax, qword_ptr[rsp + rcx + 0x1]);
		lea(rax, qword_ptr[rcx + rsp + 0x1]);
		lea(rax, qword_ptr[rbp + rcx + 0x1]);
		lea(rax, qword_ptr[rcx + rbp + 0x1]);
		lea(rax, qword_ptr[rsp + rbp + 0x1]);
		lea(rax, qword_ptr[rbp + rsp + 0x1]);
		lea(rax, qword_ptr[rax + rcx + 0x100]);
		lea(rax, qword_ptr[rcx + rax + 0x100]);
		lea(rax, qword_ptr[rsp + rcx + 0x100]);
		lea(rax, qword_ptr[rbp + rcx + 0x100]);
		lea(rax, qword_ptr[rcx + rbp + 0x100]);
		lea(rax, qword_ptr[rsp + rbp + 0x100]);
		lea(rax, qword_ptr[rbp + rsp + 0x100]);
		lea(rax, qword_ptr[rax + rcx + 0x10000]);
		lea(rax, qword_ptr[rcx + rax + 0x10000]);
		lea(rax, qword_ptr[rsp + rcx + 0x10000]);
		lea(rax, qword_ptr[rbp + rcx + 0x10000]);
		lea(rax, qword_ptr[rcx + rbp + 0x10000]);
		lea(rax, qword_ptr[rsp + rbp + 0x10000]);
		lea(rax, qword_ptr[rbp + rsp + 0x10000]);
		lea(rax, qword_ptr[rax + rcx * 2 + 0x1]);
		lea(rax, qword_ptr[rcx + rax * 2 + 0x1]);
		lea(rax, qword_ptr[rsp + rcx * 2 + 0x1]);
		lea(rax, qword_ptr[rbp + rcx * 2 + 0x1]);
		lea(rax, qword_ptr[rcx + rbp * 2 + 0x1]);
		lea(rax, qword_ptr[rsp + rbp * 2 + 0x1]);
		lea(rax, qword_ptr[rax + rcx * 2 + 0x100]);
		lea(rax, qword_ptr[rcx + rax * 2 + 0x100]);
		lea(rax, qword_ptr[rsp + rcx * 2 + 0x100]);
		lea(rax, qword_ptr[rbp + rcx * 2 + 0x100]);
		lea(rax, qword_ptr[rcx + rbp * 2 + 0x100]);
		lea(rax, qword_ptr[rsp + rbp * 2 + 0x100]);
		lea(rax, qword_ptr[rax + rcx * 2 + 0x10000]);
		lea(rax, qword_ptr[rcx + rax * 2 + 0x10000]);
		lea(rax, qword_ptr[rsp + rcx * 2 + 0x10000]);
		lea(rax, qword_ptr[rbp + rcx * 2 + 0x10000]);
		lea(rax, qword_ptr[rcx + rbp * 2 + 0x10000]);
		lea(rax, qword_ptr[rsp + rbp * 2 + 0x10000]);
#endif
	}
};

//----------------------------------------
// FLD
//----------------------------------------
extern "C" void masm_test_fld();
struct test_fld : jitasm::function0<void>
{
	virtual void naked_main()
	{
		fld(real4_ptr[esp]);
		fld(real8_ptr[esp]);
		fld(real10_ptr[esp]);
		fld(st(0));
		fld(st(7));
#ifdef JITASM64
		fld(real4_ptr[rsp]);
		fld(real8_ptr[rsp]);
		fld(real10_ptr[rsp]);
#endif
	}
};

//----------------------------------------
// JMP
//----------------------------------------
extern "C" void masm_test_jmp();
struct test_jmp : jitasm::function0<void>
{
	virtual void naked_main()
	{
		// jmp short
		nop();
		jmp("L1");
		ja("L1");
		jae("L1");
		jb("L1");
		jbe("L1");
		jc("L1");
#ifdef JITASM64
		jrcxz("L1");
#else
		jcxz("L1");
#endif
		jecxz("L1");
		je("L1");
		jg("L1");
		jge("L1");
		jl("L1");
		jle("L1");
		jna("L1");
		jnae("L1");
		jnb("L1");
		jnbe("L1");
		jnc("L1");
		jne("L1");
		jng("L1");
		jnge("L1");
		jnl("L1");
		jnle("L1");
		jno("L1");
		jnp("L1");
		jns("L1");
		jnz("L1");
		jo("L1");
		jp("L1");
		jpe("L1");
		jpo("L1");
		js("L1");
		jz("L1");
	label("L1");
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		pabsb(xmm0, xmmword_ptr[zsp + zcx + 0x100]);	// 10 bytes
		// jmp near
		jmp("L1");
		ja("L1");
		jae("L1");
		jb("L1");
		jbe("L1");
		jc("L1");
		je("L1");
		jg("L1");
		jge("L1");
		jl("L1");
		jle("L1");
		jna("L1");
		jnae("L1");
		jnb("L1");
		jnbe("L1");
		jnc("L1");
		jne("L1");
		jng("L1");
		jnge("L1");
		jnl("L1");
		jnle("L1");
		jno("L1");
		jnp("L1");
		jns("L1");
		jnz("L1");
		jo("L1");
		jp("L1");
		jpe("L1");
		jpo("L1");
		js("L1");
		jz("L1");
	}
};

//----------------------------------------
// function0_cdecl<int>
//----------------------------------------
extern "C" void masm_test_function0_cdecl();
struct test_function0_cdecl : jitasm::function0_cdecl<int>
{
	Result main()
	{
		return 16;
	}
};

//----------------------------------------
// function1_cdecl<float>
//----------------------------------------
extern "C" void masm_test_function1_cdecl();
struct test_function1_cdecl : jitasm::function1_cdecl<float, float>
{
	Result main(Arg a1)
	{
		return 1.0f;
	}
};

struct hoge : jitasm::function2_cdecl<void, short, int>
{
	virtual void main(Arg a1, Arg a2)
	{
		movzx(eax, word_ptr[a1]);
		mov(ecx, dword_ptr[a2]);
	}
};

struct mov_disp : jitasm::function0<void>
{
	virtual void naked_main()
	{
		mov(al, byte_ptr[1]);
		mov(cl, byte_ptr[1]);
		mov(al, byte_ptr[-1]);
		mov(cl, byte_ptr[-1]);
		mov(ax, word_ptr[1]);
		mov(cx, word_ptr[1]);
		mov(ax, word_ptr[-1]);
		mov(cx, word_ptr[-1]);
		mov(eax, dword_ptr[1]);
		mov(ecx, dword_ptr[1]);
		mov(eax, dword_ptr[-1]);
		mov(ecx, dword_ptr[-1]);
#ifdef JITASM64
		mov(rax, qword_ptr[1]);
		mov(rax, qword_ptr[-1]);
		mov(rax, qword_ptr[0x100000000]);
#endif
	}
};

int _tmain(int argc, _TCHAR* argv[])
{
	TEST(masm_test_sal, test_sal());
	TEST(masm_test_sar, test_sar());
	TEST(masm_test_shl, test_shl());
	TEST(masm_test_shr, test_shr());
	TEST(masm_test_inc_dec, test_inc_dec());
	TEST(masm_test_push_pop, test_push_pop());
	TEST(masm_test_add, test_add());
	TEST(masm_test_or, test_or());
	TEST(masm_test_adc, test_adc());
	TEST(masm_test_sbb, test_sbb());
	TEST(masm_test_and, test_and());
	TEST(masm_test_sub, test_sub());
	TEST(masm_test_xor, test_xor());
	TEST(masm_test_cmp, test_cmp());
	TEST(masm_test_xchg, test_xchg());
	TEST(masm_test_test, test_test());
	TEST(masm_test_mov, test_mov());
	TEST(masm_test_lea, test_lea());
	TEST(masm_test_fld, test_fld());
	TEST(masm_test_jmp, test_jmp());

	TEST(masm_test_function0_cdecl, test_function0_cdecl());
	//TEST(masm_test_function1_cdecl, test_function1_cdecl());
}
