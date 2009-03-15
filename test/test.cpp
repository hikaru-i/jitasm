#include <assert.h>
#include "jitasm.h"

#define _TOSTR(s) #s
#define TOSTR(s) _TOSTR(s)
#define TEST_M(func_name) {test_impl(TOSTR(func_name), func_name ## (), masm_ ## func_name);}
#define TEST_N(func_name) {test_impl(TOSTR(func_name), func_name ## (), nasm_ ## func_name);}

template<class Fn1, class Fn2>
void test_impl(const char* func_name, Fn1 fn1, Fn2 fn2)
{
	printf("TEST_M(%s) ... ", func_name);

	fn1.Assemble();
	size_t size = fn1.GetCodeSize();

	unsigned char* p1 = (unsigned char*) fn1.GetCode();
	unsigned char* p2 = (unsigned char*) fn2;
	if (*p2 == 0xE9) p2 = (unsigned char*) (p2 + (unsigned long&) *(p2 + 1) + 5);
	
	for (size_t i = 0; i < size; i++) {
		if (p1[i] != p2[i]) {
			printf("FAILED\n");

			size_t min = (size_t) max((int) i - 10, 0);
			size_t max = min(i + 10, size);

			printf("    J[%d] ", min);
			for (size_t j = min; j < max; j++) printf("%02X,", p1[j]);
			printf("\n");

			printf("    M[%d] ", min);
			for (size_t j = min; j < max; j++) printf("%02X,", p2[j]);
			printf("\n");

			if (::IsDebuggerPresent()) {
				::DebugBreak();
			}
			return;
		}
	}

	printf("OK\n");
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
// RCL
//----------------------------------------
extern "C" void masm_test_rcl();
struct test_rcl : jitasm::function0<void>
{
	virtual void naked_main()
	{
		rcl(al, 1);
		rcl(al, 2);
		rcl(al, -1);
		rcl(ax, 1);
		rcl(ax, 2);
		rcl(eax, 1);
		rcl(eax, 2);
		rcl(byte_ptr[eax], 1);
		rcl(byte_ptr[eax], 2);
		rcl(word_ptr[eax], 1);
		rcl(word_ptr[eax], 2);
		rcl(dword_ptr[eax], 1);
		rcl(dword_ptr[eax], 2);
#ifdef JITASM64
		rcl(r8b, 1);
		rcl(r8b, 2);
		rcl(r8w, 1);
		rcl(r8w, 2);
		rcl(r8d, 1);
		rcl(r8d, 2);
		rcl(r8, 1);
		rcl(r8, 2);
		rcl(qword_ptr[rax], 1);
		rcl(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// RCR
//----------------------------------------
extern "C" void masm_test_rcr();
struct test_rcr : jitasm::function0<void>
{
	virtual void naked_main()
	{
		rcr(al, 1);
		rcr(al, 2);
		rcr(al, -1);
		rcr(ax, 1);
		rcr(ax, 2);
		rcr(eax, 1);
		rcr(eax, 2);
		rcr(byte_ptr[eax], 1);
		rcr(byte_ptr[eax], 2);
		rcr(word_ptr[eax], 1);
		rcr(word_ptr[eax], 2);
		rcr(dword_ptr[eax], 1);
		rcr(dword_ptr[eax], 2);
#ifdef JITASM64
		rcr(r8b, 1);
		rcr(r8b, 2);
		rcr(r8w, 1);
		rcr(r8w, 2);
		rcr(r8d, 1);
		rcr(r8d, 2);
		rcr(r8, 1);
		rcr(r8, 2);
		rcr(qword_ptr[rax], 1);
		rcr(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// ROL
//----------------------------------------
extern "C" void masm_test_rol();
struct test_rol : jitasm::function0<void>
{
	virtual void naked_main()
	{
		rol(al, 1);
		rol(al, 2);
		rol(al, -1);
		rol(ax, 1);
		rol(ax, 2);
		rol(eax, 1);
		rol(eax, 2);
		rol(byte_ptr[eax], 1);
		rol(byte_ptr[eax], 2);
		rol(word_ptr[eax], 1);
		rol(word_ptr[eax], 2);
		rol(dword_ptr[eax], 1);
		rol(dword_ptr[eax], 2);
#ifdef JITASM64
		rol(r8b, 1);
		rol(r8b, 2);
		rol(r8w, 1);
		rol(r8w, 2);
		rol(r8d, 1);
		rol(r8d, 2);
		rol(r8, 1);
		rol(r8, 2);
		rol(qword_ptr[rax], 1);
		rol(qword_ptr[rax], 2);
#endif
	}
};

//----------------------------------------
// ROR
//----------------------------------------
extern "C" void masm_test_ror();
struct test_ror : jitasm::function0<void>
{
	virtual void naked_main()
	{
		ror(al, 1);
		ror(al, 2);
		ror(al, -1);
		ror(ax, 1);
		ror(ax, 2);
		ror(eax, 1);
		ror(eax, 2);
		ror(byte_ptr[eax], 1);
		ror(byte_ptr[eax], 2);
		ror(word_ptr[eax], 1);
		ror(word_ptr[eax], 2);
		ror(dword_ptr[eax], 1);
		ror(dword_ptr[eax], 2);
#ifdef JITASM64
		ror(r8b, 1);
		ror(r8b, 2);
		ror(r8w, 1);
		ror(r8w, 2);
		ror(r8d, 1);
		ror(r8d, 2);
		ror(r8, 1);
		ror(r8, 2);
		ror(qword_ptr[rax], 1);
		ror(qword_ptr[rax], 2);
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
// test
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
	L("L1");
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
// MOVSB/MOVSW/MOVSD/MOVSQ
//----------------------------------------
extern "C" void masm_test_movs();
struct test_movs : jitasm::function0<void>
{
	virtual void naked_main()
	{
		movsb();
		movsw();
		movsd();
		rep_movsb();
		rep_movsw();
		rep_movsd();
#ifdef JITASM64
		movsq();
		rep_movsq();
#endif
	}
};

//----------------------------------------
// mov with disp
//----------------------------------------
extern "C" void nasm_test_mov_disp();
struct test_mov_disp : jitasm::function0<void>
{
	virtual void naked_main()
	{
		mov(al, byte_ptr[1]);
		mov(cl, byte_ptr[1]);
		mov(ax, word_ptr[1]);
		mov(cx, word_ptr[1]);
		mov(eax, dword_ptr[1]);
		mov(ecx, dword_ptr[1]);
#ifdef JITASM64
		mov(rax, qword_ptr[1]);
		mov(rax, qword_ptr[0x100000000]);
#endif
	}
};

//----------------------------------------
// NEG/NOT
//----------------------------------------
extern "C" void masm_test_neg_not();
struct test_neg_not : jitasm::function0<void>
{
	virtual void naked_main()
	{
		neg(al);
		neg(ax);
		neg(eax);
		neg(byte_ptr[esp]);
		neg(word_ptr[esp]);
		neg(dword_ptr[esp]);
		not(al);
		not(ax);
		not(eax);
		not(byte_ptr[esp]);
		not(word_ptr[esp]);
		not(dword_ptr[esp]);
#ifdef JITASM64
		neg(r8w);
		neg(rax);
		neg(r8);
		neg(qword_ptr[rsp]);
		not(r8w);
		not(rax);
		not(r8);
		not(qword_ptr[rsp]);
#endif
	}
};

//----------------------------------------
// DIV/IDIV/MUL
//----------------------------------------
extern "C" void masm_test_div_idiv_mul();
struct test_div_idiv_mul : jitasm::function0<void>
{
	virtual void naked_main()
	{
		div(al);
		div(ax);
		div(eax);
		div(byte_ptr[esp]);
		div(word_ptr[esp]);
		div(dword_ptr[esp]);
		idiv(al);
		idiv(ax);
		idiv(eax);
		idiv(byte_ptr[esp]);
		idiv(word_ptr[esp]);
		idiv(dword_ptr[esp]);
		mul(al);
		mul(ax);
		mul(eax);
		mul(byte_ptr[esp]);
		mul(word_ptr[esp]);
		mul(dword_ptr[esp]);
#ifdef JITASM64
		div(r8w);
		div(rax);
		div(r8);
		div(qword_ptr[rsp]);
		idiv(r8w);
		idiv(rax);
		idiv(r8);
		idiv(qword_ptr[rsp]);
		mul(r8w);
		mul(rax);
		mul(r8);
		mul(qword_ptr[rsp]);
#endif
	}
};

//----------------------------------------
// IMUL
//----------------------------------------
extern "C" void masm_test_imul();
struct test_imul : jitasm::function0<void>
{
	virtual void naked_main()
	{
		imul(al);
		imul(ax);
		imul(eax);
		imul(byte_ptr[esp]);
		imul(word_ptr[esp]);
		imul(dword_ptr[esp]);
		imul(ax, ax);
		imul(ax, word_ptr[esp]);
		imul(eax, eax);
		imul(eax, dword_ptr[esp]);
		imul(ax, ax, 0x1);
		imul(ax, ax, -0x1);
		imul(ax, ax, 0x100);
		imul(eax, eax, 0x1);
		imul(eax, eax, -0x1);
		imul(eax, eax, 0x100);
#ifdef JITASM64
		imul(r8w);
		imul(rax);
		imul(r8);
		imul(qword_ptr[rsp]);
		imul(rax, rax);
		imul(rax, qword_ptr[rsp]);
		imul(rax, rax, 0x1);
		imul(rax, rax, -0x1);
		imul(rax, rax, 0x100);
#endif
	}
};

//----------------------------------------
// FST/FSTP
//----------------------------------------
extern "C" void masm_test_fst();
struct test_fst : jitasm::function0<void>
{
	virtual void naked_main()
	{
		fst(real4_ptr[esp]);
		fst(real8_ptr[esp]);
		fst(st(0));
		fst(st(7));
		fstp(real4_ptr[esp]);
		fstp(real8_ptr[esp]);
		fstp(real10_ptr[esp]);
		fstp(st(0));
		fstp(st(7));
#ifdef JITASM64
		fst(real4_ptr[rsp]);
		fst(real8_ptr[rsp]);
		fstp(real4_ptr[rsp]);
		fstp(real8_ptr[rsp]);
		fstp(real10_ptr[rsp]);
#endif
	}
};


//----------------------------------------
// SSE2 A~
//----------------------------------------
extern "C" void masm_test_sse2_a();
struct test_sse2_a : jitasm::function0<void>
{
	virtual void naked_main()
	{
		addpd(xmm0, xmmword_ptr[esp]);
		addpd(xmm0, xmm0);
		addsd(xmm0, qword_ptr[esp]);
		addsd(xmm0, xmm0);
		andpd(xmm0, xmmword_ptr[esp]);
		andpd(xmm0, xmm0);
		andnpd(xmm0, xmmword_ptr[esp]);
		andnpd(xmm0, xmm0);
#ifdef JITASM64
		addpd(xmm8, xmmword_ptr[rsp]);
		addpd(xmm8, xmmword_ptr[r8]);
		addpd(xmm8, xmm8);
		addsd(xmm8, qword_ptr[rsp]);
		addsd(xmm8, qword_ptr[r8]);
		addsd(xmm8, xmm8);
		andpd(xmm8, xmmword_ptr[rsp]);
		andpd(xmm8, xmmword_ptr[r8]);
		andpd(xmm8, xmm8);
		andnpd(xmm8, xmmword_ptr[rsp]);
		andnpd(xmm8, xmmword_ptr[r8]);
		andnpd(xmm8, xmm8);
#endif

		clflush(byte_ptr[esp]);
#ifdef JITASM64
		clflush(byte_ptr[r8]);
#endif

		cmpeqpd(xmm0, xmmword_ptr[esp]);
		cmpeqpd(xmm0, xmm0);
		cmpltpd(xmm0, xmmword_ptr[esp]);
		cmpltpd(xmm0, xmm0);
		cmplepd(xmm0, xmmword_ptr[esp]);
		cmplepd(xmm0, xmm0);
		cmpunordpd(xmm0, xmmword_ptr[esp]);
		cmpunordpd(xmm0, xmm0);
		cmpneqpd(xmm0, xmmword_ptr[esp]);
		cmpneqpd(xmm0, xmm0);
		cmpnltpd(xmm0, xmmword_ptr[esp]);
		cmpnltpd(xmm0, xmm0);
		cmpnlepd(xmm0, xmmword_ptr[esp]);
		cmpnlepd(xmm0, xmm0);
		cmpordpd(xmm0, xmmword_ptr[esp]);
		cmpordpd(xmm0, xmm0);

		cmpeqsd(xmm0, qword_ptr[esp]);
		cmpeqsd(xmm0, xmm0);
		cmpltsd(xmm0, qword_ptr[esp]);
		cmpltsd(xmm0, xmm0);
		cmplesd(xmm0, qword_ptr[esp]);
		cmplesd(xmm0, xmm0);
		cmpunordsd(xmm0, qword_ptr[esp]);
		cmpunordsd(xmm0, xmm0);
		cmpneqsd(xmm0, qword_ptr[esp]);
		cmpneqsd(xmm0, xmm0);
		cmpnltsd(xmm0, qword_ptr[esp]);
		cmpnltsd(xmm0, xmm0);
		cmpnlesd(xmm0, qword_ptr[esp]);
		cmpnlesd(xmm0, xmm0);
		cmpordsd(xmm0, qword_ptr[esp]);
		cmpordsd(xmm0, xmm0);

		comisd(xmm0, qword_ptr[esp]);
		comisd(xmm0, xmm0);
#ifdef JITASM64
		cmpeqpd(xmm8, xmmword_ptr[rsp]);
		cmpeqpd(xmm8, xmm8);
		cmpltpd(xmm8, xmmword_ptr[r8]);
		cmpltpd(xmm8, xmm8);
		cmplepd(xmm8, xmmword_ptr[rsp]);
		cmplepd(xmm8, xmm8);
		cmpunordpd(xmm8, xmmword_ptr[r8]);
		cmpunordpd(xmm8, xmm8);
		cmpneqpd(xmm8, xmmword_ptr[rsp]);
		cmpneqpd(xmm8, xmm8);
		cmpnltpd(xmm8, xmmword_ptr[r8]);
		cmpnltpd(xmm8, xmm8);
		cmpnlepd(xmm8, xmmword_ptr[rsp]);
		cmpnlepd(xmm8, xmm8);
		cmpordpd(xmm8, xmmword_ptr[r8]);
		cmpordpd(xmm8, xmm8);

		cmpeqsd(xmm8, qword_ptr[rsp]);
		cmpeqsd(xmm8, xmm8);
		cmpltsd(xmm8, qword_ptr[r8]);
		cmpltsd(xmm8, xmm8);
		cmplesd(xmm8, qword_ptr[rsp]);
		cmplesd(xmm8, xmm8);
		cmpunordsd(xmm8, qword_ptr[r8]);
		cmpunordsd(xmm8, xmm8);
		cmpneqsd(xmm8, qword_ptr[rsp]);
		cmpneqsd(xmm8, xmm8);
		cmpnltsd(xmm8, qword_ptr[r8]);
		cmpnltsd(xmm8, xmm8);
		cmpnlesd(xmm8, qword_ptr[rsp]);
		cmpnlesd(xmm8, xmm8);
		cmpordsd(xmm8, qword_ptr[r8]);
		cmpordsd(xmm8, xmm8);

		comisd(xmm8, qword_ptr[rsp]);
		comisd(xmm8, qword_ptr[r8]);
		comisd(xmm8, xmm8);

#endif
		cvtdq2pd(xmm0, xmm1);
		cvtdq2pd(xmm0, qword_ptr[esp]);
		cvtpd2dq(xmm0, xmm1);
		cvtpd2dq(xmm0, xmmword_ptr[esp]);
		cvtpd2pi(mm0, xmm1);
		cvtpd2pi(mm0, xmmword_ptr[esp]);
		cvtpd2ps(xmm0, xmm1);
		cvtpd2ps(xmm0, xmmword_ptr[esp]);
		cvtpi2pd(xmm0, mm1);
		cvtpi2pd(xmm0, qword_ptr[esp]);
		cvtps2dq(xmm0, xmm1);
		cvtps2dq(xmm0, xmmword_ptr[esp]);
		cvtdq2ps(xmm0, xmm1);
		cvtdq2ps(xmm0, xmmword_ptr[esp]);
		cvtps2pd(xmm0, xmm1);
		cvtps2pd(xmm0, qword_ptr[esp]);
		cvtsd2ss(xmm0, xmm1);
		cvtsd2ss(xmm0, qword_ptr[esp]);
		cvtsi2sd(xmm0, eax);
		cvtsi2sd(xmm0, dword_ptr[esp]);
		cvtss2sd(xmm0, xmm1);
		cvtss2sd(xmm0, dword_ptr[esp]);
		cvttpd2dq(xmm0, xmm1);
		cvttpd2dq(xmm0, xmmword_ptr[esp]);
		cvttpd2pi(mm0, xmm1);
		cvttpd2pi(mm0, xmmword_ptr[esp]);
		cvttps2dq(xmm0, xmm1);
		cvttps2dq(xmm0, xmmword_ptr[esp]);
		cvttsd2si(eax, xmm1);
		cvttsd2si(eax, qword_ptr[esp]);
#ifdef JITASM64
		cvtdq2pd(xmm8, xmm9);
		cvtdq2pd(xmm8, qword_ptr[r8]);
		cvtpd2dq(xmm8, xmm9);
		cvtpd2dq(xmm8, xmmword_ptr[r8]);
		cvtpd2pi(mm0, xmm9);
		cvtpd2pi(mm0, xmmword_ptr[r8]);
		cvtpd2ps(xmm8, xmm9);
		cvtpd2ps(xmm8, xmmword_ptr[r8]);
		cvtpi2pd(xmm8, mm1);
		cvtpi2pd(xmm8, qword_ptr[r8]);
		cvtps2dq(xmm8, xmm9);
		cvtps2dq(xmm8, xmmword_ptr[r8]);
		cvtdq2ps(xmm8, xmm9);
		cvtdq2ps(xmm8, xmmword_ptr[r8]);
		cvtps2pd(xmm8, xmm9);
		cvtps2pd(xmm8, qword_ptr[r8]);
		cvtsd2ss(xmm8, xmm9);
		cvtsd2ss(xmm8, qword_ptr[r8]);
		cvtsi2sd(xmm8, r8);
		cvtsi2sd(xmm8, dword_ptr[r8]);
		cvtss2sd(xmm8, xmm9);
		cvtss2sd(xmm8, dword_ptr[r8]);
		cvttpd2dq(xmm8, xmm9);
		cvttpd2dq(xmm8, xmmword_ptr[r8]);
		cvttpd2pi(mm0, xmm9);
		cvttpd2pi(mm0, xmmword_ptr[r8]);
		cvttps2dq(xmm8, xmm9);
		cvttps2dq(xmm8, xmmword_ptr[r8]);
		cvttsd2si(rax, xmm1);
		cvttsd2si(rax, qword_ptr[r8]);
		cvttsd2si(r8, xmm1);
		cvttsd2si(r8, qword_ptr[r8]);
#endif
	}
};

//----------------------------------------
// MOVD/MOVQ
//----------------------------------------
extern "C" void masm_test_movd_movq();
struct test_movd_movq : jitasm::function0<void>
{
	virtual void naked_main()
	{
		movd(mm0, dword_ptr[eax]);
		movd(mm0, eax);
		movq(mm0, qword_ptr[eax]);
		movd(dword_ptr[eax], mm0);
		movd(eax, mm0);
		movq(qword_ptr[eax], mm0);
		movd(xmm0, dword_ptr[eax]);
		movd(xmm0, eax);
		movq(xmm0, qword_ptr[eax]);
		movd(dword_ptr[eax], xmm0);
		movd(eax, xmm0);
		movq(qword_ptr[eax], xmm0);
		movq(mm0, mm0);
		movq(mm0, qword_ptr[eax]);
		movq(qword_ptr[eax], mm0);
		movq(xmm0, xmm0);
		movq(xmm0, qword_ptr[eax]);
		movq(qword_ptr[eax], xmm0);
#ifdef JITASM64
		movd(mm0, rax);				// movq
		movd(rax, mm0);				// movq
		movd(xmm0, rax);			// movq
		movd(rax, xmm0);			// movq
		movd(mm0, dword_ptr[rax]);
		movq(mm0, qword_ptr[rax]);
		movd(dword_ptr[rax], mm0);
		movq(qword_ptr[rax], mm0);
		movd(xmm0, dword_ptr[rax]);
		movq(xmm0, qword_ptr[rax]);
		movd(dword_ptr[rax], xmm0);
		movq(qword_ptr[rax], xmm0);
		movq(mm0, qword_ptr[rax]);
		movq(qword_ptr[rax], mm0);
		movq(xmm0, qword_ptr[rax]);
		movq(qword_ptr[rax], xmm0);
		// test REX
		movd(mm0, r8);				// movq
		movd(r8, mm0);				// movq
		movd(xmm0, rax);			// movq
		movd(xmm0, r8);				// movq
		movd(xmm8, rax);			// movq
		movd(xmm8, r8);				// movq
		movd(rax, xmm0);			// movq
		movd(rax, xmm8);			// movq
		movd(r8, xmm0);				// movq
		movd(r8, xmm8);				// movq
		movd(mm0, dword_ptr[r8]);
		movq(mm0, qword_ptr[r8]);
		movd(dword_ptr[r8], mm0);
		movq(qword_ptr[r8], mm0);
		movd(xmm0, dword_ptr[rax]);
		movd(xmm0, dword_ptr[r8]);
		movd(xmm1, dword_ptr[rax]);
		movd(xmm1, dword_ptr[r8]);
		movq(xmm0, qword_ptr[rax]);
		movq(xmm0, qword_ptr[r8]);
		movq(xmm1, qword_ptr[rax]);
		movq(xmm1, qword_ptr[r8]);
		movd(dword_ptr[rax], xmm0);
		movd(dword_ptr[rax], xmm1);
		movd(dword_ptr[r8], xmm0);
		movd(dword_ptr[r8], xmm1);
		movq(qword_ptr[rax], xmm0);
		movq(qword_ptr[rax], xmm1);
		movq(qword_ptr[r8], xmm0);
		movq(qword_ptr[r8], xmm1);
#endif
	}
};

//----------------------------------------
// MOVSD/MOVSS
//----------------------------------------
extern "C" void masm_test_movsd_movss();
struct test_movsd_movss : jitasm::function0<void>
{
	virtual void naked_main()
	{
		movsd(xmm0, xmm1);
		movsd(xmm0, qword_ptr[esp]);
		movsd(qword_ptr[esp], xmm0);
		movss(xmm0, xmm1);
		movss(xmm0, dword_ptr[esp]);
		movss(dword_ptr[esp], xmm0);
#ifdef JITASM64
		movsd(xmm8, xmm1);
		movsd(xmm8, qword_ptr[rsp]);
		movsd(qword_ptr[r8], xmm8);
		movss(xmm8, xmm1);
		movss(xmm8, dword_ptr[rsp]);
		movss(dword_ptr[r8], xmm8);
#endif
	}
};

//----------------------------------------
// function0_cdecl<char>
//----------------------------------------
extern "C" void masm_test_function_return_char();
struct test_function_return_char : jitasm::function0_cdecl<char>
{
	Result main()
	{
		movzx(esi, cl);
		return cl;	// mov ax, cl
	}
};

//----------------------------------------
// function0_cdecl<short>
//----------------------------------------
extern "C" void masm_test_function_return_short();
struct test_function_return_short : jitasm::function0_cdecl<short>
{
	Result main()
	{
		return result_ptr[zsi];	// mov ax, word_ptr[zsi]
	}
};

//----------------------------------------
// function0_cdecl<int> (return immediate)
//----------------------------------------
extern "C" void masm_test_function_return_int_imm();
struct test_function_return_int_imm : jitasm::function0_cdecl<int>
{
	Result main()
	{
		return 16;	// mov eax, 16
	}
};

//----------------------------------------
// function0_cdecl<int> (return eax)
//----------------------------------------
extern "C" void masm_test_function_return_int_eax();
struct test_function_return_int_eax : jitasm::function0_cdecl<int>
{
	Result main()
	{
		return eax;	// no instruction. (because mov eax, eax)
	}
};

//----------------------------------------
// function0_cdecl<float> (return immediate)
//----------------------------------------
extern "C" void masm_test_function_return_float_imm();
struct test_function_return_float_imm : jitasm::function0_cdecl<float>
{
	Result main()
	{
		return 11.0f;
	}
};

//----------------------------------------
// function0_cdecl<float> (return xmm)
//----------------------------------------
extern "C" void masm_test_function_return_float_xmm();
struct test_function_return_float_xmm : jitasm::function0_cdecl<float>
{
	Result main()
	{
		movss(xmm7, dword_ptr[zsp]);
		return xmm7;
	}
};

//----------------------------------------
// function1_cdecl<float> (return ptr)
//----------------------------------------
extern "C" void masm_test_function_return_float_ptr();
struct test_function_return_float_ptr : jitasm::function1_cdecl<float, float>
{
	Result main(Arg a1)
	{
		return result_ptr[a1];
	}
};

//----------------------------------------
// function1_cdecl<float> (return st(0))
//----------------------------------------
extern "C" void masm_test_function_return_float_st0();
struct test_function_return_float_st0 : jitasm::function1_cdecl<float, float>
{
	Result main(Arg a1)
	{
		fld(real4_ptr[a1]);
		return st(0);
	}
};

//----------------------------------------
// function0_cdecl<double> (return immediate)
//----------------------------------------
extern "C" void masm_test_function_return_double_imm();
struct test_function_return_double_imm : jitasm::function0_cdecl<double>
{
	Result main()
	{
		return 11.0;
	}
};

//----------------------------------------
// function0_cdecl<double> (return xmm)
//----------------------------------------
extern "C" void masm_test_function_return_double_xmm();
struct test_function_return_double_xmm : jitasm::function0_cdecl<double>
{
	Result main()
	{
		movsd(xmm7, qword_ptr[zsp]);
		return xmm7;
	}
};

//----------------------------------------
// function1_cdecl<double> (return ptr)
//----------------------------------------
extern "C" void masm_test_function_return_double_ptr();
struct test_function_return_double_ptr : jitasm::function1_cdecl<double, double>
{
	Result main(Arg a1)
	{
		return result_ptr[a1];
	}
};

//----------------------------------------
// function1_cdecl<double> (return st(0))
//----------------------------------------
extern "C" void masm_test_function_return_double_st0();
struct test_function_return_double_st0 : jitasm::function1_cdecl<double, double>
{
	Result main(Arg a1)
	{
		fld(real8_ptr[a1]);
		return st(0);
	}
};

#include <intrin.h>
struct Foo {
	char c[5];
};

struct hoge : jitasm::function2_cdecl<__m64, short, int>
{
//	hoge() : jitasm::function2_cdecl<__int64, short, int>(false) {}

	virtual Result main(Arg a1, Arg a2)
	{
		movzx(eax, word_ptr[a1]);
		mov(ecx, dword_ptr[a2]);
		mov(esi, ecx);

		mov(byte_ptr[esp - 8], al);
		mov(byte_ptr[esp - 7], cl);
		mov(byte_ptr[esp - 6], 0xAA);
		//return result_ptr[esp - 8];
		return mm0;
		//return 0xFFFFFFFF;
	}
};

struct test_func : jitasm::function0<void>
{
	virtual void naked_main()
	{
		//xor(eax, eax);
		//mov(ecx, 100);
		//REPEAT();
		//	add(eax, ecx);
		//	dec(ecx);
		//UNTIL(ecx > jitasm::Imm32(0));
		//ret();

		xor(eax, eax);
		mov(ecx, 100);
		WHILE(ecx > 0);
			add(eax, ecx);
			dec(ecx);
		ENDW();
		ret();
	}
};

int wmain()
{
	TEST_M(test_sal);
	TEST_M(test_sar);
	TEST_M(test_shl);
	TEST_M(test_shr);
	TEST_M(test_rcl);
	TEST_M(test_rcr);
	TEST_M(test_rol);
	TEST_M(test_ror);
	TEST_M(test_inc_dec);
	TEST_M(test_push_pop);
	TEST_M(test_add);
	TEST_M(test_or);
	TEST_M(test_adc);
	TEST_M(test_sbb);
	TEST_M(test_and);
	TEST_M(test_sub);
	TEST_M(test_xor);
	TEST_M(test_cmp);
	TEST_M(test_xchg);
	TEST_M(test_test);
	TEST_M(test_mov);
	TEST_N(test_mov_disp);
	TEST_M(test_lea);
	TEST_M(test_fld);
	TEST_M(test_jmp);
	TEST_M(test_movs);
	TEST_M(test_neg_not);
	TEST_M(test_div_idiv_mul);
	TEST_M(test_imul);
	TEST_M(test_fst);
	TEST_M(test_sse2_a);
	TEST_M(test_movd_movq);
	TEST_M(test_movsd_movss);

	TEST_M(test_function_return_char);
	TEST_M(test_function_return_short);
	TEST_M(test_function_return_int_imm);
	TEST_M(test_function_return_int_eax);
	TEST_M(test_function_return_float_imm);
	TEST_M(test_function_return_float_xmm);
	TEST_M(test_function_return_float_ptr);
	TEST_M(test_function_return_float_st0);
	TEST_M(test_function_return_double_imm);
	TEST_M(test_function_return_double_xmm);
	TEST_M(test_function_return_double_ptr);
	TEST_M(test_function_return_double_st0);
}
