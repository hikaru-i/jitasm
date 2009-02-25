#include "StdAfx.h"
#include "jitasm.h"
#include <assert.h>

#define ASSERT	assert


extern "C" void masm_test_shift();


struct test_add : jitasm::function0<void>
{
	virtual void main()
	{
		add(al, 1);
		add(ax, 1);
		add(eax, 1);
		add(ax, 0x100);
		add(eax, 0x10000);
		add(eax, ecx);
		add(ecx, eax);
		add(eax, dword_ptr[ecx]);
		add(dword_ptr[eax], ecx);
		or(eax, ecx);
		adc(eax, ecx);
		sbb(eax, ecx);
		and(eax, ecx);
		sub(eax, ecx);
		xor(eax, ecx);
		cmp(eax, ecx);
#	ifdef JITASM64
		add(rax, 1);
		add(r8, 1);
		add(rax, 0x100);
		add(rax, 0x10000);
		add(rax, r8);
		add(r8, rax);
		add(rax, qword_ptr[r8]);
		add(qword_ptr[rax], r8);
#	endif
	}
};

struct test_inc_dec : jitasm::function0<void>
{
	virtual void main()
	{
		inc(al);
		inc(ax);
		inc(eax);
		inc(word_ptr[eax]);
		inc(dword_ptr[eax]);
		dec(al);
		dec(ax);
		dec(eax);
		dec(word_ptr[eax]);
		dec(dword_ptr[eax]);
	}
};

struct test_push_pop : jitasm::function0<void>
{
	virtual void main()
	{
#ifdef JITASM64
		push(ax);
		push(rax);
		push(1);
		push(0x100);
		push(word_ptr[eax]);
		push(qword_ptr[eax]);
		push(qword_ptr[rax]);
		pop(ax);
		pop(rax);
		pop(word_ptr[eax]);
		pop(qword_ptr[eax]);
		pop(qword_ptr[rax]);
#else
		push(ax);
		push(eax);
		push(1);
		push(0x100);
		push(word_ptr[eax]);
		push(dword_ptr[eax]);
		pop(ax);
		pop(eax);
		pop(word_ptr[eax]);
		pop(dword_ptr[eax]);
#endif
	}
};

struct test_lea : jitasm::function0<void>
{
	virtual void main()
	{
		lea(ax, word_ptr[eax]);
		lea(eax, dword_ptr[eax]);
		lea(eax, dword_ptr[eax + 1]);
		lea(eax, dword_ptr[ecx * 4]);
		lea(eax, dword_ptr[ecx * 4 + 1]);
		lea(eax, dword_ptr[eax + ecx]);
		lea(eax, dword_ptr[eax + ecx * 4]);
		lea(eax, dword_ptr[eax + ecx * 4 + 1]);
#ifdef JITASM64
		lea(ax, word_ptr[rax]);
		lea(eax, dword_ptr[rax]);
		lea(eax, dword_ptr[rax + 1]);
		lea(eax, dword_ptr[rcx * 4]);
		lea(eax, dword_ptr[rcx * 4 + 1]);
		lea(eax, dword_ptr[rax + rcx]);
		lea(eax, dword_ptr[rax + rcx * 4]);
		lea(eax, dword_ptr[rax + rcx * 4 + 1]);
#endif
	}
};

struct test_mov : jitasm::function0<void>
{
	virtual void main()
	{
		mov(al, cl);
		mov(byte_ptr[eax], cl);
		mov(al, byte_ptr[ecx]);
		mov(al, -1);
		mov(ax, cx);
		mov(word_ptr[eax], cx);
		mov(ax, word_ptr[ecx]);
		mov(ax, -1);
		mov(eax, ecx);
		mov(dword_ptr[eax], ecx);
		mov(eax, dword_ptr[ecx]);
		mov(eax, -1);
		movzx(ax, cl);
		movzx(ax, byte_ptr[ecx]);
		movzx(eax, ch);
		movzx(eax, byte_ptr[ecx]);
		movzx(eax, cx);
		movzx(eax, word_ptr[ecx]);
#ifdef JITASM64
		mov(rax, rcx);
		mov(qword_ptr[rax], rcx);
		mov(rax, qword_ptr[rcx]);
		mov(rax, 1);
		mov(rax, -1);
		mov(rax, 0x10000000);
		mov(rax, 0x80000000);
		mov(rax, 0x100000000);
		mov(rax, 0x800000000);
		movzx(rax, cl);
		movzx(rax, byte_ptr[rcx]);
		movzx(rax, cx);
		movzx(rax, word_ptr[rcx]);
		movzx(r8, cl);
		movzx(r8, byte_ptr[rcx]);
		movzx(r8, cx);
		movzx(r8, word_ptr[rcx]);
#endif
	}
};

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

struct test_jmp : jitasm::function0<void>
{
	virtual void main()
	{
		jmp("1");
		ja("1");
		jae("1");
		jb("1");
		jbe("1");
		jc("1");
#ifdef JITASM64
		jrcxz("1");
#else
		jcxz("1");
#endif
		jecxz("1");
		je("1");
		jg("1");
		jge("1");
		jl("1");
		jle("1");
		jna("1");
		jnae("1");
		jnb("1");
		jnbe("1");
		jnc("1");
		jne("1");
		jng("1");
		jnge("1");
		jnl("1");
		jnle("1");
		jno("1");
		jnp("1");
		jns("1");
		jnz("1");
		jo("1");
		jp("1");
		jpe("1");
		jpo("1");
		js("1");
		jz("1");
		label("1");
		for (int i = 0; i < 256; i++) nop();
		jmp("1");
		jz("1");
	}
};

struct test_xchg : jitasm::function0<void>
{
	virtual void main()
	{
		xchg(al, cl);
		xchg(cl, al);
		xchg(al, byte_ptr[ecx]);
		xchg(byte_ptr[eax], cl);
		xchg(ax, cx);
		xchg(cx, ax);
		xchg(ax, word_ptr[ecx]);
		xchg(word_ptr[eax], cx);
		xchg(eax, ecx);
		xchg(ecx, eax);
		xchg(eax, dword_ptr[ecx]);
		xchg(dword_ptr[eax], ecx);
#ifdef JITASM64
		xchg(rax, r8);
		xchg(rax, qword_ptr[r8]);
		xchg(qword_ptr[rax], r8);
		xchg(r8, rax);
		xchg(r8, qword_ptr[rax]);
		xchg(qword_ptr[r8], rax);
#endif
	}
};

struct test_test : jitasm::function0<void>
{
	virtual void main()
	{
		test(al, 1);
		test(cl, 1);
		test(ax, 1);
		test(cx, 1);
		test(eax, 1);
		test(ecx, 1);
		test(al, cl);
		test(ax, cx);
		test(eax, ecx);
		test(byte_ptr[eax], 1);
		test(word_ptr[eax], 1);
		test(dword_ptr[eax], 1);
		test(byte_ptr[eax], cl);
		test(word_ptr[eax], cx);
		test(dword_ptr[eax], ecx);
#	ifdef JITASM64
		test(rax, 1);
		test(rax, r8);
		test(qword_ptr[eax], 1);
		test(qword_ptr[eax], r8);
		test(r8, 1);
		test(r8, rax);
		test(qword_ptr[r8], r9);
		test(qword_ptr[r8], r9);
#	endif
	}
};

struct test_func : jitasm::function0<int>
{
	virtual jitasm::Opd main()
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
		return zax;
	}
};

extern "C" void hoge2(PBYTE pDst, float a, PBYTE pSrc, int nLen);

struct jitasm_test_shift : jitasm::function0<void>
{
	virtual void naked_main()
	{
		sal(al, 1);
		sal(al, 2);
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
		sar(al, 1);
		sar(al, 2);
		shl(al, 1);
		shl(al, 2);
		shr(al, 1);
		shr(al, 2);
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

template<class Ty1, class Ty2>
void TEST(Ty1 func1, Ty2 func2)
{
	func2.Assemble();
	unsigned char* p1 = (unsigned char*) func1;
	unsigned char* p2 = (unsigned char*) func2.GetCode();
	size_t size = func2.GetCodeSize();

	for (size_t i = 0; i < size; i++) {
		ASSERT(p1[i] == p2[i]);
	}
}

void* get_func_ptr(void* func)
{
	unsigned char* p = (unsigned char*) func;
	if (*p == 0xE9) {
		p = (unsigned char*) (p + (unsigned long&) *(p + 1) + 5);
	}
	return p;
}

int _tmain(int argc, _TCHAR* argv[])
{
	TEST(get_func_ptr(masm_test_shift), jitasm_test_shift());
	//hoge2(NULL, 1.0f, NULL, 1);
}
