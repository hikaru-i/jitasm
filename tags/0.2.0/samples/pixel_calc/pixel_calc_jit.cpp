// Copyright (c) 2009, Hikaru Inoue, Akihiro Yamasaki,
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//    * The names of the contributors may not be used to endorse or promote
//      products derived from this software without specific prior written
//      permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include <boost/bind.hpp>
#include <boost/spirit.hpp>
#include "jitasm.h"


class RenderExpr : public jitasm::function<void, void *, size_t, void *, size_t, void *, size_t, int, int>
{
private:
	struct Calculator : public boost::spirit::grammar<Calculator>
	{
		RenderExpr& renderExpr_;
		Calculator(RenderExpr& renderExpr) : renderExpr_(renderExpr) {}

		template <typename ScannerT>
		struct definition
		{
			definition(Calculator const& self)
			{
				using namespace boost;
				using namespace boost::spirit;
				expression
					=	term
						>> *(   (L'+' >> term)[boost::bind(&do_add, ref(self.renderExpr_), _1, _2)]
							|	(L'-' >> term)[boost::bind(&do_sub, ref(self.renderExpr_), _1, _2)]
							);

				term
					=   factor
						>> *(   (L'*' >> factor)[boost::bind(&do_mul, ref(self.renderExpr_), _1, _2)]
							|   (L'/' >> factor)[boost::bind(&do_div, ref(self.renderExpr_), _1, _2)]
							);

				factor
					=   real_p[boost::bind(&do_real, ref(self.renderExpr_), _1)]
					|	chseq_p(L"src1")[boost::bind(&do_src1, ref(self.renderExpr_), _1, _2)]
					|	chseq_p(L"src2")[boost::bind(&do_src2, ref(self.renderExpr_), _1, _2)]
					|   L'(' >> expression >> L')'
					|   (L'-' >> factor)[boost::bind(&do_neg, ref(self.renderExpr_), _1, _2)]
					|   (L'+' >> factor);
			}

			boost::spirit::rule<ScannerT> expression, term, factor;

			boost::spirit::rule<ScannerT> const& start() const { return expression; }
		};
	};

	void do_add(const wchar_t *, const wchar_t *)
	{
		addps(jitasm::XmmReg(static_cast<jitasm::RegID>(regId_ - 1)), jitasm::XmmReg(static_cast<jitasm::RegID>(regId_)));
		--regId_;
	}

	void do_sub(const wchar_t *, const wchar_t *)
	{
		subps(jitasm::XmmReg(static_cast<jitasm::RegID>(regId_ - 1)), jitasm::XmmReg(static_cast<jitasm::RegID>(regId_)));
		--regId_;
	}

	void do_mul(const wchar_t *, const wchar_t *)
	{
		mulps(jitasm::XmmReg(static_cast<jitasm::RegID>(regId_ - 1)), jitasm::XmmReg(static_cast<jitasm::RegID>(regId_)));
		--regId_;
	}

	void do_div(const wchar_t *, const wchar_t *)
	{
		divps(jitasm::XmmReg(static_cast<jitasm::RegID>(regId_ - 1)), jitasm::XmmReg(static_cast<jitasm::RegID>(regId_)));
		--regId_;
	}

	void do_real(double val)
	{
		if (regId_ == lastRegId_) throw;
		float fval = static_cast<float>(val);
		mov(eax, *(unsigned int*)&fval);
		jitasm::XmmReg xmm(static_cast<jitasm::RegID>(++regId_));
		movd(xmm, eax);
		shufps(xmm, xmm, 0);
	}

	void do_src(int i)
	{
		if (regId_ == lastRegId_) throw;
		__declspec(align(16)) const static float factor8bpp[4] = {1.0f/255.0f, 1.0f/255.0f, 1.0f/255.0f, 1.0f/255.0f};
		jitasm::XmmReg xmm(static_cast<jitasm::RegID>(++regId_));
		movd(xmm, dword_ptr[i == 0 ? zsi : zbx]);
		punpcklbw(xmm, xmm0);
		punpcklwd(xmm, xmm0);
		cvtdq2ps(xmm, xmm);
		mov(zax, (uintptr_t)factor8bpp);
		mulps(xmm, xmmword_ptr[zax]);
	}

	void do_src1(const wchar_t *, const wchar_t *)
	{
		do_src(0);
	}

	void do_src2(const wchar_t *, const wchar_t *)
	{
		do_src(1);
	}

	void do_neg(const wchar_t *, const wchar_t *)
	{
		jitasm::XmmReg xmm(static_cast<jitasm::RegID>(regId_));
		subps(xmm0, xmm);
		movaps(xmm, xmm0);
		pxor(xmm0, xmm0);
	}

public:
	RenderExpr(wchar_t *expr) : expr_(expr), regId_(jitasm::XMM0),
#ifdef JITASM64
		lastRegId_(jitasm::XMM15)
#else
		lastRegId_(jitasm::XMM7)
#endif
	{}

	void main(Arg dst, Arg dstSkip, Arg src1, Arg src1Skip, Arg src2, Arg src2Skip, Arg width, Arg height)
	{
		mov(zsi, zword_ptr[src1]);
		mov(zbx, zword_ptr[src2]);
		mov(zdi, zword_ptr[dst]);
		pxor(xmm0, xmm0);

		L("LoopY");
		{
			mov(ecx, dword_ptr[width]);

			L("LoopX");
			{
				Calculator calc(*this);
				boost::spirit::parse_info<const wchar_t *> info = boost::spirit::parse(expr_, calc, boost::spirit::space_p);
				if (!info.full)
					throw info;

				__declspec(align(16)) const static float factor8bpp[4] = {255.0f, 255.0f, 255.0f, 255.0f};
				mov(zax, (uintptr_t)factor8bpp);
				mulps(xmm1, xmmword_ptr[zax]);
				cvtps2dq(xmm1, xmm1);
				packssdw(xmm1, xmm1);
				packuswb(xmm1, xmm1);
				movd(eax, xmm1);
				movnti(dword_ptr[zdi], eax);

				add(zsi, 4);
				add(zbx, 4);
				add(zdi, 4);
				dec(ecx);
				jnz("LoopX");
			}

			add(zsi, zword_ptr[src1Skip]);
			add(zbx, zword_ptr[src2Skip]);
			add(zdi, zword_ptr[dstSkip]);
			dec(dword_ptr[height]);
			jnz("LoopY");
		}
	}

private:
	wchar_t *expr_;
	int regId_;
	int lastRegId_;
};

bool RenderJIT(wchar_t *expr, void *dst, size_t dstStride, void *src1, size_t src1Stride, void *src2, size_t src2Stride, int width, int height)
{
	try {
		RenderExpr renderExpr(expr);
		renderExpr(dst, dstStride - width * 4, src1, src1Stride - width * 4, src2, src2Stride - width * 4, width, height);

		//FILE *file = fopen("render.dmp", "wb");
		//fwrite(renderExpr.GetCode(), 1, renderExpr.GetCodeSize(), file);
		//fclose(file);

		return true;
	}
	catch (...) {
		return false;
	}
}
