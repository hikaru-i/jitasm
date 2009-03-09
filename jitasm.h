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

#pragma once
#ifndef JITASM_H
#define JITASM_H

#include <windows.h>
#include <string>
#include <deque>
#include <vector>
#include <assert.h>

#ifndef ASSERT
#define ASSERT assert
#endif

#if defined(_WIN64) && (defined(_M_AMD64) || defined(_M_X64))
#define JITASM64
#endif

namespace jitasm
{

typedef signed __int8		sint8;
typedef signed __int16		sint16;
typedef signed __int32		sint32;
typedef signed __int64		sint64;
typedef unsigned __int8		uint8;
typedef unsigned __int16	uint16;
typedef unsigned __int32	uint32;
typedef unsigned __int64	uint64;

namespace detail
{
	inline bool IsInt8(sint64 n) {return (sint8) n == n;}
	inline bool IsInt16(sint64 n) {return (sint16) n == n;}
	inline bool IsInt32(sint64 n) {return (sint32) n == n;}
}	// namespace detail

/// Operand type
enum OpdType
{
	O_TYPE_NONE,
	O_TYPE_REG,
	O_TYPE_MEM,
	O_TYPE_IMM,
};

/// Operand size
enum OpdSize
{
	O_SIZE_8 = 8,
	O_SIZE_16 = 16,
	O_SIZE_32 = 32,
	O_SIZE_64 = 64,
	O_SIZE_80 = 80,
	O_SIZE_128 = 128,
};

/// Register ID
enum RegID
{
	INVALID=-1,

	EAX=0, ECX, EDX, EBX, ESP, EBP, ESI, EDI,
	AL=0, CL, DL, BL, AH, CH, DH, BH,
	AX=0, CX, DX, BX, SP, BP, SI, DI,
	RAX=0, RCX, RDX, RBX, RSP, RBP, RSI, RDI,

	R8=0x10, R9, R10, R11, R12, R13, R14, R15,
	R8B=0x10, R9B, R10B, R11B, R12B, R13B, R14B, R15B,
	R8W=0x10, R9W, R10W, R11W, R12W, R13W, R14W, R15W,
	R8D=0x10, R9D, R10D, R11D, R12D, R13D, R14D, R15D,

	ST0=0, ST1, ST2, ST3, ST4, ST5, ST6, ST7,

	MM0=0, MM1, MM2, MM3, MM4, MM5, MM6, MM7,
	XMM0=0, XMM1, XMM2, XMM3, XMM4, XMM5, XMM6, XMM7,
	XMM8=0x10, XMM9, XMM10, XMM11, XMM12, XMM13, XMM14, XMM15,
};

/// Operand base class
struct Opd
{
	OpdType	opdtype_;
	OpdSize opdsize_;

	union {
		// REG
		RegID reg_;
		// MEM
		struct {
			RegID	base_;
			RegID	index_;
			sint64	scale_;
			sint64	disp_;
			OpdSize	addrsize_;
		};
		// IMM
		sint64 imm_;
	};

	/// NONE
	Opd() : opdtype_(O_TYPE_NONE) {}
	/// REG
	explicit Opd(OpdSize opdsize, RegID reg) : opdtype_(O_TYPE_REG), opdsize_(opdsize), reg_(reg) {}
	/// MEM
	Opd(OpdSize opdsize, OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
		: opdtype_(O_TYPE_MEM), opdsize_(opdsize), addrsize_(addrsize), base_(base), index_(index), scale_(scale), disp_(disp) {}
protected:
	/// IMM
	explicit Opd(sint64 imm) : opdtype_(O_TYPE_IMM), imm_(imm)
	{
		if (detail::IsInt8(imm)) opdsize_ = O_SIZE_8;
		else if (detail::IsInt16(imm)) opdsize_ = O_SIZE_16;
		else if (detail::IsInt32(imm)) opdsize_ = O_SIZE_32;
		else opdsize_ = O_SIZE_64;
	}

public:
	bool	IsNone() const {return opdtype_ == O_TYPE_NONE;}
	bool	IsReg() const {return opdtype_ == O_TYPE_REG;}
	bool	IsMem() const {return opdtype_ == O_TYPE_MEM;}
	bool	IsImm() const {return opdtype_ == O_TYPE_IMM;}
	OpdSize	GetSize() const {return opdsize_;}
	OpdSize	GetAddressSize() const {return addrsize_;}

	RegID	GetReg() const {return reg_;}
	RegID	GetBase() const {return base_;}
	RegID	GetIndex() const {return index_;}
	sint64	GetScale() const {return scale_;}
	sint64	GetDisp() const {return disp_;}
	sint64	GetImm() const {return imm_;}
};

template<int Size>
struct OpdT : Opd
{
	/// REG
	explicit OpdT(RegID reg) : Opd(static_cast<OpdSize>(Size), reg) {}
	/// MEM
	OpdT(OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
		: Opd(static_cast<OpdSize>(Size), addrsize, base, index, scale, disp) {}
protected:
	/// IMM
	OpdT(sint64 imm) : Opd(imm) {}
};
typedef OpdT<O_SIZE_8>		Opd8;
typedef OpdT<O_SIZE_16>		Opd16;
typedef OpdT<O_SIZE_32>		Opd32;
typedef OpdT<O_SIZE_64>		Opd64;
typedef OpdT<O_SIZE_80>		Opd80;
typedef OpdT<O_SIZE_128>	Opd128;

template<class OpdN>
struct RegT : OpdN
{
	explicit RegT(RegID reg) : OpdN(reg) {}
};
typedef RegT<Opd8>		Reg8;
typedef RegT<Opd16>		Reg16;
typedef RegT<Opd32>		Reg32;
#ifdef JITASM64
typedef RegT<Opd64>		Reg64;
#endif
typedef RegT<Opd80>		FpuReg;
typedef RegT<Opd64>		MmxReg;
typedef RegT<Opd128>	XmmReg;

#ifdef JITASM64
struct Rax : Reg64
{
	Rax() : Reg64(RAX) {}
};
#endif

template<class OpdN>
struct MemT : OpdN
{
	MemT(OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp) : OpdN(addrsize, base, index, scale, disp) {}
};
typedef MemT<Opd8>		Mem8;
typedef MemT<Opd16>		Mem16;
typedef MemT<Opd32>		Mem32;
typedef MemT<Opd64>		Mem64;
typedef MemT<Opd80>		Mem80;
typedef MemT<Opd128>	Mem128;

struct Mem64Offset
{
	sint64 offset_;

	explicit Mem64Offset(sint64 offset) : offset_(offset) {}
	Mem64 ToMem64() {return Mem64(O_SIZE_64, INVALID, INVALID, 0, offset_);}
};

template<class OpdN, class U, class S>
struct ImmT : OpdN
{
	ImmT(U imm) : OpdN((S) imm) {}
};
typedef ImmT<Opd8, uint8, sint8>	Imm8;
typedef ImmT<Opd16, uint16, sint16>	Imm16;
typedef ImmT<Opd32, uint32, sint32>	Imm32;
typedef ImmT<Opd64, uint64, sint64>	Imm64;

struct Reg32Expr
{
	RegID reg_;
	sint64 disp_;
	Reg32Expr(const Reg32& obj) : reg_(obj.reg_), disp_(0) {}	// implicit
	Reg32Expr(RegID reg, sint64 disp) : reg_(reg), disp_(disp) {}
};
inline Reg32Expr operator+(const Reg32& lhs, sint64 rhs) {return Reg32Expr(lhs.reg_, rhs);}
inline Reg32Expr operator+(sint64 lhs, const Reg32& rhs) {return rhs + lhs;}
inline Reg32Expr operator-(const Reg32& lhs, sint64 rhs) {return lhs + -rhs;}
inline Reg32Expr operator+(const Reg32Expr& lhs, sint64 rhs) {return Reg32Expr(lhs.reg_, lhs.disp_ + rhs);}
inline Reg32Expr operator+(sint64 lhs, const Reg32Expr& rhs) {return rhs + lhs;}
inline Reg32Expr operator-(const Reg32Expr& lhs, sint64 rhs) {return lhs + -rhs;}

struct Reg32ExprBI
{
	RegID base_;
	RegID index_;
	sint64 disp_;
	Reg32ExprBI(RegID base, RegID index, sint64 disp) : base_(base), index_(index), disp_(disp) {}
};
inline Reg32ExprBI operator+(const Reg32Expr& lhs, const Reg32Expr& rhs) {return Reg32ExprBI(rhs.reg_, lhs.reg_, lhs.disp_ + rhs.disp_);}
inline Reg32ExprBI operator+(const Reg32ExprBI& lhs, sint64 rhs) {return Reg32ExprBI(lhs.base_, lhs.index_, lhs.disp_ + rhs);}
inline Reg32ExprBI operator+(sint64 lhs, const Reg32ExprBI& rhs) {return rhs + lhs;}
inline Reg32ExprBI operator-(const Reg32ExprBI& lhs, sint64 rhs) {return lhs + -rhs;}

struct Reg32ExprSI
{
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Reg32ExprSI(RegID index, sint64 scale, sint64 disp) : index_(index), scale_(scale), disp_(disp) {}
};
inline Reg32ExprSI operator*(const Reg32& lhs, sint64 rhs) {return Reg32ExprSI(lhs.reg_, rhs, 0);}
inline Reg32ExprSI operator*(sint64 lhs, const Reg32& rhs) {return rhs * lhs;}
inline Reg32ExprSI operator*(const Reg32ExprSI& lhs, sint64 rhs) {return Reg32ExprSI(lhs.index_, lhs.scale_ * rhs, lhs.disp_);}
inline Reg32ExprSI operator*(sint64 lhs, const Reg32ExprSI& rhs) {return rhs * lhs;}
inline Reg32ExprSI operator+(const Reg32ExprSI& lhs, sint64 rhs) {return Reg32ExprSI(lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Reg32ExprSI operator+(sint64 lhs, const Reg32ExprSI& rhs) {return rhs + lhs;}
inline Reg32ExprSI operator-(const Reg32ExprSI& lhs, sint64 rhs) {return lhs + -rhs;}

struct Reg32ExprSIB
{
	RegID base_;
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Reg32ExprSIB(RegID base, RegID index, sint64 scale, sint64 disp) : base_(base), index_(index), scale_(scale), disp_(disp) {}
};
inline Reg32ExprSIB operator+(const Reg32Expr& lhs, const Reg32ExprSI& rhs) {return Reg32ExprSIB(lhs.reg_, rhs.index_, rhs.scale_, lhs.disp_ + rhs.disp_);}
inline Reg32ExprSIB operator+(const Reg32ExprSI& lhs, const Reg32Expr& rhs) {return rhs + lhs;}
inline Reg32ExprSIB operator+(const Reg32ExprSIB& lhs, sint64 rhs) {return Reg32ExprSIB(lhs.base_, lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Reg32ExprSIB operator+(sint64 lhs, const Reg32ExprSIB& rhs) {return rhs + lhs;}
inline Reg32ExprSIB operator-(const Reg32ExprSIB& lhs, sint64 rhs) {return lhs + -rhs;}

#ifdef JITASM64
struct Reg64Expr
{
	RegID reg_;
	sint64 disp_;
	Reg64Expr(const Reg64& obj) : reg_(obj.reg_), disp_(0) {}	// implicit
	Reg64Expr(RegID reg, sint64 disp) : reg_(reg), disp_(disp) {}
};
inline Reg64Expr operator+(const Reg64& lhs, sint64 rhs) {return Reg64Expr(lhs.reg_, rhs);}
inline Reg64Expr operator+(sint64 lhs, const Reg64& rhs) {return rhs + lhs;}
inline Reg64Expr operator-(const Reg64& lhs, sint64 rhs) {return lhs + -rhs;}
inline Reg64Expr operator+(const Reg64Expr& lhs, sint64 rhs) {return Reg64Expr(lhs.reg_, lhs.disp_ + rhs);}
inline Reg64Expr operator+(sint64 lhs, const Reg64Expr& rhs) {return rhs + lhs;}
inline Reg64Expr operator-(const Reg64Expr& lhs, sint64 rhs) {return lhs + -rhs;}

struct Reg64ExprBI
{
	RegID base_;
	RegID index_;
	sint64 disp_;
	Reg64ExprBI(RegID base, RegID index, sint64 disp) : base_(base), index_(index), disp_(disp) {}
};
inline Reg64ExprBI operator+(const Reg64Expr& lhs, const Reg64Expr& rhs) {return Reg64ExprBI(rhs.reg_, lhs.reg_, lhs.disp_ + rhs.disp_);}
inline Reg64ExprBI operator+(const Reg64ExprBI& lhs, sint64 rhs) {return Reg64ExprBI(lhs.base_, lhs.index_, lhs.disp_ + rhs);}
inline Reg64ExprBI operator+(sint64 lhs, const Reg64ExprBI& rhs) {return rhs + lhs;}
inline Reg64ExprBI operator-(const Reg64ExprBI& lhs, sint64 rhs) {return lhs + -rhs;}

struct Reg64ExprSI
{
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Reg64ExprSI(RegID index, sint64 scale, sint64 disp) : index_(index), scale_(scale), disp_(disp) {}
};
inline Reg64ExprSI operator*(const Reg64& lhs, sint64 rhs) {return Reg64ExprSI(lhs.reg_, rhs, 0);}
inline Reg64ExprSI operator*(sint64 lhs, const Reg64& rhs) {return rhs * lhs;}
inline Reg64ExprSI operator*(const Reg64ExprSI& lhs, sint64 rhs) {return Reg64ExprSI(lhs.index_, lhs.scale_ * rhs, lhs.disp_);}
inline Reg64ExprSI operator*(sint64 lhs, const Reg64ExprSI& rhs) {return rhs * lhs;}
inline Reg64ExprSI operator+(const Reg64ExprSI& lhs, sint64 rhs) {return Reg64ExprSI(lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Reg64ExprSI operator+(sint64 lhs, const Reg64ExprSI& rhs) {return rhs + lhs;}
inline Reg64ExprSI operator-(const Reg64ExprSI& lhs, sint64 rhs) {return lhs + -rhs;}

struct Reg64ExprSIB
{
	RegID base_;
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Reg64ExprSIB(RegID base, RegID index, sint64 scale, sint64 disp) : base_(base), index_(index), scale_(scale), disp_(disp) {}
};
inline Reg64ExprSIB operator+(const Reg64Expr& lhs, const Reg64ExprSI& rhs) {return Reg64ExprSIB(lhs.reg_, rhs.index_, rhs.scale_, lhs.disp_ + rhs.disp_);}
inline Reg64ExprSIB operator+(const Reg64ExprSI& lhs, const Reg64Expr& rhs) {return rhs + lhs;}
inline Reg64ExprSIB operator+(const Reg64ExprSIB& lhs, sint64 rhs) {return Reg64ExprSIB(lhs.base_, lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Reg64ExprSIB operator+(sint64 lhs, const Reg64ExprSIB& rhs) {return rhs + lhs;}
inline Reg64ExprSIB operator-(const Reg64ExprSIB& lhs, sint64 rhs) {return lhs + -rhs;}
#endif

template<typename OpdN>
struct AddressingPtr
{
	// 32bit-Addressing
	MemT<OpdN> operator[](const Reg32Expr& obj)		{return MemT<OpdN>(O_SIZE_32, obj.reg_, INVALID, 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg32ExprBI& obj)	{return MemT<OpdN>(O_SIZE_32, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg32ExprSI& obj)	{return MemT<OpdN>(O_SIZE_32, INVALID, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Reg32ExprSIB& obj)	{return MemT<OpdN>(O_SIZE_32, obj.base_, obj.index_, obj.scale_, obj.disp_);}
#ifdef JITASM64
	MemT<OpdN> operator[](sint32 disp)				{return MemT<OpdN>(O_SIZE_64, INVALID, INVALID, 0, disp);}
	MemT<OpdN> operator[](uint32 disp)				{return MemT<OpdN>(O_SIZE_64, INVALID, INVALID, 0, (sint32) disp);}
#else
	MemT<OpdN> operator[](sint32 disp)				{return MemT<OpdN>(O_SIZE_32, INVALID, INVALID, 0, disp);}
	MemT<OpdN> operator[](uint32 disp)				{return MemT<OpdN>(O_SIZE_32, INVALID, INVALID, 0, (sint32) disp);}
#endif

#ifdef JITASM64
	// 64bit-Addressing
	MemT<OpdN> operator[](const Reg64Expr& obj)		{return MemT<OpdN>(O_SIZE_64, obj.reg_, INVALID, 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg64ExprBI& obj)	{return MemT<OpdN>(O_SIZE_64, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg64ExprSI& obj)	{return MemT<OpdN>(O_SIZE_64, INVALID, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Reg64ExprSIB& obj)	{return MemT<OpdN>(O_SIZE_64, obj.base_, obj.index_, obj.scale_, obj.disp_);}
	Mem64Offset operator[](sint64 offset)			{return Mem64Offset(offset);}
	Mem64Offset operator[](uint64 offset)			{return Mem64Offset((sint64) offset);}
#endif
};

/// Instruction ID
enum InstrID
{
	I_ADC, I_ADD, I_AND, I_CALL, I_CMP, I_DEC, I_DIV, I_IDIV, I_IMUL, I_INC, I_INT3,
	I_JMP, I_JA, I_JAE, I_JB, I_JBE, I_JCXZ, I_JECXZ, I_JRCXZ, I_JE,
	I_JG, I_JGE, I_JL, I_JLE, I_JNE, I_JNO, I_JNP, I_JNS, I_JO, I_JP, I_JS,
	I_LEA, I_LEAVE,
	I_MOV, I_MOVS_B, I_MOVS_W, I_MOVS_D, I_MOVS_Q, I_REP_MOVS_B, I_REP_MOVS_W, I_REP_MOVS_D, I_REP_MOVS_Q, I_MOVZX,
	I_MUL, I_NEG, I_NOP, I_NOT, I_OR, I_POP, I_PUSH, I_RET,
	I_RCL, I_RCR, I_ROL, I_ROR, I_SAR, I_SHL, I_SHR, I_SBB, I_SUB, I_TEST, I_XCHG, I_XOR,

	I_FLD, I_FST, I_FSTP,

	I_ADDPD, I_ADDSD, I_ANDPD, I_ANDNPD, I_CLFLUSH, I_CMPPS, I_CMPPD, I_CMPSD, I_COMISD, I_CVTDQ2PD, I_CVTDQ2PS,
	I_CVTPD2DQ, I_CVTPD2PI, I_CVTPD2PS, I_CVTPI2PD, I_CVTPS2DQ, I_CVTPS2PD, I_CVTSD2SI, I_CVTSD2SS,
	I_CVTSI2SD, I_CVTSS2SD, I_CVTTPD2DQ, I_CVTTPD2PI, I_CVTTPS2DQ, I_CVTTSD2SI, I_DIVPD, I_DIVSD, I_LFENCE,
	I_MASKMOVDQU, I_MAXPD, I_MAXSD, I_MFENCE, I_MINPD, I_MINSD, I_MOVAPD, I_MOVD, I_MOVDQ2Q, I_MOVDQA,
	I_MOVDQU, I_MOVHPD, I_MOVLPD, I_MOVMSKPD, I_MOVNTPD, I_MOVNTDQ, I_MOVNTI, I_MOVQ, I_MOVQ2DQ, I_MOVSD, I_MOVSS, I_MOVUPD, I_MULPD,
	I_MULSD, I_ORPD, I_PABSB, I_PABSD, I_PABSW, I_PACKSSDW, I_PACKSSWB, I_PACKUSDW, I_PACKUSWB,
	I_PADDB, I_PADDD, I_PADDQ, I_PADDSB, I_PADDSW, I_PADDUSB, I_PADDUSW, I_PADDW, I_PALIGNR,
	I_PAND, I_PANDN, I_PAUSE, I_PAVGB, I_PAVGW, I_PCMPEQB, I_PCMPEQW, I_PCMPEQD, I_PCMPEQQ,
	I_PCMPGTB, I_PCMPGTW, I_PCMPGTD, I_PCMPGTQ, I_PEXTRW, I_PINSRW, I_PMADDWD,
	I_PMAXSW, I_PMAXUB, I_PMINSW, I_PMINUB, I_PMOVMSKB, I_PMULHUW, I_PMULHW, I_PMULLW, I_PMULUDQ,
	I_POR, I_PSADBW, I_PSHUFD, I_PSHUFHW, I_PSHUFLW, I_PSLLW, I_PSLLD, I_PSLLQ, I_PSLLDQ, I_PSRAW,
	I_PSRAD, I_PSRLW, I_PSRLD, I_PSRLQ, I_PSRLDQ, I_PSUBB, I_PSUBW, I_PSUBD, I_PSUBQ, I_PSUBSB, I_PSUBSW,
	I_PSUBUSB, I_PSUBUSW, I_PUNPCKHBW, I_PUNPCKHWD, I_PUNPCKHDQ, I_PUNPCKHQDQ, I_PUNPCKLBW, I_PUNPCKLWD, I_PUNPCKLDQ, I_PUNPCKLQDQ,
	I_PXOR, I_SHUFPD, I_SQRTPD, I_SQRTSD, I_SUBPD, I_SUBSD, I_UCOMISD, I_UNPCKHPD, I_UNPCKLPD, I_XORPD,

	I_PSEUDO_ALIGN,
};

/// Instruction
struct Instr
{
	static const size_t MAX_OPERAND_COUNT = 3;

	InstrID	id_;
	Opd		opd_[MAX_OPERAND_COUNT];

	explicit Instr(InstrID id) : id_(id) {}
	Instr(InstrID id, const Opd& opd1) : id_(id) {opd_[0] = opd1;}
	Instr(InstrID id, const Opd& opd1, const Opd& opd2) : id_(id) {opd_[0] = opd1, opd_[1] = opd2;}
	Instr(InstrID id, const Opd& opd1, const Opd& opd2, const Opd& opd3) : id_(id) {opd_[0] = opd1, opd_[1] = opd2, opd_[2] = opd3;}

	InstrID GetID() const {return id_;}
	const Opd& GetOpd(size_t index) const {return opd_[index];}
};

/// Assembler backend
struct Backend
{
	uint8*	pbuff_;
	size_t	buffsize_;
	size_t	size_;

	Backend(void* pbuff = NULL, size_t buffsize = 0) : pbuff_((uint8*) pbuff), buffsize_(buffsize), size_(0)
	{
		memset(pbuff, 0xCC, buffsize);	// INT3
	}

	size_t GetSize() const
	{
		return size_;
	}

	void put_bytes(void* p, size_t n)
	{
		uint8* pb = (uint8*) p;
		while (n--) {
			if (pbuff_) {
				if (size_ == buffsize_) ASSERT(0);
				pbuff_[size_] = *pb++;
			}
			size_++;
		}
	}
	void db(uint64 b) {put_bytes(&b, 1);}
	void dw(uint64 w) {put_bytes(&w, 2);}
	void dd(uint64 d) {put_bytes(&d, 4);}
	void dq(uint64 q) {put_bytes(&q, 8);}

	void EncodeAddressSizePrefix() {db(0x67);}
	void EncodeOperandSizePrefix() {db(0x66);}

	void EncodeRexWRXB(const Opd& r_m) {EncodeRexWRXB(Opd(), r_m);}
	void EncodeRexWRXB(const Opd& reg, const Opd& r_m)
	{
		uint8 wrxb = 0;
		if (reg.IsReg()) {
			if (reg.GetSize() == O_SIZE_64) wrxb |= 8;
			if (reg.GetReg() >= R8) wrxb |= 4;
		}
		if (r_m.IsReg()) {
			if (r_m.GetSize() == O_SIZE_64) wrxb |= 8;
			if (r_m.GetReg() >= R8) wrxb |= 1;
		}
		if (r_m.IsMem()) {
			if (r_m.GetSize() == O_SIZE_64) wrxb |= 8;
			if (r_m.GetIndex() >= R8) wrxb |= 2;
			if (r_m.GetBase() >= R8) wrxb |= 1;
		}
		if (wrxb) db(0x40 | wrxb);
	}

	void EncodeRexRXB(const Opd& r_m) {EncodeRexRXB(Opd(), r_m);}
	void EncodeRexRXB(const Opd& reg, const Opd& r_m)
	{
		uint8 rxb = 0;
		if (reg.IsReg()) {
			if (reg.GetReg() >= R8) rxb |= 4;
		}
		if (r_m.IsReg()) {
			if (r_m.GetReg() >= R8) rxb |= 1;
		}
		if (r_m.IsMem()) {
			if (r_m.GetIndex() >= R8) rxb |= 2;
			if (r_m.GetBase() >= R8) rxb |= 1;
		}
		if (rxb) db(0x40 | rxb);
	}

	void EncodeRep() {db(0xF3);}

	void EncodeModRM(uint8 reg, const Opd& r_m)
	{
		reg &= 0xF;

		if (r_m.IsReg()) {
			db(0xC0 | reg << 3 | r_m.GetReg() & 0xF);
		} else if (r_m.IsMem()) {
			int base = r_m.GetBase(); if (base != INVALID) base &= 0xF;
			int index = r_m.GetIndex(); if (index != INVALID) index &= 0xF;

			if (base == INVALID && index == INVALID) {
#ifdef JITASM64
				db(reg << 3 | 4);
				db(0x25);
#else
				db(reg << 3 | 5);
#endif
				dd(r_m.GetDisp());
			} else {
				ASSERT(base != ESP || index != ESP);
				ASSERT(index != ESP || r_m.GetScale() == 0);

				if (index == ESP) {
					index = base;
					base = ESP;
				}
				bool sib = index != INVALID || r_m.GetScale() || base == ESP;

				// ModR/M
				uint8 mod;
				if (r_m.GetDisp() == 0 || sib && base == INVALID) mod = base != EBP ? 0 : 1;
				else if (detail::IsInt8(r_m.GetDisp())) mod = 1;
				else if (detail::IsInt32(r_m.GetDisp())) mod = 2;
				else ASSERT(0);
				db(mod << 6 | reg << 3 | (sib ? 4 : base));

				// SIB
				if (sib) {
					uint8 ss;
					if (r_m.GetScale() == 0) ss = 0;
					else if (r_m.GetScale() == 2) ss = 1;
					else if (r_m.GetScale() == 4) ss = 2;
					else if (r_m.GetScale() == 8) ss = 3;
					else ASSERT(0);
					if (index != INVALID && base != INVALID) {
						db(ss << 6 | index << 3 | base);
					} else if (base != INVALID) {
						db(ss << 6 | 4 << 3 | base);
					} else if (index != INVALID) {
						db(ss << 6 | index << 3 | 5);
					} else {
						ASSERT(0);
					}
				}

				// Displacement
				if (mod == 0 && sib && base == INVALID) dd(r_m.GetDisp());
				if (mod == 1) db(r_m.GetDisp());
				if (mod == 2) dd(r_m.GetDisp());
			}
		} else {
			ASSERT(0);
		}
	}

	void EncodeModRM(const Opd& reg, const Opd& r_m)
	{
		ASSERT(reg.IsReg() && (r_m.IsReg() || r_m.IsMem()));
		EncodeModRM(reg.GetReg(), r_m);
	}

	void EncodeALU(uint8 opcode, const Opd& opd1, const Opd& opd2)
	{
		if (opd2.IsImm()) {
			const Opd& r_m = opd1;
			const Opd& imm = opd2;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(r_m);
#else
			if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			if (imm.GetSize() == O_SIZE_8 && r_m.GetSize() != O_SIZE_8) {	// sign-extension
				db(0x83);	// Immediate Grp 1
				EncodeModRM(opcode / 8, r_m);
				db(imm.GetImm());
			} else {
				uint8 w = r_m.GetSize() != O_SIZE_8 ? 1 : 0;
				if (r_m.IsReg() && r_m.GetReg() == EAX) {
					db(opcode | 4 | w);
				} else {
					db(0x80 | w);	// Immediate Grp 1
					EncodeModRM(opcode / 8, r_m);
				}
				if (r_m.GetSize() == O_SIZE_8) db(imm.GetImm());
				else if (r_m.GetSize() == O_SIZE_16) dw(imm.GetImm());
				else if (r_m.GetSize() == O_SIZE_32) dd(imm.GetImm());
				else if (r_m.GetSize() == O_SIZE_64) dd(imm.GetImm());
				else ASSERT(0);
			}
		} else {
			ASSERT(opd1.GetSize() == opd2.GetSize());

			const Opd& reg = opd1.IsReg() ? opd1 : opd2;
			const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(reg, r_m);
#else
			if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			uint8 d = opd1.IsReg() ? 1 : 0;
			uint8 w = reg.GetSize() != O_SIZE_8 ? 1 : 0;
			db(opcode | d << 1 | w);
			EncodeModRM(reg, r_m);
		}
	}

	void EncodeShift(uint8 digit, const Opd& r_m, const Opd& imm)
	{
		ASSERT(imm.IsImm());

#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(r_m);
#else
		if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif
		uint8 w = r_m.GetSize() != O_SIZE_8 ? 1 : 0;
		if (imm.GetImm() == 1) {
			db(0xD0 | w);
			EncodeModRM(digit, r_m);
		} else {
			db(0xC0 | w);
			EncodeModRM(digit, r_m);
			db(imm.GetImm());
		}
	}

	void EncodeGrp3(uint8 digit, const Opd& r_m)
	{
#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(r_m);
#else
		if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif
		uint8 w = r_m.GetSize() != O_SIZE_8 ? 1 : 0;
		db(0xF6 | w);
		EncodeModRM(digit, r_m);
	}

	void EncodeCALL(const Opd& dst)
	{
		if (dst.IsReg()) {
			if (dst.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			db(0xFF);
			EncodeModRM(2, dst);
		} else {
			// TODO: Support for relative displacement
			ASSERT(0);
		}
	}

	void EncodeINC_DEC(uint8 digit, const Opd& r_m)
	{
#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(r_m);
#else
		if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

		uint8 w = r_m.GetSize() != O_SIZE_8 ? 1 : 0;

#ifdef JITASM64
		db(0xFE | w);	// Grp4, Grp5
		EncodeModRM(digit, r_m);
#else
		if (r_m.IsReg() && w) {
			db(0x40 | digit << 3 | r_m.GetReg() & 0xF);
		} else {
			db(0xFE | w);	// Grp4, Grp5
			EncodeModRM(digit, r_m);
		}
#endif
	}

	void EncodeIMUL(const Opd& dst, const Opd& src, const Opd& imm)
	{
		if (src.IsNone() && imm.IsNone()) {
			EncodeGrp3(5, dst);
		} else {
			ASSERT(dst.IsReg());

#ifdef JITASM64
			if (src.IsMem() && src.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (dst.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(src);
#else
			if (dst.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			if (imm.IsNone()) {
				db(0x0F);
				db(0xAF);
				EncodeModRM(dst, src);
			} else {
				if (detail::IsInt8(imm.GetImm())) {
					db(0x6B);
					EncodeModRM(dst, src);
					db(imm.GetImm());
				} else {
					db(0x69);
					EncodeModRM(dst, src);
					if (dst.GetSize() == O_SIZE_16) dw(imm.GetImm());
					else if (dst.GetSize() == O_SIZE_32) dd(imm.GetImm());
					else if (dst.GetSize() == O_SIZE_64) dd(imm.GetImm());
					else ASSERT(0);
				}
			}
		}
	}

	void EncodeJCC(uint8 tttn, const Opd& imm)
	{
		if (imm.GetSize() <= O_SIZE_8) db(0x70 | tttn), db(imm.GetImm());
		else if (imm.GetSize() <= O_SIZE_32) db(0x0F), db(0x80 | tttn), dd(imm.GetImm());
		else ASSERT(0);
	}

	void EncodeJMP(const Opd& imm)
	{
		if (imm.GetSize() <= O_SIZE_8) db(0xEB), db(imm.GetImm());
		else if (imm.GetSize() <= O_SIZE_32) db(0xE9), dd(imm.GetImm());
		else ASSERT(0);
	}

	void EncodeLEA(const Opd& reg, const Opd& mem)
	{
#ifdef JITASM64
		if (mem.IsMem() && mem.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(reg, mem);
#else
		if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

		db(0x8D);
		EncodeModRM(reg, mem);
	}

	void EncodeMOV(const Opd& dst, const Opd& src)
	{
		if (src.IsImm()) {
			const Opd& r_m = dst;
			const Opd& imm = src;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(r_m);
#else
			if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

#ifdef JITASM64
			if (r_m.GetSize() == O_SIZE_64 && imm.GetSize() <= O_SIZE_32) {	// sign-extension
				db(0xC7);
				EncodeModRM(0, r_m);
				dd(imm.GetImm());
				return;
			}
#endif

			uint8 w = r_m.GetSize() != O_SIZE_8 ? 1 : 0;
			if (r_m.IsReg()) {
				db(0xB0 | w << 3 | r_m.GetReg() & 0xF);
			} else {
				db(0xC6 | w);
				EncodeModRM(0, r_m);
			}
			if (r_m.GetSize() == O_SIZE_8) db(imm.GetImm());
			else if (r_m.GetSize() == O_SIZE_8) db(imm.GetImm());
			else if (r_m.GetSize() == O_SIZE_16) dw(imm.GetImm());
			else if (r_m.GetSize() == O_SIZE_32) dd(imm.GetImm());
#ifdef JITASM64
			else if (r_m.GetSize() == O_SIZE_64) dq(imm.GetImm());
#endif
			else ASSERT(0);
		} else {
			ASSERT(dst.GetSize() == src.GetSize());

			const Opd& reg = dst.IsReg() ? dst : src;
			const Opd& r_m = dst.IsReg() ? src : dst;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(reg, r_m);
#else
			if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			uint8 w = reg.GetSize() != O_SIZE_8 ? 1 : 0;
#ifdef JITASM64
			if (reg.GetReg() == EAX && r_m.IsMem() && r_m.GetBase() == INVALID && r_m.GetIndex() == INVALID && !detail::IsInt32(r_m.GetDisp())) {
				uint8 d = dst.IsReg() ? 0 : 1;
				db(0xA0 | d << 1 | w);
				dq(r_m.GetDisp());
				return;
			}
#else
			if (reg.GetReg() == EAX && r_m.IsMem() && r_m.GetBase() == INVALID && r_m.GetIndex() == INVALID) {
				uint8 d = dst.IsReg() ? 0 : 1;
				db(0xA0 | d << 1 | w);
				dd(r_m.GetDisp());
				return;
			}
#endif
			uint8 d = dst.IsReg() ? 1 : 0;
			db(0x88 | d << 1 | w);
			EncodeModRM(reg, r_m);
		}
	}

	void EncodeMOVZX(const Opd& reg, const Opd& r_m)
	{
		ASSERT(reg.IsReg());

#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(reg, r_m);
#else
		if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

		db(0x0F);
		uint8 w = r_m.GetSize() != O_SIZE_8 ? 1 : 0;
		db(0xB6 | w);
		EncodeModRM(reg, r_m);
	}

	void EncodePUSH_POP(uint8 opcode1, uint8 opcode2, uint8 digit, const Opd& src)
	{
		if (src.IsImm()) {
			if (src.GetSize() == O_SIZE_8) {	// sign-extention
				db(0x6A);
				db(src.GetImm());
			} else {
				db(0x68);
				dd(src.GetImm());
			}
		} else {
#ifdef JITASM64
			if (src.IsMem() && src.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (src.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexRXB(src);
#else
			if (src.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			if (src.IsReg()) {
				db(opcode1 | src.GetReg() & 0xF);
			} else if (src.IsMem()) {
				db(opcode2);
				EncodeModRM(digit, src);
			}
		}
	}

	void EncodeTEST(const Opd& dst, const Opd& src)
	{
		uint8 w = dst.GetSize() != O_SIZE_8 ? 1 : 0;
		if (src.IsImm()) {
			const Opd& r_m = dst;
			const Opd& imm = src;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(r_m);
#else
			if (r_m.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			if (r_m.IsReg() && r_m.GetReg() == EAX) {
				db(0xA8 | w);
			} else {
				db(0xF6 | w);
				EncodeModRM(0, r_m);
			}
			if (r_m.GetSize() == O_SIZE_8) db(imm.GetImm());
			else if (r_m.GetSize() == O_SIZE_16) dw(imm.GetImm());
			else if (r_m.GetSize() == O_SIZE_32) dd(imm.GetImm());
			else if (r_m.GetSize() == O_SIZE_64) dd(imm.GetImm());	// sign-extention
		} else {
			ASSERT(src.IsReg());
			const Opd& reg = dst.IsReg() ? dst : src;
			const Opd& r_m = dst.IsReg() ? src : dst;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexWRXB(reg, r_m);
#else
			if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

			db(0x84 | w);
			EncodeModRM(reg, r_m);
		}
	}

	void EncodeXCHG(const Opd& dst, const Opd& src)
	{
		const Opd& reg = dst.IsReg() && (src.GetSize() == O_SIZE_8 || !src.IsReg() || src.GetReg() != EAX) ? dst : src;
		const Opd& r_m = &reg == &dst ? src : dst;

#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(reg, r_m);	// TODO: In 64-bit mode, r/m8 can not be encoded to access following byte registers if a REX prefix is used: AH, BH, CH, DH.
#else
		if (reg.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

		if (reg.GetSize() != O_SIZE_8 && r_m.IsReg()) {
			if (reg.GetReg() == EAX) {
				db(0x90 | r_m.GetReg() & 0xF);
				return;
			}
			if (r_m.GetReg() == EAX) {
				db(0x90 | reg.GetReg() & 0xF);
				return;
			}
		}

		uint8 w = reg.GetSize() != O_SIZE_8 ? 1 : 0;
		db(0x86 | w);
		EncodeModRM(reg, r_m);
	}

	void EncodeFLD(const Opd& mem)
	{
		if (mem.IsReg()) {
			db(0xD9);
			db(0xC0 | mem.GetReg());
		} else {
#ifdef JITASM64
			if (mem.IsMem() && mem.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
#endif
			int digit = 0;
			if (mem.GetSize() == O_SIZE_32) db(0xD9), digit = 0;
			else if (mem.GetSize() == O_SIZE_64) db(0xDD), digit = 0;
			else if (mem.GetSize() == O_SIZE_80) db(0xDB), digit = 5;
			else ASSERT(0);
			EncodeModRM(digit, mem);
		}
	}

	void EncodeFST(uint8 digit, uint8 opcode, const Opd& mem)
	{
		if (mem.IsReg()) {
			db(0xDD);
			db(opcode | mem.GetReg());
		} else {
#ifdef JITASM64
			if (mem.IsMem() && mem.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
#endif

			if (mem.GetSize() == O_SIZE_80) {
				db(0xDB);
				EncodeModRM(7, mem);
			} else {
				if (mem.GetSize() == O_SIZE_32) db(0xD9);
				else if (mem.GetSize() == O_SIZE_64) db(0xDD);
				else ASSERT(0);
				EncodeModRM(digit, mem);
			}
		}
	}

	void EncodeMMX(uint8 opcode2, const Opd& dst, const Opd& src)
	{
		const Opd& reg = dst.IsReg() ? dst : src;
		const Opd& r_m = dst.IsReg() ? src : dst;

#ifdef JITASM64
		EncodeRexWRXB(reg, r_m);
#endif
		db(0x0F);
		db(opcode2);
		EncodeModRM(reg, r_m);
	}

	void EncodeMMX(uint8 opcode2, uint8 opcode3, const Opd& dst, const Opd& src)
	{
		const Opd& reg = dst.IsReg() ? dst : src;
		const Opd& r_m = dst.IsReg() ? src : dst;

#ifdef JITASM64
		EncodeRexWRXB(reg, r_m);
#endif
		db(0x0F);
		db(opcode2);
		db(opcode3);
		EncodeModRM(reg, r_m);
	}

	void EncodeSSE(uint8 opcode2, const Opd& dst, const Opd& src)
	{
		EncodeMMX(opcode2, dst, src);
	}

	void EncodeSSE(uint8 opcode2, uint8 opcode3, const Opd& dst, const Opd& src)
	{
		EncodeMMX(opcode2, opcode3, dst, src);
	}

	/// Encode SSE2 instruction
	/**
	 * Encode 2 bytes opcode SSE2 instruction. First-byte opcode is 0x0F.
	 * \param prefix	A mandatory prefix for SSE2.
	 * \param opcode2	Second-byte opcode.
	 * \param dst		First operand.
	 * \param src		Second operand.
	 */
	void EncodeSSE2(uint8 prefix, uint8 opcode2, const Opd& dst, const Opd& src)
	{
		db(prefix);
		EncodeMMX(opcode2, dst, src);
	}

	/// Encode SSE2 instruction
	/**
	 * Encode 3 bytes opcode SSE2 instruction. First-byte opcode is 0x0F.
	 * \param prefix	A mandatory prefix for SSE2.
	 * \param opcode2	Second-byte opcode.
	 * \param opcode3	Third-byte opcode.
	 * \param dst		First operand.
	 * \param src		Second operand.
	 */
	void EncodeSSE2(uint8 prefix, uint8 opcode2, uint8 opcode3, const Opd& dst, const Opd& src)
	{
		db(prefix);
		EncodeMMX(opcode2, opcode3, dst, src);
	}

	void EncodeMMXorSSE2(uint8 prefix, uint8 opcode2, const Opd& dst, const Opd& src)
	{
		if ((dst.IsReg() && dst.GetSize() == O_SIZE_128) ||
			(src.IsReg() && src.GetSize() == O_SIZE_128)) {
			EncodeSSE2(prefix, opcode2, dst, src);
		}
		else {
			EncodeMMX(opcode2, dst, src);
		}
	}

	void EncodeMMXorSSE2(uint8 prefix, uint8 opcode2, uint8 opcode3, const Opd& dst, const Opd& src)
	{
		if ((dst.IsReg() && dst.GetSize() == O_SIZE_128) ||
			(src.IsReg() && src.GetSize() == O_SIZE_128)) {
			EncodeSSE2(prefix, opcode2, opcode3, dst, src);
		}
		else {
			EncodeMMX(opcode2, opcode3, dst, src);
		}
	}

	void EncodeCLFLUSH(const Opd& dst)
	{
		ASSERT(dst.IsMem() && dst.GetSize() == O_SIZE_8);
		db(0x0F);
		db(0xAE);
		EncodeModRM(7, dst);
	}

	void EncodeMOVQ(const Opd& dst, const Opd& src)
	{
		if ((dst.IsReg() ? dst : src).GetSize() == O_SIZE_64) {
			EncodeMMX(0x6F | (dst.IsReg() ? 0 : 0x10), dst, src);
		}
		else if (dst.IsReg()) {
			EncodeSSE2(0xF3, 0x7E, dst, src);
		}
		else {
			EncodeSSE2(0x66, 0xD6, dst, src);
		}
	}

	void Assemble(const Instr& instr)
	{
		const Opd& o1 = instr.GetOpd(0);
		const Opd& o2 = instr.GetOpd(1);
		const Opd& o3 = instr.GetOpd(2);

		switch (instr.GetID()) {
		// General-Purpose Instructions
		case I_ADC:			EncodeALU(0x10, o1, o2); break;
		case I_ADD:			EncodeALU(0x00, o1, o2); break;
		case I_AND:			EncodeALU(0x20, o1, o2); break;
		case I_CALL:		EncodeCALL(o1); break;
		case I_CMP:			EncodeALU(0x38, o1, o2); break;
		case I_DEC:			EncodeINC_DEC(1, o1); break;
		case I_DIV:			EncodeGrp3(6, o1); break;
		case I_IDIV:		EncodeGrp3(7, o1); break;
		case I_IMUL:		EncodeIMUL(o1, o2, o3); break;
		case I_INC:			EncodeINC_DEC(0, o1); break;
		case I_INT3:		db(0xCC); break;
		case I_JMP:			EncodeJMP(o1); break;
		case I_JA:			EncodeJCC(0x7, o1); break;
		case I_JAE:			EncodeJCC(0x3, o1); break;
		case I_JB:			EncodeJCC(0x2, o1); break;
		case I_JBE:			EncodeJCC(0x6, o1); break;
#ifdef JITASM64
		case I_JECXZ:		EncodeAddressSizePrefix(); db(0xE3); db(o1.GetImm()); break;
		case I_JRCXZ:		db(0xE3); db(o1.GetImm()); break;
#else
		case I_JECXZ:		db(0xE3); db(o1.GetImm()); break;
		case I_JCXZ:		EncodeAddressSizePrefix(); db(0xE3); db(o1.GetImm()); break;
#endif
		case I_JE:			EncodeJCC(0x4, o1); break;
		case I_JG:			EncodeJCC(0xF, o1); break;
		case I_JGE:			EncodeJCC(0xD, o1); break;
		case I_JL:			EncodeJCC(0xC, o1); break;
		case I_JLE:			EncodeJCC(0xE, o1); break;
		case I_JNE:			EncodeJCC(0x5, o1); break;
		case I_JNO:			EncodeJCC(0x1, o1); break;
		case I_JNP:			EncodeJCC(0xB, o1); break;
		case I_JNS:			EncodeJCC(0x9, o1); break;
		case I_JO:			EncodeJCC(0x0, o1); break;
		case I_JP:			EncodeJCC(0xA, o1); break;
		case I_JS:			EncodeJCC(0x8, o1); break;
		case I_LEA:			EncodeLEA(o1, o2); break;
		case I_LEAVE:		db(0xC9); break;
		case I_MOV:			EncodeMOV(o1, o2); break;
		case I_MOVS_B:		db(0xA4); break;
		case I_MOVS_W:		db(0x66); db(0xA5); break;
		case I_MOVS_D:		db(0xA5); break;
		case I_MOVS_Q:		db(0x48); db(0xA5); break;
		case I_REP_MOVS_B:	EncodeRep(); db(0xA4); break;
		case I_REP_MOVS_W:	EncodeRep(); db(0x66); db(0xA5); break;
		case I_REP_MOVS_D:	EncodeRep(); db(0xA5); break;
		case I_REP_MOVS_Q:	EncodeRep(); db(0x48); db(0xA5); break;
		case I_MOVZX:		EncodeMOVZX(o1, o2); break;
		case I_MUL:			EncodeGrp3(4, o1); break;
		case I_NEG:			EncodeGrp3(3, o1); break;
		case I_NOP:			db(0x90); break;
		case I_NOT:			EncodeGrp3(2, o1); break;
		case I_OR:			EncodeALU(0x08, o1, o2); break;
		case I_POP:			EncodePUSH_POP(0x58, 0x8F, 0, o1); break;
		case I_PUSH:		EncodePUSH_POP(0x50, 0xFF, 6, o1); break;
		case I_RET:			if (o1.IsNone()) db(0xC3); else db(0xC2), dw(o1.GetImm()); break;
		case I_RCL:			EncodeShift(2, o1, o2); break;
		case I_RCR:			EncodeShift(3, o1, o2); break;
		case I_ROL:			EncodeShift(0, o1, o2); break;
		case I_ROR:			EncodeShift(1, o1, o2); break;
		case I_SAR:			EncodeShift(7, o1, o2); break;
		case I_SHL:			EncodeShift(4, o1, o2); break;
		case I_SHR:			EncodeShift(5, o1, o2); break;
		case I_SBB:			EncodeALU(0x18, o1, o2); break;
		case I_SUB:			EncodeALU(0x28, o1, o2); break;
		case I_TEST:		EncodeTEST(o1, o2); break;
		case I_XCHG:		EncodeXCHG(o1, o2); break;
		case I_XOR:			EncodeALU(0x30, o1, o2); break;

		// x87 Floating-Point Instructions
		case I_FLD:			EncodeFLD(o1); break;
		case I_FST:			EncodeFST(2, 0xD0, o1); break;
		case I_FSTP:		EncodeFST(3, 0xD8, o1); break;

		// MMX/SSE/SSE2 Instructions
		case I_ADDPD:		EncodeSSE2(0x66, 0x58, o1, o2); break;
		case I_ADDSD:		EncodeSSE2(0xF2, 0x58, o1, o2); break;
		case I_ANDPD:		EncodeSSE2(0x66, 0x54, o1, o2); break;
		case I_ANDNPD:		EncodeSSE2(0x66, 0x55, o1, o2); break;
		case I_CLFLUSH:		EncodeCLFLUSH(o1); break;
		case I_CMPPD:		EncodeSSE2(0x66, 0xC2, o1, o2); db(o3.GetImm()); break;
		case I_CMPPS:		EncodeSSE(0xC2, o1, o2); db(o3.GetImm()); break;
		case I_CMPSD:		EncodeSSE2(0xF2, 0xC2, o1, o2); db(o3.GetImm()); break;
		case I_COMISD:		EncodeSSE2(0x66, 0x2F, o1, o2); break;
		case I_CVTDQ2PD:	EncodeSSE2(0xF3, 0xE6, o1, o2); break;
		case I_CVTPD2DQ:	EncodeSSE2(0xF2, 0xE6, o1, o2); break;
		case I_CVTPD2PI:	EncodeSSE2(0x66, 0x2D, o1, o2); break;
		case I_CVTPD2PS:	EncodeSSE2(0x66, 0x5A, o1, o2); break;
		case I_CVTPI2PD:	EncodeSSE2(0x66, 0x2A, o1, o2); break;
		case I_CVTPS2DQ:	EncodeSSE2(0x66, 0x5B, o1, o2); break;
		case I_CVTDQ2PS:	EncodeSSE(0x5B, o1, o2); break;
		case I_CVTPS2PD:	EncodeSSE(0x5A, o1, o2); break;	// SSE2!!!
		//case I_CVTSD2SI:
		case I_CVTSD2SS:	EncodeSSE2(0xF2, 0x5A, o1, o2); break;
		//case I_CVTSI2SD:
		case I_CVTSS2SD:	EncodeSSE2(0xF3, 0x5A, o1, o2); break;
		case I_CVTTPD2DQ:	EncodeSSE2(0x66, 0xE6, o1, o2); break;
		case I_CVTTPD2PI:	EncodeSSE2(0x66, 0x2C, o1, o2); break;
		case I_CVTTPS2DQ:	EncodeSSE2(0xF3, 0x5B, o1, o2); break;
		//case I_CVTTSD2SI:
		case I_DIVPD:		EncodeSSE2(0x66, 0x5E, o1, o2); break;
		case I_DIVSD:		EncodeSSE2(0xF2, 0x5E, o1, o2); break;
		//case I_LFENCE:
		case I_MASKMOVDQU:	EncodeSSE2(0x66, 0xF7, o1, o2); break;
		case I_MAXPD:		EncodeSSE2(0x66, 0x5F, o1, o2); break;
		case I_MAXSD:		EncodeSSE2(0xF2, 0x5F, o1, o2); break;
		//case I_MFENCE:
		case I_MINPD:		EncodeSSE2(0x66, 0x5D, o1, o2); break;
		case I_MINSD:		EncodeSSE2(0xF2, 0x5D, o1, o2); break;
		case I_MOVAPD:		EncodeSSE2(0x66, 0x28 | (o1.IsReg() ? 0 : 0x01), o1, o2); break;
		//case I_MOVD:
		case I_MOVDQ2Q:		EncodeSSE2(0xF2, 0xD6, o1, o2); break;
		case I_MOVDQA:		EncodeSSE2(0x66, 0x6F | (o1.IsReg() ? 0 : 0x10), o1, o2); break;
		case I_MOVDQU:		EncodeSSE2(0xF3, 0x6F | (o1.IsReg() ? 0 : 0x10), o1, o2); break;
		case I_MOVHPD:		EncodeSSE2(0x66, 0x16 | (o1.IsReg() ? 0 : 0x01), o1, o2); break;
		case I_MOVLPD:		EncodeSSE2(0x66, 0x12 | (o1.IsReg() ? 0 : 0x01), o1, o2); break;
		case I_MOVMSKPD:	EncodeSSE2(0x66, 0x50, o1, o2); break;
		case I_MOVNTPD:		EncodeSSE2(0x66, 0x2B, o1, o2); break;
		case I_MOVNTDQ:		EncodeSSE2(0x66, 0xE7, o1, o2); break;
		//case I_MOVNTI:
		case I_MOVQ:		EncodeMOVQ(o1, o2); break;
		case I_MOVQ2DQ:		EncodeSSE2(0xF3, 0xD6, o1, o2); break;
		case I_MOVSD:		EncodeSSE2(0xF2, 0x10 | (o1.IsReg() ? 0 : 0x01), o1, o2); break;
		case I_MOVSS:		EncodeSSE2(0xF3, 0x10 | (o1.IsReg() ? 0 : 0x01), o1, o2); break;	// SSE
		case I_MOVUPD:		EncodeSSE2(0x66, 0x10 | (o1.IsReg() ? 0 : 0x01), o1, o2); break;
		case I_MULPD:		EncodeSSE2(0x66, 0x59, o1, o2); break;
		case I_MULSD:		EncodeSSE2(0xF2, 0x59, o1, o2); break;
		case I_ORPD:		EncodeSSE2(0x66, 0x56, o1, o2); break;
		case I_PABSB:		EncodeMMXorSSE2(0x66, 0x38, 0x1C, o1, o2); break;	// SSSE3
		case I_PABSW:		EncodeMMXorSSE2(0x66, 0x38, 0x1D, o1, o2); break;	// SSSE3
		case I_PABSD:		EncodeMMXorSSE2(0x66, 0x38, 0x1E, o1, o2); break;	// SSSE3
		case I_PACKSSWB:	EncodeMMXorSSE2(0x66, 0x63, o1, o2); break;
		case I_PACKSSDW:	EncodeMMXorSSE2(0x66, 0x6B, o1, o2); break;
		case I_PACKUSWB:	EncodeMMXorSSE2(0x66, 0x67, o1, o2); break;
		case I_PACKUSDW:	EncodeSSE2(0x66, 0x38, 0x2B, o1, o2); break;		// SSE 4.1
		case I_PADDB:		EncodeMMXorSSE2(0x66, 0xFC, o1, o2); break;
		case I_PADDW:		EncodeMMXorSSE2(0x66, 0xFD, o1, o2); break;
		case I_PADDD:		EncodeMMXorSSE2(0x66, 0xFE, o1, o2); break;
		case I_PADDQ:		EncodeMMXorSSE2(0x66, 0xD4, o1, o2); break;
		case I_PADDSB:		EncodeMMXorSSE2(0x66, 0xEC, o1, o2); break;
		case I_PADDSW:		EncodeMMXorSSE2(0x66, 0xED, o1, o2); break;
		case I_PADDUSB:		EncodeMMXorSSE2(0x66, 0xDC, o1, o2); break;
		case I_PADDUSW:		EncodeMMXorSSE2(0x66, 0xDD, o1, o2); break;
		case I_PALIGNR:		EncodeMMXorSSE2(0x66, 0x3A, 0x0F, o1, o2); break;	// SSSE3
		case I_PAND:		EncodeMMXorSSE2(0x66, 0xDB, o1, o2); break;
		case I_PANDN:		EncodeMMXorSSE2(0x66, 0xDF, o1, o2); break;
		case I_PAUSE:		break;
		case I_PAVGB:		EncodeMMXorSSE2(0x66, 0xE0, o1, o2); break;
		case I_PAVGW:		EncodeMMXorSSE2(0x66, 0xE3, o1, o2); break;
		case I_PCMPEQB:		EncodeMMXorSSE2(0x66, 0x74, o1, o2); break;
		case I_PCMPEQW:		EncodeMMXorSSE2(0x66, 0x75, o1, o2); break;
		case I_PCMPEQD:		EncodeMMXorSSE2(0x66, 0x76, o1, o2); break;
		case I_PCMPEQQ:		EncodeSSE2(0x66, 0x38, 0x29, o1, o2); break;
		case I_PCMPGTB:		EncodeMMXorSSE2(0x66, 0x64, o1, o2); break;
		case I_PCMPGTW:		EncodeMMXorSSE2(0x66, 0x65, o1, o2); break;
		case I_PCMPGTD:		EncodeMMXorSSE2(0x66, 0x66, o1, o2); break;
		case I_PCMPGTQ:		EncodeSSE2(0x66, 0x38, 0x37, o1, o2); break;
		//case I_PEXTRW:
		//case I_PINSRW:
		case I_PMADDWD:		EncodeMMXorSSE2(0x66, 0xF5, o1, o2); break;
		case I_PMAXSW:		EncodeMMXorSSE2(0x66, 0xEE, o1, o2); break;
		case I_PMAXUB:		EncodeMMXorSSE2(0x66, 0xDE, o1, o2); break;
		case I_PMINSW:		EncodeMMXorSSE2(0x66, 0xEA, o1, o2); break;
		case I_PMINUB:		EncodeMMXorSSE2(0x66, 0xDA, o1, o2); break;
		//case I_PMOVMSKB:
		case I_PMULHUW:		EncodeMMXorSSE2(0x66, 0xE4, o1, o2); break;
		case I_PMULHW:		EncodeMMXorSSE2(0x66, 0xE5, o1, o2); break;
		case I_PMULLW:		EncodeMMXorSSE2(0x66, 0xD5, o1, o2); break;
		case I_PMULUDQ:		EncodeMMXorSSE2(0x66, 0xF4, o1, o2); break;
		case I_POR:			EncodeMMXorSSE2(0x66, 0xEB, o1, o2); break;
		case I_PSADBW:		EncodeMMXorSSE2(0x66, 0xF6, o1, o2); break;
		//case I_PSHUFD:
		//case I_PSHUFHW:
		//case I_PSHUFLW:
		//case I_PSLLW:
		//case I_PSLLD:
		//case I_PSLLQ:
		//case I_PSLLDQ:
		//case I_PSRAW:
		//case I_PSRAD:
		//case I_PSRLW:
		//case I_PSRLD:
		//case I_PSRLQ:
		//case I_PSRLDQ:
		case I_PSUBB:		EncodeMMXorSSE2(0x66, 0xF8, o1, o2); break;
		case I_PSUBW:		EncodeMMXorSSE2(0x66, 0xF9, o1, o2); break;
		case I_PSUBD:		EncodeMMXorSSE2(0x66, 0xFA, o1, o2); break;
		case I_PSUBQ:		EncodeMMXorSSE2(0x66, 0xFB, o1, o2); break;
		case I_PSUBSB:		EncodeMMXorSSE2(0x66, 0xE8, o1, o2); break;
		case I_PSUBSW:		EncodeMMXorSSE2(0x66, 0xE9, o1, o2); break;
		case I_PSUBUSB:		EncodeMMXorSSE2(0x66, 0xD8, o1, o2); break;
		case I_PSUBUSW:		EncodeMMXorSSE2(0x66, 0xD9, o1, o2); break;
		case I_PUNPCKHBW:	EncodeMMXorSSE2(0x66, 0x68, o1, o2); break;
		case I_PUNPCKHWD:	EncodeMMXorSSE2(0x66, 0x69, o1, o2); break;
		case I_PUNPCKHDQ:	EncodeMMXorSSE2(0x66, 0x6A, o1, o2); break;
		case I_PUNPCKHQDQ:	EncodeSSE2(0x66, 0x6D, o1, o2); break;
		case I_PUNPCKLBW:	EncodeMMXorSSE2(0x66, 0x60, o1, o2); break;
		case I_PUNPCKLWD:	EncodeMMXorSSE2(0x66, 0x61, o1, o2); break;
		case I_PUNPCKLDQ:	EncodeMMXorSSE2(0x66, 0x62, o1, o2); break;
		case I_PUNPCKLQDQ:	EncodeSSE2(0x66, 0x6C, o1, o2); break;
		case I_PXOR:		EncodeMMXorSSE2(0x66, 0xEF, o1, o2); break;
		//case I_SHUFPD:
		case I_SQRTPD:		EncodeSSE2(0x66, 0x51, o1, o2); break;
		case I_SQRTSD:		EncodeSSE2(0xF2, 0x51, o1, o2); break;
		case I_SUBPD:		EncodeSSE2(0x66, 0x5C, o1, o2); break;
		case I_SUBSD:		EncodeSSE2(0xF2, 0x5C, o1, o2); break;
		case I_UCOMISD:		EncodeSSE2(0x66, 0x2E, o1, o2); break;
		case I_UNPCKHPD:	EncodeSSE2(0x66, 0x15, o1, o2); break;
		case I_UNPCKLPD:	EncodeSSE2(0x66, 0x14, o1, o2); break;
		case I_XORPD:		EncodeSSE2(0x66, 0x57, o1, o2); break;

		default:			ASSERT(0); break;
		}
	}

	static size_t GetInstrCodeSize(const Instr& instr)
	{
		Backend backend;
		backend.Assemble(instr);
		return backend.GetSize();
	}
};

struct VirtualMemory
{
	PVOID	pbuff_;
	size_t	buffsize_;

	VirtualMemory() : pbuff_(NULL), buffsize_(0)
	{
	}

	~VirtualMemory()
	{
		Free();
	}

	PVOID GetPointer() const
	{
		return pbuff_;
	}

	size_t GetSize() const
	{
		return buffsize_;
	}

	void Resize(size_t size)
	{
		Free();
		if (size > 0) {
			SYSTEM_INFO sysinfo;
			::GetSystemInfo(&sysinfo);
			size = (size + sysinfo.dwPageSize - 1) / sysinfo.dwPageSize * sysinfo.dwPageSize;

			pbuff_ = ::VirtualAlloc(NULL, size, MEM_COMMIT, PAGE_EXECUTE_READWRITE);
			if (pbuff_ == NULL) ASSERT(0);
			buffsize_ = size;
		}
	}

	void Free()
	{
		if (pbuff_) {
			::VirtualFree(pbuff_, 0, MEM_RELEASE);
			buffsize_ = 0;
		}
	}
};

struct Frontend
{
	Reg8 al, cl, dl, bl, ah, ch, dh, bh;
	Reg16 ax, cx, dx, bx, sp, bp, si, di;
	Reg32 eax, ecx, edx, ebx, esp, ebp, esi, edi;
	MmxReg mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7;
	XmmReg xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7;
#ifdef JITASM64
	Reg8 r8b, r9b, r10b, r11b, r12b, r13b, r14b, r15b;
	Reg16 r8w, r9w, r10w, r11w, r12w, r13w, r14w, r15w;
	Reg32 r8d, r9d, r10d, r11d, r12d, r13d, r14d, r15d;
	Rax rax;
	Reg64 rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15;
	XmmReg xmm8, xmm9, xmm10, xmm11, xmm12, xmm13, xmm14, xmm15;
#endif
	struct {FpuReg operator()(size_t n) {ASSERT(n >= ST0 && n <= ST7); return FpuReg((RegID) n);}} st;

	AddressingPtr<Opd8>		byte_ptr;
	AddressingPtr<Opd16>	word_ptr;
	AddressingPtr<Opd32>	dword_ptr;
	AddressingPtr<Opd64>	qword_ptr;
	AddressingPtr<Opd64>	mmword_ptr;
	AddressingPtr<Opd128>	xmmword_ptr;
	AddressingPtr<Opd32>	real4_ptr;
	AddressingPtr<Opd64>	real8_ptr;
	AddressingPtr<Opd80>	real10_ptr;

#ifdef JITASM64
	Rax zax;
	Reg64 zcx, zdx, zbx, zsp, zbp, zsi, zdi;
	AddressingPtr<Opd64>	zword_ptr;
#else
	Reg32 zax, zcx, zdx, zbx, zsp, zbp, zsi, zdi;
	AddressingPtr<Opd32>	zword_ptr;
#endif

	Frontend()
		: al(AL), cl(CL), dl(DL), bl(BL), ah(AH), ch(CH), dh(DH), bh(BH),
		ax(AX), cx(CX), dx(DX), bx(BX), sp(SP), bp(BP), si(SI), di(DI),
		eax(EAX), ecx(ECX), edx(EDX), ebx(EBX), esp(ESP), ebp(EBP), esi(ESI), edi(EDI),
		mm0(MM0), mm1(MM1), mm2(MM2), mm3(MM3), mm4(MM4), mm5(MM5), mm6(MM6), mm7(MM7),
		xmm0(XMM0), xmm1(XMM1), xmm2(XMM2), xmm3(XMM3), xmm4(XMM4), xmm5(XMM5), xmm6(XMM6), xmm7(XMM7),
#ifdef JITASM64
		r8b(R8B), r9b(R9B), r10b(R10B), r11b(R11B), r12b(R12B), r13b(R13B), r14b(R14B), r15b(R15B),
		r8w(R8W), r9w(R9W), r10w(R10W), r11w(R11W), r12w(R12W), r13w(R13W), r14w(R14W), r15w(R15W),
		r8d(R8D), r9d(R9D), r10d(R10D), r11d(R11D), r12d(R12D), r13d(R13D), r14d(R14D), r15d(R15D),
		rcx(RCX), rdx(RDX), rbx(RBX), rsp(RSP), rbp(RBP), rsi(RSI), rdi(RDI),
		r8(R8), r9(R9), r10(R10), r11(R11), r12(R12), r13(R13), r14(R14), r15(R15),
		xmm8(XMM8), xmm9(XMM9), xmm10(XMM10), xmm11(XMM11), xmm12(XMM12), xmm13(XMM13), xmm14(XMM14), xmm15(XMM15),
		zcx(RCX), zdx(RDX), zbx(RBX), zsp(RSP), zbp(RBP), zsi(RSI), zdi(RDI),
#else
		zax(EAX), zcx(ECX), zdx(EDX), zbx(EBX), zsp(ESP), zbp(EBP), zsi(ESI), zdi(EDI),
#endif
		buffsize_(0)
	{
	}

	typedef std::deque<Instr> InstrList;
	InstrList		instrs_;
	VirtualMemory	buff_;
	size_t			buffsize_;

	struct Label
	{
		std::string label_name;
		size_t	instr_number;
	};
	typedef std::deque<Label> LabelList;
	LabelList	labels_;

	virtual void naked_main() = 0;

	virtual void Prolog(size_t localVarSize)
	{
		push(zbp);
		mov(zbp, zsp);
		//sub(rsp, localVarSize)
		push(zbx);
		push(zdi);
		push(zsi);
#ifdef JITASM64
		push(r12);
		push(r13);
		push(r14);
		push(r15);
		sub(rsp, 160);
		movdqu(xmmword_ptr[rsp], xmm15);
		movdqu(xmmword_ptr[rsp + 16], xmm14);
		movdqu(xmmword_ptr[rsp + 32], xmm13);
		movdqu(xmmword_ptr[rsp + 48], xmm12);
		movdqu(xmmword_ptr[rsp + 64], xmm11);
		movdqu(xmmword_ptr[rsp + 80], xmm10);
		movdqu(xmmword_ptr[rsp + 96], xmm9);
		movdqu(xmmword_ptr[rsp + 112], xmm8);
		movdqu(xmmword_ptr[rsp + 128], xmm7);
		movdqu(xmmword_ptr[rsp + 144], xmm6);
#endif
	}

	virtual void Epilog()
	{
#ifdef JITASM64
		movdqu(xmm15, xmmword_ptr[rsp]);
		movdqu(xmm14, xmmword_ptr[rsp + 16]);
		movdqu(xmm13, xmmword_ptr[rsp + 32]);
		movdqu(xmm12, xmmword_ptr[rsp + 48]);
		movdqu(xmm11, xmmword_ptr[rsp + 64]);
		movdqu(xmm10, xmmword_ptr[rsp + 80]);
		movdqu(xmm9, xmmword_ptr[rsp + 96]);
		movdqu(xmm8, xmmword_ptr[rsp + 112]);
		movdqu(xmm7, xmmword_ptr[rsp + 128]);
		movdqu(xmm6, xmmword_ptr[rsp + 144]);
		add(rsp, 160);
		pop(r15);
		pop(r14);
		pop(r13);
		pop(r12);
#endif
		pop(zsi);
		pop(zdi);
		pop(zbx);
		leave();
		ret();
	}

	bool IsJmpOrJcc(InstrID id) const
	{
		return id == I_JMP || id == I_JA || id == I_JAE || id == I_JB
			|| id == I_JBE || id == I_JCXZ || id == I_JECXZ || id == I_JRCXZ
			|| id == I_JE || id == I_JG || id == I_JGE || id == I_JL
			|| id == I_JLE || id == I_JNE || id == I_JNO || id == I_JNP
			|| id == I_JNS || id == I_JO || id == I_JP || id == I_JS;
	}

	// TODO: Return an error when there is no destination.
	void ResolveJump()
	{
		// Replace label indexes with instruncion numbers.
		for (InstrList::iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			Instr& instr = *it;
			if (IsJmpOrJcc(instr.GetID()) && instr.GetOpd(0).IsImm()) {
				size_t label_id = (size_t) instr.GetOpd(0).GetImm();
				instr = Instr(instr.GetID(), Imm8(0x7F), Imm64(labels_[label_id].instr_number));	// opd1 = max value in sint8, opd2 = instruction number
			}
		}

		// Resolve operand sizes.
		std::vector<int> offsets;
		offsets.reserve(instrs_.size() + 1);
		bool retry;
		do {
			offsets.clear();
			offsets.push_back(0);
			Backend pre;
			for (InstrList::iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
				pre.Assemble(*it);
				offsets.push_back((int) pre.GetSize());
			}

			retry = false;
			for (size_t i = 0; i < instrs_.size(); i++) {
				Instr& instr = instrs_[i];
				if (IsJmpOrJcc(instr.GetID()) && instr.GetOpd(0).IsImm()) {
					size_t d = (size_t) instr.GetOpd(1).GetImm();
					int rel = offsets[d] - offsets[i] - (int) Backend::GetInstrCodeSize(instr);
					OpdSize size = instr.GetOpd(0).GetSize();
					if (size == O_SIZE_8) {
						if (!detail::IsInt8(rel)) {
							if (instr.GetID() == I_JRCXZ || instr.GetID() == I_JCXZ || instr.GetID() == I_JECXZ) ASSERT(0);	// jrcxz, jcxz, jecxz are only for short jump

							// Retry with immediate 32
							instr = Instr(instr.GetID(), Imm32(0x7FFFFFFF), Imm64(instr.GetOpd(1).GetImm()));
							retry = true;
						}
					} else if (size == O_SIZE_32) {
						ASSERT(detail::IsInt32(rel));	// There is no jump instruction larger than immediate 32.
					}
				}
			}
		} while (retry);

		// Resolve immediates
		for (size_t i = 0; i < instrs_.size(); i++) {
			Instr& instr = instrs_[i];
			if (IsJmpOrJcc(instr.GetID()) && instr.GetOpd(0).IsImm()) {
				size_t d = (size_t) instr.GetOpd(1).GetImm();
				int rel = offsets[d] - offsets[i] - (int) Backend::GetInstrCodeSize(instr);
				OpdSize size = instr.GetOpd(0).GetSize();
				if (size == O_SIZE_8) {
					ASSERT(detail::IsInt8(rel));
					instr = Instr(instr.GetID(), Imm8(rel));
				} else if (size == O_SIZE_32) {
					ASSERT(detail::IsInt32(rel));
					instr = Instr(instr.GetID(), Imm32(rel));
				}
			}
		}
	}

	void Assemble()
	{
		instrs_.clear();
		labels_.clear();
		naked_main();

		// Resolve jmp/jcc instructions
		if (!labels_.empty()) {
			ResolveJump();
		}

		// Count total size of machine code
		Backend pre;
		for (InstrList::const_iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			pre.Assemble(*it);
		}
		size_t buffsize = pre.GetSize();

		// Write machine code to the buffer
		buff_.Resize(buffsize);
		Backend backend(buff_.GetPointer(), buff_.GetSize());
		for (InstrList::const_iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			backend.Assemble(*it);
		}

		buffsize_ = buffsize;
	}

	/// Get assembled code
	void *GetCode()
	{
		if (!buff_.GetPointer()) {
			Assemble();
		}
		return buff_.GetPointer();
	}

	size_t GetCodeSize() const
	{
		return buffsize_;
	}

	void PushBack(const Instr& instr)
	{
		instrs_.push_back(instr);
	}

	size_t GetLabelId(const std::string& label_name)
	{
		for (size_t i = 0; i < labels_.size(); i++) {
			if (labels_[i].label_name.compare(label_name) == 0) {
				return i;
			}
		}
		Label label = {label_name};
		labels_.push_back(label);
		return labels_.size() - 1;
	}

	// LABEL
	void L(const std::string& label_name)
	{
		size_t label_id = GetLabelId(label_name);
		labels_[label_id].instr_number = instrs_.size();	// Label current instruction
	}

	// ALIGN
	void align(size_t n)
	{
		PushBack(Instr(I_PSEUDO_ALIGN, Imm64(n)));
	}

	// ADC
	void adc(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_ADC, dst, imm));}
	void adc(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_ADC, dst, imm));}
	void adc(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_ADC, dst, imm));}
	void adc(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_ADC, dst, imm));}
	void adc(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_ADC, dst, imm));}
	void adc(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_ADC, dst, imm));}
#ifdef JITASM64
	void adc(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_ADC, dst, src));}
	void adc(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_ADC, dst, imm));}
	void adc(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_ADC, dst, imm));}
#endif

	// ADD
	void add(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_ADD, dst, src));}
	void add(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_ADD, dst, imm));}
	void add(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_ADD, dst, imm));}
	void add(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_ADD, dst, imm));}
	void add(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_ADD, dst, imm));}
	void add(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_ADD, dst, imm));}
	void add(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_ADD, dst, imm));}
#ifdef JITASM64
	void add(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_ADD, dst, src));}
	void add(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_ADD, dst, imm));}
	void add(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_ADD, dst, imm));}
#endif

	// AND
	void and(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_AND, dst, src));}
	void and(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_AND, dst, imm));}
	void and(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_AND, dst, imm));}
	void and(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_AND, dst, imm));}
	void and(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_AND, dst, imm));}
	void and(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_AND, dst, imm));}
	void and(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_AND, dst, imm));}
#ifdef JITASM64
	void and(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_AND, dst, src));}
	void and(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_AND, dst, imm));}
	void and(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_AND, dst, imm));}
#endif

	// CALL
#ifndef JITASM64
	void call(const Reg16& dst)	{PushBack(Instr(I_CALL, dst));}
	void call(const Reg32& dst)	{PushBack(Instr(I_CALL, dst));}
#else
	void call(const Reg64& dst)	{PushBack(Instr(I_CALL, dst));}
#endif

	// CMP
	void cmp(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_CMP, dst, imm));}
	void cmp(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_CMP, dst, imm));}
	void cmp(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_CMP, dst, imm));}
	void cmp(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_CMP, dst, imm));}
	void cmp(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_CMP, dst, imm));}
	void cmp(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_CMP, dst, imm));}
#ifdef JITASM64
	void cmp(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_CMP, dst, src));}
	void cmp(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_CMP, dst, imm));}
	void cmp(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_CMP, dst, imm));}
#endif

	// DEC
	void dec(const Reg8& dst)	{PushBack(Instr(I_DEC, dst));}
	void dec(const Mem8& dst)	{PushBack(Instr(I_DEC, dst));}
	void dec(const Reg16& dst)	{PushBack(Instr(I_DEC, dst));}
	void dec(const Mem16& dst)	{PushBack(Instr(I_DEC, dst));}
	void dec(const Reg32& dst)	{PushBack(Instr(I_DEC, dst));}
	void dec(const Mem32& dst)	{PushBack(Instr(I_DEC, dst));}
#ifdef JITASM64
	void dec(const Reg64& dst)	{PushBack(Instr(I_DEC, dst));}
	void dec(const Mem64& dst)	{PushBack(Instr(I_DEC, dst));}
#endif

	// DIV
	void div(const Reg8& dst)	{PushBack(Instr(I_DIV, dst));}
	void div(const Mem8& dst)	{PushBack(Instr(I_DIV, dst));}
	void div(const Reg16& dst)	{PushBack(Instr(I_DIV, dst));}
	void div(const Mem16& dst)	{PushBack(Instr(I_DIV, dst));}
	void div(const Reg32& dst)	{PushBack(Instr(I_DIV, dst));}
	void div(const Mem32& dst)	{PushBack(Instr(I_DIV, dst));}
#ifdef JITASM64
	void div(const Reg64& dst)	{PushBack(Instr(I_DIV, dst));}
	void div(const Mem64& dst)	{PushBack(Instr(I_DIV, dst));}
#endif

	// IDIV
	void idiv(const Reg8& dst)	{PushBack(Instr(I_IDIV, dst));}
	void idiv(const Mem8& dst)	{PushBack(Instr(I_IDIV, dst));}
	void idiv(const Reg16& dst)	{PushBack(Instr(I_IDIV, dst));}
	void idiv(const Mem16& dst)	{PushBack(Instr(I_IDIV, dst));}
	void idiv(const Reg32& dst)	{PushBack(Instr(I_IDIV, dst));}
	void idiv(const Mem32& dst)	{PushBack(Instr(I_IDIV, dst));}
#ifdef JITASM64
	void idiv(const Reg64& dst)	{PushBack(Instr(I_IDIV, dst));}
	void idiv(const Mem64& dst)	{PushBack(Instr(I_IDIV, dst));}
#endif

	// IMUL
	void imul(const Reg8& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Mem8& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Reg16& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Mem16& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Reg32& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Mem32& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Reg16& dst, const Reg16& src)					{PushBack(Instr(I_IMUL, dst, src));}
	void imul(const Reg16& dst, const Mem16& src)					{PushBack(Instr(I_IMUL, dst, src));}
	void imul(const Reg32& dst, const Reg32& src)					{PushBack(Instr(I_IMUL, dst, src));}
	void imul(const Reg32& dst, const Mem32& src)					{PushBack(Instr(I_IMUL, dst, src));}
	void imul(const Reg16& dst, const Reg16& src, const Imm16& imm)	{PushBack(Instr(I_IMUL, dst, src, imm));}
	void imul(const Reg16& dst, const Mem16& src, const Imm16& imm)	{PushBack(Instr(I_IMUL, dst, src, imm));}
	void imul(const Reg32& dst, const Reg32& src, const Imm32& imm)	{PushBack(Instr(I_IMUL, dst, src, imm));}
	void imul(const Reg32& dst, const Mem32& src, const Imm32& imm)	{PushBack(Instr(I_IMUL, dst, src, imm));}
	void imul(const Reg16& dst, const Imm16& imm)					{imul(dst, dst, imm);}
	void imul(const Reg32& dst, const Imm32& imm)					{imul(dst, dst, imm);}
#ifdef JITASM64
	void imul(const Reg64& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Mem64& dst)										{PushBack(Instr(I_IMUL, dst));}
	void imul(const Reg64& dst, const Reg64& src)					{PushBack(Instr(I_IMUL, dst, src));}
	void imul(const Reg64& dst, const Mem64& src)					{PushBack(Instr(I_IMUL, dst, src));}
	void imul(const Reg64& dst, const Reg64& src, const Imm32& imm)	{PushBack(Instr(I_IMUL, dst, src, imm));}
	void imul(const Reg64& dst, const Mem64& src, const Imm32& imm)	{PushBack(Instr(I_IMUL, dst, src, imm));}
	void imul(const Reg64& dst, const Imm32& imm)					{imul(dst, dst, imm);}
#endif

	// INC
	void inc(const Reg8& dst)	{PushBack(Instr(I_INC, dst));}
	void inc(const Mem8& dst)	{PushBack(Instr(I_INC, dst));}
	void inc(const Reg16& dst)	{PushBack(Instr(I_INC, dst));}
	void inc(const Mem16& dst)	{PushBack(Instr(I_INC, dst));}
	void inc(const Reg32& dst)	{PushBack(Instr(I_INC, dst));}
	void inc(const Mem32& dst)	{PushBack(Instr(I_INC, dst));}
#ifdef JITASM64
	void inc(const Reg64& dst)	{PushBack(Instr(I_INC, dst));}
	void inc(const Mem64& dst)	{PushBack(Instr(I_INC, dst));}
#endif

	// INT3
	void debugbreak()	{PushBack(Instr(I_INT3));}

	// JMP
	void jmp(const std::string& label_name)		{PushBack(Instr(I_JMP, Imm64(GetLabelId(label_name))));}
	void ja(const std::string& label_name)		{PushBack(Instr(I_JA, Imm64(GetLabelId(label_name))));}
	void jae(const std::string& label_name)		{PushBack(Instr(I_JAE, Imm64(GetLabelId(label_name))));}
	void jb(const std::string& label_name)		{PushBack(Instr(I_JB, Imm64(GetLabelId(label_name))));}
	void jbe(const std::string& label_name)		{PushBack(Instr(I_JBE, Imm64(GetLabelId(label_name))));}
	void jc(const std::string& label_name)		{jb(label_name);}
	void jecxz(const std::string& label_name)	{PushBack(Instr(I_JECXZ, Imm64(GetLabelId(label_name))));}	// short jump only
#ifdef JITASM64
	void jrcxz (const std::string& label_name)	{PushBack(Instr(I_JRCXZ, Imm64(GetLabelId(label_name))));}	// short jump only
#else
	void jcxz(const std::string& label_name)	{PushBack(Instr(I_JCXZ, Imm64(GetLabelId(label_name))));}	// short jump only
#endif
	void je(const std::string& label_name)		{PushBack(Instr(I_JE, Imm64(GetLabelId(label_name))));}
	void jg(const std::string& label_name)		{PushBack(Instr(I_JG, Imm64(GetLabelId(label_name))));}
	void jge(const std::string& label_name)		{PushBack(Instr(I_JGE, Imm64(GetLabelId(label_name))));}
	void jl(const std::string& label_name)		{PushBack(Instr(I_JL, Imm64(GetLabelId(label_name))));}
	void jle(const std::string& label_name)		{PushBack(Instr(I_JLE, Imm64(GetLabelId(label_name))));}
	void jna(const std::string& label_name)		{jbe(label_name);}
	void jnae(const std::string& label_name)	{jb(label_name);}
	void jnb(const std::string& label_name)		{jae(label_name);}
	void jnbe(const std::string& label_name)	{ja(label_name);}
	void jnc(const std::string& label_name)		{jae(label_name);}
	void jne(const std::string& label_name)		{PushBack(Instr(I_JNE, Imm64(GetLabelId(label_name))));}
	void jng(const std::string& label_name)		{jle(label_name);}
	void jnge(const std::string& label_name)	{jl(label_name);}
	void jnl(const std::string& label_name)		{jge(label_name);}
	void jnle(const std::string& label_name)	{jg(label_name);}
	void jno(const std::string& label_name)		{PushBack(Instr(I_JNO, Imm64(GetLabelId(label_name))));}
	void jnp(const std::string& label_name)		{PushBack(Instr(I_JNP, Imm64(GetLabelId(label_name))));}
	void jns(const std::string& label_name)		{PushBack(Instr(I_JNS, Imm64(GetLabelId(label_name))));}
	void jnz(const std::string& label_name)		{jne(label_name);}
	void jo(const std::string& label_name)		{PushBack(Instr(I_JO, Imm64(GetLabelId(label_name))));}
	void jp(const std::string& label_name)		{PushBack(Instr(I_JP, Imm64(GetLabelId(label_name))));}
	void jpe(const std::string& label_name)		{jp(label_name);}
	void jpo(const std::string& label_name)		{jnp(label_name);}
	void js(const std::string& label_name)		{PushBack(Instr(I_JS, Imm64(GetLabelId(label_name))));}
	void jz(const std::string& label_name)		{je(label_name);}

	// LEA
	void lea(const Reg16& dst, const Mem8& src)		{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg16& dst, const Mem32& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg16& dst, const Mem64& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg16& dst, const Mem80& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg16& dst, const Mem128& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg32& dst, const Mem8& src)		{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg32& dst, const Mem16& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg32& dst, const Mem64& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg32& dst, const Mem80& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg32& dst, const Mem128& src)	{PushBack(Instr(I_LEA, dst, src));}

#ifdef JITASM64
	void lea(const Reg64& dst, const Mem8& src)		{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg64& dst, const Mem16& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg64& dst, const Mem32& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg64& dst, const Mem80& src)	{PushBack(Instr(I_LEA, dst, src));}
	void lea(const Reg64& dst, const Mem128& src)	{PushBack(Instr(I_LEA, dst, src));}
#endif

	// LEAVE
	void leave()	{PushBack(Instr(I_LEAVE));}

	// MOV
	void mov(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_MOV, dst, imm));}
#ifdef JITASM64
	void mov(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_MOV, dst, src));}
	void mov(const Reg64& dst, const Imm64& imm)	{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Mem64& dst, const Imm64& imm)	{PushBack(Instr(I_MOV, dst, imm));}
	void mov(const Rax& dst, Mem64Offset& src)		{PushBack(Instr(I_MOV, dst, src.ToMem64()));}
#endif
 
	// MOVSB/MOVSW/MOVSD/MOVSQ
	void movsb() {PushBack(Instr(I_MOVS_B));}
	void movsw() {PushBack(Instr(I_MOVS_W));}
	void movsd() {PushBack(Instr(I_MOVS_D));}
#ifdef JITASM64
	void movsq() {PushBack(Instr(I_MOVS_Q));}
#endif
	void rep_movsb() {PushBack(Instr(I_REP_MOVS_B));}
	void rep_movsw() {PushBack(Instr(I_REP_MOVS_W));}
	void rep_movsd() {PushBack(Instr(I_REP_MOVS_D));}
#ifdef JITASM64
	void rep_movsq() {PushBack(Instr(I_REP_MOVS_Q));}
#endif

	// MOVZX
	void movzx(const Reg16& dst, const Opd8& src)	{PushBack(Instr(I_MOVZX, dst, src));}
	void movzx(const Reg32& dst, const Opd8& src)	{PushBack(Instr(I_MOVZX, dst, src));}
	void movzx(const Reg32& dst, const Opd16& src)	{PushBack(Instr(I_MOVZX, dst, src));}
#ifdef JITASM64
	void movzx(const Reg64& dst, const Opd8& src)	{PushBack(Instr(I_MOVZX, dst, src));}
	void movzx(const Reg64& dst, const Opd16& src)	{PushBack(Instr(I_MOVZX, dst, src));}
#endif

	// MUL
	void mul(const Reg8& dst)	{PushBack(Instr(I_MUL, dst));}
	void mul(const Mem8& dst)	{PushBack(Instr(I_MUL, dst));}
	void mul(const Reg16& dst)	{PushBack(Instr(I_MUL, dst));}
	void mul(const Mem16& dst)	{PushBack(Instr(I_MUL, dst));}
	void mul(const Reg32& dst)	{PushBack(Instr(I_MUL, dst));}
	void mul(const Mem32& dst)	{PushBack(Instr(I_MUL, dst));}
#ifdef JITASM64
	void mul(const Reg64& dst)	{PushBack(Instr(I_MUL, dst));}
	void mul(const Mem64& dst)	{PushBack(Instr(I_MUL, dst));}
#endif

	// NEG
	void neg(const Reg8& dst)	{PushBack(Instr(I_NEG, dst));}
	void neg(const Mem8& dst)	{PushBack(Instr(I_NEG, dst));}
	void neg(const Reg16& dst)	{PushBack(Instr(I_NEG, dst));}
	void neg(const Mem16& dst)	{PushBack(Instr(I_NEG, dst));}
	void neg(const Reg32& dst)	{PushBack(Instr(I_NEG, dst));}
	void neg(const Mem32& dst)	{PushBack(Instr(I_NEG, dst));}
#ifdef JITASM64
	void neg(const Reg64& dst)	{PushBack(Instr(I_NEG, dst));}
	void neg(const Mem64& dst)	{PushBack(Instr(I_NEG, dst));}
#endif

	// NOP
	void nop()	{PushBack(Instr(I_NOP));}

	// NOT
	void not(const Reg8& dst)	{PushBack(Instr(I_NOT, dst));}
	void not(const Mem8& dst)	{PushBack(Instr(I_NOT, dst));}
	void not(const Reg16& dst)	{PushBack(Instr(I_NOT, dst));}
	void not(const Mem16& dst)	{PushBack(Instr(I_NOT, dst));}
	void not(const Reg32& dst)	{PushBack(Instr(I_NOT, dst));}
	void not(const Mem32& dst)	{PushBack(Instr(I_NOT, dst));}
#ifdef JITASM64
	void not(const Reg64& dst)	{PushBack(Instr(I_NOT, dst));}
	void not(const Mem64& dst)	{PushBack(Instr(I_NOT, dst));}
#endif

	// OR
	void or(const Reg8& dst, const Reg8& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg8& dst, const Mem8& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Mem8& dst, const Reg8& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_OR, dst, imm));}
	void or(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_OR, dst, imm));}
	void or(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_OR, dst, imm));}
	void or(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_OR, dst, imm));}
	void or(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_OR, dst, imm));}
	void or(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_OR, dst, imm));}
#ifdef JITASM64
	void or(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_OR, dst, src));}
	void or(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_OR, dst, imm));}
	void or(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_OR, dst, imm));}
#endif

	// POP
	void pop(const Reg16& dst)	{PushBack(Instr(I_POP, dst));}
	void pop(const Mem16& dst)	{PushBack(Instr(I_POP, dst));}
#ifdef JITASM64
	void pop(const Reg64& dst)	{PushBack(Instr(I_POP, dst));}
	void pop(const Mem64& dst)	{PushBack(Instr(I_POP, dst));}
#else
	void pop(const Reg32& dst)	{PushBack(Instr(I_POP, dst));}
	void pop(const Mem32& dst)	{PushBack(Instr(I_POP, dst));}
#endif

	// PUSH
	void push(const Reg16& dst)	{PushBack(Instr(I_PUSH, dst));}
	void push(const Mem16& dst)	{PushBack(Instr(I_PUSH, dst));}
#ifdef JITASM64
	void push(const Reg64& dst)	{PushBack(Instr(I_PUSH, dst));}
	void push(const Mem64& dst)	{PushBack(Instr(I_PUSH, dst));}
#else
	void push(const Reg32& dst)	{PushBack(Instr(I_PUSH, dst));}
	void push(const Mem32& dst)	{PushBack(Instr(I_PUSH, dst));}
#endif
	void push(const Imm32& imm)	{PushBack(Instr(I_PUSH, imm));}

	// RET
	void ret()					{PushBack(Instr(I_RET));}
	void ret(const Imm16& imm)	{PushBack(Instr(I_RET, imm));}
 
	// RCL/RCR/ROL/ROR
	void rcl(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcl(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcr(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rcr(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rol(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void rol(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void ror(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
	void ror(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
	void rcl(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcl(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcr(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rcr(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rol(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void rol(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void ror(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
	void ror(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
	void rcl(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcl(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcr(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rcr(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rol(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void rol(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void ror(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
	void ror(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
#ifdef JITASM64
	void rcl(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcl(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_RCL, dst, imm));}
	void rcr(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rcr(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_RCR, dst, imm));}
	void rol(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void rol(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_ROL, dst, imm));}
	void ror(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
	void ror(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_ROR, dst, imm));}
#endif

	// SAL/SAR/SHL/SHR
	void sal(const Reg8& dst, const Imm8& imm)	{shl(dst, imm);}
	void sal(const Mem8& dst, const Imm8& imm)	{shl(dst, imm);}
	void sar(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void sar(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void shl(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shl(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shr(const Reg8& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
	void shr(const Mem8& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
	void sal(const Reg16& dst, const Imm8& imm)	{shl(dst, imm);}
	void sal(const Mem16& dst, const Imm8& imm)	{shl(dst, imm);}
	void sar(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void sar(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void shl(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shl(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shr(const Reg16& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
	void shr(const Mem16& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
	void sal(const Reg32& dst, const Imm8& imm)	{shl(dst, imm);}
	void sal(const Mem32& dst, const Imm8& imm)	{shl(dst, imm);}
	void sar(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void sar(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void shl(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shl(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shr(const Reg32& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
	void shr(const Mem32& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
#ifdef JITASM64
	void sal(const Reg64& dst, const Imm8& imm)	{shl(dst, imm);}
	void sal(const Mem64& dst, const Imm8& imm)	{shl(dst, imm);}
	void sar(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void sar(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_SAR, dst, imm));}
	void shl(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shl(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_SHL, dst, imm));}
	void shr(const Reg64& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
	void shr(const Mem64& dst, const Imm8& imm)	{PushBack(Instr(I_SHR, dst, imm));}
#endif

	// SBB
	void sbb(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_SBB, dst, imm));}
	void sbb(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_SBB, dst, imm));}
	void sbb(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_SBB, dst, imm));}
	void sbb(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_SBB, dst, imm));}
	void sbb(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_SBB, dst, imm));}
	void sbb(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_SBB, dst, imm));}
#ifdef JITASM64
	void sbb(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_SBB, dst, src));}
	void sbb(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_SBB, dst, imm));}
	void sbb(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_SBB, dst, imm));}
#endif

	// SUB
	void sub(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_SUB, dst, imm));}
	void sub(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_SUB, dst, imm));}
	void sub(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_SUB, dst, imm));}
	void sub(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_SUB, dst, imm));}
	void sub(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_SUB, dst, imm));}
	void sub(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_SUB, dst, imm));}
#ifdef JITASM64
	void sub(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_SUB, dst, src));}
	void sub(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_SUB, dst, imm));}
	void sub(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_SUB, dst, imm));}
#endif

	// TEST
	void test(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_TEST, dst, src));}
	void test(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_TEST, dst, src));}
	void test(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_TEST, dst, src));}
	void test(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_TEST, dst, src));}
	void test(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_TEST, dst, src));}
	void test(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_TEST, dst, src));}
#ifdef JITASM64
	void test(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_TEST, dst, imm));}
	void test(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_TEST, dst, src));}
	void test(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_TEST, dst, src));}
#endif

	// XCHG
	void xchg(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_XCHG, dst, src));}
#ifdef JITASM64
	void xchg(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_XCHG, dst, src));}
	void xchg(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_XCHG, dst, src));}
#endif

	// XOR
	void xor(const Reg8& dst, const Reg8& src)		{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg8& dst, const Mem8& src)		{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Mem8& dst, const Reg8& src)		{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg8& dst, const Imm8& imm)		{PushBack(Instr(I_XOR, dst, imm));}
	void xor(const Mem8& dst, const Imm8& imm)		{PushBack(Instr(I_XOR, dst, imm));}
	void xor(const Reg16& dst, const Reg16& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg16& dst, const Mem16& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Mem16& dst, const Reg16& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg16& dst, const Imm16& imm)	{PushBack(Instr(I_XOR, dst, imm));}
	void xor(const Mem16& dst, const Imm16& imm)	{PushBack(Instr(I_XOR, dst, imm));}
	void xor(const Reg32& dst, const Reg32& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg32& dst, const Mem32& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Mem32& dst, const Reg32& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg32& dst, const Imm32& imm)	{PushBack(Instr(I_XOR, dst, imm));}
	void xor(const Mem32& dst, const Imm32& imm)	{PushBack(Instr(I_XOR, dst, imm));}
#ifdef JITASM64
	void xor(const Reg64& dst, const Reg64& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg64& dst, const Mem64& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Mem64& dst, const Reg64& src)	{PushBack(Instr(I_XOR, dst, src));}
	void xor(const Reg64& dst, const Imm32& imm)	{PushBack(Instr(I_XOR, dst, imm));}
	void xor(const Mem64& dst, const Imm32& imm)	{PushBack(Instr(I_XOR, dst, imm));}
#endif

	// FLD
	void fld(const Mem32& src)	{PushBack(Instr(I_FLD, src));}
	void fld(const Mem64& src)	{PushBack(Instr(I_FLD, src));}
	void fld(const Mem80& src)	{PushBack(Instr(I_FLD, src));}
	void fld(const FpuReg& src)	{PushBack(Instr(I_FLD, src));}

	// FST/FSTP
	void fst(const Mem32& dst) {PushBack(Instr(I_FST, dst));}
	void fst(const Mem64& dst) {PushBack(Instr(I_FST, dst));}
	void fst(const FpuReg& dst) {PushBack(Instr(I_FST, dst));}
	void fstp(const Mem32& dst) {PushBack(Instr(I_FSTP, dst));}
	void fstp(const Mem64& dst) {PushBack(Instr(I_FSTP, dst));}
	void fstp(const Mem80& dst) {PushBack(Instr(I_FSTP, dst));}
	void fstp(const FpuReg& dst) {PushBack(Instr(I_FSTP, dst));}

	void addpd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_ADDPD, dst, src));}
	void addpd(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_ADDPD, dst, src));}
	void addsd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_ADDSD, dst, src));}
	void addsd(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_ADDSD, dst, src));}

	void andpd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_ANDPD, dst, src));}
	void andpd(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_ANDPD, dst, src));}
	void andnpd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_ANDPD, dst, src));}
	void andnpd(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_ANDPD, dst, src));}

	void clflush(const Mem8& dst) {PushBack(Instr(I_CLFLUSH, dst));}

	void cmppd(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{PushBack(Instr(I_CMPPD, dst, src, opd3));}
	void cmppd(const XmmReg& dst, const Mem128& src, const Imm8& opd3)	{PushBack(Instr(I_CMPPD, dst, src, opd3));}
	void cmpeqpd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 0);}
	void cmpeqpd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 0);}
	void cmpltpd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 1);}
	void cmpltpd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 1);}
	void cmplepd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 2);}
	void cmplepd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 2);}
	void cmpunordpd(const XmmReg& dst, const XmmReg& src)	{cmppd(dst, src, 3);}
	void cmpunordpd(const XmmReg& dst, const Mem128& src)	{cmppd(dst, src, 3);}
	void cmpneqpd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 4);}
	void cmpneqpd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 4);}
	void cmpnltpd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 5);}
	void cmpnltpd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 5);}
	void cmpnlepd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 6);}
	void cmpnlepd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 6);}
	void cmpordpd(const XmmReg& dst, const XmmReg& src)		{cmppd(dst, src, 7);}
	void cmpordpd(const XmmReg& dst, const Mem128& src)		{cmppd(dst, src, 7);}

	void cmpps(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{PushBack(Instr(I_CMPPS, dst, src, opd3));}
	void cmpps(const XmmReg& dst, const Mem128& src, const Imm8& opd3)	{PushBack(Instr(I_CMPPS, dst, src, opd3));}
	void cmpeqps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 0);}
	void cmpeqps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 0);}
	void cmpltps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 1);}
	void cmpltps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 1);}
	void cmpleps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 2);}
	void cmpleps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 2);}
	void cmpunordps(const XmmReg& dst, const XmmReg& src)	{cmpps(dst, src, 3);}
	void cmpunordps(const XmmReg& dst, const Mem128& src)	{cmpps(dst, src, 3);}
	void cmpneqps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 4);}
	void cmpneqps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 4);}
	void cmpnltps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 5);}
	void cmpnltps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 5);}
	void cmpnleps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 6);}
	void cmpnleps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 6);}
	void cmpordps(const XmmReg& dst, const XmmReg& src)		{cmpps(dst, src, 7);}
	void cmpordps(const XmmReg& dst, const Mem128& src)		{cmpps(dst, src, 7);}

	void cmpsd(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{PushBack(Instr(I_CMPSD, dst, src, opd3));}
	void cmpsd(const XmmReg& dst, const Mem64& src, const Imm8& opd3)	{PushBack(Instr(I_CMPSD, dst, src, opd3));}
	void cmpeqsd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 0);}
	void cmpeqsd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 0);}
	void cmpltsd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 1);}
	void cmpltsd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 1);}
	void cmplesd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 2);}
	void cmplesd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 2);}
	void cmpunordsd(const XmmReg& dst, const XmmReg& src)	{cmpsd(dst, src, 3);}
	void cmpunordsd(const XmmReg& dst, const Mem64& src)	{cmpsd(dst, src, 3);}
	void cmpneqsd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 4);}
	void cmpneqsd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 4);}
	void cmpnltsd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 5);}
	void cmpnltsd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 5);}
	void cmpnlesd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 6);}
	void cmpnlesd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 6);}
	void cmpordsd(const XmmReg& dst, const XmmReg& src)		{cmpsd(dst, src, 7);}
	void cmpordsd(const XmmReg& dst, const Mem64& src)		{cmpsd(dst, src, 7);}

	void comisd(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_COMISD, dst, src));}
	void comisd(const XmmReg& dst, const Mem64& src)		{PushBack(Instr(I_COMISD, dst, src));}

	void cvtdq2pd(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTDQ2PD, dst, src));}
	void cvtdq2pd(const XmmReg& dst, const Mem64& src)		{PushBack(Instr(I_CVTDQ2PD, dst, src));}
	void cvtpd2dq(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTPD2DQ, dst, src));}
	void cvtpd2dq(const XmmReg& dst, const Mem128& src)		{PushBack(Instr(I_CVTPD2DQ, dst, src));}
	void cvtpd2pi(const MmxReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTPD2PI, dst, src));}
	void cvtpd2pi(const MmxReg& dst, const Mem128& src)		{PushBack(Instr(I_CVTPD2PI, dst, src));}
	void cvtpd2ps(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTPD2PS, dst, src));}
	void cvtpd2ps(const XmmReg& dst, const Mem128& src)		{PushBack(Instr(I_CVTPD2PS, dst, src));}
	void cvtpi2pd(const XmmReg& dst, const MmxReg& src)		{PushBack(Instr(I_CVTPI2PD, dst, src));}
	void cvtpi2pd(const XmmReg& dst, const Mem64& src)		{PushBack(Instr(I_CVTPI2PD, dst, src));}
	void cvtps2dq(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTPS2DQ, dst, src));}
	void cvtps2dq(const XmmReg& dst, const Mem128& src)		{PushBack(Instr(I_CVTPS2DQ, dst, src));}
	void cvtdq2ps(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTDQ2PS, dst, src));}
	void cvtdq2ps(const XmmReg& dst, const Mem128& src)		{PushBack(Instr(I_CVTDQ2PS, dst, src));}
	void cvtps2pd(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTPS2PD, dst, src));}
	void cvtps2pd(const XmmReg& dst, const Mem64& src)		{PushBack(Instr(I_CVTPS2PD, dst, src));}
	void cvtsd2ss(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTSD2SS, dst, src));}
	void cvtsd2ss(const XmmReg& dst, const Mem64& src)		{PushBack(Instr(I_CVTSD2SS, dst, src));}
	void cvtss2sd(const XmmReg& dst, const XmmReg& src)		{PushBack(Instr(I_CVTSS2SD, dst, src));}
	void cvtss2sd(const XmmReg& dst, const Mem32& src)		{PushBack(Instr(I_CVTSS2SD, dst, src));}
	void cvttpd2dq(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_CVTTPD2DQ, dst, src));}
	void cvttpd2dq(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_CVTTPD2DQ, dst, src));}
	void cvttpd2pi(const MmxReg& dst, const XmmReg& src)	{PushBack(Instr(I_CVTTPD2PI, dst, src));}
	void cvttpd2pi(const MmxReg& dst, const Mem128& src)	{PushBack(Instr(I_CVTTPD2PI, dst, src));}
	void cvttps2dq(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_CVTTPS2DQ, dst, src));}
	void cvttps2dq(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_CVTTPS2DQ, dst, src));}

	// MOVDQA
	void movdqa(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_MOVDQA, dst, src));}
	void movdqa(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_MOVDQA, dst, src));}
	void movdqa(const Mem128& dst, const XmmReg& src)	{PushBack(Instr(I_MOVDQA, dst, src));}

	// MOVDQU
	void movdqu(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_MOVDQU, dst, src));}
	void movdqu(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_MOVDQU, dst, src));}
	void movdqu(const Mem128& dst, const XmmReg& src)	{PushBack(Instr(I_MOVDQU, dst, src));}

	// MOVQ
	void movq(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_MOVQ, dst, src));}
	void movq(const MmxReg& dst, const Mem64& src)	{PushBack(Instr(I_MOVQ, dst, src));}
	void movq(const Mem64& dst, const MmxReg& src)	{PushBack(Instr(I_MOVQ, dst, src));}
	void movq(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_MOVQ, dst, src));}
	void movq(const XmmReg& dst, const Mem64& src)	{PushBack(Instr(I_MOVQ, dst, src));}
	void movq(const Mem64& dst, const XmmReg& src)	{PushBack(Instr(I_MOVQ, dst, src));}

	// MOVSD
	void movsd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_MOVSD, dst, src));}
	void movsd(const XmmReg& dst, const Mem64& src)		{PushBack(Instr(I_MOVSD, dst, src));}
	void movsd(const Mem64& dst, const XmmReg& src)		{PushBack(Instr(I_MOVSD, dst, src));}

	// MOVSS
	void movss(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_MOVSS, dst, src));}
	void movss(const XmmReg& dst, const Mem32& src)		{PushBack(Instr(I_MOVSS, dst, src));}
	void movss(const Mem32& dst, const XmmReg& src)		{PushBack(Instr(I_MOVSS, dst, src));}

	// PABSB/PABSW/PABSD
	void pabsb(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PABSB, dst, src));}
	void pabsb(const MmxReg& dst, const Mem64& src)		{PushBack(Instr(I_PABSB, dst, src));}
	void pabsb(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PABSB, dst, src));}
	void pabsb(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PABSB, dst, src));}
	void pabsw(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PABSW, dst, src));}
	void pabsw(const MmxReg& dst, const Mem64& src)		{PushBack(Instr(I_PABSW, dst, src));}
	void pabsw(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PABSW, dst, src));}
	void pabsw(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PABSW, dst, src));}
	void pabsd(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PABSD, dst, src));}
	void pabsd(const MmxReg& dst, const Mem64& src)		{PushBack(Instr(I_PABSD, dst, src));}
	void pabsd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PABSD, dst, src));}
	void pabsd(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PABSD, dst, src));}

	// PACKSSWB/PACKSSDW/PACKUSWB/PACKUSDW
	void packsswb(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PACKSSWB, dst, src));}
	void packsswb(const MmxReg& dst, const Mem64& src)	{PushBack(Instr(I_PACKSSWB, dst, src));}
	void packsswb(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PACKSSWB, dst, src));}
	void packsswb(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PACKSSWB, dst, src));}
	void packssdw(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PACKSSDW, dst, src));}
	void packssdw(const MmxReg& dst, const Mem64& src)	{PushBack(Instr(I_PACKSSDW, dst, src));}
	void packssdw(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PACKSSDW, dst, src));}
	void packssdw(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PACKSSDW, dst, src));}
	void packuswb(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PACKUSWB, dst, src));}
	void packuswb(const MmxReg& dst, const Mem64& src)	{PushBack(Instr(I_PACKUSWB, dst, src));}
	void packuswb(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PACKUSWB, dst, src));}
	void packuswb(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PACKUSWB, dst, src));}
	void packusdw(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PACKUSDW, dst, src));}
	void packusdw(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PACKUSDW, dst, src));}

	// PADDB/PADDW/PADDD
	void paddb(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PADDB, dst, src));}
	void paddb(const MmxReg& dst, const Mem64& src)		{PushBack(Instr(I_PADDB, dst, src));}
	void paddb(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PADDB, dst, src));}
	void paddb(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PADDB, dst, src));}
	void paddw(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PADDW, dst, src));}
	void paddw(const MmxReg& dst, const Mem64& src)		{PushBack(Instr(I_PADDW, dst, src));}
	void paddw(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PADDW, dst, src));}
	void paddw(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PADDW, dst, src));}
	void paddd(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PADDD, dst, src));}
	void paddd(const MmxReg& dst, const Mem64& src)		{PushBack(Instr(I_PADDD, dst, src));}
	void paddd(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PADDD, dst, src));}
	void paddd(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PADDD, dst, src));}

	// PXOR
	void pxor(const MmxReg& dst, const MmxReg& src)	{PushBack(Instr(I_PXOR, dst, src));}
	void pxor(const MmxReg& dst, const Mem64& src)	{PushBack(Instr(I_PXOR, dst, src));}
	void pxor(const XmmReg& dst, const XmmReg& src)	{PushBack(Instr(I_PXOR, dst, src));}
	void pxor(const XmmReg& dst, const Mem128& src)	{PushBack(Instr(I_PXOR, dst, src));}
};

namespace detail {

	// Flags for ArgumentTraits
	enum {
		ARG_IN_REG		= (1<<0),	///< Argument is stored in general purpose register.
		ARG_IN_STACK	= (1<<1),	///< Argument is stored on stack.
		ARG_IN_XMM		= (1<<2),	///< Argument is stored in xmm register.
		ARG_TYPE_VALUE	= (1<<3),	///< Argument is value which is passed.
		ARG_TYPE_PTR	= (1<<4)	///< Argument is pointer which is passed to.
	};

	/// cdecl argument type traits
	template<int N, class T, int Size = sizeof(T)>
	struct ArgumentTraits_cdecl {
		enum {
			size = (Size + 4 - 1) / 4 * 4,
			flag = ARG_IN_STACK | ARG_TYPE_VALUE,
			reg_id = INVALID
		};
	};

	/// Microsoft x64 fastcall argument type traits
	template<int N, class T, int Size = sizeof(T)>
	struct ArgumentTraits_win64 {
		enum {
			size = 8,
			flag = ARG_IN_STACK | (Size == 1 || Size == 2 || Size == 4 || Size == 8 ? ARG_TYPE_VALUE : ARG_TYPE_PTR),
			reg_id = INVALID
		};
	};

	/**
	 * Base class for argument which is stored in general purpose register.
	 */
	template<int GpRegID, int Flag> struct ArgumentTraits_win64_reg {
		enum {
			size = 8,
			flag = ARG_IN_REG | Flag,
			reg_id = GpRegID
		};
	};

	// specialization for argument pointer stored in register
	template<class T, int Size> struct ArgumentTraits_win64<0, T, Size> : ArgumentTraits_win64_reg<RCX, ARG_TYPE_PTR> {};
	template<class T, int Size> struct ArgumentTraits_win64<1, T, Size> : ArgumentTraits_win64_reg<RDX, ARG_TYPE_PTR> {};
	template<class T, int Size> struct ArgumentTraits_win64<2, T, Size> : ArgumentTraits_win64_reg<R8, ARG_TYPE_PTR> {};
	template<class T, int Size> struct ArgumentTraits_win64<3, T, Size> : ArgumentTraits_win64_reg<R9, ARG_TYPE_PTR> {};

	// specialization for 1 byte type
	template<class T> struct ArgumentTraits_win64<0, T, 1> : ArgumentTraits_win64_reg<RCX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<1, T, 1> : ArgumentTraits_win64_reg<RDX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<2, T, 1> : ArgumentTraits_win64_reg<R8, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<3, T, 1> : ArgumentTraits_win64_reg<R9, ARG_TYPE_VALUE> {};

	// specialization for 2 bytes type
	template<class T> struct ArgumentTraits_win64<0, T, 2> : ArgumentTraits_win64_reg<RCX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<1, T, 2> : ArgumentTraits_win64_reg<RDX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<2, T, 2> : ArgumentTraits_win64_reg<R8, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<3, T, 2> : ArgumentTraits_win64_reg<R9, ARG_TYPE_VALUE> {};

	// specialization for 4 bytes type
	template<class T> struct ArgumentTraits_win64<0, T, 4> : ArgumentTraits_win64_reg<RCX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<1, T, 4> : ArgumentTraits_win64_reg<RDX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<2, T, 4> : ArgumentTraits_win64_reg<R8, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<3, T, 4> : ArgumentTraits_win64_reg<R9, ARG_TYPE_VALUE> {};

	// specialization for 8 bytes type
	template<class T> struct ArgumentTraits_win64<0, T, 8> : ArgumentTraits_win64_reg<RCX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<1, T, 8> : ArgumentTraits_win64_reg<RDX, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<2, T, 8> : ArgumentTraits_win64_reg<R8, ARG_TYPE_VALUE> {};
	template<class T> struct ArgumentTraits_win64<3, T, 8> : ArgumentTraits_win64_reg<R9, ARG_TYPE_VALUE> {};

#ifdef __INTRIN_H_
	// specialization for __m64
	template<> struct ArgumentTraits_win64<0, __m64, 8> : ArgumentTraits_win64_reg<RCX> {};
	template<> struct ArgumentTraits_win64<1, __m64, 8> : ArgumentTraits_win64_reg<RDX> {};
	template<> struct ArgumentTraits_win64<2, __m64, 8> : ArgumentTraits_win64_reg<R8> {};
	template<> struct ArgumentTraits_win64<3, __m64, 8> : ArgumentTraits_win64_reg<R9> {};
#endif

	/**
	 * Base class for argument which is stored in xmm register.
	 */
	template<int XmmRegID> struct ArgumentTraits_win64_xmm {
		enum {
			size = 8,
			flag = ARG_IN_XMM | ARG_TYPE_VALUE,
			reg_id = XmmRegID
		};
	};

	// specialization for float
	template<> struct ArgumentTraits_win64<0, float, 4> : ArgumentTraits_win64_xmm<XMM0> {};
	template<> struct ArgumentTraits_win64<1, float, 4> : ArgumentTraits_win64_xmm<XMM1> {};
	template<> struct ArgumentTraits_win64<2, float, 4> : ArgumentTraits_win64_xmm<XMM2> {};
	template<> struct ArgumentTraits_win64<3, float, 4> : ArgumentTraits_win64_xmm<XMM3> {};

	// specialization for double
	template<> struct ArgumentTraits_win64<0, double, 8> : ArgumentTraits_win64_xmm<XMM0> {};
	template<> struct ArgumentTraits_win64<1, double, 8> : ArgumentTraits_win64_xmm<XMM1> {};
	template<> struct ArgumentTraits_win64<2, double, 8> : ArgumentTraits_win64_xmm<XMM2> {};
	template<> struct ArgumentTraits_win64<3, double, 8> : ArgumentTraits_win64_xmm<XMM3> {};


	/// Result type traits
	template<class T>
	struct ResultTraits {
		enum { size = sizeof(T) };
		typedef OpdT<sizeof(T) * 8>	OpdR;
		typedef AddressingPtr<OpdR>	ResultPtr;
	};

	// specialization for void
	template<>
	struct ResultTraits<void> {
		enum { size = 0 };
		struct OpdR {};
		struct ResultPtr {};
	};

	/// Function result
	template<class T, int Size = ResultTraits<T>::size>
	struct ResultT {
		enum { ArgR = 1 /* First (hidden) argument is pointer for copying result. */};
		typedef typename ResultTraits<T>::OpdR OpdR;
		OpdR val_;
		ResultT() : val_(INVALID) {}
		ResultT(const MemT<OpdR>& val) : val_(val) {}
		void Store(Frontend& f)
		{
			if (val_.IsMem()) {
				f.mov(f.zcx, Size);
				//f.lea(f.zsi, static_cast<MemT<OpdR>&>(val_));
#ifdef JITASM64
				f.lea(f.zsi, static_cast<Mem64&>(static_cast<Opd&>(val_)));
#else
				f.lea(f.zsi, static_cast<Mem32&>(static_cast<Opd&>(val_)));
#endif
				f.mov(f.zdi, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
				f.mov(f.zax, f.zdi);
				f.rep_movsb();
			}
		}
	};

	// specialization for void
	template<>
	struct ResultT<void, 0> {
		enum { ArgR = 0 };
		ResultT();
	};

	// specialization for 1byte type
	template<class T>
	struct ResultT<T, 1> {
		enum { ArgR = 0 };
		Opd8 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd8& val) : val_(val) {}
		ResultT(uint8 imm) : val_(Imm8(imm)) {}
		void Store(Frontend& f)
		{
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == AL)))
				f.mov(f.al, static_cast<Reg8&>(val_));
		}
	};

	// specialization for 2bytes type
	template<class T>
	struct ResultT<T, 2> {
		enum { ArgR = 0 };
		Opd16 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd16& val) : val_(val) {}
		ResultT(uint16 imm) : val_(Imm16(imm)) {}
		void Store(Frontend& f)
		{
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == AX)))
				f.mov(f.ax, static_cast<Reg16&>(val_));
		}
	};

	// specialization for 4bytes type
	template<class T>
	struct ResultT<T, 4> {
		enum { ArgR = 0 };
		Opd32 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd32& val) : val_(val) {}
		ResultT(uint32 imm) : val_(Imm32(imm)) {}
		void Store(Frontend& f)
		{
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == EAX)))
				f.mov(f.eax, static_cast<Reg32&>(val_));
		}
	};

	// specialization for 8bytes type
	template<class T>
	struct ResultT<T, 8> {
		enum { ArgR = 0 };
		Opd64 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd64& val) : val_(val) {}
		ResultT(uint64 imm) : val_(Imm64(imm)) {}
		void Store(Frontend& f)
		{
#ifdef JITASM64
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == RAX)))
				f.mov(f.rax, static_cast<Reg64&>(val_));
#else
			if (val_.IsMem()) {
				// from memory
				Mem32 lo(val_.GetAddressSize(), val_.GetBase(), val_.GetIndex(), val_.GetScale(), val_.GetDisp());
				Mem32 hi(val_.GetAddressSize(), val_.GetBase(), val_.GetIndex(), val_.GetScale(), val_.GetDisp() + 4);
				f.mov(f.eax, lo);
				f.mov(f.edx, hi);
			}
			else if (val_.IsImm()) {
				// from immediate
				f.mov(f.eax, static_cast<sint32>(val_.GetImm()));
				f.mov(f.edx, static_cast<sint32>(val_.GetImm() >> 32));
			}
#endif
		}
	};

	// specialization for float
	template<>
	struct ResultT<float, 4> {
		enum { ArgR = 0 };
		Opd val_;
		ResultT() {}
		ResultT(const FpuReg& fpu) : val_(fpu) {}
		ResultT(const Mem32& mem) : val_(mem) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const float imm) : val_(Imm32(*(uint32*)&imm)) {}
		void Store(Frontend& f)
		{
#ifdef JITASM64
			if (val_.IsReg() && val_.GetSize() == O_SIZE_80) {
				// from FPU register
				//f.fstp(f.real4_ptr[f.esp - 4]);
				f.movss(f.xmm0, f.dword_ptr[f.esp - 4]);
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_32) {
				// from memory
				f.movss(f.xmm0, static_cast<Mem32&>(val_));
			}
			else if (val_.IsReg() && val_.GetSize() == O_SIZE_128) {
				// from XMM register
				if (val_.GetReg() != XMM0)
					f.movss(f.xmm0, static_cast<XmmReg&>(val_));
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.esp - 4], static_cast<Reg32&>(val_));
				f.movss(f.xmm0, f.dword_ptr[f.esp - 4]);
			}
#else
			if (val_.IsReg() && val_.GetSize() == O_SIZE_80) {
				// from FPU register
				if (val_.GetReg() != ST0)
					f.fld(static_cast<FpuReg&>(val_));
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_32) {
				// from memory
				f.fld(static_cast<Mem32&>(val_));
			}
			else if (val_.IsReg() && val_.GetSize() == O_SIZE_128) {
				// from XMM register
				f.movss(f.dword_ptr[f.esp - 4], static_cast<XmmReg&>(val_));
				f.fld(f.real4_ptr[f.esp - 4]);
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.esp - 4], static_cast<Reg32&>(val_));
				f.fld(f.real4_ptr[f.esp - 4]);
			}
#endif
		}
	};

	// specialization for double
	template<>
	struct ResultT<double, 8> {
		enum { ArgR = 0 };
		Opd val_;
		double imm_;
		ResultT() {}
		ResultT(const FpuReg& fpu) : val_(fpu) {}
		ResultT(const Mem64& mem) : val_(mem) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const double imm) : val_(Imm32(0)), imm_(imm) {}
		void Store(Frontend& f)
		{
#ifdef JITASM64
			if (val_.IsReg() && val_.GetSize() == O_SIZE_80) {
				// from FPU register
				//f.fstp(f.real8_ptr[f.esp - 8]);
				f.movsd(f.xmm0, f.qword_ptr[f.esp - 8]);
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_64) {
				// from memory
				f.movsd(f.xmm0, static_cast<Mem64&>(val_));
			}
			else if (val_.IsReg() && val_.GetSize() == O_SIZE_128) {
				// from XMM register
				if (val_.GetReg() != XMM0)
					f.movsd(f.xmm0, static_cast<XmmReg&>(val_));
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.esp - 8], *reinterpret_cast<uint32*>(&imm_));
				f.mov(f.dword_ptr[f.esp - 4], *(reinterpret_cast<uint32*>(&imm_) + 1));
				f.movsd(f.xmm0, f.qword_ptr[f.esp - 8]);
			}
#else
			if (val_.IsReg() && val_.GetSize() == O_SIZE_80) {
				// from FPU register
				if (val_.GetReg() != ST0)
					f.fld(static_cast<FpuReg&>(val_));
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_64) {
				// from memory
				f.fld(static_cast<Mem64&>(val_));
			}
			else if (val_.IsReg() && val_.GetSize() == O_SIZE_128) {
				// from XMM register
				f.movsd(f.qword_ptr[f.esp - 8], static_cast<XmmReg&>(val_));
				f.fld(f.real8_ptr[f.esp - 8]);
			}
			else if (val_.IsImm()) {	// val_ is immediate 0
				// from double immediate
				f.mov(f.dword_ptr[f.esp - 8], *reinterpret_cast<uint32*>(&imm_));
				f.mov(f.dword_ptr[f.esp - 4], *(reinterpret_cast<uint32*>(&imm_) + 1));
				f.fld(f.real8_ptr[f.esp - 8]);
			}
#endif
		}
	};


	/// cdecl function base class
	class Function_cdecl : public Frontend
	{
	public:
#ifdef JITASM64
		typedef Reg64Expr Arg;		///< main function argument type
		template<int N, class T> struct ArgTraits : ArgumentTraits_win64<N, T> {};
		bool dump_regarg_x64_;
#else
		typedef Reg32Expr Arg;		///< main function argument type
		template<int N, class T> struct ArgTraits : ArgumentTraits_cdecl<N, T> {};
#endif

		/**
		 * \param dump_regarg_x64	On x64, the caller passes the first four arguments in register.
									If this parameter is true, the arguments in register are copied to
									their stack space in prolog. This parameter has no effect on x86.
		 */
		Function_cdecl(bool dump_regarg_x64 = true)
#ifdef JITASM64
			: dump_regarg_x64_(dump_regarg_x64) {}
#else
		{}
#endif

	private:
		template<int N, class T>
		void CopyRegArgToStack(const Arg& addr)
		{
#ifdef JITASM64
			if (dump_regarg_x64_) {
				if (ArgTraits<N, T>::flag & ARG_IN_REG)
					mov(qword_ptr[addr], Reg64(static_cast<RegID>(ArgTraits<N, T>::reg_id)));
				else if (ArgTraits<N, T>::flag & ARG_IN_XMM)
					movq(qword_ptr[addr], XmmReg(static_cast<RegID>(ArgTraits<N, T>::reg_id)));
			}
#endif
		}

	public:
		template<class R>
		Arg DumpRegArg0()
		{
			Arg addr(zbp + sizeof(void *) * 2);
			if (ResultT<R>::ArgR) {
				CopyRegArgToStack<0, R>(addr);
				addr = addr + ArgTraits<0, R>::size;
			}
			return addr;
		}

		template<class R, class A1>
		Arg DumpRegArg1()
		{
			Arg addr = DumpRegArg0<R>();
			CopyRegArgToStack<0 + ResultT<R>::ArgR, A1>(addr);
			return addr + ArgTraits<0 + ResultT<R>::ArgR, A1>::size;
		}

		template<class R, class A1, class A2>
		Arg DumpRegArg2()
		{
			Arg addr = DumpRegArg1<R, A1>();
			CopyRegArgToStack<1 + ResultT<R>::ArgR, A2>(addr);
			return addr + ArgTraits<1 + ResultT<R>::ArgR, A2>::size;
		}

		template<class R, class A1, class A2, class A3>
		Arg DumpRegArg3()
		{
			Arg addr = DumpRegArg2<R, A1, A2>();
			CopyRegArgToStack<2 + ResultT<R>::ArgR, A3>(addr);
			return addr + ArgTraits<2 + ResultT<R>::ArgR, A3>::size;
		}

		template<class R, class A1, class A2, class A3, class A4>
		Arg DumpRegArg4()
		{
			Arg addr = DumpRegArg3<R, A1, A2, A3>();
			CopyRegArgToStack<3 + ResultT<R>::ArgR, A4>(addr);
			return addr + ArgTraits<3 + ResultT<R>::ArgR, A4>::size;
		}

		template<class R>
		Arg Arg1() { return Arg(zbp + sizeof(void *) * (2 + ResultT<R>::ArgR)); }
		template<class R, class A1>
		Arg Arg2() { return Arg1<R>() + ArgTraits<0, A1>::size; }
		template<class R, class A1, class A2>
		Arg Arg3() { return Arg2<R, A1>() + ArgTraits<1, A2>::size; }
		template<class R, class A1, class A2, class A3>
		Arg Arg4() { return Arg3<R, A1, A2>() + ArgTraits<2, A3>::size; }
		template<class R, class A1, class A2, class A3, class A4>
		Arg Arg5() { return Arg4<R, A1, A2, A3>() + ArgTraits<3, A4>::size; }
		template<class R, class A1, class A2, class A3, class A4, class A5>
		Arg Arg6() { return Arg5<R, A1, A2, A3, A4>() + ArgTraits<4, A5>::size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6>
		Arg Arg7() { return Arg6<R, A1, A2, A3, A4, A5>() + ArgTraits<5, A6>::size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7>
		Arg Arg8() { return Arg7<R, A1, A2, A3, A4, A5, A6>() + ArgTraits<6, A7>::size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8>
		Arg Arg9() { return Arg8<R, A1, A2, A3, A4, A5, A6, A7>() + ArgTraits<7, A8>::size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9>
		Arg Arg10() { return Arg9<R, A1, A2, A3, A4, A5, A6, A7, A8>() + ArgTraits<8, A9>::size; }
	};

}	// namespace detail

/// cdecl function which has no argument
template<class R>
struct function0_cdecl : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)();
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function0_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main() { return Result(); }
	void naked_main() {
		Prolog(0);
		DumpRegArg0<R>();
		main().Store(*this);
		Epilog();
	}
};

template<>
struct function0_cdecl<void> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)();
	function0_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main() {}
	void naked_main()	{
		Prolog(0);
		main();
		Epilog();
	}
};

/// cdecl function which has 1 argument
template<class R, class A1>
struct function1_cdecl : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function1_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg a1) { return Result(); }
	void naked_main() {
		Prolog(0);
		DumpRegArg1<R, A1>();
		main(Arg1<R>()).Store(*this);
		Epilog();
	}
};

template<class A1>
struct function1_cdecl<void, A1> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1);
	function1_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg a1) {}
	void naked_main() {
		Prolog(0);
		DumpRegArg1<void, A1>();
		main(Arg1<void>());
		Epilog();
	}
};

/// cdecl function which has 2 arguments
template<class R, class A1, class A2>
struct function2_cdecl : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function2_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg a1, Arg a2) { return Result(); }
	void naked_main() {
		Prolog(0);
		DumpRegArg2<R, A1, A2>();
		main(Arg1<R>(), Arg2<R, A1>()).Store(*this);
		Epilog();
	}
};

template<class A1, class A2>
struct function2_cdecl<void, A1, A2> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2);
	function2_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg a1, Arg a2) {}
	void naked_main() {
		Prolog(0);
		DumpRegArg2<void, A1, A2>();
		main(Arg1<void>(), Arg2<void, A1>());
		Epilog();
	}
};

template<class R> struct function0 : function0_cdecl<R> {};
template<class R, class A1> struct function1 : function1_cdecl<R, A1> {};
template<class R, class A1, class A2> struct function2 : function2_cdecl<R, A1, A2> {};

}	// namespace jitasm
#endif	// #ifndef JITASM_H
