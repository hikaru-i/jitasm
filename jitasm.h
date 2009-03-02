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
#include <deque>
#include <vector>
#include <assert.h>

#ifndef ASSERT
#define ASSERT assert
#endif

//#if defined(_WIN64) && (defined(_M_AMD64) || defined(_M_X64))
#define JITASM64
//#endif

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

//----------------------------------------
// Operand
//----------------------------------------
enum OpdType
{
	TYPE_NONE,
	TYPE_REG,
	TYPE_MEM,
	TYPE_IMM,
};

enum OpdSize
{
	SIZE_INT8,
	SIZE_INT16,
	SIZE_INT32,
	SIZE_INT64,
	SIZE_INT80,
	SIZE_INT128,
};

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

	// NONE
	Opd() : opdtype_(TYPE_NONE) {}
	// REG
	explicit Opd(OpdSize opdsize, RegID reg) : opdtype_(TYPE_REG), opdsize_(opdsize), reg_(reg) {}
	// MEM
	Opd(OpdSize opdsize, OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
		: opdtype_(TYPE_MEM), opdsize_(opdsize), addrsize_(addrsize), base_(base), index_(index), scale_(scale), disp_(disp) {}
protected:
	// IMM
	explicit Opd(sint64 imm) : opdtype_(TYPE_IMM), imm_(imm)
	{
		if (detail::IsInt8(imm)) opdsize_ = SIZE_INT8;
		else if (detail::IsInt16(imm)) opdsize_ = SIZE_INT16;
		else if (detail::IsInt32(imm)) opdsize_ = SIZE_INT32;
#ifdef JITASM64
		else opdsize_ = SIZE_INT64;
#else
		else ASSERT(0);
#endif
	}

public:
	bool	IsNone() const {return opdtype_ == TYPE_NONE;}
	bool	IsReg() const {return opdtype_ == TYPE_REG;}
	bool	IsMem() const {return opdtype_ == TYPE_MEM;}
	bool	IsImm() const {return opdtype_ == TYPE_IMM;}
	OpdSize	GetSize() const {return opdsize_;}
	OpdSize	GetAddressSize() const {return addrsize_;}

	RegID	GetReg() const {return reg_;}
	RegID	GetBase() const {return base_;}
	RegID	GetIndex() const {return index_;}
	sint64	GetScale() const {return scale_;}
	sint64	GetDisp() const {return disp_;}
	sint64	GetImm() const {return imm_;}
};

template<OpdSize Size>
struct OpdT : Opd
{
	// REG
	explicit OpdT(RegID reg) : Opd(Size, reg) {}
	// MEM
	OpdT(OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
		: Opd(Size, addrsize, base, index, scale, disp) {}
protected:
	// IMM
	OpdT(sint64 imm) : Opd(imm) {}
};
typedef OpdT<SIZE_INT8>		Opd8;
typedef OpdT<SIZE_INT16>	Opd16;
typedef OpdT<SIZE_INT32>	Opd32;
typedef OpdT<SIZE_INT64>	Opd64;
typedef OpdT<SIZE_INT80>	Opd80;
typedef OpdT<SIZE_INT128>	Opd128;

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
typedef RegT<Opd64>		Mmx;
typedef RegT<Opd128>	Xmm;

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

struct MemOffset64
{
	sint64 offset_;

	explicit MemOffset64(sint64 offset) : offset_(offset) {}
	Mem64 ToMem64() {return Mem64(SIZE_INT64, INVALID, INVALID, 0, offset_);}
};

template<class OpdN, class U, class S>
struct ImmT : OpdN
{
	ImmT(U imm) : OpdN((S) imm) {}
};
typedef ImmT<Opd8, uint8, sint8>		Imm8;
typedef ImmT<Opd16, uint16, sint16>		Imm16;
typedef ImmT<Opd32, uint32, sint32>		Imm32;
typedef ImmT<Opd64, uint64, sint64>		Imm64;

struct Expr32_None
{
	RegID reg_;
	sint64 disp_;
	Expr32_None(const Reg32& obj) : reg_(obj.reg_), disp_(0) {}	// implicit
	Expr32_None(RegID reg, sint64 disp) : reg_(reg), disp_(disp) {}
};
inline Expr32_None operator+(const Reg32& lhs, sint64 rhs) {return Expr32_None(lhs.reg_, rhs);}
inline Expr32_None operator+(sint64 lhs, const Reg32& rhs) {return rhs + lhs;}
inline Expr32_None operator+(const Expr32_None& lhs, sint64 rhs) {return Expr32_None(lhs.reg_, lhs.disp_ + rhs);}
inline Expr32_None operator+(sint64 lhs, const Expr32_None& rhs) {return rhs + lhs;}

struct Expr32_BI
{
	RegID base_;
	RegID index_;
	sint64 disp_;
	Expr32_BI(RegID base, RegID index, sint64 disp) : base_(base), index_(index), disp_(disp) {}
};
inline Expr32_BI operator+(const Expr32_None& lhs, const Expr32_None& rhs) {return Expr32_BI(rhs.reg_, lhs.reg_, lhs.disp_ + rhs.disp_);}
inline Expr32_BI operator+(const Expr32_BI& lhs, sint64 rhs) {return Expr32_BI(lhs.base_, lhs.index_, lhs.disp_ + rhs);}
inline Expr32_BI operator+(sint64 lhs, const Expr32_BI& rhs) {return rhs + lhs;}

struct Expr32_SI
{
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Expr32_SI(RegID index, sint64 scale, sint64 disp) : index_(index), scale_(scale), disp_(disp) {}
};
inline Expr32_SI operator*(const Reg32& lhs, sint64 rhs) {return Expr32_SI(lhs.reg_, rhs, 0);}
inline Expr32_SI operator*(sint64 lhs, const Reg32& rhs) {return rhs * lhs;}
inline Expr32_SI operator*(const Expr32_SI& lhs, sint64 rhs) {return Expr32_SI(lhs.index_, lhs.scale_ * rhs, lhs.disp_);}
inline Expr32_SI operator*(sint64 lhs, const Expr32_SI& rhs) {return rhs * lhs;}
inline Expr32_SI operator+(const Expr32_SI& lhs, sint64 rhs) {return Expr32_SI(lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Expr32_SI operator+(sint64 lhs, const Expr32_SI& rhs) {return rhs + lhs;}

struct Expr32_SIB
{
	RegID base_;
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Expr32_SIB(RegID base, RegID index, sint64 scale, sint64 disp) : base_(base), index_(index), scale_(scale), disp_(disp) {}
};
inline Expr32_SIB operator+(const Expr32_None& lhs, const Expr32_SI& rhs) {return Expr32_SIB(lhs.reg_, rhs.index_, rhs.scale_, lhs.disp_ + rhs.disp_);}
inline Expr32_SIB operator+(const Expr32_SI& lhs, const Expr32_None& rhs) {return rhs + lhs;}
inline Expr32_SIB operator+(const Expr32_SIB& lhs, sint64 rhs) {return Expr32_SIB(lhs.base_, lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Expr32_SIB operator+(sint64 lhs, const Expr32_SIB& rhs) {return rhs + lhs;}

#ifdef JITASM64
struct Expr64_None
{
	RegID reg_;
	sint64 disp_;
	Expr64_None(const Reg64& obj) : reg_(obj.reg_), disp_(0) {}	// implicit
	Expr64_None(RegID reg, sint64 disp) : reg_(reg), disp_(disp) {}
};
inline Expr64_None operator+(const Reg64& lhs, sint64 rhs) {return Expr64_None(lhs.reg_, rhs);}
inline Expr64_None operator+(sint64 lhs, const Reg64& rhs) {return rhs + lhs;}
inline Expr64_None operator+(const Expr64_None& lhs, sint64 rhs) {return Expr64_None(lhs.reg_, lhs.disp_ + rhs);}
inline Expr64_None operator+(sint64 lhs, const Expr64_None& rhs) {return rhs + lhs;}

struct Expr64_BI
{
	RegID base_;
	RegID index_;
	sint64 disp_;
	Expr64_BI(RegID base, RegID index, sint64 disp) : base_(base), index_(index), disp_(disp) {}
};
inline Expr64_BI operator+(const Expr64_None& lhs, const Expr64_None& rhs) {return Expr64_BI(rhs.reg_, lhs.reg_, lhs.disp_ + rhs.disp_);}
inline Expr64_BI operator+(const Expr64_BI& lhs, sint64 rhs) {return Expr64_BI(lhs.base_, lhs.index_, lhs.disp_ + rhs);}
inline Expr64_BI operator+(sint64 lhs, const Expr64_BI& rhs) {return rhs + lhs;}

struct Expr64_SI
{
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Expr64_SI(RegID index, sint64 scale, sint64 disp) : index_(index), scale_(scale), disp_(disp) {}
};
inline Expr64_SI operator*(const Reg64& lhs, sint64 rhs) {return Expr64_SI(lhs.reg_, rhs, 0);}
inline Expr64_SI operator*(sint64 lhs, const Reg64& rhs) {return rhs * lhs;}
inline Expr64_SI operator*(const Expr64_SI& lhs, sint64 rhs) {return Expr64_SI(lhs.index_, lhs.scale_ * rhs, lhs.disp_);}
inline Expr64_SI operator*(sint64 lhs, const Expr64_SI& rhs) {return rhs * lhs;}
inline Expr64_SI operator+(const Expr64_SI& lhs, sint64 rhs) {return Expr64_SI(lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Expr64_SI operator+(sint64 lhs, const Expr64_SI& rhs) {return rhs + lhs;}

struct Expr64_SIB
{
	RegID base_;
	RegID index_;
	sint64 scale_;
	sint64 disp_;
	Expr64_SIB(RegID base, RegID index, sint64 scale, sint64 disp) : base_(base), index_(index), scale_(scale), disp_(disp) {}
};
inline Expr64_SIB operator+(const Expr64_None& lhs, const Expr64_SI& rhs) {return Expr64_SIB(lhs.reg_, rhs.index_, rhs.scale_, lhs.disp_ + rhs.disp_);}
inline Expr64_SIB operator+(const Expr64_SI& lhs, const Expr64_None& rhs) {return rhs + lhs;}
inline Expr64_SIB operator+(const Expr64_SIB& lhs, sint64 rhs) {return Expr64_SIB(lhs.base_, lhs.index_, lhs.scale_, lhs.disp_ + rhs);}
inline Expr64_SIB operator+(sint64 lhs, const Expr64_SIB& rhs) {return rhs + lhs;}
#endif

template<typename OpdN>
struct AddressingPtr
{
	// 32bit-Addressing
	MemT<OpdN> operator[](const Expr32_None& obj) {return MemT<OpdN>(SIZE_INT32, obj.reg_, INVALID, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr32_BI& obj) {return MemT<OpdN>(SIZE_INT32, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr32_SI& obj) {return MemT<OpdN>(SIZE_INT32, INVALID, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Expr32_SIB& obj) {return MemT<OpdN>(SIZE_INT32, obj.base_, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](sint32 disp) {return MemT<OpdN>(SIZE_INT64, INVALID, INVALID, 0, disp);}
	MemT<OpdN> operator[](uint32 disp) {return MemT<OpdN>(SIZE_INT64, INVALID, INVALID, 0, (sint32) disp);}
#ifdef JITASM64
	// 64bit-Addressing
	MemT<OpdN> operator[](const Expr64_None& obj) {return MemT<OpdN>(SIZE_INT64, obj.reg_, INVALID, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr64_BI& obj) {return MemT<OpdN>(SIZE_INT64, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr64_SI& obj) {return MemT<OpdN>(SIZE_INT64, INVALID, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Expr64_SIB& obj) {return MemT<OpdN>(SIZE_INT64, obj.base_, obj.index_, obj.scale_, obj.disp_);}
	MemOffset64 operator[](sint64 offset) {return MemOffset64(offset);}
	MemOffset64 operator[](uint64 offset) {return MemOffset64((sint64) offset);}
#endif
};

//----------------------------------------
// Instruction
//----------------------------------------
enum InstrID
{
	INSTR_ADC, INSTR_ADD, INSTR_AND, INSTR_CALL, INSTR_CMP, INSTR_DEC, INSTR_INC,
	INSTR_JMP, INSTR_JA, INSTR_JAE, INSTR_JB, INSTR_JBE, INSTR_JCXZ, INSTR_JECXZ, INSTR_JRCXZ, INSTR_JE,
	INSTR_JG, INSTR_JGE, INSTR_JL, INSTR_JLE, INSTR_JNE, INSTR_JNO, INSTR_JNP, INSTR_JNS, INSTR_JO, INSTR_JP, INSTR_JS,
	INSTR_LEA, INSTR_LEAVE, INSTR_MOV, INSTR_MOVZX, INSTR_NOP, INSTR_OR, INSTR_POP, INSTR_PUSH,
	INSTR_RET, INSTR_SAR, INSTR_SHL, INSTR_SHR, INSTR_SBB, INSTR_SUB, INSTR_TEST, INSTR_XCHG, INSTR_XOR,

	INSTR_FLD,

	INSTR_ADDPD, INSTR_ADDSD, INSTR_ANDPD, INSTR_ANDNPD, INSTR_CLFLUSH, INSTR_CMPPS, INSTR_CMPPD, INSTR_CMPSD, INSTR_COMISD, INSTR_CVTDQ2PD, INSTR_CVTDQ2PS,
	INSTR_CVTPD2DQ, INSTR_CVTPD2PI, INSTR_CVTPD2PS, INSTR_CVTPI2PD, INSTR_CVTPS2DQ, INSTR_CVTPS2PD, INSTR_CVTSD2SI, INSTR_CVTSD2SS,
	INSTR_CVTSI2SD, INSTR_CVTSS2SD, INSTR_CVTTPD2DQ, INSTR_CVTTPD2PI, INSTR_CVTTPS2DQ, INSTR_CVTTSD2SI, INSTR_DIVPD, INSTR_DIVSD, INSTR_LFENCE,
	INSTR_MASKMOVDQU, INSTR_MAXPD, INSTR_MAXSD, INSTR_MFENCE, INSTR_MINPD, INSTR_MINSD, INSTR_MOVAPD, INSTR_MOVD, INSTR_MOVDQ2Q, INSTR_MOVDQA,
	INSTR_MOVDQU, INSTR_MOVHPD, INSTR_MOVLPD, INSTR_MOVMSKPD, INSTR_MOVNTPD, INSTR_MOVNTDQ, INSTR_MOVNTI, INSTR_MOVQ, INSTR_MOVQ2DQ, INSTR_MOVSD, INSTR_MOVUPD, INSTR_MULPD,
	INSTR_MULSD, INSTR_ORPD, INSTR_PABSB, INSTR_PABSD, INSTR_PABSW, INSTR_PACKSSDW, INSTR_PACKSSWB, INSTR_PACKUSDW, INSTR_PACKUSWB,
	INSTR_PADDB, INSTR_PADDD, INSTR_PADDQ, INSTR_PADDSB, INSTR_PADDSW, INSTR_PADDUSB, INSTR_PADDUSW, INSTR_PADDW, INSTR_PALIGNR,
	INSTR_PAND, INSTR_PANDN, INSTR_PAUSE, INSTR_PAVGB, INSTR_PAVGW, INSTR_PCMPEQB, INSTR_PCMPEQW, INSTR_PCMPEQD, INSTR_PCMPEQQ,
	INSTR_PCMPGTB, INSTR_PCMPGTW, INSTR_PCMPGTD, INSTR_PCMPGTQ, INSTR_PEXTRW, INSTR_PINSRW, INSTR_PMADDWD,
	INSTR_PMAXSW, INSTR_PMAXUB, INSTR_PMINSW, INSTR_PMINUB, INSTR_PMOVMSKB, INSTR_PMULHUW, INSTR_PMULHW, INSTR_PMULLW, INSTR_PMULUDQ,
	INSTR_POR, INSTR_PSADBW, INSTR_PSHUFD, INSTR_PSHUFHW, INSTR_PSHUFLW, INSTR_PSLLW, INSTR_PSLLD, INSTR_PSLLQ, INSTR_PSLLDQ, INSTR_PSRAW,
	INSTR_PSRAD, INSTR_PSRLW, INSTR_PSRLD, INSTR_PSRLQ, INSTR_PSRLDQ, INSTR_PSUBB, INSTR_PSUBW, INSTR_PSUBD, INSTR_PSUBQ, INSTR_PSUBSB, INSTR_PSUBSW,
	INSTR_PSUBUSB, INSTR_PSUBUSW, INSTR_PUNPCKHBW, INSTR_PUNPCKHWD, INSTR_PUNPCKHDQ, INSTR_PUNPCKHQDQ, INSTR_PUNPCKLBW, INSTR_PUNPCKLWD, INSTR_PUNPCKLDQ, INSTR_PUNPCKLQDQ,
	INSTR_PXOR, INSTR_SHUFPD, INSTR_SQRTPD, INSTR_SQRTSD, INSTR_SUBPD, INSTR_SUBSD, INSTR_UCOMISD, INSTR_UNPCKHPD, INSTR_UNPCKLPD, INSTR_XORPD,

	INSTR_PSEUDO_ALIGN,
};

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
			if (reg.GetSize() == SIZE_INT64) wrxb |= 8;
			if (reg.GetReg() >= R8) wrxb |= 4;
		}
		if (r_m.IsReg()) {
			if (r_m.GetSize() == SIZE_INT64) wrxb |= 8;
			if (r_m.GetReg() >= R8) wrxb |= 1;
		}
		if (r_m.IsMem()) {
			if (r_m.GetSize() == SIZE_INT64) wrxb |= 8;
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

	void EncodeADD(uint8 opcode, const Opd& opd1, const Opd& opd2)
	{
		if (opd2.IsImm()) {
			const Opd& r_m = opd1;
			const Opd& imm = opd2;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexWRXB(r_m);
#else
			if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

			if (imm.GetSize() == SIZE_INT8 && r_m.GetSize() != SIZE_INT8) {	// sign-extension
				db(0x83);	// Immediate Grp 1
				EncodeModRM(opcode / 8, r_m);
				db(imm.GetImm());
			} else {
				uint8 w = r_m.GetSize() != SIZE_INT8 ? 1 : 0;
				if (r_m.IsReg() && r_m.GetReg() == EAX) {
					db(opcode | 4 | w);
				} else {
					db(0x80 | w);	// Immediate Grp 1
					EncodeModRM(opcode / 8, r_m);
				}
				if (r_m.GetSize() == SIZE_INT8) db(imm.GetImm());
				else if (r_m.GetSize() == SIZE_INT16) dw(imm.GetImm());
				else if (r_m.GetSize() == SIZE_INT32) dd(imm.GetImm());
				else if (r_m.GetSize() == SIZE_INT64) dd(imm.GetImm());
				else ASSERT(0);
			}
		} else {
			ASSERT(opd1.GetSize() == opd2.GetSize());

			const Opd& reg = opd1.IsReg() ? opd1 : opd2;
			const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexWRXB(reg, r_m);
#else
			if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

			uint8 d = opd1.IsReg() ? 1 : 0;
			uint8 w = reg.GetSize() != SIZE_INT8 ? 1 : 0;
			db(opcode | d << 1 | w);
			EncodeModRM(reg, r_m);
		}
	}

	void EncodeCALL(const Opd& opd)
	{
		if (opd.IsReg()) {
			if (opd.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			db(0xFF);
			EncodeModRM(2, opd);
		} else {
			// TODO: Support for relative displacement
			ASSERT(0);
		}
	}

	void EncodeINC(uint8 opcode1, uint8 digit, const Opd& opd)
	{
#ifdef JITASM64
		if (opd.IsMem() && opd.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
		if (opd.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
		EncodeRexWRXB(opd);
#else
		if (opd.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

		uint8 w = opd.GetSize() != SIZE_INT8 ? 1 : 0;

#ifdef JITASM64
		db(0xFE | w);	// Grp4, Grp5
		EncodeModRM(digit, opd);
#else
		if (opd.IsReg() && w) {
			db(opcode1 | opd.GetReg() & 0xF);
		} else {
			db(0xFE | w);	// Grp4, Grp5
			EncodeModRM(digit, opd);
		}
#endif
	}

	void EncodeJCC(uint8 tttn, const Opd& opd)
	{
		if (opd.GetSize() <= SIZE_INT8) db(0x70 | tttn), db(opd.GetImm());
		else if (opd.GetSize() <= SIZE_INT32) db(0x0F), db(0x80 | tttn), dd(opd.GetImm());
		else ASSERT(0);
	}

	void EncodeJMP(const Opd& opd)
	{
		if (opd.GetSize() <= SIZE_INT8) db(0xEB), db(opd.GetImm());
		else if (opd.GetSize() <= SIZE_INT32) db(0xE9), dd(opd.GetImm());
		else ASSERT(0);
	}

	void EncodeLEA(const Opd& reg, const Opd& mem)
	{
#ifdef JITASM64
		if (mem.IsMem() && mem.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
		if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
		EncodeRexWRXB(reg, mem);
#else
		if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

		db(0x8D);
		EncodeModRM(reg, mem);
	}

	void EncodeMOV(const Opd& opd1, const Opd& opd2)
	{
		if (opd2.IsImm()) {
			const Opd& r_m = opd1;
			const Opd& imm = opd2;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexWRXB(r_m);
#else
			if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

#ifdef JITASM64
			if (r_m.GetSize() == SIZE_INT64 && imm.GetSize() <= SIZE_INT32) {	// sign-extension
				db(0xC7);
				EncodeModRM(0, r_m);
				dd(imm.GetImm());
				return;
			}
#endif

			uint8 w = r_m.GetSize() != SIZE_INT8 ? 1 : 0;
			if (r_m.IsReg()) {
				db(0xB0 | w << 3 | r_m.GetReg() & 0xF);
			} else {
				db(0xC6 | w);
				EncodeModRM(0, r_m);
			}
			if (r_m.GetSize() == SIZE_INT8) db(imm.GetImm());
			else if (r_m.GetSize() == SIZE_INT8) db(imm.GetImm());
			else if (r_m.GetSize() == SIZE_INT16) dw(imm.GetImm());
			else if (r_m.GetSize() == SIZE_INT32) dd(imm.GetImm());
#ifdef JITASM64
			else if (r_m.GetSize() == SIZE_INT64) dq(imm.GetImm());
#endif
			else ASSERT(0);
		} else {
			ASSERT(opd1.GetSize() == opd2.GetSize());

			const Opd& reg = opd1.IsReg() ? opd1 : opd2;
			const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexWRXB(reg, r_m);
#else
			if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

			uint8 w = reg.GetSize() != SIZE_INT8 ? 1 : 0;
#ifdef JITASM64
			if (reg.GetReg() == EAX && r_m.IsMem() && r_m.GetBase() == INVALID && r_m.GetIndex() == INVALID && !detail::IsInt32(r_m.GetDisp())) {
				uint8 d = opd1.IsReg() ? 0 : 1;
				db(0xA0 | d << 1 | w);
				dq(r_m.GetDisp());
				return;
			}
#else
			if (reg.GetReg() == EAX && r_m.IsMem() && r_m.GetBase() == INVALID && r_m.GetIndex() == INVALID) {
				uint8 d = opd1.IsReg() ? 0 : 1;
				db(0xA0 | d << 1 | w);
				dd(r_m.GetDisp());
				return;
			}
#endif
			uint8 d = opd1.IsReg() ? 1 : 0;
			db(0x88 | d << 1 | w);
			EncodeModRM(reg, r_m);
		}
	}

	void EncodeMOVZX(const Opd& opd1, const Opd& opd2)
	{
		ASSERT(opd1.IsReg());

		const Opd& reg = opd1;
		const Opd& r_m = opd2;

#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
		if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
		EncodeRexWRXB(reg, r_m);
#else
		if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

		db(0x0F);
		uint8 w = r_m.GetSize() != SIZE_INT8 ? 1 : 0;
		db(0xB6 | w);
		EncodeModRM(reg, r_m);
	}

	void EncodePOP(uint8 opcode1, uint8 opcode2, uint8 digit, const Opd& opd)
	{
		if (opd.IsImm()) {
			if (opd.GetSize() == SIZE_INT8) {	// sign-extention
				db(0x6A);
				db(opd.GetImm());
			} else {
				db(0x68);
				dd(opd.GetImm());
			}
		} else {
#ifdef JITASM64
			if (opd.IsMem() && opd.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (opd.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexRXB(opd);
#else
			if (opd.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

			if (opd.IsReg()) {
				db(opcode1 | opd.GetReg() & 0xF);
			} else if (opd.IsMem()) {
				db(opcode2);
				EncodeModRM(digit, opd);
			}
		}
	}

	void EncodeShift(uint8 digit, const Opd& r_m, const Opd& imm)
	{
		ASSERT(imm.IsImm());

#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
		if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
		EncodeRexWRXB(r_m);
#else
		if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif
		uint8 w = r_m.GetSize() != SIZE_INT8 ? 1 : 0;
		if (imm.GetImm() == 1) {
			db(0xD0 | w);
			EncodeModRM(digit, r_m);
		} else {
			db(0xC0 | w);
			EncodeModRM(digit, r_m);
			db(imm.GetImm());
		}
	}

	void EncodeTEST(const Opd& opd1, const Opd& opd2)
	{
		uint8 w = opd1.GetSize() != SIZE_INT8 ? 1 : 0;
		if (opd2.IsImm()) {
			const Opd& r_m = opd1;
			const Opd& imm = opd2;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexWRXB(r_m);
#else
			if (r_m.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

			if (r_m.IsReg() && r_m.GetReg() == EAX) {
				db(0xA8 | w);
			} else {
				db(0xF6 | w);
				EncodeModRM(0, r_m);
			}
			if (r_m.GetSize() == SIZE_INT8) db(imm.GetImm());
			else if (r_m.GetSize() == SIZE_INT16) dw(imm.GetImm());
			else if (r_m.GetSize() == SIZE_INT32) dd(imm.GetImm());
			else if (r_m.GetSize() == SIZE_INT64) dd(imm.GetImm());	// sign-extention
		} else {
			ASSERT(opd2.IsReg());
			const Opd& reg = opd1.IsReg() ? opd1 : opd2;
			const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
			if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
			EncodeRexWRXB(reg, r_m);
#else
			if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

			db(0x84 | w);
			EncodeModRM(reg, r_m);
		}
	}

	void EncodeXCHG(const Opd& opd1, const Opd& opd2)
	{
		const Opd& reg = opd1.IsReg() && (opd2.GetSize() == SIZE_INT8 || !opd2.IsReg() || opd2.GetReg() != EAX) ? opd1 : opd2;
		const Opd& r_m = &reg == &opd1 ? opd2 : opd1;

#ifdef JITASM64
		if (r_m.IsMem() && r_m.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
		if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
		EncodeRexWRXB(reg, r_m);	// TODO: In 64-bit mode, r/m8 can not be encoded to access following byte registers if a REX prefix is used: AH, BH, CH, DH.
#else
		if (reg.GetSize() == SIZE_INT16) EncodeOperandSizePrefix();
#endif

		if (reg.GetSize() != SIZE_INT8 && r_m.IsReg()) {
			if (reg.GetReg() == EAX) {
				db(0x90 | r_m.GetReg() & 0xF);
				return;
			}
			if (r_m.GetReg() == EAX) {
				db(0x90 | reg.GetReg() & 0xF);
				return;
			}
		}

		uint8 w = reg.GetSize() != SIZE_INT8 ? 1 : 0;
		db(0x86 | w);
		EncodeModRM(reg, r_m);
	}

	void EncodeFLD(const Opd& opd)
	{
		if (opd.IsReg()) {
			db(0xD9);
			db(0xC0 | opd.GetReg());
		} else {
#ifdef JITASM64
			if (opd.IsMem() && opd.GetAddressSize() != SIZE_INT64) EncodeAddressSizePrefix();
#endif
			int digit = 0;
			if (opd.GetSize() == SIZE_INT32) db(0xD9), digit = 0;
			else if (opd.GetSize() == SIZE_INT64) db(0xDD), digit = 0;
			else if (opd.GetSize() == SIZE_INT80) db(0xDB), digit = 5;
			else ASSERT(0);
			EncodeModRM(digit, opd);
		}
	}

	void EncodeMMX(uint8 opcode2, const Opd& opd1, const Opd& opd2)
	{
		const Opd& reg = opd1.IsReg() ? opd1 : opd2;
		const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

#ifdef JITASM64
		EncodeRexWRXB(reg, r_m);
#endif
		db(0x0F);
		db(opcode2);
		EncodeModRM(reg, r_m);
	}

	void EncodeMMX(uint8 opcode2, uint8 opcode3, const Opd& opd1, const Opd& opd2)
	{
		const Opd& reg = opd1.IsReg() ? opd1 : opd2;
		const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

#ifdef JITASM64
		EncodeRexWRXB(reg, r_m);
#endif
		db(0x0F);
		db(opcode2);
		db(opcode3);
		EncodeModRM(reg, r_m);
	}

	void EncodeSSE(uint8 opcode2, const Opd& opd1, const Opd& opd2)
	{
		EncodeMMX(opcode2, opd1, opd2);
	}

	void EncodeSSE(uint8 opcode2, uint8 opcode3, const Opd& opd1, const Opd& opd2)
	{
		EncodeMMX(opcode2, opcode3, opd1, opd2);
	}

	/// Encode SSE2 instruction
	/**
	 * Encode 2 bytes opcode SSE2 instruction. First-byte opcode is 0x0F.
	 * \param prefix	A mandatory prefix for SSE2.
	 * \param opcode2	Second-byte opcode.
	 * \param opd1		First operand.
	 * \param opd2		Second operand.
	 */
	void EncodeSSE2(uint8 prefix, uint8 opcode2, const Opd& opd1, const Opd& opd2)
	{
		db(prefix);
		EncodeMMX(opcode2, opd1, opd2);
	}

	/// Encode SSE2 instruction
	/**
	 * Encode 3 bytes opcode SSE2 instruction. First-byte opcode is 0x0F.
	 * \param prefix	A mandatory prefix for SSE2.
	 * \param opcode2	Second-byte opcode.
	 * \param opcode3	Third-byte opcode.
	 * \param opd1		First operand.
	 * \param opd2		Second operand.
	 */
	void EncodeSSE2(uint8 prefix, uint8 opcode2, uint8 opcode3, const Opd& opd1, const Opd& opd2)
	{
		db(prefix);
		EncodeMMX(opcode2, opcode3, opd1, opd2);
	}

	void EncodeMMXorSSE2(uint8 prefix, uint8 opcode2, const Opd& opd1, const Opd& opd2)
	{
		if ((opd1.IsReg() && opd1.GetSize() == SIZE_INT128) ||
			(opd2.IsReg() && opd2.GetSize() == SIZE_INT128)) {
			EncodeSSE2(prefix, opcode2, opd1, opd2);
		}
		else {
			EncodeMMX(opcode2, opd1, opd2);
		}
	}

	void EncodeMMXorSSE2(uint8 prefix, uint8 opcode2, uint8 opcode3, const Opd& opd1, const Opd& opd2)
	{
		if ((opd1.IsReg() && opd1.GetSize() == SIZE_INT128) ||
			(opd2.IsReg() && opd2.GetSize() == SIZE_INT128)) {
			EncodeSSE2(prefix, opcode2, opcode3, opd1, opd2);
		}
		else {
			EncodeMMX(opcode2, opcode3, opd1, opd2);
		}
	}

	void EncodeCLFLUSH(const Opd& opd)
	{
		ASSERT(opd.IsMem() && opd.GetSize() == SIZE_INT8);
		db(0x0F);
		db(0xAE);
		EncodeModRM(7, opd);
	}

	void EncodeMOVQ(const Opd& opd1, const Opd& opd2)
	{
		if ((opd1.IsReg() ? opd1 : opd2).GetSize() == SIZE_INT64) {
			EncodeMMX(0x6F | (opd1.IsReg() ? 0 : 0x10), opd1, opd2);
		}
		else if (opd1.IsReg()) {
			EncodeSSE2(0xF3, 0x7E, opd1, opd2);
		}
		else {
			EncodeSSE2(0x66, 0xD6, opd1, opd2);
		}
	}

	void Assemble(const Instr& instr)
	{
		const Opd& opd1 = instr.GetOpd(0);
		const Opd& opd2 = instr.GetOpd(1);
		const Opd& opd3 = instr.GetOpd(2);

		switch (instr.GetID()) {
		// General-Purpose Instructions
		case INSTR_ADC:			EncodeADD(0x10, opd1, opd2); break;
		case INSTR_ADD:			EncodeADD(0x00, opd1, opd2); break;
		case INSTR_AND:			EncodeADD(0x20, opd1, opd2); break;
		case INSTR_CALL:		EncodeCALL(opd1); break;
		case INSTR_CMP:			EncodeADD(0x38, opd1, opd2); break;
		case INSTR_DEC:			EncodeINC(0x48, 1, opd1); break;
		case INSTR_INC:			EncodeINC(0x40, 0, opd1); break;
		case INSTR_JMP:			EncodeJMP(opd1); break;
		case INSTR_JA:			EncodeJCC(0x7, opd1); break;
		case INSTR_JAE:			EncodeJCC(0x3, opd1); break;
		case INSTR_JB:			EncodeJCC(0x2, opd1); break;
		case INSTR_JBE:			EncodeJCC(0x6, opd1); break;
#ifdef JITASM64
		case INSTR_JECXZ:		EncodeAddressSizePrefix(); db(0xE3); db(opd1.GetImm()); break;
		case INSTR_JRCXZ:		db(0xE3); db(opd1.GetImm()); break;
#else
		case INSTR_JECXZ:		db(0xE3); db(opd1.GetImm()); break;
		case INSTR_JCXZ:		EncodeAddressSizePrefix(); db(0xE3); db(opd1.GetImm()); break;
#endif
		case INSTR_JE:			EncodeJCC(0x4, opd1); break;
		case INSTR_JG:			EncodeJCC(0xF, opd1); break;
		case INSTR_JGE:			EncodeJCC(0xD, opd1); break;
		case INSTR_JL:			EncodeJCC(0xC, opd1); break;
		case INSTR_JLE:			EncodeJCC(0xE, opd1); break;
		case INSTR_JNE:			EncodeJCC(0x5, opd1); break;
		case INSTR_JNO:			EncodeJCC(0x1, opd1); break;
		case INSTR_JNP:			EncodeJCC(0xB, opd1); break;
		case INSTR_JNS:			EncodeJCC(0x9, opd1); break;
		case INSTR_JO:			EncodeJCC(0x0, opd1); break;
		case INSTR_JP:			EncodeJCC(0xA, opd1); break;
		case INSTR_JS:			EncodeJCC(0x8, opd1); break;
		case INSTR_LEA:			EncodeLEA(opd1, opd2); break;
		case INSTR_LEAVE:		db(0xC9); break;
		case INSTR_MOV:			EncodeMOV(opd1, opd2); break;
		case INSTR_MOVZX:		EncodeMOVZX(opd1, opd2); break;
		case INSTR_NOP:			db(0x90); break;
		case INSTR_OR:			EncodeADD(0x08, opd1, opd2); break;
		case INSTR_POP:			EncodePOP(0x58, 0x8F, 0, opd1); break;
		case INSTR_PUSH:		EncodePOP(0x50, 0xFF, 6, opd1); break;
		case INSTR_RET:			if (opd1.IsNone()) db(0xC3); else db(0xC2), dw(opd1.GetImm()); break;
		case INSTR_SAR:			EncodeShift(7, opd1, opd2); break;
		case INSTR_SHL:			EncodeShift(4, opd1, opd2); break;
		case INSTR_SHR:			EncodeShift(5, opd1, opd2); break;
		case INSTR_SBB:			EncodeADD(0x18, opd1, opd2); break;
		case INSTR_SUB:			EncodeADD(0x28, opd1, opd2); break;
		case INSTR_TEST:		EncodeTEST(opd1, opd2); break;
		case INSTR_XCHG:		EncodeXCHG(opd1, opd2); break;
		case INSTR_XOR:			EncodeADD(0x30, opd1, opd2); break;

		// x87 Floating-Point Instructions
		case INSTR_FLD:			EncodeFLD(opd1); break;

		// MMX/SSE/SSE2 Instructions
		case INSTR_ADDPD:		EncodeSSE2(0x66, 0x58, opd1, opd2); break;
		case INSTR_ADDSD:		EncodeSSE2(0xF2, 0x58, opd1, opd2); break;
		case INSTR_ANDPD:		EncodeSSE2(0x66, 0x54, opd1, opd2); break;
		case INSTR_ANDNPD:		EncodeSSE2(0x66, 0x55, opd1, opd2); break;
		case INSTR_CLFLUSH:		EncodeCLFLUSH(opd1); break;
		case INSTR_CMPPD:		EncodeSSE2(0x66, 0xC2, opd1, opd2); db(opd3.GetImm()); break;
		case INSTR_CMPPS:		EncodeSSE(0xC2, opd1, opd2); db(opd3.GetImm()); break;
		case INSTR_CMPSD:		EncodeSSE2(0xF2, 0xC2, opd1, opd2); db(opd3.GetImm()); break;
		case INSTR_COMISD:		EncodeSSE2(0x66, 0x2F, opd1, opd2); break;
		case INSTR_CVTDQ2PD:	EncodeSSE2(0xF3, 0xE6, opd1, opd2); break;
		case INSTR_CVTPD2DQ:	EncodeSSE2(0xF2, 0xE6, opd1, opd2); break;
		case INSTR_CVTPD2PI:	EncodeSSE2(0x66, 0x2D, opd1, opd2); break;
		case INSTR_CVTPD2PS:	EncodeSSE2(0x66, 0x5A, opd1, opd2); break;
		case INSTR_CVTPI2PD:	EncodeSSE2(0x66, 0x2A, opd1, opd2); break;
		case INSTR_CVTPS2DQ:	EncodeSSE2(0x66, 0x5B, opd1, opd2); break;
		case INSTR_CVTDQ2PS:	EncodeSSE(0x5B, opd1, opd2); break;
		case INSTR_CVTPS2PD:	EncodeSSE(0x5A, opd1, opd2); break;	// SSE2!!!
		//case INSTR_CVTSD2SI:
		case INSTR_CVTSD2SS:	EncodeSSE2(0xF2, 0x5A, opd1, opd2); break;
		//case INSTR_CVTSI2SD:
		case INSTR_CVTSS2SD:	EncodeSSE2(0xF3, 0x5A, opd1, opd2); break;
		case INSTR_CVTTPD2DQ:	EncodeSSE2(0x66, 0xE6, opd1, opd2); break;
		case INSTR_CVTTPD2PI:	EncodeSSE2(0x66, 0x2C, opd1, opd2); break;
		case INSTR_CVTTPS2DQ:	EncodeSSE2(0xF3, 0x5B, opd1, opd2); break;
		//case INSTR_CVTTSD2SI:
		case INSTR_DIVPD:		EncodeSSE2(0x66, 0x5E, opd1, opd2); break;
		case INSTR_DIVSD:		EncodeSSE2(0xF2, 0x5E, opd1, opd2); break;
		//case INSTR_LFENCE:
		case INSTR_MASKMOVDQU:	EncodeSSE2(0x66, 0xF7, opd1, opd2); break;
		case INSTR_MAXPD:		EncodeSSE2(0x66, 0x5F, opd1, opd2); break;
		case INSTR_MAXSD:		EncodeSSE2(0xF2, 0x5F, opd1, opd2); break;
		//case INSTR_MFENCE:
		case INSTR_MINPD:		EncodeSSE2(0x66, 0x5D, opd1, opd2); break;
		case INSTR_MINSD:		EncodeSSE2(0xF2, 0x5D, opd1, opd2); break;
		case INSTR_MOVAPD:		EncodeSSE2(0x66, 0x28 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		//case INSTR_MOVD:
		case INSTR_MOVDQ2Q:		EncodeSSE2(0xF2, 0xD6, opd1, opd2); break;
		case INSTR_MOVDQA:		EncodeSSE2(0x66, 0x6F | (opd1.IsReg() ? 0 : 0x10), opd1, opd2); break;
		case INSTR_MOVDQU:		EncodeSSE2(0xF3, 0x6F | (opd1.IsReg() ? 0 : 0x10), opd1, opd2); break;
		case INSTR_MOVHPD:		EncodeSSE2(0x66, 0x16 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case INSTR_MOVLPD:		EncodeSSE2(0x66, 0x12 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case INSTR_MOVMSKPD:	EncodeSSE2(0x66, 0x50, opd1, opd2); break;
		case INSTR_MOVNTPD:		EncodeSSE2(0x66, 0x2B, opd1, opd2); break;
		case INSTR_MOVNTDQ:		EncodeSSE2(0x66, 0xE7, opd1, opd2); break;
		//case INSTR_MOVNTI:
		case INSTR_MOVQ:		EncodeMOVQ(opd1, opd2); break;
		case INSTR_MOVQ2DQ:		EncodeSSE2(0xF3, 0xD6, opd1, opd2); break;
		case INSTR_MOVSD:		EncodeSSE2(0xF2, 0x10 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case INSTR_MOVUPD:		EncodeSSE2(0x66, 0x10 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case INSTR_MULPD:		EncodeSSE2(0x66, 0x59, opd1, opd2); break;
		case INSTR_MULSD:		EncodeSSE2(0xF2, 0x59, opd1, opd2); break;
		case INSTR_ORPD:		EncodeSSE2(0x66, 0x56, opd1, opd2); break;
		case INSTR_PABSB:		EncodeMMXorSSE2(0x66, 0x38, 0x1C, opd1, opd2); break;	// SSSE3
		case INSTR_PABSW:		EncodeMMXorSSE2(0x66, 0x38, 0x1D, opd1, opd2); break;	// SSSE3
		case INSTR_PABSD:		EncodeMMXorSSE2(0x66, 0x38, 0x1E, opd1, opd2); break;	// SSSE3
		case INSTR_PACKSSWB:	EncodeMMXorSSE2(0x66, 0x63, opd1, opd2); break;
		case INSTR_PACKSSDW:	EncodeMMXorSSE2(0x66, 0x6B, opd1, opd2); break;
		case INSTR_PACKUSWB:	EncodeMMXorSSE2(0x66, 0x67, opd1, opd2); break;
		case INSTR_PACKUSDW:	EncodeSSE2(0x66, 0x38, 0x2B, opd1, opd2); break;		// SSE 4.1
		case INSTR_PADDB:		EncodeMMXorSSE2(0x66, 0xFC, opd1, opd2); break;
		case INSTR_PADDW:		EncodeMMXorSSE2(0x66, 0xFD, opd1, opd2); break;
		case INSTR_PADDD:		EncodeMMXorSSE2(0x66, 0xFE, opd1, opd2); break;
		case INSTR_PADDQ:		EncodeMMXorSSE2(0x66, 0xD4, opd1, opd2); break;
		case INSTR_PADDSB:		EncodeMMXorSSE2(0x66, 0xEC, opd1, opd2); break;
		case INSTR_PADDSW:		EncodeMMXorSSE2(0x66, 0xED, opd1, opd2); break;
		case INSTR_PADDUSB:		EncodeMMXorSSE2(0x66, 0xDC, opd1, opd2); break;
		case INSTR_PADDUSW:		EncodeMMXorSSE2(0x66, 0xDD, opd1, opd2); break;
		case INSTR_PALIGNR:		EncodeMMXorSSE2(0x66, 0x3A, 0x0F, opd1, opd2); break;	// SSSE3
		case INSTR_PAND:		EncodeMMXorSSE2(0x66, 0xDB, opd1, opd2); break;
		case INSTR_PANDN:		EncodeMMXorSSE2(0x66, 0xDF, opd1, opd2); break;
		case INSTR_PAUSE:		break;
		case INSTR_PAVGB:		EncodeMMXorSSE2(0x66, 0xE0, opd1, opd2); break;
		case INSTR_PAVGW:		EncodeMMXorSSE2(0x66, 0xE3, opd1, opd2); break;
		case INSTR_PCMPEQB:		EncodeMMXorSSE2(0x66, 0x74, opd1, opd2); break;
		case INSTR_PCMPEQW:		EncodeMMXorSSE2(0x66, 0x75, opd1, opd2); break;
		case INSTR_PCMPEQD:		EncodeMMXorSSE2(0x66, 0x76, opd1, opd2); break;
		case INSTR_PCMPEQQ:		EncodeSSE2(0x66, 0x38, 0x29, opd1, opd2); break;
		case INSTR_PCMPGTB:		EncodeMMXorSSE2(0x66, 0x64, opd1, opd2); break;
		case INSTR_PCMPGTW:		EncodeMMXorSSE2(0x66, 0x65, opd1, opd2); break;
		case INSTR_PCMPGTD:		EncodeMMXorSSE2(0x66, 0x66, opd1, opd2); break;
		case INSTR_PCMPGTQ:		EncodeSSE2(0x66, 0x38, 0x37, opd1, opd2); break;
		//case INSTR_PEXTRW:
		//case INSTR_PINSRW:
		case INSTR_PMADDWD:		EncodeMMXorSSE2(0x66, 0xF5, opd1, opd2); break;
		case INSTR_PMAXSW:		EncodeMMXorSSE2(0x66, 0xEE, opd1, opd2); break;
		case INSTR_PMAXUB:		EncodeMMXorSSE2(0x66, 0xDE, opd1, opd2); break;
		case INSTR_PMINSW:		EncodeMMXorSSE2(0x66, 0xEA, opd1, opd2); break;
		case INSTR_PMINUB:		EncodeMMXorSSE2(0x66, 0xDA, opd1, opd2); break;
		//case INSTR_PMOVMSKB:
		case INSTR_PMULHUW:		EncodeMMXorSSE2(0x66, 0xE4, opd1, opd2); break;
		case INSTR_PMULHW:		EncodeMMXorSSE2(0x66, 0xE5, opd1, opd2); break;
		case INSTR_PMULLW:		EncodeMMXorSSE2(0x66, 0xD5, opd1, opd2); break;
		case INSTR_PMULUDQ:		EncodeMMXorSSE2(0x66, 0xF4, opd1, opd2); break;
		case INSTR_POR:			EncodeMMXorSSE2(0x66, 0xEB, opd1, opd2); break;
		case INSTR_PSADBW:		EncodeMMXorSSE2(0x66, 0xF6, opd1, opd2); break;
		//case INSTR_PSHUFD:
		//case INSTR_PSHUFHW:
		//case INSTR_PSHUFLW:
		//case INSTR_PSLLW:
		//case INSTR_PSLLD:
		//case INSTR_PSLLQ:
		//case INSTR_PSLLDQ:
		//case INSTR_PSRAW:
		//case INSTR_PSRAD:
		//case INSTR_PSRLW:
		//case INSTR_PSRLD:
		//case INSTR_PSRLQ:
		//case INSTR_PSRLDQ:
		case INSTR_PSUBB:		EncodeMMXorSSE2(0x66, 0xF8, opd1, opd2); break;
		case INSTR_PSUBW:		EncodeMMXorSSE2(0x66, 0xF9, opd1, opd2); break;
		case INSTR_PSUBD:		EncodeMMXorSSE2(0x66, 0xFA, opd1, opd2); break;
		case INSTR_PSUBQ:		EncodeMMXorSSE2(0x66, 0xFB, opd1, opd2); break;
		case INSTR_PSUBSB:		EncodeMMXorSSE2(0x66, 0xE8, opd1, opd2); break;
		case INSTR_PSUBSW:		EncodeMMXorSSE2(0x66, 0xE9, opd1, opd2); break;
		case INSTR_PSUBUSB:		EncodeMMXorSSE2(0x66, 0xD8, opd1, opd2); break;
		case INSTR_PSUBUSW:		EncodeMMXorSSE2(0x66, 0xD9, opd1, opd2); break;
		case INSTR_PUNPCKHBW:	EncodeMMXorSSE2(0x66, 0x68, opd1, opd2); break;
		case INSTR_PUNPCKHWD:	EncodeMMXorSSE2(0x66, 0x69, opd1, opd2); break;
		case INSTR_PUNPCKHDQ:	EncodeMMXorSSE2(0x66, 0x6A, opd1, opd2); break;
		case INSTR_PUNPCKHQDQ:	EncodeSSE2(0x66, 0x6D, opd1, opd2); break;
		case INSTR_PUNPCKLBW:	EncodeMMXorSSE2(0x66, 0x60, opd1, opd2); break;
		case INSTR_PUNPCKLWD:	EncodeMMXorSSE2(0x66, 0x61, opd1, opd2); break;
		case INSTR_PUNPCKLDQ:	EncodeMMXorSSE2(0x66, 0x62, opd1, opd2); break;
		case INSTR_PUNPCKLQDQ:	EncodeSSE2(0x66, 0x6C, opd1, opd2); break;
		case INSTR_PXOR:		EncodeMMXorSSE2(0x66, 0xEF, opd1, opd2); break;
		//case INSTR_SHUFPD:
		case INSTR_SQRTPD:		EncodeSSE2(0x66, 0x51, opd1, opd2); break;
		case INSTR_SQRTSD:		EncodeSSE2(0xF2, 0x51, opd1, opd2); break;
		case INSTR_SUBPD:		EncodeSSE2(0x66, 0x5C, opd1, opd2); break;
		case INSTR_SUBSD:		EncodeSSE2(0xF2, 0x5C, opd1, opd2); break;
		case INSTR_UCOMISD:		EncodeSSE2(0x66, 0x2E, opd1, opd2); break;
		case INSTR_UNPCKHPD:	EncodeSSE2(0x66, 0x15, opd1, opd2); break;
		case INSTR_UNPCKLPD:	EncodeSSE2(0x66, 0x14, opd1, opd2); break;
		case INSTR_XORPD:		EncodeSSE2(0x66, 0x57, opd1, opd2); break;

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
	Mmx mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7;
	Xmm xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7;
#ifdef JITASM64
	Reg8 r8b, r9b, r10b, r11b, r12b, r13b, r14b, r15b;
	Reg16 r8w, r9w, r10w, r11w, r12w, r13w, r14w, r15w;
	Reg32 r8d, r9d, r10d, r11d, r12d, r13d, r14d, r15d;
	Rax rax;
	Reg64 rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15;
	Xmm xmm8, xmm9, xmm10, xmm11, xmm12, xmm13, xmm14, xmm15;
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
		// TODO Save XMM registers
#endif
	}

	virtual void Epilog()
	{
#ifdef JITASM64
		// TODO Restore XMM registers
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
		return id == INSTR_JMP || id == INSTR_JA || id == INSTR_JAE || id == INSTR_JB
			|| id == INSTR_JBE || id == INSTR_JCXZ || id == INSTR_JECXZ || id == INSTR_JRCXZ
			|| id == INSTR_JE || id == INSTR_JG || id == INSTR_JGE || id == INSTR_JL
			|| id == INSTR_JLE || id == INSTR_JNE || id == INSTR_JNO || id == INSTR_JNP
			|| id == INSTR_JNS || id == INSTR_JO || id == INSTR_JP || id == INSTR_JS;
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
					if (size == SIZE_INT8) {
						if (!detail::IsInt8(rel)) {
							if (instr.GetID() == INSTR_JRCXZ || instr.GetID() == INSTR_JCXZ || instr.GetID() == INSTR_JECXZ) ASSERT(0);	// jrcxz, jcxz, jecxz are only for short jump

							// Retry with immediate 32
							instr = Instr(instr.GetID(), Imm32(0x7FFFFFFF), Imm64(instr.GetOpd(1).GetImm()));
							retry = true;
						}
					} else if (size == SIZE_INT32) {
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
				if (size == SIZE_INT8) {
					ASSERT(detail::IsInt8(rel));
					instr = Instr(instr.GetID(), Imm8(rel));
				} else if (size == SIZE_INT32) {
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
	void label(const std::string& label_name)
	{
		size_t label_id = GetLabelId(label_name);
		labels_[label_id].instr_number = instrs_.size();	// Label current instruction
	}

	// ALIGN
	void align(size_t n)
	{
		PushBack(Instr(INSTR_PSEUDO_ALIGN, Imm64(n)));
	}

	// ADC
	void adc(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_ADC, opd1, Imm8(imm)));}
	void adc(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_ADC, opd1, Imm16(imm)));}
	void adc(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_ADC, opd1, Imm32(imm)));}
#ifdef JITASM64
	void adc(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_ADC, opd1, opd2));}
	void adc(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_ADC, opd1, Imm32(imm)));}
#endif

	// ADD
	void add(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_ADD, opd1, Imm8(imm)));}
	void add(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_ADD, opd1, Imm16(imm)));}
	void add(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_ADD, opd1, Imm32(imm)));}
#ifdef JITASM64
	void add(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_ADD, opd1, opd2));}
	void add(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_ADD, opd1, Imm32(imm)));}
#endif

	// AND
	void and(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_AND, opd1, Imm8(imm)));}
	void and(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_AND, opd1, Imm16(imm)));}
	void and(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_AND, opd1, Imm32(imm)));}
#ifdef JITASM64
	void and(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_AND, opd1, opd2));}
	void and(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_AND, opd1, Imm32(imm)));}
#endif

	// CALL
#ifndef JITASM64
	void call(const Reg16& opd) {PushBack(Instr(INSTR_CALL, opd));}
	void call(const Reg32& opd) {PushBack(Instr(INSTR_CALL, opd));}
#else
	void call(const Reg64& opd) {PushBack(Instr(INSTR_CALL, opd));}
#endif

	// CMP
	void cmp(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_CMP, opd1, Imm8(imm)));}
	void cmp(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_CMP, opd1, Imm16(imm)));}
	void cmp(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_CMP, opd1, Imm32(imm)));}
#ifdef JITASM64
	void cmp(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_CMP, opd1, opd2));}
	void cmp(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_CMP, opd1, Imm32(imm)));}
#endif

	// DEC
	void dec(const Opd8& opd) {PushBack(Instr(INSTR_DEC, opd));}
	void dec(const Opd16& opd) {PushBack(Instr(INSTR_DEC, opd));}
	void dec(const Opd32& opd) {PushBack(Instr(INSTR_DEC, opd));}
#ifdef JITASM64
	void dec(const Opd64& opd) {PushBack(Instr(INSTR_DEC, opd));}
#endif

	// INC
	void inc(const Opd8& opd) {PushBack(Instr(INSTR_INC, opd));}
	void inc(const Opd16& opd) {PushBack(Instr(INSTR_INC, opd));}
	void inc(const Opd32& opd) {PushBack(Instr(INSTR_INC, opd));}
#ifdef JITASM64
	void inc(const Opd64& opd) {PushBack(Instr(INSTR_INC, opd));}
#endif

	// JMP
	void jmp(const std::string& label_name) {PushBack(Instr(INSTR_JMP, Imm64(GetLabelId(label_name))));}
	void ja(const std::string& label_name) {PushBack(Instr(INSTR_JA, Imm64(GetLabelId(label_name))));}
	void jae(const std::string& label_name) {PushBack(Instr(INSTR_JAE, Imm64(GetLabelId(label_name))));}
	void jb(const std::string& label_name) {PushBack(Instr(INSTR_JB, Imm64(GetLabelId(label_name))));}
	void jbe(const std::string& label_name) {PushBack(Instr(INSTR_JBE, Imm64(GetLabelId(label_name))));}
	void jc(const std::string& label_name) {jb(label_name);}
	void jecxz(const std::string& label_name) {PushBack(Instr(INSTR_JECXZ, Imm64(GetLabelId(label_name))));}	// short jump only
#ifdef JITASM64
	void jrcxz (const std::string& label_name) {PushBack(Instr(INSTR_JRCXZ, Imm64(GetLabelId(label_name))));}	// short jump only
#else
	void jcxz(const std::string& label_name) {PushBack(Instr(INSTR_JCXZ, Imm64(GetLabelId(label_name))));}		// short jump only
#endif
	void je(const std::string& label_name) {PushBack(Instr(INSTR_JE, Imm64(GetLabelId(label_name))));}
	void jg(const std::string& label_name) {PushBack(Instr(INSTR_JG, Imm64(GetLabelId(label_name))));}
	void jge(const std::string& label_name) {PushBack(Instr(INSTR_JGE, Imm64(GetLabelId(label_name))));}
	void jl(const std::string& label_name) {PushBack(Instr(INSTR_JL, Imm64(GetLabelId(label_name))));}
	void jle(const std::string& label_name) {PushBack(Instr(INSTR_JLE, Imm64(GetLabelId(label_name))));}
	void jna(const std::string& label_name) {jbe(label_name);}
	void jnae(const std::string& label_name) {jb(label_name);}
	void jnb(const std::string& label_name) {jae(label_name);}
	void jnbe(const std::string& label_name) {ja(label_name);}
	void jnc(const std::string& label_name) {jae(label_name);}
	void jne(const std::string& label_name) {PushBack(Instr(INSTR_JNE, Imm64(GetLabelId(label_name))));}
	void jng(const std::string& label_name) {jle(label_name);}
	void jnge(const std::string& label_name) {jl(label_name);}
	void jnl(const std::string& label_name) {jge(label_name);}
	void jnle(const std::string& label_name) {jg(label_name);}
	void jno(const std::string& label_name) {PushBack(Instr(INSTR_JNO, Imm64(GetLabelId(label_name))));}
	void jnp(const std::string& label_name) {PushBack(Instr(INSTR_JNP, Imm64(GetLabelId(label_name))));}
	void jns(const std::string& label_name) {PushBack(Instr(INSTR_JNS, Imm64(GetLabelId(label_name))));}
	void jnz(const std::string& label_name) {jne(label_name);}
	void jo(const std::string& label_name) {PushBack(Instr(INSTR_JO, Imm64(GetLabelId(label_name))));}
	void jp(const std::string& label_name) {PushBack(Instr(INSTR_JP, Imm64(GetLabelId(label_name))));}
	void jpe(const std::string& label_name) {jp(label_name);}
	void jpo(const std::string& label_name) {jnp(label_name);}
	void js(const std::string& label_name) {PushBack(Instr(INSTR_JS, Imm64(GetLabelId(label_name))));}
	void jz(const std::string& label_name) {je(label_name);}

	// LEA
	void lea(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_LEA, opd1, opd2));}
	void lea(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_LEA, opd1, opd2));}
#ifdef JITASM64
	void lea(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_LEA, opd1, opd2));}
#endif

	// LEAVE
	void leave() {PushBack(Instr(INSTR_LEAVE));}

	// MOV
	void mov(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_MOV, opd1, Imm8(imm)));}
	void mov(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_MOV, opd1, Imm16(imm)));}
	void mov(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_MOV, opd1, Imm32(imm)));}
#ifdef JITASM64
	void mov(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2));}
	void mov(const Opd64& opd1, uint64 imm) {PushBack(Instr(INSTR_MOV, opd1, Imm64(imm)));}
	void mov(const Rax& opd1, MemOffset64& opd2) {PushBack(Instr(INSTR_MOV, opd1, opd2.ToMem64()));}
#endif
 
	// MOVZX
	void movzx(const Reg16& opd1, const Opd8& opd2) {PushBack(Instr(INSTR_MOVZX, opd1, opd2));}
	void movzx(const Reg32& opd1, const Opd8& opd2) {PushBack(Instr(INSTR_MOVZX, opd1, opd2));}
	void movzx(const Reg32& opd1, const Opd16& opd2) {PushBack(Instr(INSTR_MOVZX, opd1, opd2));}
#ifdef JITASM64
	void movzx(const Reg64& opd1, const Opd8& opd2) {PushBack(Instr(INSTR_MOVZX, opd1, opd2));}
	void movzx(const Reg64& opd1, const Opd16& opd2) {PushBack(Instr(INSTR_MOVZX, opd1, opd2));}
#endif

	// NOP
	void nop() {PushBack(Instr(INSTR_NOP));}

	// OR
	void or(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_OR, opd1, Imm8(imm)));}
	void or(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_OR, opd1, Imm16(imm)));}
	void or(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_OR, opd1, Imm32(imm)));}
#ifdef JITASM64
	void or(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_OR, opd1, opd2));}
	void or(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_OR, opd1, Imm32(imm)));}
#endif

	// POP
	void pop(const Opd16& opd) {PushBack(Instr(INSTR_POP, opd));}
#ifdef JITASM64
	void pop(const Opd64& opd) {PushBack(Instr(INSTR_POP, opd));}
#else
	void pop(const Opd32& opd) {PushBack(Instr(INSTR_POP, opd));}
#endif

	// PUSH
	void push(const Opd16& opd) {PushBack(Instr(INSTR_PUSH, opd));}
#ifdef JITASM64
	void push(const Opd64& opd) {PushBack(Instr(INSTR_PUSH, opd));}
#else
	void push(const Opd32& opd) {PushBack(Instr(INSTR_PUSH, opd));}
#endif
	void push(uint64 imm) {PushBack(Instr(INSTR_PUSH, Imm64(imm)));}

	// RET
	void ret() {PushBack(Instr(INSTR_RET));}
	void ret(uint16 imm) {PushBack(Instr(INSTR_RET, Imm16(imm)));}
 
	// SAL/SAR/SHL/SHR
	void sal(const Opd8& opd1, uint8 imm) {shl(opd1, imm);}
	void sar(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_SAR, opd1, Imm8(imm)));}
	void shl(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_SHL, opd1, Imm8(imm)));}
	void shr(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_SHR, opd1, Imm8(imm)));}
	void sal(const Opd16& opd1, uint8 imm) {shl(opd1, imm);}
	void sar(const Opd16& opd1, uint8 imm) {PushBack(Instr(INSTR_SAR, opd1, Imm8(imm)));}
	void shl(const Opd16& opd1, uint8 imm) {PushBack(Instr(INSTR_SHL, opd1, Imm8(imm)));}
	void shr(const Opd16& opd1, uint8 imm) {PushBack(Instr(INSTR_SHR, opd1, Imm8(imm)));}
	void sal(const Opd32& opd1, uint8 imm) {shl(opd1, imm);}
	void sar(const Opd32& opd1, uint8 imm) {PushBack(Instr(INSTR_SAR, opd1, Imm8(imm)));}
	void shl(const Opd32& opd1, uint8 imm) {PushBack(Instr(INSTR_SHL, opd1, Imm8(imm)));}
	void shr(const Opd32& opd1, uint8 imm) {PushBack(Instr(INSTR_SHR, opd1, Imm8(imm)));}
#ifdef JITASM64
	void sal(const Opd64& opd1, uint8 imm) {shl(opd1, imm);}
	void sar(const Opd64& opd1, uint8 imm) {PushBack(Instr(INSTR_SAR, opd1, Imm8(imm)));}
	void shl(const Opd64& opd1, uint8 imm) {PushBack(Instr(INSTR_SHL, opd1, Imm8(imm)));}
	void shr(const Opd64& opd1, uint8 imm) {PushBack(Instr(INSTR_SHR, opd1, Imm8(imm)));}
#endif

	// SBB
	void sbb(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_SBB, opd1, Imm8(imm)));}
	void sbb(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_SBB, opd1, Imm16(imm)));}
	void sbb(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_SBB, opd1, Imm32(imm)));}
#ifdef JITASM64
	void sbb(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_SBB, opd1, opd2));}
	void sbb(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_SBB, opd1, Imm32(imm)));}
#endif

	// SUB
	void sub(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_SUB, opd1, Imm8(imm)));}
	void sub(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_SUB, opd1, Imm16(imm)));}
	void sub(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_SUB, opd1, Imm32(imm)));}
#ifdef JITASM64
	void sub(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_SUB, opd1, opd2));}
	void sub(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_SUB, opd1, Imm32(imm)));}
#endif

	// TEST
	void test(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_TEST, opd1, Imm8(imm)));}
	void test(const Opd8& opd1, Reg8& opd2) {PushBack(Instr(INSTR_TEST, opd1, opd2));}
	void test(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_TEST, opd1, Imm16(imm)));}
	void test(const Opd16& opd1, Reg16& opd2) {PushBack(Instr(INSTR_TEST, opd1, opd2));}
	void test(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_TEST, opd1, Imm32(imm)));}
	void test(const Opd32& opd1, Reg32& opd2) {PushBack(Instr(INSTR_TEST, opd1, opd2));}
#ifdef JITASM64
	void test(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_TEST, opd1, Imm32(imm)));}
	void test(const Opd64& opd1, Reg64& opd2) {PushBack(Instr(INSTR_TEST, opd1, opd2));}
#endif

	// XCHG
	void xchg(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
#ifdef JITASM64
	void xchg(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
	void xchg(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_XCHG, opd1, opd2));}
#endif

	// XOR
	void xor(const Reg8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Reg8& opd1, const Mem8& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Mem8& opd1, const Reg8& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Opd8& opd1, uint8 imm) {PushBack(Instr(INSTR_XOR, opd1, Imm8(imm)));}
	void xor(const Reg16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Reg16& opd1, const Mem16& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Mem16& opd1, const Reg16& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Opd16& opd1, uint16 imm) {PushBack(Instr(INSTR_XOR, opd1, Imm16(imm)));}
	void xor(const Reg32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Reg32& opd1, const Mem32& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Mem32& opd1, const Reg32& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Opd32& opd1, uint32 imm) {PushBack(Instr(INSTR_XOR, opd1, Imm32(imm)));}
#ifdef JITASM64
	void xor(const Reg64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Reg64& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Mem64& opd1, const Reg64& opd2) {PushBack(Instr(INSTR_XOR, opd1, opd2));}
	void xor(const Opd64& opd1, uint32 imm) {PushBack(Instr(INSTR_XOR, opd1, Imm32(imm)));}
#endif

	// FLD
	void fld(const Mem32& opd) {PushBack(Instr(INSTR_FLD, opd));}
	void fld(const Mem64& opd) {PushBack(Instr(INSTR_FLD, opd));}
	void fld(const Mem80& opd) {PushBack(Instr(INSTR_FLD, opd));}
	void fld(const FpuReg& opd) {PushBack(Instr(INSTR_FLD, opd));}

	void addpd(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(INSTR_ADDPD, opd1, opd2));}
	void addpd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_ADDPD, opd1, opd2));}
	void addsd(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(INSTR_ADDSD, opd1, opd2));}
	void addsd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_ADDSD, opd1, opd2));}

	void andpd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_ANDPD, opd1, opd2));}
	void andpd(const Xmm& opd1, const Mem128& opd2)		{PushBack(Instr(INSTR_ANDPD, opd1, opd2));}
	void andnpd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_ANDPD, opd1, opd2));}
	void andnpd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_ANDPD, opd1, opd2));}

	void clflush(const Mem8& opd) {PushBack(Instr(INSTR_CLFLUSH, opd));}

	void cmppd(const Xmm& opd1, const Xmm& opd2, uint8 opd3)	{PushBack(Instr(INSTR_CMPPD, opd1, opd2, Imm8(opd3)));}
	void cmppd(const Xmm& opd1, const Mem128& opd2, uint8 opd3)	{PushBack(Instr(INSTR_CMPPD, opd1, opd2, Imm8(opd3)));}
	void cmpeqpd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 0);}
	void cmpeqpd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 0);}
	void cmpltpd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 1);}
	void cmpltpd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 1);}
	void cmplepd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 2);}
	void cmplepd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 2);}
	void cmpunordpd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 3);}
	void cmpunordpd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 3);}
	void cmpneqpd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 4);}
	void cmpneqpd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 4);}
	void cmpnltpd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 5);}
	void cmpnltpd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 5);}
	void cmpnlepd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 6);}
	void cmpnlepd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 6);}
	void cmpordpd(const Xmm& opd1, const Xmm& opd2)				{cmppd(opd1, opd2, 7);}
	void cmpordpd(const Xmm& opd1, const Mem128& opd2)			{cmppd(opd1, opd2, 7);}

	void cmpps(const Xmm& opd1, const Xmm& opd2, uint8 opd3)	{PushBack(Instr(INSTR_CMPPS, opd1, opd2, Imm8(opd3)));}
	void cmpps(const Xmm& opd1, const Mem128& opd2, uint8 opd3)	{PushBack(Instr(INSTR_CMPPS, opd1, opd2, Imm8(opd3)));}
	void cmpeqps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 0);}
	void cmpeqps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 0);}
	void cmpltps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 1);}
	void cmpltps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 1);}
	void cmpleps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 2);}
	void cmpleps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 2);}
	void cmpunordps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 3);}
	void cmpunordps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 3);}
	void cmpneqps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 4);}
	void cmpneqps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 4);}
	void cmpnltps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 5);}
	void cmpnltps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 5);}
	void cmpnleps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 6);}
	void cmpnleps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 6);}
	void cmpordps(const Xmm& opd1, const Xmm& opd2)				{cmpps(opd1, opd2, 7);}
	void cmpordps(const Xmm& opd1, const Mem128& opd2)			{cmpps(opd1, opd2, 7);}

	void cmpsd(const Xmm& opd1, const Xmm& opd2, uint8 opd3)	{PushBack(Instr(INSTR_CMPSD, opd1, opd2, Imm8(opd3)));}
	void cmpsd(const Xmm& opd1, const Mem64& opd2, uint8 opd3)	{PushBack(Instr(INSTR_CMPSD, opd1, opd2, Imm8(opd3)));}
	void cmpeqsd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 0);}
	void cmpeqsd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 0);}
	void cmpltsd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 1);}
	void cmpltsd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 1);}
	void cmplesd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 2);}
	void cmplesd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 2);}
	void cmpunordsd(const Xmm& opd1, const Xmm& opd2)			{cmpsd(opd1, opd2, 3);}
	void cmpunordsd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 3);}
	void cmpneqsd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 4);}
	void cmpneqsd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 4);}
	void cmpnltsd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 5);}
	void cmpnltsd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 5);}
	void cmpnlesd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 6);}
	void cmpnlesd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 6);}
	void cmpordsd(const Xmm& opd1, const Xmm& opd2)				{cmpsd(opd1, opd2, 7);}
	void cmpordsd(const Xmm& opd1, const Mem64& opd2)			{cmpsd(opd1, opd2, 7);}

	void comisd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_COMISD, opd1, opd2));}
	void comisd(const Xmm& opd1, const Mem64& opd2)		{PushBack(Instr(INSTR_COMISD, opd1, opd2));}

	void cvtdq2pd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTDQ2PD, opd1, opd2));}
	void cvtdq2pd(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(INSTR_CVTDQ2PD, opd1, opd2));}
	void cvtpd2dq(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTPD2DQ, opd1, opd2));}
	void cvtpd2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTPD2DQ, opd1, opd2));}
	void cvtpd2pi(const Mmx& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTPD2PI, opd1, opd2));}
	void cvtpd2pi(const Mmx& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTPD2PI, opd1, opd2));}
	void cvtpd2ps(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTPD2PS, opd1, opd2));}
	void cvtpd2ps(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTPD2PS, opd1, opd2));}
	void cvtpi2pd(const Xmm& opd1, const Mmx& opd2)		{PushBack(Instr(INSTR_CVTPI2PD, opd1, opd2));}
	void cvtpi2pd(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(INSTR_CVTPI2PD, opd1, opd2));}
	void cvtps2dq(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTPS2DQ, opd1, opd2));}
	void cvtps2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTPS2DQ, opd1, opd2));}
	void cvtdq2ps(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTDQ2PS, opd1, opd2));}
	void cvtdq2ps(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTDQ2PS, opd1, opd2));}
	void cvtps2pd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTPS2PD, opd1, opd2));}
	void cvtps2pd(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(INSTR_CVTPS2PD, opd1, opd2));}
	void cvtsd2ss(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTSD2SS, opd1, opd2));}
	void cvtsd2ss(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(INSTR_CVTSD2SS, opd1, opd2));}
	void cvtss2sd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(INSTR_CVTSS2SD, opd1, opd2));}
	void cvtss2sd(const Xmm& opd1, const Mem32& opd2)	{PushBack(Instr(INSTR_CVTSS2SD, opd1, opd2));}
	void cvttpd2dq(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(INSTR_CVTTPD2DQ, opd1, opd2));}
	void cvttpd2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTTPD2DQ, opd1, opd2));}
	void cvttpd2pi(const Mmx& opd1, const Xmm& opd2)	{PushBack(Instr(INSTR_CVTTPD2PI, opd1, opd2));}
	void cvttpd2pi(const Mmx& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTTPD2PI, opd1, opd2));}
	void cvttps2dq(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(INSTR_CVTTPS2DQ, opd1, opd2));}
	void cvttps2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(INSTR_CVTTPS2DQ, opd1, opd2));}

	// MOVDQA
	void movdqa(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_MOVDQA, opd1, opd2));}
	void movdqa(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_MOVDQA, opd1, opd2));}
	void movdqa(const Mem128& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_MOVDQA, opd1, opd2));}

	// MOVDQU
	void movdqu(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_MOVDQU, opd1, opd2));}
	void movdqu(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_MOVDQU, opd1, opd2));}
	void movdqu(const Mem128& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_MOVDQU, opd1, opd2));}

	// PABSB/PABSW/PABSD
	void pabsb(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PABSB, opd1, opd2));}
	void pabsb(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PABSB, opd1, opd2));}
	void pabsb(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PABSB, opd1, opd2));}
	void pabsb(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PABSB, opd1, opd2));}
	void pabsw(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PABSW, opd1, opd2));}
	void pabsw(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PABSW, opd1, opd2));}
	void pabsw(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PABSW, opd1, opd2));}
	void pabsw(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PABSW, opd1, opd2));}
	void pabsd(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PABSD, opd1, opd2));}
	void pabsd(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PABSD, opd1, opd2));}
	void pabsd(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PABSD, opd1, opd2));}
	void pabsd(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PABSD, opd1, opd2));}

	// PACKSSWB/PACKSSDW/PACKUSWB/PACKUSDW
	void packsswb(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PACKSSWB, opd1, opd2));}
	void packsswb(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PACKSSWB, opd1, opd2));}
	void packsswb(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PACKSSWB, opd1, opd2));}
	void packsswb(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PACKSSWB, opd1, opd2));}
	void packssdw(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PACKSSDW, opd1, opd2));}
	void packssdw(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PACKSSDW, opd1, opd2));}
	void packssdw(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PACKSSDW, opd1, opd2));}
	void packssdw(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PACKSSDW, opd1, opd2));}
	void packuswb(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PACKUSWB, opd1, opd2));}
	void packuswb(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PACKUSWB, opd1, opd2));}
	void packuswb(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PACKUSWB, opd1, opd2));}
	void packuswb(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PACKUSWB, opd1, opd2));}
	void packusdw(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PACKUSDW, opd1, opd2));}
	void packusdw(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PACKUSDW, opd1, opd2));}

	// PADDB/PADDW/PADDD
	void paddb(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PADDB, opd1, opd2));}
	void paddb(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PADDB, opd1, opd2));}
	void paddb(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PADDB, opd1, opd2));}
	void paddb(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PADDB, opd1, opd2));}
	void paddw(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PADDW, opd1, opd2));}
	void paddw(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PADDW, opd1, opd2));}
	void paddw(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PADDW, opd1, opd2));}
	void paddw(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PADDW, opd1, opd2));}
	void paddd(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PADDD, opd1, opd2));}
	void paddd(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PADDD, opd1, opd2));}
	void paddd(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PADDD, opd1, opd2));}
	void paddd(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PADDD, opd1, opd2));}

	// PXOR
	void pxor(const Mmx& opd1, const Mmx& opd2) {PushBack(Instr(INSTR_PXOR, opd1, opd2));}
	void pxor(const Mmx& opd1, const Mem64& opd2) {PushBack(Instr(INSTR_PXOR, opd1, opd2));}
	void pxor(const Xmm& opd1, const Xmm& opd2) {PushBack(Instr(INSTR_PXOR, opd1, opd2));}
	void pxor(const Xmm& opd1, const Mem128& opd2) {PushBack(Instr(INSTR_PXOR, opd1, opd2));}
};


#ifdef JITASM64
typedef Expr64_None Arg;
#else
typedef Expr32_None Arg;
#endif

namespace detail {

	template<int N> size_t AlignSize(size_t size) {
		return (size + N - 1) / N * N;
	}

	template<class T, int Size = sizeof(T)>
	struct ResultT {
	};

	template<class T>
	struct ResultT<T, 1> {
		Opd8 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd8& val) : val_(val) {}
		ResultT(uint8 imm) : val_(Imm8(imm)) {}
		void Store(Frontend& f) {
			if (!val_.IsReg() || val_.GetReg() != INVALID)
				f.mov(f.al, static_cast<Reg8&>(val_));
		}
	};

	template<class T>
	struct ResultT<T, 2> {
		Opd16 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd16& val) : val_(val) {}
		ResultT(uint16 imm) : val_(Imm16(imm)) {}
		void Store(Frontend& f) {
			if (!val_.IsReg() || val_.GetReg() != INVALID)
				f.mov(f.ax, static_cast<Reg16&>(val_));
		}
	};

	template<class T>
	struct ResultT<T, 4> {
		Opd32 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd32& val) : val_(val) {}
		ResultT(uint32 imm) : val_(Imm32(imm)) {}
		void Store(Frontend& f) {
			if (!val_.IsReg() || val_.GetReg() != INVALID)
				f.mov(f.eax, static_cast<Reg32&>(val_));
		}
	};

#ifdef JITASM64
	template<class T>
	struct ResultT<T, 8> {
		Opd64 value_;
		ResultT(const Opd64& val) : value_(val) {}
		ResultT(uint64 imm) : value_(Imm64(imm)) {}
	};
#endif

	template<>
	struct ResultT<float, sizeof(float)> {
	};

	template<>
	struct ResultT<double, sizeof(double)> {
	};


	/// cdecl function base class
	template<class FuncPtr>
	struct Function : Frontend
	{
		Arg Arg1() { return Arg(zbp + 8); }
		template<class A1> Arg Arg2() { return Arg1() + AlignSize<sizeof(void*)>(sizeof(A1)); }
		template<class A1, class A2> Arg Arg3() { return Arg2() + AlignSize<sizeof(void*)>(sizeof(A2)); }

		operator FuncPtr() { return (FuncPtr)GetCode(); }
	};

}	// namespace detail

/// cdecl function which has no argument
template<class R>
struct function0_cdecl : detail::Function<R (__cdecl *)()>
{
	typedef detail::ResultT<R> Result;
	virtual Result main() { return Result(); }
	void naked_main() {
		Prolog(0);
		main().Store(*this);
		Epilog();
	}
};

template<>
struct function0_cdecl<void> : detail::Function<void (__cdecl *)()>
{
	virtual void main() {}
	void naked_main()	{
		Prolog(0);
		main();
		Epilog();
	}
};

/// cdecl function which has 1 argument
template<class R, class A1>
struct function1_cdecl : detail::Function<R (__cdecl *)(A1)>
{
	typedef detail::ResultT<R> Result;
	virtual Result main(Arg a1) { return Result(); }
	void naked_main() {
		Prolog(0);
		main(Arg1()).Store(*this);
		Epilog();
	}
};

template<class A1>
struct function1_cdecl<void, A1> : detail::Function<void (__cdecl *)(A1)>
{
	virtual void main(Arg a1) {}
	void naked_main() {
		Prolog(0);
		main(Arg1());
		Epilog();
	}
};

/// cdecl function which has 2 arguments
template<class R, class A1, class A2>
struct function2_cdecl : detail::Function<R (__cdecl *)(A1, A2)>
{
	typedef detail::ResultT<R> Result;
	virtual Result main(Arg a1, Arg a2) { return Result(); }
	void naked_main() {
		Prolog(0);
		main(Arg1(), Arg2<A1>()).Store(*this);
		Epilog();
	}
};

template<class A1, class A2>
struct function2_cdecl<void, A1, A2> : detail::Function<void (__cdecl *)(A1, A2)>
{
	virtual void main(Arg a1, Arg a2) {}
	void naked_main() {
		Prolog(0);
		main(Arg1(), Arg2<A1>());
		Epilog();
	}
};

template<class R> struct function0 : function0_cdecl<R> {};
template<class R, class A1> struct function1 : function1_cdecl<R, A1> {};
template<class R, class A1, class A2> struct function2 : function2_cdecl<R, A1, A2> {};

}	// namespace jitasm
#endif	// #ifndef JITASM_H
