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

//----------------------------------------
// Operand
//----------------------------------------
enum OpdType
{
	O_TYPE_NONE,
	O_TYPE_REG,
	O_TYPE_MEM,
	O_TYPE_IMM,
};

enum OpdSize
{
	O_SIZE_8,
	O_SIZE_16,
	O_SIZE_32,
	O_SIZE_64,
	O_SIZE_80,
	O_SIZE_128,
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
	Opd() : opdtype_(O_TYPE_NONE) {}
	// REG
	explicit Opd(OpdSize opdsize, RegID reg) : opdtype_(O_TYPE_REG), opdsize_(opdsize), reg_(reg) {}
	// MEM
	Opd(OpdSize opdsize, OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
		: opdtype_(O_TYPE_MEM), opdsize_(opdsize), addrsize_(addrsize), base_(base), index_(index), scale_(scale), disp_(disp) {}
protected:
	// IMM
	explicit Opd(sint64 imm) : opdtype_(O_TYPE_IMM), imm_(imm)
	{
		if (detail::IsInt8(imm)) opdsize_ = O_SIZE_8;
		else if (detail::IsInt16(imm)) opdsize_ = O_SIZE_16;
		else if (detail::IsInt32(imm)) opdsize_ = O_SIZE_32;
#ifdef JITASM64
		else opdsize_ = O_SIZE_64;
#else
		else ASSERT(0);
#endif
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
	Mem64 ToMem64() {return Mem64(O_SIZE_64, INVALID, INVALID, 0, offset_);}
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
inline Expr32_None operator-(const Reg32& lhs, sint64 rhs) {return lhs + -rhs;}
inline Expr32_None operator+(const Expr32_None& lhs, sint64 rhs) {return Expr32_None(lhs.reg_, lhs.disp_ + rhs);}
inline Expr32_None operator+(sint64 lhs, const Expr32_None& rhs) {return rhs + lhs;}
inline Expr32_None operator-(const Expr32_None& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr32_BI operator-(const Expr32_BI& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr32_SI operator-(const Expr32_SI& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr32_SIB operator-(const Expr32_SIB& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr64_None operator-(const Reg64& lhs, sint64 rhs) {return lhs + -rhs;}
inline Expr64_None operator+(const Expr64_None& lhs, sint64 rhs) {return Expr64_None(lhs.reg_, lhs.disp_ + rhs);}
inline Expr64_None operator+(sint64 lhs, const Expr64_None& rhs) {return rhs + lhs;}
inline Expr64_None operator-(const Expr64_None& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr64_BI operator-(const Expr64_BI& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr64_SI operator-(const Expr64_SI& lhs, sint64 rhs) {return lhs + -rhs;}

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
inline Expr64_SIB operator-(const Expr64_SIB& lhs, sint64 rhs) {return lhs + -rhs;}
#endif

template<typename OpdN>
struct AddressingPtr
{
	// 32bit-Addressing
	MemT<OpdN> operator[](const Expr32_None& obj) {return MemT<OpdN>(O_SIZE_32, obj.reg_, INVALID, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr32_BI& obj) {return MemT<OpdN>(O_SIZE_32, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr32_SI& obj) {return MemT<OpdN>(O_SIZE_32, INVALID, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Expr32_SIB& obj) {return MemT<OpdN>(O_SIZE_32, obj.base_, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](sint32 disp) {return MemT<OpdN>(O_SIZE_64, INVALID, INVALID, 0, disp);}
	MemT<OpdN> operator[](uint32 disp) {return MemT<OpdN>(O_SIZE_64, INVALID, INVALID, 0, (sint32) disp);}
#ifdef JITASM64
	// 64bit-Addressing
	MemT<OpdN> operator[](const Expr64_None& obj) {return MemT<OpdN>(O_SIZE_64, obj.reg_, INVALID, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr64_BI& obj) {return MemT<OpdN>(O_SIZE_64, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Expr64_SI& obj) {return MemT<OpdN>(O_SIZE_64, INVALID, obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Expr64_SIB& obj) {return MemT<OpdN>(O_SIZE_64, obj.base_, obj.index_, obj.scale_, obj.disp_);}
	MemOffset64 operator[](sint64 offset) {return MemOffset64(offset);}
	MemOffset64 operator[](uint64 offset) {return MemOffset64((sint64) offset);}
#endif
};

//----------------------------------------
// Instruction
//----------------------------------------
enum InstrID
{
	I_ADC, I_ADD, I_AND, I_CALL, I_CMP, I_DEC, I_INC,
	I_JMP, I_JA, I_JAE, I_JB, I_JBE, I_JCXZ, I_JECXZ, I_JRCXZ, I_JE,
	I_JG, I_JGE, I_JL, I_JLE, I_JNE, I_JNO, I_JNP, I_JNS, I_JO, I_JP, I_JS,
	I_LEA, I_LEAVE, I_MOV, I_MOVZX, I_NOP, I_OR, I_POP, I_PUSH,
	I_RET, I_SAR, I_SHL, I_SHR, I_SBB, I_SUB, I_TEST, I_XCHG, I_XOR,

	I_FLD,

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

	void EncodeCALL(const Opd& opd)
	{
		if (opd.IsReg()) {
			if (opd.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
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
		if (opd.IsMem() && opd.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
		if (opd.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
		EncodeRexWRXB(opd);
#else
		if (opd.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
#endif

		uint8 w = opd.GetSize() != O_SIZE_8 ? 1 : 0;

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
		if (opd.GetSize() <= O_SIZE_8) db(0x70 | tttn), db(opd.GetImm());
		else if (opd.GetSize() <= O_SIZE_32) db(0x0F), db(0x80 | tttn), dd(opd.GetImm());
		else ASSERT(0);
	}

	void EncodeJMP(const Opd& opd)
	{
		if (opd.GetSize() <= O_SIZE_8) db(0xEB), db(opd.GetImm());
		else if (opd.GetSize() <= O_SIZE_32) db(0xE9), dd(opd.GetImm());
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

	void EncodeMOV(const Opd& opd1, const Opd& opd2)
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

			uint8 w = reg.GetSize() != O_SIZE_8 ? 1 : 0;
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

	void EncodePOP(uint8 opcode1, uint8 opcode2, uint8 digit, const Opd& opd)
	{
		if (opd.IsImm()) {
			if (opd.GetSize() == O_SIZE_8) {	// sign-extention
				db(0x6A);
				db(opd.GetImm());
			} else {
				db(0x68);
				dd(opd.GetImm());
			}
		} else {
#ifdef JITASM64
			if (opd.IsMem() && opd.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
			if (opd.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
			EncodeRexRXB(opd);
#else
			if (opd.GetSize() == O_SIZE_16) EncodeOperandSizePrefix();
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

	void EncodeTEST(const Opd& opd1, const Opd& opd2)
	{
		uint8 w = opd1.GetSize() != O_SIZE_8 ? 1 : 0;
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
			ASSERT(opd2.IsReg());
			const Opd& reg = opd1.IsReg() ? opd1 : opd2;
			const Opd& r_m = opd1.IsReg() ? opd2 : opd1;

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

	void EncodeXCHG(const Opd& opd1, const Opd& opd2)
	{
		const Opd& reg = opd1.IsReg() && (opd2.GetSize() == O_SIZE_8 || !opd2.IsReg() || opd2.GetReg() != EAX) ? opd1 : opd2;
		const Opd& r_m = &reg == &opd1 ? opd2 : opd1;

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

	void EncodeFLD(const Opd& opd)
	{
		if (opd.IsReg()) {
			db(0xD9);
			db(0xC0 | opd.GetReg());
		} else {
#ifdef JITASM64
			if (opd.IsMem() && opd.GetAddressSize() != O_SIZE_64) EncodeAddressSizePrefix();
#endif
			int digit = 0;
			if (opd.GetSize() == O_SIZE_32) db(0xD9), digit = 0;
			else if (opd.GetSize() == O_SIZE_64) db(0xDD), digit = 0;
			else if (opd.GetSize() == O_SIZE_80) db(0xDB), digit = 5;
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
		if ((opd1.IsReg() && opd1.GetSize() == O_SIZE_128) ||
			(opd2.IsReg() && opd2.GetSize() == O_SIZE_128)) {
			EncodeSSE2(prefix, opcode2, opd1, opd2);
		}
		else {
			EncodeMMX(opcode2, opd1, opd2);
		}
	}

	void EncodeMMXorSSE2(uint8 prefix, uint8 opcode2, uint8 opcode3, const Opd& opd1, const Opd& opd2)
	{
		if ((opd1.IsReg() && opd1.GetSize() == O_SIZE_128) ||
			(opd2.IsReg() && opd2.GetSize() == O_SIZE_128)) {
			EncodeSSE2(prefix, opcode2, opcode3, opd1, opd2);
		}
		else {
			EncodeMMX(opcode2, opcode3, opd1, opd2);
		}
	}

	void EncodeCLFLUSH(const Opd& opd)
	{
		ASSERT(opd.IsMem() && opd.GetSize() == O_SIZE_8);
		db(0x0F);
		db(0xAE);
		EncodeModRM(7, opd);
	}

	void EncodeMOVQ(const Opd& opd1, const Opd& opd2)
	{
		if ((opd1.IsReg() ? opd1 : opd2).GetSize() == O_SIZE_64) {
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
		case I_ADC:			EncodeADD(0x10, opd1, opd2); break;
		case I_ADD:			EncodeADD(0x00, opd1, opd2); break;
		case I_AND:			EncodeADD(0x20, opd1, opd2); break;
		case I_CALL:		EncodeCALL(opd1); break;
		case I_CMP:			EncodeADD(0x38, opd1, opd2); break;
		case I_DEC:			EncodeINC(0x48, 1, opd1); break;
		case I_INC:			EncodeINC(0x40, 0, opd1); break;
		case I_JMP:			EncodeJMP(opd1); break;
		case I_JA:			EncodeJCC(0x7, opd1); break;
		case I_JAE:			EncodeJCC(0x3, opd1); break;
		case I_JB:			EncodeJCC(0x2, opd1); break;
		case I_JBE:			EncodeJCC(0x6, opd1); break;
#ifdef JITASM64
		case I_JECXZ:		EncodeAddressSizePrefix(); db(0xE3); db(opd1.GetImm()); break;
		case I_JRCXZ:		db(0xE3); db(opd1.GetImm()); break;
#else
		case I_JECXZ:		db(0xE3); db(opd1.GetImm()); break;
		case I_JCXZ:		EncodeAddressSizePrefix(); db(0xE3); db(opd1.GetImm()); break;
#endif
		case I_JE:			EncodeJCC(0x4, opd1); break;
		case I_JG:			EncodeJCC(0xF, opd1); break;
		case I_JGE:			EncodeJCC(0xD, opd1); break;
		case I_JL:			EncodeJCC(0xC, opd1); break;
		case I_JLE:			EncodeJCC(0xE, opd1); break;
		case I_JNE:			EncodeJCC(0x5, opd1); break;
		case I_JNO:			EncodeJCC(0x1, opd1); break;
		case I_JNP:			EncodeJCC(0xB, opd1); break;
		case I_JNS:			EncodeJCC(0x9, opd1); break;
		case I_JO:			EncodeJCC(0x0, opd1); break;
		case I_JP:			EncodeJCC(0xA, opd1); break;
		case I_JS:			EncodeJCC(0x8, opd1); break;
		case I_LEA:			EncodeLEA(opd1, opd2); break;
		case I_LEAVE:		db(0xC9); break;
		case I_MOV:			EncodeMOV(opd1, opd2); break;
		case I_MOVZX:		EncodeMOVZX(opd1, opd2); break;
		case I_NOP:			db(0x90); break;
		case I_OR:			EncodeADD(0x08, opd1, opd2); break;
		case I_POP:			EncodePOP(0x58, 0x8F, 0, opd1); break;
		case I_PUSH:		EncodePOP(0x50, 0xFF, 6, opd1); break;
		case I_RET:			if (opd1.IsNone()) db(0xC3); else db(0xC2), dw(opd1.GetImm()); break;
		case I_SAR:			EncodeShift(7, opd1, opd2); break;
		case I_SHL:			EncodeShift(4, opd1, opd2); break;
		case I_SHR:			EncodeShift(5, opd1, opd2); break;
		case I_SBB:			EncodeADD(0x18, opd1, opd2); break;
		case I_SUB:			EncodeADD(0x28, opd1, opd2); break;
		case I_TEST:		EncodeTEST(opd1, opd2); break;
		case I_XCHG:		EncodeXCHG(opd1, opd2); break;
		case I_XOR:			EncodeADD(0x30, opd1, opd2); break;

		// x87 Floating-Point Instructions
		case I_FLD:			EncodeFLD(opd1); break;

		// MMX/SSE/SSE2 Instructions
		case I_ADDPD:		EncodeSSE2(0x66, 0x58, opd1, opd2); break;
		case I_ADDSD:		EncodeSSE2(0xF2, 0x58, opd1, opd2); break;
		case I_ANDPD:		EncodeSSE2(0x66, 0x54, opd1, opd2); break;
		case I_ANDNPD:		EncodeSSE2(0x66, 0x55, opd1, opd2); break;
		case I_CLFLUSH:		EncodeCLFLUSH(opd1); break;
		case I_CMPPD:		EncodeSSE2(0x66, 0xC2, opd1, opd2); db(opd3.GetImm()); break;
		case I_CMPPS:		EncodeSSE(0xC2, opd1, opd2); db(opd3.GetImm()); break;
		case I_CMPSD:		EncodeSSE2(0xF2, 0xC2, opd1, opd2); db(opd3.GetImm()); break;
		case I_COMISD:		EncodeSSE2(0x66, 0x2F, opd1, opd2); break;
		case I_CVTDQ2PD:	EncodeSSE2(0xF3, 0xE6, opd1, opd2); break;
		case I_CVTPD2DQ:	EncodeSSE2(0xF2, 0xE6, opd1, opd2); break;
		case I_CVTPD2PI:	EncodeSSE2(0x66, 0x2D, opd1, opd2); break;
		case I_CVTPD2PS:	EncodeSSE2(0x66, 0x5A, opd1, opd2); break;
		case I_CVTPI2PD:	EncodeSSE2(0x66, 0x2A, opd1, opd2); break;
		case I_CVTPS2DQ:	EncodeSSE2(0x66, 0x5B, opd1, opd2); break;
		case I_CVTDQ2PS:	EncodeSSE(0x5B, opd1, opd2); break;
		case I_CVTPS2PD:	EncodeSSE(0x5A, opd1, opd2); break;	// SSE2!!!
		//case I_CVTSD2SI:
		case I_CVTSD2SS:	EncodeSSE2(0xF2, 0x5A, opd1, opd2); break;
		//case I_CVTSI2SD:
		case I_CVTSS2SD:	EncodeSSE2(0xF3, 0x5A, opd1, opd2); break;
		case I_CVTTPD2DQ:	EncodeSSE2(0x66, 0xE6, opd1, opd2); break;
		case I_CVTTPD2PI:	EncodeSSE2(0x66, 0x2C, opd1, opd2); break;
		case I_CVTTPS2DQ:	EncodeSSE2(0xF3, 0x5B, opd1, opd2); break;
		//case I_CVTTSD2SI:
		case I_DIVPD:		EncodeSSE2(0x66, 0x5E, opd1, opd2); break;
		case I_DIVSD:		EncodeSSE2(0xF2, 0x5E, opd1, opd2); break;
		//case I_LFENCE:
		case I_MASKMOVDQU:	EncodeSSE2(0x66, 0xF7, opd1, opd2); break;
		case I_MAXPD:		EncodeSSE2(0x66, 0x5F, opd1, opd2); break;
		case I_MAXSD:		EncodeSSE2(0xF2, 0x5F, opd1, opd2); break;
		//case I_MFENCE:
		case I_MINPD:		EncodeSSE2(0x66, 0x5D, opd1, opd2); break;
		case I_MINSD:		EncodeSSE2(0xF2, 0x5D, opd1, opd2); break;
		case I_MOVAPD:		EncodeSSE2(0x66, 0x28 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		//case I_MOVD:
		case I_MOVDQ2Q:		EncodeSSE2(0xF2, 0xD6, opd1, opd2); break;
		case I_MOVDQA:		EncodeSSE2(0x66, 0x6F | (opd1.IsReg() ? 0 : 0x10), opd1, opd2); break;
		case I_MOVDQU:		EncodeSSE2(0xF3, 0x6F | (opd1.IsReg() ? 0 : 0x10), opd1, opd2); break;
		case I_MOVHPD:		EncodeSSE2(0x66, 0x16 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case I_MOVLPD:		EncodeSSE2(0x66, 0x12 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case I_MOVMSKPD:	EncodeSSE2(0x66, 0x50, opd1, opd2); break;
		case I_MOVNTPD:		EncodeSSE2(0x66, 0x2B, opd1, opd2); break;
		case I_MOVNTDQ:		EncodeSSE2(0x66, 0xE7, opd1, opd2); break;
		//case I_MOVNTI:
		case I_MOVQ:		EncodeMOVQ(opd1, opd2); break;
		case I_MOVQ2DQ:		EncodeSSE2(0xF3, 0xD6, opd1, opd2); break;
		case I_MOVSD:		EncodeSSE2(0xF2, 0x10 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case I_MOVSS:		EncodeSSE2(0xF3, 0x10 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;	// SSE
		case I_MOVUPD:		EncodeSSE2(0x66, 0x10 | (opd1.IsReg() ? 0 : 0x01), opd1, opd2); break;
		case I_MULPD:		EncodeSSE2(0x66, 0x59, opd1, opd2); break;
		case I_MULSD:		EncodeSSE2(0xF2, 0x59, opd1, opd2); break;
		case I_ORPD:		EncodeSSE2(0x66, 0x56, opd1, opd2); break;
		case I_PABSB:		EncodeMMXorSSE2(0x66, 0x38, 0x1C, opd1, opd2); break;	// SSSE3
		case I_PABSW:		EncodeMMXorSSE2(0x66, 0x38, 0x1D, opd1, opd2); break;	// SSSE3
		case I_PABSD:		EncodeMMXorSSE2(0x66, 0x38, 0x1E, opd1, opd2); break;	// SSSE3
		case I_PACKSSWB:	EncodeMMXorSSE2(0x66, 0x63, opd1, opd2); break;
		case I_PACKSSDW:	EncodeMMXorSSE2(0x66, 0x6B, opd1, opd2); break;
		case I_PACKUSWB:	EncodeMMXorSSE2(0x66, 0x67, opd1, opd2); break;
		case I_PACKUSDW:	EncodeSSE2(0x66, 0x38, 0x2B, opd1, opd2); break;		// SSE 4.1
		case I_PADDB:		EncodeMMXorSSE2(0x66, 0xFC, opd1, opd2); break;
		case I_PADDW:		EncodeMMXorSSE2(0x66, 0xFD, opd1, opd2); break;
		case I_PADDD:		EncodeMMXorSSE2(0x66, 0xFE, opd1, opd2); break;
		case I_PADDQ:		EncodeMMXorSSE2(0x66, 0xD4, opd1, opd2); break;
		case I_PADDSB:		EncodeMMXorSSE2(0x66, 0xEC, opd1, opd2); break;
		case I_PADDSW:		EncodeMMXorSSE2(0x66, 0xED, opd1, opd2); break;
		case I_PADDUSB:		EncodeMMXorSSE2(0x66, 0xDC, opd1, opd2); break;
		case I_PADDUSW:		EncodeMMXorSSE2(0x66, 0xDD, opd1, opd2); break;
		case I_PALIGNR:		EncodeMMXorSSE2(0x66, 0x3A, 0x0F, opd1, opd2); break;	// SSSE3
		case I_PAND:		EncodeMMXorSSE2(0x66, 0xDB, opd1, opd2); break;
		case I_PANDN:		EncodeMMXorSSE2(0x66, 0xDF, opd1, opd2); break;
		case I_PAUSE:		break;
		case I_PAVGB:		EncodeMMXorSSE2(0x66, 0xE0, opd1, opd2); break;
		case I_PAVGW:		EncodeMMXorSSE2(0x66, 0xE3, opd1, opd2); break;
		case I_PCMPEQB:		EncodeMMXorSSE2(0x66, 0x74, opd1, opd2); break;
		case I_PCMPEQW:		EncodeMMXorSSE2(0x66, 0x75, opd1, opd2); break;
		case I_PCMPEQD:		EncodeMMXorSSE2(0x66, 0x76, opd1, opd2); break;
		case I_PCMPEQQ:		EncodeSSE2(0x66, 0x38, 0x29, opd1, opd2); break;
		case I_PCMPGTB:		EncodeMMXorSSE2(0x66, 0x64, opd1, opd2); break;
		case I_PCMPGTW:		EncodeMMXorSSE2(0x66, 0x65, opd1, opd2); break;
		case I_PCMPGTD:		EncodeMMXorSSE2(0x66, 0x66, opd1, opd2); break;
		case I_PCMPGTQ:		EncodeSSE2(0x66, 0x38, 0x37, opd1, opd2); break;
		//case I_PEXTRW:
		//case I_PINSRW:
		case I_PMADDWD:		EncodeMMXorSSE2(0x66, 0xF5, opd1, opd2); break;
		case I_PMAXSW:		EncodeMMXorSSE2(0x66, 0xEE, opd1, opd2); break;
		case I_PMAXUB:		EncodeMMXorSSE2(0x66, 0xDE, opd1, opd2); break;
		case I_PMINSW:		EncodeMMXorSSE2(0x66, 0xEA, opd1, opd2); break;
		case I_PMINUB:		EncodeMMXorSSE2(0x66, 0xDA, opd1, opd2); break;
		//case I_PMOVMSKB:
		case I_PMULHUW:		EncodeMMXorSSE2(0x66, 0xE4, opd1, opd2); break;
		case I_PMULHW:		EncodeMMXorSSE2(0x66, 0xE5, opd1, opd2); break;
		case I_PMULLW:		EncodeMMXorSSE2(0x66, 0xD5, opd1, opd2); break;
		case I_PMULUDQ:		EncodeMMXorSSE2(0x66, 0xF4, opd1, opd2); break;
		case I_POR:			EncodeMMXorSSE2(0x66, 0xEB, opd1, opd2); break;
		case I_PSADBW:		EncodeMMXorSSE2(0x66, 0xF6, opd1, opd2); break;
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
		case I_PSUBB:		EncodeMMXorSSE2(0x66, 0xF8, opd1, opd2); break;
		case I_PSUBW:		EncodeMMXorSSE2(0x66, 0xF9, opd1, opd2); break;
		case I_PSUBD:		EncodeMMXorSSE2(0x66, 0xFA, opd1, opd2); break;
		case I_PSUBQ:		EncodeMMXorSSE2(0x66, 0xFB, opd1, opd2); break;
		case I_PSUBSB:		EncodeMMXorSSE2(0x66, 0xE8, opd1, opd2); break;
		case I_PSUBSW:		EncodeMMXorSSE2(0x66, 0xE9, opd1, opd2); break;
		case I_PSUBUSB:		EncodeMMXorSSE2(0x66, 0xD8, opd1, opd2); break;
		case I_PSUBUSW:		EncodeMMXorSSE2(0x66, 0xD9, opd1, opd2); break;
		case I_PUNPCKHBW:	EncodeMMXorSSE2(0x66, 0x68, opd1, opd2); break;
		case I_PUNPCKHWD:	EncodeMMXorSSE2(0x66, 0x69, opd1, opd2); break;
		case I_PUNPCKHDQ:	EncodeMMXorSSE2(0x66, 0x6A, opd1, opd2); break;
		case I_PUNPCKHQDQ:	EncodeSSE2(0x66, 0x6D, opd1, opd2); break;
		case I_PUNPCKLBW:	EncodeMMXorSSE2(0x66, 0x60, opd1, opd2); break;
		case I_PUNPCKLWD:	EncodeMMXorSSE2(0x66, 0x61, opd1, opd2); break;
		case I_PUNPCKLDQ:	EncodeMMXorSSE2(0x66, 0x62, opd1, opd2); break;
		case I_PUNPCKLQDQ:	EncodeSSE2(0x66, 0x6C, opd1, opd2); break;
		case I_PXOR:		EncodeMMXorSSE2(0x66, 0xEF, opd1, opd2); break;
		//case I_SHUFPD:
		case I_SQRTPD:		EncodeSSE2(0x66, 0x51, opd1, opd2); break;
		case I_SQRTSD:		EncodeSSE2(0xF2, 0x51, opd1, opd2); break;
		case I_SUBPD:		EncodeSSE2(0x66, 0x5C, opd1, opd2); break;
		case I_SUBSD:		EncodeSSE2(0xF2, 0x5C, opd1, opd2); break;
		case I_UCOMISD:		EncodeSSE2(0x66, 0x2E, opd1, opd2); break;
		case I_UNPCKHPD:	EncodeSSE2(0x66, 0x15, opd1, opd2); break;
		case I_UNPCKLPD:	EncodeSSE2(0x66, 0x14, opd1, opd2); break;
		case I_XORPD:		EncodeSSE2(0x66, 0x57, opd1, opd2); break;

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
	void label(const std::string& label_name)
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
	void adc(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_ADC, opd1, imm));}
	void adc(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_ADC, opd1, imm));}
	void adc(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_ADC, opd1, imm));}
	void adc(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_ADC, opd1, imm));}
	void adc(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_ADC, opd1, imm));}
	void adc(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_ADC, opd1, imm));}
#ifdef JITASM64
	void adc(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_ADC, opd1, opd2));}
	void adc(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_ADC, opd1, imm));}
	void adc(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_ADC, opd1, imm));}
#endif

	// ADD
	void add(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_ADD, opd1, imm));}
	void add(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_ADD, opd1, imm));}
	void add(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_ADD, opd1, imm));}
	void add(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_ADD, opd1, imm));}
	void add(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_ADD, opd1, imm));}
	void add(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_ADD, opd1, imm));}
#ifdef JITASM64
	void add(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_ADD, opd1, opd2));}
	void add(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_ADD, opd1, imm));}
	void add(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_ADD, opd1, imm));}
#endif

	// AND
	void and(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_AND, opd1, imm));}
	void and(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_AND, opd1, imm));}
	void and(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_AND, opd1, imm));}
	void and(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_AND, opd1, imm));}
	void and(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_AND, opd1, imm));}
	void and(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_AND, opd1, imm));}
#ifdef JITASM64
	void and(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_AND, opd1, opd2));}
	void and(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_AND, opd1, imm));}
	void and(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_AND, opd1, imm));}
#endif

	// CALL
#ifndef JITASM64
	void call(const Reg16& opd)	{PushBack(Instr(I_CALL, opd));}
	void call(const Reg32& opd)	{PushBack(Instr(I_CALL, opd));}
#else
	void call(const Reg64& opd)	{PushBack(Instr(I_CALL, opd));}
#endif

	// CMP
	void cmp(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_CMP, opd1, imm));}
	void cmp(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_CMP, opd1, imm));}
	void cmp(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_CMP, opd1, imm));}
	void cmp(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_CMP, opd1, imm));}
	void cmp(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_CMP, opd1, imm));}
	void cmp(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_CMP, opd1, imm));}
#ifdef JITASM64
	void cmp(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_CMP, opd1, opd2));}
	void cmp(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_CMP, opd1, imm));}
	void cmp(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_CMP, opd1, imm));}
#endif

	// DEC
	void dec(const Reg8& opd)	{PushBack(Instr(I_DEC, opd));}
	void dec(const Mem8& opd)	{PushBack(Instr(I_DEC, opd));}
	void dec(const Reg16& opd)	{PushBack(Instr(I_DEC, opd));}
	void dec(const Mem16& opd)	{PushBack(Instr(I_DEC, opd));}
	void dec(const Reg32& opd)	{PushBack(Instr(I_DEC, opd));}
	void dec(const Mem32& opd)	{PushBack(Instr(I_DEC, opd));}
#ifdef JITASM64
	void dec(const Reg64& opd)	{PushBack(Instr(I_DEC, opd));}
	void dec(const Mem64& opd)	{PushBack(Instr(I_DEC, opd));}
#endif

	// INC
	void inc(const Reg8& opd)	{PushBack(Instr(I_INC, opd));}
	void inc(const Mem8& opd)	{PushBack(Instr(I_INC, opd));}
	void inc(const Reg16& opd)	{PushBack(Instr(I_INC, opd));}
	void inc(const Mem16& opd)	{PushBack(Instr(I_INC, opd));}
	void inc(const Reg32& opd)	{PushBack(Instr(I_INC, opd));}
	void inc(const Mem32& opd)	{PushBack(Instr(I_INC, opd));}
#ifdef JITASM64
	void inc(const Reg64& opd)	{PushBack(Instr(I_INC, opd));}
	void inc(const Mem64& opd)	{PushBack(Instr(I_INC, opd));}
#endif

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
	void lea(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_LEA, opd1, opd2));}
	void lea(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_LEA, opd1, opd2));}
#ifdef JITASM64
	void lea(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_LEA, opd1, opd2));}
#endif

	// LEAVE
	void leave()	{PushBack(Instr(I_LEAVE));}

	// MOV
	void mov(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_MOV, opd1, imm));}
#ifdef JITASM64
	void mov(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_MOV, opd1, opd2));}
	void mov(const Reg64& opd1, const Imm64& imm)	{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Mem64& opd1, const Imm64& imm)	{PushBack(Instr(I_MOV, opd1, imm));}
	void mov(const Rax& opd1, MemOffset64& opd2)	{PushBack(Instr(I_MOV, opd1, opd2.ToMem64()));}
#endif
 
	// MOVZX
	void movzx(const Reg16& opd1, const Opd8& opd2)		{PushBack(Instr(I_MOVZX, opd1, opd2));}
	void movzx(const Reg32& opd1, const Opd8& opd2)		{PushBack(Instr(I_MOVZX, opd1, opd2));}
	void movzx(const Reg32& opd1, const Opd16& opd2)	{PushBack(Instr(I_MOVZX, opd1, opd2));}
#ifdef JITASM64
	void movzx(const Reg64& opd1, const Opd8& opd2)		{PushBack(Instr(I_MOVZX, opd1, opd2));}
	void movzx(const Reg64& opd1, const Opd16& opd2)	{PushBack(Instr(I_MOVZX, opd1, opd2));}
#endif

	// NOP
	void nop()	{PushBack(Instr(I_NOP));}

	// OR
	void or(const Reg8& opd1, const Reg8& opd2)		{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg8& opd1, const Mem8& opd2)		{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Mem8& opd1, const Reg8& opd2)		{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_OR, opd1, imm));}
	void or(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_OR, opd1, imm));}
	void or(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_OR, opd1, imm));}
	void or(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_OR, opd1, imm));}
	void or(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_OR, opd1, imm));}
	void or(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_OR, opd1, imm));}
#ifdef JITASM64
	void or(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_OR, opd1, opd2));}
	void or(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_OR, opd1, imm));}
	void or(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_OR, opd1, imm));}
#endif

	// POP
	void pop(const Reg16& opd)	{PushBack(Instr(I_POP, opd));}
	void pop(const Mem16& opd)	{PushBack(Instr(I_POP, opd));}
#ifdef JITASM64
	void pop(const Reg64& opd)	{PushBack(Instr(I_POP, opd));}
	void pop(const Mem64& opd)	{PushBack(Instr(I_POP, opd));}
#else
	void pop(const Reg32& opd)	{PushBack(Instr(I_POP, opd));}
	void pop(const Mem32& opd)	{PushBack(Instr(I_POP, opd));}
#endif

	// PUSH
	void push(const Reg16& opd)	{PushBack(Instr(I_PUSH, opd));}
	void push(const Mem16& opd)	{PushBack(Instr(I_PUSH, opd));}
#ifdef JITASM64
	void push(const Reg64& opd)	{PushBack(Instr(I_PUSH, opd));}
	void push(const Mem64& opd)	{PushBack(Instr(I_PUSH, opd));}
#else
	void push(const Reg32& opd)	{PushBack(Instr(I_PUSH, opd));}
	void push(const Mem32& opd)	{PushBack(Instr(I_PUSH, opd));}
#endif
	void push(const Imm64& imm)	{PushBack(Instr(I_PUSH, imm));}

	// RET
	void ret()					{PushBack(Instr(I_RET));}
	void ret(const Imm16& imm)	{PushBack(Instr(I_RET, imm));}
 
	// SAL/SAR/SHL/SHR
	void sal(const Reg8& opd1, const Imm8& imm)		{shl(opd1, imm);}
	void sal(const Mem8& opd1, const Imm8& imm)		{shl(opd1, imm);}
	void sar(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_SAR, opd1, imm));}
	void sar(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_SAR, opd1, imm));}
	void shl(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_SHL, opd1, imm));}
	void shl(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_SHL, opd1, imm));}
	void shr(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_SHR, opd1, imm));}
	void shr(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_SHR, opd1, imm));}
	void sal(const Reg16& opd1, const Imm8& imm)	{shl(opd1, imm);}
	void sal(const Mem16& opd1, const Imm8& imm)	{shl(opd1, imm);}
	void sar(const Reg16& opd1, const Imm8& imm)	{PushBack(Instr(I_SAR, opd1, imm));}
	void sar(const Mem16& opd1, const Imm8& imm)	{PushBack(Instr(I_SAR, opd1, imm));}
	void shl(const Reg16& opd1, const Imm8& imm)	{PushBack(Instr(I_SHL, opd1, imm));}
	void shl(const Mem16& opd1, const Imm8& imm)	{PushBack(Instr(I_SHL, opd1, imm));}
	void shr(const Reg16& opd1, const Imm8& imm)	{PushBack(Instr(I_SHR, opd1, imm));}
	void shr(const Mem16& opd1, const Imm8& imm)	{PushBack(Instr(I_SHR, opd1, imm));}
	void sal(const Reg32& opd1, const Imm8& imm)	{shl(opd1, imm);}
	void sal(const Mem32& opd1, const Imm8& imm)	{shl(opd1, imm);}
	void sar(const Reg32& opd1, const Imm8& imm)	{PushBack(Instr(I_SAR, opd1, imm));}
	void sar(const Mem32& opd1, const Imm8& imm)	{PushBack(Instr(I_SAR, opd1, imm));}
	void shl(const Reg32& opd1, const Imm8& imm)	{PushBack(Instr(I_SHL, opd1, imm));}
	void shl(const Mem32& opd1, const Imm8& imm)	{PushBack(Instr(I_SHL, opd1, imm));}
	void shr(const Reg32& opd1, const Imm8& imm)	{PushBack(Instr(I_SHR, opd1, imm));}
	void shr(const Mem32& opd1, const Imm8& imm)	{PushBack(Instr(I_SHR, opd1, imm));}
#ifdef JITASM64
	void sal(const Reg64& opd1, const Imm8& imm)	{shl(opd1, imm);}
	void sal(const Mem64& opd1, const Imm8& imm)	{shl(opd1, imm);}
	void sar(const Reg64& opd1, const Imm8& imm)	{PushBack(Instr(I_SAR, opd1, imm));}
	void sar(const Mem64& opd1, const Imm8& imm)	{PushBack(Instr(I_SAR, opd1, imm));}
	void shl(const Reg64& opd1, const Imm8& imm)	{PushBack(Instr(I_SHL, opd1, imm));}
	void shl(const Mem64& opd1, const Imm8& imm)	{PushBack(Instr(I_SHL, opd1, imm));}
	void shr(const Reg64& opd1, const Imm8& imm)	{PushBack(Instr(I_SHR, opd1, imm));}
	void shr(const Mem64& opd1, const Imm8& imm)	{PushBack(Instr(I_SHR, opd1, imm));}
#endif

	// SBB
	void sbb(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_SBB, opd1, imm));}
	void sbb(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_SBB, opd1, imm));}
	void sbb(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_SBB, opd1, imm));}
	void sbb(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_SBB, opd1, imm));}
	void sbb(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_SBB, opd1, imm));}
	void sbb(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_SBB, opd1, imm));}
#ifdef JITASM64
	void sbb(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_SBB, opd1, opd2));}
	void sbb(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_SBB, opd1, imm));}
	void sbb(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_SBB, opd1, imm));}
#endif

	// SUB
	void sub(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_SUB, opd1, imm));}
	void sub(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_SUB, opd1, imm));}
	void sub(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_SUB, opd1, imm));}
	void sub(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_SUB, opd1, imm));}
	void sub(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_SUB, opd1, imm));}
	void sub(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_SUB, opd1, imm));}
#ifdef JITASM64
	void sub(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_SUB, opd1, opd2));}
	void sub(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_SUB, opd1, imm));}
	void sub(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_SUB, opd1, imm));}
#endif

	// TEST
	void test(const Reg8& opd1, const Imm8& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Mem8& opd1, const Imm8& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_TEST, opd1, opd2));}
	void test(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_TEST, opd1, opd2));}
	void test(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_TEST, opd1, opd2));}
	void test(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_TEST, opd1, opd2));}
	void test(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_TEST, opd1, opd2));}
	void test(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_TEST, opd1, opd2));}
#ifdef JITASM64
	void test(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_TEST, opd1, imm));}
	void test(const Reg64& opd1, const Reg64& opd2)		{PushBack(Instr(I_TEST, opd1, opd2));}
	void test(const Mem64& opd1, const Reg64& opd2)		{PushBack(Instr(I_TEST, opd1, opd2));}
#endif

	// XCHG
	void xchg(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
#ifdef JITASM64
	void xchg(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
	void xchg(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_XCHG, opd1, opd2));}
#endif

	// XOR
	void xor(const Reg8& opd1, const Reg8& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg8& opd1, const Mem8& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Mem8& opd1, const Reg8& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg8& opd1, const Imm8& imm)		{PushBack(Instr(I_XOR, opd1, imm));}
	void xor(const Mem8& opd1, const Imm8& imm)		{PushBack(Instr(I_XOR, opd1, imm));}
	void xor(const Reg16& opd1, const Reg16& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg16& opd1, const Mem16& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Mem16& opd1, const Reg16& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg16& opd1, const Imm16& imm)	{PushBack(Instr(I_XOR, opd1, imm));}
	void xor(const Mem16& opd1, const Imm16& imm)	{PushBack(Instr(I_XOR, opd1, imm));}
	void xor(const Reg32& opd1, const Reg32& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg32& opd1, const Mem32& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Mem32& opd1, const Reg32& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg32& opd1, const Imm32& imm)	{PushBack(Instr(I_XOR, opd1, imm));}
	void xor(const Mem32& opd1, const Imm32& imm)	{PushBack(Instr(I_XOR, opd1, imm));}
#ifdef JITASM64
	void xor(const Reg64& opd1, const Reg64& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg64& opd1, const Mem64& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Mem64& opd1, const Reg64& opd2)	{PushBack(Instr(I_XOR, opd1, opd2));}
	void xor(const Reg64& opd1, const Imm32& imm)	{PushBack(Instr(I_XOR, opd1, imm));}
	void xor(const Mem64& opd1, const Imm32& imm)	{PushBack(Instr(I_XOR, opd1, imm));}
#endif

	// FLD
	void fld(const Mem32& opd)	{PushBack(Instr(I_FLD, opd));}
	void fld(const Mem64& opd)	{PushBack(Instr(I_FLD, opd));}
	void fld(const Mem80& opd)	{PushBack(Instr(I_FLD, opd));}
	void fld(const FpuReg& opd)	{PushBack(Instr(I_FLD, opd));}

	void addpd(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_ADDPD, opd1, opd2));}
	void addpd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_ADDPD, opd1, opd2));}
	void addsd(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_ADDSD, opd1, opd2));}
	void addsd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_ADDSD, opd1, opd2));}

	void andpd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_ANDPD, opd1, opd2));}
	void andpd(const Xmm& opd1, const Mem128& opd2)		{PushBack(Instr(I_ANDPD, opd1, opd2));}
	void andnpd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_ANDPD, opd1, opd2));}
	void andnpd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_ANDPD, opd1, opd2));}

	void clflush(const Mem8& opd) {PushBack(Instr(I_CLFLUSH, opd));}

	void cmppd(const Xmm& opd1, const Xmm& opd2, const Imm8& opd3)		{PushBack(Instr(I_CMPPD, opd1, opd2, opd3));}
	void cmppd(const Xmm& opd1, const Mem128& opd2, const Imm8& opd3)	{PushBack(Instr(I_CMPPD, opd1, opd2, opd3));}
	void cmpeqpd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 0);}
	void cmpeqpd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 0);}
	void cmpltpd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 1);}
	void cmpltpd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 1);}
	void cmplepd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 2);}
	void cmplepd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 2);}
	void cmpunordpd(const Xmm& opd1, const Xmm& opd2)		{cmppd(opd1, opd2, 3);}
	void cmpunordpd(const Xmm& opd1, const Mem128& opd2)	{cmppd(opd1, opd2, 3);}
	void cmpneqpd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 4);}
	void cmpneqpd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 4);}
	void cmpnltpd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 5);}
	void cmpnltpd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 5);}
	void cmpnlepd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 6);}
	void cmpnlepd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 6);}
	void cmpordpd(const Xmm& opd1, const Xmm& opd2)			{cmppd(opd1, opd2, 7);}
	void cmpordpd(const Xmm& opd1, const Mem128& opd2)		{cmppd(opd1, opd2, 7);}

	void cmpps(const Xmm& opd1, const Xmm& opd2, const Imm8& opd3)		{PushBack(Instr(I_CMPPS, opd1, opd2, opd3));}
	void cmpps(const Xmm& opd1, const Mem128& opd2, const Imm8& opd3)	{PushBack(Instr(I_CMPPS, opd1, opd2, opd3));}
	void cmpeqps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 0);}
	void cmpeqps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 0);}
	void cmpltps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 1);}
	void cmpltps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 1);}
	void cmpleps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 2);}
	void cmpleps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 2);}
	void cmpunordps(const Xmm& opd1, const Xmm& opd2)		{cmpps(opd1, opd2, 3);}
	void cmpunordps(const Xmm& opd1, const Mem128& opd2)	{cmpps(opd1, opd2, 3);}
	void cmpneqps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 4);}
	void cmpneqps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 4);}
	void cmpnltps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 5);}
	void cmpnltps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 5);}
	void cmpnleps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 6);}
	void cmpnleps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 6);}
	void cmpordps(const Xmm& opd1, const Xmm& opd2)			{cmpps(opd1, opd2, 7);}
	void cmpordps(const Xmm& opd1, const Mem128& opd2)		{cmpps(opd1, opd2, 7);}

	void cmpsd(const Xmm& opd1, const Xmm& opd2, const Imm8& opd3)		{PushBack(Instr(I_CMPSD, opd1, opd2, opd3));}
	void cmpsd(const Xmm& opd1, const Mem64& opd2, const Imm8& opd3)	{PushBack(Instr(I_CMPSD, opd1, opd2, opd3));}
	void cmpeqsd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 0);}
	void cmpeqsd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 0);}
	void cmpltsd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 1);}
	void cmpltsd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 1);}
	void cmplesd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 2);}
	void cmplesd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 2);}
	void cmpunordsd(const Xmm& opd1, const Xmm& opd2)	{cmpsd(opd1, opd2, 3);}
	void cmpunordsd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 3);}
	void cmpneqsd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 4);}
	void cmpneqsd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 4);}
	void cmpnltsd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 5);}
	void cmpnltsd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 5);}
	void cmpnlesd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 6);}
	void cmpnlesd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 6);}
	void cmpordsd(const Xmm& opd1, const Xmm& opd2)		{cmpsd(opd1, opd2, 7);}
	void cmpordsd(const Xmm& opd1, const Mem64& opd2)	{cmpsd(opd1, opd2, 7);}

	void comisd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_COMISD, opd1, opd2));}
	void comisd(const Xmm& opd1, const Mem64& opd2)		{PushBack(Instr(I_COMISD, opd1, opd2));}

	void cvtdq2pd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTDQ2PD, opd1, opd2));}
	void cvtdq2pd(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(I_CVTDQ2PD, opd1, opd2));}
	void cvtpd2dq(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTPD2DQ, opd1, opd2));}
	void cvtpd2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTPD2DQ, opd1, opd2));}
	void cvtpd2pi(const Mmx& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTPD2PI, opd1, opd2));}
	void cvtpd2pi(const Mmx& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTPD2PI, opd1, opd2));}
	void cvtpd2ps(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTPD2PS, opd1, opd2));}
	void cvtpd2ps(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTPD2PS, opd1, opd2));}
	void cvtpi2pd(const Xmm& opd1, const Mmx& opd2)		{PushBack(Instr(I_CVTPI2PD, opd1, opd2));}
	void cvtpi2pd(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(I_CVTPI2PD, opd1, opd2));}
	void cvtps2dq(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTPS2DQ, opd1, opd2));}
	void cvtps2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTPS2DQ, opd1, opd2));}
	void cvtdq2ps(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTDQ2PS, opd1, opd2));}
	void cvtdq2ps(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTDQ2PS, opd1, opd2));}
	void cvtps2pd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTPS2PD, opd1, opd2));}
	void cvtps2pd(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(I_CVTPS2PD, opd1, opd2));}
	void cvtsd2ss(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTSD2SS, opd1, opd2));}
	void cvtsd2ss(const Xmm& opd1, const Mem64& opd2)	{PushBack(Instr(I_CVTSD2SS, opd1, opd2));}
	void cvtss2sd(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_CVTSS2SD, opd1, opd2));}
	void cvtss2sd(const Xmm& opd1, const Mem32& opd2)	{PushBack(Instr(I_CVTSS2SD, opd1, opd2));}
	void cvttpd2dq(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_CVTTPD2DQ, opd1, opd2));}
	void cvttpd2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTTPD2DQ, opd1, opd2));}
	void cvttpd2pi(const Mmx& opd1, const Xmm& opd2)	{PushBack(Instr(I_CVTTPD2PI, opd1, opd2));}
	void cvttpd2pi(const Mmx& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTTPD2PI, opd1, opd2));}
	void cvttps2dq(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_CVTTPS2DQ, opd1, opd2));}
	void cvttps2dq(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_CVTTPS2DQ, opd1, opd2));}

	// MOVDQA
	void movdqa(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_MOVDQA, opd1, opd2));}
	void movdqa(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_MOVDQA, opd1, opd2));}
	void movdqa(const Mem128& opd1, const Xmm& opd2)	{PushBack(Instr(I_MOVDQA, opd1, opd2));}

	// MOVDQU
	void movdqu(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_MOVDQU, opd1, opd2));}
	void movdqu(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_MOVDQU, opd1, opd2));}
	void movdqu(const Mem128& opd1, const Xmm& opd2)	{PushBack(Instr(I_MOVDQU, opd1, opd2));}

	// MOVSS
	void movss(const Xmm& dst, const Xmm& src)		{PushBack(Instr(I_MOVSS, dst, src));}
	void movss(const Xmm& dst, const Mem32& src)	{PushBack(Instr(I_MOVSS, dst, src));}
	void movss(const Mem32& dst, const Xmm& src)	{PushBack(Instr(I_MOVSS, dst, src));}

	// PABSB/PABSW/PABSD
	void pabsb(const Mmx& opd1, const Mmx& opd2)	{PushBack(Instr(I_PABSB, opd1, opd2));}
	void pabsb(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PABSB, opd1, opd2));}
	void pabsb(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_PABSB, opd1, opd2));}
	void pabsb(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PABSB, opd1, opd2));}
	void pabsw(const Mmx& opd1, const Mmx& opd2)	{PushBack(Instr(I_PABSW, opd1, opd2));}
	void pabsw(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PABSW, opd1, opd2));}
	void pabsw(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_PABSW, opd1, opd2));}
	void pabsw(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PABSW, opd1, opd2));}
	void pabsd(const Mmx& opd1, const Mmx& opd2)	{PushBack(Instr(I_PABSD, opd1, opd2));}
	void pabsd(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PABSD, opd1, opd2));}
	void pabsd(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_PABSD, opd1, opd2));}
	void pabsd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PABSD, opd1, opd2));}

	// PACKSSWB/PACKSSDW/PACKUSWB/PACKUSDW
	void packsswb(const Mmx& opd1, const Mmx& opd2)		{PushBack(Instr(I_PACKSSWB, opd1, opd2));}
	void packsswb(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PACKSSWB, opd1, opd2));}
	void packsswb(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_PACKSSWB, opd1, opd2));}
	void packsswb(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PACKSSWB, opd1, opd2));}
	void packssdw(const Mmx& opd1, const Mmx& opd2)		{PushBack(Instr(I_PACKSSDW, opd1, opd2));}
	void packssdw(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PACKSSDW, opd1, opd2));}
	void packssdw(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_PACKSSDW, opd1, opd2));}
	void packssdw(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PACKSSDW, opd1, opd2));}
	void packuswb(const Mmx& opd1, const Mmx& opd2)		{PushBack(Instr(I_PACKUSWB, opd1, opd2));}
	void packuswb(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PACKUSWB, opd1, opd2));}
	void packuswb(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_PACKUSWB, opd1, opd2));}
	void packuswb(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PACKUSWB, opd1, opd2));}
	void packusdw(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_PACKUSDW, opd1, opd2));}
	void packusdw(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PACKUSDW, opd1, opd2));}

	// PADDB/PADDW/PADDD
	void paddb(const Mmx& opd1, const Mmx& opd2)	{PushBack(Instr(I_PADDB, opd1, opd2));}
	void paddb(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PADDB, opd1, opd2));}
	void paddb(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_PADDB, opd1, opd2));}
	void paddb(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PADDB, opd1, opd2));}
	void paddw(const Mmx& opd1, const Mmx& opd2)	{PushBack(Instr(I_PADDW, opd1, opd2));}
	void paddw(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PADDW, opd1, opd2));}
	void paddw(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_PADDW, opd1, opd2));}
	void paddw(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PADDW, opd1, opd2));}
	void paddd(const Mmx& opd1, const Mmx& opd2)	{PushBack(Instr(I_PADDD, opd1, opd2));}
	void paddd(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PADDD, opd1, opd2));}
	void paddd(const Xmm& opd1, const Xmm& opd2)	{PushBack(Instr(I_PADDD, opd1, opd2));}
	void paddd(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PADDD, opd1, opd2));}

	// PXOR
	void pxor(const Mmx& opd1, const Mmx& opd2)		{PushBack(Instr(I_PXOR, opd1, opd2));}
	void pxor(const Mmx& opd1, const Mem64& opd2)	{PushBack(Instr(I_PXOR, opd1, opd2));}
	void pxor(const Xmm& opd1, const Xmm& opd2)		{PushBack(Instr(I_PXOR, opd1, opd2));}
	void pxor(const Xmm& opd1, const Mem128& opd2)	{PushBack(Instr(I_PXOR, opd1, opd2));}
};

namespace detail {

	template<int N> size_t AlignSize(size_t size) {
		return (size + N - 1) / N * N;
	}

	template<class T, int Size = sizeof(T)>
	struct ResultT {
		enum { ArgR = 1 };

	};

	template<class T>
	struct ResultT<T, 1> {
		enum { ArgR = 0 };
		Opd8 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd8& val) : val_(val) {}
		ResultT(uint8 imm) : val_(Imm8(imm)) {}
		void Store(Frontend& f) {
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == AL)))
				f.mov(f.al, static_cast<Reg8&>(val_));
		}
	};

	template<class T>
	struct ResultT<T, 2> {
		enum { ArgR = 0 };
		Opd16 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd16& val) : val_(val) {}
		ResultT(uint16 imm) : val_(Imm16(imm)) {}
		void Store(Frontend& f) {
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == AX)))
				f.mov(f.ax, static_cast<Reg16&>(val_));
		}
	};

	template<class T>
	struct ResultT<T, 4> {
		enum { ArgR = 0 };
		Opd32 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd32& val) : val_(val) {}
		ResultT(uint32 imm) : val_(Imm32(imm)) {}
		void Store(Frontend& f) {
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == EAX)))
				f.mov(f.eax, static_cast<Reg32&>(val_));
		}
	};

#ifdef JITASM64
	template<class T>
	struct ResultT<T, 8> {
		enum { ArgR = 0 };
		Opd64 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const Opd64& val) : val_(val) {}
		ResultT(uint64 imm) : val_(Imm64(imm)) {}
		void Store(Frontend& f) {
			if (!(val_.IsReg() && (val_.GetReg() == INVALID || val_.GetReg() == RAX)))
				f.mov(f.rax, static_cast<Reg64&>(val_));
		}
	};
#endif

	template<>
	struct ResultT<float, sizeof(float)> {
		enum { ArgR = 0 };
		Opd val_;
		ResultT() {}
		ResultT(const FpuReg& fpu) : val_(fpu) {}
		ResultT(const Mem32& mem) : val_(mem) {}
		ResultT(const Xmm& xmm) : val_(xmm) {}
		ResultT(const float imm) : val_(Imm32(*(uint32*)&imm)) {}
		void Store(Frontend& f) {
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
				f.movss(f.dword_ptr[f.esp - 4], static_cast<Xmm&>(val_));
				f.fld(f.dword_ptr[f.esp - 4]);
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.esp - 4], static_cast<Reg32&>(val_));
				f.fld(f.dword_ptr[f.esp - 4]);
			}
		}
	};

	template<>
	struct ResultT<double, sizeof(double)> {
		enum { ArgR = 0 };
	};


	/// cdecl function base class
	template<class FuncPtr>
	struct Function : Frontend
	{
#ifdef JITASM64
		typedef Expr64_None Arg;
#else
		typedef Expr32_None Arg;
#endif

		template<int ArgR> Arg Arg1() { return Arg(zbp + sizeof(void *) * (2 + ArgR)); }
		template<int ArgR, class A1> Arg Arg2() { return Arg1<ArgR>() + AlignSize<sizeof(void *)>(sizeof(A1)); }
		template<int ArgR, class A1, class A2> Arg Arg3() { return Arg2<A1, ArgR>() + AlignSize<sizeof(void *)>(sizeof(A2)); }

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
		main(Arg1<Result::ArgR>()).Store(*this);
		Epilog();
	}
};

template<class A1>
struct function1_cdecl<void, A1> : detail::Function<void (__cdecl *)(A1)>
{
	virtual void main(Arg a1) {}
	void naked_main() {
		Prolog(0);
		main(Arg1<0>());
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
		main(Arg1<Result::ArgR>(), Arg2<Result::ArgR, A1>()).Store(*this);
		Epilog();
	}
};

template<class A1, class A2>
struct function2_cdecl<void, A1, A2> : detail::Function<void (__cdecl *)(A1, A2)>
{
	virtual void main(Arg a1, Arg a2) {}
	void naked_main() {
		Prolog(0);
		main(Arg1<0>(), Arg2<0, A1>());
		Epilog();
	}
};

template<class R> struct function0 : function0_cdecl<R> {};
template<class R, class A1> struct function1 : function1_cdecl<R, A1> {};
template<class R, class A1, class A2> struct function2 : function2_cdecl<R, A1, A2> {};

}	// namespace jitasm
#endif	// #ifndef JITASM_H