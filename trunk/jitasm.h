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

#pragma warning( push )
#pragma warning( disable : 4127 )	// conditional expression is constant.
#pragma warning( disable : 4201 )	// nonstandard extension used : nameless struct/union

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

	ST0=0x100, ST1, ST2, ST3, ST4, ST5, ST6, ST7,

	MM0=0x200, MM1, MM2, MM3, MM4, MM5, MM6, MM7,
	XMM0=0x300, XMM1, XMM2, XMM3, XMM4, XMM5, XMM6, XMM7,
	XMM8=0x310, XMM9, XMM10, XMM11, XMM12, XMM13, XMM14, XMM15,
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
	explicit Opd(OpdSize opdsize, sint64 imm) : opdtype_(O_TYPE_IMM), opdsize_(opdsize), imm_(imm) {}

public:
	bool	IsNone() const	{return opdtype_ == O_TYPE_NONE;}
	bool	IsReg() const	{return opdtype_ == O_TYPE_REG;}
	bool	IsMem() const	{return opdtype_ == O_TYPE_MEM;}
	bool	IsImm() const	{return opdtype_ == O_TYPE_IMM;}

	OpdSize	GetSize() const		{return opdsize_;}
	OpdSize	GetAddressSize() const	{return addrsize_;}
	RegID	GetReg() const		{return reg_;}
	RegID	GetBase() const		{return base_;}
	RegID	GetIndex() const	{return index_;}
	sint64	GetScale() const	{return scale_;}
	sint64	GetDisp() const		{return disp_;}
	sint64	GetImm() const		{return imm_;}
};

namespace detail
{
	inline bool IsGpReg(const Opd& reg)		{return reg.IsReg() && (reg.GetReg() & 0xF00) == EAX;}
	inline bool IsFpuReg(const Opd& reg)	{return reg.IsReg() && (reg.GetReg() & 0xF00) == ST0;}
	inline bool IsMmxReg(const Opd& reg)	{return reg.IsReg() && (reg.GetReg() & 0xF00) == MM0;}
	inline bool IsXmmReg(const Opd& reg)	{return reg.IsReg() && (reg.GetReg() & 0xF00) == XMM0;}
}	// namespace detail

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
	OpdT(sint64 imm) : Opd(static_cast<OpdSize>(Size), imm) {}
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
struct Reg8 : Opd8 { explicit Reg8(RegID reg) : Opd8(reg) {} };
struct Reg16 : Opd16 { explicit Reg16(RegID reg) : Opd16(reg) {} };
struct Reg32 : Opd32 { explicit Reg32(RegID reg) : Opd32(reg) {} };
#ifdef JITASM64
struct Reg64 : Opd64 { explicit Reg64(RegID reg) : Opd64(reg) {} };
#endif
struct FpuReg : Opd80 { explicit FpuReg(RegID reg) : Opd80(reg) {} };
struct MmxReg : Opd64 { explicit MmxReg(RegID reg) : Opd64(reg) {} };
struct XmmReg : Opd128 { explicit XmmReg(RegID reg) : Opd128(reg) {} };

struct Reg8_al : Reg8 {Reg8_al() : Reg8(AL) {}};
struct Reg8_cl : Reg8 {Reg8_cl() : Reg8(CL) {}};
struct Reg16_ax : Reg16 {Reg16_ax() : Reg16(AX) {}};
struct Reg32_eax : Reg32 {Reg32_eax() : Reg32(EAX) {}};
#ifdef JITASM64
struct Reg64_rax : Reg64 {Reg64_rax() : Reg64(RAX) {}};
#endif
struct XmmReg_xmm0 : XmmReg {XmmReg_xmm0() : XmmReg(XMM0) {}};

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
	sint64 GetOffset() const {return offset_;}
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

namespace detail
{
	inline bool IsInt8(sint64 n) {return (sint8) n == n;}
	inline bool IsInt16(sint64 n) {return (sint16) n == n;}
	inline bool IsInt32(sint64 n) {return (sint32) n == n;}
	inline Opd ImmXor8(const Imm16& imm)	{return IsInt8(imm.GetImm()) ? (Opd) Imm8((sint8) imm.GetImm()) : (Opd) imm;}
	inline Opd ImmXor8(const Imm32& imm)	{return IsInt8(imm.GetImm()) ? (Opd) Imm8((sint8) imm.GetImm()) : (Opd) imm;}
	inline Opd ImmXor8(const Imm64& imm)	{return IsInt8(imm.GetImm()) ? (Opd) Imm8((sint8) imm.GetImm()) : (Opd) imm;}
}	// namespace detail

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
	MemOffset64 operator[](sint64 offset)			{return MemOffset64(offset);}
	MemOffset64 operator[](uint64 offset)			{return MemOffset64((sint64) offset);}
#endif
};

/// Instruction ID
enum InstrID
{
	I_ADC, I_ADD, I_AND, I_BSF, I_BSR, I_BSWAP, I_BT, I_BTC, I_BTR, I_BTS, I_CALL, I_CBW, I_CLC, I_CLD, I_CLI, I_CLTS, I_CMC, I_CMOVCC, I_CMP,
	I_CMPXCHG, I_CMPXCHG8B, I_CMPXCHG16B, I_CPUID, I_CWD, I_CDQ, I_CQO, I_DEC, I_DIV, I_ENTER, I_HLT, I_IDIV, I_IMUL, I_INC, I_INVD,
	I_INVLPG, I_INT3, I_IRET, I_IRETD, I_IRETQ, I_LAR, I_JMP, I_JCC, I_LEA, I_LEAVE, I_LODS_B, I_LODS_W, I_LODS_D, I_LODS_Q, I_LOOP,
	I_MOV, I_MOVBE, I_MOVS_B, I_MOVS_W, I_MOVS_D, I_MOVS_Q, I_MOVZX, I_MOVSX, I_MOVSXD,	I_MUL, I_NEG, I_NOP, I_NOT,
	I_OR, I_POP, I_PUSH, I_RDTSC, I_RET, I_RCL, I_RCR, I_ROL, I_ROR, I_SAR, I_SHL, I_SHR, I_SBB, I_SETCC, I_SHLD, I_SHRD, I_STC, I_STD, I_STI,
	I_STOS_B, I_STOS_W, I_STOS_D, I_STOS_Q, I_SUB, I_TEST, I_UD2, I_FWAIT, I_XADD, I_XCHG, I_XOR,

	I_FISTTP, I_FLD, I_FST, I_FSTP,

	I_CRC32, I_ADDPS, I_ADDSS, I_ADDPD, I_ADDSD, I_ADDSUBPS, I_ADDSUBPD, I_ANDPS, I_ANDPD, I_ANDNPS, I_ANDNPD, I_BLENDPS, I_BLENDPD, I_BLENDVPS, I_BLENDVPD, I_CLFLUSH, I_CMPPS, I_CMPSS, I_CMPPD, I_CMPSD, I_COMISS, I_COMISD,
	I_CVTDQ2PD, I_CVTDQ2PS, I_CVTPD2DQ, I_CVTPD2PI, I_CVTPD2PS, I_CVTPI2PD, I_CVTPI2PS, I_CVTPS2DQ, I_CVTPS2PD, I_CVTPS2PI, I_CVTSD2SI,
	I_CVTSD2SS, I_CVTSI2SD, I_CVTSI2SS, I_CVTSS2SD, I_CVTSS2SI, I_CVTTPD2DQ, I_CVTTPD2PI, I_CVTTPS2DQ, I_CVTTPS2PI, I_CVTTSD2SI, I_CVTTSS2SI,
	I_DIVPS, I_DIVSS, I_DIVPD, I_DIVSD, I_DPPS, I_DPPD, I_EMMS, I_EXTRACTPS, I_HADDPS, I_HADDPD, I_HSUBPS, I_HSUBPD, I_INSERTPS, I_LDDQU, I_LDMXCSR, I_LFENCE,
	I_MASKMOVDQU, I_MASKMOVQ, I_MAXPS, I_MAXSS, I_MAXPD, I_MAXSD, I_MFENCE, I_MINPS, I_MINSS, I_MINPD, I_MINSD, I_MONITOR,
	I_MOVAPD, I_MOVAPS, I_MOVD, I_MOVDDUP, I_MOVDQA, I_MOVDQU, I_MOVDQ2Q, I_MOVHLPS, I_MOVLHPS, I_MOVHPS, I_MOVHPD, I_MOVLPS, I_MOVLPD,
	I_MOVMSKPS, I_MOVMSKPD, I_MOVNTDQ, I_MOVNTDQA, I_MOVNTI, I_MOVNTPD, I_MOVNTPS, I_MOVNTQ, I_MOVQ, I_MOVQ2DQ, I_MOVSD, I_MOVSS, I_MOVSHDUP, I_MOVSLDUP, I_MOVUPS, I_MOVUPD,
	I_MPSADBW, I_MULPS, I_MULSS, I_MULPD, I_MULSD, I_MWAIT, I_ORPS, I_ORPD, I_PABSB, I_PABSD, I_PABSW, I_PACKSSDW, I_PACKSSWB, I_PACKUSDW, I_PACKUSWB,
	I_PADDB, I_PADDD, I_PADDQ, I_PADDSB, I_PADDSW, I_PADDUSB, I_PADDUSW, I_PADDW, I_PALIGNR,
	I_PAND, I_PANDN, I_PAUSE, I_PAVGB, I_PAVGW, I_PBLENDVB, I_PBLENDW, I_PCMPEQB, I_PCMPEQW, I_PCMPEQD, I_PCMPEQQ, I_PCMPESTRI, I_PCMPESTRM, I_PCMPISTRI, I_PCMPISTRM,
	I_PCMPGTB, I_PCMPGTW, I_PCMPGTD, I_PCMPGTQ, I_PEXTRB, I_PEXTRW, I_PEXTRD, I_PEXTRQ, I_PHADDW, I_PHADDD, I_PHADDSW, I_PHSUBW, I_PHSUBD, I_PHSUBSW, 
	I_PINSRB, I_PINSRW, I_PINSRD, I_PINSRQ, I_PMADDUBSW, I_PMADDWD,
	I_PMAXSB, I_PMAXSW, I_PMAXSD, I_PMAXUB, I_PMAXUW, I_PMAXUD, I_PMINSB, I_PMINSW, I_PMINSD, I_PMINUB, I_PMINUW, I_PMINUD, I_PMOVMSKB,
	I_PMOVSXBW, I_PMOVSXBD, I_PMOVSXBQ, I_PMOVSXWD, I_PMOVSXWQ, I_PMOVSXDQ, I_PMOVZXBW, I_PMOVZXBD, I_PMOVZXBQ, I_PMOVZXWD, I_PMOVZXWQ, I_PMOVZXDQ, 
	I_PMULDQ, I_PMULHRSW, I_PMULHUW, I_PMULHW, I_PMULLW, I_PMULLD, I_PMULUDQ, I_POPCNT,
	I_POR, I_PREFETCH, I_PSADBW, I_PSHUFB, I_PSHUFD, I_PSHUFHW, I_PSHUFLW, I_PSHUFW, I_PSIGNB, I_PSIGNW, I_PSIGND, I_PSLLW, I_PSLLD, I_PSLLQ, I_PSLLDQ, I_PSRAW,
	I_PSRAD, I_PSRLW, I_PSRLD, I_PSRLQ, I_PSRLDQ, I_PSUBB, I_PSUBW, I_PSUBD, I_PSUBQ, I_PSUBSB, I_PSUBSW,
	I_PSUBUSB, I_PSUBUSW, I_PTEST, I_PUNPCKHBW, I_PUNPCKHWD, I_PUNPCKHDQ, I_PUNPCKHQDQ, I_PUNPCKLBW, I_PUNPCKLWD, I_PUNPCKLDQ, I_PUNPCKLQDQ,
	I_PXOR, I_RCPPS, I_RCPSS, I_ROUNDPS, I_ROUNDPD, I_ROUNDSS, I_ROUNDSD, I_RSQRTPS, I_RSQRTSS, I_SFENCE, I_SHUFPS, I_SHUFPD, I_SQRTPS, I_SQRTSS, I_SQRTPD, I_SQRTSD, I_STMXCSR, 
	I_SUBPS, I_SUBSS, I_SUBPD, I_SUBSD, I_UCOMISS, I_UCOMISD, I_UNPCKHPS, I_UNPCKHPD, I_UNPCKLPS, I_UNPCKLPD, I_XORPS, I_XORPD,
};

enum JumpCondition
{
	JCC_O, JCC_NO, JCC_B, JCC_AE, JCC_E, JCC_NE, JCC_BE, JCC_A, JCC_S, JCC_NS,
	JCC_P, JCC_NP, JCC_L, JCC_GE, JCC_LE, JCC_G,
	JCC_CXZ, JCC_ECXZ, JCC_RCXZ,
};

enum EncodingFlags
{
	E_SPECIAL					= 0x00000001,
	E_OPERAND_SIZE_PREFIX		= 0x00000002,
	E_REPEAT_PREFIX				= 0x00000004,
	E_REXW_PREFIX				= 0x00000008,
	E_MANDATORY_PREFIX_66		= 0x00000010,
	E_MANDATORY_PREFIX_F2		= 0x00000020,
	E_MANDATORY_PREFIX_F3		= 0x00000040,
};

/// Instruction
struct Instr
{
	static const size_t MAX_OPERAND_COUNT = 3;

	InstrID	id_;
	uint32  opcode_;					///< Opcode
	uint32  encoding_flag_;				///< EncodingFlags
	Opd		opd_[MAX_OPERAND_COUNT];	///< Operands

	Instr(InstrID id, uint32 opcode, uint32 encoding_flag, const Opd& opd1 = Opd(), const Opd& opd2 = Opd(), const Opd& opd3 = Opd())
		: id_(id), opcode_(opcode), encoding_flag_(encoding_flag) {opd_[0] = opd1, opd_[1] = opd2, opd_[2] = opd3;}

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

	uint8 GetRexPrefix(int w, const Opd& reg, const Opd& r_m)
	{
		uint8 wrxb = w ? 8 : 0;
		if (reg.IsReg()) {
			if (reg.GetReg() != INVALID && reg.GetReg() & 0x10) wrxb |= 4;
		}
		if (r_m.IsReg()) {
			if (r_m.GetReg() & 0x10) wrxb |= 1;
		}
		if (r_m.IsMem()) {
			if (r_m.GetIndex() != INVALID && r_m.GetIndex() & 0x10) wrxb |= 2;
			if (r_m.GetBase() != INVALID && r_m.GetBase() & 0x10) wrxb |= 1;
		}
		return wrxb;
	}

	void EncodePrefixes(uint32 encoding_flag, const Opd& reg, const Opd& r_m)
	{
		uint8 rex_wrxb = GetRexPrefix(encoding_flag & E_REXW_PREFIX, reg, r_m);
		if (rex_wrxb) {
			ASSERT(!reg.IsReg() || reg.GetSize() != O_SIZE_8 || reg.GetReg() < AH || reg.GetReg() >= R8B);	// AH, BH, CH, or DH may not be used with REX.
			ASSERT(!r_m.IsReg() || r_m.GetSize() != O_SIZE_8 || r_m.GetReg() < AH || r_m.GetReg() >= R8B);	// AH, BH, CH, or DH may not be used with REX.

			if (encoding_flag & E_REPEAT_PREFIX) db(0xF3);
#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) db(0x67);
#endif
			if (encoding_flag & E_OPERAND_SIZE_PREFIX) db(0x66);

			if (encoding_flag & E_MANDATORY_PREFIX_66) db(0x66);
			else if (encoding_flag & E_MANDATORY_PREFIX_F2) db(0xF2);
			else if (encoding_flag & E_MANDATORY_PREFIX_F3) db(0xF3);

			db(0x40 | rex_wrxb);
		} else {
			if (encoding_flag & E_MANDATORY_PREFIX_66) db(0x66);
			else if (encoding_flag & E_MANDATORY_PREFIX_F2) db(0xF2);
			else if (encoding_flag & E_MANDATORY_PREFIX_F3) db(0xF3);

			if (encoding_flag & E_REPEAT_PREFIX) db(0xF3);
#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) db(0x67);
#endif
			if (encoding_flag & E_OPERAND_SIZE_PREFIX) db(0x66);
		}
	}

	void EncodeModRM(const Opd& reg, const Opd& r_m) {EncodeModRM((uint8) reg.GetReg(), r_m);}
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
				uint8 mod = 0;
				if (r_m.GetDisp() == 0 || sib && base == INVALID) mod = base != EBP ? 0 : 1;
				else if (detail::IsInt8(r_m.GetDisp())) mod = 1;
				else if (detail::IsInt32(r_m.GetDisp())) mod = 2;
				else ASSERT(0);
				db(mod << 6 | reg << 3 | (sib ? 4 : base));

				// SIB
				if (sib) {
					uint8 ss = 0;
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

	void EncodeOpcode(uint32 opcode)
	{
		if (opcode & 0xFF000000) db((opcode >> 24) & 0xFF);
		if (opcode & 0xFFFF0000) db((opcode >> 16) & 0xFF);
		if (opcode & 0xFFFFFF00) db((opcode >> 8)  & 0xFF);
		db(opcode & 0xFF);
	}

	void EncodeImm(const Opd& imm)
	{
		const OpdSize size = imm.GetSize();
		if (size == O_SIZE_8) db(imm.GetImm());
		else if (size == O_SIZE_16) dw(imm.GetImm());
		else if (size == O_SIZE_32) dd(imm.GetImm());
		else if (size == O_SIZE_64) dq(imm.GetImm());
		else ASSERT(0);
	}

	void Encode(const Instr& instr)
	{
		uint32 opcode = instr.opcode_;

		const Opd& opd1 = instr.GetOpd(0);
		const Opd& opd2 = instr.GetOpd(1);
		const Opd& opd3 = instr.GetOpd(2);

		// +rb, +rw, +rd, +ro
		if (opd1.IsReg() && (opd2.IsNone() || opd2.IsImm())) {
			opcode += opd1.GetReg() & 0xF;
		}

		if ((opd1.IsImm() || opd1.IsReg()) && (opd2.IsReg() || opd2.IsMem())) {	// ModR/M
			const Opd& reg = opd1;
			const Opd& r_m = opd2;

			EncodePrefixes(instr.encoding_flag_, reg, r_m);
			EncodeOpcode(opcode);

			if (reg.IsImm())
				EncodeModRM(static_cast<uint8>(reg.GetImm()), r_m);
			else
				EncodeModRM(reg, r_m);
		} else {
			const Opd& reg = Opd();
			const Opd& r_m = opd1.IsReg() ? opd1 : Opd();

			EncodePrefixes(instr.encoding_flag_, reg, r_m);
			EncodeOpcode(opcode);
		}

		if (opd1.IsImm() && !opd2.IsReg() && !opd2.IsMem())	EncodeImm(opd1);
		if (opd2.IsImm())	EncodeImm(opd2);
		if (opd3.IsImm())	EncodeImm(opd3);
	}

	void EncodeALU(const Instr& instr, uint32 opcode)
	{
		const Opd& reg = instr.GetOpd(1);
		const Opd& imm = instr.GetOpd(2);
		ASSERT(instr.GetOpd(0).IsImm() && reg.IsReg() && imm.IsImm());

		if (reg.GetReg() == EAX && (reg.GetSize() == O_SIZE_8 || !detail::IsInt8(imm.GetImm()))) {
			opcode |= (reg.GetSize() == O_SIZE_8 ? 0 : 1);
			Encode(Instr(instr.GetID(), opcode, instr.encoding_flag_, reg, imm));
		} else {
			Encode(instr);
		}
	}

	void EncodeJMP(const Instr& instr)
	{
		const Opd& imm = instr.GetOpd(0);
		if (instr.GetID() == I_JMP) {
			Encode(Instr(instr.GetID(), imm.GetSize() == O_SIZE_8 ? 0xEB : 0xE9, instr.encoding_flag_, imm));
		} else if (instr.GetID() == I_JCC) {
#ifndef JITASM64
			uint32 tttn = instr.opcode_;
			if (tttn == JCC_CXZ)		Encode(Instr(instr.GetID(), 0x67E3, instr.encoding_flag_, imm));
			else if (tttn == JCC_ECXZ)	Encode(Instr(instr.GetID(), 0xE3, instr.encoding_flag_, imm));
			else Encode(Instr(instr.GetID(), (imm.GetSize() == O_SIZE_8 ? 0x70 : 0x0F80) | tttn, instr.encoding_flag_, imm));
#else
			uint32 tttn = instr.opcode_;
			if (tttn == JCC_ECXZ)		Encode(Instr(instr.GetID(), 0x67E3, instr.encoding_flag_, imm));
			else if (tttn == JCC_RCXZ)	Encode(Instr(instr.GetID(), 0xE3, instr.encoding_flag_, imm));
			else Encode(Instr(instr.GetID(), (imm.GetSize() == O_SIZE_8 ? 0x70 : 0x0F80) | tttn, instr.encoding_flag_, imm));
#endif
		} else if (instr.GetID() == I_LOOP) {
			Encode(Instr(instr.GetID(), instr.opcode_, instr.encoding_flag_, imm));
		} else {
			ASSERT(0);
		}
	}

	void EncodeMOV(const Instr& instr)
	{
		const Opd& reg = instr.GetOpd(0);
		const Opd& mem = instr.GetOpd(1);
		ASSERT(reg.IsReg() && mem.IsMem());

#ifndef JITASM64
		if (reg.GetReg() == EAX && mem.GetBase() == INVALID && mem.GetIndex() == INVALID) {
			uint32 opcode = 0xA0 | ~instr.opcode_ & 0x2 | instr.opcode_ & 1;
			Encode(Instr(instr.GetID(), opcode, instr.encoding_flag_, Imm32((sint32) mem.GetDisp())));
		} else {
			Encode(instr);
		}
#else
		Encode(instr);
#endif
	}

	void EncodeTEST(const Instr& instr)
	{
		const Opd& reg = instr.GetOpd(1);
		const Opd& imm = instr.GetOpd(2);
		ASSERT(instr.GetOpd(0).IsImm() && reg.IsReg() && imm.IsImm());

		if (reg.GetReg() == EAX) {
			uint32 opcode = 0xA8 | (reg.GetSize() == O_SIZE_8 ? 0 : 1);
			Encode(Instr(instr.GetID(), opcode, instr.encoding_flag_, reg, imm));
		} else {
			Encode(instr);
		}
	}

	void EncodeXCHG(const Instr& instr)
	{
		const Opd& dst = instr.GetOpd(0);
		const Opd& src = instr.GetOpd(1);
		ASSERT(dst.IsReg() && src.IsReg());

		if (dst.GetReg() == EAX) {
			Encode(Instr(instr.GetID(), 0x90, instr.encoding_flag_, src));
		} else if (src.GetReg() == EAX) {
			Encode(Instr(instr.GetID(), 0x90, instr.encoding_flag_, dst));
		} else {
			Encode(instr);
		}
	}

	void Assemble(const Instr& instr)
	{
		if (instr.encoding_flag_ & E_SPECIAL) {
			switch (instr.GetID()) {
			case I_ADD:		EncodeALU(instr, 0x04); break;
			case I_OR:		EncodeALU(instr, 0x0C); break;
			case I_ADC:		EncodeALU(instr, 0x14); break;
			case I_SBB:		EncodeALU(instr, 0x1C); break;
			case I_AND:		EncodeALU(instr, 0x24); break;
			case I_SUB:		EncodeALU(instr, 0x2C); break;
			case I_XOR:		EncodeALU(instr, 0x34); break;
			case I_CMP:		EncodeALU(instr, 0x3C); break;
			case I_JMP:		EncodeJMP(instr); break;
			case I_JCC:		EncodeJMP(instr); break;
			case I_LOOP:	EncodeJMP(instr); break;
			case I_MOV:		EncodeMOV(instr); break;
			case I_TEST:	EncodeTEST(instr); break;
			case I_XCHG:	EncodeXCHG(instr); break;
			}
		} else {
			Encode(instr);
		}
	}

	static size_t GetInstrCodeSize(const Instr& instr)
	{
		Backend backend;
		backend.Assemble(instr);
		return backend.GetSize();
	}
};

namespace detail
{
	/// Counting 1-Bits
	inline uint32 Count1Bits(uint32 x)
	{
		x = x - ((x >> 1) & 0x55555555);
		x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
		x = (x + (x >> 4)) & 0x0F0F0F0F;
		x = x + (x >> 8);
		x = x + (x >> 16);
		return x & 0x0000003F;
	}

	class CodeBuffer
	{
		void*	pbuff_;
		size_t	codesize_;
		size_t	buffsize_;

	public:
		CodeBuffer() : pbuff_(NULL), codesize_(0), buffsize_(0) {}
		~CodeBuffer() {Reset(0);}

		void* GetPointer() const {return pbuff_;}
		size_t GetCodeSize() const {return codesize_;}
		size_t GetBufferSize() const {return buffsize_;}

		void Reset(size_t codesize)
		{
			if (pbuff_) {
				::VirtualFree(pbuff_, 0, MEM_RELEASE);
				pbuff_ = NULL;
				codesize_ = 0;
				buffsize_ = 0;
			}
			if (codesize) {
				void* pbuff = ::VirtualAlloc(NULL, codesize, MEM_COMMIT, PAGE_EXECUTE_READWRITE);
				if (pbuff == NULL) ASSERT(0);
				MEMORY_BASIC_INFORMATION info;
				::VirtualQuery(pbuff, &info, sizeof(info));

				pbuff_ = pbuff;
				codesize_ = codesize;
				buffsize_ = info.RegionSize;
			}
		}
	};

	class SpinLock
	{
		long lock_;
	public:
		SpinLock() : lock_(0) {}
		void Lock() {while (::InterlockedExchange(&lock_, 1));}
		void Unlock() {::InterlockedExchange(&lock_, 0);}
	};

	template<class Ty>
	class ScopedLock
	{
		Ty& lock_;
		ScopedLock<Ty>& operator=(const ScopedLock<Ty>&);
	public:
		ScopedLock(Ty& lock) : lock_(lock) {lock.Lock();}
		~ScopedLock() {lock_.Unlock();}
	};
}	// namespace detail

struct Frontend
{
	Reg8_al		al;
	Reg8_cl		cl;
	Reg8		dl, bl, ah, ch, dh, bh;
	Reg16_ax	ax;
	Reg16		cx, dx, bx, sp, bp, si, di;
	Reg32_eax	eax;
	Reg32		ecx, edx, ebx, esp, ebp, esi, edi;
	MmxReg		mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7;
	XmmReg_xmm0 xmm0;
	XmmReg		xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7;
#ifdef JITASM64
	Reg8		r8b, r9b, r10b, r11b, r12b, r13b, r14b, r15b;
	Reg16		r8w, r9w, r10w, r11w, r12w, r13w, r14w, r15w;
	Reg32		r8d, r9d, r10d, r11d, r12d, r13d, r14d, r15d;
	Reg64_rax	rax;
	Reg64		rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15;
	XmmReg		xmm8, xmm9, xmm10, xmm11, xmm12, xmm13, xmm14, xmm15;
#endif
	struct {FpuReg operator()(size_t n) const {ASSERT(n >= 0 && n <= 7); return FpuReg((RegID) (ST0 + n));}} st;

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
	Reg64_rax	zax;
	Reg64		zcx, zdx, zbx, zsp, zbp, zsi, zdi;
	AddressingPtr<Opd64>	zword_ptr;
#else
	Reg32_eax	zax;
	Reg32		zcx, zdx, zbx, zsp, zbp, zsi, zdi;
	AddressingPtr<Opd32>	zword_ptr;
#endif

	Frontend()
		: dl(DL), bl(BL), ah(AH), ch(CH), dh(DH), bh(BH),
		cx(CX), dx(DX), bx(BX), sp(SP), bp(BP), si(SI), di(DI),
		ecx(ECX), edx(EDX), ebx(EBX), esp(ESP), ebp(EBP), esi(ESI), edi(EDI),
		mm0(MM0), mm1(MM1), mm2(MM2), mm3(MM3), mm4(MM4), mm5(MM5), mm6(MM6), mm7(MM7),
		xmm1(XMM1), xmm2(XMM2), xmm3(XMM3), xmm4(XMM4), xmm5(XMM5), xmm6(XMM6), xmm7(XMM7),
#ifdef JITASM64
		r8b(R8B), r9b(R9B), r10b(R10B), r11b(R11B), r12b(R12B), r13b(R13B), r14b(R14B), r15b(R15B),
		r8w(R8W), r9w(R9W), r10w(R10W), r11w(R11W), r12w(R12W), r13w(R13W), r14w(R14W), r15w(R15W),
		r8d(R8D), r9d(R9D), r10d(R10D), r11d(R11D), r12d(R12D), r13d(R13D), r14d(R14D), r15d(R15D),
		rcx(RCX), rdx(RDX), rbx(RBX), rsp(RSP), rbp(RBP), rsi(RSI), rdi(RDI),
		r8(R8), r9(R9), r10(R10), r11(R11), r12(R12), r13(R13), r14(R14), r15(R15),
		xmm8(XMM8), xmm9(XMM9), xmm10(XMM10), xmm11(XMM11), xmm12(XMM12), xmm13(XMM13), xmm14(XMM14), xmm15(XMM15),
		zcx(RCX), zdx(RDX), zbx(RBX), zsp(RSP), zbp(RBP), zsi(RSI), zdi(RDI),
#else
		zcx(ECX), zdx(EDX), zbx(EBX), zsp(ESP), zbp(EBP), zsi(ESI), zdi(EDI),
#endif
		assembled_(false)
	{
	}

	typedef std::deque<Instr> InstrList;
	InstrList			instrs_;
	bool				assembled_;
	detail::CodeBuffer	codebuff_;
	detail::SpinLock	codelock_;

	struct Label
	{
		std::string	name;
		size_t		instr_number;
		explicit Label(const std::string& name_) : name(name_), instr_number(0) {}
	};
	typedef std::deque<Label> LabelList;
	LabelList	labels_;

	virtual void naked_main() = 0;

	/// Make function prolog and epilog
	void MakePrologAndEpilog()
	{
		// Find out which registers need to save
		uint32 gpreg = 0, xmmreg = 0;
		for (InstrList::iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			for (size_t i = 0; i < Instr::MAX_OPERAND_COUNT; ++i) {
				const Opd& opd = it->GetOpd(i);
				if (detail::IsGpReg(opd)) {
					gpreg |= 1 << (opd.GetReg() - EAX);
				}
				else if (detail::IsXmmReg(opd)) {
					xmmreg |= 1 << (opd.GetReg() - XMM0);
				}
			}
		}

		// Prolog
		InstrList main_instr;
		main_instr.swap(instrs_);	// Put main instructions aside for prolog
		push(zbp);
		mov(zbp, zsp);
		uint32 count = 0;
		if (gpreg & (1 << (EBX - EAX))) { push(zbx); ++count; }
		if (gpreg & (1 << (EDI - EAX))) { push(zdi); ++count; }
		if (gpreg & (1 << (ESI - EAX))) { push(zsi); ++count; }
#ifdef JITASM64
		if (gpreg & (1 << (R12 - RAX))) { push(r12); ++count; }
		if (gpreg & (1 << (R13 - RAX))) { push(r13); ++count; }
		if (gpreg & (1 << (R14 - RAX))) { push(r14); ++count; }
		if (gpreg & (1 << (R15 - RAX))) { push(r15); ++count; }
		uint32 xmm_store = detail::Count1Bits(xmmreg & 0xFFC0) * 16;
		if (xmm_store > 0) {
			xmm_store += (count & 1) * 8;	// align 16 bytes
			sub(rsp, xmm_store);
		}
		size_t offset = 0;
		for (int reg_id = XMM15; reg_id >= XMM6; --reg_id) {
			if (xmmreg & (1 << (reg_id - XMM0))) {
				movdqa(xmmword_ptr[rsp + offset], XmmReg(static_cast<RegID>(reg_id)));
				offset += 16;
			}
		}
#endif

		// Adjust labels
		const size_t prolog_instr_count = instrs_.size();
		for (LabelList::iterator it = labels_.begin(); it != labels_.end(); ++it) {
			it->instr_number += prolog_instr_count;
		}

		// Put main instructions after prolog
		instrs_.insert(instrs_.end(), main_instr.begin(), main_instr.end());

		// Epilog
#ifdef JITASM64
		for (int reg_id = XMM6; reg_id <= XMM15; ++reg_id) {
			if (xmmreg & (1 << (reg_id - XMM0))) {
				offset -= 16;
				movdqa(XmmReg(static_cast<RegID>(reg_id)), xmmword_ptr[rsp + offset]);
			}
		}
		if (xmm_store > 0)
			add(rsp, xmm_store);
		if (gpreg & (1 << (R15 - RAX))) pop(r15);
		if (gpreg & (1 << (R14 - RAX))) pop(r14);
		if (gpreg & (1 << (R13 - RAX))) pop(r13);
		if (gpreg & (1 << (R12 - RAX))) pop(r12);
#endif
		if (gpreg & (1 << (ESI - EAX))) pop(zsi);
		if (gpreg & (1 << (EDI - EAX))) pop(zdi);
		if (gpreg & (1 << (EBX - EAX))) pop(zbx);
		leave();
		ret();
	}

	bool IsJump(InstrID id) const
	{
		return id == I_JMP || id == I_JCC || id == I_LOOP;
	}

	// TODO: Return an error when there is no destination.
	void ResolveJump()
	{
		// Replace label indexes with instruncion numbers.
		for (InstrList::iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			Instr& instr = *it;
			if (IsJump(instr.GetID())) {
				size_t label_id = (size_t) instr.GetOpd(0).GetImm();
				instr = Instr(instr.GetID(), instr.opcode_, instr.encoding_flag_, Imm8(0x7F), Imm64(labels_[label_id].instr_number));	// Opd(0) = max value in sint8, Opd(1) = instruction number
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
			for (InstrList::const_iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
				pre.Assemble(*it);
				offsets.push_back((int) pre.GetSize());
			}

			retry = false;
			for (size_t i = 0; i < instrs_.size(); i++) {
				Instr& instr = instrs_[i];
				if (IsJump(instr.GetID())) {
					size_t d = (size_t) instr.GetOpd(1).GetImm();
					int rel = (int) offsets[d] - offsets[i + 1];
					OpdSize size = instr.GetOpd(0).GetSize();
					if (size == O_SIZE_8) {
						if (!detail::IsInt8(rel)) {
							// jrcxz, jcxz, jecxz, loop, loope, loopne are only for short jump
							uint32 tttn = instr.opcode_;
							if (instr.GetID() == I_JCC && (tttn == JCC_CXZ || tttn == JCC_ECXZ || tttn == JCC_RCXZ)) ASSERT(0);
							if (instr.GetID() == I_LOOP) ASSERT(0);

							// Retry with immediate 32
							instr = Instr(instr.GetID(), instr.opcode_, instr.encoding_flag_, Imm32(0x7FFFFFFF), Imm64(instr.GetOpd(1).GetImm()));
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
			if (IsJump(instr.GetID())) {
				size_t d = (size_t) instr.GetOpd(1).GetImm();
				int rel = (int) offsets[d] - offsets[i + 1];
				OpdSize size = instr.GetOpd(0).GetSize();
				if (size == O_SIZE_8) {
					ASSERT(detail::IsInt8(rel));
					instr = Instr(instr.GetID(), instr.opcode_, instr.encoding_flag_, Imm8((uint8) rel));
				} else if (size == O_SIZE_32) {
					ASSERT(detail::IsInt32(rel));
					instr = Instr(instr.GetID(), instr.opcode_, instr.encoding_flag_, Imm32((uint32) rel));
				}
			}
		}
	}

	/// Assemble
	void Assemble()
	{
		detail::ScopedLock<detail::SpinLock> lock(codelock_);
		if (assembled_) return;

		instrs_.clear();
		labels_.clear();
		naked_main();

		// Resolve jump instructions
		if (!labels_.empty()) {
			ResolveJump();
		}

		// Count total size of machine code
		Backend pre;
		for (InstrList::const_iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			pre.Assemble(*it);
		}
		size_t codesize = pre.GetSize();

		// Write machine code to the buffer
		codebuff_.Reset(codesize);
		Backend backend(codebuff_.GetPointer(), codebuff_.GetBufferSize());
		for (InstrList::const_iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			backend.Assemble(*it);
		}

		assembled_ = true;
	}

	/// Get assembled code
	void *GetCode()
	{
		if (!assembled_) {
			Assemble();
		}
		return codebuff_.GetPointer();
	}

	/// Get total size of machine code
	size_t GetCodeSize() const
	{
		return codebuff_.GetCodeSize();
	}

	void AppendInstr(InstrID id, uint32 opcode, uint32 encoding_flag, const Opd& opd1 = Opd(), const Opd& opd2 = Opd(), const Opd& opd3 = Opd())
	{
		instrs_.push_back(Instr(id, opcode, encoding_flag, opd1, opd2, opd3));
	}

	void AppendJmp(size_t label_id)
	{
		AppendInstr(I_JMP, 0, E_SPECIAL, Imm64(label_id));
	}

	void AppendJcc(JumpCondition jcc, size_t label_id)
	{
		AppendInstr(I_JCC, jcc, E_SPECIAL, Imm64(label_id));
	}

	size_t NewLabelID(const std::string& label_name)
	{
		labels_.push_back(Label(label_name));
		return labels_.size() - 1;
	}

	size_t GetLabelID(const std::string& label_name)
	{
		for (size_t i = 0; i < labels_.size(); i++) {
			if (labels_[i].name == label_name) {
				return i;
			}
		}
		return NewLabelID(label_name);
	}

	void L(size_t label_id)
	{
		labels_[label_id].instr_number = instrs_.size();	// Label current instruction
	}

	// Label
	void L(const std::string& label_name)
	{
		L(GetLabelID(label_name));
	}

	// General-Purpose Instructions
	void adc(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_ADC, 0x80, E_SPECIAL, Imm8(2), dst, imm);}
	void adc(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_ADC, 0x80, 0, Imm8(2), dst, imm);}
	void adc(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_ADC, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(2), dst, detail::ImmXor8(imm));}
	void adc(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_ADC, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(2), dst, detail::ImmXor8(imm));}
	void adc(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_ADC, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(2), dst, detail::ImmXor8(imm));}
	void adc(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_ADC, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(2), dst, detail::ImmXor8(imm));}
	void adc(const Reg8& dst, const Reg8& src)		{AppendInstr(I_ADC, 0x10, 0, src, dst);}
	void adc(const Mem8& dst, const Reg8& src)		{AppendInstr(I_ADC, 0x10, 0, src, dst);}
	void adc(const Reg8& dst, const Mem8& src)		{AppendInstr(I_ADC, 0x12, 0, dst, src);}
	void adc(const Reg16& dst, const Reg16& src)	{AppendInstr(I_ADC, 0x11, E_OPERAND_SIZE_PREFIX, src, dst);}
	void adc(const Mem16& dst, const Reg16& src)	{AppendInstr(I_ADC, 0x11, E_OPERAND_SIZE_PREFIX, src, dst);}
	void adc(const Reg16& dst, const Mem16& src)	{AppendInstr(I_ADC, 0x13, E_OPERAND_SIZE_PREFIX, dst, src);}
	void adc(const Reg32& dst, const Reg32& src)	{AppendInstr(I_ADC, 0x11, 0, src, dst);}
	void adc(const Mem32& dst, const Reg32& src)	{AppendInstr(I_ADC, 0x11, 0, src, dst);}
	void adc(const Reg32& dst, const Mem32& src)	{AppendInstr(I_ADC, 0x13, 0, dst, src);}
#ifdef JITASM64
	void adc(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_ADC, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(2), dst, detail::ImmXor8(imm));}
	void adc(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_ADC, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(2), dst, detail::ImmXor8(imm));}
	void adc(const Reg64& dst, const Reg64& src)	{AppendInstr(I_ADC, 0x11, E_REXW_PREFIX, src, dst);}
	void adc(const Mem64& dst, const Reg64& src)	{AppendInstr(I_ADC, 0x11, E_REXW_PREFIX, src, dst);}
	void adc(const Reg64& dst, const Mem64& src)	{AppendInstr(I_ADC, 0x13, E_REXW_PREFIX, dst, src);}
#endif
	void add(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_ADD, 0x80, E_SPECIAL, Imm8(0), dst, imm);}
	void add(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_ADD, 0x80, 0, Imm8(0), dst, imm);}
	void add(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_ADD, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(0), dst, detail::ImmXor8(imm));}
	void add(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_ADD, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(0), dst, detail::ImmXor8(imm));}
	void add(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_ADD, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(0), dst, detail::ImmXor8(imm));}
	void add(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_ADD, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(0), dst, detail::ImmXor8(imm));}
	void add(const Reg8& dst, const Reg8& src)		{AppendInstr(I_ADD, 0x00, 0, src, dst);}
	void add(const Mem8& dst, const Reg8& src)		{AppendInstr(I_ADD, 0x00, 0, src, dst);}
	void add(const Reg8& dst, const Mem8& src)		{AppendInstr(I_ADD, 0x02, 0, dst, src);}
	void add(const Reg16& dst, const Reg16& src)	{AppendInstr(I_ADD, 0x01, E_OPERAND_SIZE_PREFIX, src, dst);}
	void add(const Mem16& dst, const Reg16& src)	{AppendInstr(I_ADD, 0x01, E_OPERAND_SIZE_PREFIX, src, dst);}
	void add(const Reg16& dst, const Mem16& src)	{AppendInstr(I_ADD, 0x03, E_OPERAND_SIZE_PREFIX, dst, src);}
	void add(const Reg32& dst, const Reg32& src)	{AppendInstr(I_ADD, 0x01, 0, src, dst);}
	void add(const Mem32& dst, const Reg32& src)	{AppendInstr(I_ADD, 0x01, 0, src, dst);}
	void add(const Reg32& dst, const Mem32& src)	{AppendInstr(I_ADD, 0x03, 0, dst, src);}
#ifdef JITASM64
	void add(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_ADD, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(0), dst, detail::ImmXor8(imm));}
	void add(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_ADD, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(0), dst, detail::ImmXor8(imm));}
	void add(const Reg64& dst, const Reg64& src)	{AppendInstr(I_ADD, 0x01, E_REXW_PREFIX, src, dst);}
	void add(const Mem64& dst, const Reg64& src)	{AppendInstr(I_ADD, 0x01, E_REXW_PREFIX, src, dst);}
	void add(const Reg64& dst, const Mem64& src)	{AppendInstr(I_ADD, 0x03, E_REXW_PREFIX, dst, src);}
#endif
	void and(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_AND, 0x80, E_SPECIAL, Imm8(4), dst, imm);}
	void and(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_AND, 0x80, 0, Imm8(4), dst, imm);}
	void and(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_AND, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(4), dst, detail::ImmXor8(imm));}
	void and(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_AND, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(4), dst, detail::ImmXor8(imm));}
	void and(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_AND, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(4), dst, detail::ImmXor8(imm));}
	void and(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_AND, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(4), dst, detail::ImmXor8(imm));}
	void and(const Reg8& dst, const Reg8& src)		{AppendInstr(I_AND, 0x20, 0, src, dst);}
	void and(const Mem8& dst, const Reg8& src)		{AppendInstr(I_AND, 0x20, 0, src, dst);}
	void and(const Reg8& dst, const Mem8& src)		{AppendInstr(I_AND, 0x22, 0, dst, src);}
	void and(const Reg16& dst, const Reg16& src)	{AppendInstr(I_AND, 0x21, E_OPERAND_SIZE_PREFIX, src, dst);}
	void and(const Mem16& dst, const Reg16& src)	{AppendInstr(I_AND, 0x21, E_OPERAND_SIZE_PREFIX, src, dst);}
	void and(const Reg16& dst, const Mem16& src)	{AppendInstr(I_AND, 0x23, E_OPERAND_SIZE_PREFIX, dst, src);}
	void and(const Reg32& dst, const Reg32& src)	{AppendInstr(I_AND, 0x21, 0, src, dst);}
	void and(const Mem32& dst, const Reg32& src)	{AppendInstr(I_AND, 0x21, 0, src, dst);}
	void and(const Reg32& dst, const Mem32& src)	{AppendInstr(I_AND, 0x23, 0, dst, src);}
#ifdef JITASM64
	void and(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_AND, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(4), dst, detail::ImmXor8(imm));}
	void and(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_AND, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(4), dst, detail::ImmXor8(imm));}
	void and(const Reg64& dst, const Reg64& src)	{AppendInstr(I_AND, 0x21, E_REXW_PREFIX, src, dst);}
	void and(const Mem64& dst, const Reg64& src)	{AppendInstr(I_AND, 0x21, E_REXW_PREFIX, src, dst);}
	void and(const Reg64& dst, const Mem64& src)	{AppendInstr(I_AND, 0x23, E_REXW_PREFIX, dst, src);}
#endif
	void bsf(const Reg16& dst, const Reg16& src)	{AppendInstr(I_BSF, 0x0FBC, E_OPERAND_SIZE_PREFIX, dst, src);}
	void bsf(const Reg16& dst, const Mem16& src)	{AppendInstr(I_BSF, 0x0FBC, E_OPERAND_SIZE_PREFIX, dst, src);}
	void bsf(const Reg32& dst, const Reg32& src)	{AppendInstr(I_BSF, 0x0FBC, 0, dst, src);}
	void bsf(const Reg32& dst, const Mem32& src)	{AppendInstr(I_BSF, 0x0FBC, 0, dst, src);}
#ifdef JITASM64
	void bsf(const Reg64& dst, const Reg64& src)	{AppendInstr(I_BSF, 0x0FBC, E_REXW_PREFIX, dst, src);}
	void bsf(const Reg64& dst, const Mem64& src)	{AppendInstr(I_BSF, 0x0FBC, E_REXW_PREFIX, dst, src);}
#endif
	void bsr(const Reg16& dst, const Reg16& src)	{AppendInstr(I_BSR, 0x0FBD, E_OPERAND_SIZE_PREFIX, dst, src);}
	void bsr(const Reg16& dst, const Mem16& src)	{AppendInstr(I_BSR, 0x0FBD, E_OPERAND_SIZE_PREFIX, dst, src);}
	void bsr(const Reg32& dst, const Reg32& src)	{AppendInstr(I_BSR, 0x0FBD, 0, dst, src);}
	void bsr(const Reg32& dst, const Mem32& src)	{AppendInstr(I_BSR, 0x0FBD, 0, dst, src);}
#ifdef JITASM64
	void bsr(const Reg64& dst, const Reg64& src)	{AppendInstr(I_BSR, 0x0FBD, E_REXW_PREFIX, dst, src);}
	void bsr(const Reg64& dst, const Mem64& src)	{AppendInstr(I_BSR, 0x0FBD, E_REXW_PREFIX, dst, src);}
#endif
	void bswap(const Reg32& dst)	{AppendInstr(I_BSWAP, 0x0FC8, 0, dst);}
#ifdef JITASM64
	void bswap(const Reg64& dst)	{AppendInstr(I_BSWAP, 0x0FC8, E_REXW_PREFIX, dst);}
#endif
	void bt(const Reg16& dst, const Reg16& src)	{AppendInstr(I_BT, 0x0FA3, E_OPERAND_SIZE_PREFIX, src, dst);}
	void bt(const Mem16& dst, const Reg16& src)	{AppendInstr(I_BT, 0x0FA3, E_OPERAND_SIZE_PREFIX, src, dst);}
	void bt(const Reg32& dst, const Reg32& src)	{AppendInstr(I_BT, 0x0FA3, 0, src, dst);}
	void bt(const Mem32& dst, const Reg32& src)	{AppendInstr(I_BT, 0x0FA3, 0, src, dst);}
	void bt(const Reg16& dst, const Imm8& imm)	{AppendInstr(I_BT, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(4), dst, imm);}
	void bt(const Mem16& dst, const Imm8& imm)	{AppendInstr(I_BT, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(4), dst, imm);}
	void bt(const Reg32& dst, const Imm8& imm)	{AppendInstr(I_BT, 0x0FBA, 0, Imm8(4), dst, imm);}
	void bt(const Mem32& dst, const Imm8& imm)	{AppendInstr(I_BT, 0x0FBA, 0, Imm8(4), dst, imm);}
#ifdef JITASM64
	void bt(const Reg64& dst, const Reg64& src)	{AppendInstr(I_BT, 0x0FA3, E_REXW_PREFIX, src, dst);}
	void bt(const Mem64& dst, const Reg64& src)	{AppendInstr(I_BT, 0x0FA3, E_REXW_PREFIX, src, dst);}
	void bt(const Reg64& dst, const Imm8& imm)	{AppendInstr(I_BT, 0x0FBA, E_REXW_PREFIX, Imm8(4), dst, imm);}
	void bt(const Mem64& dst, const Imm8& imm)	{AppendInstr(I_BT, 0x0FBA, E_REXW_PREFIX, Imm8(4), dst, imm);}
#endif
	void btc(const Reg16& dst, const Reg16& src)	{AppendInstr(I_BTC, 0x0FBB, E_OPERAND_SIZE_PREFIX, src, dst);}
	void btc(const Mem16& dst, const Reg16& src)	{AppendInstr(I_BTC, 0x0FBB, E_OPERAND_SIZE_PREFIX, src, dst);}
	void btc(const Reg32& dst, const Reg32& src)	{AppendInstr(I_BTC, 0x0FBB, 0, src, dst);}
	void btc(const Mem32& dst, const Reg32& src)	{AppendInstr(I_BTC, 0x0FBB, 0, src, dst);}
	void btc(const Reg16& dst, const Imm8& imm)		{AppendInstr(I_BTC, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(7), dst, imm);}
	void btc(const Mem16& dst, const Imm8& imm)		{AppendInstr(I_BTC, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(7), dst, imm);}
	void btc(const Reg32& dst, const Imm8& imm)		{AppendInstr(I_BTC, 0x0FBA, 0, Imm8(7), dst, imm);}
	void btc(const Mem32& dst, const Imm8& imm)		{AppendInstr(I_BTC, 0x0FBA, 0, Imm8(7), dst, imm);}
#ifdef JITASM64
	void btc(const Reg64& dst, const Reg64& src)	{AppendInstr(I_BTC, 0x0FBB, E_REXW_PREFIX, src, dst);}
	void btc(const Mem64& dst, const Reg64& src)	{AppendInstr(I_BTC, 0x0FBB, E_REXW_PREFIX, src, dst);}
	void btc(const Reg64& dst, const Imm8& imm)		{AppendInstr(I_BTC, 0x0FBA, E_REXW_PREFIX, Imm8(7), dst, imm);}
	void btc(const Mem64& dst, const Imm8& imm)		{AppendInstr(I_BTC, 0x0FBA, E_REXW_PREFIX, Imm8(7), dst, imm);}
#endif
	void btr(const Reg16& dst, const Reg16& src)	{AppendInstr(I_BTR, 0x0FB3, E_OPERAND_SIZE_PREFIX, src, dst);}
	void btr(const Mem16& dst, const Reg16& src)	{AppendInstr(I_BTR, 0x0FB3, E_OPERAND_SIZE_PREFIX, src, dst);}
	void btr(const Reg32& dst, const Reg32& src)	{AppendInstr(I_BTR, 0x0FB3, 0, src, dst);}
	void btr(const Mem32& dst, const Reg32& src)	{AppendInstr(I_BTR, 0x0FB3, 0, src, dst);}
	void btr(const Reg16& dst, const Imm8& imm)		{AppendInstr(I_BTR, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(6), dst, imm);}
	void btr(const Mem16& dst, const Imm8& imm)		{AppendInstr(I_BTR, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(6), dst, imm);}
	void btr(const Reg32& dst, const Imm8& imm)		{AppendInstr(I_BTR, 0x0FBA, 0, Imm8(6), dst, imm);}
	void btr(const Mem32& dst, const Imm8& imm)		{AppendInstr(I_BTR, 0x0FBA, 0, Imm8(6), dst, imm);}
#ifdef JITASM64
	void btr(const Reg64& dst, const Reg64& src)	{AppendInstr(I_BTR, 0x0FB3, E_REXW_PREFIX, src, dst);}
	void btr(const Mem64& dst, const Reg64& src)	{AppendInstr(I_BTR, 0x0FB3, E_REXW_PREFIX, src, dst);}
	void btr(const Reg64& dst, const Imm8& imm)		{AppendInstr(I_BTR, 0x0FBA, E_REXW_PREFIX, Imm8(6), dst, imm);}
	void btr(const Mem64& dst, const Imm8& imm)		{AppendInstr(I_BTR, 0x0FBA, E_REXW_PREFIX, Imm8(6), dst, imm);}
#endif
	void bts(const Reg16& dst, const Reg16& src)	{AppendInstr(I_BTS, 0x0FAB, E_OPERAND_SIZE_PREFIX, src, dst);}
	void bts(const Mem16& dst, const Reg16& src)	{AppendInstr(I_BTS, 0x0FAB, E_OPERAND_SIZE_PREFIX, src, dst);}
	void bts(const Reg32& dst, const Reg32& src)	{AppendInstr(I_BTS, 0x0FAB, 0, src, dst);}
	void bts(const Mem32& dst, const Reg32& src)	{AppendInstr(I_BTS, 0x0FAB, 0, src, dst);}
	void bts(const Reg16& dst, const Imm8& imm)		{AppendInstr(I_BTS, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(5), dst, imm);}
	void bts(const Mem16& dst, const Imm8& imm)		{AppendInstr(I_BTS, 0x0FBA, E_OPERAND_SIZE_PREFIX, Imm8(5), dst, imm);}
	void bts(const Reg32& dst, const Imm8& imm)		{AppendInstr(I_BTS, 0x0FBA, 0, Imm8(5), dst, imm);}
	void bts(const Mem32& dst, const Imm8& imm)		{AppendInstr(I_BTS, 0x0FBA, 0, Imm8(5), dst, imm);}
#ifdef JITASM64
	void bts(const Reg64& dst, const Reg64& src)	{AppendInstr(I_BTS, 0x0FAB, E_REXW_PREFIX, src, dst);}
	void bts(const Mem64& dst, const Reg64& src)	{AppendInstr(I_BTS, 0x0FAB, E_REXW_PREFIX, src, dst);}
	void bts(const Reg64& dst, const Imm8& imm)		{AppendInstr(I_BTS, 0x0FBA, E_REXW_PREFIX, Imm8(5), dst, imm);}
	void bts(const Mem64& dst, const Imm8& imm)		{AppendInstr(I_BTS, 0x0FBA, E_REXW_PREFIX, Imm8(5), dst, imm);}
#endif
#ifndef JITASM64
	void call(const Reg16& dst)	{AppendInstr(I_CALL, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(2), dst);}
	void call(const Reg32& dst)	{AppendInstr(I_CALL, 0xFF, 0, Imm8(2), dst);}
#else
	void call(const Reg64& dst)	{AppendInstr(I_CALL, 0xFF, 0, Imm8(2), dst);}
#endif
	void cbw()	{AppendInstr(I_CBW,	0x98, E_OPERAND_SIZE_PREFIX);}
	void cwde()	{AppendInstr(I_CBW,	0x98, 0);}
#ifdef JITASM64
	void cdqe()	{AppendInstr(I_CBW,	0x98, E_REXW_PREFIX);}
#endif
	void clc()	{AppendInstr(I_CLC,	0xF8, 0);}
	void cld()	{AppendInstr(I_CLD,	0xFC, 0);}
	void cli()	{AppendInstr(I_CLI,	0xFA, 0);}
#ifdef JITASM64
	void clts()	{AppendInstr(I_CLTS,	0x0F06, 0);}
#endif
	void cmc()	{AppendInstr(I_CMC,	0xF5, 0);}
	void cmova(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F47, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmova(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F47, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovae(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F43, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovae(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F43, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovb(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F42, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovb(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F42, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovbe(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F46, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovbe(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F46, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovc(const Reg16& dst, const Reg16& src)		{cmovb(dst, src);}
	void cmovc(const Reg16& dst, const Mem16& src)		{cmovb(dst, src);}
	void cmove(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F44, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmove(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F44, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovg(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F4F, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovg(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F4F, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovge(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F4D, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovge(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F4D, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovl(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F4C, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovl(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F4C, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovle(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F4E, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovle(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F4E, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovna(const Reg16& dst, const Reg16& src)		{cmovbe(dst, src);}
	void cmovna(const Reg16& dst, const Mem16& src)		{cmovbe(dst, src);}
	void cmovnae(const Reg16& dst, const Reg16& src)	{cmovb(dst, src);}
	void cmovnae(const Reg16& dst, const Mem16& src)	{cmovb(dst, src);}
	void cmovnb(const Reg16& dst, const Reg16& src)		{cmovae(dst, src);}
	void cmovnb(const Reg16& dst, const Mem16& src)		{cmovae(dst, src);}
	void cmovnbe(const Reg16& dst, const Reg16& src)	{cmova(dst, src);}
	void cmovnbe(const Reg16& dst, const Mem16& src)	{cmova(dst, src);}
	void cmovnc(const Reg16& dst, const Reg16& src)		{cmovae(dst, src);}
	void cmovnc(const Reg16& dst, const Mem16& src)		{cmovae(dst, src);}
	void cmovne(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F45, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovne(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F45, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovng(const Reg16& dst, const Reg16& src)		{cmovle(dst, src);}
	void cmovng(const Reg16& dst, const Mem16& src)		{cmovle(dst, src);}
	void cmovnge(const Reg16& dst, const Reg16& src)	{cmovl(dst, src);}
	void cmovnge(const Reg16& dst, const Mem16& src)	{cmovl(dst, src);}
	void cmovnl(const Reg16& dst, const Reg16& src)		{cmovge(dst, src);}
	void cmovnl(const Reg16& dst, const Mem16& src)		{cmovge(dst, src);}
	void cmovnle(const Reg16& dst, const Reg16& src)	{cmovg(dst, src);}
	void cmovnle(const Reg16& dst, const Mem16& src)	{cmovg(dst, src);}
	void cmovno(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F41, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovno(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F41, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovnp(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F4B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovnp(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F4B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovns(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F49, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovns(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F49, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovnz(const Reg16& dst, const Reg16& src)		{cmovne(dst, src);}
	void cmovnz(const Reg16& dst, const Mem16& src)		{cmovne(dst, src);}
	void cmovo(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F40, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovo(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F40, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovp(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F4A, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovp(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F4A, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovpe(const Reg16& dst, const Reg16& src)		{cmovp(dst, src);}
	void cmovpe(const Reg16& dst, const Mem16& src)		{cmovp(dst, src);}
	void cmovpo(const Reg16& dst, const Reg16& src)		{cmovnp(dst, src);}
	void cmovpo(const Reg16& dst, const Mem16& src)		{cmovnp(dst, src);}
	void cmovs(const Reg16& dst, const Reg16& src)		{AppendInstr(I_CMOVCC, 0x0F48, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovs(const Reg16& dst, const Mem16& src)		{AppendInstr(I_CMOVCC, 0x0F48, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmovz(const Reg16& dst, const Reg16& src)		{cmove(dst, src);}
	void cmovz(const Reg16& dst, const Mem16& src)		{cmove(dst, src);}
	void cmova(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F47, 0, dst, src);}
	void cmova(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F47, 0, dst, src);}
	void cmovae(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F43, 0, dst, src);}
	void cmovae(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F43, 0, dst, src);}
	void cmovb(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F42, 0, dst, src);}
	void cmovb(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F42, 0, dst, src);}
	void cmovbe(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F46, 0, dst, src);}
	void cmovbe(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F46, 0, dst, src);}
	void cmovc(const Reg32& dst, const Reg32& src)		{cmovb(dst, src);}
	void cmovc(const Reg32& dst, const Mem32& src)		{cmovb(dst, src);}
	void cmove(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F44, 0, dst, src);}
	void cmove(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F44, 0, dst, src);}
	void cmovg(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F4F, 0, dst, src);}
	void cmovg(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F4F, 0, dst, src);}
	void cmovge(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F4D, 0, dst, src);}
	void cmovge(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F4D, 0, dst, src);}
	void cmovl(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F4C, 0, dst, src);}
	void cmovl(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F4C, 0, dst, src);}
	void cmovle(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F4E, 0, dst, src);}
	void cmovle(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F4E, 0, dst, src);}
	void cmovna(const Reg32& dst, const Reg32& src)		{cmovbe(dst, src);}
	void cmovna(const Reg32& dst, const Mem32& src)		{cmovbe(dst, src);}
	void cmovnae(const Reg32& dst, const Reg32& src)	{cmovb(dst, src);}
	void cmovnae(const Reg32& dst, const Mem32& src)	{cmovb(dst, src);}
	void cmovnb(const Reg32& dst, const Reg32& src)		{cmovae(dst, src);}
	void cmovnb(const Reg32& dst, const Mem32& src)		{cmovae(dst, src);}
	void cmovnbe(const Reg32& dst, const Reg32& src)	{cmova(dst, src);}
	void cmovnbe(const Reg32& dst, const Mem32& src)	{cmova(dst, src);}
	void cmovnc(const Reg32& dst, const Reg32& src)		{cmovae(dst, src);}
	void cmovnc(const Reg32& dst, const Mem32& src)		{cmovae(dst, src);}
	void cmovne(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F45, 0, dst, src);}
	void cmovne(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F45, 0, dst, src);}
	void cmovng(const Reg32& dst, const Reg32& src)		{cmovle(dst, src);}
	void cmovng(const Reg32& dst, const Mem32& src)		{cmovle(dst, src);}
	void cmovnge(const Reg32& dst, const Reg32& src)	{cmovl(dst, src);}
	void cmovnge(const Reg32& dst, const Mem32& src)	{cmovl(dst, src);}
	void cmovnl(const Reg32& dst, const Reg32& src)		{cmovge(dst, src);}
	void cmovnl(const Reg32& dst, const Mem32& src)		{cmovge(dst, src);}
	void cmovnle(const Reg32& dst, const Reg32& src)	{cmovg(dst, src);}
	void cmovnle(const Reg32& dst, const Mem32& src)	{cmovg(dst, src);}
	void cmovno(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F41, 0, dst, src);}
	void cmovno(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F41, 0, dst, src);}
	void cmovnp(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F4B, 0, dst, src);}
	void cmovnp(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F4B, 0, dst, src);}
	void cmovns(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F49, 0, dst, src);}
	void cmovns(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F49, 0, dst, src);}
	void cmovnz(const Reg32& dst, const Reg32& src)		{cmovne(dst, src);}
	void cmovnz(const Reg32& dst, const Mem32& src)		{cmovne(dst, src);}
	void cmovo(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F40, 0, dst, src);}
	void cmovo(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F40, 0, dst, src);}
	void cmovp(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F4A, 0, dst, src);}
	void cmovp(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F4A, 0, dst, src);}
	void cmovpe(const Reg32& dst, const Reg32& src)		{cmovp(dst, src);}
	void cmovpe(const Reg32& dst, const Mem32& src)		{cmovp(dst, src);}
	void cmovpo(const Reg32& dst, const Reg32& src)		{cmovnp(dst, src);}
	void cmovpo(const Reg32& dst, const Mem32& src)		{cmovnp(dst, src);}
	void cmovs(const Reg32& dst, const Reg32& src)		{AppendInstr(I_CMOVCC, 0x0F48, 0, dst, src);}
	void cmovs(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CMOVCC, 0x0F48, 0, dst, src);}
	void cmovz(const Reg32& dst, const Reg32& src)		{cmove(dst, src);}
	void cmovz(const Reg32& dst, const Mem32& src)		{cmove(dst, src);}
#ifdef JITASM64
	void cmova(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F47, E_REXW_PREFIX, dst, src);}
	void cmova(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F47, E_REXW_PREFIX, dst, src);}
	void cmovae(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F43, E_REXW_PREFIX, dst, src);}
	void cmovae(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F43, E_REXW_PREFIX, dst, src);}
	void cmovb(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F42, E_REXW_PREFIX, dst, src);}
	void cmovb(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F42, E_REXW_PREFIX, dst, src);}
	void cmovbe(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F46, E_REXW_PREFIX, dst, src);}
	void cmovbe(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F46, E_REXW_PREFIX, dst, src);}
	void cmovc(const Reg64& dst, const Reg64& src)		{cmovb(dst, src);}
	void cmovc(const Reg64& dst, const Mem64& src)		{cmovb(dst, src);}
	void cmove(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F44, E_REXW_PREFIX, dst, src);}
	void cmove(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F44, E_REXW_PREFIX, dst, src);}
	void cmovg(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F4F, E_REXW_PREFIX, dst, src);}
	void cmovg(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F4F, E_REXW_PREFIX, dst, src);}
	void cmovge(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F4D, E_REXW_PREFIX, dst, src);}
	void cmovge(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F4D, E_REXW_PREFIX, dst, src);}
	void cmovl(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F4C, E_REXW_PREFIX, dst, src);}
	void cmovl(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F4C, E_REXW_PREFIX, dst, src);}
	void cmovle(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F4E, E_REXW_PREFIX, dst, src);}
	void cmovle(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F4E, E_REXW_PREFIX, dst, src);}
	void cmovna(const Reg64& dst, const Reg64& src)		{cmovbe(dst, src);}
	void cmovna(const Reg64& dst, const Mem64& src)		{cmovbe(dst, src);}
	void cmovnae(const Reg64& dst, const Reg64& src)	{cmovb(dst, src);}
	void cmovnae(const Reg64& dst, const Mem64& src)	{cmovb(dst, src);}
	void cmovnb(const Reg64& dst, const Reg64& src)		{cmovae(dst, src);}
	void cmovnb(const Reg64& dst, const Mem64& src)		{cmovae(dst, src);}
	void cmovnbe(const Reg64& dst, const Reg64& src)	{cmova(dst, src);}
	void cmovnbe(const Reg64& dst, const Mem64& src)	{cmova(dst, src);}
	void cmovnc(const Reg64& dst, const Reg64& src)		{cmovae(dst, src);}
	void cmovnc(const Reg64& dst, const Mem64& src)		{cmovae(dst, src);}
	void cmovne(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F45, E_REXW_PREFIX, dst, src);}
	void cmovne(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F45, E_REXW_PREFIX, dst, src);}
	void cmovng(const Reg64& dst, const Reg64& src)		{cmovle(dst, src);}
	void cmovng(const Reg64& dst, const Mem64& src)		{cmovle(dst, src);}
	void cmovnge(const Reg64& dst, const Reg64& src)	{cmovl(dst, src);}
	void cmovnge(const Reg64& dst, const Mem64& src)	{cmovl(dst, src);}
	void cmovnl(const Reg64& dst, const Reg64& src)		{cmovge(dst, src);}
	void cmovnl(const Reg64& dst, const Mem64& src)		{cmovge(dst, src);}
	void cmovnle(const Reg64& dst, const Reg64& src)	{cmovg(dst, src);}
	void cmovnle(const Reg64& dst, const Mem64& src)	{cmovg(dst, src);}
	void cmovno(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F41, E_REXW_PREFIX, dst, src);}
	void cmovno(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F41, E_REXW_PREFIX, dst, src);}
	void cmovnp(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F4B, E_REXW_PREFIX, dst, src);}
	void cmovnp(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F4B, E_REXW_PREFIX, dst, src);}
	void cmovns(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F49, E_REXW_PREFIX, dst, src);}
	void cmovns(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F49, E_REXW_PREFIX, dst, src);}
	void cmovnz(const Reg64& dst, const Reg64& src)		{cmovne(dst, src);}
	void cmovnz(const Reg64& dst, const Mem64& src)		{cmovne(dst, src);}
	void cmovo(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F40, E_REXW_PREFIX, dst, src);}
	void cmovo(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F40, E_REXW_PREFIX, dst, src);}
	void cmovp(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F4A, E_REXW_PREFIX, dst, src);}
	void cmovp(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F4A, E_REXW_PREFIX, dst, src);}
	void cmovpe(const Reg64& dst, const Reg64& src)		{cmovp(dst, src);}
	void cmovpe(const Reg64& dst, const Mem64& src)		{cmovp(dst, src);}
	void cmovpo(const Reg64& dst, const Reg64& src)		{cmovnp(dst, src);}
	void cmovpo(const Reg64& dst, const Mem64& src)		{cmovnp(dst, src);}
	void cmovs(const Reg64& dst, const Reg64& src)		{AppendInstr(I_CMOVCC, 0x0F48, E_REXW_PREFIX, dst, src);}
	void cmovs(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CMOVCC, 0x0F48, E_REXW_PREFIX, dst, src);}
	void cmovz(const Reg64& dst, const Reg64& src)		{cmove(dst, src);}
	void cmovz(const Reg64& dst, const Mem64& src)		{cmove(dst, src);}
#endif
	void cmp(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_CMP, 0x80, E_SPECIAL, Imm8(7), dst, imm);}
	void cmp(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_CMP, 0x80, 0, Imm8(7), dst, imm);}
	void cmp(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(7), dst, detail::ImmXor8(imm));}
	void cmp(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(7), dst, detail::ImmXor8(imm));}
	void cmp(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(7), dst, detail::ImmXor8(imm));}
	void cmp(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(7), dst, detail::ImmXor8(imm));}
	void cmp(const Reg8& dst, const Reg8& src)		{AppendInstr(I_CMP, 0x38, 0, src, dst);}
	void cmp(const Mem8& dst, const Reg8& src)		{AppendInstr(I_CMP, 0x38, 0, src, dst);}
	void cmp(const Reg8& dst, const Mem8& src)		{AppendInstr(I_CMP, 0x3A, 0, dst, src);}
	void cmp(const Reg16& dst, const Reg16& src)	{AppendInstr(I_CMP, 0x39, E_OPERAND_SIZE_PREFIX, src, dst);}
	void cmp(const Mem16& dst, const Reg16& src)	{AppendInstr(I_CMP, 0x39, E_OPERAND_SIZE_PREFIX, src, dst);}
	void cmp(const Reg16& dst, const Mem16& src)	{AppendInstr(I_CMP, 0x3B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void cmp(const Reg32& dst, const Reg32& src)	{AppendInstr(I_CMP, 0x39, 0, src, dst);}
	void cmp(const Mem32& dst, const Reg32& src)	{AppendInstr(I_CMP, 0x39, 0, src, dst);}
	void cmp(const Reg32& dst, const Mem32& src)	{AppendInstr(I_CMP, 0x3B, 0, dst, src);}
#ifdef JITASM64
	void cmp(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(7), dst, detail::ImmXor8(imm));}
	void cmp(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(7), dst, detail::ImmXor8(imm));}
	void cmp(const Reg64& dst, const Reg64& src)	{AppendInstr(I_CMP, 0x39, E_REXW_PREFIX, src, dst);}
	void cmp(const Mem64& dst, const Reg64& src)	{AppendInstr(I_CMP, 0x39, E_REXW_PREFIX, src, dst);}
	void cmp(const Reg64& dst, const Mem64& src)	{AppendInstr(I_CMP, 0x3B, E_REXW_PREFIX, dst, src);}
#endif
	void cmpxchg(const Reg8& dst, const Reg8& src)		{AppendInstr(I_CMPXCHG, 0x0FB0, 0, src, dst);}
	void cmpxchg(const Mem8& dst, const Reg8& src)		{AppendInstr(I_CMPXCHG, 0x0FB0, 0, src, dst);}
	void cmpxchg(const Reg16& dst, const Reg16& src)	{AppendInstr(I_CMPXCHG, 0x0FB1, E_OPERAND_SIZE_PREFIX, src, dst);}
	void cmpxchg(const Mem16& dst, const Reg16& src)	{AppendInstr(I_CMPXCHG, 0x0FB1, E_OPERAND_SIZE_PREFIX, src, dst);}
	void cmpxchg(const Reg32& dst, const Reg32& src)	{AppendInstr(I_CMPXCHG, 0x0FB1, 0, src, dst);}
	void cmpxchg(const Mem32& dst, const Reg32& src)	{AppendInstr(I_CMPXCHG, 0x0FB1, 0, src, dst);}
#ifdef JITASM64
	void cmpxchg(const Reg64& dst, const Reg64& src)	{AppendInstr(I_CMPXCHG, 0x0FB1, E_REXW_PREFIX, src, dst);}
	void cmpxchg(const Mem64& dst, const Reg64& src)	{AppendInstr(I_CMPXCHG, 0x0FB1, E_REXW_PREFIX, src, dst);}
#endif
	void cmpxchg8b(const Mem64& dst)	{AppendInstr(I_CMPXCHG8B, 0x0FC7, 0, Imm8(1), dst);}
#ifdef JITASM64
	void cmpxchg16b(const Mem128& dst)	{AppendInstr(I_CMPXCHG16B, 0x0FC7, E_REXW_PREFIX, Imm8(1), dst);}
#endif
	void cpuid()	{AppendInstr(I_CPUID, 0x0FA2, 0);}
	void cwd()		{AppendInstr(I_CWD, 0x99, E_OPERAND_SIZE_PREFIX);}
	void cdq()		{AppendInstr(I_CDQ, 0x99, 0);}
#ifdef JITASM64
	void cqo()		{AppendInstr(I_CQO, 0x99, E_REXW_PREFIX);}
#endif
	void dec(const Reg8& dst)	{AppendInstr(I_DEC, 0xFE, 0, Imm8(1), dst);}
	void dec(const Mem8& dst)	{AppendInstr(I_DEC, 0xFE, 0, Imm8(1), dst);}
	void dec(const Mem16& dst)	{AppendInstr(I_DEC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(1), dst);}
	void dec(const Mem32& dst)	{AppendInstr(I_DEC, 0xFF, 0, Imm8(1), dst);}
#ifndef JITASM64
	void dec(const Reg16& dst)	{AppendInstr(I_DEC, 0x48, E_OPERAND_SIZE_PREFIX, dst);}
	void dec(const Reg32& dst)	{AppendInstr(I_DEC, 0x48, 0, dst);}
#else
	void dec(const Reg16& dst)	{AppendInstr(I_DEC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(1), dst);}
	void dec(const Reg32& dst)	{AppendInstr(I_DEC, 0xFF, 0, Imm8(1), dst);}
	void dec(const Reg64& dst)	{AppendInstr(I_DEC, 0xFF, E_REXW_PREFIX, Imm8(1), dst);}
	void dec(const Mem64& dst)	{AppendInstr(I_DEC, 0xFF, E_REXW_PREFIX, Imm8(1), dst);}
#endif
	void div(const Reg8& dst)	{AppendInstr(I_DIV, 0xF6, 0, Imm8(6), dst);}
	void div(const Mem8& dst)	{AppendInstr(I_DIV, 0xF6, 0, Imm8(6), dst);}
	void div(const Reg16& dst)	{AppendInstr(I_DIV, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(6), dst);}
	void div(const Mem16& dst)	{AppendInstr(I_DIV, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(6), dst);}
	void div(const Reg32& dst)	{AppendInstr(I_DIV, 0xF7, 0, Imm8(6), dst);}
	void div(const Mem32& dst)	{AppendInstr(I_DIV, 0xF7, 0, Imm8(6), dst);}
#ifdef JITASM64
	void div(const Reg64& dst)	{AppendInstr(I_DIV, 0xF7, E_REXW_PREFIX, Imm8(6), dst);}
	void div(const Mem64& dst)	{AppendInstr(I_DIV, 0xF7, E_REXW_PREFIX, Imm8(6), dst);}
#endif
	void enter(const Imm16& imm16, const Imm8& imm8) {AppendInstr(I_ENTER, 0xC8, 0, imm16, imm8);}
	void hlt()	{AppendInstr(I_HLT, 0xF4, 0);}
	void idiv(const Reg8& dst)	{AppendInstr(I_IDIV, 0xF6, 0, Imm8(7), dst);}
	void idiv(const Mem8& dst)	{AppendInstr(I_IDIV, 0xF6, 0, Imm8(7), dst);}
	void idiv(const Reg16& dst)	{AppendInstr(I_IDIV, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(7), dst);}
	void idiv(const Mem16& dst)	{AppendInstr(I_IDIV, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(7), dst);}
	void idiv(const Reg32& dst)	{AppendInstr(I_IDIV, 0xF7, 0, Imm8(7), dst);}
	void idiv(const Mem32& dst)	{AppendInstr(I_IDIV, 0xF7, 0, Imm8(7), dst);}
#ifdef JITASM64
	void idiv(const Reg64& dst)	{AppendInstr(I_IDIV, 0xF7, E_REXW_PREFIX, Imm8(7), dst);}
	void idiv(const Mem64& dst)	{AppendInstr(I_IDIV, 0xF7, E_REXW_PREFIX, Imm8(7), dst);}
#endif
	void imul(const Reg8& dst)										{AppendInstr(I_IMUL, 0xF6, 0, Imm8(5), dst);}
	void imul(const Mem8& dst)										{AppendInstr(I_IMUL, 0xF6, 0, Imm8(5), dst);}
	void imul(const Reg16& dst)										{AppendInstr(I_IMUL, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(5), dst);}
	void imul(const Mem16& dst)										{AppendInstr(I_IMUL, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(5), dst);}
	void imul(const Reg32& dst)										{AppendInstr(I_IMUL, 0xF7, 0, Imm8(5), dst);}
	void imul(const Mem32& dst)										{AppendInstr(I_IMUL, 0xF7, 0, Imm8(5), dst);}
	void imul(const Reg16& dst, const Reg16& src)					{AppendInstr(I_IMUL, 0x0FAF, E_OPERAND_SIZE_PREFIX, dst, src);}
	void imul(const Reg16& dst, const Mem16& src)					{AppendInstr(I_IMUL, 0x0FAF, E_OPERAND_SIZE_PREFIX, dst, src);}
	void imul(const Reg32& dst, const Reg32& src)					{AppendInstr(I_IMUL, 0x0FAF, 0, dst, src);}
	void imul(const Reg32& dst, const Mem32& src)					{AppendInstr(I_IMUL, 0x0FAF, 0, dst, src);}
	void imul(const Reg16& dst, const Reg16& src, const Imm16& imm)	{AppendInstr(I_IMUL, detail::IsInt8(imm.GetImm()) ? 0x6B : 0x69, E_OPERAND_SIZE_PREFIX, dst, src, detail::ImmXor8(imm));}
	void imul(const Reg16& dst, const Mem16& src, const Imm16& imm)	{AppendInstr(I_IMUL, detail::IsInt8(imm.GetImm()) ? 0x6B : 0x69, E_OPERAND_SIZE_PREFIX, dst, src, detail::ImmXor8(imm));}
	void imul(const Reg32& dst, const Reg32& src, const Imm32& imm)	{AppendInstr(I_IMUL, detail::IsInt8(imm.GetImm()) ? 0x6B : 0x69, 0, dst, src, detail::ImmXor8(imm));}
	void imul(const Reg32& dst, const Mem32& src, const Imm32& imm)	{AppendInstr(I_IMUL, detail::IsInt8(imm.GetImm()) ? 0x6B : 0x69, 0, dst, src, detail::ImmXor8(imm));}
	void imul(const Reg16& dst, const Imm16& imm)					{imul(dst, dst, imm);}
	void imul(const Reg32& dst, const Imm32& imm)					{imul(dst, dst, imm);}
#ifdef JITASM64
	void imul(const Reg64& dst)										{AppendInstr(I_IMUL, 0xF7, E_REXW_PREFIX, Imm8(5), dst);}
	void imul(const Mem64& dst)										{AppendInstr(I_IMUL, 0xF7, E_REXW_PREFIX, Imm8(5), dst);}
	void imul(const Reg64& dst, const Reg64& src)					{AppendInstr(I_IMUL, 0x0FAF, E_REXW_PREFIX, dst, src);}
	void imul(const Reg64& dst, const Mem64& src)					{AppendInstr(I_IMUL, 0x0FAF, E_REXW_PREFIX, dst, src);}
	void imul(const Reg64& dst, const Reg64& src, const Imm32& imm)	{AppendInstr(I_IMUL, detail::IsInt8(imm.GetImm()) ? 0x6B : 0x69, E_REXW_PREFIX, dst, src, detail::ImmXor8(imm));}
	void imul(const Reg64& dst, const Mem64& src, const Imm32& imm)	{AppendInstr(I_IMUL, detail::IsInt8(imm.GetImm()) ? 0x6B : 0x69, E_REXW_PREFIX, dst, src, detail::ImmXor8(imm));}
	void imul(const Reg64& dst, const Imm32& imm)					{imul(dst, dst, imm);}
#endif
	void inc(const Reg8& dst)	{AppendInstr(I_INC, 0xFE, 0, Imm8(0), dst);}
	void inc(const Mem8& dst)	{AppendInstr(I_INC, 0xFE, 0, Imm8(0), dst);}
	void inc(const Mem16& dst)	{AppendInstr(I_INC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(0), dst);}
	void inc(const Mem32& dst)	{AppendInstr(I_INC, 0xFF, 0, Imm8(0), dst);}
#ifndef JITASM64
	void inc(const Reg16& dst)	{AppendInstr(I_INC, 0x40, E_OPERAND_SIZE_PREFIX, dst);}
	void inc(const Reg32& dst)	{AppendInstr(I_INC, 0x40, 0, dst);}
#else
	void inc(const Reg16& dst)	{AppendInstr(I_INC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(0), dst);}
	void inc(const Reg32& dst)	{AppendInstr(I_INC, 0xFF, 0, Imm8(0), dst);}
	void inc(const Reg64& dst)	{AppendInstr(I_INC, 0xFF, E_REXW_PREFIX, Imm8(0), dst);}
	void inc(const Mem64& dst)	{AppendInstr(I_INC, 0xFF, E_REXW_PREFIX, Imm8(0), dst);}
#endif
	void int3()	{AppendInstr(I_INT3, 0xCC, 0);}
	void invd()	{AppendInstr(I_INVD, 0x0F08, 0);}
	template<class Ty> void invlpg(const MemT<Ty>& dst)	{AppendInstr(I_INVLPG, 0x0F01, 0, Imm8(7), dst);}
	void iret()		{AppendInstr(I_IRET, 0xCF, E_OPERAND_SIZE_PREFIX);}
	void iretd()	{AppendInstr(I_IRETD, 0xCF, 0);}
#ifdef JITASM64
	void iretq()	{AppendInstr(I_IRETQ, 0xCF, E_REXW_PREFIX);}
#endif
	void jmp(const std::string& label_name)		{AppendJmp(GetLabelID(label_name));}
	void ja(const std::string& label_name)		{AppendJcc(JCC_A, GetLabelID(label_name));}
	void jae(const std::string& label_name)		{AppendJcc(JCC_AE, GetLabelID(label_name));}
	void jb(const std::string& label_name)		{AppendJcc(JCC_B, GetLabelID(label_name));}
	void jbe(const std::string& label_name)		{AppendJcc(JCC_BE, GetLabelID(label_name));}
	void jc(const std::string& label_name)		{jb(label_name);}
#ifdef JITASM64
	void jecxz(const std::string& label_name)	{AppendJcc(JCC_ECXZ, GetLabelID(label_name));}	// short jump only
	void jrcxz (const std::string& label_name)	{AppendJcc(JCC_RCXZ, GetLabelID(label_name));}	// short jump only
#else
	void jcxz(const std::string& label_name)	{AppendJcc(JCC_CXZ, GetLabelID(label_name));}	// short jump only
	void jecxz(const std::string& label_name)	{AppendJcc(JCC_ECXZ, GetLabelID(label_name));}	// short jump only
#endif
	void je(const std::string& label_name)		{AppendJcc(JCC_E, GetLabelID(label_name));}
	void jg(const std::string& label_name)		{AppendJcc(JCC_G, GetLabelID(label_name));}
	void jge(const std::string& label_name)		{AppendJcc(JCC_GE, GetLabelID(label_name));}
	void jl(const std::string& label_name)		{AppendJcc(JCC_L, GetLabelID(label_name));}
	void jle(const std::string& label_name)		{AppendJcc(JCC_LE, GetLabelID(label_name));}
	void jna(const std::string& label_name)		{jbe(label_name);}
	void jnae(const std::string& label_name)	{jb(label_name);}
	void jnb(const std::string& label_name)		{jae(label_name);}
	void jnbe(const std::string& label_name)	{ja(label_name);}
	void jnc(const std::string& label_name)		{jae(label_name);}
	void jne(const std::string& label_name)		{AppendJcc(JCC_NE, GetLabelID(label_name));}
	void jng(const std::string& label_name)		{jle(label_name);}
	void jnge(const std::string& label_name)	{jl(label_name);}
	void jnl(const std::string& label_name)		{jge(label_name);}
	void jnle(const std::string& label_name)	{jg(label_name);}
	void jno(const std::string& label_name)		{AppendJcc(JCC_NO, GetLabelID(label_name));}
	void jnp(const std::string& label_name)		{AppendJcc(JCC_NP, GetLabelID(label_name));}
	void jns(const std::string& label_name)		{AppendJcc(JCC_NS, GetLabelID(label_name));}
	void jnz(const std::string& label_name)		{jne(label_name);}
	void jo(const std::string& label_name)		{AppendJcc(JCC_O, GetLabelID(label_name));}
	void jp(const std::string& label_name)		{AppendJcc(JCC_P, GetLabelID(label_name));}
	void jpe(const std::string& label_name)		{jp(label_name);}
	void jpo(const std::string& label_name)		{jnp(label_name);}
	void js(const std::string& label_name)		{AppendJcc(JCC_S, GetLabelID(label_name));}
	void jz(const std::string& label_name)		{je(label_name);}
	void lar(const Reg16& dst, const Reg16& src)	{AppendInstr(I_LAR, 0x0F02, E_OPERAND_SIZE_PREFIX, dst, src);}
	void lar(const Reg16& dst, const Mem16& src)	{AppendInstr(I_LAR, 0x0F02, E_OPERAND_SIZE_PREFIX, dst, src);}
	void lar(const Reg32& dst, const Reg32& src)	{AppendInstr(I_LAR, 0x0F02, 0, dst, src);}
	void lar(const Reg32& dst, const Mem16& src)	{AppendInstr(I_LAR, 0x0F02, 0, dst, src);}
#ifdef JITASM64
	void lar(const Reg64& dst, const Reg64& src)	{AppendInstr(I_LAR, 0x0F02, E_REXW_PREFIX, dst, src);}
	void lar(const Reg64& dst, const Mem16& src)	{AppendInstr(I_LAR, 0x0F02, E_REXW_PREFIX, dst, src);}
#endif
	template<class Ty> void lea(const Reg16& dst, const MemT<Ty>& src)	{AppendInstr(I_LEA, 0x8D, E_OPERAND_SIZE_PREFIX, dst, src);}
	template<class Ty> void lea(const Reg32& dst, const MemT<Ty>& src)	{AppendInstr(I_LEA, 0x8D, 0, dst, src);}
#ifdef JITASM64
	template<class Ty> void lea(const Reg64& dst, const MemT<Ty>& src)	{AppendInstr(I_LEA, 0x8D, E_REXW_PREFIX, dst, src);}
#endif
	void leave()		{AppendInstr(I_LEAVE, 0xC9, 0);}
	void lodsb()		{AppendInstr(I_LODS_B, 0xAC, 0);}
	void lodsw()		{AppendInstr(I_LODS_W, 0xAD, E_OPERAND_SIZE_PREFIX);}
	void lodsd()		{AppendInstr(I_LODS_D, 0xAD, 0);}
#ifdef JITASM64
	void lodsq()		{AppendInstr(I_LODS_Q, 0xAD, E_REXW_PREFIX);}
#endif
	void loop(const std::string& label_name)	{AppendInstr(I_LOOP, 0xE2, E_SPECIAL, Imm64(GetLabelID(label_name)));}	// short jump only
	void loope(const std::string& label_name)	{AppendInstr(I_LOOP, 0xE1, E_SPECIAL, Imm64(GetLabelID(label_name)));}	// short jump only
	void loopne(const std::string& label_name)	{AppendInstr(I_LOOP, 0xE0, E_SPECIAL, Imm64(GetLabelID(label_name)));}	// short jump only
	void rep_lodsb()	{AppendInstr(I_LODS_B, 0xAC, E_REPEAT_PREFIX);}
	void rep_lodsw()	{AppendInstr(I_LODS_W, 0xAD, E_REPEAT_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_lodsd()	{AppendInstr(I_LODS_D, 0xAD, E_REPEAT_PREFIX);}
#ifdef JITASM64
	void rep_lodsq()	{AppendInstr(I_LODS_Q, 0xAD, E_REPEAT_PREFIX | E_REXW_PREFIX);}
#endif
	void mov(const Reg8& dst, const Reg8& src)		{AppendInstr(I_MOV, 0x8A, 0, dst, src);}
	void mov(const Mem8& dst, const Reg8& src)		{AppendInstr(I_MOV, 0x88, E_SPECIAL, src, dst);}
	void mov(const Reg16& dst, const Reg16& src)	{AppendInstr(I_MOV, 0x8B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void mov(const Mem16& dst, const Reg16& src)	{AppendInstr(I_MOV, 0x89, E_OPERAND_SIZE_PREFIX | E_SPECIAL, src, dst);}
	void mov(const Reg32& dst, const Reg32& src)	{AppendInstr(I_MOV, 0x8B, 0, dst, src);}
	void mov(const Mem32& dst, const Reg32& src)	{AppendInstr(I_MOV, 0x89, E_SPECIAL, src, dst);}
	void mov(const Reg8& dst, const Mem8& src)		{AppendInstr(I_MOV, 0x8A, E_SPECIAL, dst, src);}
	void mov(const Reg16& dst, const Mem16& src)	{AppendInstr(I_MOV, 0x8B, E_OPERAND_SIZE_PREFIX | E_SPECIAL, dst, src);}
	void mov(const Reg32& dst, const Mem32& src)	{AppendInstr(I_MOV, 0x8B, E_SPECIAL, dst, src);}
	void mov(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_MOV, 0xB0, 0, dst, imm);}
	void mov(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_MOV, 0xB8, E_OPERAND_SIZE_PREFIX, dst, imm);}
	void mov(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_MOV, 0xB8, 0, dst, imm);}
	void mov(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_MOV, 0xC6, 0, Imm8(0), dst, imm);}
	void mov(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_MOV, 0xC7, E_OPERAND_SIZE_PREFIX, Imm8(0), dst, imm);}
	void mov(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_MOV, 0xC7, 0, Imm8(0), dst, imm);}
#ifdef JITASM64
	void mov(const Reg64& dst, const Reg64& src)	{AppendInstr(I_MOV, 0x8B, E_REXW_PREFIX, dst, src);}
	void mov(const Mem64& dst, const Reg64& src)	{AppendInstr(I_MOV, 0x89, E_REXW_PREFIX, src, dst);}
	void mov(const Reg64& dst, const Mem64& src)	{AppendInstr(I_MOV, 0x8B, E_REXW_PREFIX, dst, src);}
	void mov(const Reg64& dst, const Imm64& imm)	{detail::IsInt32(imm.GetImm()) ? AppendInstr(I_MOV, 0xC7, E_REXW_PREFIX, Imm8(0), dst, Imm32((sint32) imm.GetImm())) : AppendInstr(I_MOV, 0xB8, E_REXW_PREFIX, dst, imm);}
	void mov(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_MOV, 0xC7, E_REXW_PREFIX, Imm8(0), dst, imm);}
	void mov(const Reg64_rax& dst, const MemOffset64& src)	{AppendInstr(I_MOV, 0xA1, E_REXW_PREFIX, Imm64(src.GetOffset()));}
	void mov(const MemOffset64& dst, const Reg64_rax& src)	{AppendInstr(I_MOV, 0xA3, E_REXW_PREFIX, Imm64(dst.GetOffset()));}
#endif
	//void movbe(const Reg16& dst, const Mem16& src)	{AppendInstr(I_MOVBE, 0x0F38F0, E_OPERAND_SIZE_PREFIX, dst, src);}
	//void movbe(const Reg32& dst, const Mem32& src)	{AppendInstr(I_MOVBE, 0x0F38F0, 0, dst, src);}
	//void movbe(const Mem16& dst, const Reg16& src)	{AppendInstr(I_MOVBE, 0x0F38F1, E_OPERAND_SIZE_PREFIX, src, dst);}
	//void movbe(const Mem32& dst, const Reg32& src)	{AppendInstr(I_MOVBE, 0x0F38F1, 0, src, dst);}
#ifdef JITASM64
	//void movbe(const Reg64& dst, const Mem64& src)	{AppendInstr(I_MOVBE, 0x0F38F0, E_REXW_PREFIX, dst, src);}
	//void movbe(const Mem64& dst, const Reg64& src)	{AppendInstr(I_MOVBE, 0x0F38F1, E_REXW_PREFIX, src, dst);}
#endif
	void movsb()		{AppendInstr(I_MOVS_B, 0xA4, 0);}
	void movsw()		{AppendInstr(I_MOVS_W, 0xA5, E_OPERAND_SIZE_PREFIX);}
	void movsd()		{AppendInstr(I_MOVS_D, 0xA5, 0);}
#ifdef JITASM64
	void movsq()		{AppendInstr(I_MOVS_Q, 0xA5, E_REXW_PREFIX);}
#endif
	void rep_movsb()	{AppendInstr(I_MOVS_B, 0xA4, E_REPEAT_PREFIX);}
	void rep_movsw()	{AppendInstr(I_MOVS_W, 0xA5, E_REPEAT_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_movsd()	{AppendInstr(I_MOVS_D, 0xA5, E_REPEAT_PREFIX);}
#ifdef JITASM64
	void rep_movsq()	{AppendInstr(I_MOVS_Q, 0xA5, E_REPEAT_PREFIX | E_REXW_PREFIX);}
#endif
	void movsx(const Reg16& dst, const Reg8& src)	{AppendInstr(I_MOVSX, 0x0FBE, E_OPERAND_SIZE_PREFIX, dst, src);}
	void movsx(const Reg16& dst, const Mem8& src)	{AppendInstr(I_MOVSX, 0x0FBE, E_OPERAND_SIZE_PREFIX, dst, src);}
	void movsx(const Reg32& dst, const Reg8& src)	{AppendInstr(I_MOVSX, 0x0FBE, 0, dst, src);}
	void movsx(const Reg32& dst, const Mem8& src)	{AppendInstr(I_MOVSX, 0x0FBE, 0, dst, src);}
	void movsx(const Reg32& dst, const Reg16& src)	{AppendInstr(I_MOVSX, 0x0FBF, 0, dst, src);}
	void movsx(const Reg32& dst, const Mem16& src)	{AppendInstr(I_MOVSX, 0x0FBF, 0, dst, src);}
#ifdef JITASM64
	void movsx(const Reg64& dst, const Reg8& src)	{AppendInstr(I_MOVSX, 0x0FBE, E_REXW_PREFIX, dst, src);}
	void movsx(const Reg64& dst, const Mem8& src)	{AppendInstr(I_MOVSX, 0x0FBE, E_REXW_PREFIX, dst, src);}
	void movsx(const Reg64& dst, const Reg16& src)	{AppendInstr(I_MOVSX, 0x0FBF, E_REXW_PREFIX, dst, src);}
	void movsx(const Reg64& dst, const Mem16& src)	{AppendInstr(I_MOVSX, 0x0FBF, E_REXW_PREFIX, dst, src);}
	void movsxd(const Reg64& dst, const Reg32& src)	{AppendInstr(I_MOVSXD, 0x63, E_REXW_PREFIX, dst, src);}
	void movsxd(const Reg64& dst, const Mem32& src)	{AppendInstr(I_MOVSXD, 0x63, E_REXW_PREFIX, dst, src);}
#endif
	void movzx(const Reg16& dst, const Reg8& src)	{AppendInstr(I_MOVZX, 0x0FB6, E_OPERAND_SIZE_PREFIX, dst, src);}
	void movzx(const Reg16& dst, const Mem8& src)	{AppendInstr(I_MOVZX, 0x0FB6, E_OPERAND_SIZE_PREFIX, dst, src);}
	void movzx(const Reg32& dst, const Reg8& src)	{AppendInstr(I_MOVZX, 0x0FB6, 0, dst, src);}
	void movzx(const Reg32& dst, const Mem8& src)	{AppendInstr(I_MOVZX, 0x0FB6, 0, dst, src);}
	void movzx(const Reg32& dst, const Reg16& src)	{AppendInstr(I_MOVZX, 0x0FB7, 0, dst, src);}
	void movzx(const Reg32& dst, const Mem16& src)	{AppendInstr(I_MOVZX, 0x0FB7, 0, dst, src);}
#ifdef JITASM64
	void movzx(const Reg64& dst, const Reg8& src)	{AppendInstr(I_MOVZX, 0x0FB6, E_REXW_PREFIX, dst, src);}
	void movzx(const Reg64& dst, const Mem8& src)	{AppendInstr(I_MOVZX, 0x0FB6, E_REXW_PREFIX, dst, src);}
	void movzx(const Reg64& dst, const Reg16& src)	{AppendInstr(I_MOVZX, 0x0FB7, E_REXW_PREFIX, dst, src);}
	void movzx(const Reg64& dst, const Mem16& src)	{AppendInstr(I_MOVZX, 0x0FB7, E_REXW_PREFIX, dst, src);}
#endif
	void mul(const Reg8& dst)	{AppendInstr(I_MUL, 0xF6, 0, Imm8(4), dst);}
	void mul(const Mem8& dst)	{AppendInstr(I_MUL, 0xF6, 0, Imm8(4), dst);}
	void mul(const Reg16& dst)	{AppendInstr(I_MUL, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(4), dst);}
	void mul(const Mem16& dst)	{AppendInstr(I_MUL, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(4), dst);}
	void mul(const Reg32& dst)	{AppendInstr(I_MUL, 0xF7, 0, Imm8(4), dst);}
	void mul(const Mem32& dst)	{AppendInstr(I_MUL, 0xF7, 0, Imm8(4), dst);}
#ifdef JITASM64
	void mul(const Reg64& dst)	{AppendInstr(I_MUL, 0xF7, E_REXW_PREFIX, Imm8(4), dst);}
	void mul(const Mem64& dst)	{AppendInstr(I_MUL, 0xF7, E_REXW_PREFIX, Imm8(4), dst);}
#endif
	void neg(const Reg8& dst)	{AppendInstr(I_NEG, 0xF6, 0, Imm8(3), dst);}
	void neg(const Mem8& dst)	{AppendInstr(I_NEG, 0xF6, 0, Imm8(3), dst);}
	void neg(const Reg16& dst)	{AppendInstr(I_NEG, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(3), dst);}
	void neg(const Mem16& dst)	{AppendInstr(I_NEG, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(3), dst);}
	void neg(const Reg32& dst)	{AppendInstr(I_NEG, 0xF7, 0, Imm8(3), dst);}
	void neg(const Mem32& dst)	{AppendInstr(I_NEG, 0xF7, 0, Imm8(3), dst);}
#ifdef JITASM64
	void neg(const Reg64& dst)	{AppendInstr(I_NEG, 0xF7, E_REXW_PREFIX, Imm8(3), dst);}
	void neg(const Mem64& dst)	{AppendInstr(I_NEG, 0xF7, E_REXW_PREFIX, Imm8(3), dst);}
#endif
	void nop()	{AppendInstr(I_NOP, 0x90, 0);}
	void not(const Reg8& dst)	{AppendInstr(I_NOT, 0xF6, 0, Imm8(2), dst);}
	void not(const Mem8& dst)	{AppendInstr(I_NOT, 0xF6, 0, Imm8(2), dst);}
	void not(const Reg16& dst)	{AppendInstr(I_NOT, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(2), dst);}
	void not(const Mem16& dst)	{AppendInstr(I_NOT, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(2), dst);}
	void not(const Reg32& dst)	{AppendInstr(I_NOT, 0xF7, 0, Imm8(2), dst);}
	void not(const Mem32& dst)	{AppendInstr(I_NOT, 0xF7, 0, Imm8(2), dst);}
#ifdef JITASM64
	void not(const Reg64& dst)	{AppendInstr(I_NOT, 0xF7, E_REXW_PREFIX, Imm8(2), dst);}
	void not(const Mem64& dst)	{AppendInstr(I_NOT, 0xF7, E_REXW_PREFIX, Imm8(2), dst);}
#endif
	void or(const Reg8& dst, const Imm8& imm)	{AppendInstr(I_OR, 0x80, E_SPECIAL, Imm8(1), dst, imm);}
	void or(const Mem8& dst, const Imm8& imm)	{AppendInstr(I_OR, 0x80, 0, Imm8(1), dst, imm);}
	void or(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_OR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(1), dst, detail::ImmXor8(imm));}
	void or(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_OR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(1), dst, detail::ImmXor8(imm));}
	void or(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_OR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(1), dst, detail::ImmXor8(imm));}
	void or(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_OR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(1), dst, detail::ImmXor8(imm));}
	void or(const Reg8& dst, const Reg8& src)	{AppendInstr(I_OR, 0x08, 0, src, dst);}
	void or(const Mem8& dst, const Reg8& src)	{AppendInstr(I_OR, 0x08, 0, src, dst);}
	void or(const Reg8& dst, const Mem8& src)	{AppendInstr(I_OR, 0x0A, 0, dst, src);}
	void or(const Reg16& dst, const Reg16& src)	{AppendInstr(I_OR, 0x09, E_OPERAND_SIZE_PREFIX, src, dst);}
	void or(const Mem16& dst, const Reg16& src)	{AppendInstr(I_OR, 0x09, E_OPERAND_SIZE_PREFIX, src, dst);}
	void or(const Reg16& dst, const Mem16& src)	{AppendInstr(I_OR, 0x0B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void or(const Reg32& dst, const Reg32& src)	{AppendInstr(I_OR, 0x09, 0, src, dst);}
	void or(const Mem32& dst, const Reg32& src)	{AppendInstr(I_OR, 0x09, 0, src, dst);}
	void or(const Reg32& dst, const Mem32& src)	{AppendInstr(I_OR, 0x0B, 0, dst, src);}
#ifdef JITASM64
	void or(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_OR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(1), dst, detail::ImmXor8(imm));}
	void or(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_OR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(1), dst, detail::ImmXor8(imm));}
	void or(const Reg64& dst, const Reg64& src)	{AppendInstr(I_OR, 0x09, E_REXW_PREFIX, src, dst);}
	void or(const Mem64& dst, const Reg64& src)	{AppendInstr(I_OR, 0x09, E_REXW_PREFIX, src, dst);}
	void or(const Reg64& dst, const Mem64& src)	{AppendInstr(I_OR, 0x0B, E_REXW_PREFIX, dst, src);}
#endif
	void pop(const Reg16& dst)	{AppendInstr(I_POP, 0x58, E_OPERAND_SIZE_PREFIX, dst);}
	void pop(const Mem16& dst)	{AppendInstr(I_POP, 0x8F, E_OPERAND_SIZE_PREFIX, Imm8(0), dst);}
#ifndef JITASM64
	void pop(const Reg32& dst)	{AppendInstr(I_POP, 0x58, 0, dst);}
	void pop(const Mem32& dst)	{AppendInstr(I_POP, 0x8F, 0, Imm8(0), dst);}
#else
	void pop(const Reg64& dst)	{AppendInstr(I_POP, 0x58, 0, dst);}
	void pop(const Mem64& dst)	{AppendInstr(I_POP, 0x8F, 0, Imm8(0), dst);}
#endif
	void push(const Reg16& dst)	{AppendInstr(I_PUSH, 0x50, E_OPERAND_SIZE_PREFIX, dst);}
	void push(const Mem16& dst)	{AppendInstr(I_PUSH, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(6), dst);}
#ifndef JITASM64
	void push(const Reg32& dst)	{AppendInstr(I_PUSH, 0x50, 0, dst);}
	void push(const Mem32& dst)	{AppendInstr(I_PUSH, 0xFF, 0, Imm8(6), dst);}
#else
	void push(const Reg64& dst)	{AppendInstr(I_PUSH, 0x50, 0, dst);}
	void push(const Mem64& dst)	{AppendInstr(I_PUSH, 0xFF, 0, Imm8(6), dst);}
#endif
	void push(const Imm32& imm)	{AppendInstr(I_PUSH, detail::IsInt8(imm.GetImm()) ? 0x6A : 0x68, 0, detail::ImmXor8(imm));}
	void rdtsc()	{AppendInstr(I_RDTSC, 0x0F31, 0);}
	void ret()					{AppendInstr(I_RET, 0xC3, 0);}
	void ret(const Imm16& imm)	{AppendInstr(I_RET, 0xC2, 0, imm);}
 	void rcl(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD0, 0, Imm8(2), dst) : AppendInstr(I_RCL, 0xC0, 0, Imm8(2), dst, shift);}
	void rcl(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD0, 0, Imm8(2), dst) : AppendInstr(I_RCL, 0xC0, 0, Imm8(2), dst, shift);}
	void rcr(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD0, 0, Imm8(3), dst) : AppendInstr(I_RCR, 0xC0, 0, Imm8(3), dst, shift);}
	void rcr(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD0, 0, Imm8(3), dst) : AppendInstr(I_RCR, 0xC0, 0, Imm8(3), dst, shift);}
	void rol(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD0, 0, Imm8(0), dst) : AppendInstr(I_ROL, 0xC0, 0, Imm8(0), dst, shift);}
	void rol(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD0, 0, Imm8(0), dst) : AppendInstr(I_ROL, 0xC0, 0, Imm8(0), dst, shift);}
	void ror(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD0, 0, Imm8(1), dst) : AppendInstr(I_ROR, 0xC0, 0, Imm8(1), dst, shift);}
	void ror(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD0, 0, Imm8(1), dst) : AppendInstr(I_ROR, 0xC0, 0, Imm8(1), dst, shift);}
	void rcl(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(2), dst) : AppendInstr(I_RCL, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(2), dst, shift);}
	void rcl(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(2), dst) : AppendInstr(I_RCL, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(2), dst, shift);}
	void rcr(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(3), dst) : AppendInstr(I_RCR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(3), dst, shift);}
	void rcr(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(3), dst) : AppendInstr(I_RCR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(3), dst, shift);}
	void rol(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(0), dst) : AppendInstr(I_ROL, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(0), dst, shift);}
	void rol(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(0), dst) : AppendInstr(I_ROL, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(0), dst, shift);}
	void ror(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(1), dst) : AppendInstr(I_ROR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(1), dst, shift);}
	void ror(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(1), dst) : AppendInstr(I_ROR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(1), dst, shift);}
	void rcl(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD1, 0, Imm8(2), dst) : AppendInstr(I_RCL, 0xC1, 0, Imm8(2), dst, shift);}
	void rcl(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD1, 0, Imm8(2), dst) : AppendInstr(I_RCL, 0xC1, 0, Imm8(2), dst, shift);}
	void rcr(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD1, 0, Imm8(3), dst) : AppendInstr(I_RCR, 0xC1, 0, Imm8(3), dst, shift);}
	void rcr(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD1, 0, Imm8(3), dst) : AppendInstr(I_RCR, 0xC1, 0, Imm8(3), dst, shift);}
	void rol(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD1, 0, Imm8(0), dst) : AppendInstr(I_ROL, 0xC1, 0, Imm8(0), dst, shift);}
	void rol(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD1, 0, Imm8(0), dst) : AppendInstr(I_ROL, 0xC1, 0, Imm8(0), dst, shift);}
	void ror(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD1, 0, Imm8(1), dst) : AppendInstr(I_ROR, 0xC1, 0, Imm8(1), dst, shift);}
	void ror(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD1, 0, Imm8(1), dst) : AppendInstr(I_ROR, 0xC1, 0, Imm8(1), dst, shift);}
#ifdef JITASM64
	void rcl(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD1, E_REXW_PREFIX, Imm8(2), dst) : AppendInstr(I_RCL, 0xC1, E_REXW_PREFIX, Imm8(2), dst, shift);}
	void rcl(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCL, 0xD1, E_REXW_PREFIX, Imm8(2), dst) : AppendInstr(I_RCL, 0xC1, E_REXW_PREFIX, Imm8(2), dst, shift);}
	void rcr(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD1, E_REXW_PREFIX, Imm8(3), dst) : AppendInstr(I_RCR, 0xC1, E_REXW_PREFIX, Imm8(3), dst, shift);}
	void rcr(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_RCR, 0xD1, E_REXW_PREFIX, Imm8(3), dst) : AppendInstr(I_RCR, 0xC1, E_REXW_PREFIX, Imm8(3), dst, shift);}
	void rol(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD1, E_REXW_PREFIX, Imm8(0), dst) : AppendInstr(I_ROL, 0xC1, E_REXW_PREFIX, Imm8(0), dst, shift);}
	void rol(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROL, 0xD1, E_REXW_PREFIX, Imm8(0), dst) : AppendInstr(I_ROL, 0xC1, E_REXW_PREFIX, Imm8(0), dst, shift);}
	void ror(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD1, E_REXW_PREFIX, Imm8(1), dst) : AppendInstr(I_ROR, 0xC1, E_REXW_PREFIX, Imm8(1), dst, shift);}
	void ror(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_ROR, 0xD1, E_REXW_PREFIX, Imm8(1), dst) : AppendInstr(I_ROR, 0xC1, E_REXW_PREFIX, Imm8(1), dst, shift);}
#endif
	void sal(const Reg8& dst, const Imm8& shift)	{shl(dst, shift);}
	void sal(const Mem8& dst, const Imm8& shift)	{shl(dst, shift);}
	void sar(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD0, 0, Imm8(7), dst) : AppendInstr(I_SAR, 0xC0, 0, Imm8(7), dst, shift);}
	void sar(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD0, 0, Imm8(7), dst) : AppendInstr(I_SAR, 0xC0, 0, Imm8(7), dst, shift);}
	void shl(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD0, 0, Imm8(4), dst) : AppendInstr(I_SHL, 0xC0, 0, Imm8(4), dst, shift);}
	void shl(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD0, 0, Imm8(4), dst) : AppendInstr(I_SHL, 0xC0, 0, Imm8(4), dst, shift);}
	void shr(const Reg8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD0, 0, Imm8(5), dst) : AppendInstr(I_SHR, 0xC0, 0, Imm8(5), dst, shift);}
	void shr(const Mem8& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD0, 0, Imm8(5), dst) : AppendInstr(I_SHR, 0xC0, 0, Imm8(5), dst, shift);}
	void sal(const Reg16& dst, const Imm8& shift)	{shl(dst, shift);}
	void sal(const Mem16& dst, const Imm8& shift)	{shl(dst, shift);}
	void sar(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(7), dst) : AppendInstr(I_SAR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(7), dst, shift);}
	void sar(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(7), dst) : AppendInstr(I_SAR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(7), dst, shift);}
	void shl(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(4), dst) : AppendInstr(I_SHL, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(4), dst, shift);}
	void shl(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(4), dst) : AppendInstr(I_SHL, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(4), dst, shift);}
	void shr(const Reg16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(5), dst) : AppendInstr(I_SHR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(5), dst, shift);}
	void shr(const Mem16& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD1, E_OPERAND_SIZE_PREFIX, Imm8(5), dst) : AppendInstr(I_SHR, 0xC1, E_OPERAND_SIZE_PREFIX, Imm8(5), dst, shift);}
	void sal(const Reg32& dst, const Imm8& shift)	{shl(dst, shift);}
	void sal(const Mem32& dst, const Imm8& shift)	{shl(dst, shift);}
	void sar(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD1, 0, Imm8(7), dst) : AppendInstr(I_SAR, 0xC1, 0, Imm8(7), dst, shift);}
	void sar(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD1, 0, Imm8(7), dst) : AppendInstr(I_SAR, 0xC1, 0, Imm8(7), dst, shift);}
	void shl(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD1, 0, Imm8(4), dst) : AppendInstr(I_SHL, 0xC1, 0, Imm8(4), dst, shift);}
	void shl(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD1, 0, Imm8(4), dst) : AppendInstr(I_SHL, 0xC1, 0, Imm8(4), dst, shift);}
	void shr(const Reg32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD1, 0, Imm8(5), dst) : AppendInstr(I_SHR, 0xC1, 0, Imm8(5), dst, shift);}
	void shr(const Mem32& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD1, 0, Imm8(5), dst) : AppendInstr(I_SHR, 0xC1, 0, Imm8(5), dst, shift);}
#ifdef JITASM64
	void sal(const Reg64& dst, const Imm8& shift)	{shl(dst, shift);}
	void sal(const Mem64& dst, const Imm8& shift)	{shl(dst, shift);}
	void sar(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD1, E_REXW_PREFIX, Imm8(7), dst) : AppendInstr(I_SAR, 0xC1, E_REXW_PREFIX, Imm8(7), dst, shift);}
	void sar(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SAR, 0xD1, E_REXW_PREFIX, Imm8(7), dst) : AppendInstr(I_SAR, 0xC1, E_REXW_PREFIX, Imm8(7), dst, shift);}
	void shl(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD1, E_REXW_PREFIX, Imm8(4), dst) : AppendInstr(I_SHL, 0xC1, E_REXW_PREFIX, Imm8(4), dst, shift);}
	void shl(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHL, 0xD1, E_REXW_PREFIX, Imm8(4), dst) : AppendInstr(I_SHL, 0xC1, E_REXW_PREFIX, Imm8(4), dst, shift);}
	void shr(const Reg64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD1, E_REXW_PREFIX, Imm8(5), dst) : AppendInstr(I_SHR, 0xC1, E_REXW_PREFIX, Imm8(5), dst, shift);}
	void shr(const Mem64& dst, const Imm8& shift)	{shift.GetImm() == 1 ? AppendInstr(I_SHR, 0xD1, E_REXW_PREFIX, Imm8(5), dst) : AppendInstr(I_SHR, 0xC1, E_REXW_PREFIX, Imm8(5), dst, shift);}
#endif
	void sbb(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_SBB, 0x80, E_SPECIAL, Imm8(3), dst, imm);}
	void sbb(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_SBB, 0x80, 0, Imm8(3), dst, imm);}
	void sbb(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_SBB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(3), dst, detail::ImmXor8(imm));}
	void sbb(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_SBB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(3), dst, detail::ImmXor8(imm));}
	void sbb(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_SBB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(3), dst, detail::ImmXor8(imm));}
	void sbb(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_SBB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(3), dst, detail::ImmXor8(imm));}
	void sbb(const Reg8& dst, const Reg8& src)		{AppendInstr(I_SBB, 0x18, 0, src, dst);}
	void sbb(const Mem8& dst, const Reg8& src)		{AppendInstr(I_SBB, 0x18, 0, src, dst);}
	void sbb(const Reg8& dst, const Mem8& src)		{AppendInstr(I_SBB, 0x1A, 0, dst, src);}
	void sbb(const Reg16& dst, const Reg16& src)	{AppendInstr(I_SBB, 0x19, E_OPERAND_SIZE_PREFIX, src, dst);}
	void sbb(const Mem16& dst, const Reg16& src)	{AppendInstr(I_SBB, 0x19, E_OPERAND_SIZE_PREFIX, src, dst);}
	void sbb(const Reg16& dst, const Mem16& src)	{AppendInstr(I_SBB, 0x1B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void sbb(const Reg32& dst, const Reg32& src)	{AppendInstr(I_SBB, 0x19, 0, src, dst);}
	void sbb(const Mem32& dst, const Reg32& src)	{AppendInstr(I_SBB, 0x19, 0, src, dst);}
	void sbb(const Reg32& dst, const Mem32& src)	{AppendInstr(I_SBB, 0x1B, 0, dst, src);}
#ifdef JITASM64
	void sbb(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_SBB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(3), dst, detail::ImmXor8(imm));}
	void sbb(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_SBB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(3), dst, detail::ImmXor8(imm));}
	void sbb(const Reg64& dst, const Reg64& src)	{AppendInstr(I_SBB, 0x19, E_REXW_PREFIX, src, dst);}
	void sbb(const Mem64& dst, const Reg64& src)	{AppendInstr(I_SBB, 0x19, E_REXW_PREFIX, src, dst);}
	void sbb(const Reg64& dst, const Mem64& src)	{AppendInstr(I_SBB, 0x1B, E_REXW_PREFIX, dst, src);}
#endif
	void seta(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F97, 0, Imm8(0), dst);}
	void seta(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F97, 0, Imm8(0), dst);}
	void setae(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F93, 0, Imm8(0), dst);}
	void setae(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F93, 0, Imm8(0), dst);}
	void setb(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F92, 0, Imm8(0), dst);}
	void setb(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F92, 0, Imm8(0), dst);}
	void setbe(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F96, 0, Imm8(0), dst);}
	void setbe(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F96, 0, Imm8(0), dst);}
	void setc(const Reg8& dst)		{setb(dst);}
	void setc(const Mem8& dst)		{setb(dst);}
	void sete(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F94, 0, Imm8(0), dst);}
	void sete(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F94, 0, Imm8(0), dst);}
	void setg(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F9F, 0, Imm8(0), dst);}
	void setg(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F9F, 0, Imm8(0), dst);}
	void setge(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F9D, 0, Imm8(0), dst);}
	void setge(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F9D, 0, Imm8(0), dst);}
	void setl(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F9C, 0, Imm8(0), dst);}
	void setl(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F9C, 0, Imm8(0), dst);}
	void setle(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F9E, 0, Imm8(0), dst);}
	void setle(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F9E, 0, Imm8(0), dst);}
	void setna(const Reg8& dst)		{setbe(dst);}
	void setna(const Mem8& dst)		{setbe(dst);}
	void setnae(const Reg8& dst)	{setb(dst);}
	void setnae(const Mem8& dst)	{setb(dst);}
	void setnb(const Reg8& dst)		{setae(dst);}
	void setnb(const Mem8& dst)		{setae(dst);}
	void setnbe(const Reg8& dst)	{seta(dst);}
	void setnbe(const Mem8& dst)	{seta(dst);}
	void setnc(const Reg8& dst)		{setae(dst);}
	void setnc(const Mem8& dst)		{setae(dst);}
	void setne(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F95, 0, Imm8(0), dst);}
	void setne(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F95, 0, Imm8(0), dst);}
	void setng(const Reg8& dst)		{setle(dst);}
	void setng(const Mem8& dst)		{setle(dst);}
	void setnge(const Reg8& dst)	{setl(dst);}
	void setnge(const Mem8& dst)	{setl(dst);}
	void setnl(const Reg8& dst)		{setge(dst);}
	void setnl(const Mem8& dst)		{setge(dst);}
	void setnle(const Reg8& dst)	{setg(dst);}
	void setnle(const Mem8& dst)	{setg(dst);}
	void setno(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F91, 0, Imm8(0), dst);}
	void setno(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F91, 0, Imm8(0), dst);}
	void setnp(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F9B, 0, Imm8(0), dst);}
	void setnp(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F9B, 0, Imm8(0), dst);}
	void setns(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F99, 0, Imm8(0), dst);}
	void setns(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F99, 0, Imm8(0), dst);}
	void setnz(const Reg8& dst)		{setne(dst);}
	void setnz(const Mem8& dst)		{setne(dst);}
	void seto(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F90, 0, Imm8(0), dst);}
	void seto(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F90, 0, Imm8(0), dst);}
	void setp(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F9A, 0, Imm8(0), dst);}
	void setp(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F9A, 0, Imm8(0), dst);}
	void setpe(const Reg8& dst)		{setp(dst);}
	void setpe(const Mem8& dst)		{setp(dst);}
	void setpo(const Reg8& dst)		{setnp(dst);}
	void setpo(const Mem8& dst)		{setnp(dst);}
	void sets(const Reg8& dst)		{AppendInstr(I_SETCC, 0x0F98, 0, Imm8(0), dst);}
	void sets(const Mem8& dst)		{AppendInstr(I_SETCC, 0x0F98, 0, Imm8(0), dst);}
	void setz(const Reg8& dst)		{sete(dst);}
	void setz(const Mem8& dst)		{sete(dst);}
	void shld(const Reg16& dst, const Reg16& src, const Imm8& place)	{AppendInstr(I_SHLD, 0x0FA4, E_OPERAND_SIZE_PREFIX, src, dst, place);}
	void shld(const Mem16& dst, const Reg16& src, const Imm8& place)	{AppendInstr(I_SHLD, 0x0FA4, E_OPERAND_SIZE_PREFIX, src, dst, place);}
	void shld(const Reg16& dst, const Reg16& src, const Reg8_cl&)		{AppendInstr(I_SHLD, 0x0FA5, E_OPERAND_SIZE_PREFIX, src, dst);}
	void shld(const Mem16& dst, const Reg16& src, const Reg8_cl&)		{AppendInstr(I_SHLD, 0x0FA5, E_OPERAND_SIZE_PREFIX, src, dst);}
	void shld(const Reg32& dst, const Reg32& src, const Imm8& place)	{AppendInstr(I_SHLD, 0x0FA4, 0, src, dst, place);}
	void shld(const Mem32& dst, const Reg32& src, const Imm8& place)	{AppendInstr(I_SHLD, 0x0FA4, 0, src, dst, place);}
	void shld(const Reg32& dst, const Reg32& src, const Reg8_cl&)		{AppendInstr(I_SHLD, 0x0FA5, 0, src, dst);}
	void shld(const Mem32& dst, const Reg32& src, const Reg8_cl&)		{AppendInstr(I_SHLD, 0x0FA5, 0, src, dst);}
#ifdef JITASM64
	void shld(const Reg64& dst, const Reg64& src, const Imm8& place)	{AppendInstr(I_SHLD, 0x0FA4, E_REXW_PREFIX, src, dst, place);}
	void shld(const Mem64& dst, const Reg64& src, const Imm8& place)	{AppendInstr(I_SHLD, 0x0FA4, E_REXW_PREFIX, src, dst, place);}
	void shld(const Reg64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHLD, 0x0FA5, E_REXW_PREFIX, src, dst);}
	void shld(const Mem64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHLD, 0x0FA5, E_REXW_PREFIX, src, dst);}
#endif
	void shrd(const Reg16& dst, const Reg16& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_OPERAND_SIZE_PREFIX, src, dst, place);}
	void shrd(const Mem16& dst, const Reg16& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_OPERAND_SIZE_PREFIX, src, dst, place);}
	void shrd(const Reg16& dst, const Reg16& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_OPERAND_SIZE_PREFIX, src, dst);}
	void shrd(const Mem16& dst, const Reg16& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_OPERAND_SIZE_PREFIX, src, dst);}
	void shrd(const Reg32& dst, const Reg32& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, 0, src, dst, place);}
	void shrd(const Mem32& dst, const Reg32& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, 0, src, dst, place);}
	void shrd(const Reg32& dst, const Reg32& src, const Reg8_cl&)		{AppendInstr(I_SHRD, 0x0FAD, 0, src, dst);}
	void shrd(const Mem32& dst, const Reg32& src, const Reg8_cl&)		{AppendInstr(I_SHRD, 0x0FAD, 0, src, dst);}
#ifdef JITASM64
	void shrd(const Reg64& dst, const Reg64& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_REXW_PREFIX, src, dst, place);}
	void shrd(const Mem64& dst, const Reg64& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_REXW_PREFIX, src, dst, place);}
	void shrd(const Reg64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_REXW_PREFIX, src, dst);}
	void shrd(const Mem64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_REXW_PREFIX, src, dst);}
#endif
	void stc()	{AppendInstr(I_STC, 0xF9, 0);}
	void std()	{AppendInstr(I_STD, 0xFD, 0);}
	void sti()	{AppendInstr(I_STI, 0xFB, 0);}
	void stosb()		{AppendInstr(I_STOS_B, 0xAA, 0);}
	void stosw()		{AppendInstr(I_STOS_W, 0xAB, E_OPERAND_SIZE_PREFIX);}
	void stosd()		{AppendInstr(I_STOS_D, 0xAB, 0);}
#ifdef JITASM64
	void stosq()		{AppendInstr(I_STOS_Q, 0xAB, E_REXW_PREFIX);}
#endif
	void rep_stosb()	{AppendInstr(I_STOS_B, 0xAA, E_REPEAT_PREFIX);}
	void rep_stosw()	{AppendInstr(I_STOS_W, 0xAB, E_REPEAT_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_stosd()	{AppendInstr(I_STOS_D, 0xAB, E_REPEAT_PREFIX);}
#ifdef JITASM64
	void rep_stosq()	{AppendInstr(I_STOS_Q, 0xAB, E_REPEAT_PREFIX | E_REXW_PREFIX);}
#endif
	void sub(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_SUB, 0x80, E_SPECIAL, Imm8(5), dst, imm);}
	void sub(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_SUB, 0x80, 0, Imm8(5), dst, imm);}
	void sub(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_SUB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(5), dst, detail::ImmXor8(imm));}
	void sub(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_SUB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(5), dst, detail::ImmXor8(imm));}
	void sub(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_SUB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(5), dst, detail::ImmXor8(imm));}
	void sub(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_SUB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(5), dst, detail::ImmXor8(imm));}
	void sub(const Reg8& dst, const Reg8& src)		{AppendInstr(I_SUB, 0x28, 0, src, dst);}
	void sub(const Mem8& dst, const Reg8& src)		{AppendInstr(I_SUB, 0x28, 0, src, dst);}
	void sub(const Reg8& dst, const Mem8& src)		{AppendInstr(I_SUB, 0x2A, 0, dst, src);}
	void sub(const Reg16& dst, const Reg16& src)	{AppendInstr(I_SUB, 0x29, E_OPERAND_SIZE_PREFIX, src, dst);}
	void sub(const Mem16& dst, const Reg16& src)	{AppendInstr(I_SUB, 0x29, E_OPERAND_SIZE_PREFIX, src, dst);}
	void sub(const Reg16& dst, const Mem16& src)	{AppendInstr(I_SUB, 0x2B, E_OPERAND_SIZE_PREFIX, dst, src);}
	void sub(const Reg32& dst, const Reg32& src)	{AppendInstr(I_SUB, 0x29, 0, src, dst);}
	void sub(const Mem32& dst, const Reg32& src)	{AppendInstr(I_SUB, 0x29, 0, src, dst);}
	void sub(const Reg32& dst, const Mem32& src)	{AppendInstr(I_SUB, 0x2B, 0, dst, src);}
#ifdef JITASM64
	void sub(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_SUB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(5), dst, detail::ImmXor8(imm));}
	void sub(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_SUB, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(5), dst, detail::ImmXor8(imm));}
	void sub(const Reg64& dst, const Reg64& src)	{AppendInstr(I_SUB, 0x29, E_REXW_PREFIX, src, dst);}
	void sub(const Mem64& dst, const Reg64& src)	{AppendInstr(I_SUB, 0x29, E_REXW_PREFIX, src, dst);}
	void sub(const Reg64& dst, const Mem64& src)	{AppendInstr(I_SUB, 0x2B, E_REXW_PREFIX, dst, src);}
#endif
	void test(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_TEST, 0xF6, E_SPECIAL, Imm8(0), dst, imm);}
	void test(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_TEST, 0xF6, 0, Imm8(0), dst, imm);}
	void test(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_TEST, 0xF7, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(0), dst, imm);}
	void test(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_TEST, 0xF7, E_OPERAND_SIZE_PREFIX, Imm8(0), dst, imm);}
	void test(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_TEST, 0xF7, E_SPECIAL, Imm8(0), dst, imm);}
	void test(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_TEST, 0xF7, 0, Imm8(0), dst, imm);}
	void test(const Reg8& dst, const Reg8& src)		{AppendInstr(I_TEST, 0x84, 0, dst, src);}
	void test(const Mem8& dst, const Reg8& src)		{AppendInstr(I_TEST, 0x84, 0, src, dst);}
	void test(const Reg16& dst, const Reg16& src)	{AppendInstr(I_TEST, 0x85, E_OPERAND_SIZE_PREFIX, dst, src);}
	void test(const Mem16& dst, const Reg16& src)	{AppendInstr(I_TEST, 0x85, E_OPERAND_SIZE_PREFIX, src, dst);}
	void test(const Reg32& dst, const Reg32& src)	{AppendInstr(I_TEST, 0x85, 0, dst, src);}
	void test(const Mem32& dst, const Reg32& src)	{AppendInstr(I_TEST, 0x85, 0, src, dst);}
#ifdef JITASM64
	void test(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_TEST, 0xF7, E_REXW_PREFIX | E_SPECIAL, Imm8(0), dst, imm);}
	void test(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_TEST, 0xF7, E_REXW_PREFIX, Imm8(0), dst, imm);}
	void test(const Reg64& dst, const Reg64& src)	{AppendInstr(I_TEST, 0x85, E_REXW_PREFIX, dst, src);}
	void test(const Mem64& dst, const Reg64& src)	{AppendInstr(I_TEST, 0x85, E_REXW_PREFIX, src, dst);}
#endif
	void ud2()		{AppendInstr(I_UD2, 0x0F0B, 0);}
	void wait()		{fwait();}
	void fwait()	{AppendInstr(I_FWAIT, 0x9B, 0);}
	void xadd(const Reg8& dst, const Reg8& src)		{AppendInstr(I_XADD, 0x0FC0, 0, src, dst);}
	void xadd(const Mem8& dst, const Reg8& src)		{AppendInstr(I_XADD, 0x0FC0, 0, src, dst);}
	void xadd(const Reg16& dst, const Reg16& src)	{AppendInstr(I_XADD, 0x0FC1, E_OPERAND_SIZE_PREFIX, src, dst);}
	void xadd(const Mem16& dst, const Reg16& src)	{AppendInstr(I_XADD, 0x0FC1, E_OPERAND_SIZE_PREFIX, src, dst);}
	void xadd(const Reg32& dst, const Reg32& src)	{AppendInstr(I_XADD, 0x0FC1, 0, src, dst);}
	void xadd(const Mem32& dst, const Reg32& src)	{AppendInstr(I_XADD, 0x0FC1, 0, src, dst);}
#ifdef JITASM64
	void xadd(const Reg64& dst, const Reg64& src)	{AppendInstr(I_XADD, 0x0FC1, E_REXW_PREFIX, src, dst);}
	void xadd(const Mem64& dst, const Reg64& src)	{AppendInstr(I_XADD, 0x0FC1, E_REXW_PREFIX, src, dst);}
#endif
	void xchg(const Reg8& dst, const Reg8& src)		{AppendInstr(I_XCHG, 0x86, 0, dst, src);}
	void xchg(const Mem8& dst, const Reg8& src)		{AppendInstr(I_XCHG, 0x86, 0, src, dst);}
	void xchg(const Reg8& dst, const Mem8& src)		{AppendInstr(I_XCHG, 0x86, 0, dst, src);}
	void xchg(const Reg16& dst, const Reg16& src)	{AppendInstr(I_XCHG, 0x87, E_OPERAND_SIZE_PREFIX | E_SPECIAL, dst, src);}
	void xchg(const Mem16& dst, const Reg16& src)	{AppendInstr(I_XCHG, 0x87, E_OPERAND_SIZE_PREFIX, src, dst);}
	void xchg(const Reg16& dst, const Mem16& src)	{AppendInstr(I_XCHG, 0x87, E_OPERAND_SIZE_PREFIX, dst, src);}
	void xchg(const Reg32& dst, const Reg32& src)	{AppendInstr(I_XCHG, 0x87, E_SPECIAL, dst, src);}
	void xchg(const Mem32& dst, const Reg32& src)	{AppendInstr(I_XCHG, 0x87, 0, src, dst);}
	void xchg(const Reg32& dst, const Mem32& src)	{AppendInstr(I_XCHG, 0x87, 0, dst, src);}
#ifdef JITASM64
	void xchg(const Reg64& dst, const Reg64& src)	{AppendInstr(I_XCHG, 0x87, E_REXW_PREFIX | E_SPECIAL, dst, src);}
	void xchg(const Mem64& dst, const Reg64& src)	{AppendInstr(I_XCHG, 0x87, E_REXW_PREFIX, src, dst);}
	void xchg(const Reg64& dst, const Mem64& src)	{AppendInstr(I_XCHG, 0x87, E_REXW_PREFIX, dst, src);}
#endif
	void xor(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_XOR, 0x80, E_SPECIAL, Imm8(6), dst, imm);}
	void xor(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_XOR, 0x80, 0, Imm8(6), dst, imm);}
	void xor(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_XOR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(6), dst, detail::ImmXor8(imm));}
	void xor(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_XOR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(6), dst, detail::ImmXor8(imm));}
	void xor(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_XOR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(6), dst, detail::ImmXor8(imm));}
	void xor(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_XOR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(6), dst, detail::ImmXor8(imm));}
	void xor(const Reg8& dst, const Reg8& src)		{AppendInstr(I_XOR, 0x30, 0, src, dst);}
	void xor(const Mem8& dst, const Reg8& src)		{AppendInstr(I_XOR, 0x30, 0, src, dst);}
	void xor(const Reg8& dst, const Mem8& src)		{AppendInstr(I_XOR, 0x32, 0, dst, src);}
	void xor(const Reg16& dst, const Reg16& src)	{AppendInstr(I_XOR, 0x31, E_OPERAND_SIZE_PREFIX, src, dst);}
	void xor(const Mem16& dst, const Reg16& src)	{AppendInstr(I_XOR, 0x31, E_OPERAND_SIZE_PREFIX, src, dst);}
	void xor(const Reg16& dst, const Mem16& src)	{AppendInstr(I_XOR, 0x33, E_OPERAND_SIZE_PREFIX, dst, src);}
	void xor(const Reg32& dst, const Reg32& src)	{AppendInstr(I_XOR, 0x31, 0, src, dst);}
	void xor(const Mem32& dst, const Reg32& src)	{AppendInstr(I_XOR, 0x31, 0, src, dst);}
	void xor(const Reg32& dst, const Mem32& src)	{AppendInstr(I_XOR, 0x33, 0, dst, src);}
#ifdef JITASM64
	void xor(const Reg64& dst, const Imm32& imm)	{AppendInstr(I_XOR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(6), dst, detail::ImmXor8(imm));}
	void xor(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_XOR, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(6), dst, detail::ImmXor8(imm));}
	void xor(const Reg64& dst, const Reg64& src)	{AppendInstr(I_XOR, 0x31, E_REXW_PREFIX, src, dst);}
	void xor(const Mem64& dst, const Reg64& src)	{AppendInstr(I_XOR, 0x31, E_REXW_PREFIX, src, dst);}
	void xor(const Reg64& dst, const Mem64& src)	{AppendInstr(I_XOR, 0x33, E_REXW_PREFIX, dst, src);}
#endif

	// x87 Floating-Point Instructions
	void fld(const Mem32& src)	{AppendInstr(I_FLD, 0xD9, 0, Imm8(0), src);}
	void fld(const Mem64& src)	{AppendInstr(I_FLD, 0xDD, 0, Imm8(0), src);}
	void fld(const Mem80& src)	{AppendInstr(I_FLD, 0xDB, 0, Imm8(5), src);}
	void fld(const FpuReg& src)	{AppendInstr(I_FLD, 0xD9C0, 0, src);}
	void fst(const Mem32& dst)		{AppendInstr(I_FST,	0xD9, 0, Imm8(2), dst);}
	void fst(const Mem64& dst)		{AppendInstr(I_FST,	0xDD, 0, Imm8(2), dst);}
	void fst(const FpuReg& dst)		{AppendInstr(I_FST,	0xDDD0, 0, dst);}
	void fstp(const Mem32& dst)		{AppendInstr(I_FSTP, 0xD9, 0, Imm8(3), dst);}
	void fstp(const Mem64& dst)		{AppendInstr(I_FSTP, 0xDD, 0, Imm8(3), dst);}
	void fstp(const Mem80& dst)		{AppendInstr(I_FSTP, 0xDB, 0, Imm8(7), dst);}
	void fstp(const FpuReg& dst)	{AppendInstr(I_FSTP, 0xDDD8, 0, dst);}

	// MMX
	void emms() {AppendInstr(I_EMMS, 0x0F77, 0);}
	void movd(const MmxReg& dst, const Reg32& src)	{AppendInstr(I_MOVD, 0x0F6E, 0, dst, src);}
	void movd(const MmxReg& dst, const Mem32& src)	{AppendInstr(I_MOVD, 0x0F6E, 0, dst, src);}
	void movd(const Reg32& dst, const MmxReg& src)	{AppendInstr(I_MOVD, 0x0F7E, 0, src, dst);}
	void movd(const Mem32& dst, const MmxReg& src)	{AppendInstr(I_MOVD, 0x0F7E, 0, src, dst);}
#ifdef JITASM64
	void movd(const MmxReg& dst, const Reg64& src)	{AppendInstr(I_MOVD, 0x0F6E, E_REXW_PREFIX, dst, src);}
	void movd(const Reg64& dst, const MmxReg& src)	{AppendInstr(I_MOVD, 0x0F7E, E_REXW_PREFIX, src, dst);}
#endif
	void movq(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_MOVQ, 0x0F6F, 0, dst, src);}
	void movq(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_MOVQ, 0x0F7F, 0, src, dst);}
	void movq(const Mem64& dst, const MmxReg& src)	{AppendInstr(I_MOVQ, 0x0F7F, 0, src, dst);}
#ifdef JITASM64
	void movq(const MmxReg& dst, const Reg64& src)	{movd(dst, src);}
	void movq(const Reg64& dst, const MmxReg& src)	{movd(dst, src);}
#endif
	void packsswb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PACKSSWB, 0x0F63, 0, dst, src);}
	void packsswb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PACKSSWB, 0x0F63, 0, dst, src);}
	void packssdw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PACKSSDW, 0x0F6B, 0, dst, src);}
	void packssdw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PACKSSDW, 0x0F6B, 0, dst, src);}
	void packuswb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PACKUSWB, 0x0F67, 0, dst, src);}
	void packuswb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PACKUSWB, 0x0F67, 0, dst, src);}
	void paddb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDB,	0x0FFC, 0, dst, src);}
	void paddb(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PADDB,	0x0FFC, 0, dst, src);}
	void paddw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDW,	0x0FFD, 0, dst, src);}
	void paddw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PADDW,	0x0FFD, 0, dst, src);}
	void paddd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDD,	0x0FFE, 0, dst, src);}
	void paddd(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PADDD,	0x0FFE, 0, dst, src);}
	void paddsb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDSB,	0x0FEC, 0, dst, src);}
	void paddsb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PADDSB,	0x0FEC, 0, dst, src);}
	void paddsw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDSW,	0x0FED, 0, dst, src);}
	void paddsw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PADDSW,	0x0FED, 0, dst, src);}
	void paddusb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDUSB,	0x0FDC, 0, dst, src);}
	void paddusb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PADDUSB,	0x0FDC, 0, dst, src);}
	void paddusw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDUSW,	0x0FDD, 0, dst, src);}
	void paddusw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PADDUSW,	0x0FDD, 0, dst, src);}
	void pand(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PAND,		0x0FDB, 0, dst, src);}
	void pand(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PAND,		0x0FDB, 0, dst, src);}
	void pandn(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PANDN,	0x0FDF, 0, dst, src);}
	void pandn(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PANDN,	0x0FDF, 0, dst, src);}
	void pcmpeqb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PCMPEQB,	0x0F74, 0, dst, src);}
	void pcmpeqb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PCMPEQB,	0x0F74, 0, dst, src);}
	void pcmpeqw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PCMPEQW,	0x0F75, 0, dst, src);}
	void pcmpeqw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PCMPEQW,	0x0F75, 0, dst, src);}
	void pcmpeqd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PCMPEQD,	0x0F76, 0, dst, src);}
	void pcmpeqd(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PCMPEQD,	0x0F76, 0, dst, src);}
	void pcmpgtb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PCMPGTB,	0x0F64, 0, dst, src);}
	void pcmpgtb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PCMPGTB,	0x0F64, 0, dst, src);}
	void pcmpgtw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PCMPGTW,	0x0F65, 0, dst, src);}
	void pcmpgtw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PCMPGTW,	0x0F65, 0, dst, src);}
	void pcmpgtd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PCMPGTD,	0x0F66, 0, dst, src);}
	void pcmpgtd(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PCMPGTD,	0x0F66, 0, dst, src);}
	void pmaddwd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMADDWD,	0x0FF5, 0, dst, src);}
	void pmaddwd(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMADDWD,	0x0FF5, 0, dst, src);}
	void pmulhw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMULHW,	0x0FE5, 0, dst, src);}
	void pmulhw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMULHW,	0x0FE5, 0, dst, src);}
	void pmullw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMULLW,	0x0FD5, 0, dst, src);}
	void pmullw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMULLW,	0x0FD5, 0, dst, src);}
	void por(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_POR,		0x0FEB, 0, dst, src);}
	void por(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_POR,		0x0FEB, 0, dst, src);}
	void psllw(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSLLW,	0x0FF1, 0, dst, count);}
	void psllw(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSLLW,	0x0FF1, 0, dst, count);}
	void psllw(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSLLW,	0x0F71, 0, Imm8(6), dst, count);}
	void pslld(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSLLD,	0x0FF2, 0, dst, count);}
	void pslld(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSLLD,	0x0FF2, 0, dst, count);}
	void pslld(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSLLD,	0x0F72, 0, Imm8(6), dst, count);}
	void psllq(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSLLQ,	0x0FF3, 0, dst, count);}
	void psllq(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSLLQ,	0x0FF3, 0, dst, count);}
	void psllq(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSLLQ,	0x0F73, 0, Imm8(6), dst, count);}
	void psraw(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSRAW,	0x0FE1, 0, dst, count);}
	void psraw(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSRAW,	0x0FE1, 0, dst, count);}
	void psraw(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSRAW,	0x0F71, 0, Imm8(4), dst, count);}
	void psrad(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSRAD,	0x0FE2, 0, dst, count);}
	void psrad(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSRAD,	0x0FE2, 0, dst, count);}
	void psrad(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSRAD,	0x0F72, 0, Imm8(4), dst, count);}
	void psrlw(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSRLW,	0x0FD1, 0, dst, count);}
	void psrlw(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSRLW,	0x0FD1, 0, dst, count);}
	void psrlw(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSRLW,	0x0F71, 0, Imm8(2), dst, count);}
	void psrld(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSRLD,	0x0FD2, 0, dst, count);}
	void psrld(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSRLD,	0x0FD2, 0, dst, count);}
	void psrld(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSRLD,	0x0F72, 0, Imm8(2), dst, count);}
	void psrlq(const MmxReg& dst, const MmxReg& count)	{AppendInstr(I_PSRLQ,	0x0FD3, 0, dst, count);}
	void psrlq(const MmxReg& dst, const Mem64& count)	{AppendInstr(I_PSRLQ,	0x0FD3, 0, dst, count);}
	void psrlq(const MmxReg& dst, const Imm8& count)	{AppendInstr(I_PSRLQ,	0x0F73, 0, Imm8(2), dst, count);}
	void psubb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBB,	0x0FF8, 0, dst, src);}
	void psubb(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSUBB,	0x0FF8, 0, dst, src);}
	void psubw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBW,	0x0FF9, 0, dst, src);}
	void psubw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSUBW,	0x0FF9, 0, dst, src);}
	void psubd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBD,	0x0FFA, 0, dst, src);}
	void psubd(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSUBD,	0x0FFA, 0, dst, src);}
	void psubsb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBSB,	0x0FE8, 0, dst, src);}
	void psubsb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PSUBSB,	0x0FE8, 0, dst, src);}
	void psubsw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBSW,	0x0FE9, 0, dst, src);}
	void psubsw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PSUBSW,	0x0FE9, 0, dst, src);}
	void psubusb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBUSB,	0x0FD8, 0, dst, src);}
	void psubusb(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PSUBUSB,	0x0FD8, 0, dst, src);}
	void psubusw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBUSW,	0x0FD9, 0, dst, src);}
	void psubusw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PSUBUSW,	0x0FD9, 0, dst, src);}
	void punpckhbw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PUNPCKHBW, 0x0F68, 0, dst, src);}
	void punpckhbw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PUNPCKHBW, 0x0F68, 0, dst, src);}
	void punpckhwd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PUNPCKHWD, 0x0F69, 0, dst, src);}
	void punpckhwd(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PUNPCKHWD, 0x0F69, 0, dst, src);}
	void punpckhdq(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PUNPCKHDQ, 0x0F6A, 0, dst, src);}
	void punpckhdq(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PUNPCKHDQ, 0x0F6A, 0, dst, src);}
	void punpcklbw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PUNPCKLBW, 0x0F60, 0, dst, src);}
	void punpcklbw(const MmxReg& dst, const Mem32& src)		{AppendInstr(I_PUNPCKLBW, 0x0F60, 0, dst, src);}
	void punpcklwd(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PUNPCKLWD, 0x0F61, 0, dst, src);}
	void punpcklwd(const MmxReg& dst, const Mem32& src)		{AppendInstr(I_PUNPCKLWD, 0x0F61, 0, dst, src);}
	void punpckldq(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PUNPCKLDQ, 0x0F62, 0, dst, src);}
	void punpckldq(const MmxReg& dst, const Mem32& src)		{AppendInstr(I_PUNPCKLDQ, 0x0F62, 0, dst, src);}
	void pxor(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PXOR, 0x0FEF, 0, dst, src);}
	void pxor(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PXOR, 0x0FEF, 0, dst, src);}

	// MMX2
	void pavgb(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PAVGB, 0x0FE0, 0, dst, src);}
	void pavgb(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PAVGB, 0x0FE0, 0, dst, src);}
	void pavgw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PAVGW, 0x0FE3, 0, dst, src);}
	void pavgw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PAVGW, 0x0FE3, 0, dst, src);}
	void pextrw(const Reg32& dst, const MmxReg& src, const Imm8& i)	{AppendInstr(I_PEXTRW, 0x0FC5, 0, dst, src, i);}
#ifdef JITASM64
	void pextrw(const Reg64& dst, const MmxReg& src, const Imm8& i)	{AppendInstr(I_PEXTRW, 0x0FC5, E_REXW_PREFIX, dst, src, i);}
#endif
	void pinsrw(const MmxReg& dst, const Reg32& src, const Imm8& i)	{AppendInstr(I_PINSRW, 0x0FC4, 0, dst, src, i);}
	void pinsrw(const MmxReg& dst, const Mem16& src, const Imm8& i)	{AppendInstr(I_PINSRW, 0x0FC4, 0, dst, src, i);}
#ifdef JITASM64
	void pinsrw(const MmxReg& dst, const Reg64& src, const Imm8& i)	{AppendInstr(I_PINSRW, 0x0FC4, E_REXW_PREFIX, dst, src, i);}
#endif
	void pmaxsw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMAXSW,	0x0FEE, 0, dst, src);}
	void pmaxsw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMAXSW,	0x0FEE, 0, dst, src);}
	void pmaxub(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMAXUB,	0x0FDE, 0, dst, src);}
	void pmaxub(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMAXUB,	0x0FDE, 0, dst, src);}
	void pminsw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMINSW,	0x0FEA, 0, dst, src);}
	void pminsw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMINSW,	0x0FEA, 0, dst, src);}
	void pminub(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMINUB,	0x0FDA, 0, dst, src);}
	void pminub(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMINUB,	0x0FDA, 0, dst, src);}
	void pmovmskb(const Reg32& dst, const MmxReg& src)	{AppendInstr(I_PMOVMSKB, 0x0FD7, 0, dst, src);}
#ifdef JITASM64
	void pmovmskb(const Reg64& dst, const MmxReg& src)	{AppendInstr(I_PMOVMSKB, 0x0FD7, E_REXW_PREFIX, dst, src);}
#endif
	void pmulhuw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMULHUW,	0x0FE4, 0, dst, src);}
	void pmulhuw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMULHUW,	0x0FE4, 0, dst, src);}
	void psadbw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSADBW,	0x0FF6, 0, dst, src);}
	void psadbw(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PSADBW,	0x0FF6, 0, dst, src);}
	void pshufw(const MmxReg& dst, const MmxReg& src, const Imm8& order)	{AppendInstr(I_PSHUFW, 0x0F70, 0, dst, src, order);}
	void pshufw(const MmxReg& dst, const Mem64& src, const Imm8& order)		{AppendInstr(I_PSHUFW, 0x0F70, 0, dst, src, order);}

	// SSE
	void addps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ADDPS,	0x0F58, 0, dst, src);}
	void addps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_ADDPS,	0x0F58, 0, dst, src);}
	void addss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ADDSS,	0x0F58, E_MANDATORY_PREFIX_F3, dst, src);}
	void addss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_ADDSS,	0x0F58, E_MANDATORY_PREFIX_F3, dst, src);}
	void andps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ANDPS,	0x0F54, 0, dst, src);}
	void andps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_ANDPS,	0x0F54, 0, dst, src);}
	void andnps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ANDNPS,	0x0F55, 0, dst, src);}
	void andnps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_ANDNPS,	0x0F55, 0, dst, src);}
	void cmpps(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{AppendInstr(I_CMPPS, 0x0FC2, 0, dst, src, opd3);}
	void cmpps(const XmmReg& dst, const Mem128& src, const Imm8& opd3)	{AppendInstr(I_CMPPS, 0x0FC2, 0, dst, src, opd3);}
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
	void cmpss(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{AppendInstr(I_CMPSS, 0x0FC2, E_MANDATORY_PREFIX_F3, dst, src, opd3);}
	void cmpss(const XmmReg& dst, const Mem32& src, const Imm8& opd3)	{AppendInstr(I_CMPSS, 0x0FC2, E_MANDATORY_PREFIX_F3, dst, src, opd3);}
	void cmpeqss(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 0);}
	void cmpeqss(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 0);}
	void cmpltss(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 1);}
	void cmpltss(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 1);}
	void cmpless(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 2);}
	void cmpless(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 2);}
	void cmpunordss(const XmmReg& dst, const XmmReg& src)	{cmpss(dst, src, 3);}
	void cmpunordss(const XmmReg& dst, const Mem32& src)	{cmpss(dst, src, 3);}
	void cmpneqss(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 4);}
	void cmpneqss(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 4);}
	void cmpnltss(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 5);}
	void cmpnltss(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 5);}
	void cmpnless(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 6);}
	void cmpnless(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 6);}
	void cmpordss(const XmmReg& dst, const XmmReg& src)		{cmpss(dst, src, 7);}
	void cmpordss(const XmmReg& dst, const Mem32& src)		{cmpss(dst, src, 7);}
	void comiss(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_COMISS,	0x0F2F, 0, dst, src);}
	void comiss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_COMISS,	0x0F2F, 0, dst, src);}
	void cvtpi2ps(const XmmReg& dst, const MmxReg& src)		{AppendInstr(I_CVTPI2PS, 0x0F2A, 0, dst, src);}
	void cvtpi2ps(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTPI2PS, 0x0F2A, 0, dst, src);}
	void cvtps2pi(const MmxReg& dst, const XmmReg& src)		{AppendInstr(I_CVTPS2PI, 0x0F2D, 0, dst, src);}
	void cvtps2pi(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_CVTPS2PI, 0x0F2D, 0, dst, src);}
	void cvtsi2ss(const XmmReg& dst, const Reg32& src)		{AppendInstr(I_CVTSI2SS, 0x0F2A, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvtsi2ss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_CVTSI2SS, 0x0F2A, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvtss2si(const Reg32& dst, const XmmReg& src)		{AppendInstr(I_CVTSS2SI, 0x0F2D, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvtss2si(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CVTSS2SI, 0x0F2D, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvttps2pi(const MmxReg& dst, const XmmReg& src)	{AppendInstr(I_CVTTPS2PI, 0x0F2C, 0, dst, src);}
	void cvttps2pi(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_CVTTPS2PI, 0x0F2C, 0, dst, src);}
	void cvttss2si(const Reg32& dst, const XmmReg& src)		{AppendInstr(I_CVTTSS2SI, 0x0F2C, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvttss2si(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CVTTSS2SI, 0x0F2C, E_MANDATORY_PREFIX_F3, dst, src);}
#ifdef JITASM64
	void cvtsi2ss(const XmmReg& dst, const Reg64& src)		{AppendInstr(I_CVTSI2SS, 0x0F2A, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
	void cvtsi2ss(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTSI2SS, 0x0F2A, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
	void cvtss2si(const Reg64& dst, const XmmReg& src)		{AppendInstr(I_CVTSS2SI, 0x0F2D, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
	void cvtss2si(const Reg64& dst, const Mem32& src)		{AppendInstr(I_CVTSS2SI, 0x0F2D, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
	void cvttss2si(const Reg64& dst, const XmmReg& src)		{AppendInstr(I_CVTTSS2SI, 0x0F2C, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
	void cvttss2si(const Reg64& dst, const Mem32& src)		{AppendInstr(I_CVTTSS2SI, 0x0F2C, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
#endif
	void divps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_DIVPS,	0x0F5E, 0, dst, src);}
	void divps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_DIVPS,	0x0F5E, 0, dst, src);}
	void divss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_DIVSS,	0x0F5E, E_MANDATORY_PREFIX_F3, dst, src);}
	void divss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_DIVSS,	0x0F5E, E_MANDATORY_PREFIX_F3, dst, src);}
	void ldmxcsr(const Mem32& src)						{AppendInstr(I_LDMXCSR,	0x0FAE, 0, Imm8(2), src);}
	void maskmovq(const MmxReg& dst, const MmxReg& mask){AppendInstr(I_MASKMOVQ,	0x0FF7, 0, dst, mask);}
	void maxps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MAXPS,	0x0F5F, 0, dst, src);}
	void maxps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MAXPS,	0x0F5F, 0, dst, src);}
	void maxss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MAXSS,	0x0F5F, E_MANDATORY_PREFIX_F3, dst, src);}
	void maxss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_MAXSS,	0x0F5F, E_MANDATORY_PREFIX_F3, dst, src);}
	void minps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MINPS,	0x0F5D, 0, dst, src);}
	void minps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MINPS,	0x0F5D, 0, dst, src);}
	void minss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MINSS,	0x0F5D, E_MANDATORY_PREFIX_F3, dst, src);}
	void minss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_MINSS,	0x0F5D, E_MANDATORY_PREFIX_F3, dst, src);}
	void movaps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVAPS,	0x0F28, 0, dst, src);}
	void movaps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MOVAPS,	0x0F28, 0, dst, src);}
	void movaps(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVAPS,	0x0F29, 0, src, dst);}
	void movhlps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVHLPS,	0x0F12, 0, dst, src);}
	void movhps(const XmmReg& dst, const Mem64& src)	{AppendInstr(I_MOVHPS,	0x0F16, 0, dst, src);}
	void movhps(const Mem64& dst, const XmmReg& src)	{AppendInstr(I_MOVHPS,	0x0F17, 0, src, dst);}
	void movlhps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVLHPS,	0x0F16, 0, dst, src);}
	void movlps(const XmmReg& dst, const Mem64& src)	{AppendInstr(I_MOVLPS,	0x0F12, 0, dst, src);}
	void movlps(const Mem64& dst, const XmmReg& src)	{AppendInstr(I_MOVLPS,	0x0F13, 0, src, dst);}
	void movmskps(const Reg32& dst, const XmmReg& src)	{AppendInstr(I_MOVMSKPS, 0x0F50, 0, dst, src);}
#ifdef JITASM64
	void movmskps(const Reg64& dst, const XmmReg& src)	{AppendInstr(I_MOVMSKPS, 0x0F50, E_REXW_PREFIX, dst, src);}
#endif
	void movntps(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVNTPS,	0x0F2B, 0, src, dst);}
	void movntq(const Mem64& dst, const MmxReg& src)	{AppendInstr(I_MOVNTQ,	0x0FE7, 0, src, dst);}
	void movss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVSS,	0x0F10, E_MANDATORY_PREFIX_F3, dst, src);}
	void movss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_MOVSS,	0x0F10, E_MANDATORY_PREFIX_F3, dst, src);}
	void movss(const Mem32& dst, const XmmReg& src)		{AppendInstr(I_MOVSS,	0x0F11, E_MANDATORY_PREFIX_F3, src, dst);}
	void movups(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVUPS,	0x0F10, 0, dst, src);}
	void movups(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MOVUPS,	0x0F10, 0, dst, src);}
	void movups(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVUPS,	0x0F11, 0, src, dst);}
	void mulps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MULPS,	0x0F59, 0, dst, src);}
	void mulps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MULPS,	0x0F59, 0, dst, src);}
	void mulss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MULSS,	0x0F59, E_MANDATORY_PREFIX_F3, dst, src);}
	void mulss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_MULSS,	0x0F59, E_MANDATORY_PREFIX_F3, dst, src);}
	void orps(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_ORPS,		0x0F56, 0, dst, src);}
	void orps(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_ORPS,		0x0F56, 0, dst, src);}
	void prefetcht0(const Mem8& mem)					{AppendInstr(I_PREFETCH,	0x0F18, 0, Imm8(1), mem);}
	void prefetcht1(const Mem8& mem)					{AppendInstr(I_PREFETCH,	0x0F18, 0, Imm8(2), mem);}
	void prefetcht2(const Mem8& mem)					{AppendInstr(I_PREFETCH,	0x0F18, 0, Imm8(3), mem);}
	void prefetchnta(const Mem8& mem)					{AppendInstr(I_PREFETCH,	0x0F18, 0, Imm8(0), mem);}
	void rcpps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_RCPPS,	0x0F53, 0, dst, src);}
	void rcpps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_RCPPS,	0x0F53, 0, dst, src);}
	void rcpss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_RCPSS,	0x0F53, E_MANDATORY_PREFIX_F3, dst, src);}
	void rcpss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_RCPSS,	0x0F53, E_MANDATORY_PREFIX_F3, dst, src);}
	void rsqrtps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_RSQRTPS,	0x0F52, 0, dst, src);}
	void rsqrtps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_RSQRTPS,	0x0F52, 0, dst, src);}
	void rsqrtss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_RSQRTSS,	0x0F52, E_MANDATORY_PREFIX_F3, dst, src);}
	void rsqrtss(const XmmReg& dst, const Mem32& src)	{AppendInstr(I_RSQRTSS,	0x0F52, E_MANDATORY_PREFIX_F3, dst, src);}
	void sfence()										{AppendInstr(I_SFENCE,	0x0FAEF8, 0);}
	void shufps(const XmmReg& dst, const XmmReg& src, const Imm8& sel)	{AppendInstr(I_SHUFPS, 0x0FC6, 0, dst, src, sel);}
	void shufps(const XmmReg& dst, const Mem128& src, const Imm8& sel)	{AppendInstr(I_SHUFPS, 0x0FC6, 0, dst, src, sel);}
	void sqrtps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_SQRTPS,	0x0F51, 0, dst, src);}
	void sqrtps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_SQRTPS,	0x0F51, 0, dst, src);}
	void sqrtss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_SQRTSS,	0x0F51, E_MANDATORY_PREFIX_F3, dst, src);}
	void sqrtss(const XmmReg& dst, const Mem32& src)	{AppendInstr(I_SQRTSS,	0x0F51, E_MANDATORY_PREFIX_F3, dst, src);}
	void stmxcsr(const Mem32& dst)						{AppendInstr(I_STMXCSR,	0x0FAE, 0, Imm8(3), dst);}
	void subps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_SUBPS,	0x0F5C, 0, dst, src);}
	void subps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_SUBPS,	0x0F5C, 0, dst, src);}
	void subss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_SUBSS,	0x0F5C, E_MANDATORY_PREFIX_F3, dst, src);}
	void subss(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_SUBSS,	0x0F5C, E_MANDATORY_PREFIX_F3, dst, src);}
	void ucomiss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_UCOMISS,	0x0F2E, 0, dst, src);}
	void ucomiss(const XmmReg& dst, const Mem32& src)	{AppendInstr(I_UCOMISS,	0x0F2E, 0, dst, src);}
	void unpckhps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_UNPCKHPS, 0x0F15, 0, dst, src);}
	void unpckhps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_UNPCKHPS, 0x0F15, 0, dst, src);}
	void unpcklps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_UNPCKLPS, 0x0F14, 0, dst, src);}
	void unpcklps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_UNPCKLPS, 0x0F14, 0, dst, src);}
	void xorps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_XORPS,	0x0F57, 0, dst, src);}
	void xorps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_XORPS,	0x0F57, 0, dst, src);}

	// SSE2
	void addpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ADDPD,	0x0F58, E_MANDATORY_PREFIX_66, dst, src);}
	void addpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_ADDPD,	0x0F58, E_MANDATORY_PREFIX_66, dst, src);}
	void addsd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ADDSD,	0x0F58, E_MANDATORY_PREFIX_F2, dst, src);}
	void addsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_ADDSD,	0x0F58, E_MANDATORY_PREFIX_F2, dst, src);}
	void andpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ANDPD,	0x0F54, E_MANDATORY_PREFIX_66, dst, src);}
	void andpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_ANDPD,	0x0F54, E_MANDATORY_PREFIX_66, dst, src);}
	void andnpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_ANDNPD,	0x0F55, E_MANDATORY_PREFIX_66, dst, src);}
	void andnpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_ANDNPD,	0x0F55, E_MANDATORY_PREFIX_66, dst, src);}
	void clflush(const Mem8& dst) {AppendInstr(I_CLFLUSH, 0x0FAE, 0, Imm8(7), dst);}
	void cmppd(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{AppendInstr(I_CMPPD, 0x0FC2, E_MANDATORY_PREFIX_66, dst, src, opd3);}
	void cmppd(const XmmReg& dst, const Mem128& src, const Imm8& opd3)	{AppendInstr(I_CMPPD, 0x0FC2, E_MANDATORY_PREFIX_66, dst, src, opd3);}
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
	void cmpsd(const XmmReg& dst, const XmmReg& src, const Imm8& opd3)	{AppendInstr(I_CMPSD, 0x0FC2, E_MANDATORY_PREFIX_F2, dst, src, opd3);}
	void cmpsd(const XmmReg& dst, const Mem64& src, const Imm8& opd3)	{AppendInstr(I_CMPSD, 0x0FC2, E_MANDATORY_PREFIX_F2, dst, src, opd3);}
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
	void comisd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_COMISD,	0x0F2F, E_MANDATORY_PREFIX_66, dst, src);}
	void comisd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_COMISD,	0x0F2F, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtdq2pd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTDQ2PD, 0x0FE6, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvtdq2pd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTDQ2PD, 0x0FE6, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvtpd2dq(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTPD2DQ, 0x0FE6, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvtpd2dq(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_CVTPD2DQ, 0x0FE6, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvtpd2pi(const MmxReg& dst, const XmmReg& src)		{AppendInstr(I_CVTPD2PI, 0x0F2D, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtpd2pi(const MmxReg& dst, const Mem128& src)		{AppendInstr(I_CVTPD2PI, 0x0F2D, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtpd2ps(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTPD2PS, 0x0F5A, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtpd2ps(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_CVTPD2PS, 0x0F5A, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtpi2pd(const XmmReg& dst, const MmxReg& src)		{AppendInstr(I_CVTPI2PD, 0x0F2A, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtpi2pd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTPI2PD, 0x0F2A, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtps2dq(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTPS2DQ, 0x0F5B, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtps2dq(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_CVTPS2DQ, 0x0F5B, E_MANDATORY_PREFIX_66, dst, src);}
	void cvtdq2ps(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTDQ2PS, 0x0F5B, 0, dst, src);}
	void cvtdq2ps(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_CVTDQ2PS, 0x0F5B, 0, dst, src);}
	void cvtps2pd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTPS2PD, 0x0F5A, 0, dst, src);}
	void cvtps2pd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTPS2PD, 0x0F5A, 0, dst, src);}
	void cvtsd2si(const Reg32& dst, const XmmReg& src)		{AppendInstr(I_CVTSD2SI, 0x0F2D, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvtsd2si(const Reg32& dst, const Mem64& src)		{AppendInstr(I_CVTSD2SI, 0x0F2D, E_MANDATORY_PREFIX_F2, dst, src);}
#ifdef JITASM64
	void cvtsd2si(const Reg64& dst, const XmmReg& src)		{AppendInstr(I_CVTSD2SI, 0x0F2D, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
	void cvtsd2si(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CVTSD2SI, 0x0F2D, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
#endif
	void cvtsd2ss(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTSD2SS, 0x0F5A, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvtsd2ss(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTSD2SS, 0x0F5A, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvtsi2sd(const XmmReg& dst, const Reg32& src)		{AppendInstr(I_CVTSI2SD, 0x0F2A, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvtsi2sd(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_CVTSI2SD, 0x0F2A, E_MANDATORY_PREFIX_F2, dst, src);}
#ifdef JITASM64
	void cvtsi2sd(const XmmReg& dst, const Reg64& src)		{AppendInstr(I_CVTSI2SD, 0x0F2A, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
	void cvtsi2sd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTSI2SD, 0x0F2A, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
#endif
	void cvtss2sd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_CVTSS2SD,  0x0F5A, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvtss2sd(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_CVTSS2SD,  0x0F5A, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvttpd2dq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTTPD2DQ, 0x0FE6, E_MANDATORY_PREFIX_66, dst, src);}
	void cvttpd2dq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTTPD2DQ, 0x0FE6, E_MANDATORY_PREFIX_66, dst, src);}
	void cvttpd2pi(const MmxReg& dst, const XmmReg& src)	{AppendInstr(I_CVTTPD2PI, 0x0F2C, E_MANDATORY_PREFIX_66, dst, src);}
	void cvttpd2pi(const MmxReg& dst, const Mem128& src)	{AppendInstr(I_CVTTPD2PI, 0x0F2C, E_MANDATORY_PREFIX_66, dst, src);}
	void cvttps2dq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTTPS2DQ, 0x0F5B, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvttps2dq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTTPS2DQ, 0x0F5B, E_MANDATORY_PREFIX_F3, dst, src);}
	void cvttsd2si(const Reg32& dst, const XmmReg& src)		{AppendInstr(I_CVTTSD2SI, 0x0F2C, E_MANDATORY_PREFIX_F2, dst, src);}
	void cvttsd2si(const Reg32& dst, const Mem64& src)		{AppendInstr(I_CVTTSD2SI, 0x0F2C, E_MANDATORY_PREFIX_F2, dst, src);}
#ifdef JITASM64
	void cvttsd2si(const Reg64& dst, const XmmReg& src)		{AppendInstr(I_CVTTSD2SI, 0x0F2C, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
	void cvttsd2si(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CVTTSD2SI, 0x0F2C, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
#endif
	void divpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_DIVPD,	0x0F5E, E_MANDATORY_PREFIX_66, dst, src);}
	void divpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_DIVPD,	0x0F5E, E_MANDATORY_PREFIX_66, dst, src);}
	void divsd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_DIVSD,	0x0F5E, E_MANDATORY_PREFIX_F2, dst, src);}
	void divsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_DIVSD,	0x0F5E, E_MANDATORY_PREFIX_F2, dst, src);}
	void lfence()										{AppendInstr(I_LFENCE,	0x0FAEE8, 0);}
	void maskmovdqu(const XmmReg& dst, const XmmReg& mask)	{AppendInstr(I_MASKMOVDQU, 0x0FF7, E_MANDATORY_PREFIX_66, dst, mask);}
	void maxpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MAXPD,	0x0F5F, E_MANDATORY_PREFIX_66, dst, src);}
	void maxpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MAXPD,	0x0F5F, E_MANDATORY_PREFIX_66, dst, src);}
	void maxsd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MAXSD,	0x0F5F, E_MANDATORY_PREFIX_F2, dst, src);}
	void maxsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_MAXSD,	0x0F5F, E_MANDATORY_PREFIX_F2, dst, src);}
	void mfence()										{AppendInstr(I_MFENCE,	0x0FAEF0, 0);}
	void minpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MINPD,	0x0F5D, E_MANDATORY_PREFIX_66, dst, src);}
	void minpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MINPD,	0x0F5D, E_MANDATORY_PREFIX_66, dst, src);}
	void minsd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MINSD,	0x0F5D, E_MANDATORY_PREFIX_F2, dst, src);}
	void minsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_MINSD,	0x0F5D, E_MANDATORY_PREFIX_F2, dst, src);}
	void movapd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVAPD,	0x0F28, E_MANDATORY_PREFIX_66, dst, src);}
	void movapd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MOVAPD,	0x0F28, E_MANDATORY_PREFIX_66, dst, src);}
	void movapd(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVAPD,	0x0F29, E_MANDATORY_PREFIX_66, src, dst);}
	void movd(const XmmReg& dst, const Reg32& src)		{AppendInstr(I_MOVD,		0x0F6E, E_MANDATORY_PREFIX_66, dst, src);}
	void movd(const XmmReg& dst, const Mem32& src)		{AppendInstr(I_MOVD,		0x0F6E, E_MANDATORY_PREFIX_66, dst, src);}
	void movd(const Reg32& dst, const XmmReg& src)		{AppendInstr(I_MOVD,		0x0F7E, E_MANDATORY_PREFIX_66, src, dst);}
	void movd(const Mem32& dst, const XmmReg& src)		{AppendInstr(I_MOVD,		0x0F7E, E_MANDATORY_PREFIX_66, src, dst);}
#ifdef JITASM64
	void movd(const XmmReg& dst, const Reg64& src)		{AppendInstr(I_MOVD,		0x0F6E, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src);}
	void movd(const Reg64& dst, const XmmReg& src)		{AppendInstr(I_MOVD,		0x0F7E, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, src, dst);}
#endif
	void movdqa(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVDQA,	0x0F6F, E_MANDATORY_PREFIX_66, dst, src);}
	void movdqa(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MOVDQA,	0x0F6F, E_MANDATORY_PREFIX_66, dst, src);}
	void movdqa(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVDQA,	0x0F7F, E_MANDATORY_PREFIX_66, src, dst);}
	void movdqu(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVDQU,	0x0F6F, E_MANDATORY_PREFIX_F3, dst, src);}
	void movdqu(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MOVDQU,	0x0F6F, E_MANDATORY_PREFIX_F3, dst, src);}
	void movdqu(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVDQU,	0x0F7F, E_MANDATORY_PREFIX_F3, src, dst);}
	void movdq2q(const MmxReg& dst, const XmmReg& src)	{AppendInstr(I_MOVDQ2Q,	0x0FD6, E_MANDATORY_PREFIX_F2, dst, src);}
	void movhpd(const XmmReg& dst, const Mem64& src)	{AppendInstr(I_MOVHPD,	0x0F16, E_MANDATORY_PREFIX_66, dst, src);}
	void movhpd(const Mem64& dst, const XmmReg& src)	{AppendInstr(I_MOVHPD,	0x0F17, E_MANDATORY_PREFIX_66, src, dst);}
	void movlpd(const XmmReg& dst, const Mem64& src)	{AppendInstr(I_MOVLPD,	0x0F12, E_MANDATORY_PREFIX_66, dst, src);}
	void movlpd(const Mem64& dst, const XmmReg& src)	{AppendInstr(I_MOVLPD,	0x0F13, E_MANDATORY_PREFIX_66, src, dst);}
	void movmskpd(const Reg32& dst, XmmReg& src)		{AppendInstr(I_MOVMSKPD, 0x0F50, E_MANDATORY_PREFIX_66, dst, src);}
#ifdef JITASM64
	void movmskpd(const Reg64& dst, XmmReg& src)		{AppendInstr(I_MOVMSKPD, 0x0F50, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src);}
#endif
	void movntdq(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVNTDQ,	0x0FE7, E_MANDATORY_PREFIX_66, src, dst);}
	void movnti(const Mem32& dst, const Reg32& src)		{AppendInstr(I_MOVNTI,	0x0FC3, 0, src, dst);}
#ifdef JITASM64
	void movnti(const Mem64& dst, const Reg64& src)		{AppendInstr(I_MOVNTI,	0x0FC3, E_REXW_PREFIX, src, dst);}
#endif
	void movntpd(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVNTPD,	0x0F2B, E_MANDATORY_PREFIX_66, src, dst);}
	void movq(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_MOVQ,		0x0F7E, E_MANDATORY_PREFIX_F3, dst, src);}
	void movq(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_MOVQ,		0x0F7E, E_MANDATORY_PREFIX_F3, dst, src);}
	void movq(const Mem64& dst, const XmmReg& src)		{AppendInstr(I_MOVQ,		0x0FD6, E_MANDATORY_PREFIX_66, src, dst);}
#ifdef JITASM64
	void movq(const XmmReg& dst, const Reg64& src)		{movd(dst, src);}
	void movq(const Reg64& dst, const XmmReg& src)		{movd(dst, src);}
#endif
	void movq2dq(const XmmReg& dst, const MmxReg& src)	{AppendInstr(I_MOVQ2DQ,	0x0FD6, E_MANDATORY_PREFIX_F3, dst, src);}
	void movsd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVSD,	0x0F10, E_MANDATORY_PREFIX_F2, dst, src);}
	void movsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_MOVSD,	0x0F10, E_MANDATORY_PREFIX_F2, dst, src);}
	void movsd(const Mem64& dst, const XmmReg& src)		{AppendInstr(I_MOVSD,	0x0F11, E_MANDATORY_PREFIX_F2, src, dst);}
	void movupd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MOVUPD,	0x0F10, E_MANDATORY_PREFIX_66, dst, src);}
	void movupd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MOVUPD,	0x0F10, E_MANDATORY_PREFIX_66, dst, src);}
	void movupd(const Mem128& dst, const XmmReg& src)	{AppendInstr(I_MOVUPD,	0x0F11, E_MANDATORY_PREFIX_66, src, dst);}
	void mulpd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MULPD,	0x0F59, E_MANDATORY_PREFIX_66, dst, src);}
	void mulpd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_MULPD,	0x0F59, E_MANDATORY_PREFIX_66, dst, src);}
	void mulsd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_MULSD,	0x0F59, E_MANDATORY_PREFIX_F2, dst, src);}
	void mulsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_MULSD,	0x0F59, E_MANDATORY_PREFIX_F2, dst, src);}
	void orpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_ORPD,		0x0F56, E_MANDATORY_PREFIX_66, dst, src);}
	void orpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_ORPD,		0x0F56, E_MANDATORY_PREFIX_66, dst, src);}
	void packsswb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PACKSSWB, 0x0F63, E_MANDATORY_PREFIX_66, dst, src);}
	void packsswb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PACKSSWB, 0x0F63, E_MANDATORY_PREFIX_66, dst, src);}
	void packssdw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PACKSSDW, 0x0F6B, E_MANDATORY_PREFIX_66, dst, src);}
	void packssdw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PACKSSDW, 0x0F6B, E_MANDATORY_PREFIX_66, dst, src);}
	void packuswb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PACKUSWB, 0x0F67, E_MANDATORY_PREFIX_66, dst, src);}
	void packuswb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PACKUSWB, 0x0F67, E_MANDATORY_PREFIX_66, dst, src);}
	void paddb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDB,	0x0FFC, E_MANDATORY_PREFIX_66, dst, src);}
	void paddb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDB,	0x0FFC, E_MANDATORY_PREFIX_66, dst, src);}
	void paddw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDW,	0x0FFD, E_MANDATORY_PREFIX_66, dst, src);}
	void paddw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDW,	0x0FFD, E_MANDATORY_PREFIX_66, dst, src);}
	void paddd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDD,	0x0FFE, E_MANDATORY_PREFIX_66, dst, src);}
	void paddd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDD,	0x0FFE, E_MANDATORY_PREFIX_66, dst, src);}
	void paddq(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PADDQ,	0x0FD4, 0, dst, src);}
	void paddq(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PADDQ,	0x0FD4, 0, dst, src);}
	void paddq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDQ,	0x0FD4, E_MANDATORY_PREFIX_66, dst, src);}
	void paddq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDQ,	0x0FD4, E_MANDATORY_PREFIX_66, dst, src);}
	void paddsb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDSB,	0x0FEC, E_MANDATORY_PREFIX_66, dst, src);}
	void paddsb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDSB,	0x0FEC, E_MANDATORY_PREFIX_66, dst, src);}
	void paddsw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDSW,	0x0FED, E_MANDATORY_PREFIX_66, dst, src);}
	void paddsw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDSW,	0x0FED, E_MANDATORY_PREFIX_66, dst, src);}
	void paddusb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDUSB,	0x0FDC, E_MANDATORY_PREFIX_66, dst, src);}
	void paddusb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDUSB,	0x0FDC, E_MANDATORY_PREFIX_66, dst, src);}
	void paddusw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PADDUSW,	0x0FDD, E_MANDATORY_PREFIX_66, dst, src);}
	void paddusw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PADDUSW,	0x0FDD, E_MANDATORY_PREFIX_66, dst, src);}
	void pand(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PAND,		0x0FDB, E_MANDATORY_PREFIX_66, dst, src);}
	void pand(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PAND,		0x0FDB, E_MANDATORY_PREFIX_66, dst, src);}
	void pandn(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PANDN,	0x0FDF, E_MANDATORY_PREFIX_66, dst, src);}
	void pandn(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PANDN,	0x0FDF, E_MANDATORY_PREFIX_66, dst, src);}
	void pause()										{AppendInstr(I_PAUSE,	0xF390,	0);}
	void pavgb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PAVGB,	0x0FE0, E_MANDATORY_PREFIX_66, dst, src);}
	void pavgb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PAVGB,	0x0FE0, E_MANDATORY_PREFIX_66, dst, src);}
	void pavgw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PAVGW,	0x0FE3, E_MANDATORY_PREFIX_66, dst, src);}
	void pavgw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PAVGW,	0x0FE3, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PCMPEQB,	0x0F74, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PCMPEQB,	0x0F74, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PCMPEQW,	0x0F75, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PCMPEQW,	0x0F75, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PCMPEQD,	0x0F76, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PCMPEQD,	0x0F76, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PCMPGTB,	0x0F64, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PCMPGTB,	0x0F64, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PCMPGTW,	0x0F65, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PCMPGTW,	0x0F65, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PCMPGTD,	0x0F66, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PCMPGTD,	0x0F66, E_MANDATORY_PREFIX_66, dst, src);}
	void pextrw(const Reg32& dst, const XmmReg& src, const Imm8& i)	{AppendInstr(I_PEXTRW, 0x0FC5, E_MANDATORY_PREFIX_66, dst, src, i);}
#ifdef JITASM64
	void pextrw(const Reg64& dst, const XmmReg& src, const Imm8& i)	{AppendInstr(I_PEXTRW, 0x0FC5, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src, i);}
#endif
	void pinsrw(const XmmReg& dst, const Reg32& src, const Imm8& i)	{AppendInstr(I_PINSRW, 0x0FC4, E_MANDATORY_PREFIX_66, dst, src, i);}
	void pinsrw(const XmmReg& dst, const Mem16& src, const Imm8& i)	{AppendInstr(I_PINSRW, 0x0FC4, E_MANDATORY_PREFIX_66, dst, src, i);}
#ifdef JITASM64
	void pinsrw(const XmmReg& dst, const Reg64& src, const Imm8& i)	{AppendInstr(I_PINSRW, 0x0FC4, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src, i);}
#endif
	void pmaddwd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMADDWD,	0x0FF5, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaddwd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMADDWD,	0x0FF5, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxsw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMAXSW,	0x0FEE, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxsw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMAXSW,	0x0FEE, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxub(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMAXUB,	0x0FDE, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxub(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMAXUB,	0x0FDE, E_MANDATORY_PREFIX_66, dst, src);}
	void pminsw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMINSW,	0x0FEA, E_MANDATORY_PREFIX_66, dst, src);}
	void pminsw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMINSW,	0x0FEA, E_MANDATORY_PREFIX_66, dst, src);}
	void pminub(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMINUB,	0x0FDA, E_MANDATORY_PREFIX_66, dst, src);}
	void pminub(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMINUB,	0x0FDA, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovmskb(const Reg32& dst, const XmmReg& src)	{AppendInstr(I_PMOVMSKB, 0x0FD7, E_MANDATORY_PREFIX_66, dst, src);}
#ifdef JITASM64
	void pmovmskb(const Reg64& dst, const XmmReg& src)	{AppendInstr(I_PMOVMSKB, 0x0FD7, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src);}
#endif
	void pmulhuw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMULHUW,	0x0FE4, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulhuw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMULHUW,	0x0FE4, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulhw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMULHW,	0x0FE5, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulhw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMULHW,	0x0FE5, E_MANDATORY_PREFIX_66, dst, src);}
	void pmullw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMULLW,	0x0FD5, E_MANDATORY_PREFIX_66, dst, src);}
	void pmullw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMULLW,	0x0FD5, E_MANDATORY_PREFIX_66, dst, src);}
	void pmuludq(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMULUDQ,	0x0FF4, 0, dst, src);}
	void pmuludq(const MmxReg& dst, const Mem64& src)	{AppendInstr(I_PMULUDQ,	0x0FF4, 0, dst, src);}
	void pmuludq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMULUDQ,	0x0FF4, E_MANDATORY_PREFIX_66, dst, src);}
	void pmuludq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMULUDQ,	0x0FF4, E_MANDATORY_PREFIX_66, dst, src);}
	void por(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_POR,		0x0FEB, E_MANDATORY_PREFIX_66, dst, src);}
	void por(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_POR,		0x0FEB, E_MANDATORY_PREFIX_66, dst, src);}
	void psadbw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSADBW,	0x0FF6, E_MANDATORY_PREFIX_66, dst, src);}
	void psadbw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSADBW,	0x0FF6, E_MANDATORY_PREFIX_66, dst, src);}
	void pshufd(const XmmReg& dst, const XmmReg& src, const Imm8& order)	{AppendInstr(I_PSHUFD,	0x0F70, E_MANDATORY_PREFIX_66, dst, src, order);}
	void pshufd(const XmmReg& dst, const Mem128& src, const Imm8& order)	{AppendInstr(I_PSHUFD,	0x0F70, E_MANDATORY_PREFIX_66, dst, src, order);}
	void pshufhw(const XmmReg& dst, const XmmReg& src, const Imm8& order)	{AppendInstr(I_PSHUFHW,	0x0F70, E_MANDATORY_PREFIX_F3, dst, src, order);}
	void pshufhw(const XmmReg& dst, const Mem128& src, const Imm8& order)	{AppendInstr(I_PSHUFHW,	0x0F70, E_MANDATORY_PREFIX_F3, dst, src, order);}
	void pshuflw(const XmmReg& dst, const XmmReg& src, const Imm8& order)	{AppendInstr(I_PSHUFLW,	0x0F70, E_MANDATORY_PREFIX_F2, dst, src, order);}
	void pshuflw(const XmmReg& dst, const Mem128& src, const Imm8& order)	{AppendInstr(I_PSHUFLW,	0x0F70, E_MANDATORY_PREFIX_F2, dst, src, order);}
	void psllw(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSLLW,	0x0FF1, E_MANDATORY_PREFIX_66, dst, count);}
	void psllw(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSLLW,	0x0FF1, E_MANDATORY_PREFIX_66, dst, count);}
	void psllw(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSLLW,	0x0F71, E_MANDATORY_PREFIX_66, Imm8(6), dst, count);}
	void pslld(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSLLD,	0x0FF2, E_MANDATORY_PREFIX_66, dst, count);}
	void pslld(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSLLD,	0x0FF2, E_MANDATORY_PREFIX_66, dst, count);}
	void pslld(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSLLD,	0x0F72, E_MANDATORY_PREFIX_66, Imm8(6), dst, count);}
	void psllq(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSLLQ,	0x0FF3, E_MANDATORY_PREFIX_66, dst, count);}
	void psllq(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSLLQ,	0x0FF3, E_MANDATORY_PREFIX_66, dst, count);}
	void psllq(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSLLQ,	0x0F73, E_MANDATORY_PREFIX_66, Imm8(6), dst, count);}
	void pslldq(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSLLDQ,	0x0F73, E_MANDATORY_PREFIX_66, Imm8(7), dst, count);}
	void psraw(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSRAW,	0x0FE1, E_MANDATORY_PREFIX_66, dst, count);}
	void psraw(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSRAW,	0x0FE1, E_MANDATORY_PREFIX_66, dst, count);}
	void psraw(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSRAW,	0x0F71, E_MANDATORY_PREFIX_66, Imm8(4), dst, count);}
	void psrad(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSRAD,	0x0FE2, E_MANDATORY_PREFIX_66, dst, count);}
	void psrad(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSRAD,	0x0FE2, E_MANDATORY_PREFIX_66, dst, count);}
	void psrad(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSRAD,	0x0F72, E_MANDATORY_PREFIX_66, Imm8(4), dst, count);}
	void psrlw(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSRLW,	0x0FD1, E_MANDATORY_PREFIX_66, dst, count);}
	void psrlw(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSRLW,	0x0FD1, E_MANDATORY_PREFIX_66, dst, count);}
	void psrlw(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSRLW,	0x0F71, E_MANDATORY_PREFIX_66, Imm8(2), dst, count);}
	void psrld(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSRLD,	0x0FD2, E_MANDATORY_PREFIX_66, dst, count);}
	void psrld(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSRLD,	0x0FD2, E_MANDATORY_PREFIX_66, dst, count);}
	void psrld(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSRLD,	0x0F72, E_MANDATORY_PREFIX_66, Imm8(2), dst, count);}
	void psrlq(const XmmReg& dst, const XmmReg& count)	{AppendInstr(I_PSRLQ,	0x0FD3, E_MANDATORY_PREFIX_66, dst, count);}
	void psrlq(const XmmReg& dst, const Mem128& count)	{AppendInstr(I_PSRLQ,	0x0FD3, E_MANDATORY_PREFIX_66, dst, count);}
	void psrlq(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSRLQ,	0x0F73, E_MANDATORY_PREFIX_66, Imm8(2), dst, count);}
	void psrldq(const XmmReg& dst, const Imm8& count)	{AppendInstr(I_PSRLDQ,	0x0F73, E_MANDATORY_PREFIX_66, Imm8(3), dst, count);}
	void psubb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBB,	0x0FF8, E_MANDATORY_PREFIX_66, dst, src);}
	void psubb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBB,	0x0FF8, E_MANDATORY_PREFIX_66, dst, src);}
	void psubw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBW,	0x0FF9, E_MANDATORY_PREFIX_66, dst, src);}
	void psubw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBW,	0x0FF9, E_MANDATORY_PREFIX_66, dst, src);}
	void psubd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBD,	0x0FFA, E_MANDATORY_PREFIX_66, dst, src);}
	void psubd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBD,	0x0FFA, E_MANDATORY_PREFIX_66, dst, src);}
	void psubq(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PSUBQ,	0x0FFB, 0, dst, src);}
	void psubq(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSUBQ,	0x0FFB, 0, dst, src);}
	void psubq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBQ,	0x0FFB, E_MANDATORY_PREFIX_66, dst, src);}
	void psubq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBQ,	0x0FFB, E_MANDATORY_PREFIX_66, dst, src);}
	void psubsb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBSB,	0x0FE8, E_MANDATORY_PREFIX_66, dst, src);}
	void psubsb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBSB,	0x0FE8, E_MANDATORY_PREFIX_66, dst, src);}
	void psubsw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBSW,	0x0FE9, E_MANDATORY_PREFIX_66, dst, src);}
	void psubsw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBSW,	0x0FE9, E_MANDATORY_PREFIX_66, dst, src);}
	void psubusb(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBUSB,	0x0FD8, E_MANDATORY_PREFIX_66, dst, src);}
	void psubusb(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBUSB,	0x0FD8, E_MANDATORY_PREFIX_66, dst, src);}
	void psubusw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PSUBUSW,	0x0FD9, E_MANDATORY_PREFIX_66, dst, src);}
	void psubusw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PSUBUSW,	0x0FD9, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhbw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKHBW,	0x0F68, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhbw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKHBW,	0x0F68, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhwd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKHWD,	0x0F69, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhwd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKHWD,	0x0F69, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhdq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKHDQ,	0x0F6A, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhdq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKHDQ,	0x0F6A, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhqdq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKHQDQ,	0x0F6D, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckhqdq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKHQDQ,	0x0F6D, E_MANDATORY_PREFIX_66, dst, src);}
	void punpcklbw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKLBW,	0x0F60, E_MANDATORY_PREFIX_66, dst, src);}
	void punpcklbw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKLBW,	0x0F60, E_MANDATORY_PREFIX_66, dst, src);}
	void punpcklwd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKLWD,	0x0F61, E_MANDATORY_PREFIX_66, dst, src);}
	void punpcklwd(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKLWD,	0x0F61, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckldq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKLDQ,	0x0F62, E_MANDATORY_PREFIX_66, dst, src);}
	void punpckldq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKLDQ,	0x0F62, E_MANDATORY_PREFIX_66, dst, src);}
	void punpcklqdq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PUNPCKLQDQ,	0x0F6C, E_MANDATORY_PREFIX_66, dst, src);}
	void punpcklqdq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PUNPCKLQDQ,	0x0F6C, E_MANDATORY_PREFIX_66, dst, src);}
	void pxor(const XmmReg& dst, const XmmReg& src)			{AppendInstr(I_PXOR,			0x0FEF, E_MANDATORY_PREFIX_66, dst, src);}
	void pxor(const XmmReg& dst, const Mem128& src)			{AppendInstr(I_PXOR,			0x0FEF, E_MANDATORY_PREFIX_66, dst, src);}
	void shufpd(const XmmReg& dst, const XmmReg& src, const Imm8& sel)	{AppendInstr(I_SHUFPD, 0x0FC6, E_MANDATORY_PREFIX_66, dst, src, sel);}
	void shufpd(const XmmReg& dst, const Mem128& src, const Imm8& sel)	{AppendInstr(I_SHUFPD, 0x0FC6, E_MANDATORY_PREFIX_66, dst, src, sel);}
	void sqrtpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_SQRTPD,	0x0F51, E_MANDATORY_PREFIX_66, dst, src);}
	void sqrtpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_SQRTPD,	0x0F51, E_MANDATORY_PREFIX_66, dst, src);}
	void sqrtsd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_SQRTSD,	0x0F51, E_MANDATORY_PREFIX_F2, dst, src);}
	void sqrtsd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_SQRTSD,	0x0F51, E_MANDATORY_PREFIX_F2, dst, src);}
	void subpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_SUBPD,	0x0F5C, E_MANDATORY_PREFIX_66, dst, src);}
	void subpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_SUBPD,	0x0F5C, E_MANDATORY_PREFIX_66, dst, src);}
	void subsd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_SUBSD,	0x0F5C, E_MANDATORY_PREFIX_F2, dst, src);}
	void subsd(const XmmReg& dst, const Mem64& src)			{AppendInstr(I_SUBSD,	0x0F5C, E_MANDATORY_PREFIX_F2, dst, src);}
	void ucomisd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_UCOMISD,	0x0F2E, E_MANDATORY_PREFIX_66, dst, src);}
	void ucomisd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_UCOMISD,	0x0F2E, E_MANDATORY_PREFIX_66, dst, src);}
	void unpckhpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_UNPCKHPD, 0x0F15, E_MANDATORY_PREFIX_66, dst, src);}
	void unpckhpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_UNPCKHPD, 0x0F15, E_MANDATORY_PREFIX_66, dst, src);}
	void unpcklpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_UNPCKLPD, 0x0F14, E_MANDATORY_PREFIX_66, dst, src);}
	void unpcklpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_UNPCKLPD, 0x0F14, E_MANDATORY_PREFIX_66, dst, src);}
	void xorpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_XORPD,	0x0F57, E_MANDATORY_PREFIX_66, dst, src);}
	void xorpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_XORPD,	0x0F57, E_MANDATORY_PREFIX_66, dst, src);}

	// SSE3
	void addsubps(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_ADDSUBPS,	0x0FD0, E_MANDATORY_PREFIX_F2, dst, src);}
	void addsubps(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_ADDSUBPS,	0x0FD0, E_MANDATORY_PREFIX_F2, dst, src);}
	void addsubpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_ADDSUBPD,	0x0FD0, E_MANDATORY_PREFIX_66, dst, src);}
	void addsubpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_ADDSUBPD,	0x0FD0, E_MANDATORY_PREFIX_66, dst, src);}
	void fisttp(const Mem16& dst)							{AppendInstr(I_FISTTP,	0xDF, 0, Imm8(1), dst);}
	void fisttp(const Mem32& dst)							{AppendInstr(I_FISTTP,	0xDB, 0, Imm8(1), dst);}
	void fisttp(const Mem64& dst)							{AppendInstr(I_FISTTP,	0xDD, 0, Imm8(1), dst);}
	void haddps(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_HADDPS,	0x0F7C, E_MANDATORY_PREFIX_F2, dst, src);}
	void haddps(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_HADDPS,	0x0F7C, E_MANDATORY_PREFIX_F2, dst, src);}
	void haddpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_HADDPD,	0x0F7C, E_MANDATORY_PREFIX_66, dst, src);}
	void haddpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_HADDPD,	0x0F7C, E_MANDATORY_PREFIX_66, dst, src);}
	void hsubps(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_HSUBPS,	0x0F7D, E_MANDATORY_PREFIX_F2, dst, src);}
	void hsubps(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_HSUBPS,	0x0F7D, E_MANDATORY_PREFIX_F2, dst, src);}
	void hsubpd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_HSUBPD,	0x0F7D, E_MANDATORY_PREFIX_66, dst, src);}
	void hsubpd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_HSUBPD,	0x0F7D, E_MANDATORY_PREFIX_66, dst, src);}
	void lddqu(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_LDDQU,	0x0FF0, E_MANDATORY_PREFIX_F2, dst, src);}
	void monitor()											{AppendInstr(I_MONITOR,	0x0F01C8, 0);}
	void movddup(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_MOVDDUP,	0x0F12, E_MANDATORY_PREFIX_F2, dst, src);}
	void movddup(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_MOVDDUP,	0x0F12, E_MANDATORY_PREFIX_F2, dst, src);}
	void movshdup(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_MOVSHDUP,	0x0F16, E_MANDATORY_PREFIX_F3, dst, src);}
	void movshdup(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_MOVSHDUP,	0x0F16, E_MANDATORY_PREFIX_F3, dst, src);}
	void movsldup(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_MOVSLDUP,	0x0F12, E_MANDATORY_PREFIX_F3, dst, src);}
	void movsldup(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_MOVSLDUP,	0x0F12, E_MANDATORY_PREFIX_F3, dst, src);}
	void mwait()											{AppendInstr(I_MWAIT,	0x0F01C9, 0);}

	// SSSE3
	void pabsb(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PABSB,	0x0F381C, 0, dst, src);}
	void pabsb(const MmxReg& dst, const Mem64& src)			{AppendInstr(I_PABSB,	0x0F381C, 0, dst, src);}
	void pabsb(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PABSB,	0x0F381C, E_MANDATORY_PREFIX_66, dst, src);}
	void pabsb(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PABSB,	0x0F381C, E_MANDATORY_PREFIX_66, dst, src);}
	void pabsw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PABSW,	0x0F381D, 0, dst, src);}
	void pabsw(const MmxReg& dst, const Mem64& src)			{AppendInstr(I_PABSW,	0x0F381D, 0, dst, src);}
	void pabsw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PABSW,	0x0F381D, E_MANDATORY_PREFIX_66, dst, src);}
	void pabsw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PABSW,	0x0F381D, E_MANDATORY_PREFIX_66, dst, src);}
	void pabsd(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PABSD,	0x0F381E, 0, dst, src);}
	void pabsd(const MmxReg& dst, const Mem64& src)			{AppendInstr(I_PABSD,	0x0F381E, 0, dst, src);}
	void pabsd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PABSD,	0x0F381E, E_MANDATORY_PREFIX_66, dst, src);}
	void pabsd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PABSD,	0x0F381E, E_MANDATORY_PREFIX_66, dst, src);}
	void palignr(const MmxReg& dst, const MmxReg& src, const Imm8& n)	{AppendInstr(I_PALIGNR, 0x0F3A0F, 0, dst, src, n);}
	void palignr(const MmxReg& dst, const Mem64& src, const Imm8& n)	{AppendInstr(I_PALIGNR, 0x0F3A0F, 0, dst, src, n);}
	void palignr(const XmmReg& dst, const XmmReg& src, const Imm8& n)	{AppendInstr(I_PALIGNR, 0x0F3A0F, E_MANDATORY_PREFIX_66, dst, src, n);}
	void palignr(const XmmReg& dst, const Mem128& src, const Imm8& n)	{AppendInstr(I_PALIGNR, 0x0F3A0F, E_MANDATORY_PREFIX_66, dst, src, n);}
	void phaddw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PHADDW,	0x0F3801, 0, dst, src);}
	void phaddw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PHADDW,	0x0F3801, 0, dst, src);}
	void phaddw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PHADDW,	0x0F3801, E_MANDATORY_PREFIX_66, dst, src);}
	void phaddw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PHADDW,	0x0F3801, E_MANDATORY_PREFIX_66, dst, src);}
	void phaddd(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PHADDD,	0x0F3802, 0, dst, src);}
	void phaddd(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PHADDD,	0x0F3802, 0, dst, src);}
	void phaddd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PHADDD,	0x0F3802, E_MANDATORY_PREFIX_66, dst, src);}
	void phaddd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PHADDD,	0x0F3802, E_MANDATORY_PREFIX_66, dst, src);}
	void phaddsw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PHADDSW,	0x0F3803, 0, dst, src);}
	void phaddsw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PHADDSW,	0x0F3803, 0, dst, src);}
	void phaddsw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PHADDSW,	0x0F3803, E_MANDATORY_PREFIX_66, dst, src);}
	void phaddsw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PHADDSW,	0x0F3803, E_MANDATORY_PREFIX_66, dst, src);}
	void phsubw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PHSUBW,	0x0F3805, 0, dst, src);}
	void phsubw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PHSUBW,	0x0F3805, 0, dst, src);}
	void phsubw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PHSUBW,	0x0F3805, E_MANDATORY_PREFIX_66, dst, src);}
	void phsubw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PHSUBW,	0x0F3805, E_MANDATORY_PREFIX_66, dst, src);}
	void phsubd(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PHSUBD,	0x0F3806, 0, dst, src);}
	void phsubd(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PHSUBD,	0x0F3806, 0, dst, src);}
	void phsubd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PHSUBD,	0x0F3806, E_MANDATORY_PREFIX_66, dst, src);}
	void phsubd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PHSUBD,	0x0F3806, E_MANDATORY_PREFIX_66, dst, src);}
	void phsubsw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PHSUBSW,	0x0F3807, 0, dst, src);}
	void phsubsw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PHSUBSW,	0x0F3807, 0, dst, src);}
	void phsubsw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PHSUBSW,	0x0F3807, E_MANDATORY_PREFIX_66, dst, src);}
	void phsubsw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PHSUBSW,	0x0F3807, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaddubsw(const MmxReg& dst, const MmxReg& src)	{AppendInstr(I_PMADDUBSW,0x0F3804, 0, dst, src);}
	void pmaddubsw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PMADDUBSW,0x0F3804, 0, dst, src);}
	void pmaddubsw(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_PMADDUBSW,0x0F3804, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaddubsw(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_PMADDUBSW,0x0F3804, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulhrsw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PMULHRSW,	0x0F380B, 0, dst, src);}
	void pmulhrsw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PMULHRSW,	0x0F380B, 0, dst, src);}
	void pmulhrsw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PMULHRSW,	0x0F380B, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulhrsw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PMULHRSW,	0x0F380B, E_MANDATORY_PREFIX_66, dst, src);}
	void pshufb(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PSHUFB,	0x0F3800, 0, dst, src);}
	void pshufb(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSHUFB,	0x0F3800, 0, dst, src);}
	void pshufb(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PSHUFB,	0x0F3800, E_MANDATORY_PREFIX_66, dst, src);}
	void pshufb(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PSHUFB,	0x0F3800, E_MANDATORY_PREFIX_66, dst, src);}
	void psignb(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PSIGNB,	0x0F3808, 0, dst, src);}
	void psignb(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSIGNB,	0x0F3808, 0, dst, src);}
	void psignb(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PSIGNB,	0x0F3808, E_MANDATORY_PREFIX_66, dst, src);}
	void psignb(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PSIGNB,	0x0F3808, E_MANDATORY_PREFIX_66, dst, src);}
	void psignw(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PSIGNW,	0x0F3809, 0, dst, src);}
	void psignw(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSIGNW,	0x0F3809, 0, dst, src);}
	void psignw(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PSIGNW,	0x0F3809, E_MANDATORY_PREFIX_66, dst, src);}
	void psignw(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PSIGNW,	0x0F3809, E_MANDATORY_PREFIX_66, dst, src);}
	void psignd(const MmxReg& dst, const MmxReg& src)		{AppendInstr(I_PSIGND,	0x0F380A, 0, dst, src);}
	void psignd(const MmxReg& dst, const Mem64& src)		{AppendInstr(I_PSIGND,	0x0F380A, 0, dst, src);}
	void psignd(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_PSIGND,	0x0F380A, E_MANDATORY_PREFIX_66, dst, src);}
	void psignd(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_PSIGND,	0x0F380A, E_MANDATORY_PREFIX_66, dst, src);}

	// SSE4.1
	void blendps(const XmmReg& dst, const XmmReg& src, const Imm8& mask)	{AppendInstr(I_BLENDPS,	0x0F3A0C, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void blendps(const XmmReg& dst, const Mem128& src, const Imm8& mask)	{AppendInstr(I_BLENDPS,	0x0F3A0C, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void blendpd(const XmmReg& dst, const XmmReg& src, const Imm8& mask)	{AppendInstr(I_BLENDPD,	0x0F3A0D, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void blendpd(const XmmReg& dst, const Mem128& src, const Imm8& mask)	{AppendInstr(I_BLENDPD,	0x0F3A0D, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void blendvps(const XmmReg& dst, const XmmReg& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPS,	0x0F3814, E_MANDATORY_PREFIX_66, dst, src);}
	void blendvps(const XmmReg& dst, const Mem128& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPS,	0x0F3814, E_MANDATORY_PREFIX_66, dst, src);}
	void blendvpd(const XmmReg& dst, const XmmReg& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPD,	0x0F3815, E_MANDATORY_PREFIX_66, dst, src);}
	void blendvpd(const XmmReg& dst, const Mem128& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPD,	0x0F3815, E_MANDATORY_PREFIX_66, dst, src);}
	void dpps(const XmmReg& dst, const XmmReg& src, const Imm8& mask)		{AppendInstr(I_DPPS,		0x0F3A40, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void dpps(const XmmReg& dst, const Mem128& src, const Imm8& mask)		{AppendInstr(I_DPPS,		0x0F3A40, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void dppd(const XmmReg& dst, const XmmReg& src, const Imm8& mask)		{AppendInstr(I_DPPD,		0x0F3A41, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void dppd(const XmmReg& dst, const Mem128& src, const Imm8& mask)		{AppendInstr(I_DPPD,		0x0F3A41, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void extractps(const Reg32& dst, const XmmReg& src, const Imm8& i)		{AppendInstr(I_EXTRACTPS,0x0F3A17, E_MANDATORY_PREFIX_66, src, dst, i);}
	void extractps(const Mem32& dst, const XmmReg& src, const Imm8& i)		{AppendInstr(I_EXTRACTPS,0x0F3A17, E_MANDATORY_PREFIX_66, src, dst, i);}
#ifdef JITASM64
	void extractps(const Reg64& dst, const XmmReg& src, const Imm8& i)		{AppendInstr(I_EXTRACTPS,0x0F3A17, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, src, dst, i);}
#endif
	void insertps(const XmmReg& dst, const XmmReg& src, const Imm8& i)		{AppendInstr(I_INSERTPS,	0x0F3A21, E_MANDATORY_PREFIX_66, dst, src, i);}
	void insertps(const XmmReg& dst, const Mem32& src, const Imm8& i)		{AppendInstr(I_INSERTPS,	0x0F3A21, E_MANDATORY_PREFIX_66, dst, src, i);}
	void movntdqa(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_MOVNTDQA,	0x0F382A, E_MANDATORY_PREFIX_66, dst, src);}
	void mpsadbw(const XmmReg& dst, const XmmReg& src, const Imm8& offsets)	{AppendInstr(I_MPSADBW,	0x0F3A42, E_MANDATORY_PREFIX_66, dst, src, offsets);}
	void mpsadbw(const XmmReg& dst, const Mem128& src, const Imm8& offsets)	{AppendInstr(I_MPSADBW,	0x0F3A42, E_MANDATORY_PREFIX_66, dst, src, offsets);}
	void packusdw(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PACKUSDW, 0x0F382B, E_MANDATORY_PREFIX_66, dst, src);}
	void packusdw(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PACKUSDW, 0x0F382B, E_MANDATORY_PREFIX_66, dst, src);}
	void pblendvb(const XmmReg& dst, const XmmReg& src, const XmmReg_xmm0& mask)	{AppendInstr(I_PBLENDVB, 0x0F3810, E_MANDATORY_PREFIX_66, dst, src);}
	void pblendvb(const XmmReg& dst, const Mem128& src, const XmmReg_xmm0& mask)	{AppendInstr(I_PBLENDVB, 0x0F3810, E_MANDATORY_PREFIX_66, dst, src);}
	void pblendw(const XmmReg& dst, const XmmReg& src, const Imm8& mask)	{AppendInstr(I_PBLENDW,	0x0F3A0E, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void pblendw(const XmmReg& dst, const Mem128& src, const Imm8& mask)	{AppendInstr(I_PBLENDW,	0x0F3A0E, E_MANDATORY_PREFIX_66, dst, src, mask);}
	void pcmpeqq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PCMPEQQ,	0x0F3829, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpeqq(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PCMPEQQ,	0x0F3829, E_MANDATORY_PREFIX_66, dst, src);}
	void pextrb(const Reg32& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRB,	0x0F3A14, E_MANDATORY_PREFIX_66, src, dst, i);}
	void pextrb(const Mem8& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRB,	0x0F3A14, E_MANDATORY_PREFIX_66, src, dst, i);}
	void pextrw(const Mem16& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRW,	0x0F3A15, E_MANDATORY_PREFIX_66, src, dst, i);}
	void pextrd(const Reg32& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRD,	0x0F3A16, E_MANDATORY_PREFIX_66, src, dst, i);}
	void pextrd(const Mem32& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRD,	0x0F3A16, E_MANDATORY_PREFIX_66, src, dst, i);}
#ifdef JITASM64
	void pextrb(const Reg64& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRB,	0x0F3A14, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, src, dst, i);}
	void pextrq(const Reg64& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRQ,	0x0F3A16, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, src, dst, i);}
	void pextrq(const Mem64& dst, const XmmReg& src, const Imm8& i)			{AppendInstr(I_PEXTRQ,	0x0F3A16, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, src, dst, i);}
#endif
	void pinsrb(const XmmReg& dst, const Reg32& src, const Imm8& i)			{AppendInstr(I_PINSRB, 0x0F3A20, E_MANDATORY_PREFIX_66, dst, src, i);}
	void pinsrb(const XmmReg& dst, const Mem8& src, const Imm8& i)			{AppendInstr(I_PINSRB, 0x0F3A20, E_MANDATORY_PREFIX_66, dst, src, i);}
	void pinsrd(const XmmReg& dst, const Reg32& src, const Imm8& i)			{AppendInstr(I_PINSRD, 0x0F3A22, E_MANDATORY_PREFIX_66, dst, src, i);}
	void pinsrd(const XmmReg& dst, const Mem32& src, const Imm8& i)			{AppendInstr(I_PINSRD, 0x0F3A22, E_MANDATORY_PREFIX_66, dst, src, i);}
#ifdef JITASM64
	void pinsrb(const XmmReg& dst, const Reg64& src, const Imm8& i)			{AppendInstr(I_PINSRB, 0x0F3A20, E_MANDATORY_PREFIX_66, dst, src, i);}
	void pinsrq(const XmmReg& dst, const Reg64& src, const Imm8& i)			{AppendInstr(I_PINSRQ, 0x0F3A22, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src, i);}
	void pinsrq(const XmmReg& dst, const Mem64& src, const Imm8& i)			{AppendInstr(I_PINSRQ, 0x0F3A22, E_MANDATORY_PREFIX_66 | E_REXW_PREFIX, dst, src, i);}
#endif
	void pmaxsb(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMAXSB, 0x0F383C, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxsb(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMAXSB, 0x0F383C, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxsd(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMAXSD, 0x0F383D, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxsd(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMAXSD, 0x0F383D, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxuw(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMAXUW, 0x0F383E, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxuw(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMAXUW, 0x0F383E, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxud(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMAXUD, 0x0F383F, E_MANDATORY_PREFIX_66, dst, src);}
	void pmaxud(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMAXUD, 0x0F383F, E_MANDATORY_PREFIX_66, dst, src);}
	void pminsb(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMINSB, 0x0F3838, E_MANDATORY_PREFIX_66, dst, src);}
	void pminsb(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMINSB, 0x0F3838, E_MANDATORY_PREFIX_66, dst, src);}
	void pminsd(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMINSD, 0x0F3839, E_MANDATORY_PREFIX_66, dst, src);}
	void pminsd(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMINSD, 0x0F3839, E_MANDATORY_PREFIX_66, dst, src);}
	void pminuw(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMINUW, 0x0F383A, E_MANDATORY_PREFIX_66, dst, src);}
	void pminuw(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMINUW, 0x0F383A, E_MANDATORY_PREFIX_66, dst, src);}
	void pminud(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMINUD, 0x0F383B, E_MANDATORY_PREFIX_66, dst, src);}
	void pminud(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMINUD, 0x0F383B, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxbw(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVSXBW, 0x0F3820, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxbw(const XmmReg& dst, const Mem64& src)						{AppendInstr(I_PMOVSXBW, 0x0F3820, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxbd(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVSXBD, 0x0F3821, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxbd(const XmmReg& dst, const Mem32& src)						{AppendInstr(I_PMOVSXBD, 0x0F3821, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxbq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVSXBQ, 0x0F3822, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxbq(const XmmReg& dst, const Mem16& src)						{AppendInstr(I_PMOVSXBQ, 0x0F3822, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxwd(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVSXWD, 0x0F3823, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxwd(const XmmReg& dst, const Mem64& src)						{AppendInstr(I_PMOVSXWD, 0x0F3823, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxwq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVSXWQ, 0x0F3824, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxwq(const XmmReg& dst, const Mem32& src)						{AppendInstr(I_PMOVSXWQ, 0x0F3824, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxdq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVSXDQ, 0x0F3825, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovsxdq(const XmmReg& dst, const Mem64& src)						{AppendInstr(I_PMOVSXDQ, 0x0F3825, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxbw(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVZXBW, 0x0F3830, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxbw(const XmmReg& dst, const Mem64& src)						{AppendInstr(I_PMOVZXBW, 0x0F3830, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxbd(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVZXBD, 0x0F3831, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxbd(const XmmReg& dst, const Mem32& src)						{AppendInstr(I_PMOVZXBD, 0x0F3831, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxbq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVZXBQ, 0x0F3832, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxbq(const XmmReg& dst, const Mem16& src)						{AppendInstr(I_PMOVZXBQ, 0x0F3832, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxwd(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVZXWD, 0x0F3833, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxwd(const XmmReg& dst, const Mem64& src)						{AppendInstr(I_PMOVZXWD, 0x0F3833, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxwq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVZXWQ, 0x0F3834, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxwq(const XmmReg& dst, const Mem32& src)						{AppendInstr(I_PMOVZXWQ, 0x0F3834, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxdq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMOVZXDQ, 0x0F3835, E_MANDATORY_PREFIX_66, dst, src);}
	void pmovzxdq(const XmmReg& dst, const Mem64& src)						{AppendInstr(I_PMOVZXDQ, 0x0F3835, E_MANDATORY_PREFIX_66, dst, src);}
	void pmuldq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMULDQ,	0x0F3828, E_MANDATORY_PREFIX_66, dst, src);}
	void pmuldq(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMULDQ,	0x0F3828, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulld(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PMULLD,	0x0F3840, E_MANDATORY_PREFIX_66, dst, src);}
	void pmulld(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PMULLD,	0x0F3840, E_MANDATORY_PREFIX_66, dst, src);}
	void ptest(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PTEST,	0x0F3817, E_MANDATORY_PREFIX_66, dst, src);}
	void ptest(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PTEST,	0x0F3817, E_MANDATORY_PREFIX_66, dst, src);}
	void roundps(const XmmReg& dst, const XmmReg& src, const Imm8& mode)	{AppendInstr(I_ROUNDPS,	0x0F3A08, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundps(const XmmReg& dst, const Mem128& src, const Imm8& mode)	{AppendInstr(I_ROUNDPS,	0x0F3A08, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundpd(const XmmReg& dst, const XmmReg& src, const Imm8& mode)	{AppendInstr(I_ROUNDPD,	0x0F3A09, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundpd(const XmmReg& dst, const Mem128& src, const Imm8& mode)	{AppendInstr(I_ROUNDPD,	0x0F3A09, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundss(const XmmReg& dst, const XmmReg& src, const Imm8& mode)	{AppendInstr(I_ROUNDSS,	0x0F3A0A, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundss(const XmmReg& dst, const Mem32& src, const Imm8& mode)		{AppendInstr(I_ROUNDSS,	0x0F3A0A, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundsd(const XmmReg& dst, const XmmReg& src, const Imm8& mode)	{AppendInstr(I_ROUNDSD,	0x0F3A0B, E_MANDATORY_PREFIX_66, dst, src, mode);}
	void roundsd(const XmmReg& dst, const Mem64& src, const Imm8& mode)		{AppendInstr(I_ROUNDSD,	0x0F3A0B, E_MANDATORY_PREFIX_66, dst, src, mode);}

	// SSE4.2
	void crc32(const Reg32& dst, const Reg8& src)							{AppendInstr(I_CRC32,		0x0F38F0, E_MANDATORY_PREFIX_F2, dst, src);}
	void crc32(const Reg32& dst, const Mem8& src)							{AppendInstr(I_CRC32,		0x0F38F0, E_MANDATORY_PREFIX_F2, dst, src);}
	void crc32(const Reg32& dst, const Reg16& src)							{AppendInstr(I_CRC32,		0x0F38F1, E_MANDATORY_PREFIX_F2 | E_OPERAND_SIZE_PREFIX, dst, src);}
	void crc32(const Reg32& dst, const Mem16& src)							{AppendInstr(I_CRC32,		0x0F38F1, E_MANDATORY_PREFIX_F2 | E_OPERAND_SIZE_PREFIX, dst, src);}
	void crc32(const Reg32& dst, const Reg32& src)							{AppendInstr(I_CRC32,		0x0F38F1, E_MANDATORY_PREFIX_F2, dst, src);}
	void crc32(const Reg32& dst, const Mem32& src)							{AppendInstr(I_CRC32,		0x0F38F1, E_MANDATORY_PREFIX_F2, dst, src);}
#ifdef JITASM64
	void crc32(const Reg64& dst, const Reg8& src)							{AppendInstr(I_CRC32,		0x0F38F0, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
	void crc32(const Reg64& dst, const Mem8& src)							{AppendInstr(I_CRC32,		0x0F38F0, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
	void crc32(const Reg64& dst, const Reg64& src)							{AppendInstr(I_CRC32,		0x0F38F1, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
	void crc32(const Reg64& dst, const Mem64& src)							{AppendInstr(I_CRC32,		0x0F38F1, E_MANDATORY_PREFIX_F2 | E_REXW_PREFIX, dst, src);}
#endif
	void pcmpestri(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPESTRI,	0x0F3A61, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpestrm(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPESTRM,	0x0F3A60, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpistri(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPISTRI,	0x0F3A63, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpistrm(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPISTRM,	0x0F3A62, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpgtq(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_PCMPGTQ,		0x0F3837, E_MANDATORY_PREFIX_66, dst, src);}
	void pcmpgtq(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_PCMPGTQ,		0x0F3837, E_MANDATORY_PREFIX_66, dst, src);}
	void popcnt(const Reg16& dst, const Reg16& src)							{AppendInstr(I_POPCNT,		0x0FB8, E_MANDATORY_PREFIX_F3 | E_OPERAND_SIZE_PREFIX, dst, src);}
	void popcnt(const Reg16& dst, const Mem16& src)							{AppendInstr(I_POPCNT,		0x0FB8, E_MANDATORY_PREFIX_F3 | E_OPERAND_SIZE_PREFIX, dst, src);}
	void popcnt(const Reg32& dst, const Reg32& src)							{AppendInstr(I_POPCNT,		0x0FB8, E_MANDATORY_PREFIX_F3, dst, src);}
	void popcnt(const Reg32& dst, const Mem32& src)							{AppendInstr(I_POPCNT,		0x0FB8, E_MANDATORY_PREFIX_F3, dst, src);}
#ifdef JITASM64
	void popcnt(const Reg64& dst, const Reg64& src)							{AppendInstr(I_POPCNT, 0x0FB8, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
	void popcnt(const Reg64& dst, const Mem64& src)							{AppendInstr(I_POPCNT, 0x0FB8, E_MANDATORY_PREFIX_F3 | E_REXW_PREFIX, dst, src);}
#endif

	struct ControlState
	{
		size_t beg;
		size_t end;
	};
	ControlState ctrl_state_;
	std::deque<ControlState> ctrl_state_stack_;

	ControlState NewControlState()
	{
		ControlState state = {NewLabelID(""), NewLabelID(""),};
		return state;
	}

	void Repeat()
	{
		ctrl_state_stack_.push_back(ctrl_state_);
		ctrl_state_ = NewControlState();

		L(ctrl_state_.beg);
	}
	template<class Ty>
	void Until(const Ty& expr)
	{
		expr(*this, ctrl_state_.beg, ctrl_state_.end);
		L(ctrl_state_.end);

		ctrl_state_ = *ctrl_state_stack_.rbegin();
		ctrl_state_stack_.pop_back();
	}

	template<class Ty>
	void While(const Ty& expr)
	{
		ctrl_state_stack_.push_back(ctrl_state_);
		ctrl_state_ = NewControlState();
		size_t label = NewLabelID("");

		L(ctrl_state_.beg);
		expr(*this, label, ctrl_state_.end);
		L(label);
	}
	void EndW()
	{
		AppendJmp(ctrl_state_.beg);
		L(ctrl_state_.end);

		ctrl_state_ = *ctrl_state_stack_.rbegin();
		ctrl_state_stack_.pop_back();
	}

	template<class Ty>
	void If(const Ty& expr)
	{
		ctrl_state_stack_.push_back(ctrl_state_);
		ctrl_state_ = NewControlState();

		expr(*this, ctrl_state_.beg, ctrl_state_.end);
		L(ctrl_state_.beg);
	}
	void EndIf()
	{
		L(ctrl_state_.end);
		ctrl_state_ = *ctrl_state_stack_.rbegin();
		ctrl_state_stack_.pop_back();
	}
};

namespace detail
{
	struct CondExpr {
		virtual void operator()(Frontend& f, size_t beg, size_t end) const = 0;
	};

	struct CondExpr_ExprAnd : CondExpr {
		const CondExpr&	lhs_;
		const CondExpr&	rhs_;
		CondExpr_ExprAnd(const CondExpr& lhs, const CondExpr& rhs) : lhs_(lhs), rhs_(rhs) {}
		void operator()(Frontend& f, size_t beg, size_t end) const {
			size_t label = f.NewLabelID("");
			lhs_(f, label, end);
			f.L(label);
			rhs_(f, beg, end);
		}
	};

	struct CondExpr_ExprOr : CondExpr {
		const CondExpr&	lhs_;
		const CondExpr&	rhs_;
		CondExpr_ExprOr(const CondExpr& lhs, const CondExpr& rhs) : lhs_(lhs), rhs_(rhs) {}
		void operator()(Frontend& f, size_t beg, size_t end) const {
			size_t label = f.NewLabelID("");
			lhs_(f, beg, label);
			f.L(label);
			rhs_(f, beg, end);
		}
	};

	template<class L, class R, JumpCondition Jcc>
	struct CondExpr_Cmp : CondExpr {
		L	lhs_;
		R	rhs_;
		CondExpr_Cmp(const L& lhs, const R& rhs) : lhs_(lhs), rhs_(rhs) {}
		void operator()(Frontend& f, size_t beg, size_t end) const {
			f.cmp(lhs_, rhs_);
			f.AppendJcc(Jcc, beg);
			f.AppendJmp(end);
		}
	};

	template<class L, class R, JumpCondition Jcc>
	struct CondExpr_Or : CondExpr {
		L	lhs_;
		R	rhs_;
		CondExpr_Or(const L& lhs, const R& rhs) : lhs_(lhs), rhs_(rhs) {}
		void operator()(Frontend& f, size_t beg, size_t end) const {
			f.or(lhs_, rhs_);
			f.AppendJcc(Jcc, beg);
			f.AppendJmp(end);
		}
	};
}

// &&
inline detail::CondExpr_ExprAnd	operator&&(const detail::CondExpr& lhs, const detail::CondExpr& rhs)	{return detail::CondExpr_ExprAnd(lhs, rhs);}
// ||
inline detail::CondExpr_ExprOr	operator||(const detail::CondExpr& lhs, const detail::CondExpr& rhs)	{return detail::CondExpr_ExprOr(lhs, rhs);}

// !
inline detail::CondExpr_Or<Reg8, Reg8, JCC_E>		operator!(const Reg8& lhs)		{return detail::CondExpr_Or<Reg8, Reg8, JCC_E>(lhs, lhs);}
inline detail::CondExpr_Or<Reg16, Reg16, JCC_E>		operator!(const Reg16& lhs)		{return detail::CondExpr_Or<Reg16, Reg16, JCC_E>(lhs, lhs);}
inline detail::CondExpr_Or<Reg32, Reg32, JCC_E>		operator!(const Reg32& lhs)		{return detail::CondExpr_Or<Reg32, Reg32, JCC_E>(lhs, lhs);}
#ifdef JITASM64
inline detail::CondExpr_Or<Reg64, Reg64, JCC_E>		operator!(const Reg64& lhs)		{return detail::CondExpr_Or<Reg64, Reg64, JCC_E>(lhs, lhs);}
#endif
inline detail::CondExpr_Cmp<Mem8, Imm8, JCC_E>		operator!(const Mem8& lhs)		{return detail::CondExpr_Cmp<Mem8, Imm8, JCC_E>(lhs, 0);}
inline detail::CondExpr_Cmp<Mem16, Imm16, JCC_E>	operator!(const Mem16& lhs)		{return detail::CondExpr_Cmp<Mem16, Imm16, JCC_E>(lhs, 0);}
inline detail::CondExpr_Cmp<Mem32, Imm32, JCC_E>	operator!(const Mem32& lhs)		{return detail::CondExpr_Cmp<Mem32, Imm32, JCC_E>(lhs, 0);}
#ifdef JITASM64
inline detail::CondExpr_Cmp<Mem64, Imm32, JCC_E>	operator!(const Mem64& lhs)		{return detail::CondExpr_Cmp<Mem64, Imm32, JCC_E>(lhs, 0);}
#endif

// <
template<class R> detail::CondExpr_Cmp<Reg8, R, JCC_B>		operator<(const Reg8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg8, R, JCC_B>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg16, R, JCC_B>		operator<(const Reg16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg16, R, JCC_B>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg32, R, JCC_B>		operator<(const Reg32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg32, R, JCC_B>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Reg64, R, JCC_B>		operator<(const Reg64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg64, R, JCC_B>(lhs, rhs);}
#endif
template<class R> detail::CondExpr_Cmp<Mem8, R, JCC_B>		operator<(const Mem8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem8, R, JCC_B>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem16, R, JCC_B>		operator<(const Mem16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem16, R, JCC_B>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem32, R, JCC_B>		operator<(const Mem32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem32, R, JCC_B>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Mem64, R, JCC_B>		operator<(const Mem64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem64, R, JCC_B>(lhs, rhs);}
#endif

// >
template<class R> detail::CondExpr_Cmp<Reg8, R, JCC_A>		operator>(const Reg8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg8, R, JCC_A>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg16, R, JCC_A>		operator>(const Reg16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg16, R, JCC_A>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg32, R, JCC_A>		operator>(const Reg32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg32, R, JCC_A>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Reg64, R, JCC_A>		operator>(const Reg64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg64, R, JCC_A>(lhs, rhs);}
#endif
template<class R> detail::CondExpr_Cmp<Mem8, R, JCC_A>		operator>(const Mem8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem8, R, JCC_A>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem16, R, JCC_A>		operator>(const Mem16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem16, R, JCC_A>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem32, R, JCC_A>		operator>(const Mem32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem32, R, JCC_A>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Mem64, R, JCC_A>		operator>(const Mem64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem64, R, JCC_A>(lhs, rhs);}
#endif

// <=
template<class R> detail::CondExpr_Cmp<Reg8, R, JCC_BE>		operator<=(const Reg8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg8, R, JCC_BE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg16, R, JCC_BE>	operator<=(const Reg16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg16, R, JCC_BE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg32, R, JCC_BE>	operator<=(const Reg32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg32, R, JCC_BE>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Reg64, R, JCC_BE>	operator<=(const Reg64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg64, R, JCC_BE>(lhs, rhs);}
#endif
template<class R> detail::CondExpr_Cmp<Mem8, R, JCC_BE>		operator<=(const Mem8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem8, R, JCC_BE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem16, R, JCC_BE>	operator<=(const Mem16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem16, R, JCC_BE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem32, R, JCC_BE>	operator<=(const Mem32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem32, R, JCC_BE>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Mem64, R, JCC_BE>	operator<=(const Mem64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem64, R, JCC_BE>(lhs, rhs);}
#endif

// >=
template<class R> detail::CondExpr_Cmp<Reg8, R, JCC_AE>		operator>=(const Reg8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg8, R, JCC_AE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg16, R, JCC_AE>	operator>=(const Reg16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg16, R, JCC_AE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg32, R, JCC_AE>	operator>=(const Reg32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg32, R, JCC_AE>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Reg64, R, JCC_AE>	operator>=(const Reg64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg64, R, JCC_AE>(lhs, rhs);}
#endif
template<class R> detail::CondExpr_Cmp<Mem8, R, JCC_AE>		operator>=(const Mem8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem8, R, JCC_AE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem16, R, JCC_AE>	operator>=(const Mem16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem16, R, JCC_AE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem32, R, JCC_AE>	operator>=(const Mem32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem32, R, JCC_AE>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Mem64, R, JCC_AE>	operator>=(const Mem64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem64, R, JCC_AE>(lhs, rhs);}
#endif

// ==
template<class R> detail::CondExpr_Cmp<Reg8, R, JCC_E>		operator==(const Reg8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg8, R, JCC_E>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg16, R, JCC_E>		operator==(const Reg16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg16, R, JCC_E>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg32, R, JCC_E>		operator==(const Reg32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg32, R, JCC_E>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Reg64, R, JCC_E>		operator==(const Reg64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg64, R, JCC_E>(lhs, rhs);}
#endif
template<class R> detail::CondExpr_Cmp<Mem8, R, JCC_E>		operator==(const Mem8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem8, R, JCC_E>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem16, R, JCC_E>		operator==(const Mem16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem16, R, JCC_E>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem32, R, JCC_E>		operator==(const Mem32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem32, R, JCC_E>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Mem64, R, JCC_E>		operator==(const Mem64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem64, R, JCC_E>(lhs, rhs);}
#endif

// !=
template<class R> detail::CondExpr_Cmp<Reg8, R, JCC_NE>		operator!=(const Reg8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg8, R, JCC_NE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg16, R, JCC_NE>	operator!=(const Reg16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg16, R, JCC_NE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Reg32, R, JCC_NE>	operator!=(const Reg32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg32, R, JCC_NE>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Reg64, R, JCC_NE>	operator!=(const Reg64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Reg64, R, JCC_NE>(lhs, rhs);}
#endif
template<class R> detail::CondExpr_Cmp<Mem8, R, JCC_NE>		operator!=(const Mem8& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem8, R, JCC_NE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem16, R, JCC_NE>	operator!=(const Mem16& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem16, R, JCC_NE>(lhs, rhs);}
template<class R> detail::CondExpr_Cmp<Mem32, R, JCC_NE>	operator!=(const Mem32& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem32, R, JCC_NE>(lhs, rhs);}
#ifdef JITASM64
template<class R> detail::CondExpr_Cmp<Mem64, R, JCC_NE>	operator!=(const Mem64& lhs, const R& rhs)	{return detail::CondExpr_Cmp<Mem64, R, JCC_NE>(lhs, rhs);}
#endif

namespace detail {
	// Flags for ArgumentTraits
	enum {
		ARG_IN_REG		= (1<<0),	///< Argument is stored in general purpose register.
		ARG_IN_STACK	= (1<<1),	///< Argument is stored on stack.
		ARG_IN_XMM_SP	= (1<<2),	///< Argument is stored in xmm register as single precision.
		ARG_IN_XMM_DP	= (1<<3),	///< Argument is stored in xmm register as double precision.
		ARG_TYPE_VALUE	= (1<<4),	///< Argument is value which is passed.
		ARG_TYPE_PTR	= (1<<5)	///< Argument is pointer which is passed to.
	};

	/// cdecl argument type traits
	template<int N, class T, int Size>
	struct ArgumentTraits_cdecl {
		enum {
			stack_size = (Size + 4 - 1) / 4 * 4,
			flag = ARG_IN_STACK | ARG_TYPE_VALUE,
			reg_id = INVALID
		};
	};

	/// Microsoft x64 fastcall argument type traits
	template<int N, class T, int Size>
	struct ArgumentTraits_win64 {
		enum {
			stack_size = 8,
			flag = ARG_IN_STACK | (Size == 1 || Size == 2 || Size == 4 || Size == 8 ? ARG_TYPE_VALUE : ARG_TYPE_PTR),
			reg_id = INVALID
		};
	};

	/**
	 * Base class for argument which is stored in general purpose register.
	 */
	template<int GpRegID, int Flag> struct ArgumentTraits_win64_reg {
		enum {
			stack_size = 8,
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

#ifdef _MMINTRIN_H_INCLUDED
	// specialization for __m64
	template<> struct ArgumentTraits_win64<0, __m64, 8> : ArgumentTraits_win64_reg<RCX, ARG_TYPE_VALUE> {};
	template<> struct ArgumentTraits_win64<1, __m64, 8> : ArgumentTraits_win64_reg<RDX, ARG_TYPE_VALUE> {};
	template<> struct ArgumentTraits_win64<2, __m64, 8> : ArgumentTraits_win64_reg<R8, ARG_TYPE_VALUE> {};
	template<> struct ArgumentTraits_win64<3, __m64, 8> : ArgumentTraits_win64_reg<R9, ARG_TYPE_VALUE> {};
#endif

	/**
	 * Base class for argument which is stored in xmm register.
	 */
	template<int XmmRegID, int Flag> struct ArgumentTraits_win64_xmm {
		enum {
			stack_size = 8,
			flag = Flag | ARG_TYPE_VALUE,
			reg_id = XmmRegID
		};
	};

	// specialization for float
	template<> struct ArgumentTraits_win64<0, float, 4> : ArgumentTraits_win64_xmm<XMM0, ARG_IN_XMM_SP> {};
	template<> struct ArgumentTraits_win64<1, float, 4> : ArgumentTraits_win64_xmm<XMM1, ARG_IN_XMM_SP> {};
	template<> struct ArgumentTraits_win64<2, float, 4> : ArgumentTraits_win64_xmm<XMM2, ARG_IN_XMM_SP> {};
	template<> struct ArgumentTraits_win64<3, float, 4> : ArgumentTraits_win64_xmm<XMM3, ARG_IN_XMM_SP> {};

	// specialization for double
	template<> struct ArgumentTraits_win64<0, double, 8> : ArgumentTraits_win64_xmm<XMM0, ARG_IN_XMM_DP> {};
	template<> struct ArgumentTraits_win64<1, double, 8> : ArgumentTraits_win64_xmm<XMM1, ARG_IN_XMM_DP> {};
	template<> struct ArgumentTraits_win64<2, double, 8> : ArgumentTraits_win64_xmm<XMM2, ARG_IN_XMM_DP> {};
	template<> struct ArgumentTraits_win64<3, double, 8> : ArgumentTraits_win64_xmm<XMM3, ARG_IN_XMM_DP> {};


	/// Special argument type
	struct ArgNone {};

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
				f.lea(f.zsi, static_cast<MemT<OpdR>&>(val_));
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
			if (detail::IsGpReg(val_)) {
				if (val_.GetReg() != AL)
					f.mov(f.al, static_cast<Reg8&>(val_));
			}
			else if (val_.IsMem()) {
				f.mov(f.al, static_cast<Mem8&>(val_));
			}
			else if (val_.IsImm()) {
				f.mov(f.al, static_cast<Imm8&>(val_));
			}
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
			if (detail::IsGpReg(val_)) {
				if (val_.GetReg() != AX)
					f.mov(f.ax, static_cast<Reg16&>(val_));
			}
			else if (val_.IsMem()) {
				f.mov(f.ax, static_cast<Mem16&>(val_));
			}
			else if (val_.IsImm()) {
				f.mov(f.ax, static_cast<Imm16&>(val_));
			}
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
			if (detail::IsGpReg(val_)) {
				if (val_.GetReg() != EAX)
					f.mov(f.eax, static_cast<Reg32&>(val_));
			}
			else if (val_.IsMem()) {
				f.mov(f.eax, static_cast<Mem32&>(val_));
			}
			else if (val_.IsImm()) {
				f.mov(f.eax, static_cast<Imm32&>(val_));
			}
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
			if (detail::IsGpReg(val_)) {
				if (val_.GetReg() != RAX)
					f.mov(f.rax, static_cast<Reg64&>(val_));
			}
			else if (val_.IsMem()) {
				f.mov(f.rax, static_cast<Mem64&>(val_));
			}
			else if (val_.IsImm()) {
				f.mov(f.rax, static_cast<Imm64&>(val_));
			}
			else if (detail::IsMmxReg(val_)) {
				f.movq(f.rax, static_cast<MmxReg&>(val_));
			}
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
			if (detail::IsFpuReg(val_)) {
				// from FPU register
				f.fstp(f.real4_ptr[f.rsp - 4]);
				f.movss(f.xmm0, f.dword_ptr[f.rsp - 4]);
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_32) {
				// from memory
				f.movss(f.xmm0, static_cast<Mem32&>(val_));
			}
			else if (detail::IsXmmReg(val_)) {
				// from XMM register
				if (val_.GetReg() != XMM0)
					f.movss(f.xmm0, static_cast<XmmReg&>(val_));
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.rsp - 4], static_cast<Imm32&>(val_));
				f.movss(f.xmm0, f.dword_ptr[f.rsp - 4]);
			}
#else
			if (detail::IsFpuReg(val_)) {
				// from FPU register
				if (val_.GetReg() != ST0)
					f.fld(static_cast<FpuReg&>(val_));
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_32) {
				// from memory
				f.fld(static_cast<Mem32&>(val_));
			}
			else if (detail::IsXmmReg(val_)) {
				// from XMM register
				f.movss(f.dword_ptr[f.esp - 4], static_cast<XmmReg&>(val_));
				f.fld(f.real4_ptr[f.esp - 4]);
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.esp - 4], static_cast<Imm32&>(val_));
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
			if (detail::IsFpuReg(val_)) {
				// from FPU register
				f.fstp(f.real8_ptr[f.rsp - 8]);
				f.movsd(f.xmm0, f.qword_ptr[f.rsp - 8]);
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_64) {
				// from memory
				f.movsd(f.xmm0, static_cast<Mem64&>(val_));
			}
			else if (detail::IsXmmReg(val_)) {
				// from XMM register
				if (val_.GetReg() != XMM0)
					f.movsd(f.xmm0, static_cast<XmmReg&>(val_));
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.rsp - 8], *reinterpret_cast<uint32*>(&imm_));
				f.mov(f.dword_ptr[f.rsp - 4], *(reinterpret_cast<uint32*>(&imm_) + 1));
				f.movsd(f.xmm0, f.qword_ptr[f.rsp - 8]);
			}
#else
			if (detail::IsFpuReg(val_)) {
				// from FPU register
				if (val_.GetReg() != ST0)
					f.fld(static_cast<FpuReg&>(val_));
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_64) {
				// from memory
				f.fld(static_cast<Mem64&>(val_));
			}
			else if (detail::IsXmmReg(val_)) {
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

#ifdef JITASM64

#ifdef _INCLUDED_MM2
	// specialization for __m128
	template<>
	struct ResultT<__m128, 16> {
		enum { ArgR = 1 };
		Opd128 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			f.mov(f.zax, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
			if (detail::IsXmmReg(val_)) {
				f.movaps(f.xmmword_ptr[f.zax], static_cast<const XmmReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movaps(f.xmm0, static_cast<const Mem128&>(val_));
				f.movaps(f.xmmword_ptr[f.zax], f.xmm0);
			}
		}
	};
#endif	// _INCLUDED_MM2

#ifdef _INCLUDED_EMM
	// specialization for __m128d
	template<>
	struct ResultT<__m128d, 16> {
		enum { ArgR = 1 };
		Opd128 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			f.mov(f.zax, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
			if (detail::IsXmmReg(val_)) {
				f.movapd(f.xmmword_ptr[f.zax], static_cast<const XmmReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movapd(f.xmm0, static_cast<const Mem128&>(val_));
				f.movapd(f.xmmword_ptr[f.zax], f.xmm0);
			}
		}
	};

	// specialization for __m128i
	template<>
	struct ResultT<__m128i, 16> {
		enum { ArgR = 1 };
		Opd128 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			f.mov(f.zax, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
			if (detail::IsXmmReg(val_)) {
				f.movdqa(f.xmmword_ptr[f.zax], static_cast<const XmmReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movdqa(f.xmm0, static_cast<const Mem128&>(val_));
				f.movdqa(f.xmmword_ptr[f.zax], f.xmm0);
			}
		}
	};
#endif	// _INCLUDED_EMM

#else	// JITASM64

#ifdef _MMINTRIN_H_INCLUDED
	// specialization for __m64
	template<>
	struct ResultT<__m64, 8> {
		enum { ArgR = 0 };
		Opd64 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const MmxReg& mm) : val_(mm) {}
		ResultT(const Mem64& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (detail::IsMmxReg(val_)) {
				if (val_.GetReg() != MM0)
					f.movq(f.mm0, static_cast<const MmxReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movq(f.mm0, static_cast<const Mem64&>(val_));
			}
		}
	};
#endif	// _MMINTRIN_H_INCLUDED

#ifdef _INCLUDED_MM2
	// specialization for __m128
	template<>
	struct ResultT<__m128, 16> {
		enum { ArgR = 0 };
		Opd128 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (detail::IsXmmReg(val_)) {
				if (val_.GetReg() != XMM0)
					f.movaps(f.xmm0, static_cast<const XmmReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movaps(f.xmm0, static_cast<const Mem128&>(val_));
			}
		}
	};
#endif	// _INCLUDED_MM2

#ifdef _INCLUDED_EMM
	// specialization for __m128d
	template<>
	struct ResultT<__m128d, 16> {
		enum { ArgR = 0 };
		Opd128 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (detail::IsXmmReg(val_)) {
				if (val_.GetReg() != XMM0)
					f.movapd(f.xmm0, static_cast<const XmmReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movapd(f.xmm0, static_cast<const Mem128&>(val_));
			}
		}
	};

	// specialization for __m128i
	template<>
	struct ResultT<__m128i, 16> {
		enum { ArgR = 0 };
		Opd128 val_;
		ResultT() : val_(INVALID) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (detail::IsXmmReg(val_)) {
				if (val_.GetReg() != XMM0)
					f.movdqa(f.xmm0, static_cast<const XmmReg&>(val_));
			}
			else if (val_.IsMem()) {
				f.movdqa(f.xmm0, static_cast<const Mem128&>(val_));
			}
		}
	};
#endif	// _INCLUDED_EMM

#endif	// JITASM64


	/// cdecl function base class
	class Function_cdecl : public Frontend
	{
	public:
#ifdef JITASM64
		typedef Reg64Expr Arg;		///< main function argument type
		template<int N, class T, int Size = sizeof(T)> struct ArgTraits : ArgumentTraits_win64<N, T, Size> {};
		bool dump_regarg_x64_;
#else
		typedef Reg32Expr Arg;		///< main function argument type
		template<int N, class T, int Size = sizeof(T)> struct ArgTraits : ArgumentTraits_cdecl<N, T, Size> {};
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
#ifdef JITASM64
		template<int N, class T>
		void CopyRegArgToStack(const Arg& addr, bool bForceCopy)
		{
			if (dump_regarg_x64_ || bForceCopy) {
				if (ArgTraits<N, T>::flag & ARG_IN_REG)
					mov(qword_ptr[addr], Reg64(static_cast<RegID>(ArgTraits<N, T>::reg_id)));
				else if (ArgTraits<N, T>::flag & ARG_IN_XMM_SP)
					movss(dword_ptr[addr], XmmReg(static_cast<RegID>(ArgTraits<N, T>::reg_id)));
				else if (ArgTraits<N, T>::flag & ARG_IN_XMM_DP)
					movsd(qword_ptr[addr], XmmReg(static_cast<RegID>(ArgTraits<N, T>::reg_id)));
			}
		}
#else
		template<int N, class T>
		void CopyRegArgToStack(const Arg&, bool) {}
#endif

	public:
		template<class R>
		Arg DumpRegArg0()
		{
			Arg addr(zbp + sizeof(void *) * 2);
			if (ResultT<R>::ArgR) {
				// Copy result address to stack irrespectively of dump_regarg_x64_ for result.
				CopyRegArgToStack<0, R>(addr, true);
				addr = addr + ArgTraits<0, R>::stack_size;
			}
			return addr;
		}

		template<>
		Arg DumpRegArg0<void>()
		{
			return Arg(zbp + sizeof(void *) * 2);
		}

		template<class R, class A1>
		Arg DumpRegArg1()
		{
			Arg addr = DumpRegArg0<R>();
			CopyRegArgToStack<0 + ResultT<R>::ArgR, A1>(addr, false);
			return addr + ArgTraits<0 + ResultT<R>::ArgR, A1>::stack_size;
		}

		template<class R, class A1, class A2>
		Arg DumpRegArg2()
		{
			Arg addr = DumpRegArg1<R, A1>();
			CopyRegArgToStack<1 + ResultT<R>::ArgR, A2>(addr, false);
			return addr + ArgTraits<1 + ResultT<R>::ArgR, A2>::stack_size;
		}

		template<class R, class A1, class A2, class A3>
		Arg DumpRegArg3()
		{
			Arg addr = DumpRegArg2<R, A1, A2>();
			CopyRegArgToStack<2 + ResultT<R>::ArgR, A3>(addr, false);
			return addr + ArgTraits<2 + ResultT<R>::ArgR, A3>::stack_size;
		}

		template<class R, class A1, class A2, class A3, class A4>
		Arg DumpRegArg4()
		{
			Arg addr = DumpRegArg3<R, A1, A2, A3>();
			CopyRegArgToStack<3 + ResultT<R>::ArgR, A4>(addr, false);
			return addr + ArgTraits<3 + ResultT<R>::ArgR, A4>::stack_size;
		}

		template<class R>
		Arg Arg1() { return Arg(zbp + sizeof(void *) * (2 + ResultT<R>::ArgR)); }
		template<class R, class A1>
		Arg Arg2() { return Arg1<R>() + ArgTraits<0, A1>::stack_size; }
		template<class R, class A1, class A2>
		Arg Arg3() { return Arg2<R, A1>() + ArgTraits<1, A2>::stack_size; }
		template<class R, class A1, class A2, class A3>
		Arg Arg4() { return Arg3<R, A1, A2>() + ArgTraits<2, A3>::stack_size; }
		template<class R, class A1, class A2, class A3, class A4>
		Arg Arg5() { return Arg4<R, A1, A2, A3>() + ArgTraits<3, A4>::stack_size; }
		template<class R, class A1, class A2, class A3, class A4, class A5>
		Arg Arg6() { return Arg5<R, A1, A2, A3, A4>() + ArgTraits<4, A5>::stack_size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6>
		Arg Arg7() { return Arg6<R, A1, A2, A3, A4, A5>() + ArgTraits<5, A6>::stack_size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7>
		Arg Arg8() { return Arg7<R, A1, A2, A3, A4, A5, A6>() + ArgTraits<6, A7>::stack_size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8>
		Arg Arg9() { return Arg8<R, A1, A2, A3, A4, A5, A6, A7>() + ArgTraits<7, A8>::stack_size; }
		template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9>
		Arg Arg10() { return Arg9<R, A1, A2, A3, A4, A5, A6, A7, A8>() + ArgTraits<8, A9>::stack_size; }
	};

}	// namespace detail

/// cdecl function
template<
	class R,
	class A1 = detail::ArgNone,
	class A2 = detail::ArgNone,
	class A3 = detail::ArgNone,
	class A4 = detail::ArgNone,
	class A5 = detail::ArgNone,
	class A6 = detail::ArgNone,
	class A7 = detail::ArgNone,
	class A8 = detail::ArgNone,
	class A9 = detail::ArgNone,
	class A10 = detail::ArgNone>
struct function_cdecl : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7, A8, A9, A10);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/, Arg /*a8*/, Arg /*a9*/, Arg /*a10*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>(), Arg5<R,A1,A2,A4,A5>(), Arg6<R,A1,A2,A4,A5,A6>(), Arg7<R,A1,A2,A4,A5,A6,A7>(), Arg8<R,A1,A2,A4,A5,A6,A7,A8>(), Arg9<R,A1,A2,A4,A5,A6,A7,A8,A9>(), Arg10<R,A1,A2,A4,A5,A6,A7,A8,A9,A10>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 10 arguments and no result
template<class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9, class A10>
struct function_cdecl<void, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7, A8, A9, A10);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/, Arg /*a8*/, Arg /*a9*/, Arg /*a10*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>(), Arg5<void,A1,A2,A4,A5>(), Arg6<void,A1,A2,A4,A5,A6>(), Arg7<void,A1,A2,A4,A5,A6,A7>(), Arg8<void,A1,A2,A4,A5,A6,A7,A8>(), Arg9<void,A1,A2,A4,A5,A6,A7,A8,A9>(), Arg10<void,A1,A2,A4,A5,A6,A7,A8,A9,A10>());
		MakePrologAndEpilog();
	}
};

// specialization for 9 arguments
template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9>
struct function_cdecl<R, A1, A2, A3, A4, A5, A6, A7, A8, A9, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7, A8, A9);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/, Arg /*a8*/, Arg /*a9*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>(), Arg5<R,A1,A2,A4,A5>(), Arg6<R,A1,A2,A4,A5,A6>(), Arg7<R,A1,A2,A4,A5,A6,A7>(), Arg8<R,A1,A2,A4,A5,A6,A7,A8>(), Arg9<R,A1,A2,A4,A5,A6,A7,A8,A9>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 9 arguments and no result
template<class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8, class A9>
struct function_cdecl<void, A1, A2, A3, A4, A5, A6, A7, A8, A9, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7, A8, A9);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/, Arg /*a8*/, Arg /*a9*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>(), Arg5<void,A1,A2,A4,A5>(), Arg6<void,A1,A2,A4,A5,A6>(), Arg7<void,A1,A2,A4,A5,A6,A7>(), Arg8<void,A1,A2,A4,A5,A6,A7,A8>(), Arg9<void,A1,A2,A4,A5,A6,A7,A8,A9>());
		MakePrologAndEpilog();
	}
};

// specialization for 8 arguments
template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8>
struct function_cdecl<R, A1, A2, A3, A4, A5, A6, A7, A8, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7, A8);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/, Arg /*a8*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>(), Arg5<R,A1,A2,A4,A5>(), Arg6<R,A1,A2,A4,A5,A6>(), Arg7<R,A1,A2,A4,A5,A6,A7>(), Arg8<R,A1,A2,A4,A5,A6,A7,A8>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 8 arguments and no result
template<class A1, class A2, class A3, class A4, class A5, class A6, class A7, class A8>
struct function_cdecl<void, A1, A2, A3, A4, A5, A6, A7, A8, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7, A8);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/, Arg /*a8*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>(), Arg5<void,A1,A2,A4,A5>(), Arg6<void,A1,A2,A4,A5,A6>(), Arg7<void,A1,A2,A4,A5,A6,A7>(), Arg8<void,A1,A2,A4,A5,A6,A7,A8>());
		MakePrologAndEpilog();
	}
};

// specialization for 7 arguments
template<class R, class A1, class A2, class A3, class A4, class A5, class A6, class A7>
struct function_cdecl<R, A1, A2, A3, A4, A5, A6, A7, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>(), Arg5<R,A1,A2,A4,A5>(), Arg6<R,A1,A2,A4,A5,A6>(), Arg7<R,A1,A2,A4,A5,A6,A7>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 7 arguments and no result
template<class A1, class A2, class A3, class A4, class A5, class A6, class A7>
struct function_cdecl<void, A1, A2, A3, A4, A5, A6, A7, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6, A7);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/, Arg /*a7*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>(), Arg5<void,A1,A2,A4,A5>(), Arg6<void,A1,A2,A4,A5,A6>(), Arg7<void,A1,A2,A4,A5,A6,A7>());
		MakePrologAndEpilog();
	}
};

// specialization for 6 arguments
template<class R, class A1, class A2, class A3, class A4, class A5, class A6>
struct function_cdecl<R, A1, A2, A3, A4, A5, A6, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>(), Arg5<R,A1,A2,A4,A5>(), Arg6<R,A1,A2,A4,A5,A6>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 6 arguments and no result
template<class A1, class A2, class A3, class A4, class A5, class A6>
struct function_cdecl<void, A1, A2, A3, A4, A5, A6, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4, A5, A6);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/, Arg /*a6*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>(), Arg5<void,A1,A2,A4,A5>(), Arg6<void,A1,A2,A4,A5,A6>());
		MakePrologAndEpilog();
	}
};

// specialization for 5 arguments
template<class R, class A1, class A2, class A3, class A4, class A5>
struct function_cdecl<R, A1, A2, A3, A4, A5, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4, A5);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>(), Arg5<R,A1,A2,A4,A5>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 5 arguments and no result
template<class A1, class A2, class A3, class A4, class A5>
struct function_cdecl<void, A1, A2, A3, A4, A5, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4, A5);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/, Arg /*a5*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>(), Arg5<void,A1,A2,A4,A5>());
		MakePrologAndEpilog();
	}
};

// specialization for 4 arguments
template<class R, class A1, class A2, class A3, class A4>
struct function_cdecl<R, A1, A2, A3, A4, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3, A4);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/) { return Result(); }
	void naked_main() {
		DumpRegArg4<R, A1, A2, A3, A4>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>(), Arg4<R,A1,A2,A4>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 4 arguments and no result
template<class A1, class A2, class A3, class A4>
struct function_cdecl<void, A1, A2, A3, A4, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3, A4);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/, Arg /*a4*/) {}
	void naked_main() {
		DumpRegArg4<void, A1, A2, A3, A4>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>(), Arg4<void,A1,A2,A4>());
		MakePrologAndEpilog();
	}
};

// specialization for 3 arguments
template<class R, class A1, class A2, class A3>
struct function_cdecl<R, A1, A2, A3, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2, A3);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/) { return Result(); }
	void naked_main() {
		DumpRegArg3<R, A1, A2, A3>();
		main(Arg1<R>(), Arg2<R,A1>(), Arg3<R,A1,A2>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 3 arguments and no result
template<class A1, class A2, class A3>
struct function_cdecl<void, A1, A2, A3, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2, A3);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/, Arg /*a3*/) {}
	void naked_main() {
		DumpRegArg3<void, A1, A2, A3>();
		main(Arg1<void>(), Arg2<void,A1>(), Arg3<void,A1,A2>());
		MakePrologAndEpilog();
	}
};

// specialization for 2 arguments
template<class R, class A1, class A2>
struct function_cdecl<R, A1, A2, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1, A2);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/, Arg /*a2*/) { return Result(); }
	void naked_main() {
		DumpRegArg2<R, A1, A2>();
		main(Arg1<R>(), Arg2<R,A1>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 2 arguments and no result
template<class A1, class A2>
struct function_cdecl<void, A1, A2, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1, A2);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/, Arg /*a2*/) {}
	void naked_main() {
		DumpRegArg2<void, A1, A2>();
		main(Arg1<void>(), Arg2<void,A1>());
		MakePrologAndEpilog();
	}
};

// specialization for 1 arguments
template<class R, class A1>
struct function_cdecl<R, A1, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)(A1);
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main(Arg /*a1*/) { return Result(); }
	void naked_main() {
		DumpRegArg1<R, A1>();
		main(Arg1<R>()).Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for 1 arguments and no result
template<class A1>
struct function_cdecl<void, A1, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)(A1);
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main(Arg /*a1*/) {}
	void naked_main() {
		DumpRegArg1<void, A1>();
		main(Arg1<void>());
		MakePrologAndEpilog();
	}
};

// specialization for no arguments
template<class R>
struct function_cdecl<R, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef R (__cdecl *FuncPtr)();
	typedef detail::ResultT<R> Result;	///< main function result type
	typename detail::ResultTraits<R>::ResultPtr result_ptr;

	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual Result main() { return Result(); }
	void naked_main() {
		DumpRegArg0<R>();
		main().Store(*this);
		MakePrologAndEpilog();
	}
};

// specialization for no arguments and no result
template<>
struct function_cdecl<void, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone, detail::ArgNone> : detail::Function_cdecl
{
	typedef void (__cdecl *FuncPtr)();
	function_cdecl(bool dump_regarg_x64 = true) : detail::Function_cdecl(dump_regarg_x64) {}
	operator FuncPtr() { return (FuncPtr)GetCode(); }
	virtual void main() {}
	void naked_main() {
		main();
		MakePrologAndEpilog();
	}
};


/// function
template<
	class R,
	class A1 = detail::ArgNone,
	class A2 = detail::ArgNone,
	class A3 = detail::ArgNone,
	class A4 = detail::ArgNone,
	class A5 = detail::ArgNone,
	class A6 = detail::ArgNone,
	class A7 = detail::ArgNone,
	class A8 = detail::ArgNone,
	class A9 = detail::ArgNone,
	class A10 = detail::ArgNone>
struct function : function_cdecl<R, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10> {};

}	// namespace jitasm

#pragma warning( pop )

#endif	// #ifndef JITASM_H
