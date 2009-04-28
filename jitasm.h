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
#include <set>
#include <map>
#include <algorithm>
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

template<typename T> inline void avoid_unused_warn(const T&) {}
template<typename T, typename U> inline void avoid_unused_warn(const T&, const U&) {}

namespace detail
{
	inline long interlocked_increment(long *addend)				{ return ::InterlockedIncrement(addend); }
	inline long interlocked_decrement(long *addend)				{ return ::InterlockedDecrement(addend); }
	inline long interlocked_exchange(long *target, long value)	{ return ::InterlockedExchange(target, value); }
}	// namespace detail

/// Physical register ID
enum PhysicalRegID
{
	INVALID=-1,
	EAX=0, ECX, EDX, EBX, ESP, EBP, ESI, EDI, R8D, R9D, R10D, R11D, R12D, R13D, R14D, R15D,
	AL=0, CL, DL, BL, AH, CH, DH, BH, R8B, R9B, R10B, R11B, R12B, R13B, R14B, R15B,
	AX=0, CX, DX, BX, SP, BP, SI, DI, R8W, R9W, R10W, R11W, R12W, R13W, R14W, R15W,
	RAX=0, RCX, RDX, RBX, RSP, RBP, RSI, RDI, R8, R9, R10, R11, R12, R13, R14, R15,
	ST0=0, ST1, ST2, ST3, ST4, ST5, ST6, ST7,
	MM0=0, MM1, MM2, MM3, MM4, MM5, MM6, MM7,
	XMM0=0, XMM1, XMM2, XMM3, XMM4, XMM5, XMM6, XMM7, XMM8, XMM9, XMM10, XMM11, XMM12, XMM13, XMM14, XMM15,
	YMM0=0, YMM1, YMM2, YMM3, YMM4, YMM5, YMM6, YMM7, YMM8, YMM9, YMM10, YMM11, YMM12, YMM13, YMM14, YMM15,
};

/// Register type
enum RegType
{
	R_TYPE_GP,				///< General purpose register
	R_TYPE_FPU,				///< FPU register
	R_TYPE_MMX,				///< MMX register
	R_TYPE_XMM,				///< XMM register
	R_TYPE_YMM,				///< YMM register
	R_TYPE_SYMBOLIC_GP,		///< Symbolic general purpose register
	R_TYPE_SYMBOLIC_MMX,	///< Symbolic MMX register
	R_TYPE_SYMBOLIC_XMM,	///< Symbolic XMM register
	R_TYPE_SYMBOLIC_YMM		///< Symbolic YMM register
};

/// Register identifier
struct RegID
{
	RegType type;
	int id;		///< PhysicalRegID or symbolic register id

	bool operator<(const RegID& rhs) const {return type != rhs.type ? type < rhs.type : id < rhs.id;}
	bool IsInvalid() const	{return type == R_TYPE_GP && id == INVALID;}
	bool IsSymbolic() const {return type == R_TYPE_SYMBOLIC_GP || type == R_TYPE_SYMBOLIC_MMX || type == R_TYPE_SYMBOLIC_XMM;}

	static RegID Invalid() {
		RegID reg;
		reg.type = R_TYPE_GP;
		reg.id = INVALID;
		return reg;
	}
	static RegID CreatePhysicalRegID(RegType type_, PhysicalRegID id_) {
		RegID reg;
		reg.type = type_;
		reg.id = id_;
		return reg;
	}
	static RegID CreateSymbolicRegID(RegType type_) {
		static long s_id = 0;
		RegID reg;
		reg.type = type_;
		reg.id = static_cast<int>(detail::interlocked_increment(&s_id));
		return reg;
	}
};

/// Operand type
enum OpdType
{
	O_TYPE_NONE,
	O_TYPE_REG,
	O_TYPE_MEM,
	O_TYPE_IMM,
	O_TYPE_TYPE_MASK		= 0x0F,

	O_TYPE_DUMMY	= 1 << 8,	///< The operand which has this flag is not encoded. This is for register allocator.
	O_TYPE_READ		= 1 << 9,	///< The operand is used for reading.
	O_TYPE_WRITE	= 1 << 10	///< The operand is used for writing.
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
	O_SIZE_224 = 224,
	O_SIZE_256 = 256,
	O_SIZE_864 = 864,
	O_SIZE_4096 = 4096,
};

namespace detail
{
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
		Opd(OpdSize opdsize, const RegID& reg) : opdtype_(O_TYPE_REG), opdsize_(opdsize), reg_(reg) {}
		/// MEM
		Opd(OpdSize opdsize, OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
			: opdtype_(O_TYPE_MEM), opdsize_(opdsize), addrsize_(addrsize), base_(base), index_(index), scale_(scale), disp_(disp) {}
	protected:
		/// IMM
		explicit Opd(OpdSize opdsize, sint64 imm) : opdtype_(O_TYPE_IMM), opdsize_(opdsize), imm_(imm) {}

	public:
		bool	IsNone() const		{return (opdtype_ & O_TYPE_TYPE_MASK) == O_TYPE_NONE;}
		bool	IsReg() const		{return (opdtype_ & O_TYPE_TYPE_MASK) == O_TYPE_REG;}
		bool	IsGpReg() const		{return IsReg() && (reg_.type == R_TYPE_GP || reg_.type == R_TYPE_SYMBOLIC_GP);}
		bool	IsFpuReg() const	{return IsReg() && reg_.type == R_TYPE_FPU;}
		bool	IsMmxReg() const	{return IsReg() && (reg_.type == R_TYPE_MMX || reg_.type == R_TYPE_SYMBOLIC_MMX);}
		bool	IsXmmReg() const	{return IsReg() && (reg_.type == R_TYPE_XMM || reg_.type == R_TYPE_SYMBOLIC_XMM);}
		bool	IsYmmReg() const	{return IsReg() && (reg_.type == R_TYPE_YMM || reg_.type == R_TYPE_SYMBOLIC_YMM);}
		bool	IsMem() const		{return (opdtype_ & O_TYPE_TYPE_MASK) == O_TYPE_MEM;}
		bool	IsImm() const		{return (opdtype_ & O_TYPE_TYPE_MASK) == O_TYPE_IMM;}
		bool	IsDummy() const		{return (opdtype_ & O_TYPE_DUMMY) != 0;}

		OpdSize	GetSize() const		{return opdsize_;}
		OpdSize	GetAddressSize() const	{return addrsize_;}
		RegID	GetReg() const		{ASSERT(IsReg()); return reg_;}
		RegID	GetBase() const		{ASSERT(IsMem()); return base_;}
		RegID	GetIndex() const	{ASSERT(IsMem()); return index_;}
		sint64	GetScale() const	{ASSERT(IsMem()); return scale_;}
		sint64	GetDisp() const		{ASSERT(IsMem()); return disp_;}
		sint64	GetImm() const		{ASSERT(IsImm()); return imm_;}
	};

	/// Add O_TYPE_DUMMY to the specified operand
	inline Opd Dummy(const Opd& opd)
	{
		Opd o(opd);
		o.opdtype_ = static_cast<OpdType>(static_cast<int>(o.opdtype_) | O_TYPE_DUMMY);
		return o;
	}

	/// Add O_TYPE_READ to the specified operand
	inline Opd R(const Opd& opd)
	{
		Opd o(opd);
		o.opdtype_ = static_cast<OpdType>(static_cast<int>(o.opdtype_) | O_TYPE_READ);
		return o;
	}

	/// Add O_TYPE_WRITE to the specified operand
	inline Opd W(const Opd& opd)
	{
		Opd o(opd);
		o.opdtype_ = static_cast<OpdType>(static_cast<int>(o.opdtype_) | O_TYPE_WRITE);
		return o;
	}

	/// Add O_TYPE_READ | O_TYPE_WRITE to the specified operand
	inline Opd RW(const Opd& opd)
	{
		Opd o(opd);
		o.opdtype_ = static_cast<OpdType>(static_cast<int>(o.opdtype_) | O_TYPE_READ | O_TYPE_WRITE);
		return o;
	}

	template<int Size>
	struct OpdT : Opd
	{
		/// NONE
		OpdT() : Opd() {}
		/// REG
		explicit OpdT(const RegID& reg) : Opd(static_cast<OpdSize>(Size), reg) {}
		/// MEM
		OpdT(OpdSize addrsize, RegID base, RegID index, sint64 scale, sint64 disp)
			: Opd(static_cast<OpdSize>(Size), addrsize, base, index, scale, disp) {}
	protected:
		/// IMM
		OpdT(sint64 imm) : Opd(static_cast<OpdSize>(Size), imm) {}
	};

}	// namespace detail

typedef detail::OpdT<O_SIZE_8>		Opd8;
typedef detail::OpdT<O_SIZE_16>		Opd16;
typedef detail::OpdT<O_SIZE_32>		Opd32;
typedef detail::OpdT<O_SIZE_64>		Opd64;
typedef detail::OpdT<O_SIZE_80>		Opd80;
typedef detail::OpdT<O_SIZE_128>	Opd128;
typedef detail::OpdT<O_SIZE_224>	Opd224;		// FPU environment
typedef detail::OpdT<O_SIZE_256>	Opd256;
typedef detail::OpdT<O_SIZE_864>	Opd864;		// FPU state
typedef detail::OpdT<O_SIZE_4096>	Opd4096;	// FPU, MMX, XMM, MXCSR state

/// 8bit general purpose register
struct Reg8 : Opd8 {
	Reg8() : Opd8(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_GP)) {}
	explicit Reg8(PhysicalRegID id) : Opd8(RegID::CreatePhysicalRegID(R_TYPE_GP, id)) {}
};
/// 16bit general purpose register
struct Reg16 : Opd16 {
	Reg16() : Opd16(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_GP)) {}
	explicit Reg16(PhysicalRegID id) : Opd16(RegID::CreatePhysicalRegID(R_TYPE_GP, id)) {}
};
/// 32bit general purpose register
struct Reg32 : Opd32 {
	Reg32() : Opd32(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_GP)) {}
	explicit Reg32(PhysicalRegID id) : Opd32(RegID::CreatePhysicalRegID(R_TYPE_GP, id)) {}
};
#ifdef JITASM64
/// 64bit general purpose register
struct Reg64 : Opd64 {
	Reg64() : Opd64(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_GP)) {}
	explicit Reg64(PhysicalRegID id) : Opd64(RegID::CreatePhysicalRegID(R_TYPE_GP, id)) {}
};
#endif
/// FPU register
struct FpuReg : Opd80 {
	explicit FpuReg(PhysicalRegID id) : Opd80(RegID::CreatePhysicalRegID(R_TYPE_FPU, id)) {}
};
/// MMX register
struct MmxReg : Opd64 {
	MmxReg() : Opd64(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_MMX)) {}
	explicit MmxReg(PhysicalRegID id) : Opd64(RegID::CreatePhysicalRegID(R_TYPE_MMX, id)) {}
};
/// XMM register
struct XmmReg : Opd128 {
	XmmReg() : Opd128(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_XMM)) {}
	explicit XmmReg(PhysicalRegID id) : Opd128(RegID::CreatePhysicalRegID(R_TYPE_XMM, id)) {}
};
/// YMM register
struct YmmReg : Opd256 {
	YmmReg() : Opd256(RegID::CreateSymbolicRegID(R_TYPE_SYMBOLIC_YMM)) {}
	explicit YmmReg(PhysicalRegID id) : Opd256(RegID::CreatePhysicalRegID(R_TYPE_YMM, id)) {}
};

struct Reg8_al : Reg8 {Reg8_al() : Reg8(AL) {}};
struct Reg8_cl : Reg8 {Reg8_cl() : Reg8(CL) {}};
struct Reg16_ax : Reg16 {Reg16_ax() : Reg16(AX) {}};
struct Reg16_dx : Reg16 {Reg16_dx() : Reg16(DX) {}};
struct Reg32_eax : Reg32 {Reg32_eax() : Reg32(EAX) {}};
#ifdef JITASM64
struct Reg64_rax : Reg64 {Reg64_rax() : Reg64(RAX) {}};
#endif
struct FpuReg_st0 : FpuReg {FpuReg_st0() : FpuReg(ST0) {}};
struct XmmReg_xmm0 : XmmReg {XmmReg_xmm0() : XmmReg(XMM0) {}};

template<class OpdN>
struct MemT : OpdN
{
	MemT(OpdSize addrsize, const RegID& base, const RegID& index, sint64 scale, sint64 disp) : OpdN(addrsize, base, index, scale, disp) {}
};
typedef MemT<Opd8>		Mem8;
typedef MemT<Opd16>		Mem16;
typedef MemT<Opd32>		Mem32;
typedef MemT<Opd64>		Mem64;
typedef MemT<Opd80>		Mem80;
typedef MemT<Opd128>	Mem128;
typedef MemT<Opd224>	Mem224;		// FPU environment
typedef MemT<Opd256>	Mem256;
typedef MemT<Opd864>	Mem864;		// FPU state
typedef MemT<Opd4096>	Mem4096;	// FPU, MMX, XMM, MXCSR state

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
	Reg32Expr(const RegID& reg, sint64 disp) : reg_(reg), disp_(disp) {}
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
	Reg32ExprBI(const RegID& base, const RegID& index, sint64 disp) : base_(base), index_(index), disp_(disp) {}
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
	Reg32ExprSI(const RegID& index, sint64 scale, sint64 disp) : index_(index), scale_(scale), disp_(disp) {}
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
	Reg32ExprSIB(const RegID& base, const RegID& index, sint64 scale, sint64 disp) : base_(base), index_(index), scale_(scale), disp_(disp) {}
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
	Reg64Expr(const RegID& reg, sint64 disp) : reg_(reg), disp_(disp) {}
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
	Reg64ExprBI(const RegID& base, const RegID& index, sint64 disp) : base_(base), index_(index), disp_(disp) {}
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
	Reg64ExprSI(const RegID& index, sint64 scale, sint64 disp) : index_(index), scale_(scale), disp_(disp) {}
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
	Reg64ExprSIB(const RegID& base, const RegID& index, sint64 scale, sint64 disp) : base_(base), index_(index), scale_(scale), disp_(disp) {}
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
	MemT<OpdN> operator[](const Reg32Expr& obj)		{return MemT<OpdN>(O_SIZE_32, obj.reg_, RegID::Invalid(), 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg32ExprBI& obj)	{return MemT<OpdN>(O_SIZE_32, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg32ExprSI& obj)	{return MemT<OpdN>(O_SIZE_32, RegID::Invalid(), obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Reg32ExprSIB& obj)	{return MemT<OpdN>(O_SIZE_32, obj.base_, obj.index_, obj.scale_, obj.disp_);}
#ifdef JITASM64
	MemT<OpdN> operator[](sint32 disp)				{return MemT<OpdN>(O_SIZE_64, RegID::Invalid(), RegID::Invalid(), 0, disp);}
	MemT<OpdN> operator[](uint32 disp)				{return MemT<OpdN>(O_SIZE_64, RegID::Invalid(), RegID::Invalid(), 0, (sint32) disp);}
#else
	MemT<OpdN> operator[](sint32 disp)				{return MemT<OpdN>(O_SIZE_32, RegID::Invalid(), RegID::Invalid(), 0, disp);}
	MemT<OpdN> operator[](uint32 disp)				{return MemT<OpdN>(O_SIZE_32, RegID::Invalid(), RegID::Invalid(), 0, (sint32) disp);}
#endif

#ifdef JITASM64
	// 64bit-Addressing
	MemT<OpdN> operator[](const Reg64Expr& obj)		{return MemT<OpdN>(O_SIZE_64, obj.reg_, RegID::Invalid(), 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg64ExprBI& obj)	{return MemT<OpdN>(O_SIZE_64, obj.base_, obj.index_, 0, obj.disp_);}
	MemT<OpdN> operator[](const Reg64ExprSI& obj)	{return MemT<OpdN>(O_SIZE_64, RegID::Invalid(), obj.index_, obj.scale_, obj.disp_);}
	MemT<OpdN> operator[](const Reg64ExprSIB& obj)	{return MemT<OpdN>(O_SIZE_64, obj.base_, obj.index_, obj.scale_, obj.disp_);}
	MemOffset64 operator[](sint64 offset)			{return MemOffset64(offset);}
	MemOffset64 operator[](uint64 offset)			{return MemOffset64((sint64) offset);}
#endif
};

/// Instruction ID
enum InstrID
{
	I_ADC, I_ADD, I_AND,
	I_BSF, I_BSR, I_BSWAP, I_BT, I_BTC, I_BTR, I_BTS,
	I_CALL, I_CBW, I_CLC, I_CLD, I_CLI, I_CLTS, I_CMC, I_CMOVCC, I_CMP, I_CMPS_B, I_CMPS_W, I_CMPS_D, I_CMPS_Q, I_CMPXCHG,
	I_CMPXCHG8B, I_CMPXCHG16B, I_CPUID, I_CWD, I_CDQ, I_CQO,
	I_DEC, I_DIV,
	I_ENTER,
	I_HLT,
	I_IDIV, I_IMUL, I_IN, I_INC, I_INS_B, I_INS_W, I_INS_D, I_INVD, I_INVLPG, I_INT3, I_INTN, I_INTO, I_IRET, I_IRETD, I_IRETQ,
	I_JMP, I_JCC,
	I_LAR, I_LEA, I_LEAVE, I_LLDT, I_LMSW, I_LSL, I_LTR, I_LODS_B, I_LODS_W, I_LODS_D, I_LODS_Q, I_LOOP,
	I_MOV, I_MOVBE, I_MOVS_B, I_MOVS_W, I_MOVS_D, I_MOVS_Q, I_MOVZX, I_MOVSX, I_MOVSXD,	I_MUL,
	I_NEG, I_NOP, I_NOT,
	I_OR, I_OUT, I_OUTS_B, I_OUTS_W, I_OUTS_D,
	I_POP, I_POPF, I_POPFD, I_POPFQ, I_PUSH, I_PUSHF, I_PUSHFD, I_PUSHFQ,
	I_RDMSR, I_RDPMC, I_RDTSC, I_RET, I_RCL, I_RCR, I_ROL, I_ROR, I_RSM, 
	I_SAR, I_SHL, I_SHR, I_SBB, I_SCAS_B, I_SCAS_W, I_SCAS_D, I_SCAS_Q, I_SETCC, I_SHLD, I_SHRD, I_SGDT, I_SIDT, I_SLDT, I_SMSW, I_STC, I_STD, I_STI,
	I_STOS_B, I_STOS_W, I_STOS_D, I_STOS_Q, I_SUB, I_SWAPGS, I_SYSCALL, I_SYSENTER, I_SYSEXIT, I_SYSRET,
	I_TEST,
	I_UD2,
	I_VERR, I_VERW,
	I_WAIT, I_WBINVD, I_WRMSR,
	I_XADD, I_XCHG, I_XLATB, I_XOR,

	I_F2XM1, I_FABS, I_FADD, I_FADDP, I_FIADD,
	I_FBLD, I_FBSTP, I_FCHS, I_FCLEX, I_FNCLEX, I_FCMOVCC, I_FCOM, I_FCOMP, I_FCOMPP, I_FCOMI, I_FCOMIP, I_FCOS,
	I_FDECSTP, I_FDIV, I_FDIVP, I_FIDIV, I_FDIVR, I_FDIVRP, I_FIDIVR,
	I_FFREE,
	I_FICOM, I_FICOMP, I_FILD, I_FINCSTP, I_FINIT, I_FNINIT, I_FIST, I_FISTP,
	I_FLD, I_FLD1, I_FLDCW, I_FLDENV, I_FLDL2E, I_FLDL2T, I_FLDLG2, I_FLDLN2, I_FLDPI, I_FLDZ,
	I_FMUL, I_FMULP, I_FIMUL,
	I_FNOP,
	I_FPATAN, I_FPREM, I_FPREM1, I_FPTAN,
	I_FRNDINT, I_FRSTOR,
	I_FSAVE, I_FNSAVE, I_FSCALE, I_FSIN, I_FSINCOS, I_FSQRT, I_FST, I_FSTP, I_FSTCW, I_FNSTCW, I_FSTENV, I_FNSTENV, I_FSTSW, I_FNSTSW,
	I_FSUB, I_FSUBP, I_FISUB, I_FSUBR, I_FSUBRP, I_FISUBR,
	I_FTST,
	I_FUCOM, I_FUCOMP, I_FUCOMPP, I_FUCOMI, I_FUCOMIP,
	I_FXAM, I_FXCH, I_FXRSTOR, I_FXSAVE, I_FXTRACT,
	I_FYL2X, I_FYL2XP1,

	I_ADDPS, I_ADDSS, I_ADDPD, I_ADDSD, I_ADDSUBPS, I_ADDSUBPD, I_ANDPS, I_ANDPD, I_ANDNPS, I_ANDNPD, I_BLENDPS, I_BLENDPD, I_BLENDVPS, I_BLENDVPD,
	I_CLFLUSH, I_CMPPS, I_CMPSS, I_CMPPD, I_CMPSD, I_COMISS, I_COMISD, I_CRC32,
	I_CVTDQ2PD, I_CVTDQ2PS, I_CVTPD2DQ, I_CVTPD2PI, I_CVTPD2PS, I_CVTPI2PD, I_CVTPI2PS, I_CVTPS2DQ, I_CVTPS2PD, I_CVTPS2PI, I_CVTSD2SI,
	I_CVTSD2SS, I_CVTSI2SD, I_CVTSI2SS, I_CVTSS2SD, I_CVTSS2SI, I_CVTTPD2DQ, I_CVTTPD2PI, I_CVTTPS2DQ, I_CVTTPS2PI, I_CVTTSD2SI, I_CVTTSS2SI,
	I_DIVPS, I_DIVSS, I_DIVPD, I_DIVSD, I_DPPS, I_DPPD, I_EMMS, I_EXTRACTPS, I_FISTTP, I_HADDPS, I_HADDPD, I_HSUBPS, I_HSUBPD, I_INSERTPS, I_LDDQU, I_LDMXCSR, I_LFENCE,
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

	I_VBROADCASTSS, I_VBROADCASTSD, I_VBROADCASTF128,

	I_AESENC, I_AESENCLAST, I_AESDEC, I_AESDECLAST, I_AESIMC, I_AESKEYGENASSIST,
};

enum JumpCondition
{
	JCC_O, JCC_NO, JCC_B, JCC_AE, JCC_E, JCC_NE, JCC_BE, JCC_A, JCC_S, JCC_NS, JCC_P, JCC_NP, JCC_L, JCC_GE, JCC_LE, JCC_G,
	JCC_CXZ, JCC_ECXZ, JCC_RCXZ,
};

enum EncodingFlags
{
	E_SPECIAL				= 1 << 0,
	E_OPERAND_SIZE_PREFIX	= 1 << 1,	///< Operand-size override prefix
	E_REP_PREFIX			= 1 << 2,	///< REP prefix
	E_REXW_PREFIX			= 1 << 3,	///< REX.W
	E_MANDATORY_PREFIX_66	= 1 << 4,	///< Mandatory prefix 66
	E_MANDATORY_PREFIX_F2	= 1 << 5,	///< Mandatory prefix F2
	E_MANDATORY_PREFIX_F3	= 1 << 6,	///< Mandatory prefix F3
	E_VEX					= 1 << 7,
	E_VEX_L					= 1 << 8,
	E_VEX_W					= 1 << 9,
	E_VEX_0F				= 1 << 10,
	E_VEX_0F38				= 1 << 11,
	E_VEX_0F3A				= 1 << 12,
	E_VEX_66				= 1 << 13,
	E_VEX_F2				= 1 << 14,
	E_VEX_F3				= 1 << 15,

	E_VEX_128		= E_VEX,
	E_VEX_256		= E_VEX | E_VEX_L,
	E_VEX_66_0F		= E_VEX_66 | E_VEX_0F,
	E_VEX_66_0F38	= E_VEX_66 | E_VEX_0F38,
	E_VEX_66_0F3A	= E_VEX_66 | E_VEX_0F3A,
	E_VEX_F2_0F		= E_VEX_F2 | E_VEX_0F,
	E_VEX_F2_0F38	= E_VEX_F2 | E_VEX_0F38,
	E_VEX_F2_0F3A	= E_VEX_F2 | E_VEX_0F3A,
	E_VEX_F3_0F		= E_VEX_F3 | E_VEX_0F,
	E_VEX_F3_0F38	= E_VEX_F3 | E_VEX_0F38,
	E_VEX_F3_0F3A	= E_VEX_F3 | E_VEX_0F3A,
	E_VEX_W0		= 0,
	E_VEX_W1		= E_VEX_W,
};

/// Instruction
struct Instr
{
	static const size_t MAX_OPERAND_COUNT = 4;

	InstrID	id_;
	uint32  opcode_;					///< Opcode
	uint32  encoding_flag_;				///< EncodingFlags
	detail::Opd	opd_[MAX_OPERAND_COUNT];	///< Operands

	Instr(InstrID id, uint32 opcode, uint32 encoding_flag, const detail::Opd& opd1 = detail::Opd(), const detail::Opd& opd2 = detail::Opd(), const detail::Opd& opd3 = detail::Opd(), const detail::Opd& opd4 = detail::Opd())
		: id_(id), opcode_(opcode), encoding_flag_(encoding_flag) {opd_[0] = opd1, opd_[1] = opd2, opd_[2] = opd3, opd_[3] = opd4;}

	InstrID GetID() const {return id_;}
	const detail::Opd& GetOpd(size_t index) const {return opd_[index];}
	detail::Opd& GetOpd(size_t index) {return opd_[index];}
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

	uint8 GetWRXB(int w, const detail::Opd& reg, const detail::Opd& r_m)
	{
		uint8 wrxb = w ? 8 : 0;
		if (reg.IsReg()) {
			if (!reg.GetReg().IsInvalid() && reg.GetReg().id >= R8) wrxb |= 4;
		}
		if (r_m.IsReg()) {
			if (r_m.GetReg().id >= R8) wrxb |= 1;
		}
		if (r_m.IsMem()) {
			if (!r_m.GetIndex().IsInvalid() && r_m.GetIndex().id >= R8) wrxb |= 2;
			if (!r_m.GetBase().IsInvalid() && r_m.GetBase().id >= R8) wrxb |= 1;
		}
		return wrxb;
	}

	void EncodePrefixes(uint32 flag, const detail::Opd& reg, const detail::Opd& r_m, const detail::Opd& vex)
	{
		if (flag & E_VEX) {
			// Encode VEX prefix
#ifdef JITASM64
			if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) db(0x67);
#endif
			uint8 vvvv = vex.IsReg() ? 0xF - (uint8) vex.GetReg().id : 0xF;

			uint8 pp = 0;
			if (flag & E_VEX_66) pp = 1;
			else if (flag & E_VEX_F3) pp = 2;
			else if (flag & E_VEX_F2) pp = 3;

			uint8 wrxb = GetWRXB(flag & E_VEX_W, reg, r_m);
			if (wrxb & 0xB || (flag & (E_VEX_0F38 | E_VEX_0F3A))) {
				uint8 mmmmm = 0;
				if (flag & E_VEX_0F) mmmmm = 1;
				else if (flag & E_VEX_0F38) mmmmm = 2;
				else if (flag & E_VEX_0F3A) mmmmm = 3;
				else ASSERT(0);

				db(0xC4);
				db((~wrxb & 7) << 5 | mmmmm);
				db((wrxb & 8) << 4 | vvvv << 3 | (flag & E_VEX_L ? 4 : 0) | pp);
			} else {
				db(0xC5);
				db((~wrxb & 4) << 5 | vvvv << 3 | (flag & E_VEX_L ? 4 : 0) | pp);
			}
		} else {
			uint8 wrxb = GetWRXB(flag & E_REXW_PREFIX, reg, r_m);
			if (wrxb) {
				// Encode REX prefix
				ASSERT(!reg.IsReg() || reg.GetSize() != O_SIZE_8 || reg.GetReg().id < AH || reg.GetReg().id >= R8B);	// AH, BH, CH, or DH may not be used with REX.
				ASSERT(!r_m.IsReg() || r_m.GetSize() != O_SIZE_8 || r_m.GetReg().id < AH || r_m.GetReg().id >= R8B);	// AH, BH, CH, or DH may not be used with REX.

				if (flag & E_REP_PREFIX) db(0xF3);
#ifdef JITASM64
				if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) db(0x67);
#endif
				if (flag & E_OPERAND_SIZE_PREFIX) db(0x66);

				if (flag & E_MANDATORY_PREFIX_66) db(0x66);
				else if (flag & E_MANDATORY_PREFIX_F2) db(0xF2);
				else if (flag & E_MANDATORY_PREFIX_F3) db(0xF3);

				db(0x40 | wrxb);
			} else {
				if (flag & E_MANDATORY_PREFIX_66) db(0x66);
				else if (flag & E_MANDATORY_PREFIX_F2) db(0xF2);
				else if (flag & E_MANDATORY_PREFIX_F3) db(0xF3);

				if (flag & E_REP_PREFIX) db(0xF3);
#ifdef JITASM64
				if (r_m.IsMem() && r_m.GetAddressSize() != O_SIZE_64) db(0x67);
#endif
				if (flag & E_OPERAND_SIZE_PREFIX) db(0x66);
			}
		}
	}

	void EncodeModRM(uint8 reg, const detail::Opd& r_m)
	{
		reg &= 0x7;

		if (r_m.IsReg()) {
			db(0xC0 | reg << 3 | r_m.GetReg().id & 0x7);
		} else if (r_m.IsMem()) {
			ASSERT(r_m.GetBase().type == R_TYPE_GP && r_m.GetIndex().type == R_TYPE_GP);
			int base = r_m.GetBase().id; if (base != INVALID) base &= 0x7;
			int index = r_m.GetIndex().id; if (index != INVALID) index &= 0x7;

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

	void EncodeImm(const detail::Opd& imm)
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

		const detail::Opd& opd1 = instr.GetOpd(0).IsDummy() ? detail::Opd() : instr.GetOpd(0);	ASSERT(!(opd1.IsReg() && opd1.GetReg().IsSymbolic()));
		const detail::Opd& opd2 = instr.GetOpd(1).IsDummy() ? detail::Opd() : instr.GetOpd(1);	ASSERT(!(opd2.IsReg() && opd2.GetReg().IsSymbolic()));
		const detail::Opd& opd3 = instr.GetOpd(2).IsDummy() ? detail::Opd() : instr.GetOpd(2);	ASSERT(!(opd3.IsReg() && opd3.GetReg().IsSymbolic()));
		const detail::Opd& opd4 = instr.GetOpd(3).IsDummy() ? detail::Opd() : instr.GetOpd(3);	ASSERT(!(opd4.IsReg() && opd4.GetReg().IsSymbolic()));

		// +rb, +rw, +rd, +ro
		if (opd1.IsReg() && (opd2.IsNone() || opd2.IsImm())) {
			opcode += opd1.GetReg().id & 0x7;
		}

		if ((opd1.IsImm() || opd1.IsReg()) && (opd2.IsReg() || opd2.IsMem())) {	// ModR/M
			const detail::Opd& reg = opd1;
			const detail::Opd& r_m = opd2;
			const detail::Opd& vex = opd4.IsReg() ? opd4 : opd3.IsReg() ? opd3 : detail::Opd();
			EncodePrefixes(instr.encoding_flag_, reg, r_m, vex);
			EncodeOpcode(opcode);
			EncodeModRM((uint8) (reg.IsImm() ? reg.GetImm() : reg.GetReg().id), r_m);

			// Encode 4th operand
			if (opd4.IsReg() && opd3.IsReg()) {
				EncodeImm(Imm8(static_cast<uint8>(opd3.GetReg().id << 4)));
			}
		} else {
			const detail::Opd& reg = detail::Opd();
			const detail::Opd& r_m = opd1.IsReg() ? opd1 : detail::Opd();
			const detail::Opd& vex = detail::Opd();
			EncodePrefixes(instr.encoding_flag_, reg, r_m, vex);
			EncodeOpcode(opcode);
		}

		if (opd1.IsImm() && !opd2.IsReg() && !opd2.IsMem())	EncodeImm(opd1);
		if (opd2.IsImm())	EncodeImm(opd2);
		if (opd3.IsImm())	EncodeImm(opd3);
	}

	void EncodeALU(const Instr& instr, uint32 opcode)
	{
		const detail::Opd& reg = instr.GetOpd(1);
		const detail::Opd& imm = instr.GetOpd(2);
		ASSERT(instr.GetOpd(0).IsImm() && reg.IsReg() && imm.IsImm());

		if (reg.GetReg().id == EAX && (reg.GetSize() == O_SIZE_8 || !detail::IsInt8(imm.GetImm()))) {
			opcode |= (reg.GetSize() == O_SIZE_8 ? 0 : 1);
			Encode(Instr(instr.GetID(), opcode, instr.encoding_flag_, reg, imm));
		} else {
			Encode(instr);
		}
	}

	void EncodeJMP(const Instr& instr)
	{
		const detail::Opd& imm = instr.GetOpd(0);
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
#ifndef JITASM64
		const detail::Opd& reg = instr.GetOpd(0);
		const detail::Opd& mem = instr.GetOpd(1);
		ASSERT(reg.IsReg() && mem.IsMem());

		if (reg.GetReg().id == EAX && mem.GetBase().IsInvalid() && mem.GetIndex().IsInvalid()) {
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
		const detail::Opd& reg = instr.GetOpd(1);
		const detail::Opd& imm = instr.GetOpd(2);
		ASSERT(instr.GetOpd(0).IsImm() && reg.IsReg() && imm.IsImm());

		if (reg.GetReg().id == EAX) {
			uint32 opcode = 0xA8 | (reg.GetSize() == O_SIZE_8 ? 0 : 1);
			Encode(Instr(instr.GetID(), opcode, instr.encoding_flag_, reg, imm));
		} else {
			Encode(instr);
		}
	}

	void EncodeXCHG(const Instr& instr)
	{
		const detail::Opd& dst = instr.GetOpd(0);
		const detail::Opd& src = instr.GetOpd(1);
		ASSERT(dst.IsReg() && src.IsReg());

		if (dst.GetReg().id == EAX) {
			Encode(Instr(instr.GetID(), 0x90, instr.encoding_flag_, src));
		} else if (src.GetReg().id == EAX) {
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

	template<class It> It prior(const It &it) {
		It i = it;
		return --i;
	}

	template<class It> It next(const It &it) {
		It i = it;
		return ++i;
	}

	inline void append_num(std::string& str, size_t num) {
		if (num >= 10)
			append_num(str, num / 10);
		str.append(1, '0' + num % 10);
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
		void Lock() {while (interlocked_exchange(&lock_, 1));}
		void Unlock() {interlocked_exchange(&lock_, 0);}
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
	Reg16_dx	dx;
	Reg16		cx, bx, sp, bp, si, di;
	Reg32_eax	eax;
	Reg32		ecx, edx, ebx, esp, ebp, esi, edi;
	FpuReg_st0	st0;
	FpuReg		st1, st2, st3, st4, st5, st6, st7;
	MmxReg		mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7;
	XmmReg_xmm0 xmm0;
	XmmReg		xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7;
	YmmReg		ymm0, ymm1, ymm2, ymm3, ymm4, ymm5, ymm6, ymm7;
#ifdef JITASM64
	Reg8		r8b, r9b, r10b, r11b, r12b, r13b, r14b, r15b;
	Reg16		r8w, r9w, r10w, r11w, r12w, r13w, r14w, r15w;
	Reg32		r8d, r9d, r10d, r11d, r12d, r13d, r14d, r15d;
	Reg64_rax	rax;
	Reg64		rcx, rdx, rbx, rsp, rbp, rsi, rdi, r8, r9, r10, r11, r12, r13, r14, r15;
	XmmReg		xmm8, xmm9, xmm10, xmm11, xmm12, xmm13, xmm14, xmm15;
	YmmReg		ymm8, ymm9, ymm10, ymm11, ymm12, ymm13, ymm14, ymm15;
#endif

	AddressingPtr<Opd8>		byte_ptr;
	AddressingPtr<Opd16>	word_ptr;
	AddressingPtr<Opd32>	dword_ptr;
	AddressingPtr<Opd64>	qword_ptr;
	AddressingPtr<Opd64>	mmword_ptr;
	AddressingPtr<Opd128>	xmmword_ptr;
	AddressingPtr<Opd256>	ymmword_ptr;
	AddressingPtr<Opd32>	real4_ptr;
	AddressingPtr<Opd64>	real8_ptr;
	AddressingPtr<Opd80>	real10_ptr;
	AddressingPtr<Opd16>	m2byte_ptr;
	AddressingPtr<Opd224>	m28byte_ptr;
	AddressingPtr<Opd864>	m108byte_ptr;
	AddressingPtr<Opd4096>	m512byte_ptr;

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
		cx(CX), bx(BX), sp(SP), bp(BP), si(SI), di(DI),
		ecx(ECX), edx(EDX), ebx(EBX), esp(ESP), ebp(EBP), esi(ESI), edi(EDI),
		st1(ST1), st2(ST2), st3(ST3), st4(ST4), st5(ST5), st6(ST6), st7(ST7),
		mm0(MM0), mm1(MM1), mm2(MM2), mm3(MM3), mm4(MM4), mm5(MM5), mm6(MM6), mm7(MM7),
		xmm1(XMM1), xmm2(XMM2), xmm3(XMM3), xmm4(XMM4), xmm5(XMM5), xmm6(XMM6), xmm7(XMM7),
		ymm0(YMM0), ymm1(YMM1), ymm2(YMM2), ymm3(YMM3), ymm4(YMM4), ymm5(YMM5), ymm6(YMM6), ymm7(YMM7),
#ifdef JITASM64
		r8b(R8B), r9b(R9B), r10b(R10B), r11b(R11B), r12b(R12B), r13b(R13B), r14b(R14B), r15b(R15B),
		r8w(R8W), r9w(R9W), r10w(R10W), r11w(R11W), r12w(R12W), r13w(R13W), r14w(R14W), r15w(R15W),
		r8d(R8D), r9d(R9D), r10d(R10D), r11d(R11D), r12d(R12D), r13d(R13D), r14d(R14D), r15d(R15D),
		rcx(RCX), rdx(RDX), rbx(RBX), rsp(RSP), rbp(RBP), rsi(RSI), rdi(RDI),
		r8(R8), r9(R9), r10(R10), r11(R11), r12(R12), r13(R13), r14(R14), r15(R15),
		xmm8(XMM8), xmm9(XMM9), xmm10(XMM10), xmm11(XMM11), xmm12(XMM12), xmm13(XMM13), xmm14(XMM14), xmm15(XMM15),
		ymm8(YMM8), ymm9(YMM9), ymm10(YMM10), ymm11(YMM11), ymm12(YMM12), ymm13(YMM13), ymm14(YMM14), ymm15(YMM15),
		zcx(RCX), zdx(RDX), zbx(RBX), zsp(RSP), zbp(RBP), zsi(RSI), zdi(RDI),
#else
		zcx(ECX), zdx(EDX), zbx(EBX), zsp(ESP), zbp(EBP), zsi(ESI), zdi(EDI),
#endif
		assembled_(false)
	{
	}

	typedef std::vector<Instr> InstrList;
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
				const detail::Opd& opd = it->GetOpd(i);
				if (opd.IsGpReg()) {
					gpreg |= 1 << (opd.GetReg().id - EAX);
				}
				else if (opd.IsXmmReg()) {
					xmmreg |= 1 << (opd.GetReg().id - XMM0);
				}
			}
		}

		// Prolog
		InstrList main_instr;
		main_instr.reserve(32);
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
				movdqa(xmmword_ptr[rsp + offset], XmmReg(static_cast<PhysicalRegID>(reg_id)));
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
				movdqa(XmmReg(static_cast<PhysicalRegID>(reg_id)), xmmword_ptr[rsp + offset]);
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

	static bool IsJump(InstrID id)
	{
		return id == I_JMP || id == I_JCC || id == I_LOOP;
	}

	size_t GetJumpTo(const Instr& instr) const
	{
		size_t label_id = (size_t) instr.GetOpd(0).GetImm();
		return labels_[label_id].instr_number;
	}

	// TODO: Return an error when there is no destination.
	void ResolveJump()
	{
		// Replace label indexes with instruncion numbers.
		for (InstrList::iterator it = instrs_.begin(); it != instrs_.end(); ++it) {
			Instr& instr = *it;
			if (IsJump(instr.GetID())) {
				instr = Instr(instr.GetID(), instr.opcode_, instr.encoding_flag_, Imm8(0x7F), Imm64(GetJumpTo(instr)));	// Opd(0) = max value in sint8, Opd(1) = instruction number
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
		instrs_.reserve(128);
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

		InstrList().swap(instrs_);
		LabelList().swap(labels_);
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

	void AppendInstr(InstrID id, uint32 opcode, uint32 encoding_flag, const detail::Opd& opd1 = detail::Opd(), const detail::Opd& opd2 = detail::Opd(), const detail::Opd& opd3 = detail::Opd(), const detail::Opd& opd4 = detail::Opd())
	{
		instrs_.push_back(Instr(id, opcode, encoding_flag, opd1, opd2, opd3, opd4));
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
	void cmp(const Reg8& lhs, const Imm8& imm)		{AppendInstr(I_CMP, 0x80, E_SPECIAL, Imm8(7), R(lhs), imm);}
	void cmp(const Mem8& lhs, const Imm8& imm)		{AppendInstr(I_CMP, 0x80, 0, Imm8(7), R(lhs), imm);}
	void cmp(const Reg16& lhs, const Imm16& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX | E_SPECIAL, Imm8(7), R(lhs), detail::ImmXor8(imm));}
	void cmp(const Mem16& lhs, const Imm16& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_OPERAND_SIZE_PREFIX, Imm8(7), R(lhs), detail::ImmXor8(imm));}
	void cmp(const Reg32& lhs, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_SPECIAL, Imm8(7), R(lhs), detail::ImmXor8(imm));}
	void cmp(const Mem32& lhs, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, 0, Imm8(7), R(lhs), detail::ImmXor8(imm));}
	void cmp(const Reg8& lhs, const Reg8& rhs)		{AppendInstr(I_CMP, 0x38, 0, R(rhs), R(lhs));}
	void cmp(const Mem8& lhs, const Reg8& rhs)		{AppendInstr(I_CMP, 0x38, 0, R(rhs), R(lhs));}
	void cmp(const Reg8& lhs, const Mem8& rhs)		{AppendInstr(I_CMP, 0x3A, 0, R(lhs), R(rhs));}
	void cmp(const Reg16& lhs, const Reg16& rhs)	{AppendInstr(I_CMP, 0x39, E_OPERAND_SIZE_PREFIX, R(rhs), R(lhs));}
	void cmp(const Mem16& lhs, const Reg16& rhs)	{AppendInstr(I_CMP, 0x39, E_OPERAND_SIZE_PREFIX, R(rhs), R(lhs));}
	void cmp(const Reg16& lhs, const Mem16& rhs)	{AppendInstr(I_CMP, 0x3B, E_OPERAND_SIZE_PREFIX, R(lhs), R(rhs));}
	void cmp(const Reg32& lhs, const Reg32& rhs)	{AppendInstr(I_CMP, 0x39, 0, R(rhs), R(lhs));}
	void cmp(const Mem32& lhs, const Reg32& rhs)	{AppendInstr(I_CMP, 0x39, 0, R(rhs), R(lhs));}
	void cmp(const Reg32& lhs, const Mem32& rhs)	{AppendInstr(I_CMP, 0x3B, 0, R(lhs), R(rhs));}
#ifdef JITASM64
	void cmp(const Reg64& lhs, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX | E_SPECIAL, Imm8(7), R(lhs), detail::ImmXor8(imm));}
	void cmp(const Mem64& lhs, const Imm32& imm)	{AppendInstr(I_CMP, detail::IsInt8(imm.GetImm()) ? 0x83 : 0x81, E_REXW_PREFIX, Imm8(7), R(lhs), detail::ImmXor8(imm));}
	void cmp(const Reg64& lhs, const Reg64& rhs)	{AppendInstr(I_CMP, 0x39, E_REXW_PREFIX, R(rhs), R(lhs));}
	void cmp(const Mem64& lhs, const Reg64& rhs)	{AppendInstr(I_CMP, 0x39, E_REXW_PREFIX, R(rhs), R(lhs));}
	void cmp(const Reg64& lhs, const Mem64& rhs)	{AppendInstr(I_CMP, 0x3B, E_REXW_PREFIX, R(lhs), R(rhs));}
#endif
	void cmpsb()		{AppendInstr(I_CMPS_B, 0xA6, 0);}
	void cmpsw()		{AppendInstr(I_CMPS_W, 0xA7, E_OPERAND_SIZE_PREFIX);}
	void cmpsd()		{AppendInstr(I_CMPS_D, 0xA7, 0);}
#ifdef JITASM64
	void cmpsq()		{AppendInstr(I_CMPS_Q, 0xA7, E_REXW_PREFIX);}
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
	void dec(const Reg8& dst)	{AppendInstr(I_DEC, 0xFE, 0, Imm8(1), RW(dst));}
	void dec(const Mem8& dst)	{AppendInstr(I_DEC, 0xFE, 0, Imm8(1), RW(dst));}
	void dec(const Mem16& dst)	{AppendInstr(I_DEC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(1), RW(dst));}
	void dec(const Mem32& dst)	{AppendInstr(I_DEC, 0xFF, 0, Imm8(1), RW(dst));}
#ifndef JITASM64
	void dec(const Reg16& dst)	{AppendInstr(I_DEC, 0x48, E_OPERAND_SIZE_PREFIX, RW(dst));}
	void dec(const Reg32& dst)	{AppendInstr(I_DEC, 0x48, 0, RW(dst));}
#else
	void dec(const Reg16& dst)	{AppendInstr(I_DEC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(1), RW(dst));}
	void dec(const Reg32& dst)	{AppendInstr(I_DEC, 0xFF, 0, Imm8(1), RW(dst));}
	void dec(const Reg64& dst)	{AppendInstr(I_DEC, 0xFF, E_REXW_PREFIX, Imm8(1), RW(dst));}
	void dec(const Mem64& dst)	{AppendInstr(I_DEC, 0xFF, E_REXW_PREFIX, Imm8(1), RW(dst));}
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
	void in(const Reg8_al& dst, const Imm8& src)		{AppendInstr(I_IN, 0xE4, 0, src); avoid_unused_warn(dst);}
	void in(const Reg16_ax& dst, const Imm8& src)		{AppendInstr(I_IN, 0xE5, E_OPERAND_SIZE_PREFIX, src); avoid_unused_warn(dst);}
	void in(const Reg32_eax& dst, const Imm8& src)		{AppendInstr(I_IN, 0xE5, 0, src); avoid_unused_warn(dst);}
	void in(const Reg8_al& dst, const Reg16_dx& src)	{AppendInstr(I_IN, 0xEC, 0); avoid_unused_warn(dst, src);}
	void in(const Reg16_ax& dst, const Reg16_dx& src)	{AppendInstr(I_IN, 0xED, E_OPERAND_SIZE_PREFIX); avoid_unused_warn(dst, src);}
	void in(const Reg32_eax& dst, const Reg16_dx& src)	{AppendInstr(I_IN, 0xED, 0); avoid_unused_warn(dst, src);}
	void inc(const Reg8& dst)	{AppendInstr(I_INC, 0xFE, 0, Imm8(0), RW(dst));}
	void inc(const Mem8& dst)	{AppendInstr(I_INC, 0xFE, 0, Imm8(0), RW(dst));}
	void inc(const Mem16& dst)	{AppendInstr(I_INC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(0), RW(dst));}
	void inc(const Mem32& dst)	{AppendInstr(I_INC, 0xFF, 0, Imm8(0), RW(dst));}
#ifndef JITASM64
	void inc(const Reg16& dst)	{AppendInstr(I_INC, 0x40, E_OPERAND_SIZE_PREFIX, RW(dst));}
	void inc(const Reg32& dst)	{AppendInstr(I_INC, 0x40, 0, RW(dst));}
#else
	void inc(const Reg16& dst)	{AppendInstr(I_INC, 0xFF, E_OPERAND_SIZE_PREFIX, Imm8(0), RW(dst));}
	void inc(const Reg32& dst)	{AppendInstr(I_INC, 0xFF, 0, Imm8(0), RW(dst));}
	void inc(const Reg64& dst)	{AppendInstr(I_INC, 0xFF, E_REXW_PREFIX, Imm8(0), RW(dst));}
	void inc(const Mem64& dst)	{AppendInstr(I_INC, 0xFF, E_REXW_PREFIX, Imm8(0), RW(dst));}
#endif
	void insb()		{AppendInstr(I_INS_B, 0x6C, 0);}
	void insw()		{AppendInstr(I_INS_W, 0x6D, E_OPERAND_SIZE_PREFIX);}
	void insd()		{AppendInstr(I_INS_D, 0x6D, 0);}
	void rep_insb()	{AppendInstr(I_INS_B, 0x6C, E_REP_PREFIX);}
	void rep_insw()	{AppendInstr(I_INS_W, 0x6D, E_REP_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_insd()	{AppendInstr(I_INS_D, 0x6D, E_REP_PREFIX);}
	void int3()					{AppendInstr(I_INT3, 0xCC, 0);}
	void intn(const Imm8& n)	{AppendInstr(I_INTN, 0xCD, 0, n);}
#ifndef JITASM64
	void into()					{AppendInstr(I_INTO, 0xCE, 0);}
#endif
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
	//lgdt
	//lidt
	void lldt(const Reg16& src)	{AppendInstr(I_LLDT, 0x0F00, 0, Imm8(2), src);}
	void lldt(const Mem16& src)	{AppendInstr(I_LLDT, 0x0F00, 0, Imm8(2), src);}
	void lmsw(const Reg16& src)	{AppendInstr(I_LMSW, 0x0F01, 0, Imm8(6), src);}
	void lmsw(const Mem16& src)	{AppendInstr(I_LMSW, 0x0F01, 0, Imm8(6), src);}
	void lodsb()		{AppendInstr(I_LODS_B, 0xAC, 0);}
	void lodsw()		{AppendInstr(I_LODS_W, 0xAD, E_OPERAND_SIZE_PREFIX);}
	void lodsd()		{AppendInstr(I_LODS_D, 0xAD, 0);}
#ifdef JITASM64
	void lodsq()		{AppendInstr(I_LODS_Q, 0xAD, E_REXW_PREFIX);}
#endif
	void rep_lodsb()	{AppendInstr(I_LODS_B, 0xAC, E_REP_PREFIX);}
	void rep_lodsw()	{AppendInstr(I_LODS_W, 0xAD, E_REP_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_lodsd()	{AppendInstr(I_LODS_D, 0xAD, E_REP_PREFIX);}
#ifdef JITASM64
	void rep_lodsq()	{AppendInstr(I_LODS_Q, 0xAD, E_REP_PREFIX | E_REXW_PREFIX);}
#endif
	void loop(const std::string& label_name)	{AppendInstr(I_LOOP, 0xE2, E_SPECIAL, Imm64(GetLabelID(label_name)));}	// short jump only
	void loope(const std::string& label_name)	{AppendInstr(I_LOOP, 0xE1, E_SPECIAL, Imm64(GetLabelID(label_name)));}	// short jump only
	void loopne(const std::string& label_name)	{AppendInstr(I_LOOP, 0xE0, E_SPECIAL, Imm64(GetLabelID(label_name)));}	// short jump only
	void lsl(const Reg16& dst, const Reg16& src)	{AppendInstr(I_LSL, 0x0F03, E_OPERAND_SIZE_PREFIX, dst, src);}
	void lsl(const Reg16& dst, const Mem16& src)	{AppendInstr(I_LSL, 0x0F03, E_OPERAND_SIZE_PREFIX, dst, src);}
	void lsl(const Reg32& dst, const Reg32& src)	{AppendInstr(I_LSL, 0x0F03, 0, dst, src);}
	void lsl(const Reg32& dst, const Mem16& src)	{AppendInstr(I_LSL, 0x0F03, 0, dst, src);}
#ifdef JITASM64
	void lsl(const Reg64& dst, const Reg32& src)	{AppendInstr(I_LSL, 0x0F03, E_REXW_PREFIX, dst, src);}
	void lsl(const Reg64& dst, const Mem16& src)	{AppendInstr(I_LSL, 0x0F03, E_REXW_PREFIX, dst, src);}
#endif
	void ltr(const Reg16& dst)	{AppendInstr(I_LTR, 0x0F00, 0, Imm8(3), dst);}
	void ltr(const Mem16& dst)	{AppendInstr(I_LTR, 0x0F00, 0, Imm8(3), dst);}
	void mov(const Reg8& dst, const Reg8& src)		{AppendInstr(I_MOV, 0x8A, 0, W(dst), R(src));}
	void mov(const Mem8& dst, const Reg8& src)		{AppendInstr(I_MOV, 0x88, E_SPECIAL, R(src), W(dst));}
	void mov(const Reg16& dst, const Reg16& src)	{AppendInstr(I_MOV, 0x8B, E_OPERAND_SIZE_PREFIX, W(dst), R(src));}
	void mov(const Mem16& dst, const Reg16& src)	{AppendInstr(I_MOV, 0x89, E_OPERAND_SIZE_PREFIX | E_SPECIAL, R(src), W(dst));}
	void mov(const Reg32& dst, const Reg32& src)	{AppendInstr(I_MOV, 0x8B, 0, W(dst), R(src));}
	void mov(const Mem32& dst, const Reg32& src)	{AppendInstr(I_MOV, 0x89, E_SPECIAL, R(src), W(dst));}
	void mov(const Reg8& dst, const Mem8& src)		{AppendInstr(I_MOV, 0x8A, E_SPECIAL, W(dst), R(src));}
	void mov(const Reg16& dst, const Mem16& src)	{AppendInstr(I_MOV, 0x8B, E_OPERAND_SIZE_PREFIX | E_SPECIAL, W(dst), R(src));}
	void mov(const Reg32& dst, const Mem32& src)	{AppendInstr(I_MOV, 0x8B, E_SPECIAL, W(dst), R(src));}
	void mov(const Reg8& dst, const Imm8& imm)		{AppendInstr(I_MOV, 0xB0, 0, W(dst), imm);}
	void mov(const Reg16& dst, const Imm16& imm)	{AppendInstr(I_MOV, 0xB8, E_OPERAND_SIZE_PREFIX, W(dst), imm);}
	void mov(const Reg32& dst, const Imm32& imm)	{AppendInstr(I_MOV, 0xB8, 0, W(dst), imm);}
	void mov(const Mem8& dst, const Imm8& imm)		{AppendInstr(I_MOV, 0xC6, 0, Imm8(0), W(dst), imm);}
	void mov(const Mem16& dst, const Imm16& imm)	{AppendInstr(I_MOV, 0xC7, E_OPERAND_SIZE_PREFIX, Imm8(0), W(dst), imm);}
	void mov(const Mem32& dst, const Imm32& imm)	{AppendInstr(I_MOV, 0xC7, 0, Imm8(0), W(dst), imm);}
#ifdef JITASM64
	void mov(const Reg64& dst, const Reg64& src)	{AppendInstr(I_MOV, 0x8B, E_REXW_PREFIX, W(dst), R(src));}
	void mov(const Mem64& dst, const Reg64& src)	{AppendInstr(I_MOV, 0x89, E_REXW_PREFIX, R(src), W(dst));}
	void mov(const Reg64& dst, const Mem64& src)	{AppendInstr(I_MOV, 0x8B, E_REXW_PREFIX, W(dst), R(src));}
	void mov(const Reg64& dst, const Imm64& imm)	{detail::IsInt32(imm.GetImm()) ? AppendInstr(I_MOV, 0xC7, E_REXW_PREFIX, Imm8(0), W(dst), Imm32((sint32) imm.GetImm())) : AppendInstr(I_MOV, 0xB8, E_REXW_PREFIX, W(dst), imm);}
	void mov(const Mem64& dst, const Imm32& imm)	{AppendInstr(I_MOV, 0xC7, E_REXW_PREFIX, Imm8(0), W(dst), imm);}
	void mov(const Reg64_rax& dst, const MemOffset64& src)	{AppendInstr(I_MOV, 0xA1, E_REXW_PREFIX, Imm64(src.GetOffset()), Dummy(W(dst)));}
	void mov(const MemOffset64& dst, const Reg64_rax& src)	{AppendInstr(I_MOV, 0xA3, E_REXW_PREFIX, Imm64(dst.GetOffset()), Dummy(R(src)));}
#endif
	void movbe(const Reg16& dst, const Mem16& src)	{AppendInstr(I_MOVBE, 0x0F38F0, E_OPERAND_SIZE_PREFIX, dst, src);}
	void movbe(const Reg32& dst, const Mem32& src)	{AppendInstr(I_MOVBE, 0x0F38F0, 0, dst, src);}
	void movbe(const Mem16& dst, const Reg16& src)	{AppendInstr(I_MOVBE, 0x0F38F1, E_OPERAND_SIZE_PREFIX, src, dst);}
	void movbe(const Mem32& dst, const Reg32& src)	{AppendInstr(I_MOVBE, 0x0F38F1, 0, src, dst);}
#ifdef JITASM64
	void movbe(const Reg64& dst, const Mem64& src)	{AppendInstr(I_MOVBE, 0x0F38F0, E_REXW_PREFIX, dst, src);}
	void movbe(const Mem64& dst, const Reg64& src)	{AppendInstr(I_MOVBE, 0x0F38F1, E_REXW_PREFIX, src, dst);}
#endif
	void movsb()		{AppendInstr(I_MOVS_B, 0xA4, 0);}
	void movsw()		{AppendInstr(I_MOVS_W, 0xA5, E_OPERAND_SIZE_PREFIX);}
	void movsd()		{AppendInstr(I_MOVS_D, 0xA5, 0);}
#ifdef JITASM64
	void movsq()		{AppendInstr(I_MOVS_Q, 0xA5, E_REXW_PREFIX);}
#endif
	void rep_movsb()	{AppendInstr(I_MOVS_B, 0xA4, E_REP_PREFIX);}
	void rep_movsw()	{AppendInstr(I_MOVS_W, 0xA5, E_REP_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_movsd()	{AppendInstr(I_MOVS_D, 0xA5, E_REP_PREFIX);}
#ifdef JITASM64
	void rep_movsq()	{AppendInstr(I_MOVS_Q, 0xA5, E_REP_PREFIX | E_REXW_PREFIX);}
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
	void out(const Imm8& dst, const Reg8_al& src)		{AppendInstr(I_OUT, 0xE6, 0, dst); avoid_unused_warn(src);}
	void out(const Imm8& dst, const Reg16_ax& src)		{AppendInstr(I_OUT, 0xE7, E_OPERAND_SIZE_PREFIX, dst); avoid_unused_warn(src);}
	void out(const Imm8& dst, const Reg32_eax& src)		{AppendInstr(I_OUT, 0xE7, 0, dst); avoid_unused_warn(src);}
	void out(const Reg16_dx& dst, const Reg8_al& src)	{AppendInstr(I_OUT, 0xEE, 0); avoid_unused_warn(dst, src);}
	void out(const Reg16_dx& dst, const Reg16_ax& src)	{AppendInstr(I_OUT, 0xEF, E_OPERAND_SIZE_PREFIX); avoid_unused_warn(dst, src);}
	void out(const Reg16_dx& dst, const Reg32_eax& src)	{AppendInstr(I_OUT, 0xEF, 0); avoid_unused_warn(dst, src);}
	void outsb()		{AppendInstr(I_OUTS_B, 0x6E, 0);}
	void outsw()		{AppendInstr(I_OUTS_W, 0x6F, E_OPERAND_SIZE_PREFIX);}
	void outsd()		{AppendInstr(I_OUTS_D, 0x6F, 0);}
	void rep_outsb()	{AppendInstr(I_OUTS_B, 0x6E, E_REP_PREFIX);}
	void rep_outsw()	{AppendInstr(I_OUTS_W, 0x6F, E_REP_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_outsd()	{AppendInstr(I_OUTS_D, 0x6F, E_REP_PREFIX);}
	void pop(const Reg16& dst)	{AppendInstr(I_POP, 0x58, E_OPERAND_SIZE_PREFIX, dst);}
	void pop(const Mem16& dst)	{AppendInstr(I_POP, 0x8F, E_OPERAND_SIZE_PREFIX, Imm8(0), dst);}
#ifndef JITASM64
	void pop(const Reg32& dst)	{AppendInstr(I_POP, 0x58, 0, dst);}
	void pop(const Mem32& dst)	{AppendInstr(I_POP, 0x8F, 0, Imm8(0), dst);}
#else
	void pop(const Reg64& dst)	{AppendInstr(I_POP, 0x58, 0, dst);}
	void pop(const Mem64& dst)	{AppendInstr(I_POP, 0x8F, 0, Imm8(0), dst);}
#endif
#ifndef JITASM64
	void popf()		{AppendInstr(I_POPF, 0x9D, E_OPERAND_SIZE_PREFIX);}
	void popfd()	{AppendInstr(I_POPFD, 0x9D, 0);}
#else
	void popf()		{AppendInstr(I_POPF, 0x9D, E_OPERAND_SIZE_PREFIX);}
	void popfq()	{AppendInstr(I_POPFQ, 0x9D, 0);}
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
#ifndef JITASM64
	void pushf()	{AppendInstr(I_PUSHF, 0x9C, E_OPERAND_SIZE_PREFIX);}
	void pushfd()	{AppendInstr(I_PUSHFD, 0x9C, 0);}
#else
	void pushf()	{AppendInstr(I_PUSHF, 0x9C, E_OPERAND_SIZE_PREFIX);}
	void pushfq()	{AppendInstr(I_PUSHFQ, 0x9C, 0);}
#endif
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
	void rdmsr()	{AppendInstr(I_RDMSR, 0x0F32, 0);}
	void rdpmc()	{AppendInstr(I_RDMSR, 0x0F33, 0);}
	void rdtsc()	{AppendInstr(I_RDPMC, 0x0F31, 0);}
	void ret()					{AppendInstr(I_RET, 0xC3, 0);}
	void ret(const Imm16& imm)	{AppendInstr(I_RET, 0xC2, 0, imm);}
	void rsm()		{AppendInstr(I_RSM, 0x0FAA, 0);}
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
	void scasb()		{AppendInstr(I_SCAS_B, 0xAE, 0);}
	void scasw()		{AppendInstr(I_SCAS_W, 0xAF, E_OPERAND_SIZE_PREFIX);}
	void scasd()		{AppendInstr(I_SCAS_D, 0xAF, 0);}
#ifdef JITASM64
	void scasq()		{AppendInstr(I_SCAS_Q, 0xAF, E_REXW_PREFIX);}
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
	void shld(const Reg64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHLD, 0x0FA5, E_REXW_PREFIX, src, dst); avoid_unused_warn(place);}
	void shld(const Mem64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHLD, 0x0FA5, E_REXW_PREFIX, src, dst); avoid_unused_warn(place);}
#endif
	void shrd(const Reg16& dst, const Reg16& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_OPERAND_SIZE_PREFIX, src, dst, place);}
	void shrd(const Mem16& dst, const Reg16& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_OPERAND_SIZE_PREFIX, src, dst, place);}
	void shrd(const Reg16& dst, const Reg16& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_OPERAND_SIZE_PREFIX, src, dst); avoid_unused_warn(place);}
	void shrd(const Mem16& dst, const Reg16& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_OPERAND_SIZE_PREFIX, src, dst); avoid_unused_warn(place);}
	void shrd(const Reg32& dst, const Reg32& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, 0, src, dst, place);}
	void shrd(const Mem32& dst, const Reg32& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, 0, src, dst, place);}
	void shrd(const Reg32& dst, const Reg32& src, const Reg8_cl&)		{AppendInstr(I_SHRD, 0x0FAD, 0, src, dst);}
	void shrd(const Mem32& dst, const Reg32& src, const Reg8_cl&)		{AppendInstr(I_SHRD, 0x0FAD, 0, src, dst);}
#ifdef JITASM64
	void shrd(const Reg64& dst, const Reg64& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_REXW_PREFIX, src, dst, place);}
	void shrd(const Mem64& dst, const Reg64& src, const Imm8& place)	{AppendInstr(I_SHRD, 0x0FAC, E_REXW_PREFIX, src, dst, place);}
	void shrd(const Reg64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_REXW_PREFIX, src, dst); avoid_unused_warn(place);}
	void shrd(const Mem64& dst, const Reg64& src, const Reg8_cl& place)	{AppendInstr(I_SHRD, 0x0FAD, E_REXW_PREFIX, src, dst); avoid_unused_warn(place);}
#endif
	template<class Ty> void sgdt(const MemT<Ty>& dst)	{AppendInstr(I_SGDT, 0x0F01, 0, Imm8(0), dst);}
	template<class Ty> void sidt(const MemT<Ty>& dst)	{AppendInstr(I_SIDT, 0x0F01, 0, Imm8(1), dst);}
	void sldt(const Reg16& dst)	{AppendInstr(I_SLDT, 0x0F00, E_OPERAND_SIZE_PREFIX, Imm8(0), dst);}
	void sldt(const Mem16& dst)	{AppendInstr(I_SLDT, 0x0F00, 0, Imm8(0), dst);}
#ifdef JITASM64
	void sldt(const Reg64& dst)	{AppendInstr(I_SLDT, 0x0F00, E_REXW_PREFIX, Imm8(0), dst);}
#endif
	void smsw(const Reg16& dst)	{AppendInstr(I_SMSW, 0x0F01, E_OPERAND_SIZE_PREFIX, Imm8(4), dst);}
	void smsw(const Mem16& dst)	{AppendInstr(I_SMSW, 0x0F01, 0, Imm8(4), dst);}
#ifdef JITASM64
	void smsw(const Reg64& dst)	{AppendInstr(I_SMSW, 0x0F01, E_REXW_PREFIX, Imm8(4), dst);}
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
	void rep_stosb()	{AppendInstr(I_STOS_B, 0xAA, E_REP_PREFIX);}
	void rep_stosw()	{AppendInstr(I_STOS_W, 0xAB, E_REP_PREFIX | E_OPERAND_SIZE_PREFIX);}
	void rep_stosd()	{AppendInstr(I_STOS_D, 0xAB, E_REP_PREFIX);}
#ifdef JITASM64
	void rep_stosq()	{AppendInstr(I_STOS_Q, 0xAB, E_REP_PREFIX | E_REXW_PREFIX);}
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
#ifndef JITASM64
	void sysenter()	{AppendInstr(I_SYSENTER, 0x0F34, 0);}
	void sysexit()	{AppendInstr(I_SYSEXIT, 0x0F35, 0);}
#else
	void swapgs()	{AppendInstr(I_SWAPGS, 0x0F01F8, 0);}	// 0F 01 /7
	void syscall()	{AppendInstr(I_SYSCALL, 0x0F05, 0);}
	void sysret()	{AppendInstr(I_SYSRET, 0x0F07, 0);}
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
	void verr(const Reg16& dst)	{AppendInstr(I_VERR, 0x0F00, 0, Imm8(4), dst);}
	void verr(const Mem16& dst)	{AppendInstr(I_VERR, 0x0F00, 0, Imm8(4), dst);}
	void verw(const Reg16& dst)	{AppendInstr(I_VERW, 0x0F00, 0, Imm8(5), dst);}
	void verw(const Mem16& dst)	{AppendInstr(I_VERW, 0x0F00, 0, Imm8(5), dst);}
	void wait()		{AppendInstr(I_WAIT, 0x9B, 0);}
	void wbinvd()	{AppendInstr(I_WBINVD, 0x0F09, 0);}
	void wrmsr()	{AppendInstr(I_WRMSR, 0x0F30, 0);}
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
	void xlatb()				{AppendInstr(I_XLATB, 0xD7, 0);}
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
	void f2xm1()	{AppendInstr(I_F2XM1, 0xD9F0, 0);}
	void fabs()		{AppendInstr(I_FABS, 0xD9E1, 0);}
	void fadd(const FpuReg_st0& dst, const FpuReg& src)		{AppendInstr(I_FADD, 0xD8C0, 0, src); avoid_unused_warn(dst);}
	void fadd(const FpuReg& dst, const FpuReg_st0& src)		{AppendInstr(I_FADD, 0xDCC0, 0, dst); avoid_unused_warn(src);}
	void fadd(const Mem32& dst)								{AppendInstr(I_FADD, 0xD8, 0, Imm8(0), dst);}
	void fadd(const Mem64& dst)								{AppendInstr(I_FADD, 0xDC, 0, Imm8(0), dst);}
	void faddp()											{AppendInstr(I_FADDP, 0xDEC1, 0);}
	void faddp(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FADDP, 0xDEC0, 0, dst);  avoid_unused_warn(src);}
	void fiadd(const Mem16& dst)							{AppendInstr(I_FIADD, 0xDE, 0, Imm8(0), dst);}
	void fiadd(const Mem32& dst)							{AppendInstr(I_FIADD, 0xDA, 0, Imm8(0), dst);}
	void fbld(const Mem80& dst)		{AppendInstr(I_FBLD, 0xDF, 0, Imm8(4), dst);}
	void fbstp(const Mem80& dst)	{AppendInstr(I_FBSTP, 0xDF, 0, Imm8(6), dst);}
	void fchs()		{AppendInstr(I_FCHS, 0xD9E0, 0);}
	void fclex()	{AppendInstr(I_FCLEX, 0x9BDBE2, 0);}
	void fnclex()	{AppendInstr(I_FNCLEX, 0xDBE2, 0);}
	void fcmovb(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDAC0, 0, src); avoid_unused_warn(dst);}
	void fcmovbe(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDAD0, 0, src); avoid_unused_warn(dst);}
	void fcmove(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDAC8, 0, src); avoid_unused_warn(dst);}
	void fcmovnb(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDBC0, 0, src); avoid_unused_warn(dst);}
	void fcmovnbe(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDBD0, 0, src); avoid_unused_warn(dst);}
	void fcmovne(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDBC8, 0, src); avoid_unused_warn(dst);}
	void fcmovnu(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDBD8, 0, src); avoid_unused_warn(dst);}
	void fcmovu(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCMOVCC, 0xDAD8, 0, src); avoid_unused_warn(dst);}
	void fcom()												{AppendInstr(I_FCOM, 0xD8D1, 0);}
	void fcom(const FpuReg& dst)							{AppendInstr(I_FCOM, 0xD8D0, 0, dst);}
	void fcom(const Mem32& dst)								{AppendInstr(I_FCOM, 0xD8, 0, Imm8(2), dst);}
	void fcom(const Mem64& dst)								{AppendInstr(I_FCOM, 0xDC, 0, Imm8(2), dst);}
	void fcomp()											{AppendInstr(I_FCOMP, 0xD8D9, 0);}
	void fcomp(const FpuReg& dst)							{AppendInstr(I_FCOMP, 0xD8D8, 0, dst);}
	void fcomp(const Mem32& dst)							{AppendInstr(I_FCOMP, 0xD8, 0, Imm8(3), dst);}
	void fcomp(const Mem64& dst)							{AppendInstr(I_FCOMP, 0xDC, 0, Imm8(3), dst);}
	void fcompp()											{AppendInstr(I_FCOMPP, 0xDED9, 0);}
	void fcomi(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCOMI, 0xDBF0, 0, src); avoid_unused_warn(dst);}
	void fcomip(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FCOMIP, 0xDFF0, 0, src); avoid_unused_warn(dst);}
	void fcos()		{AppendInstr(I_FCOS, 0xD9FF, 0);}
	void fdecstp()	{AppendInstr(I_FDECSTP, 0xD9F6, 0);}
	void fdiv(const FpuReg_st0& dst, const FpuReg& src)		{AppendInstr(I_FDIV, 0xD8F0, 0, src); avoid_unused_warn(dst);}
	void fdiv(const FpuReg& dst, const FpuReg_st0& src)		{AppendInstr(I_FDIV, 0xDCF8, 0, dst); avoid_unused_warn(src);}
	void fdiv(const Mem32& dst)								{AppendInstr(I_FDIV, 0xD8, 0, Imm8(6), dst);}
	void fdiv(const Mem64& dst)								{AppendInstr(I_FDIV, 0xDC, 0, Imm8(6), dst);}
	void fdivp()											{AppendInstr(I_FDIVP, 0xDEF9, 0);}
	void fdivp(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FDIVP, 0xDEF8, 0, dst); avoid_unused_warn(src);}
	void fidiv(const Mem16& dst)							{AppendInstr(I_FIDIV, 0xDE, 0, Imm8(6), dst);}
	void fidiv(const Mem32& dst)							{AppendInstr(I_FIDIV, 0xDA, 0, Imm8(6), dst);}
	void fdivr(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FDIVR, 0xD8F8, 0, src); avoid_unused_warn(dst);}
	void fdivr(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FDIVR, 0xDCF0, 0, dst); avoid_unused_warn(src);}
	void fdivr(const Mem32& dst)							{AppendInstr(I_FDIVR, 0xD8, 0, Imm8(7), dst);}
	void fdivr(const Mem64& dst)							{AppendInstr(I_FDIVR, 0xDC, 0, Imm8(7), dst);}
	void fdivrp()											{AppendInstr(I_FDIVRP, 0xDEF1, 0);}
	void fdivrp(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FDIVRP, 0xDEF0, 0, dst); avoid_unused_warn(src);}
	void fidivr(const Mem16& dst)							{AppendInstr(I_FIDIVR, 0xDE, 0, Imm8(7), dst);}
	void fidivr(const Mem32& dst)							{AppendInstr(I_FIDIVR, 0xDA, 0, Imm8(7), dst);}
	void ffree(const FpuReg& dst)	{AppendInstr(I_FFREE, 0xDDC0, 0, dst);}
	void ficom(const Mem16& dst)	{AppendInstr(I_FICOM, 0xDE, 0, Imm8(2), dst);}
	void ficom(const Mem32& dst)	{AppendInstr(I_FICOM, 0xDA, 0, Imm8(2), dst);}
	void ficomp(const Mem16& dst)	{AppendInstr(I_FICOMP, 0xDE, 0, Imm8(3), dst);}
	void ficomp(const Mem32& dst)	{AppendInstr(I_FICOMP, 0xDA, 0, Imm8(3), dst);}
	void fild(const Mem16& dst)		{AppendInstr(I_FILD, 0xDF, 0, Imm8(0), dst);}
	void fild(const Mem32& dst)		{AppendInstr(I_FILD, 0xDB, 0, Imm8(0), dst);}
	void fild(const Mem64& dst)		{AppendInstr(I_FILD, 0xDF, 0, Imm8(5), dst);}
	void fincstp()	{AppendInstr(I_FINCSTP, 0xD9F7, 0);}
	void finit()	{AppendInstr(I_FINIT, 0x9BDBE3, 0);}
	void fninit()	{AppendInstr(I_FNINIT, 0xDBE3, 0);}
	void fist(const Mem16& dst)		{AppendInstr(I_FIST, 0xDF, 0, Imm8(2), dst);}
	void fist(const Mem32& dst)		{AppendInstr(I_FIST, 0xDB, 0, Imm8(2), dst);}
	void fistp(const Mem16& dst)	{AppendInstr(I_FISTP, 0xDF, 0, Imm8(3), dst);}
	void fistp(const Mem32& dst)	{AppendInstr(I_FISTP, 0xDB, 0, Imm8(3), dst);}
	void fistp(const Mem64& dst)	{AppendInstr(I_FISTP, 0xDF, 0, Imm8(7), dst);}
	void fisttp(const Mem16& dst)	{AppendInstr(I_FISTP, 0xDF, 0, Imm8(1), dst);}
	void fisttp(const Mem32& dst)	{AppendInstr(I_FISTP, 0xDB, 0, Imm8(1), dst);}
	void fisttp(const Mem64& dst)	{AppendInstr(I_FISTP, 0xDD, 0, Imm8(1), dst);}
	void fld(const Mem32& src)		{AppendInstr(I_FLD, 0xD9, 0, Imm8(0), src);}
	void fld(const Mem64& src)		{AppendInstr(I_FLD, 0xDD, 0, Imm8(0), src);}
	void fld(const Mem80& src)		{AppendInstr(I_FLD, 0xDB, 0, Imm8(5), src);}
	void fld(const FpuReg& src)		{AppendInstr(I_FLD, 0xD9C0, 0, src);}
	void fld1()		{AppendInstr(I_FLD1, 0xD9E8, 0);}
	void fldcw(const Mem16& src)	{AppendInstr(I_FLDCW, 0xD9, 0, Imm8(5), src);}
	void fldenv(const Mem224& src)	{AppendInstr(I_FLDENV, 0xD9, 0, Imm8(4), src);}
	void fldl2e()	{AppendInstr(I_FLDL2E, 0xD9EA, 0);}
	void fldl2t()	{AppendInstr(I_FLDL2T, 0xD9E9, 0);}
	void fldlg2()	{AppendInstr(I_FLDLG2, 0xD9EC, 0);}
	void fldln2()	{AppendInstr(I_FLDLN2, 0xD9ED, 0);}
	void fldpi()	{AppendInstr(I_FLDPI, 0xD9EB, 0);}
	void fldz()		{AppendInstr(I_FLDZ, 0xD9EE, 0);}
	void fmul(const FpuReg_st0& dst, const FpuReg& src)		{AppendInstr(I_FMUL, 0xD8C8, 0, src); avoid_unused_warn(dst);}
	void fmul(const FpuReg& dst, const FpuReg_st0& src)		{AppendInstr(I_FMUL, 0xDCC8, 0, dst); avoid_unused_warn(src);}
	void fmul(const Mem32& dst)								{AppendInstr(I_FMUL, 0xD8, 0, Imm8(1), dst);}
	void fmul(const Mem64& dst)								{AppendInstr(I_FMUL, 0xDC, 0, Imm8(1), dst);}
	void fmulp()											{AppendInstr(I_FMULP, 0xDEC9, 0);}
	void fmulp(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FMULP, 0xDEC8, 0, dst); avoid_unused_warn(src);}
	void fimul(const Mem16& dst)							{AppendInstr(I_FIMUL, 0xDE, 0, Imm8(1), dst);}
	void fimul(const Mem32& dst)							{AppendInstr(I_FIMUL, 0xDA, 0, Imm8(1), dst);}
	void fnop()		{AppendInstr(I_FNOP, 0xD9D0, 0);}
	void fpatan()	{AppendInstr(I_FPATAN, 0xD9F3, 0);}
	void fprem()	{AppendInstr(I_FPREM, 0xD9F8, 0);}
	void fprem1()	{AppendInstr(I_FPREM1, 0xD9F5, 0);}
	void fptan()	{AppendInstr(I_FPTAN, 0xD9F2, 0);}
	void frndint()	{AppendInstr(I_FRNDINT, 0xD9FC, 0);}
	void frstor(const Mem864& src)	{AppendInstr(I_FRSTOR, 0xDD, 0, Imm8(4), src);}
	void fsave(const Mem864& dst)	{AppendInstr(I_FSAVE, 0x9BDD, 0, Imm8(6), dst);}
	void fnsave(const Mem864& dst)	{AppendInstr(I_FNSAVE, 0xDD, 0, Imm8(6), dst);}
	void fscale()	{AppendInstr(I_FSCALE, 0xD9FD, 0);}
	void fsin()		{AppendInstr(I_FSIN, 0xD9FE, 0);}
	void fsincos()	{AppendInstr(I_FSINCOS, 0xD9FB, 0);}
	void fsqrt()	{AppendInstr(I_FSQRT, 0xD9FA, 0);}
	void fst(const Mem32& dst)		{AppendInstr(I_FST,	0xD9, 0, Imm8(2), dst);}
	void fst(const Mem64& dst)		{AppendInstr(I_FST,	0xDD, 0, Imm8(2), dst);}
	void fst(const FpuReg& dst)		{AppendInstr(I_FST,	0xDDD0, 0, dst);}
	void fstp(const FpuReg& dst)	{AppendInstr(I_FSTP, 0xDDD8, 0, dst);}
	void fstp(const Mem32& dst)		{AppendInstr(I_FSTP, 0xD9, 0, Imm8(3), dst);}
	void fstp(const Mem64& dst)		{AppendInstr(I_FSTP, 0xDD, 0, Imm8(3), dst);}
	void fstp(const Mem80& dst)		{AppendInstr(I_FSTP, 0xDB, 0, Imm8(7), dst);}
	void fstcw(const Mem16& dst)	{AppendInstr(I_FSTCW, 0x9BD9, 0, Imm8(7), dst);}
	void fnstcw(const Mem16& dst)	{AppendInstr(I_FNSTCW, 0xD9, 0, Imm8(7), dst);}
	void fstenv(const Mem224& dst)	{AppendInstr(I_FSTENV, 0x9BD9, 0, Imm8(6), dst);}
	void fnstenv(const Mem224& dst)	{AppendInstr(I_FNSTENV, 0xD9, 0, Imm8(6), dst);}
	void fstsw(const Mem16& dst)		{AppendInstr(I_FSTSW, 0x9BDD, 0, Imm8(7), dst);}
	void fstsw(const Reg16_ax& dst)		{AppendInstr(I_FSTSW, 0x9BDFE0, 0); avoid_unused_warn(dst);}
	void fnstsw(const Mem16& dst)		{AppendInstr(I_FNSTSW, 0xDD, 0, Imm8(7), dst);}
	void fnstsw(const Reg16_ax& dst)	{AppendInstr(I_FNSTSW, 0xDFE0, 0); avoid_unused_warn(dst);}
	void fsub(const FpuReg_st0& dst, const FpuReg& src)		{AppendInstr(I_FSUB, 0xD8E0, 0, src); avoid_unused_warn(dst);}
	void fsub(const FpuReg& dst, const FpuReg_st0& src)		{AppendInstr(I_FSUB, 0xDCE8, 0, dst); avoid_unused_warn(src);}
	void fsub(const Mem32& dst)								{AppendInstr(I_FSUB, 0xD8, 0, Imm8(4), dst);}
	void fsub(const Mem64& dst)								{AppendInstr(I_FSUB, 0xDC, 0, Imm8(4), dst);}
	void fsubp()											{AppendInstr(I_FSUBP, 0xDEE9, 0);}
	void fsubp(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FSUBP, 0xDEE8, 0, dst); avoid_unused_warn(src);}
	void fisub(const Mem16& dst)							{AppendInstr(I_FISUB, 0xDE, 0, Imm8(4), dst);}
	void fisub(const Mem32& dst)							{AppendInstr(I_FISUB, 0xDA, 0, Imm8(4), dst);}
	void fsubr(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FSUBR, 0xD8E8, 0, src); avoid_unused_warn(dst);}
	void fsubr(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FSUBR, 0xDCE0, 0, dst); avoid_unused_warn(src);}
	void fsubr(const Mem32& dst)							{AppendInstr(I_FSUBR, 0xD8, 0, Imm8(5), dst);}
	void fsubr(const Mem64& dst)							{AppendInstr(I_FSUBR, 0xDC, 0, Imm8(5), dst);}
	void fsubrp()											{AppendInstr(I_FSUBRP, 0xDEE1, 0);}
	void fsubrp(const FpuReg& dst, const FpuReg_st0& src)	{AppendInstr(I_FSUBRP, 0xDEE0, 0, dst); avoid_unused_warn(src);}
	void fisubr(const Mem16& dst)							{AppendInstr(I_FISUBR, 0xDE, 0, Imm8(5), dst);}
	void fisubr(const Mem32& dst)							{AppendInstr(I_FISUBR, 0xDA, 0, Imm8(5), dst);}
	void ftst()		{AppendInstr(I_FTST, 0xD9E4, 0);}
	void fucom()											{AppendInstr(I_FUCOM, 0xDDE1, 0);}
	void fucom(const FpuReg& dst)							{AppendInstr(I_FUCOM, 0xDDE0, 0, dst);}
	void fucomp()											{AppendInstr(I_FUCOMP, 0xDDE9, 0);}
	void fucomp(const FpuReg& dst)							{AppendInstr(I_FUCOMP, 0xDDE8, 0, dst);}
	void fucompp()											{AppendInstr(I_FUCOMPP, 0xDAE9, 0);}
	void fucomi(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FUCOMI, 0xDBE8, 0, src); avoid_unused_warn(dst);}
	void fucomip(const FpuReg_st0& dst, const FpuReg& src)	{AppendInstr(I_FUCOMIP, 0xDFE8, 0, src); avoid_unused_warn(dst);}
	void fwait()	{wait();}
	void fxam()		{AppendInstr(I_FXAM, 0xD9E5, 0);}
	void fxch()						{AppendInstr(I_FXCH, 0xD9C9, 0);}
	void fxch(const FpuReg& dst)	{AppendInstr(I_FXCH, 0xD9C8, 0, dst);}
	void fxrstor(const Mem4096& src)	{AppendInstr(I_FXRSTOR, 0x0FAE, 0, Imm8(1), src);}
	void fxsave(const Mem4096& dst)		{AppendInstr(I_FXSAVE, 0x0FAE, 0, Imm8(0), dst);}
	void fxtract()	{AppendInstr(I_FXTRACT, 0xD9F4, 0);}
	void fyl2x()	{AppendInstr(I_FYL2X, 0xD9F1, 0);}
	void fyl2xp1()	{AppendInstr(I_FYL2XP1, 0xD9F9, 0);}

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
	void blendvps(const XmmReg& dst, const XmmReg& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPS,	0x0F3814, E_MANDATORY_PREFIX_66, dst, src); avoid_unused_warn(mask);}
	void blendvps(const XmmReg& dst, const Mem128& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPS,	0x0F3814, E_MANDATORY_PREFIX_66, dst, src); avoid_unused_warn(mask);}
	void blendvpd(const XmmReg& dst, const XmmReg& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPD,	0x0F3815, E_MANDATORY_PREFIX_66, dst, src); avoid_unused_warn(mask);}
	void blendvpd(const XmmReg& dst, const Mem128& src, const XmmReg_xmm0& mask)	{AppendInstr(I_BLENDVPD,	0x0F3815, E_MANDATORY_PREFIX_66, dst, src); avoid_unused_warn(mask);}
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
	void pblendvb(const XmmReg& dst, const XmmReg& src, const XmmReg_xmm0& mask)	{AppendInstr(I_PBLENDVB, 0x0F3810, E_MANDATORY_PREFIX_66, dst, src); avoid_unused_warn(mask);}
	void pblendvb(const XmmReg& dst, const Mem128& src, const XmmReg_xmm0& mask)	{AppendInstr(I_PBLENDVB, 0x0F3810, E_MANDATORY_PREFIX_66, dst, src); avoid_unused_warn(mask);}
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
	void pcmpestri(const XmmReg& src1, const Mem128& src2, const Imm8& mode){AppendInstr(I_PCMPESTRI,	0x0F3A61, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpestrm(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPESTRM,	0x0F3A60, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpestrm(const XmmReg& src1, const Mem128& src2, const Imm8& mode){AppendInstr(I_PCMPESTRM,	0x0F3A60, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpistri(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPISTRI,	0x0F3A63, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpistri(const XmmReg& src1, const Mem128& src2, const Imm8& mode){AppendInstr(I_PCMPISTRI,	0x0F3A63, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpistrm(const XmmReg& src1, const XmmReg& src2, const Imm8& mode){AppendInstr(I_PCMPISTRM,	0x0F3A62, E_MANDATORY_PREFIX_66, src1, src2, mode);}
	void pcmpistrm(const XmmReg& src1, const Mem128& src2, const Imm8& mode){AppendInstr(I_PCMPISTRM,	0x0F3A62, E_MANDATORY_PREFIX_66, src1, src2, mode);}
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

	// AVX
	void vaddpd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ADDPD, 0x58, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vaddpd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ADDPD, 0x58, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vaddpd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ADDPD, 0x58, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vaddpd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ADDPD, 0x58, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vaddps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ADDPS, 0x58, E_VEX_128 | E_VEX_0F, dst, src2, src1);}
	void vaddps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ADDPS, 0x58, E_VEX_128 | E_VEX_0F, dst, src2, src1);}
	void vaddps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ADDPS, 0x58, E_VEX_256 | E_VEX_0F, dst, src2, src1);}
	void vaddps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ADDPS, 0x58, E_VEX_256 | E_VEX_0F, dst, src2, src1);}
	void vaddsd(const XmmReg& dst, const XmmReg& src1, const Mem64& src2)	{AppendInstr(I_ADDSD, 0x58, E_VEX_128 | E_VEX_F2_0F, dst, src2, src1);}
	void vaddss(const XmmReg& dst, const XmmReg& src1, const Mem32& src2)	{AppendInstr(I_ADDSS, 0x58, E_VEX_128 | E_VEX_F3_0F, dst, src2, src1);}
	void vaddsubpd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ADDSUBPD, 0xD0, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vaddsubpd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ADDSUBPD, 0xD0, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vaddsubpd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ADDSUBPD, 0xD0, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vaddsubpd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ADDSUBPD, 0xD0, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vaddsubps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ADDSUBPS, 0xD0, E_VEX_128 | E_VEX_F2_0F, dst, src2, src1);}
	void vaddsubps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ADDSUBPS, 0xD0, E_VEX_128 | E_VEX_F2_0F, dst, src2, src1);}
	void vaddsubps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ADDSUBPS, 0xD0, E_VEX_256 | E_VEX_F2_0F, dst, src2, src1);}
	void vaddsubps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ADDSUBPS, 0xD0, E_VEX_256 | E_VEX_F2_0F, dst, src2, src1);}
	void aesenc(const XmmReg& dst, const XmmReg& src)							{AppendInstr(I_AESENC, 0x0F38DC, E_MANDATORY_PREFIX_66, dst, src);}
	void aesenc(const XmmReg& dst, const Mem128& src)							{AppendInstr(I_AESENC, 0x0F38DC, E_MANDATORY_PREFIX_66, dst, src);}
	void vaesenc(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)		{AppendInstr(I_AESENC, 0xDC, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void vaesenc(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)		{AppendInstr(I_AESENC, 0xDC, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void aesenclast(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_AESENCLAST, 0x0F38DD, E_MANDATORY_PREFIX_66, dst, src);}
	void aesenclast(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_AESENCLAST, 0x0F38DD, E_MANDATORY_PREFIX_66, dst, src);}
	void vaesenclast(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_AESENCLAST, 0xDD, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void vaesenclast(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_AESENCLAST, 0xDD, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void aesdec(const XmmReg& dst, const XmmReg& src)							{AppendInstr(I_AESDEC, 0x0F38DE, E_MANDATORY_PREFIX_66, dst, src);}
	void aesdec(const XmmReg& dst, const Mem128& src)							{AppendInstr(I_AESDEC, 0x0F38DE, E_MANDATORY_PREFIX_66, dst, src);}
	void vaesdec(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)		{AppendInstr(I_AESDEC, 0xDE, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void vaesdec(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)		{AppendInstr(I_AESDEC, 0xDE, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void aesdeclast(const XmmReg& dst, const XmmReg& src)						{AppendInstr(I_AESDECLAST, 0x0F38DF, E_MANDATORY_PREFIX_66, dst, src);}
	void aesdeclast(const XmmReg& dst, const Mem128& src)						{AppendInstr(I_AESDECLAST, 0x0F38DF, E_MANDATORY_PREFIX_66, dst, src);}
	void vaesdeclast(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_AESDECLAST, 0xDF, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void vaesdeclast(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_AESDECLAST, 0xDF, E_VEX_128 | E_VEX_66_0F38, dst, src2, src1);}
	void aesimc(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_AESIMC, 0x0F38DB, E_MANDATORY_PREFIX_66, dst, src);}
	void aesimc(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_AESIMC, 0x0F38DB, E_MANDATORY_PREFIX_66, dst, src);}
	void vaesimc(const XmmReg& dst, const XmmReg& src)		{AppendInstr(I_AESIMC, 0xDB, E_VEX_128 | E_VEX_66_0F38, dst, src);}
	void vaesimc(const XmmReg& dst, const Mem128& src)		{AppendInstr(I_AESIMC, 0xDB, E_VEX_128 | E_VEX_66_0F38, dst, src);}
	void aeskeygenassist(const XmmReg& dst, const XmmReg& src, const Imm8& imm)		{AppendInstr(I_AESKEYGENASSIST, 0x0F3ADF, E_MANDATORY_PREFIX_66, dst, src, imm);}
	void aeskeygenassist(const XmmReg& dst, const Mem128& src, const Imm8& imm)		{AppendInstr(I_AESKEYGENASSIST, 0x0F3ADF, E_MANDATORY_PREFIX_66, dst, src, imm);}
	void vaeskeygenassist(const XmmReg& dst, const XmmReg& src, const Imm8& imm)	{AppendInstr(I_AESKEYGENASSIST, 0xDF, E_VEX_128 | E_VEX_66_0F3A, dst, src, imm);}
	void vaeskeygenassist(const XmmReg& dst, const Mem128& src, const Imm8& imm)	{AppendInstr(I_AESKEYGENASSIST, 0xDF, E_VEX_128 | E_VEX_66_0F3A, dst, src, imm);}
	void vandpd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ANDPD, 0x54, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vandpd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ANDPD, 0x54, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vandpd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ANDPD, 0x54, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vandpd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ANDPD, 0x54, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vandps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ANDPS, 0x54, E_VEX_128 | E_VEX_0F, dst, src2, src1);}
	void vandps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ANDPS, 0x54, E_VEX_128 | E_VEX_0F, dst, src2, src1);}
	void vandps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ANDPS, 0x54, E_VEX_256 | E_VEX_0F, dst, src2, src1);}
	void vandps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ANDPS, 0x54, E_VEX_256 | E_VEX_0F, dst, src2, src1);}
	void vandnpd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ANDNPD, 0x55, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vandnpd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ANDNPD, 0x55, E_VEX_128 | E_VEX_66_0F, dst, src2, src1);}
	void vandnpd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ANDNPD, 0x55, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vandnpd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ANDNPD, 0x55, E_VEX_256 | E_VEX_66_0F, dst, src2, src1);}
	void vandnps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_ANDNPS, 0x55, E_VEX_128 | E_VEX_0F, dst, src2, src1);}
	void vandnps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2)	{AppendInstr(I_ANDNPS, 0x55, E_VEX_128 | E_VEX_0F, dst, src2, src1);}
	void vandnps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2)	{AppendInstr(I_ANDNPS, 0x55, E_VEX_256 | E_VEX_0F, dst, src2, src1);}
	void vandnps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2)	{AppendInstr(I_ANDNPS, 0x55, E_VEX_256 | E_VEX_0F, dst, src2, src1);}
	void vblendpd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const Imm8& mask)	{AppendInstr(I_BLENDPD, 0x0D, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendpd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2, const Imm8& mask)	{AppendInstr(I_BLENDPD, 0x0D, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendpd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2, const Imm8& mask)	{AppendInstr(I_BLENDPD, 0x0D, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendpd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2, const Imm8& mask)	{AppendInstr(I_BLENDPD, 0x0D, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const Imm8& mask)	{AppendInstr(I_BLENDPS, 0x0C, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2, const Imm8& mask)	{AppendInstr(I_BLENDPS, 0x0C, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2, const Imm8& mask)	{AppendInstr(I_BLENDPS, 0x0C, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2, const Imm8& mask)	{AppendInstr(I_BLENDPS, 0x0C, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvpd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const XmmReg& mask)	{AppendInstr(I_BLENDVPD, 0x4B, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvpd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2, const XmmReg& mask)	{AppendInstr(I_BLENDVPD, 0x4B, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvpd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2, const YmmReg& mask)	{AppendInstr(I_BLENDVPD, 0x4B, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvpd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2, const YmmReg& mask)	{AppendInstr(I_BLENDVPD, 0x4B, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const XmmReg& mask)	{AppendInstr(I_BLENDVPS, 0x4A, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2, const XmmReg& mask)	{AppendInstr(I_BLENDVPS, 0x4A, E_VEX_128 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2, const YmmReg& mask)	{AppendInstr(I_BLENDVPS, 0x4A, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vblendvps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2, const YmmReg& mask)	{AppendInstr(I_BLENDVPS, 0x4A, E_VEX_256 | E_VEX_66_0F3A, dst, src2, mask, src1);}
	void vbroadcastss(const XmmReg& dst, const Mem32& src)	{AppendInstr(I_VBROADCASTSS, 0x18, E_VEX_128 | E_VEX_66_0F38, dst, src);}
	void vbroadcastss(const YmmReg& dst, const Mem32& src)	{AppendInstr(I_VBROADCASTSS, 0x18, E_VEX_256 | E_VEX_66_0F38, dst, src);}
	void vbroadcastsd(const YmmReg& dst, const Mem64 src)	{AppendInstr(I_VBROADCASTSD, 0x19, E_VEX_256 | E_VEX_66_0F38, dst, src);}
	void vbroadcastf128(const YmmReg& dst, const Mem128& src)	{AppendInstr(I_VBROADCASTF128, 0x1A, E_VEX_256 | E_VEX_66_0F38, dst, src);}
	void vcmppd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const Imm8& imm)	{AppendInstr(I_CMPPD, 0xC2, E_VEX_128 | E_VEX_66_0F, dst, src2, imm, src1);}
	void vcmppd(const XmmReg& dst, const XmmReg& src1, const Mem128& src2, const Imm8& imm)	{AppendInstr(I_CMPPD, 0xC2, E_VEX_128 | E_VEX_66_0F, dst, src2, imm, src1);}
	void vcmppd(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2, const Imm8& imm)	{AppendInstr(I_CMPPD, 0xC2, E_VEX_256 | E_VEX_66_0F, dst, src2, imm, src1);}
	void vcmppd(const YmmReg& dst, const YmmReg& src1, const Mem256& src2, const Imm8& imm)	{AppendInstr(I_CMPPD, 0xC2, E_VEX_256 | E_VEX_66_0F, dst, src2, imm, src1);}
	void vcmpps(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const Imm8& imm)	{AppendInstr(I_CMPPS, 0xC2, E_VEX_128 | E_VEX_0F, dst, src2, imm, src1);}
	void vcmpps(const XmmReg& dst, const XmmReg& src1, const Mem128& src2, const Imm8& imm)	{AppendInstr(I_CMPPS, 0xC2, E_VEX_128 | E_VEX_0F, dst, src2, imm, src1);}
	void vcmpps(const YmmReg& dst, const YmmReg& src1, const YmmReg& src2, const Imm8& imm)	{AppendInstr(I_CMPPS, 0xC2, E_VEX_256 | E_VEX_0F, dst, src2, imm, src1);}
	void vcmpps(const YmmReg& dst, const YmmReg& src1, const Mem256& src2, const Imm8& imm)	{AppendInstr(I_CMPPS, 0xC2, E_VEX_256 | E_VEX_0F, dst, src2, imm, src1);}
	void vcmpsd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const Imm8& imm)	{AppendInstr(I_CMPSD, 0xC2, E_VEX_128 | E_VEX_F2_0F, dst, src2, imm, src1);}
	void vcmpsd(const XmmReg& dst, const XmmReg& src1, const Mem64& src2, const Imm8& imm)	{AppendInstr(I_CMPSD, 0xC2, E_VEX_128 | E_VEX_F2_0F, dst, src2, imm, src1);}
	void vcmpss(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2, const Imm8& imm)	{AppendInstr(I_CMPSS, 0xC2, E_VEX_128 | E_VEX_F3_0F, dst, src2, imm, src1);}
	void vcmpss(const XmmReg& dst, const XmmReg& src1, const Mem32& src2, const Imm8& imm)	{AppendInstr(I_CMPSS, 0xC2, E_VEX_128 | E_VEX_F3_0F, dst, src2, imm, src1);}
	void vcomisd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_COMISD, 0x2F, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcomisd(const XmmReg& dst, const Mem64& src)	{AppendInstr(I_COMISD, 0x2F, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcomiss(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_COMISS, 0x2F, E_VEX_128 | E_VEX_0F, dst, src);}
	void vcomiss(const XmmReg& dst, const Mem32& src)	{AppendInstr(I_COMISS, 0x2F, E_VEX_128 | E_VEX_0F, dst, src);}
	void vcvtdq2pd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTDQ2PD, 0xE6, E_VEX_128 | E_VEX_F3_0F, dst, src);}
	void vcvtdq2pd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTDQ2PD, 0xE6, E_VEX_128 | E_VEX_F3_0F, dst, src);}
	void vcvtdq2pd(const YmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTDQ2PD, 0xE6, E_VEX_256 | E_VEX_F3_0F, dst, src);}
	void vcvtdq2pd(const YmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTDQ2PD, 0xE6, E_VEX_256 | E_VEX_F3_0F, dst, src);}
	void vcvtdq2ps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTDQ2PS, 0x5B, E_VEX_128 | E_VEX_0F, dst, src);}
	void vcvtdq2ps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTDQ2PS, 0x5B, E_VEX_128 | E_VEX_0F, dst, src);}
	void vcvtdq2ps(const YmmReg& dst, const YmmReg& src)	{AppendInstr(I_CVTDQ2PS, 0x5B, E_VEX_256 | E_VEX_0F, dst, src);}
	void vcvtdq2ps(const YmmReg& dst, const Mem256& src)	{AppendInstr(I_CVTDQ2PS, 0x5B, E_VEX_256 | E_VEX_0F, dst, src);}
	void vcvtpd2dq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTPD2DQ, 0xE6, E_VEX_128 | E_VEX_F2_0F, dst, src);}
	void vcvtpd2dq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTPD2DQ, 0xE6, E_VEX_128 | E_VEX_F2_0F, dst, src);}
	void vcvtpd2dq(const XmmReg& dst, const YmmReg& src)	{AppendInstr(I_CVTPD2DQ, 0xE6, E_VEX_256 | E_VEX_F2_0F, dst, src);}
	void vcvtpd2dq(const XmmReg& dst, const Mem256& src)	{AppendInstr(I_CVTPD2DQ, 0xE6, E_VEX_256 | E_VEX_F2_0F, dst, src);}
	void vcvtpd2ps(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTPD2PS, 0x5A, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcvtpd2ps(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTPD2PS, 0x5A, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcvtpd2ps(const XmmReg& dst, const YmmReg& src)	{AppendInstr(I_CVTPD2PS, 0x5A, E_VEX_256 | E_VEX_66_0F, dst, src);}
	void vcvtpd2ps(const XmmReg& dst, const Mem256& src)	{AppendInstr(I_CVTPD2PS, 0x5A, E_VEX_256 | E_VEX_66_0F, dst, src);}
	void vcvtps2dq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTPS2DQ, 0x5B, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcvtps2dq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTPS2DQ, 0x5B, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcvtps2dq(const YmmReg& dst, const YmmReg& src)	{AppendInstr(I_CVTPS2DQ, 0x5B, E_VEX_256 | E_VEX_66_0F, dst, src);}
	void vcvtps2dq(const YmmReg& dst, const Mem256& src)	{AppendInstr(I_CVTPS2DQ, 0x5B, E_VEX_256 | E_VEX_66_0F, dst, src);}
	void vcvtps2pd(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTPS2PD, 0x5A, E_VEX_128 | E_VEX_0F, dst, src);}
	void vcvtps2pd(const XmmReg& dst, const Mem64& src)		{AppendInstr(I_CVTPS2PD, 0x5A, E_VEX_128 | E_VEX_0F, dst, src);}
	void vcvtps2pd(const YmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTPS2PD, 0x5A, E_VEX_256 | E_VEX_0F, dst, src);}
	void vcvtps2pd(const YmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTPS2PD, 0x5A, E_VEX_256 | E_VEX_0F, dst, src);}
	void vcvtsd2si(const Reg32 dst, const XmmReg& src)	{AppendInstr(I_CVTSD2SI, 0x2D, E_VEX_128 | E_VEX_F2_0F | E_VEX_W0, dst, src);}
	void vcvtsd2si(const Reg32 dst, const Mem64& src)	{AppendInstr(I_CVTSD2SI, 0x2D, E_VEX_128 | E_VEX_F2_0F | E_VEX_W0, dst, src);}
#ifdef JITASM64
	void vcvtsd2si(const Reg64 dst, const XmmReg& src)	{AppendInstr(I_CVTSD2SI, 0x2D, E_VEX_128 | E_VEX_F2_0F | E_VEX_W1, dst, src);}
	void vcvtsd2si(const Reg64 dst, const Mem64& src)	{AppendInstr(I_CVTSD2SI, 0x2D, E_VEX_128 | E_VEX_F2_0F | E_VEX_W1, dst, src);}
#endif
	void vcvtsd2ss(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_CVTSD2SS, 0x5A, E_VEX_128 | E_VEX_F2_0F, dst, src2, src1);}
	void vcvtsd2ss(const XmmReg& dst, const XmmReg& src1, const Mem64& src2)	{AppendInstr(I_CVTSD2SS, 0x5A, E_VEX_128 | E_VEX_F2_0F, dst, src2, src1);}
	void vcvtsi2sd(const XmmReg& dst, const XmmReg& src1, const Reg32& src2)	{AppendInstr(I_CVTSI2SD, 0x2A, E_VEX_128 | E_VEX_F2_0F | E_VEX_W0, dst, src2, src1);}
	void vcvtsi2sd(const XmmReg& dst, const XmmReg& src1, const Mem32& src2)	{AppendInstr(I_CVTSI2SD, 0x2A, E_VEX_128 | E_VEX_F2_0F | E_VEX_W0, dst, src2, src1);}
#ifdef JITASM64
	void vcvtsi2sd(const XmmReg& dst, const XmmReg& src1, const Reg64& src2)	{AppendInstr(I_CVTSI2SD, 0x2A, E_VEX_128 | E_VEX_F2_0F | E_VEX_W1, dst, src2, src1);}
	void vcvtsi2sd(const XmmReg& dst, const XmmReg& src1, const Mem64& src2)	{AppendInstr(I_CVTSI2SD, 0x2A, E_VEX_128 | E_VEX_F2_0F | E_VEX_W1, dst, src2, src1);}
#endif
	void vcvtsi2ss(const XmmReg& dst, const XmmReg& src1, const Reg32& src2)	{AppendInstr(I_CVTSI2SS, 0x2A, E_VEX_128 | E_VEX_F3_0F | E_VEX_W0, dst, src2, src1);}
	void vcvtsi2ss(const XmmReg& dst, const XmmReg& src1, const Mem32& src2)	{AppendInstr(I_CVTSI2SS, 0x2A, E_VEX_128 | E_VEX_F3_0F | E_VEX_W0, dst, src2, src1);}
#ifdef JITASM64
	void vcvtsi2ss(const XmmReg& dst, const XmmReg& src1, const Reg64& src2)	{AppendInstr(I_CVTSI2SS, 0x2A, E_VEX_128 | E_VEX_F3_0F | E_VEX_W1, dst, src2, src1);}
	void vcvtsi2ss(const XmmReg& dst, const XmmReg& src1, const Mem64& src2)	{AppendInstr(I_CVTSI2SS, 0x2A, E_VEX_128 | E_VEX_F3_0F | E_VEX_W1, dst, src2, src1);}
#endif
	void vcvtss2sd(const XmmReg& dst, const XmmReg& src1, const XmmReg& src2)	{AppendInstr(I_CVTSS2SD, 0x5A, E_VEX_128 | E_VEX_F3_0F, dst, src2, src1);}
	void vcvtss2sd(const XmmReg& dst, const XmmReg& src1, const Mem32& src2)	{AppendInstr(I_CVTSS2SD, 0x5A, E_VEX_128 | E_VEX_F3_0F, dst, src2, src1);}
	void vcvtss2si(const Reg32& dst, const XmmReg& src)	{AppendInstr(I_CVTSS2SI, 0x2D, E_VEX_128 | E_VEX_F3_0F | E_VEX_W0, dst, src);}
	void vcvtss2si(const Reg32& dst, const Mem32& src)	{AppendInstr(I_CVTSS2SI, 0x2D, E_VEX_128 | E_VEX_F3_0F | E_VEX_W0, dst, src);}
#ifdef JITASM64
	void vcvtss2si(const Reg64& dst, const XmmReg& src)	{AppendInstr(I_CVTSS2SI, 0x2D, E_VEX_128 | E_VEX_F3_0F | E_VEX_W1, dst, src);}
	void vcvtss2si(const Reg64& dst, const Mem32& src)	{AppendInstr(I_CVTSS2SI, 0x2D, E_VEX_128 | E_VEX_F3_0F | E_VEX_W1, dst, src);}
#endif
	void vcvttpd2dq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTTPD2DQ, 0xE6, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcvttpd2dq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTTPD2DQ, 0xE6, E_VEX_128 | E_VEX_66_0F, dst, src);}
	void vcvttpd2dq(const XmmReg& dst, const YmmReg& src)	{AppendInstr(I_CVTTPD2DQ, 0xE6, E_VEX_256 | E_VEX_66_0F, dst, src);}
	void vcvttpd2dq(const XmmReg& dst, const Mem256& src)	{AppendInstr(I_CVTTPD2DQ, 0xE6, E_VEX_256 | E_VEX_66_0F, dst, src);}
	void vcvttps2dq(const XmmReg& dst, const XmmReg& src)	{AppendInstr(I_CVTTPS2DQ, 0x5B, E_VEX_128 | E_VEX_F3_0F, dst, src);}
	void vcvttps2dq(const XmmReg& dst, const Mem128& src)	{AppendInstr(I_CVTTPS2DQ, 0x5B, E_VEX_128 | E_VEX_F3_0F, dst, src);}
	void vcvttps2dq(const YmmReg& dst, const YmmReg& src)	{AppendInstr(I_CVTTPS2DQ, 0x5B, E_VEX_256 | E_VEX_F3_0F, dst, src);}
	void vcvttps2dq(const YmmReg& dst, const Mem256& src)	{AppendInstr(I_CVTTPS2DQ, 0x5B, E_VEX_256 | E_VEX_F3_0F, dst, src);}
	void vcvttsd2si(const Reg32& dst, const XmmReg& src)	{AppendInstr(I_CVTSD2SI, 0x2C, E_VEX_128 | E_VEX_F2_0F | E_VEX_W0, dst, src);}
	void vcvttsd2si(const Reg32& dst, const Mem64& src)		{AppendInstr(I_CVTSD2SI, 0x2C, E_VEX_128 | E_VEX_F2_0F | E_VEX_W0, dst, src);}
#ifdef JITASM64
	void vcvttsd2si(const Reg64& dst, const XmmReg& src)	{AppendInstr(I_CVTSD2SI, 0x2C, E_VEX_128 | E_VEX_F2_0F | E_VEX_W1, dst, src);}
	void vcvttsd2si(const Reg64& dst, const Mem64& src)		{AppendInstr(I_CVTSD2SI, 0x2C, E_VEX_128 | E_VEX_F2_0F | E_VEX_W1, dst, src);}
#endif
	void vcvttss2si(const Reg32& dst, const XmmReg& src)	{AppendInstr(I_CVTSS2SI, 0x2C, E_VEX_128 | E_VEX_F3_0F | E_VEX_W0, dst, src);}
	void vcvttss2si(const Reg32& dst, const Mem32& src)		{AppendInstr(I_CVTSS2SI, 0x2C, E_VEX_128 | E_VEX_F3_0F | E_VEX_W0, dst, src);}
#ifdef JITASM64
	void vcvttss2si(const Reg64& dst, const XmmReg& src)	{AppendInstr(I_CVTSS2SI, 0x2C, E_VEX_128 | E_VEX_F3_0F | E_VEX_W1, dst, src);}
	void vcvttss2si(const Reg64& dst, const Mem32& src)		{AppendInstr(I_CVTSS2SI, 0x2C, E_VEX_128 | E_VEX_F3_0F | E_VEX_W1, dst, src);}
#endif

	struct ControlState
	{
		size_t label1;
		size_t label2;
	};
	ControlState ctrl_state_;
	std::deque<ControlState> ctrl_state_stack_;

	// Repeat, Until
	void Repeat()
	{
		ctrl_state_stack_.push_back(ctrl_state_);
		ctrl_state_.label1 = NewLabelID("");	// begin
		ctrl_state_.label2 = 0;

		L(ctrl_state_.label1);
	}
	template<class Ty>
	void Until(const Ty& expr)
	{
		size_t label = NewLabelID("");
		expr(*this, ctrl_state_.label1, label);
		L(label);

		ctrl_state_ = *ctrl_state_stack_.rbegin();
		ctrl_state_stack_.pop_back();
	}

	// While, EndW
	template<class Ty>
	void While(const Ty& expr)
	{
		ctrl_state_stack_.push_back(ctrl_state_);
		ctrl_state_.label1 = NewLabelID("");	// begin
		ctrl_state_.label2 = NewLabelID("");	// end

		size_t label = NewLabelID("");
		L(ctrl_state_.label1);
		expr(*this, label, ctrl_state_.label2);
		L(label);
	}
	void EndW()
	{
		AppendJmp(ctrl_state_.label1);
		L(ctrl_state_.label2);

		ctrl_state_ = *ctrl_state_stack_.rbegin();
		ctrl_state_stack_.pop_back();
	}

	// If, ElseIf, Else, EndIf
	template<class Ty>
	void If(const Ty& expr)
	{
		ctrl_state_stack_.push_back(ctrl_state_);
		ctrl_state_.label1 = NewLabelID("");	// else
		ctrl_state_.label2 = NewLabelID("");	// end

		size_t label = NewLabelID("");
		expr(*this, label, ctrl_state_.label1);
		L(label);
	}
	template<class Ty>
	void ElseIf(const Ty& expr)
	{
		Else();

		size_t label = NewLabelID("");
		expr(*this, label, ctrl_state_.label1);
		L(label);
	}
	void Else()
	{
		AppendJmp(ctrl_state_.label2);
		L(ctrl_state_.label1);
		ctrl_state_.label1 = NewLabelID("");
	}
	void EndIf()
	{
		L(ctrl_state_.label1);
		L(ctrl_state_.label2);

		ctrl_state_ = *ctrl_state_stack_.rbegin();
		ctrl_state_stack_.pop_back();
	}
};

namespace compiler
{
	struct BitVector : std::vector<uint32>
	{
		bool get_bit(size_t idx) const
		{
			const size_t i = idx / 32;
			return i < size() && (at(i) & (1 << (idx % 32))) != 0;
		}

		void set_bit(size_t idx, bool b)
		{
			const size_t i = idx / 32;
			const uint32 mask = (1 << (idx % 32));
			if (i >= size()) resize(i + 1);
			if (b)	at(i) |= mask;
			else	at(i) &= ~mask;
		}

		bool is_equal(const BitVector& rhs) const
		{
			const size_t min_size = size() < rhs.size() ? size() : rhs.size();
			for (size_t i = 0; i < min_size; ++i) {
				if (at(i) != rhs[i]) return false;
			}

			const BitVector& larger = size() < rhs.size() ? rhs : *this;
			for (size_t i = min_size; i < larger.size(); ++i) {
				if (larger[i] != 0) return false;
			}

			return true;
		}

		void set_union(const BitVector& rhs)
		{
			if (size() < rhs.size()) resize(rhs.size());
			for (size_t i = 0; i < rhs.size(); ++i) {
				at(i) |= rhs[i];
			}
		}

		void set_subtract(const BitVector& rhs)
		{
			const size_t min_size = size() < rhs.size() ? size() : rhs.size();
			for (size_t i = 0; i < min_size; ++i) {
				at(i) &= ~rhs[i];
			}
		}
	};

	struct RegUsePoint
	{
		size_t instr_idx;	///< Instruction index offset from basic block start point
		OpdType type;

		RegUsePoint(size_t idx, OpdType t) : instr_idx(idx), type(t) {}
	};

	struct Lifetime
	{
		// x86      x64
		// 0  - 7   0  - 15  General purpose register
		// 8  - 15  16 - 23  MMX register
		// 16 - 23  24 - 39  XMM register
		// 24 - 31  40 - 55  YMM register
		// 32 -     56 -     Symbolic register
		std::vector< std::vector<RegUsePoint> > use_points;
		BitVector gen;				///< The set of variables used before any assignment
		BitVector kill;				///< The set of variables assigned a value before any use
		BitVector live_in;			///< The set of live variables at the start of this block
		BitVector live_out;			///< The set of live variables at the end of this block
		bool dirty_live_out;		///< The dirty flag of live_out

		Lifetime() : use_points(16), dirty_live_out(true) {}

		void AddUsePoint(size_t instr_idx, const RegID& reg, OpdType opd_type)
		{
			const size_t reg_idx = reg.IsSymbolic() ? reg.id + 16 : reg.id;
			if (use_points.size() <= reg_idx)
				use_points.resize(reg_idx + 1);

			use_points[reg_idx].push_back(RegUsePoint(instr_idx, opd_type));
		}

		template<class Fn>
		void EnumLifetimeInterval(Fn& fn)
		{
			BitVector liveness = live_in;
			size_t instr_idx = 0;
			fn(instr_idx, liveness);

		}

		static std::string GetRegName(RegType type, size_t reg_idx)
		{
#ifdef JITASM64
			const static std::string s_gp_reg_name[] = {"rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"};
#else
			const static std::string s_gp_reg_name[] = {"eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi"};
#endif
			std::string name;
			if (reg_idx >= 16) {
				name.assign("sym");
				detail::append_num(name, reg_idx - 16);
			} else if (type == R_TYPE_GP) {
				return s_gp_reg_name[reg_idx];
			} else if (type == R_TYPE_MMX) {
				name.assign("mm");
				detail::append_num(name, reg_idx);
			} else if (type == R_TYPE_XMM) {
				name.assign("xmm");
				detail::append_num(name, reg_idx);
			} else if (type == R_TYPE_YMM) {
				name.assign("ymm");
				detail::append_num(name, reg_idx);
			}
			return name;
		}
	};

	struct BasicBlock
	{
		BasicBlock *successor[2];
		std::vector<BasicBlock *> predecessor;
		size_t instr_begin;					///< Begin instruction index of the basic block (inclusive)
		size_t instr_end;					///< End instruction index of the basic block (exclusive)
		size_t depth;						///< Depth-first order of Control flow
		size_t dfs_instr_begin;				///< Begin instruction index in depth-first order
		BasicBlock *dfs_parent;				///< Depth-first search tree parent
		BasicBlock *immediate_dominator;	///< Immediate dominator
		size_t loop_depth;					///< Loop nesting depth
		Lifetime lifetime[4];				///< Variable lifetime (0: GP, 1: MMX, 2: XMM, 3: YMM)

		BasicBlock(size_t instr_begin_, size_t instr_end_, BasicBlock *successor0 = NULL, BasicBlock *successor1 = NULL) : instr_begin(instr_begin_), instr_end(instr_end_), depth((size_t)-1), dfs_instr_begin(0), dfs_parent(NULL), immediate_dominator(NULL), loop_depth(0) {
			successor[0] = successor0;
			successor[1] = successor1;
		}

		bool operator<(const BasicBlock& rhs) const { return instr_begin < rhs.instr_begin; }

		void remove_predecessor(BasicBlock *block) {
			std::vector<BasicBlock *>::iterator it = std::find(predecessor.begin(), predecessor.end(), block);
			if (it != predecessor.end())
				predecessor.erase(it);
		}

		bool replace_predecessor(BasicBlock *old_pred, BasicBlock *new_pred) {
			std::vector<BasicBlock *>::iterator it = std::find(predecessor.begin(), predecessor.end(), old_pred);
			if (it != predecessor.end()) {
				*it = new_pred;
				return true;
			}
			return false;
		}

		bool is_dominated(BasicBlock *block) const {
			if (block == this) return true;
			return immediate_dominator ? immediate_dominator->is_dominated(block) : false;
		}
	};

	/**
	 * The Lengauer-Tarjan algorithm
	 */
	class DominatorFinder
	{
	private:
		std::vector<size_t> sdom_;			// semidominator
		std::vector<size_t> ancestor_;
		std::vector<size_t> best_;

		void Link(size_t v, size_t w)
		{
			ancestor_[w] = v;
		}

#if 1
		size_t Eval(size_t v)
		{
			size_t a = ancestor_[v];
			while (ancestor_[a] != 0) {
				if (sdom_[v] > sdom_[a])
					v = a;
				a = ancestor_[a];
			}
			return v;
		}
#else
		size_t Eval(size_t v)
		{
			if (ancestor_[v] == 0) return v;
			Compress(v);
			return best_[v];
		}
#endif

		void Compress(size_t v)
		{
			size_t a = ancestor_[v];
			if (ancestor_[a] == 0)
				return;

			Compress(a);

			if (sdom_[best_[v]] > sdom_[best_[a]])
				best_[v] = best_[a];

			ancestor_[v] = ancestor_[a];
		}

	public:
		void operator()(std::deque<BasicBlock *>& depth_first_blocks)
		{
			const size_t num_of_nodes = depth_first_blocks.size();
			if (num_of_nodes == 0) return;

			// initialize
			sdom_.resize(num_of_nodes);			// semidominator
			ancestor_.clear();
			ancestor_.resize(num_of_nodes);
			best_.resize(num_of_nodes);
			std::vector< std::vector<size_t> > bucket(num_of_nodes);
			std::vector<size_t> dom(num_of_nodes);
			for (size_t i = 0; i < num_of_nodes; ++i) {
				sdom_[i] = i;
				best_[i] = i;
			}

			for (size_t w = num_of_nodes - 1; w > 0; --w) {
				BasicBlock *wb = depth_first_blocks[w];
				size_t p = wb->dfs_parent->depth;

				// Compute the semidominator
				for (std::vector<BasicBlock *>::iterator v = wb->predecessor.begin(); v != wb->predecessor.end(); ++v) {
					if ((*v)->depth != (size_t)-1) {	// skip out of DFS tree
						size_t u = Eval((*v)->depth);
						if (sdom_[u] < sdom_[w])
							sdom_[w] = sdom_[u];
					}
				}
				bucket[sdom_[w]].push_back(w);
				Link(p, w);

				// Implicity compute immediate dominator
				for (std::vector<size_t>::iterator v = bucket[p].begin(); v != bucket[p].end(); ++v) {
					size_t u = Eval(*v);
					dom[*v] = sdom_[u] < sdom_[*v] ? u : p;
				}
				bucket[p].clear();
			}

			// Explicity compute immediate dominator
			for (size_t w = 1; w < num_of_nodes; ++w) {
				if (dom[w] != sdom_[w])
					dom[w] = dom[dom[w]];
				depth_first_blocks[w]->immediate_dominator = depth_first_blocks[dom[w]];
			}
			depth_first_blocks[0]->immediate_dominator = NULL;
		}
	};

	class ControlFlowGraph
	{
	public:
		typedef std::set<BasicBlock> BlockList;

	private:
		BlockList blocks_;
		std::deque<BasicBlock *> depth_first_blocks_;

		void MakeDepthFirstBlocks(BasicBlock *block)
		{
			block->depth = 0;	// mark "visited"
			for (size_t i = 0; i < 2; ++i) {
				BasicBlock *s = block->successor[i];
				if (s && s->depth != 0) {
					s->dfs_parent = block;
					MakeDepthFirstBlocks(s);
				}
			}
			depth_first_blocks_.push_front(block);
		}

		void DetectLoops()
		{
			// make dominator tree
			DominatorFinder dom_finder;
			dom_finder(depth_first_blocks_);

			// identify backedges
			std::vector< std::pair<size_t, size_t> > backedges;
			for (size_t i = 0; i < depth_first_blocks_.size(); ++i) {
				BasicBlock *block = depth_first_blocks_[i];
				for (size_t j = 0; j < 2; ++j) {
					if (block->successor[j] && block->depth > block->successor[j]->depth) {		// retreating edge
						if (block->is_dominated(block->successor[j])) {
							backedges.push_back(std::make_pair(block->depth, block->successor[j]->depth));
						}
					}
				}
			}

			// merge loops with the same loop header
			struct sort_backedge {
				bool operator()(const std::pair<size_t, size_t>& lhs, const std::pair<size_t, size_t>& rhs) const {
					if (lhs.second < rhs.second) return true;						// smaller depth loop header first
					if (lhs.second == rhs.second) return lhs.first > rhs.first;		// larger depth of end of loop first if same loop header
					return false;
				}
			};
			std::sort(backedges.begin(), backedges.end(), sort_backedge());
			if (backedges.size() >= 2) {
				std::vector< std::pair<size_t, size_t> >::iterator it = backedges.begin() + 1;
				while (it != backedges.end()) {
					if (detail::prior(it)->second == it->second) {
						// erase backedge of smaller loop
						it = backedges.erase(it);
					} else {
						++it;
					}
				}
			}

			// set loop depth
			for (std::vector< std::pair<size_t, size_t> >::iterator it = backedges.begin(); it != backedges.end(); ++it) {
				for (size_t i = it->second; i <= it->first; ++i) {
					depth_first_blocks_[i]->loop_depth++;
				}
			}
		}

	public:
		BlockList::iterator initialize(size_t num_of_instructions) {
			blocks_.clear();
			depth_first_blocks_.clear();
			BlockList::iterator enter_block = blocks_.insert(BasicBlock(0, num_of_instructions)).first;
			if (num_of_instructions > 0) {
				BlockList::iterator exit_block = blocks_.insert(BasicBlock(num_of_instructions, num_of_instructions)).first;	// exit block
				enter_block->successor[0] = &*exit_block;
				exit_block->predecessor.push_back(&*enter_block);
			}
			return enter_block;
		}

		BlockList::iterator split(BlockList::iterator target_block, size_t instr_idx) {
			if (target_block->instr_begin == instr_idx)
				return target_block;

			BlockList::iterator new_block = blocks_.insert(detail::next(target_block), BasicBlock(instr_idx, target_block->instr_end));
			ASSERT(detail::next(target_block) == new_block);
			new_block->successor[0] = target_block->successor[0];
			new_block->successor[1] = target_block->successor[1];
			new_block->predecessor.push_back(&*target_block);
			target_block->successor[0] = &*new_block;
			target_block->successor[1] = NULL;
			target_block->instr_end = instr_idx;

			// replace predecessor of successors
			if (new_block->successor[0]) new_block->successor[0]->replace_predecessor(&*target_block, &*new_block);
			if (new_block->successor[1]) new_block->successor[1]->replace_predecessor(&*target_block, &*new_block);

			return new_block;
		}

		BlockList::iterator get_block(size_t instr_idx) {
			BlockList::iterator it = blocks_.upper_bound(BasicBlock(instr_idx, instr_idx));
			return it != blocks_.begin() ? --it : blocks_.end();
		}

		BlockList::iterator get_exit_block() {
			BlockList::iterator it = blocks_.end();
			return --it;
		}

		size_t size() { return blocks_.size(); }

		BlockList::iterator begin() { return blocks_.begin(); }
		BlockList::iterator end() { return blocks_.end(); }

		void DumpDot() const
		{
			printf("digraph CFG {\n");
			printf("\tnode[shape=box];\n");
			for (BlockList::const_iterator it = blocks_.begin(); it != blocks_.end(); ++it) {
				std::string live_in = "live in:";
				for (size_t i = 0; i < it->lifetime[0].live_in.size() * 32; ++i) {
					if (it->lifetime[0].live_in.get_bit(i)) {
						live_in.append(" ");
						live_in.append(Lifetime::GetRegName(R_TYPE_GP, i));
					}
				}
				std::string live_out = "live out:";
				for (size_t i = 0; i < it->lifetime[0].live_out.size() * 32; ++i) {
					if (it->lifetime[0].live_out.get_bit(i)) {
						live_out.append(" ");
						live_out.append(Lifetime::GetRegName(R_TYPE_GP, i));
					}
				}
				printf("\tnode%d[label=\"Block%d\\ninstruction %d - %d\\nloop depth %d\\n%s\\n%s\"];\n", it->instr_begin, it->depth, it->instr_begin, it->instr_end - 1, it->loop_depth, live_in.c_str(), live_out.c_str());
				if (it->successor[0]) printf("\t\"node%d\" -> \"node%d\" [constraint=false];\n", it->instr_begin, it->successor[0]->instr_begin);
				if (it->successor[1]) printf("\t\"node%d\" -> \"node%d\" [constraint=false];\n", it->instr_begin, it->successor[1]->instr_begin);
				if (it->dfs_parent) printf("\t\"node%d\" -> \"node%d\" [color=\"#ff0000\"];\n", it->instr_begin, it->dfs_parent->instr_begin);
				if (it->immediate_dominator) printf("\t\"node%d\" -> \"node%d\" [constraint=false, color=\"#0000ff\"];\n", it->instr_begin, it->immediate_dominator->instr_begin);
				//for (size_t i = 0; i < it->predecessor.size(); ++i) {
				//	printf("\t\"node%d\" -> \"node%d\" [constraint=false, color=\"#808080\"];\n", it->instr_begin, it->predecessor[i]->instr_begin);
				//}
			}
			printf("}\n");
		}

		void Build(const Frontend& f)
		{
			typedef BlockList::iterator BlockIterator;
			BlockIterator cur_block = initialize(f.instrs_.size());
			for (Frontend::InstrList::const_iterator it = f.instrs_.begin(); it != f.instrs_.end(); ++it) {
				InstrID instr_id = it->GetID();
				if (Frontend::IsJump(instr_id) || instr_id == I_RET || instr_id == I_IRET) {
					// jump instruction always terminate basic block
					const size_t instr_idx = std::distance(f.instrs_.begin(), it);
					BlockIterator next_block;
					if (instr_idx + 1 < cur_block->instr_end) {
						// split basic block
						next_block = split(cur_block, instr_idx + 1);
					}
					else {
						// already splitted
						next_block = detail::next(cur_block);
					}

					// set successors of current block
					if (instr_id == I_RET || instr_id == I_IRET) {
						if (cur_block->successor[0])
							cur_block->successor[0]->remove_predecessor(&*cur_block);
						cur_block->successor[0] = &*get_exit_block();
						get_exit_block()->predecessor.push_back(&*cur_block);
					}
					else {
						const size_t jump_to = f.GetJumpTo(*it);	// jump target instruction index
						BlockIterator jump_target = split(get_block(jump_to), jump_to);
						if (instr_id == I_JMP) {
							if (cur_block->successor[0])
								cur_block->successor[0]->remove_predecessor(&*cur_block);
							cur_block->successor[0] = &*jump_target;
							jump_target->predecessor.push_back(&*cur_block);
						}
						else {
							ASSERT(instr_id == I_JCC || instr_id == I_LOOP);
							if (cur_block->successor[1])
								cur_block->successor[1]->remove_predecessor(&*cur_block);
							cur_block->successor[1] = &*jump_target;
							jump_target->predecessor.push_back(&*cur_block);
						}
					}

					cur_block = next_block;
				}
			}

			// make depth first orderd list
			MakeDepthFirstBlocks(&*get_block(0));

			// numbering depth and set dfs_instr_begin
			size_t dfs_instr_begin = 0;
			for (size_t i = 0; i < depth_first_blocks_.size(); ++i) {
				depth_first_blocks_[i]->depth = i;
				depth_first_blocks_[i]->dfs_instr_begin = dfs_instr_begin;
				dfs_instr_begin += depth_first_blocks_[i]->instr_end - depth_first_blocks_[i]->instr_begin;
			}

			// detect loops
			DetectLoops();
		}
	};

	/// Re-number symbolic register ID
	/**
	 * \return Number of symbolic registers
	 */
	inline int NormalizeSymbolicReg(Frontend& f)
	{
		struct RegIDMap {
			int next_id_;
			std::map<int, int> id_map_;
			RegIDMap() : next_id_(0) {}
			int GetNewID(int id) {
				std::map<int, int>::iterator it = id_map_.find(id);
				if (it != id_map_.end())
					return it->second;
				int new_id = next_id_++;
				id_map_.insert(std::pair<int, int>(id, new_id));
				return new_id;
			}
		} reg_id_map;

		for (Frontend::InstrList::iterator it = f.instrs_.begin(); it != f.instrs_.end(); ++it) {
			for (size_t i = 0; i < Instr::MAX_OPERAND_COUNT; ++i) {
				detail::Opd& opd = it->GetOpd(i);
				if (opd.IsReg()) {
					RegID reg = opd.GetReg();
					if (reg.IsSymbolic())
						opd.reg_.id = reg_id_map.GetNewID(reg.id);
				} else if (opd.IsMem()) {
					RegID base = opd.GetBase();
					if (base.IsSymbolic())
						opd.base_.id = reg_id_map.GetNewID(base.id);
					RegID index = opd.GetIndex();
					if (index.IsSymbolic())
						opd.index_.id = reg_id_map.GetNewID(index.id);
				}
			}
		}

		return reg_id_map.next_id_;		// Number of symbolic registers
	}

	inline void LiveVariableAnalysis(const Frontend& f, ControlFlowGraph& cfg)
	{
		std::vector<BasicBlock *> update_target;
		update_target.reserve(cfg.size());

		for (ControlFlowGraph::BlockList::iterator it = cfg.begin(); it != cfg.end(); ++it) {
			// Scanning instructions of basic block and make register lifetime table
			for (size_t i = it->instr_begin; i != it->instr_end; ++i) {
				const size_t instr_offset = i - it->instr_begin;
				for (size_t j = 0; j < Instr::MAX_OPERAND_COUNT; ++j) {
					const detail::Opd& opd = f.instrs_[i].GetOpd(j);
					if (opd.IsGpReg()) {
						it->lifetime[0].AddUsePoint(instr_offset, opd.GetReg(), opd.opdtype_);
					} else if (opd.IsMmxReg()) {
						it->lifetime[1].AddUsePoint(instr_offset, opd.GetReg(), opd.opdtype_);
					} else if (opd.IsXmmReg()) {
						it->lifetime[2].AddUsePoint(instr_offset, opd.GetReg(), opd.opdtype_);
					} else if (opd.IsYmmReg()) {
						it->lifetime[3].AddUsePoint(instr_offset, opd.GetReg(), opd.opdtype_);
					} else if (opd.IsMem()) {
						RegID base = opd.GetBase();
						if (!base.IsInvalid())
							it->lifetime[0].AddUsePoint(instr_offset, base, static_cast<OpdType>(O_TYPE_REG | O_TYPE_READ));
						RegID index = opd.GetIndex();
						if (!index.IsInvalid())
							it->lifetime[0].AddUsePoint(instr_offset, index, static_cast<OpdType>(O_TYPE_REG | O_TYPE_READ));
					}
				}
			}

			// Make GEN and KILL set
			for (size_t reg_type = 0; reg_type < 4; ++reg_type) {
				const size_t num_of_used_reg = it->lifetime[reg_type].use_points.size();
				for (size_t i = 0; i < num_of_used_reg; ++i) {
					if (!it->lifetime[reg_type].use_points[i].empty()) {
						OpdType type = it->lifetime[reg_type].use_points[i][0].type;
						if (type & O_TYPE_READ) {
							it->lifetime[reg_type].gen.set_bit(i, true);	// GEN
						} else {
							ASSERT(type & O_TYPE_WRITE);
							it->lifetime[reg_type].kill.set_bit(i, true);	// KILL
						}
					}
				}
			}

			update_target.push_back(&*it);
		}

		while (!update_target.empty()) {
			BasicBlock *block = update_target.back();
			update_target.pop_back();
			for (size_t reg_type = 0; reg_type < 4; ++reg_type) {
				Lifetime& lifetime = block->lifetime[reg_type];
				if (lifetime.dirty_live_out) {
					// live_out is the union of the live_in of the successors
					for (size_t i = 0; i < 2; ++i) {
						if (block->successor[i])
							lifetime.live_out.set_union(block->successor[i]->lifetime[reg_type].live_in);
					}
					lifetime.dirty_live_out = false;

					// live_in = gen OR (live_out - kill)
					BitVector new_live_in = lifetime.live_out;
					new_live_in.set_subtract(lifetime.kill);
					new_live_in.set_union(lifetime.gen);

					if (!lifetime.live_in.is_equal(new_live_in)) {
						lifetime.live_in.swap(new_live_in);

						for (size_t i = 0; i < block->predecessor.size(); ++i) {
							block->predecessor[i]->lifetime[reg_type].dirty_live_out = true;
							update_target.push_back(block->predecessor[i]);
						}
					}
				}
			}
		}
	}

	inline bool LinearScanRegisterAlloc(Frontend& f)
	{
		int num_of_sym_reg = NormalizeSymbolicReg(f);
		if (num_of_sym_reg == 0)
			return true;	// no need to register allocation

		ControlFlowGraph cfg;
		cfg.Build(f);

		LiveVariableAnalysis(f, cfg);

		cfg.DumpDot();

		std::vector< std::pair<size_t, size_t> > live_count;

		// Spill identification

		// Spill resurrection

		return true;
	}

}	// namespace compiler

namespace detail
{
	struct CondExpr {
		virtual void operator()(Frontend& f, size_t beg, size_t end) const = 0;
	};

	// &&
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
		CondExpr_ExprAnd& operator=(const CondExpr_ExprAnd&);
	};

	// ||
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
		CondExpr_ExprOr& operator=(const CondExpr_ExprOr&);
	};

	// cmp
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

	// or
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
		ResultT() {}
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
		ResultT() {}
		ResultT(const Opd8& val) : val_(val) {}
		ResultT(uint8 imm) : val_(Imm8(imm)) {}
		void Store(Frontend& f)
		{
			if (val_.IsGpReg()) {
				if (val_.GetReg().id != AL)
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
		ResultT() {}
		ResultT(const Opd16& val) : val_(val) {}
		ResultT(uint16 imm) : val_(Imm16(imm)) {}
		void Store(Frontend& f)
		{
			if (val_.IsGpReg()) {
				if (val_.GetReg().id != AX)
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
		ResultT() {}
		ResultT(const Opd32& val) : val_(val) {}
		ResultT(uint32 imm) : val_(Imm32(imm)) {}
		void Store(Frontend& f)
		{
			if (val_.IsGpReg()) {
				if (val_.GetReg().id != EAX)
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
		ResultT() {}
		ResultT(const Opd64& val) : val_(val) {}
		ResultT(uint64 imm) : val_(Imm64(imm)) {}
		void Store(Frontend& f)
		{
#ifdef JITASM64
			if (val_.IsGpReg()) {
				if (val_.GetReg().id != RAX)
					f.mov(f.rax, static_cast<Reg64&>(val_));
			}
			else if (val_.IsMem()) {
				f.mov(f.rax, static_cast<Mem64&>(val_));
			}
			else if (val_.IsImm()) {
				f.mov(f.rax, static_cast<Imm64&>(val_));
			}
			else if (val_.IsMmxReg()) {
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
		detail::Opd val_;
		ResultT() {}
		ResultT(const FpuReg& fpu) : val_(fpu) {}
		ResultT(const Mem32& mem) : val_(mem) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const float imm) : val_(Imm32(*(uint32*)&imm)) {}
		void Store(Frontend& f)
		{
#ifdef JITASM64
			if (val_.IsFpuReg()) {
				// from FPU register
				f.fstp(f.real4_ptr[f.rsp - 4]);
				f.movss(f.xmm0, f.dword_ptr[f.rsp - 4]);
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_32) {
				// from memory
				f.movss(f.xmm0, static_cast<Mem32&>(val_));
			}
			else if (val_.IsXmmReg()) {
				// from XMM register
				if (val_.GetReg().id != XMM0)
					f.movss(f.xmm0, static_cast<XmmReg&>(val_));
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.rsp - 4], static_cast<Imm32&>(val_));
				f.movss(f.xmm0, f.dword_ptr[f.rsp - 4]);
			}
#else
			if (val_.IsFpuReg()) {
				// from FPU register
				if (val_.GetReg().id != ST0)
					f.fld(static_cast<FpuReg&>(val_));
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_32) {
				// from memory
				f.fld(static_cast<Mem32&>(val_));
			}
			else if (val_.IsXmmReg()) {
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
		detail::Opd val_;
		double imm_;
		ResultT() {}
		ResultT(const FpuReg& fpu) : val_(fpu) {}
		ResultT(const Mem64& mem) : val_(mem) {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const double imm) : val_(Imm32(0)), imm_(imm) {}
		void Store(Frontend& f)
		{
#ifdef JITASM64
			if (val_.IsFpuReg()) {
				// from FPU register
				f.fstp(f.real8_ptr[f.rsp - 8]);
				f.movsd(f.xmm0, f.qword_ptr[f.rsp - 8]);
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_64) {
				// from memory
				f.movsd(f.xmm0, static_cast<Mem64&>(val_));
			}
			else if (val_.IsXmmReg()) {
				// from XMM register
				if (val_.GetReg().id != XMM0)
					f.movsd(f.xmm0, static_cast<XmmReg&>(val_));
			}
			else if (val_.IsImm()) {
				// from float immediate
				f.mov(f.dword_ptr[f.rsp - 8], *reinterpret_cast<uint32*>(&imm_));
				f.mov(f.dword_ptr[f.rsp - 4], *(reinterpret_cast<uint32*>(&imm_) + 1));
				f.movsd(f.xmm0, f.qword_ptr[f.rsp - 8]);
			}
#else
			if (val_.IsFpuReg()) {
				// from FPU register
				if (val_.GetReg().id != ST0)
					f.fld(static_cast<FpuReg&>(val_));
			}
			else if (val_.IsMem() && val_.GetSize() == O_SIZE_64) {
				// from memory
				f.fld(static_cast<Mem64&>(val_));
			}
			else if (val_.IsXmmReg()) {
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
		ResultT() {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			f.mov(f.zax, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
			if (val_.IsXmmReg()) {
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
		ResultT() {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			f.mov(f.zax, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
			if (val_.IsXmmReg()) {
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
		ResultT() {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			f.mov(f.zax, f.zword_ptr[f.zbp + sizeof(void *) * 2]);
			if (val_.IsXmmReg()) {
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
		ResultT() {}
		ResultT(const MmxReg& mm) : val_(mm) {}
		ResultT(const Mem64& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (val_.IsMmxReg()) {
				if (val_.GetReg().id != MM0)
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
		ResultT() {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (val_.IsXmmReg()) {
				if (val_.GetReg().id != XMM0)
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
		ResultT() {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (val_.IsXmmReg()) {
				if (val_.GetReg().id != XMM0)
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
		ResultT() {}
		ResultT(const XmmReg& xmm) : val_(xmm) {}
		ResultT(const Mem128& mem) : val_(mem) {}
		void Store(Frontend& f)
		{
			if (val_.IsXmmReg()) {
				if (val_.GetReg().id != XMM0)
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
		{avoid_unused_warn(dump_regarg_x64);}
#endif

	private:
		template<int N, class T>
		void CopyRegArgToStack(const Arg& addr, bool bForceCopy)
		{
			avoid_unused_warn(addr);
			avoid_unused_warn(bForceCopy);
#ifdef JITASM64
			if (dump_regarg_x64_ || bForceCopy) {
				if (ArgTraits<N, T>::flag & ARG_IN_REG)
					mov(qword_ptr[addr], Reg64(static_cast<PhysicalRegID>(ArgTraits<N, T>::reg_id)));
				else if (ArgTraits<N, T>::flag & ARG_IN_XMM_SP)
					movss(dword_ptr[addr], XmmReg(static_cast<PhysicalRegID>(ArgTraits<N, T>::reg_id)));
				else if (ArgTraits<N, T>::flag & ARG_IN_XMM_DP)
					movsd(qword_ptr[addr], XmmReg(static_cast<PhysicalRegID>(ArgTraits<N, T>::reg_id)));
			}
#endif
		}

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

	// specialization for void
	template<>
	inline Function_cdecl::Arg Function_cdecl::DumpRegArg0<void>()
	{
		return Arg(zbp + sizeof(void *) * 2);
	}

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

		compiler::LinearScanRegisterAlloc(*this);
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
