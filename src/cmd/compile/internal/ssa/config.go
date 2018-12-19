// Copyright 2015 The Go Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package ssa

import (
	"cmd/compile/internal/types"
	"cmd/internal/obj"
	"cmd/internal/objabi"
	"cmd/internal/src"
)

// A Config holds readonly compilation information.
// It is created once, early during compilation,
// and shared across all compilations.
type Config struct {
	arch           string // "amd64", etc.
	PtrSize        int64  // 4 or 8; copy of cmd/internal/sys.Arch.PtrSize
	RegSize        int64  // 4 or 8; copy of cmd/internal/sys.Arch.RegSize
	Types          Types
	lowerBlock     blockRewriter // lowering function
	lowerValue     valueRewriter // lowering function
	registers      []Register    // machine registers
	gpRegMask      regMask       // general purpose integer register mask
	fpRegMask      regMask       // floating point register mask
	specialRegMask regMask       // special register mask
	GCRegMap       []*Register   // garbage collector register map, by GC register index
	FPReg          int8          // register number of frame pointer, -1 if not used
	LinkReg        int8          // register number of link register if it is a general purpose register, -1 if not used
	hasGReg        bool          // has hardware g register
	ctxt           *obj.Link     // Generic arch information
	optimize       bool          // Do optimization
	noDuffDevice   bool          // Don't use Duff's device
	useSSE         bool          // Use SSE for non-float operations
	useAvg         bool          // Use optimizations that need Avg* operations
	useHmul        bool          // Use optimizations that need Hmul* operations
	nacl           bool          // GOOS=nacl
	use387         bool          // GO386=387
	SoftFloat      bool          //
	NeedsFpScratch bool          // No direct move between GP and FP register sets
	BigEndian      bool          //
}

type (
	blockRewriter func(*Block) bool
	valueRewriter func(*Value) bool
)

type Types struct {
	Bool       *types.Type
	Int8       *types.Type
	Int16      *types.Type
	Int32      *types.Type
	Int64      *types.Type
	UInt8      *types.Type
	UInt16     *types.Type
	UInt32     *types.Type
	UInt64     *types.Type
	Int        *types.Type
	Float32    *types.Type
	Float64    *types.Type
	UInt       *types.Type
	Uintptr    *types.Type
	String     *types.Type
	BytePtr    *types.Type // TODO: use unsafe.Pointer instead?
	Int32Ptr   *types.Type
	UInt32Ptr  *types.Type
	IntPtr     *types.Type
	UintptrPtr *types.Type
	Float32Ptr *types.Type
	Float64Ptr *types.Type
	BytePtrPtr *types.Type
}

// NewTypes creates and populates a Types.
func NewTypes() *Types {
	t := new(Types)
	t.SetTypPtrs()
	return t
}

// SetTypPtrs populates t.
func (t *Types) SetTypPtrs() {
	t.Bool = types.Types[types.TBOOL]
	t.Int8 = types.Types[types.TINT8]
	t.Int16 = types.Types[types.TINT16]
	t.Int32 = types.Types[types.TINT32]
	t.Int64 = types.Types[types.TINT64]
	t.UInt8 = types.Types[types.TUINT8]
	t.UInt16 = types.Types[types.TUINT16]
	t.UInt32 = types.Types[types.TUINT32]
	t.UInt64 = types.Types[types.TUINT64]
	t.Int = types.Types[types.TINT]
	t.Float32 = types.Types[types.TFLOAT32]
	t.Float64 = types.Types[types.TFLOAT64]
	t.UInt = types.Types[types.TUINT]
	t.Uintptr = types.Types[types.TUINTPTR]
	t.String = types.Types[types.TSTRING]
	t.BytePtr = types.NewPtr(types.Types[types.TUINT8])
	t.Int32Ptr = types.NewPtr(types.Types[types.TINT32])
	t.UInt32Ptr = types.NewPtr(types.Types[types.TUINT32])
	t.IntPtr = types.NewPtr(types.Types[types.TINT])
	t.UintptrPtr = types.NewPtr(types.Types[types.TUINTPTR])
	t.Float32Ptr = types.NewPtr(types.Types[types.TFLOAT32])
	t.Float64Ptr = types.NewPtr(types.Types[types.TFLOAT64])
	t.BytePtrPtr = types.NewPtr(types.NewPtr(types.Types[types.TUINT8]))
}

type Logger interface {
	// Logf logs a message from the compiler.
	Logf(string, ...interface{})

	// Log returns true if logging is not a no-op
	// some logging calls account for more than a few heap allocations.
	Log() bool

	// Fatal reports a compiler error and exits.
	Fatalf(pos src.XPos, msg string, args ...interface{})

	// Warnl writes compiler messages in the form expected by "errorcheck" tests
	Warnl(pos src.XPos, fmt_ string, args ...interface{})

	// Forwards the Debug flags from gc
	Debug_checknil() bool
}

type Frontend interface {
	CanSSA(t *types.Type) bool

	Logger

	// StringData returns a symbol pointing to the given string's contents.
	StringData(string) interface{} // returns *gc.Sym

	// Auto returns a Node for an auto variable of the given type.
	// The SSA compiler uses this function to allocate space for spills.
	Auto(src.XPos, *types.Type) GCNode

	// Given the name for a compound type, returns the name we should use
	// for the parts of that compound type.
	SplitString(LocalSlot) (LocalSlot, LocalSlot)
	SplitInterface(LocalSlot) (LocalSlot, LocalSlot)
	SplitSlice(LocalSlot) (LocalSlot, LocalSlot, LocalSlot)
	SplitComplex(LocalSlot) (LocalSlot, LocalSlot)
	SplitStruct(LocalSlot, int) LocalSlot
	SplitArray(LocalSlot) LocalSlot              // array must be length 1
	SplitInt64(LocalSlot) (LocalSlot, LocalSlot) // returns (hi, lo)

	// DerefItab dereferences an itab function
	// entry, given the symbol of the itab and
	// the byte offset of the function pointer.
	// It may return nil.
	DerefItab(sym *obj.LSym, offset int64) *obj.LSym

	// Line returns a string describing the given position.
	Line(src.XPos) string

	// AllocFrame assigns frame offsets to all live auto variables.
	AllocFrame(f *Func)

	// Syslook returns a symbol of the runtime function/variable with the
	// given name.
	Syslook(string) *obj.LSym

	// UseWriteBarrier returns whether write barrier is enabled
	UseWriteBarrier() bool

	// SetWBPos indicates that a write barrier has been inserted
	// in this function at position pos.
	SetWBPos(pos src.XPos)
}

// interface used to hold a *gc.Node (a stack variable).
// We'd use *gc.Node directly but that would lead to an import cycle.
type GCNode interface {
	Typ() *types.Type
	String() string
	IsSynthetic() bool
	StorageClass() StorageClass
}

type StorageClass uint8

const (
	ClassAuto     StorageClass = iota // local stack variable
	ClassParam                        // argument
	ClassParamOut                     // return value
)

// NewConfig returns a new configuration object for the given architecture.
func NewConfig(arch string, types Types, ctxt *obj.Link, optimize bool) *Config {
	c := &Config{arch: arch, Types: types}
	c.useAvg = true
	c.useHmul = true
	switch arch {
	case "amd64":
		c.PtrSize = 8
		c.RegSize = 8
		c.lowerBlock = rewriteBlockAMD64
		c.lowerValue = rewriteValueAMD64
		c.registers = registersAMD64[:]
		c.gpRegMask = gpRegMaskAMD64
		c.fpRegMask = fpRegMaskAMD64
		c.FPReg = framepointerRegAMD64
		c.LinkReg = linkRegAMD64
		c.hasGReg = false
	default:
		ctxt.Diag("arch %s not implemented", arch)
	}
	c.ctxt = ctxt
	c.optimize = optimize
	c.nacl = objabi.GOOS == "nacl"
	c.useSSE = true

	// Don't use Duff's device nor SSE on Plan 9 AMD64, because
	// floating point operations are not allowed in note handler.
	if objabi.GOOS == "plan9" && arch == "amd64" {
		c.noDuffDevice = true
		c.useSSE = false
	}

	if c.nacl {
		c.noDuffDevice = true // Don't use Duff's device on NaCl

		// Returns clobber BP on nacl/386, so the write
		// barrier does.
		opcodeTable[Op386LoweredWB].reg.clobbers |= 1 << 5 // BP

		// ... and SI on nacl/amd64.
		opcodeTable[OpAMD64LoweredWB].reg.clobbers |= 1 << 6 // SI
	}

	if ctxt.Flag_shared {
		// LoweredWB is secretly a CALL and CALLs on 386 in
		// shared mode get rewritten by obj6.go to go through
		// the GOT, which clobbers BX.
		opcodeTable[Op386LoweredWB].reg.clobbers |= 1 << 3 // BX
	}

	// Create the GC register map index.
	// TODO: This is only used for debug printing. Maybe export config.registers?
	gcRegMapSize := int16(0)
	for _, r := range c.registers {
		if r.gcNum+1 > gcRegMapSize {
			gcRegMapSize = r.gcNum + 1
		}
	}
	c.GCRegMap = make([]*Register, gcRegMapSize)
	for i, r := range c.registers {
		if r.gcNum != -1 {
			c.GCRegMap[r.gcNum] = &c.registers[i]
		}
	}

	return c
}

func (c *Config) Set387(b bool) {
	c.NeedsFpScratch = b
	c.use387 = b
}

func (c *Config) Ctxt() *obj.Link { return c.ctxt }
