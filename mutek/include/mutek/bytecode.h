/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
 */

#ifndef MUTEK_BYTECODE_H_
#define MUTEK_BYTECODE_H_

#include <hexo/types.h>
#include <hexo/decls.h>
#include <stdarg.h>
#include <assert.h>
#include <inttypes.h>

/**
   @file
   @module {Core::Kernel services}
   @short Generic bytecode
   @index {Generic bytecode} {Kernel services}

   This kernel service provides a simple and small bytecode virtual
   machine with a customisable instruction set. A set of generic
   instructions is provided which can be extended.

   The virtual machine state contains a program counter register as
   well as 16 general purpose registers. Opcodes words are 16 bits
   wide. The most significant bit of the opcode word is always zero
   for generic instructions, leaving half of the opcode space for
   custom opcodes.

   Once the virtual machine state has benn initialized using the @ref
   bc_init function, the @ref bc_run function can start execution of
   the bytecode. It will return on the first encountered custom
   instruction, leaving the caller with the task of performing the
   custom operation. This makes the meaning of custom instructions
   context dependent; different parts of the kernel and application
   can have there own interpretation of custom bytecode instructions.

   The bytecode program can be resumed at any time once a custom
   instruction has been handled. The C code is free to access the
   virtual registers and change the value of the program counter to
   point at any entry point.

   Relying on bytecode is useful in various situations:

   @list
     @item A bytecode program may be executed in a sandbox
       or live in the same memory space as the C code.
     @item Repetitive operations requiring complex processing or
       complex invocation can be encoded as a single 16 bits opcode.
     @item When a simple program using a specific set of blocking
       operations needs to be executed, custom instructions may
       actually be implemented using asynchronous operations. The
       bytecode may be resumed when the operation terminates.
     @item The resumable bytecode has the advantage of requiring
       less than 100 bytes of execution state on a 32 bits
       architecture. It does not require a dedicated thread and
       associated execution stack in order to implement
       blocking operations.
   @end list

   When a processor specific backend is available, bytecode programs
   which does not require sandboxing can be translated into machine
   code at compile time rather than relying on the virtual
   machine. This requires definition of the @ref
   #CONFIG_MUTEK_BYTECODE_NATIVE token. This allows fast execution
   while retaining the resumable bytecode feature.

   @section {Bytecode debug and trace}
   Bytecode instructions and C functions are available to dump the
   virtual machine state and enable execution trace. This requires
   definition of the @ref #CONFIG_MUTEK_BYTECODE_DEBUG and
   @ref #CONFIG_MUTEK_BYTECODE_TRACE tokens.
   @see bc_set_trace @see bc_dump
   @end section

   @section {Bytecode portability}
   The virtual machine register width depends on the width of the
   registers of the host processor. It is at least 32 bits wide and
   large enough to hold a pointer. Memory accesses are performed using
   the endianess of the host processor. These rules enable efficient
   execution of the bytecode on any platform.

   Nonetheless, the instruction set is designed to allow writing
   bytecode programs which are portable across platforms.

   Most ALU instructions only work on the lower 32 bits of the virtual
   machine registers. On 64 bits architectures, the upper half of the
   destination register is zeroed by these instructions. Other
   instructions like @tt add and @tt sub work on the full register
   width because they are useful to handle pointers. This requires
   use of additional sign extension and zero extension instructions as
   appropriate.

   The @ref #CONFIG_MUTEK_BYTECODE_VM64 token can be used to test
   portability of bytecode programs.
   @end section

   @section {Bytecode program syntax}
   A bytecode source file is basically an assembly source file
   composed of generic and custom bytecode instructions. Custom
   instruction sets are described in @em perl modules which can be
   loaded dynamically by the assembler tool.

   C style expressions can be used where constants are
   expected. Moreover, the @tt {bitpos()} operator is available which
   computes a bit index from a power of 2 constant.

   The bytecode source file is piped in the @sourcelink
   scripts/decl_filter.pl script before being assembled. This allows
   inclusion of C headers files as well as use of @tt _sizeof, @tt _offsetof
   and @tt _const operators on C declarations in the bytecode program.

   The following general purpose directives are available:
   @list
   @item @tt{.define name expr} : define an expression macro.
     The C preprocessor is also available in bytecode programs
     and @xref{configuration tokens} can be tested.
   @item @tt{.backend name} : use a specific output backend.
     @em bytecode may be specified in order to prevent generation
     of native machine code for the program.
   @item @tt{.name name} : set the bytecode program name.
   @item @tt{.custom name [mode, mode ...]} : load a custom instruction set module and
     optionally restrict modes which are allowed for instructions of the module.
   @item @tt{.mode m} : declare the default mode used in the current @tt{.func}.
   @item @tt{.export label} : export a label as a global symbol.
   @end list

   ALU instructions use a 2 registers form with the first register
   used as both the source and destination register.

   See @sourcelink tests/pool/bytecode for an example application.
   @end section

   @section {Static analysis}

   The bytecode assembler is capable of performing register usage
   checking provided that some directive are used to declare the role
   of registers. Checking is most efficient when the whole bytecode
   program is written using functions.

   The following directives are available to declare register usage:
   @list
   @item @tt{.assert expr} : make the compilation fail if the expression
     does not reduce to 1.
   @item @tt{.global %1 [aliasA] [, %7 [aliasB] ...]} : declare some registers
     as always initialized.
   @item @tt{.const %1 [aliasA] [, %7 [aliasB] ...]} : declare some registers
     as always initialized which should not be modified by the bytecode.
   @item @tt{.entry %1 [, %7 ...]} : export the previous label as
     a global symbol and declare some registers as initialized
     at this point. This is better to use functions as
     bytecode entry-points.
   @item @tt{.func name}, @tt {.endfunc} : declare a function. This
     is similar to declaring a label but allows better static analysis
     on function calls.
   @item @tt{.proto name}, @tt {.endproto} : declare a function prototype.
   @item @tt{.input %1 [aliasA] [, %7 [aliasB] ...]} : declare some registers used as
     input parameters by the current function or prototype.
   @item @tt{.output %1 [aliasA] [, %7 [aliasB] ...]} : declare some registers used
     return values by the current function or prototype.
   @item @tt{.clobber %1 [aliasA] [, %7 [aliasB] ...]} : declare some registers used
     as temporaries which are left clobbered by the current function or prototype.
   @item @tt{.preserve %1 [aliasA] [, %7 [aliasB] ...]} : declare some registers used
     as temporaries which are saved and restored by the current function or prototype.
   @item @tt{.implement name} : make the current function inherit from
     register declarations of the specified function prototype.
   @end list
   @end section

   @section {Generic instruction set}

   The following operations are available to load values into registers:
   @table 2
     @item Instruction @item Description
     @item @tt{cst8 reg, value} @item Set a register to an unsigned 8 bits
       constant.
     @item @tt{cst16 reg, value, shift} @item Set a register
       to an signed constant in range [-2**16, 2**16-1]. The constant
       may be shifted by a mulitple of 8 bits. This uses 2 opcode words.
     @item @tt{cst32 reg, value, shift} @item Set a register
       to an signed constant in range [-2**32, 2**32-1]. The constant
       may be shifted by a mulitple of 8 bits. This uses 3 opcode words.
     @item @tt{mov reg, reg} @item Copy a value between 2 registers.
   @end table

   The following operations are available to perform usual integer
   operations on register values:
   @table 2
     @item Instruction @item Description
     @item @tt{add8 reg, value} @item Add a signed 8 bits value to a register.
     @item @tt{add reg, reg} @item Add the value of the source register to the destination register.
     @item @tt{sub reg, reg} @item Subtract the value of the source register from the destination register.
     @item @tt{neg reg} @item Subtract the value from zero.
     @item @tt{mul32 reg, reg} @item Multiply the values of the source register and destination registers.
     @item @tt{div32 reg, reg} @item Perform a division, store both quotient and modulus in registers.
     @item @tt{or32 reg, reg} @item 32 bits bitwise or.
     @item @tt{xor32 reg, reg} @item 32 bits bitwise exclusive or.
     @item @tt{and32 reg, reg} @item 32 bits bitwise and.
     @item @tt{not32 reg} @item 32 bits bitwise not.
     @item @tt{shl32 reg, reg} @item 32 bits variable left shift.
     @item @tt{shr32 reg, reg} @item 32 bits variable right shift.
     @item @tt{sha32 reg, reg} @item 32 bits variable right arithmetic shift.
     @item @tt{shi32l reg, bit_index} @item Left shift a register by a constant amount.
       Amount must be in the range [0,31].
     @item @tt{shi32r reg, bit_index} @item Right shift a register by a constant amount.
       Amount must be in the range [0,31].
     @item @tt{shi32a reg, bit_index} @item Right arithmetic shift a register by a constant amount.
       Amount must be in the range [0,31].
     @item @tt{msbs32 reg} @item Find the position of the most significant bit set in range [0, 31].
     @item @tt{exts reg, bit_index} @item Sign extend a register using the specified sign bit in the set {7, 15, 31}.
     @item @tt{extz reg, bit_index} @item Clear all bits in a register above specified bit in the range [0,31].
     @item @tt{swapN reg} @item Exchange bytes of a 16 bits or 32 bits values.
     @item @tt{swapNle reg} @item Exchange bytes only when running on a big endian processor, used to access little endian data in portable way.
     @item @tt{swapNbe reg} @item Exchange bytes only when running on a little endian processor, used to access big endian data in portable way.
     @item @tt{rand32 reg} @item Set the register to a random value
   @end table

   The following comparison operations are available:
   @table 2
     @item Instruction @item Description
     @item @tt{eq reg, reg} @item Compare two registers and skip the next instruction if they are not equal.
     @item @tt{neq reg, reg} @item Compare two registers and skip the next instruction if they are equal.
     @item @tt{eq0 reg} @item Skip the next instruction if the register is not zero.
     @item @tt{neq0 reg} @item Skip the next instruction if the register is zero.
     @item @tt{lt reg, reg} @item Perform an unsigned comparison of two registers and skip
       the next instruction if first reg >= second reg.
     @item @tt{lteq reg, reg} @item Perform an unsigned comparison of two registers and skip
       the next instruction if first reg > second reg.
     @item @tt{lts reg} @item Perform an signed comparison of reg with r0 and skip
       the next instruction if reg >= r0.
     @item @tt{lteqs reg} @item Perform an signed comparison of reg with r0 and skip
       the next instruction if reg > r0.
   @end table

   The following bit oriented operations are available:
   @table 2
     @item Instruction @item Description
     @item @tt{tst32c reg, bit_index} @item Extract a bit from a register and skip the next instruction if the
     bit is not cleared. Bit index is in the range [0,31].
     @item @tt{tst32s reg, bit_index} @item Extract a bit from a register and skip the next instruction if the
     bit is not set. Bit index is in the range [0,31].
     @item @tt{bit32c reg, bit_index} @item Clear a bit in a register. Bit index is in the range [0,31].
     @item @tt{bit32s reg, bit_index} @item Set a bit in a register. Bit index is in the range [0,31].
   @end table

   The following branch instructions are available:
   @table 2
     @item Instruction @item Description
     @item @tt{end} @item Terminate bytecode execution.
     @item @tt{jmp8 label} @item Jump relative. The branch target must be in
       range [-128, +127] from this instruction.
     @item @tt{jmp[16,32] label} @item Jump absolute.
     @item @tt{jmp[16,32]r label} @item Jump relative.
     @item @tt{call8 reg, label} @item Jump relative and save the return address in a register.
       The branch target must be in range [-128, +127] from this instruction.
     @item @tt{call[16,32] reg, label} @item Jump absolute and save the return address in a register.
     @item @tt{call[16,32]r reg, label} @item Jump relative and save the return address in a register.
     @item @tt{ret reg} @item Return to the address saved in a link register.
     @item @tt{jmp reg} @item Jump to the address specified by a register.
     @item @tt{call reg, proto} @item Jump to the address specified by a register and
       save the return address in the same register. The function prototype is used for
       register usage checking.
     @item @tt{loop reg, label} @item If the jump target is backward, this instruction decrements the
       register which should not be initially zero and branch if the
       result is not zero. If the jump target is forward, this instruction decrement the
       register if its initial value is not zero. If the register initial
       value is zero, the branch is taken and the register is left
       untouched.
   @end table

   The following memory access instructions are available:
   @table 2
     @item Instruction @item Description
     @item @tt{ld[8,16,32,64] data_reg, addr_reg} @item Load a value from a
     memory address given by a register into an other register.
     @item @tt{ld[8,16,32,64]i data_reg, addr_reg} @item Load a value from a
     memory address given by a register into an other register then
     add the width of the access to the address register.
     @item @tt{ld[8,16,32,64]e data_reg, addr_reg, offset} @item Load a value from a memory
     address given by a register and a 16 bits signed offset into an
     other register.
     @item @tt{st[8,16,32,64] data_reg, addr_reg} @item Store a value to
     a memory address given by a register from an other register.
     @item @tt{st[8,16,32,64]i data_reg, addr_reg} @item Store a value to a
     memory address given by a register from an other register then
     add the width of the access to the address register.
     @item @tt{st[8,16,32,64]d data_reg, addr_reg} @item Subtract the width of
     the access to the address register then store a value from an
     other register to the resulting memory address.
     @item @tt{st[8,16,32,64]e data_reg, addr_reg, offset} @item Store a value
     from a register to a memory address given by an other register
     and a 16 bits signed offset.
   @end table

   Zero extension is performed by load instructions. In
   order to achieve portability, additional @tt{ld}, @tt{st},
   @tt{ldi}, @tt{sti}, @tt{std}, @tt{lde} and @tt{ste} instructions
   are available to perform memory accesses with a width suitable to
   handle pointers.

   The following miscellaneous instructions are available:
   @table 2
     @item Instruction @item Description
     @item @tt{dump} @item Dump registers to debug output. @see #CONFIG_MUTEK_BYTECODE_DEBUG
     @item @tt{abort} @item Terminate bytecode execution and report an error.
     @item @tt{die} @item Terminate by calling the libc @ref abort function.
     @item @tt{trace mode} @item Enable or disable debug trace. @see bc_set_trace
     @item @tt{ccall reg} @item Call a C function. The address of the function is in the source
     register. Bytecode in compiled form will not be portable.
     @see bc_ccall_function_t
     @item @tt{laddr[16,32,64] reg, label} @item Set a register to the absolute address of a bytecode label.
     @item @tt{laddr[16,32]r reg, label} @item Set a register to the pc relative address of a bytecode label.
     @item @tt{gaddr reg, label} @item Set a register to the address of a
     global symbol. Bytecode in compiled form will not be portable.
     @item @tt{mode m} @item Select custom bytecode interpretation mode in range [0, 15].
   @end table

   Some instructions are provided to handle packing of some register values
   as an array of bytes. The storage space of vm registers is used to store the array,
   clobbering the original register values. When in packed form, register values are
   meaningless for other generic bytecode instructions.
   Packed values can be used by custom operations which needs to
   deal with byte buffers. They may be accessed from the C code using the
   @ref bc_get_bytepack function. The following array packing and unpacking
   instructions are available:
   @table 2
     @item Instruction @item Description
     @item @tt{pack8 reg_1st, reg_count} @item Converts multiple vm register values to
       an array of bytes. The value of a single register is stored as a single byte in the array.
     @item @tt{pack16le reg_1st, reg_count, byte_count} @item Converts multiple vm register
       values to an array of bytes. The value of a single register is stored as a pair of bytes
       in the array, with the less significant byte stored first.
     @item @tt{pack16be reg_1st, reg_count, byte_count} @item Converts multiple vm register
       values to an array of bytes. The value of a single register is stored as a pair of bytes
       in the array, with the most significant byte stored first.
     @item @tt{pack32le reg_1st, reg_count, byte_count} @item Converts multiple vm register
       values to an array of bytes. The value of a single register is stored as 4 bytes
       in the array, with the less significant byte stored first.
     @item @tt{pack32be reg_1st, reg_count, byte_count} @item Converts multiple vm register
       values to an array of bytes. The value of a single register is stored as 4 bytes
       in the array, with the most significant byte stored first.
     @item @tt{unpack8 reg_1st, reg_count} @item Converts an array of bytes to multiple vm
       register values. The value of a single register is loaded from a single byte of the array.
     @item @tt{unpack16le reg_1st, reg_count, byte_count} @item Converts an array of bytes to
       multiple vm register values. The value of a single register is loaded from a pair of bytes
       of the array, with the less significant byte stored first.
     @item @tt{unpack16be reg_1st, reg_count, byte_count} @item Converts an array of bytes to
       multiple vm register values. The value of a single register is loaded from a pair of bytes
       of the array, with the most significant byte stored first.
     @item @tt{unpack32le reg_1st, reg_count, byte_count} @item Converts an array of bytes to
       multiple vm register values. The value of a single register is loaded from 4 bytes
       of the array, with the less significant byte stored first.
     @item @tt{unpack32be reg_1st, reg_count, byte_count} @item Converts an array of bytes to
       multiple vm register values. The value of a single register is loaded from 4 bytes
       of the array, with the most significant byte stored first.
   @end table

   The following instructions are available to dump raw data in the program:
   @table 2
     @item Instruction @item Description
     @item @tt{data8 value, ...} @item Dump at least one 8 bits word value in the program.
     @item @tt{data[16,32] value, ...} @item Dump at least one aligned multi-bytes value with platform endianess.
     @item @tt{data[16,32][le,be] value, ...} @item Dump at least one aligned multi-bytes value with specified endianess.
     @item @tt{str "string"} @item Dump a character string.
     @item @tt{strc "string"} @item Dump a null terminated string.
     @item @tt{strp "string"} @item Dump a pascal string (with a leading length byte).
   @end table

   @end section

   @section {Generic opcodes table}
   @table 4
    @item instruction         @item operands      @item opcode 16 bits word(s)   @item  format

    @item end                 @item               @item @tt{0000 0000 0000 0000} @item  0
    @item dump                @item               @item @tt{0000 0000 0000 0001} @item  0
    @item abort               @item               @item @tt{0000 0000 0000 0010} @item  0
    @item die                 @item               @item @tt{0000 0000 0000 0011} @item  0
    @item nop                 @item               @item @tt{0000 0000 0000 0100} @item  0
    @item trace               @item m             @item @tt{0000 0000 0000 1mmm} @item  0
    @item add8                @item r, +/-v       @item @tt{0000 vvvv vvvv rrrr} @item  0
    @item cst8                @item r, v          @item @tt{0001 vvvv vvvv rrrr} @item  0
    @item call8               @item r, lbl        @item @tt{0010 llll llll rrrr} @item  0
    @item jmp8                @item lbl           @item @tt{0010 llll llll 0000} @item  0
    @item ret                 @item r             @item @tt{0010 0000 0000 rrrr} @item  0
    @item call                @item r             @item @tt{0010 1111 1111 rrrr} @item  0
    @item loop                @item r, lbl        @item @tt{0011 0lll llll rrrr} @item  0

    @item {un,}pack8          @item r, c          @item @tt{0011 1ccc oooo rrrr} @item  4
    @item {un,}pack{16,32}{le,be} @item r, c, b   @item @tt{0011 1ccc oooo rrrr} @item  4
    @item swap{16,32}{le,be,} @item r             @item @tt{0011 1000 oooo rrrr} @item  4

    @item eq                  @item a, b          @item @tt{0100 0000 bbbb aaaa} a > b @item  1
    @item neq                 @item a, b          @item @tt{0100 0000 bbbb aaaa} a < b @item  1
    @item eq0                 @item a             @item @tt{0100 0000 bbbb aaaa} a == b @item  1
    @item mov                 @item a, b          @item @tt{0100 0001 bbbb aaaa} a != b @item  1
    @item neq0                @item a             @item @tt{0100 0001 bbbb aaaa} a == b @item  1
    @item lt                  @item a, b          @item @tt{0100 0010 bbbb aaaa} a != b @item  1
    @item exts                @item a, 7          @item @tt{0100 0010 bbbb aaaa} a == b @item  1
    @item lts                 @item a, b          @item @tt{0100 0011 bbbb aaaa} a != b @item  1
    @item exts                @item a, 15         @item @tt{0100 0011 bbbb aaaa} a == b @item  1
    @item lteq                @item a, b          @item @tt{0100 0100 bbbb aaaa} a != b @item  1
    @item exts                @item a, 31         @item @tt{0100 0100 bbbb aaaa} a == b @item  1
    @item lteqs               @item a, b          @item @tt{0100 0101 bbbb aaaa} a != b @item  1
    @item ---                 @item a             @item @tt{0100 0101 bbbb aaaa} a == b @item  1
    @item add                 @item a, b          @item @tt{0100 0110 bbbb aaaa} a != b @item  1
    @item ---                 @item a             @item @tt{0100 0110 bbbb aaaa} a == b @item  1
    @item sub                 @item a, b          @item @tt{0100 0111 bbbb aaaa} a != b @item  1
    @item neg                 @item a             @item @tt{0100 0111 bbbb aaaa} a == b @item  1
    @item or32                @item a, b          @item @tt{0100 1000 bbbb aaaa} a != b @item  1
    @item rand32              @item a             @item @tt{0100 1000 bbbb aaaa} a == b @item  1
    @item xor32               @item a, b          @item @tt{0100 1001 bbbb aaaa} a != b @item  1
    @item ccall               @item a             @item @tt{0100 1001 bbbb aaaa} a == b @item  1
    @item and32               @item a, b          @item @tt{0100 1010 bbbb aaaa} a != b @item  1
    @item ---                 @item a             @item @tt{0100 1010 bbbb aaaa} a == b @item  1
    @item sha32               @item a, b          @item @tt{0100 1011 bbbb aaaa} a != b @item  1
    @item not32               @item a             @item @tt{0100 1011 bbbb aaaa} a == b @item  1
    @item shl32               @item a, b          @item @tt{0100 1100 bbbb aaaa} a != b @item  1
    @item ---                 @item a             @item @tt{0100 1100 bbbb aaaa} a == b @item  1
    @item shr32               @item a, b          @item @tt{0100 1101 bbbb aaaa} a != b @item  1
    @item ---                 @item a             @item @tt{0100 1101 bbbb aaaa} a == b @item  1
    @item mul32               @item a, b          @item @tt{0100 1110 bbbb aaaa} @item  1
    @item div32               @item a, b          @item @tt{0100 1111 bbbb aaaa} a != b @item  1
    @item msbs32              @item a             @item @tt{0100 1111 bbbb aaaa} a == b @item  1

    @item tst32[c,s]          @item r, bit        @item @tt{0101 00sb bbbb rrrr} @item  2
    @item bit32[c,s]          @item r, bit        @item @tt{0101 01sb bbbb rrrr} @item  2
    @item shi32[l,r]          @item r, bit        @item @tt{0101 10rb bbbb rrrr} @item  2
    @item shi32a              @item r, bit        @item @tt{0101 110b bbbb rrrr} @item  2
    @item extz                @item r, bit        @item @tt{0101 111b bbbb rrrr} @item  2

    @item ld[8,16,32,64][i]   @item r, ra         @item @tt{0110 ss0i aaaa rrrr} @item  3
    @item st[8,16,32,64][i]   @item r, ra         @item @tt{0110 ss1i aaaa rrrr} @item  3

    @item st[8,16,32,64]d     @item r, ra         @item @tt{0111 ss10 aaaa rrrr} @item  3
    @item ld[8,16,32,64]e     @item r, ra, +/-v   @item @tt{0111 ss01 aaaa rrrr, v} @item  3
    @item st[8,16,32,64]e     @item r, ra, +/-v   @item @tt{0111 ss11 aaaa rrrr, v} @item  3

    @item gaddr               @item r, lbl        @item @tt{0111 0000 0000 rrrr, v?} @item  3
    @item mode                @item mode          @item @tt{0111 0000 0001 mmmm} @item  3
    @item pick                @item r, mask       @item @tt{0111 0100 0000 mmmm, v} m != 0 @item  3
    @item place               @item r, mask       @item @tt{0111 0100 0001 mmmm, v} m != 0 @item  3
@c  @item ---                 @item               @item @tt{0111 1-00 000- ----} @item  3
    @item cst[16,32]          @item r, v, b       @item @tt{0111 xs00 1bbb rrrr, v, v?} @item  3
    @item laddr[16,32][r]     @item r, lbl        @item @tt{0111 Rs00 0x1- rrrr, v, v?} @item  3
    @item jmp[16,32][r]       @item lbl           @item @tt{0111 Rs00 0100 rrrr, v, v?} @item  3
    @item call[16,32][r]      @item r, lbl        @item @tt{0111 Rs00 0101 rrrr, v, v?} @item  3

    @item custom              @item               @item @tt{1--- ---- ---- ----} @item
   @end table
   @end section
*/

typedef uint16_t bc_opcode_t;

#if INT_REG_SIZE <= 32 && !defined(CONFIG_MUTEK_BYTECODE_VM64)
typedef uint32_t bc_reg_t;
typedef int32_t bc_sreg_t;
# define BC_REG_FORMAT "08" PRIx32
#else
typedef uint64_t bc_reg_t;
typedef int64_t bc_sreg_t;
# define BC_REG_FORMAT "016" PRIx64
#endif

/** @internal */
enum bc_flags_s
{
  BC_FLAGS_NATIVE   = 0x01000000,
  BC_FLAGS_SANDBOX  = 0x02000000,
  BC_FLAGS_SIZEMASK = 0x00ffffff,
};

#define _BC_TRACE_ALL 1
#define _BC_TRACE_JUMP 2
#define _BC_TRACE_REGS 4

enum bc_trace_e
{
  BC_TRACE_DISABLED  = 0,
  BC_TRACE_ALL       = _BC_TRACE_ALL,
  BC_TRACE_JUMP      = _BC_TRACE_JUMP,
  BC_TRACE_ALL_REGS  = _BC_TRACE_ALL | _BC_TRACE_REGS,
  BC_TRACE_JUMP_REGS = _BC_TRACE_JUMP | _BC_TRACE_REGS,
};

struct bc_context_s;

/** @This can be used to declare bytecode entry points. @see bc_set_pc */
typedef struct bytecode_entry_s bytecode_entry_t;

/** @This specifes status codes returned by the @ref bc_run function.
    @see #BC_STATUS_CUSTOM */
enum bc_run_status_e
{
  BC_RUN_STATUS_END = 0,
  BC_RUN_STATUS_CYCLES = 1,
  BC_RUN_STATUS_BREAK = 2,
  BC_RUN_STATUS_FAULT = 3,
};

/** @This tests if the return status of @ref bc_run is a custom opcode
    or a value specified in @ref bc_run_status_e */
#define BC_STATUS_CUSTOM(op) ((op) & 0x8000)

/** @internal */
typedef bc_opcode_t (bc_run_t)(struct bc_context_s *ctx);

/** @This is the bytecode descriptor header */
struct bc_descriptor_s
{
  const void *code;
  bc_run_t *run;
  uint32_t flags;
};

/** @This defines the virtual machine context.
    @internalmembers */
struct bc_context_s
{
  bc_reg_t v[16];
  union {
    /** Bytecode resume execution pointer. For native code, this is a
        pointer to the machine code instruction. For vm bytecode, this
        is a 16 bits aligned pointer to the next instruction word with
        the bit 0 indicating if the next instruction must be skipped
        on resume. */
    uintptr_t pc;
    const void *vpc;
  };

  const struct bc_descriptor_s *desc;

#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  uint8_t skip;
#endif
  uint8_t mode;

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  /** address of writable data segment when sandboxed */
  void *data_base;
  /** mask address of writable data segment when sandboxed */
  uintptr_t data_end;
  /** maximum number of executed cycles by a single call to @ref bc_run_sandbox */
  uint16_t max_cycles;
#endif
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  uintptr_t bp_list[CONFIG_MUTEK_BYTECODE_BREAKPOINTS];
  uint16_t bp_mask;
  bool_t BITFIELD(bp_skip,1);
#endif
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  enum bc_trace_e BITFIELD(trace,3);
#endif
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  /** @see bc_init_sandbox */
  bool_t BITFIELD(sandbox,1);
#endif
};

FIRST_FIELD_ASSERT(bc_context_s, v); /* used in asm backends */

/** @This initializes the virtual machine. The initial value of the
    registers is undefined. */
void
bc_init(struct bc_context_s *ctx,
        const struct bc_descriptor_s *desc);

/** @This initializes the virtual machine in sandbox mode. When
    working in sandbox mode, address are translated and the following
    checks are performed:

    @list
      @item Execution of instructions are not allowed outside of the
        code segment specified in the bytecode descriptor. Code base
        address inside the virtual machine is 0.
      @item Load and store instructions addresses are translated from
        0x80000000 to the @tt data_base address. Loads
        below 0x8000000 are translated to the code segment.
      @item The @tt ccall instruction can not be used.
      @item The @tt abort instruction is equivalent to @tt die.
    @end list

    When in sandbox mode on a 64 bits target, instructions wont touch
    registers above bit 31. This makes the sandbox a 32 bits virtual
    machine.

    The @tt cycles parameter specifies the initial number of cycles
    executed by the @ref bc_run function before it returns @ref
    BC_RUN_STATUS_CYCLES.
*/
config_depend(CONFIG_MUTEK_BYTECODE_SANDBOX)
void bc_init_sandbox(struct bc_context_s *ctx, const struct bc_descriptor_s *desc,
                     void *data_base, size_t data_size,
                     uint_fast16_t cycles);

/** @This updates the remaining number of cycles before the @ref bc_run
    function returns @ref BC_RUN_STATUS_CYCLES. */
config_depend_alwaysinline(CONFIG_MUTEK_BYTECODE_SANDBOX,
void bc_set_cycles(struct bc_context_s *ctx, uint_fast16_t cycles),
{
  assert(ctx->sandbox);
  ctx->max_cycles = cycles;
});

/** @see bc_set_cycles */
config_depend_alwaysinline(CONFIG_MUTEK_BYTECODE_SANDBOX,
uint_fast16_t bc_get_cycles(const struct bc_context_s *ctx),
{
  assert(ctx->sandbox);
  return ctx->max_cycles;
});

/** @This translates an address from the sandbox virtual machine
    address space to an usable pointer. This returns @tt NULL if the
    address range is not valid and contiguous inside the sandbox.

    When the @tt writable argument is set, the function return @tt
    NULL if the address range is not writable in the sandbox. */
config_depend(CONFIG_MUTEK_BYTECODE_SANDBOX)
void * bc_translate_addr(struct bc_context_s *ctx, bc_reg_t addr,
                         size_t size, bool_t writable);

/** @This initializes a bytecode descriptor from a bytecode loadable
    blob. The format of the blob is:
    @list
      @item flags in 16 bits little endian representation
      @item words count in 16 bits little endian representation
      @item instruction words
    @end list
    The @tt blob pointer must be 16 bits aligned.

    @see bc_desc_init
*/
error_t
bc_load(struct bc_descriptor_s *desc,
        const uint8_t *blob, size_t len);

/** @This initializes a bytecode descriptor. This can be used in place
    of @ref bc_load when some raw bytecode is loaded in memory. */
error_t
bc_desc_init(struct bc_descriptor_s *desc,
             const void *code, size_t len,
             enum bc_flags_s flags);

/** @see bc_set_regs */
void
bc_set_regs_va(struct bc_context_s *ctx, uint16_t mask, va_list ap);

/** @This set the value of multiple registers of the virtual
    machine. The @tt mask parameter specifies which register must be
    initialized. An additional value of type @ref uintptr_t must be
    passed for each bit set in @tt mask. */
void
bc_set_regs(struct bc_context_s *ctx, uint16_t mask, ...);

/** @This returns the value of one of the 16 virtual machine registers */
ALWAYS_INLINE uintptr_t
bc_get_reg(const struct bc_context_s *ctx, uint_fast8_t i)
{
  return ctx->v[i];
}

/** @This returns a pointer to a packed array of bytes stored in virtual
    machine register storage. See the @tt pack and @tt unpack instructions. */
ALWAYS_INLINE uint8_t *
bc_get_bytepack(const struct bc_context_s *ctx, uint_fast8_t i)
{
  return (uint8_t*)(ctx->v + i);
}

/** @This returns a pointer to a packed array of bytes stored in
    virtual machine register storage. When the index of the register
    is to high for the specified number of bytes, a pointer to
    register 0 is retured instead. */
ALWAYS_INLINE uint8_t *
bc_get_bytepack_safe(const struct bc_context_s *ctx, uint_fast8_t i,
                     size_t bytes)
{
  size_t reg_count = (((bytes - 1) | 3) + 1) >> 2;
  int32_t m = i + reg_count - 17;
  i &= m >> 31;
  return (uint8_t*)(ctx->v + i);
}

/** @This sets the value of one of the 16 virtual machine registers */
ALWAYS_INLINE void
bc_set_reg(struct bc_context_s *ctx, uint_fast8_t i, uintptr_t value)
{
  ctx->v[i] = value;
}

/** @This returns the value of the virtual machine program counter in
    the host address space. @see bc_get_sandbox_pc */
ALWAYS_INLINE const void *
bc_get_pc(const struct bc_context_s *ctx)
{
  return (void*)(ctx->pc & ~(uintptr_t)1);
}

/** @This changes the value of the virtual machine program counter in the
    host address space. @see bc_set_sandbox_pc */
ALWAYS_INLINE void
bc_set_pc(struct bc_context_s *ctx, const void *pc)
{
  ctx->vpc = pc;
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_skip = 0;
#endif
}

/** @This changes the program counter of a sandboxed virtual machine. */
config_depend(CONFIG_MUTEK_BYTECODE_SANDBOX)
error_t bc_set_sandbox_pc(struct bc_context_s *ctx, uint32_t pc);

/** @This returns the program counter of a sandboxed virtual machine. */
config_depend_alwaysinline(CONFIG_MUTEK_BYTECODE_SANDBOX,
uint32_t bc_get_sandbox_pc(const struct bc_context_s *ctx),
{
  assert(ctx->sandbox);
  return bc_get_pc(ctx) - ctx->desc->code;
});

/** @This function enables or disable the bytecode execution trace
    debug output. If the @ref #CONFIG_MUTEK_BYTECODE_TRACE token is
    not defined, this function has no effect. The @tt trace
    instruction can be used to enable and disable trace output. */
ALWAYS_INLINE void
bc_set_trace(struct bc_context_s *ctx, enum bc_trace_e mode)
{
#ifdef CONFIG_MUTEK_BYTECODE_TRACE
  ctx->trace = mode;
#endif
}

/** @This function set the address of a breakpoint. This does not
    enable the breakpoint.
    @see bc_enable_bp
    @see #CONFIG_MUTEK_BYTECODE_BREAKPOINTS */
ALWAYS_INLINE error_t
bc_set_bp(struct bc_context_s *ctx, uint_fast8_t idx, uintptr_t pc)
{
  if (idx >= CONFIG_MUTEK_BYTECODE_BREAKPOINTS)
    return -ENOENT;
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
# ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  if (ctx->sandbox)
    pc += (uintptr_t)ctx->desc->code;
# endif
  ctx->bp_list[idx] = pc;
#endif
  return 0;
}

/** @This function enable or disable a breakpoint.
    @see #CONFIG_MUTEK_BYTECODE_BREAKPOINTS */
ALWAYS_INLINE void
bc_enable_bp(struct bc_context_s *ctx, uint_fast8_t idx, bool_t en)
{
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  if (en)
    ctx->bp_mask |= 1 << idx;
  else
    ctx->bp_mask &= ~(1 << idx);
#endif
}

/** @This function enable or disable multiple breakpoints.
    @see #CONFIG_MUTEK_BYTECODE_BREAKPOINTS */
ALWAYS_INLINE void
bc_enable_bps(struct bc_context_s *ctx, uint16_t mask)
{
#if CONFIG_MUTEK_BYTECODE_BREAKPOINTS > 0
  ctx->bp_mask = mask;
#endif
}

/** @This returns 1 if the provided address is an enabled
    breakpoint */
bool_t
bc_test_bp(const struct bc_context_s *ctx, const void *pc);

/** @This skip the next instruction. This can only be called if the
    execution has stopped on a conditional custom instruction.

    A single instruction can be skipped. When executed more than once
    without restarting execution of the bytecode, this function does
    nothing. */
ALWAYS_INLINE void
bc_skip(struct bc_context_s *ctx)
{
#ifdef CONFIG_MUTEK_BYTECODE_NATIVE
  ctx->pc += ctx->skip;
  ctx->skip = 0;
#else
  ctx->pc |= 1;
#endif
}

/** @This returns the current bytecode execution mode */
ALWAYS_INLINE uint_fast8_t bc_get_mode(const struct bc_context_s *ctx)
{
  return ctx->mode;
}

/** @This sets the current bytecode execution mode */
ALWAYS_INLINE void bc_set_mode(struct bc_context_s *ctx, uint_fast8_t mode)
{
  ctx->mode = mode & 15;
}

/** @This returns the instruction name from an opcode. */
config_depend(CONFIG_MUTEK_BYTECODE_VM)
const char * bc_opname(uint16_t op);

/** @This dumps the virtual machine state. If the @ref
    #CONFIG_MUTEK_BYTECODE_DEBUG token is not defined, this
    function has no effect. */
void bc_dump(const struct bc_context_s *ctx, bool_t regs);

/** This function starts or resumes executions of the bytecode.

    It returns under the following conditions:
    @list
      @item When the @tt end instruction is encountered, the function
        returns @ref BC_RUN_STATUS_END.
      @item When an error occurs, the function returns @ref
        BC_RUN_STATUS_FAULT.
      @item When a custom instruction is encountered, the opcode
        of the instruction is returned. The opcode is a 16 bits value
        not less than 0x8000. The caller must handle the custom
        instruction and resume execution of the bytecode.
      @item When sandboxing is used and the maximum number of cycles
        has been reached, the function returns @ref BC_RUN_STATUS_CYCLES.
        The limit can be updated and the bytecode may then be resumed.
      @item When breakpoints are used, the function may return
        @ref BC_RUN_STATUS_BREAK. The bytecode may then be resumed.
    @end list

    This function will either run the virtual machine or jump to the
    machine compiled bytecode. The type of bytecode is guessed from
    the descriptor.
*/
ALWAYS_INLINE bc_opcode_t bc_run(struct bc_context_s *ctx)
{
  return ctx->desc->run(ctx);
}

/** @internal @This specifies packing and byteswap opcode operations */
enum bc_opcode_pack_e
{
  BC_OP_PACK8       = 0,
  BC_OP_PACK16LE    = 1,
  BC_OP_PACK16BE    = 2,
  BC_OP_UNPACK16LE  = 3,
  BC_OP_UNPACK16BE  = 4,
  BC_OP_SWAP16LE    = 5,
  BC_OP_SWAP16BE    = 6,
  BC_OP_SWAP16      = 7,
  BC_OP_UNPACK8     = 8,
  BC_OP_PACK32LE    = 9,
  BC_OP_PACK32BE    = 10,
  BC_OP_UNPACK32LE  = 11,
  BC_OP_UNPACK32BE  = 12,
  BC_OP_SWAP32LE    = 13,
  BC_OP_SWAP32BE    = 14,
  BC_OP_SWAP32      = 15,
};

/** @see #BC_CCALL_FUNCTION */
#define BC_CCALL_FUNCTION(n) bc_reg_t (n)(const struct bc_context_s *ctx)
/** C function type invoked by the @tt ccall instruction. */
typedef BC_CCALL_FUNCTION(bc_ccall_function_t);

#endif

