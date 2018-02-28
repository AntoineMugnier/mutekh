#    This file is part of MutekH.
#    
#    MutekH is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation; version 2.1 of the
#    License.
#    
#    MutekH is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.
#    
#    You should have received a copy of the GNU Lesser General Public
#    License along with MutekH; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
#    02110-1301 USA.
#
#    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2017
#    Copyright Nicolas Pouillon <nipo@ssji.net>, 2017

#    This file contains ISA definitions for the EFR32 radio sequencer.
#    This work is fully based on public info + reverse engineering,
#    no information have been provided by the chip vendor.

use strict;

package seq_isa;

our @regname = ( 'r0', 'r1', 'r2', 'r3', 'r4', 'r5', 'sp', 'pc',
		 'sr0', 'sr1', 'sr2', 'sr3', 'bad0', 'bad1', 'bad2', 'bad3',
		 'neg', 'pos', 'zero', 'carry' );

our %reg_by_name;

for ( my $i = 0; my $n = $regname[$i]; $i++ ) {
    $reg_by_name{$n} = $i;
}

sub signext
{
    my ( $x, $bits ) = @_;

    my $m = (1 << $bits) - 1;
    my $c = 1 << ($bits - 1);
    return (($x & $m) ^ $c) - $c;
}

sub decode_unknown {
    my ( $ctx, $bin, $a, $pc ) = @_;
    $a->{arg} = sprintf("0x%02x, 0x%02x", $bin->{$pc}, $bin->{$pc + 1});
}

sub encode_unknown {
    my ( $ctx, $thisop ) = @_;
    my $b0 = $ctx->{check_num}->( $thisop, 0, 0, 255 );
    my $b1 = $ctx->{check_num}->( $thisop, 1, 0, 255 );
    $thisop->{bin} = [ $b0, $b1 ];
}

sub decode_noarg {
    my ( $ctx, $bin, $a, $pc ) = @_;
}

sub encode_noarg {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    $thisop->{bin} = [ $op->{code}->[0],
                       $op->{code}->[1] ];
}

sub decode_jmp {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $t;
    my $mode = $bin->{$pc} & 3;

    if ( $mode == 0 ) {
	$t = ($bin->{$pc + 5} << 24) | ($bin->{$pc + 4} << 16) | ($bin->{$pc + 3} << 8) | $bin->{$pc + 2};
	$t += $pc;
	$a->{variant} = ".r32*";
    } elsif ( $mode == 1 ) {
	$t = ($bin->{$pc + 5} << 24) | ($bin->{$pc + 4} << 16) | ($bin->{$pc + 3} << 8) | $bin->{$pc + 2};
	$a->{variant} = ".a32*";
    } elsif ( $mode == 2 ) {
	$t = ($bin->{$pc + 3} << 8) | $bin->{$pc + 2};
	$t |= ($pc & 0xffff0000);
	$a->{variant} = ".a16";
    } else {
	$a->{variant} = ".unknown*";
	$t = 0;
    }

    $a->{arg} = $ctx->{translate_label}->( $t );
    $a->{badtarget}++ if !defined $bin->{$t};
    $a->{badtarget}++ if $t & 1;
    $a->{target} = $t;
}

sub encode_jmp2 {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $l = $ctx->{check_label}->( $thisop, 0 );
    my $t = $l->{addr};
    $thisop->{bin} = [ $op->{code}->[0], $op->{code}->[1],
                       $t & 0xff, ($t >> 8) & 0xff ];
}

sub decode_jc {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $t;
    my $mode = $bin->{$pc + 1} & 7;

    $t = $pc + signext($bin->{$pc}, 8);
    $a->{arg} = $ctx->{translate_label}->( $t );

    if ( $mode != 7 ) {
	$a->{badtarget}++ if !defined $bin->{$t};
	$a->{badtarget}++ if $t & 1;
	$a->{target} = $t;
    }
}

sub encode_jc {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $l = $ctx->{check_label}->( $thisop, 0 );
    my $t = $l->{addr} - $thisop->{addr};
    $ctx->{error}->( $thisop, "jump target out of range\n" )
	if ( $t > 127 || $t < -128 );
    $thisop->{bin} = [ $t & 0xff, $op->{code}->[1] ];
}

sub decode_cst32 {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $t = ($bin->{$pc + 5} << 24) | ($bin->{$pc + 4} << 16) | ($bin->{$pc + 3} << 8) | $bin->{$pc + 2};
    my $r = $bin->{$pc + 1} & 0x7;
    $a->{arg} = sprintf("%s, %s", $regname[$r], $ctx->{translate_addr}->($t));
    $a->{variant} = ".u32";
}

sub encode_cst32 {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $r = $ctx->{check_reg}->( $thisop, 0, 8 );
    my $v = $ctx->{check_num}->( $thisop, 1, 0, 0xffffffff );
    $thisop->{bin} = [ $op->{code}->[0],
                       $op->{code}->[1] | $r,
                       $v & 0xff, ($v >> 8) & 0xff,
                       ($v >> 16) & 0xff, ($v >> 24) & 0xff ];
}

sub decode_st {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rm = $bin->{$pc + 1} & 0x7;
    my $rd = ($bin->{$pc} >> 4) & 0x7;
    $a->{arg} = sprintf("[%s], %s", $regname[$rm], $regname[$rd]);
}

sub encode_st {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rm = $ctx->{check_reg}->( $thisop, 0, 8, 1 );
    my $rs = $ctx->{check_reg}->( $thisop, 1, 8 );
    $thisop->{bin} = [ $op->{code}->[0] | ($rs << 4),
                       $op->{code}->[1] | $rm ];
}

sub decode_ld {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0xf;
    my $rm = ($bin->{$pc} >> 4) & 0x7;
    $a->{arg} = sprintf("%s, [%s]", $regname[$rd], $regname[$rm]);
}

sub encode_ld {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rd = $ctx->{check_reg}->( $thisop, 0, 16 );
    my $rm = $ctx->{check_reg}->( $thisop, 1, 8, 1 );
    $thisop->{bin} = [ $op->{code}->[0] | ($rm << 4),
                       $op->{code}->[1] | $rd ];
}

sub decode_alu {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0x7;
    my $rs = ($bin->{$pc} >> 4) & 0x7;
    $a->{arg} = sprintf("%s, %s", $regname[$rd], $regname[$rs]);
}

sub encode_alu {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rd = $ctx->{check_reg}->( $thisop, 0, 8 );
    my $rs = $ctx->{check_reg}->( $thisop, 1, 8 );
    $thisop->{bin} = [ $op->{code}->[0] | ($rs << 4),
                       $op->{code}->[1] | $rd ];
}

sub decode_mov {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0xf;
    my $rs = ($bin->{$pc} >> 4) & 0xf;
    $a->{arg} = sprintf("%s, %s", $regname[$rd], $regname[$rs]);
}

sub encode_mov {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rd = $ctx->{check_reg}->( $thisop, 0, 16 );
    my $rs = $ctx->{check_reg}->( $thisop, 1, 16 );
    $thisop->{bin} = [ $op->{code}->[0] | ($rs << 4),
                       $op->{code}->[1] | $rd ];
}

sub decode_push {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $r = ($bin->{$pc} >> 4) & 0x7;
    $a->{arg} = $regname[$r];
}

sub encode_push {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rs = $ctx->{check_reg}->( $thisop, 0, 8 );
    $thisop->{bin} = [ $op->{code}->[0] | ($rs << 4),
                       $op->{code}->[1] ];
}

sub decode_pop {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $r = $bin->{$pc + 1} & 0x7;
    $a->{arg} = $regname[$r];
}

sub encode_pop {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rd = $ctx->{check_reg}->( $thisop, 0, 8 );
    $thisop->{bin} = [ $op->{code}->[0],
                       $op->{code}->[1] | $rd ];
}

sub decode_add8 {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0x7;
    my $i = signext($bin->{$pc}, 8);
    $a->{arg} = sprintf("%s, %i	  // 0x%08x", $regname[$rd], $i, $i & 0xffffffff);
    $a->{variant} = ".s8";
}

sub encode_add8 {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $r = $ctx->{check_reg}->( $thisop, 0, 8 );
    my $v = $ctx->{check_num}->( $thisop, 1, -128, 127 );
    $thisop->{bin} = [ $v & 0xff, $op->{code}->[1] | $r ];
}

sub decode_cst8 {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0x7;
    my $i = $bin->{$pc};
    $a->{arg} = sprintf("%s, 0x%02x", $regname[$rd], $i);
    $a->{variant} = ".u8";
}

sub encode_cst8 {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $r = $ctx->{check_reg}->( $thisop, 0, 8 );
    my $v = $ctx->{check_num}->( $thisop, 1, 0, 255 );
    $thisop->{bin} = [ $v & 0xff, $op->{code}->[1] | $r ];
}

sub decode_timer {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $v = (($bin->{$pc + 1} & 0x3) << 8) | $bin->{$pc};
    $a->{arg} = sprintf("0x%03x", $v);
    $a->{variant} = ".u10";
}

sub encode_timer {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $v = $ctx->{check_num}->( $thisop, 0, 0, 0x3ff );
    $thisop->{bin} = [ $v & 0xff, $op->{code}->[1] | ($v >> 8) ];
}

sub decode_waitmask {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $v = (($bin->{$pc + 1} & 0x1) << 8) | $bin->{$pc};
    $a->{arg} = sprintf("0x%03x", $v);
    $a->{variant} = ".u9";
}

sub encode_waitmask {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $v = $ctx->{check_num}->( $thisop, 0, 0, 0x1ff );
    $thisop->{bin} = [ $v & 0xff, $op->{code}->[1] | ($v >> 8) ];
}

sub decode_wait2 {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $v = ($bin->{$pc} >> 4) & 3;
    $a->{arg} = sprintf("%u", $v);
    $a->{variant} = ".u2";
}

sub encode_wait2 {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $v = $ctx->{check_num}->( $thisop, 0, 0, 3 );
    $thisop->{bin} = [ $op->{code}->[0] | ($v << 4),
                       $op->{code}->[1] ];
}

sub decode_wait1 {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $r = $bin->{$pc + 1} & 0x7;
    $a->{arg} = $regname[$r];
}

sub encode_wait1 {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rd = $ctx->{check_reg}->( $thisop, 0, 8 );
    $thisop->{bin} = [ $op->{code}->[0],
                       $op->{code}->[1] | $rd ];
}

sub decode_strac {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0x7;
    my $i = $bin->{$pc} * 4 + $ctx->{racaddr};
    $a->{arg} = sprintf("[%s], %s", $ctx->{translate_addr}->($i), $regname[$rd]);
}

sub encode_strac {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $racaddr = $ctx->{racaddr};
    my $v = $ctx->{check_num}->( $thisop, 0, $racaddr, $racaddr + 1023, 1 ) - $racaddr;
    my $rd = $ctx->{check_reg}->( $thisop, 1, 8 );
    $thisop->{bin} = [ $v >> 2, $op->{code}->[1] | $rd ];
}

sub decode_ldrac {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0x7;
    my $i = $bin->{$pc} * 4 + $ctx->{racaddr};
    $a->{arg} = sprintf("%s, [%s]", $regname[$rd], $ctx->{translate_addr}->($i));
}

sub encode_ldrac {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $rs = $ctx->{check_reg}->( $thisop, 0, 8 );
    my $racaddr = $ctx->{racaddr};
    my $v = $ctx->{check_num}->( $thisop, 1, $racaddr, $racaddr + 1023, 1 ) - $racaddr;
    $thisop->{bin} = [ $v >> 2, $op->{code}->[1] | $rs ];
}

sub decode_rot {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rd = $bin->{$pc + 1} & 0x7;
    my $i = ($bin->{$pc} & 0x7) + 1;
    $a->{arg} = sprintf("%s, %u", $regname[$rd], $i);
}

sub encode_rot {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $r = $ctx->{check_reg}->( $thisop, 0, 8 );
    my $v = $ctx->{check_num}->( $thisop, 1, 1, 8 ) - 1;
    $thisop->{bin} = [ $op->{code}->[0] | $v, $op->{code}->[1] | $r ];
}

sub decode_bitband {
    my ( $ctx, $bin, $a, $pc ) = @_;
    my $rm = $bin->{$pc + 1} & 0x7;
    my $mode = $bin->{$pc} >> 6;
    my $b = $bin->{$pc} & 0x1f;
    my $op = $bin->{$pc} & 0x20 ? "SET" : "CLEAR";
    $a->{arg} = sprintf("[%s], %d, %s, %u", $regname[$rm], $b, $op, $mode);
}

sub encode_bitband {
    my ( $ctx, $thisop ) = @_;
    my $op = $thisop->{op};
    my $r = $ctx->{check_reg}->( $thisop, 0, 8, 1 );
    my $v = $ctx->{check_num}->( $thisop, 1, 0, 31 );
    my $x;
    if ( uc( $thisop->{args}->[2] ) eq "SET" ) {
        $x = 1;
    } elsif ( uc( $thisop->{args}->[2] ) eq "CLEAR" ) {
        $x = 0;
    } else {
	$ctx->{error}( $thisop, "expected SET or CLEAR as operand 2 of `$thisop->{name}'.\n");
    }
    my $m = $ctx->{check_num}->( $thisop, 3, 0, 3 );
    $thisop->{bin} = [ ($m << 6) | ($x << 5) | $v, $op->{code}->[1] | $r ];
}

use constant REG_LEFT_8    => 1;
use constant REG_LEFT_16   => 2;
use constant REG_RIGHT_8   => 4;
use constant REG_RIGHT_16  => 8;
use constant REG_STACKPTR  => 16;
use constant REG_CARRY     => 32;
use constant REG_POS       => 64;
use constant REG_NEG       => 128;
use constant REG_ZERO      => 256;
use constant REG_FLAGS     => REG_CARRY | REG_POS | REG_NEG | REG_ZERO;

our @ops = (
    { name => 'mov',       format => 'ssss 0001 0001 dddd',
      code => [ 0x01, 0x10 ], mask => [ 0x0f, 0xf0 ], len => 2,
      decode => \&decode_mov, encode => \&encode_mov,
      regrd => REG_LEFT_16, regwr => REG_RIGHT_16,
      pseudo => "Rd = Rs"
    },

    { name => 'ld',        format => '0mmm 0100 0001 dddd',
      code => [ 0x04, 0x10 ], mask => [ 0x8f, 0xf0 ], len => 2,
      decode => \&decode_ld, encode => \&encode_ld,
      regrd => REG_LEFT_8, regwr => REG_RIGHT_16,
      pseuso => "Rd = load32(Rm)"
    },

    { name => 'pop',       format => '0000 0000 1001 0rrr',
      code => [ 0x00, 0x90 ], mask => [ 0xff, 0xf8 ], len => 2,
      decode => \&decode_pop, encode => \&encode_pop,
      regrd => REG_STACKPTR, regwr => REG_RIGHT_8 | REG_STACKPTR,
      pseuso => "R6 -= 4; flags(R6); Rd = load32(R6)"
    },

    { name => 'st',        format => '0sss 1-00 0001 0mmm',
      code => [ 0x08, 0x10 ], mask => [ 0x8b, 0xf8 ], len => 2,
      decode => \&decode_st, encode => \&encode_st,
      regrd => REG_RIGHT_8 | REG_LEFT_8,
      pseuso => "store32(Rm, Rd)"
    },

    { name => 'push',      format => '0rrr 0000 1000 1000',
      code => [ 0x00, 0x88 ], mask => [ 0x8f, 0xff ], len => 2,
      decode => \&decode_push, encode => \&encode_push,
      regrd => REG_LEFT_8 | REG_STACKPTR, regwr => REG_STACKPTR,
      pseuso => "store32(R6, Rd); R6 += 4; flags(R6)"
    },

    { name => 'bitband',   format => '??xb bbbb 0111 1mmm',
      code => [ 0x00, 0x78 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_bitband, encode => \&encode_bitband,
      regrd => REG_RIGHT_8,
      pseuso => "T = load32(Rm); S = (x << b) | (T & ~(1 << b)); store32(Rm, S)"
    },

    { name => 'addi',       format => 'iiii iiii 0010 0ddd',
      code => [ 0x00, 0x20 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_add8, encode => \&encode_add8,
      regrd => REG_RIGHT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rd + sign_ext(i); flags(Rd)"
    },

    { name => 'addc',      format => '0sss 0001 0010 1ddd',
      code => [ 0x01, 0x28 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8 | REG_CARRY, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rd + Rs + carry; flags(Rd)"
    },

    { name => 'addc.nowb', format => '0sss 0000 0010 1ddd',
      code => [ 0x00, 0x28 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8 | REG_CARRY, regwr => REG_FLAGS,
      pseudo => "T = Rd + Rs + carry; flags(T)"
    },

    { name => 'andi',       format => 'iiii iiii 0011 0ddd',
      code => [ 0x00, 0x30 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_cst8, encode => \&encode_cst8,
      regrd => REG_RIGHT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rs & zero_ext(i); flags(Rd)"
    },

    { name => 'and',       format => '0sss 0001 0011 1ddd',
      code => [ 0x01, 0x38 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rs & Rd; flags(Rd)"
    },

    { name => 'andi.nowb',  format => 'iiii iiii 1101 1ddd',
      code => [ 0x00, 0xd8 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_cst8, encode => \&encode_cst8,
      regrd => REG_RIGHT_8, regwr => REG_FLAGS,
      pseudo => "T = Rd & zero_ext(i); flags(T)"
    },

    { name => 'and.nowb',  format => '0sss 0000 0011 1ddd',
      code => [ 0x00, 0x38 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8, regwr => REG_FLAGS,
      pseudo => "T = Rs & Rd; flags(T)"
    },

    { name => 'or',        format => '0sss 0001 0100 0ddd',
      code => [ 0x01, 0x40 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rs | Rd; flags(Rd)"
    },

    { name => 'or.nowb*',  format => '0sss 0000 0100 0ddd',
      code => [ 0x00, 0x40 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu, reserved => 1,
      regrd => REG_RIGHT_8 | REG_LEFT_8, regwr => REG_FLAGS,
      pseudo => "T = Rs | Rd; flags(T)"
    },

    { name => 'xori',       format => 'iiii iiii 0100 1ddd',
      code => [ 0x00, 0x48 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_cst8, encode => \&encode_cst8,
      regrd => REG_RIGHT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rd ^ zero_ext(i); flags(Rd)"
    },

    { name => 'neg',       format => '0sss 0001 0101 1ddd',
      code => [ 0x01, 0x58 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = -Rs; flags(Rd)"
    },

    { name => 'xor',      format => '0sss 0001 0101 0ddd',
      code => [ 0x01, 0x50 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = Rd ^ Rs; flags(Rd)"
    },

    { name => 'xor.nowb', format => '0sss 0000 0101 0ddd',
      code => [ 0x00, 0x50 ], mask => [ 0x8f, 0xf8 ], len => 2,
      decode => \&decode_alu, encode => \&encode_alu,
      regrd => REG_RIGHT_8 | REG_LEFT_8, regwr => REG_FLAGS,
      pseudo => "T = Rd ^ Rs; flags(T)"
    },

    { name => 'ror',       format => '0000 0aaa 0110 0ddd',
      code => [ 0x00, 0x60 ], mask => [ 0xf8, 0xf8 ], len => 2,
      decode => \&decode_rot, encode => \&encode_rot,
      regrd => REG_RIGHT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = rotate_right(Rd, a + 1); flags(Rd)"
    },

    { name => 'rol',       format => '0000 0aaa 0110 1ddd',
      code => [ 0x00, 0x68 ], mask => [ 0xf8, 0xf8 ], len => 2,
      decode => \&decode_rot, encode => \&encode_rot,
      regrd => REG_RIGHT_8, regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = rotate_left(Rd, a + 1); flags(Rd)"
    },

    { name => 'jmp*',       format => '0000 0000 1001 1000',
      code => [ 0x00, 0x98 ], mask => [ 0xff, 0xff ], len => 6,
      decode => \&decode_jmp,
      tail => 1, jmp => 1, reserved => 1,
      pseudo => "R7 = target(mode, constant)"
    },

    { name => 'jmp*',       format => '0000 0001 1001 1000',
      code => [ 0x01, 0x98 ], mask => [ 0xff, 0xff ], len => 6,
      decode => \&decode_jmp,
      tail => 1, jmp => 1, reserved => 1,
      pseudo => "R7 = target(mode, constant)"
    },

    { name => 'jmp',       format => '0000 0002 1001 1000',
      code => [ 0x02, 0x98 ], mask => [ 0xff, 0xff ], len => 4,
      decode => \&decode_jmp, encode => \&encode_jmp2,
      tail => 1, jmp => 1,
      pseudo => "R7 = target(mode, constant)"
    },

    { name => 'call*',      format => '0000 0100 1001 1000',
      code => [ 0x04, 0x98 ], mask => [ 0xff, 0xff ], len => 6,
      decode => \&decode_jmp,
      regrd => REG_STACKPTR, regwr => REG_STACKPTR, call => 1, reserved => 1,
      pseudo => "store32(R6, R7); R6 += 4; flags(R6); R7 = target(mode, constant)"
    },

    { name => 'call*',      format => '0000 0101 1001 1000',
      code => [ 0x05, 0x98 ], mask => [ 0xff, 0xff ], len => 6,
      decode => \&decode_jmp,
      regrd => REG_STACKPTR, regwr => REG_STACKPTR, call => 1, reserved => 1,
      pseudo => "store32(R6, R7); R6 += 4; flags(R6); R7 = target(mode, constant)"
    },

    { name => 'call',      format => '0000 0110 1001 1000',
      code => [ 0x06, 0x98 ], mask => [ 0xff, 0xff ], len => 4,
      decode => \&decode_jmp, encode => \&encode_jmp2,
      regrd => REG_STACKPTR, regwr => REG_STACKPTR, call => 1,
      pseudo => "store32(R6, R7); R6 += 4; flags(R6); R7 = target(mode, constant)"
    },

    { name => 'ret',       format => '0000 0000 1010 0000',
      code => [ 0x00, 0xa0 ], mask => [ 0xff, 0xff ], len => 2,
      decode => \&decode_noarg, encode => \&encode_noarg,
      regrd => REG_STACKPTR, regwr => REG_STACKPTR, tail => 1,
      pseudo => "R6 -= 4; flags(R6); R7 = load32(R6)"
    },

    { name => 'st.rac',    format => 'iiii iiii 1010 1sss',
      code => [ 0x00, 0xa8 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_strac, encode => \&encode_strac,
      regrd => REG_RIGHT_8,
      pseuso => "store32(RAC_BASE + i * 4, Rs)"
    },

    { name => 'ld.rac',    format => 'iiii iiii 1011 0ddd',
      code => [ 0x00, 0xb0 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_ldrac, encode => \&encode_ldrac,
      regwr => REG_LEFT_8,
      pseuso => "Rd = load32(RAC_BASE + i * 4)"
    },

    { name => 'cst8',      format => 'iiii iiii 1011 1ddd',
      code => [ 0x00, 0xb8 ], mask => [ 0x00, 0xf8 ], len => 2,
      decode => \&decode_cst8, encode => \&encode_cst8,
      regwr => REG_RIGHT_8 | REG_FLAGS,
      pseuso => "Rd = zero_ext(i); flags(Rd)"
    },

    { name => 'ja',        format => 'iiii iiii 0111 0000',
      code => [ 0x00, 0x70 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      tail => 1, jmp => 1,
      pseudo => "R7 = R7 + sign_ext(i)"
    },

    { name => 'jz',       format => 'iiii iiii 0111 0001',
      code => [ 0x00, 0x71 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      regrd => REG_ZERO, jmp => 1,
      pseudo => "if (zero) R7 += sign_ext(i)"
    },

    { name => 'jnz',      format => 'iiii iiii 0111 0010',
      code => [ 0x00, 0x72 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      regrd => REG_ZERO, jmp => 1,
      pseudo => "if (!zero) R7 += sign_ext(i)"
    },

    { name => 'jc',       format => 'iiii iiii 0111 0011',
      code => [ 0x00, 0x73 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      regrd => REG_CARRY, jmp => 1,
      pseudo => "if (carry) R7 += sign_ext(i)"
    },

    { name => 'jnc',      format => 'iiii iiii 0111 0100',
      code => [ 0x00, 0x74 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      regrd => REG_CARRY, jmp => 1,
      pseudo => "if (!carry) R7 += sign_ext(i)"
    },

    { name => 'jneg',     format => 'iiii iiii 0111 0101',
      code => [ 0x00, 0x75 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      regrd => REG_NEG, jmp => 1,
      pseudo => "if (neg) R7 += sign_ext(i)"
    },

    { name => 'jpos',     format => 'iiii iiii 0111 0110',
      code => [ 0x00, 0x76 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc,
      regrd => REG_POS, jmp => 1,
      pseudo => "if (pos) R7 += sign_ext(i)"
    },

    { name => 'jnop*',     format => 'iiii iiii 0111 0111',
      code => [ 0x00, 0x77 ], mask => [ 0x00, 0xff ], len => 2,
      decode => \&decode_jc, encode => \&encode_jc, reserved => 1,
      pseudo => "nop()"
    },

    { name => 'cst32',     format => '0000 0000 1100 0ddd',
      code => [ 0x00, 0xc0 ], mask => [ 0xff, 0xf8 ], len => 6,
      decode => \&decode_cst32, encode => \&encode_cst32,
      regwr => REG_RIGHT_8 | REG_FLAGS,
      pseudo => "Rd = load32(R7 + 2); flags(Rd)"
    },

    { name => 'end',       format => '0000 0000 1111 1000',
      code => [ 0x00, 0xf8 ], mask => [ 0xff, 0xff ], len => 2,
      decode => \&decode_noarg, encode => \&encode_noarg,
      tail => 1, pseudo => "End of execution"
    },

    { name => 'sleep',    format => 'nnnn nnnn 0000 11nn',
      code => [ 0x00, 0x0c ], mask => [ 0x00, 0xfc ], len => 2,
      decode => \&decode_timer, encode => \&encode_timer,
      pseudo => "store32(RAC_STIMERCOMP, n); stimer_start()"
    },

    { name => 'waitmask',  format => 'nnnn nnnn 1110 10nn',
      code => [ 0x00, 0xec ], mask => [ 0x00, 0xfe ], len => 2,
      decode => \&decode_waitmask, encode => \&encode_waitmask,
      pseudo => "store32(RAC_WAITMASK, n); ?"
    },

    { name => 'wait1',      format => '0000 0000 1100 1ddd',
      code => [ 0x00, 0xc8 ], mask => [ 0xff, 0xf8 ], len => 2,
      decode => \&decode_wait1, encode => \&encode_wait1,
      regwr => REG_RIGHT_8,
      pseudo => "?"
    },

    { name => 'wait2',      format => '00?? 0000 1101 0000',
      code => [ 0x00, 0xd0 ], mask => [ 0xcf, 0xff ], len => 2,
      decode => \&decode_wait2, encode => \&encode_wait2,
      pseudo => "?"
    },

    { name => 'unknown',   format => '???? ???? ???? ????',
      code => [ 0, 0 ],       mask => [ 0, 0 ], len => 2,
      decode => \&decode_unknown, encode => \&encode_unknown,
      pseudo => "?"
    }
);

our %ops_by_name;

foreach my $op ( @ops ) {
    $ops_by_name{$op->{name}} = $op;
}

sub get_op
{
    my ( $bin, $pc ) = @_;
    foreach my $o ( @ops ) {
	my $m;
	my $i;
	for ( $i = 0; defined ($m = $o->{mask}->[$i]); $i++ ) {
	    last if ( $o->{mask}->[$i] & $bin->{$pc + $i} ) != $o->{code}->[$i];
	}
	if ( !defined $m ) {
	    return $o;
	}
    }

    return undef;
}

sub get_regs
{
    my ( $bin, $pc, $op ) = @_;

    my $rd;
    my $wr;

    my $s = sub {
	my $c = shift;
	my $r = 0;
	my $rr = $bin->{$pc + 1};
	my $rl = ($bin->{$pc} >> 4);

	$r |= 1 << ($rl & 7)  if ( $c & REG_LEFT_8 );
	$r |= 1 << ($rl & 15) if ( $c & REG_LEFT_16 );
	$r |= 1 << ($rr & 7)  if ( $c & REG_RIGHT_8 );
	$r |= 1 << ($rr & 15) if ( $c & REG_RIGHT_16 );
	$r |= 1 << 6          if ( $c & REG_STACKPTR );
	$r |= 1 << 16         if ( $c & REG_NEG );
	$r |= 1 << 17         if ( $c & REG_POS );
	$r |= 1 << 18         if ( $c & REG_ZERO );
	$r |= 1 << 19         if ( $c & REG_CARRY );

	return $r;
    };

    return ( $s->( $op->{regrd} ),
	     $s->( $op->{regwr} ) );
}

1;
