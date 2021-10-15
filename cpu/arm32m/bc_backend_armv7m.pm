
package bc_backend_armv7m;

use File::Basename;
use lib dirname (__FILE__);
use base 'bc_backend_armv6m';

use strict;

@bc_backend_armv6m::reg = ('r1', 'r2', 'r3', 'r5', 'r6', 'r7', 'r8', 'r12');
our @reg = @bc_backend_armv6m::reg;

$bc_backend_armv6m::max_op_regs = (scalar @reg) - 2;
$bc_backend_armv6m::caller_saved = 0x87;

sub out_begin {
    my ( $b ) = @_;
    return "    .cpu cortex-m3\n".
           "    .syntax unified\n".
           "    .section .rodata,\"a\"\n".
	   "    .globl ${main::bc_name}_bytecode\n".
           "    .balign 4\n".
           "${main::bc_name}_bytecode:\n".
	   # struct bc_descriptor_s
	   "    .long 1f\n".
	   "    .long _${main::bc_name}_bytecode\n".
	   "    .long $main::bcflags\n".

           "    .section .text\n".
           "    .code 16\n".
           "    .thumb_func\n".
           "    .globl _${main::bc_name}_bytecode\n".
           "    .func _${main::bc_name}_bytecode\n".
           "    .type _${main::bc_name}_bytecode, \%function\n".
           "_${main::bc_name}_bytecode:\n".
           "    push    {r4, r5, r6, r7, r8, lr}\n".
           # vm regs array
           "    mov     r4, r0\n".
	   # jump to vm start
           "    ldr     r0, [r4, #".(16 * 4)."]\n".
           "    orrs    r0, r0, #1\n".
           "    bx      r0\n".
           "1:\n";
}

sub out_mode {
    my ($thisop) = @_;
    return "    movs r0, ".$thisop->{args}->[0]."\n".
           "    strb r0, [r4, #".(18 * 4 + 1)."]\n";
}

sub out_end {
    return "    movs r0, #0\n".
           "    pop    {r4, r5, r6, r7, r8, pc}\n".
	   "    .ltorg\n";
}

sub out_abort {
    my ($thisop) = @_;
    return "    movs r0, #3\n".
           "    pop    {r4, r5, r6, r7, r8, pc}\n".
	   "    .ltorg\n";
}

sub out_eq0 {
    my ($thisop, $wi0) = @_;
    if ( $wi0 >= 6 ) {
        return "    tst $reg[$wi0], $reg[$wi0]\n".
               "    bne 1f\n";
    } else {
        return "    cbnz $reg[$wi0], 1f\n";
    }
}

sub out_neq0 {
    my ($thisop, $wi0) = @_;
    if ( $wi0 >= 6 ) {
        return "    tst $reg[$wi0], $reg[$wi0]\n".
               "    beq 1f\n";
    } else {
        return "    cbz $reg[$wi0], 1f\n";
    }
}

sub out_mul {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    mul $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_or {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    orrs $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_xor {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    eors $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_and {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    ands $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_shl {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    lsls $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_shr {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    lsrs $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_sha {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    asrs $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub parse_msbs {
}

sub out_msbs {
    my ($thisop, $wo, $wi) = @_;
    return "    clz $reg[$wo], $reg[$wi]\n".
           "    eor $reg[$wo], $reg[$wo], #31\n";
}

sub parse_div {
}

sub out_div {
    my ($thisop, $wo0, $wo1, $wi0, $wi1) = @_;
    if ( $wo0 == $wi0 ) {
        return "    mov r0, $reg[$wi0]\n".
               "    udiv $reg[$wo0], r0, $reg[$wi1]\n".
               "    mul $reg[$wo1], $reg[$wo0], $reg[$wi1]\n".
               "    sub $reg[$wo1], r0, $reg[$wo1]\n";
    } elsif ( $wo0 == $wi1 ) {
        return "    mov r0, $reg[$wi1]\n".
               "    udiv $reg[$wo0], $reg[$wi0], r0\n".
               "    mul r0, $reg[$wo0], r0\n".
               "    sub $reg[$wo1], $reg[$wi0], r0\n";
    } else {
        return "    udiv $reg[$wo0], $reg[$wi0], $reg[$wi1]\n".
               "    mul $reg[$wo1], $reg[$wo0], $reg[$wi1]\n".
               "    sub $reg[$wo1], $reg[$wi0], $reg[$wo1]\n";
    }
}

sub out_bitc {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    if ($x >= 12) {
        if ($x >= 16) {
            $r .= "    movs r0, #1\n".
                  "    lsls r0, #$x\n";
        } else {
            $r .= "    movw r0, #".(1 << $x)."\n";
        }
        $r .= "    bics $reg[$wo], r0\n";
    } else {
	$r .= "    bic $reg[$wo], $reg[$wi], #".(1 << $x)."\n";
    }
    return $r;
}

sub out_bits {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    if ($x >= 12) {
        if ($x >= 16) {
            $r .= "    movs r0, #1\n".
                  "    lsls r0, #$x\n";
        } else {
            $r .= "    movw r0, #".(1 << $x)."\n";
        }
        $r .= "    orrs $reg[$wo], r0\n";
    } else {
	$r .= "    orr $reg[$wo], $reg[$wi], #".(1 << $x)."\n";
    }
    return $r;
}

sub mov_imm {
    my ($d, $x) = @_;
    my $r = "    movw $d, #".($x & 0xffff)."\n";
    if ( $x & 0xffff0000 ) {
        $r .= "    movt $d, #".($x >> 16)."\n";
    }
    return $r;
}

sub out_ste {
    my ($thisop, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $x = $thisop->{args}->[2];
    my $r;
    if ($x < -128 || $x > 4095) {
        $r = mov_imm( "r0", $x );
	if ($s == 0) {
	    $r .= "    strb $reg[$wi0], [$reg[$wi1], r0]\n";
	} elsif ($s == 1) {
	    $r .= "    strh $reg[$wi0], [$reg[$wi1], r0]\n";
	} elsif ($s == 2) {
	    $r .= "    str $reg[$wi0], [$reg[$wi1], r0]\n";
	} else {
	    main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	    $r .= "    movs r12, #0\n".
		  "    str $reg[$wi0], [$reg[$wi1], r0]\n".
		  "    str r12, [$reg[$wi1], r0]\n";
	}
    } else {
	if ($s == 0) {
	    $r = "    strb $reg[$wi0], [$reg[$wi1], #$x]\n";
	} elsif ($s == 1) {
	    $r = "    strh $reg[$wi0], [$reg[$wi1], #$x]\n";
	} elsif ($s == 2) {
	    $r = "    str $reg[$wi0], [$reg[$wi1], #$x]\n";
	} else {
	    main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	    $r = "    movs r0, #0\n".
		 "    str $reg[$wi0], [$reg[$wi1], #$x]\n".
		 "    str r0, [$reg[$wi1], #$x]\n";
	}
    }
    return $r;
}

sub out_lde {
    my ($thisop, $wo, $wi) = @_;
    my $s = $thisop->{width};
    my $x = $thisop->{args}->[2];
    my $r;
    if ($x < -128 || $x > 4095) {
        $r = mov_imm( "r0", $x );
	if ($s == 0) {
	    $r .= "    ldrb $reg[$wo], [$reg[$wi], r0]\n";
	} elsif ($s == 1) {
	    $r .= "    ldrh $reg[$wo], [$reg[$wi], r0]\n";
	} elsif ($s == 2) {
	    $r .= "    ldr $reg[$wo], [$reg[$wi], r0]\n";
	} else {
	    main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	    $r .= "    movs r0, #0\n".
		  "    ldr $reg[$wo], [$reg[$wi], r0]\n";
	}
    } else {
	if ($s == 0) {
	    $r = "    ldrb $reg[$wo], [$reg[$wi], #$x]\n";
	} elsif ($s == 1) {
	    $r = "    ldrh $reg[$wo], [$reg[$wi], #$x]\n";
	} elsif ($s == 2) {
	    $r = "    ldr $reg[$wo], [$reg[$wi], #$x]\n";
	} else {
	    main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	    $r = "    movs r0, #0\n".
		 "    ldr $reg[$wo], [$reg[$wi], #$x]\n";
	}
    }
    return $r;
}

sub out_cst {
    my ($thisop, $wo) = @_;
    my $x = ($thisop->{args}->[1] << $thisop->{args}->[2]) & 0xffffffff;
    if ( $thisop->{width} >= 2 ) {
        main::warning($thisop, "64 bit constant truncated to 32 bits.\n");
    }
    return mov_imm( $reg[$wo], $x );
}

sub out_calla {
    return bc_backend_armv6m::out_call8(shift);
}

sub out_callr {
    return bc_backend_armv6m::out_call8(shift);
}

sub out_jmpa {
    return bc_backend_armv6m::out_jmp8(shift);
}

sub out_jmpr {
    return bc_backend_armv6m::out_jmp8(shift);
}

sub out_pick {
    my ($thisop, @w) = @_;

    return "    adds r0, r4, #".($thisop->{packout_reg} * 4)."\n".
           "    movw r1, #".($thisop->{args}->[1])."\n".
           "    bl bc_pick\n";
}

sub out_place {
    my ($thisop, @w) = @_;

    return "    adds r0, r4, #".($thisop->{packout_reg} * 4)."\n".
           "    movw r1, #".($thisop->{args}->[1])."\n".
           "    bl bc_place\n";
}

return 1;

