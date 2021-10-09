
package bc_backend_armv6m;

use strict;

our @reg = ( 'r1', 'r2', 'r3', 'r5', 'r6', 'r7' );
our $caller_saved = 0x0007; # vm working regs are caller saved
our $max_op_regs = (scalar @reg) - 2;

sub out_begin {
    my ( $b ) = @_;
    return "    .cpu cortex-m0\n".
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
           "    push    {r4, r5, r6, r7, lr}\n".
           # vm regs array
           "    mov     r4, r0\n".
	   # jump to vm start
           "    ldr     r0, [r4, #".(16 * 4)."]\n".
           "    movs    r1, #1\n".
           "    orrs    r0, r1\n".
           "    bx      r0\n".
           "1:\n";
}

sub out_eof {
    return "    .balign 4\n".
           "Lbytecode_end:\n".
           "    pop    {r4, r5, r6, r7, pc}\n".
           "    .endfunc\n".
           "    .size ${main::bc_name}_bytecode, . - _${main::bc_name}_bytecode\n";
}

sub out_load {
    my ($b, $ri, $wo) = @_;
    return "    ldr $reg[$wo], [r4, #".($ri * 4)."]\n";
}

sub out_store {
    my ($b, $ro, $wi) = @_;
    return "    str $reg[$wi], [r4, #".($ro * 4)."]\n";
}

sub out_custom {
    my ($thisop) = @_;
    my $op = $thisop->{code} | $thisop->{op}->{code};
    return
        "    bl arm32m_bc_trampoline\n".
        "    .2byte ".($op & 0xffff)."\n";    # opcode
}

sub out_custom_cond {
    my ($thisop) = @_;
    my $op = $thisop->{code} | $thisop->{op}->{code};
    return
        "    bl arm32m_bc_trampoline_cond\n".
        "    .2byte ".($op & 0xffff)."\n".    # opcode
        "    .2byte 1f - 2f\n".               # conditional skip amount
        "2:\n";
}

sub out_mode {
    my ($thisop) = @_;
    return "    movs r0, ".$thisop->{args}->[0]."\n".
           "    adds r4, #".(18 * 4 + 1)."\n".
           "    strb r0, [r4]\n".
           "    subs r4, #".(18 * 4 + 1)."\n";
}

sub out_end {
    return "    movs r0, #0\n".
           "    pop    {r4, r5, r6, r7, pc}\n".
	   "    .ltorg\n";
}

sub parse_dump {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;
}

sub out_dump {
    return "    mov r0, r4\n".
	   "    adr r1, 2f\n".
           "    str r1, [r4, #".(16 * 4)."]\n".
           "    movs r1, #1\n".
           "    bl bc_dump\n".
	   "    .balign 4\n".
	   "2:\n";
}

sub out_nop {
}

sub out_abort {
    return "    movs r0, #3\n".
           "    pop    {r4, r5, r6, r7, pc}\n".
	   "    .ltorg\n";
}

sub out_die {
    my ($thisop) = @_;
    return "    bl abort\n".
	   "    .ltorg\n";
}

sub out_trace {
}

sub out_add8 {
    my ($thisop, $wo, $wi) = @_;
    my $r;
    my $x = $thisop->{args}->[1];
    $r .= "    mov $reg[$wo], $reg[$wi]\n" if ( $wi != $wo );
    if ( $thisop->{args}->[1] >= 0 ) {
	$r .= "    adds $reg[$wo], #$x\n";
    } else {
	$r .= "    subs $reg[$wo], #".(0xff & -$x)."\n";
    }
    return $r;
}

sub out_cst8 {
    my ($thisop, $wo) = @_;
    return "    movs $reg[$wo], #$thisop->{args}->[1]\n";
}

sub out_jmp8 {
    my ($thisop) = @_;
    return "    b $thisop->{args}->[0]\n".
	   "    .ltorg\n";
}

sub out_call8 {
    my ($thisop) = @_;
    return "    adr r0, 2f\n".
           "    adds r0, #1\n".
           "    str r0, [r4, #".($thisop->{out}->[0] * 4)."]\n".
	   "    b $thisop->{args}->[1]\n".
           "    .balign 4\n".
           "2:\n"
           ;
}

sub out_calla {
    my ($thisop) = @_;
    return "    adr r0, 2f\n".
           "    adds r0, #1\n".
           "    str r0, [r4, #".($thisop->{out}->[0] * 4)."]\n".
           "    ldr r0, = $thisop->{args}->[1] + 1\n".
	   "    bx r0\n".
           "    .balign 4\n".
           "2:\n";
}

sub out_callr {
    return out_calla(shift);
}

sub out_jmpa {
    my ($thisop) = @_;
    return "    ldr r0, = $thisop->{args}->[0] + 1\n".
	   "    bx r0\n".
	   "    .ltorg\n";
}

sub out_jmpr {
    return out_jmpa(shift);
}

sub out_ret {
    my ($thisop, $wi) = @_;
    return "    bx $reg[$wi]\n".
	   "    .ltorg\n";
}

sub out_jmp {
    my ($thisop, $wi) = @_;
    return "    adds $reg[$wi], #1\n".   # set thumb mode bit
           "    bx $reg[$wi]\n".
	   "    .ltorg\n";
}

sub parse_call {
    my ($thisop) = @_;
    $thisop->{wbin} = 1;
}

sub out_call {
    my ($thisop, $wi) = @_;
    return "    adr r0, 2f\n".
           "    adds r0, #1\n".
           "    str r0, [r4, #".($thisop->{out}->[0] * 4)."]\n".
	   "    adds $reg[$wi], #1\n".   # set thumb mode bit
           "    bx $reg[$wi]\n".
           "    .balign 4\n".
           "2:\n";
}

sub out_pack {
    my ($thisop, @w) = @_;

    my $sym = $main::packops{$thisop->{name}};
    return "    # pack: nothing to do\n" if !$sym;

    if ( $thisop->{count} > $max_op_regs ) {
        return "    mov r0, r4\n".
               "    movs r1, #".($thisop->{packout_reg} + $thisop->{count} * 16)."\n".
               "    bl $sym\n";
    } else {
        my $r;
        for ( my $i = 0; $i < $thisop->{count}; $i++ ) {
            my %op = (
                'bc_pack_op8' => sub {
                    my $o = (4 * $thisop->{packout_reg} + $i);
                    if ( $o > 31 ) {
                        $r .= "    movs r0, #".$o."\n";
                        $r .= "    strb $reg[$w[$i]], [r4, r0]\n";
                    } else {
                        $r .= "    strb $reg[$w[$i]], [r4, #".$o."]\n";
                    }
                }, 'bc_pack_op16' => sub {
                    $r .= "    strh $reg[$w[$i]], [r4, #".(4 * $thisop->{packout_reg} + $i * 2)."]\n";
                }, 'bc_swap_pack_op16' => sub {
                    $r .= "    rev16 r0, $reg[$w[$i]]\n";
                    $r .= "    strh r0, [r4, #".(4 * $thisop->{packout_reg} + $i * 2)."]\n";
                }, 'bc_swap_pack_op32' => sub {
                    $r .= "    rev r0, $reg[$w[$i]]\n";
                    $r .= "    str r0, [r4, #".(4 * $thisop->{packout_reg} + $i * 4)."]\n";
                });
            $op{$sym}->();
        }
        return $r;
    }
}

sub out_unpack {
    my ($thisop, @w) = @_;

    my $sym = $main::packops{$thisop->{name}};
    return "    # unpack: nothing to do\n" if !$sym;

    if ( $thisop->{count} > $max_op_regs ) {
        return "    mov r0, r4\n".
               "    movs r1, #".($thisop->{packin_reg} + $thisop->{count} * 16)."\n".
               "    bl $sym\n";
    } else {
        my $r;
        for ( my $i = 0; $i < $thisop->{count}; $i++ ) {
            my %op = (
                'bc_unpack_op8' => sub {
                    my $o = (4 * $thisop->{packin_reg} + $i);
                    if ( $o > 31 ) {
                        $r .= "    movs r0, #".$o."\n";
                        $r .= "    ldrb $reg[$w[$i]], [r4, r0]\n";
                    } else {
                        $r .= "    ldrb $reg[$w[$i]], [r4, #".$o."]\n";
                    }
                }, 'bc_unpack_op16' => sub {
                    $r .= "    ldrh $reg[$w[$i]], [r4, #".(4 * $thisop->{packin_reg} + $i * 2)."]\n";
                }, 'bc_unpack_swap_op16' => sub {
                    $r .= "    ldrh r0, [r4, #".(4 * $thisop->{packin_reg} + $i * 2)."]\n";
                    $r .= "    rev16 $reg[$w[$i]], r0\n";
                }, 'bc_unpack_swap_op32' => sub {
                    $r .= "    ldr r0, [r4, #".(4 * $thisop->{packin_reg} + $i * 4)."]\n";
                    $r .= "    rev $reg[$w[$i]], r0\n";
                });
            $op{$sym}->();
        }
        return $r;
    }
}

sub parse_pack {
    my ($thisop) = @_;

    my $sym = $main::packops{$thisop->{name}};

    if ( !$sym ) {
        $thisop->{flushin} = (1 << $thisop->{count}) - 1;
    } elsif ( $thisop->{count} > $max_op_regs ) {
        # use function call
        $thisop->{flushin} = (1 << $thisop->{count}) - 1;
        $thisop->{clobber} = $caller_saved;
    } else {
        # prevent overwritting of packed data
        $thisop->{wbin} = (1 << $thisop->{count}) - 1;
    }
}

sub parse_unpack {
    my ($thisop) = @_;

    my $sym = $main::packops{$thisop->{name}};

    if ( !$sym ) {
        $thisop->{reloadout} = (1 << $thisop->{count}) - 1;
    } elsif ( $thisop->{count} > $max_op_regs ) {
        # use function call
        $thisop->{reloadout} = (1 << $thisop->{count}) - 1;
        $thisop->{clobber} = $caller_saved;
    }
}

sub parse_swap {
    my ($thisop) = @_;

    if ( $thisop->{name} =~ /le$/ ) {
        $thisop->{nop} = 1;
    }
}

sub out_swap {
    my ($thisop, $wo, $wi) = @_;
    my $r;

    if ( !defined $wi ) {
        $r = "    # swaple: nothing to do\n";
    } elsif ( $thisop->{name} =~ /^swap32/ ) {
        $r = "    rev $reg[$wo], $reg[$wi]\n";
    } else {
        $r = "    rev16 $reg[$wo], $reg[$wi]\n";
    }

    return $r;
}

sub out_loop {
    my ($thisop, $wi) = @_;
    if ($thisop->{target}->{addr} < $thisop->{addr}) {
        return "    subs $reg[$wi], #1\n".
               "    str $reg[$wi], [r4, #".($thisop->{in}->[0] * 4)."]\n".
               "    bne $thisop->{args}->[1]\n";
    } else {
        return
               "    tst $reg[$wi], $reg[$wi]\n".
               "    beq $thisop->{args}->[1]\n".
               "    subs $reg[$wi], #1\n";
    }
}

sub out_eq {
    my ($thisop, $wi0, $wi1) = @_;
    return "    cmp $reg[$wi0], $reg[$wi1]\n".
	   "    bne 1f\n";
}

sub out_eq0 {
    my ($thisop, $wi0) = @_;
    return "    tst $reg[$wi0], $reg[$wi0]\n".
	   "    bne 1f\n";
}

sub out_neq {
    my ($thisop, $wi0, $wi1) = @_;
    return "    cmp $reg[$wi0], $reg[$wi1]\n".
	   "    beq 1f\n";
}

sub out_neq0 {
    my ($thisop, $wi0) = @_;
    return "    tst $reg[$wi0], $reg[$wi0]\n".
	   "    beq 1f\n";
}

sub out_lt {
    my ($thisop, $wi0, $wi1) = @_;
    return "    cmp $reg[$wi0], $reg[$wi1]\n".
           "    bhs 1f\n";
}

sub out_lteq {
    my ($thisop, $wi0, $wi1) = @_;
    return "    cmp $reg[$wi0], $reg[$wi1]\n".
           "    bhi 1f\n";
}

sub out_lts {
    my ($thisop, $wi0, $wi1) = @_;
    return "    cmp $reg[$wi0], $reg[$wi1]\n".
           "    bge 1f\n";
}

sub out_lteqs {
    my ($thisop, $wi0, $wi1) = @_;
    return "    cmp $reg[$wi0], $reg[$wi1]\n".
           "    bgt 1f\n";
}

sub mov_op {
    my ($wo, $wi0, $wi1, $op, $rop) = @_;

    if ($wo == $wi0) {
	return "    $op $reg[$wo], $reg[$wi1]\n";
    } elsif ($wo == $wi1) {
	if ( defined $rop ) {
	    return "    $rop $reg[$wo], $reg[$wi0]\n";
	} else {
	    return "    mov r0, $reg[$wi0]\n".
		   "    $op r0, $reg[$wi1]\n".
		   "    mov $reg[$wo], r0\n";
	}
    } else {
	return "    mov $reg[$wo], $reg[$wi0]\n".
	       "    $op $reg[$wo], $reg[$wi1]\n";
    }
}

sub out_add {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    adds $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_sub {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    subs $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_neg {
    my ($thisop, $wo, $wi) = @_;
    return "    rsbs $reg[$wo], $reg[$wi], #0\n";
}

sub out_mov {
    my ($thisop, $wo, $wi) = @_;
    return "    mov $reg[$wo], $reg[$wi]\n";
}

sub out_mul {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "muls", "muls");
}

sub out_or {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "orrs", "orrs");
}

sub out_xor {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "eors", "eors");
}

sub out_and {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "ands", "ands");
}

sub out_not {
    my ($thisop, $wo, $wi) = @_;
    return "    mvns $reg[$wo], $reg[$wi]\n";
}

sub parse_msbs {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;    # some vm working regs are caller saved
}

sub out_msbs {
    my ($thisop, $wo, $wi) = @_;
    return "    mov r0, $reg[$wi]\n".
           "    bl __clzsi2\n".
           "    movs $reg[$wo], #31\n".
           "    eors $reg[$wo], r0\n";
}

sub parse_div {
    my ($thisop) = @_;

    # no need to wb 2nd input, will be overwritten as 2nd output
    $thisop->{wbin} = 2;
    # 2nd output is written by the function call
    $thisop->{reloadout} = 2;
    $thisop->{clobber} = $caller_saved;
}

sub out_div {
    my ($thisop, $wo0, $wo1, $wi0, $wi1) = @_;
    my $r = "    mov r0, $reg[$wi0]\n";
    $r .= "    mov r1, $reg[$wi1]\n" if $reg[$wi1] ne 'r1';
    $r .= "    mov r2, r4\n".
          "    adds r2, #".($thisop->{out}->[1] * 4)."\n".
          "    bl arm32m_bc_div32\n".
          "    mov $reg[$wo0], r0\n";
    return $r;
}

sub parse_ccall {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;    # some vm working regs are caller saved
}

sub out_ccall {
    my ($thisop, $wo, $wi0) = @_;
    my $r = "    mov r0, r4\n";
    $r .= "    blx $reg[$wi0]\n";
    $r .= "    mov $reg[$wo], r0\n";
    return $r;
}

sub parse_rand {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;    # some vm working regs are caller saved
}

sub out_rand {
    my ($thisop, $wo) = @_;

    return "    bl rand_64\n".
           "    movs $reg[$wo], r0\n";
}

sub out_shl {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "lsls");
}

sub out_shr {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "lsrs");
}

sub out_sha {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return mov_op($wo, $wi0, $wi1, "asrs");
}

sub out_tstc {
    my ($thisop, $wi) = @_;
    my $x = $thisop->{args}->[1] + 1;
    return "    lsrs r0, $reg[$wi], #$x\n".
	   "    bcs 1f\n";
}

sub out_tsts {
    my ($thisop, $wi) = @_;
    my $x = $thisop->{args}->[1] + 1;
    return "    lsrs r0, $reg[$wi], #$x\n".
	   "    bcc 1f\n";
}

sub out_bitc {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    $r .= "    mov $reg[$wo], $reg[$wi]\n" if ( $wi != $wo );
    if ($x >= 8) {
	$r .= "    movs r0, #1\n".
	      "    lsls r0, #$x\n";
    } else {
	$r .= "    movs r0, #".(1 << $x)."\n";
    }
    return $r."    bics $reg[$wo], r0\n";
}

sub out_bits {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    $r .= "    mov $reg[$wo], $reg[$wi]\n" if ( $wi != $wo );
    if ($x >= 8) {
	$r .= "    movs r0, #1\n".
	      "    lsls r0, #$x\n";
    } else {
	$r .= "    movs r0, #".(1 << $x)."\n";
    }
    return $r."    orrs $reg[$wo], r0\n";
}

sub out_shil {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    $r .= "    mov $reg[$wo], $reg[$wi]\n" if ( $wi != $wo );
    return $r."    lsls $reg[$wo], #$x\n";
}

sub out_shir {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    $r .= "    mov $reg[$wo], $reg[$wi]\n" if ( $wi != $wo );
    return $r."    lsrs $reg[$wo], #$x\n";
}

sub out_shia {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    my $r;
    $r .= "    mov $reg[$wo], $reg[$wi]\n" if ( $wi != $wo );
    return $r."    asrs $reg[$wo], #$x\n";
}

sub out_extz {
    my ($thisop, $wo, $wi) = @_;
    my $r;
    my $x = $thisop->{args}->[1];
    if ( $x == 7 ) {
        $r = "    uxtb $reg[$wo], $reg[$wi]\n";
    } elsif ( $x == 15 ) {
        $r = "    uxth $reg[$wo], $reg[$wi]\n";
    } elsif ( $x < 31 ) {
        $x = 31 - $x;
        $r = "    lsls r0, $reg[$wi], $x\n".
             "    lsrs $reg[$wo], r0, $x\n";
    }
    return $r;
}

sub out_exts {
    my ($thisop, $wo, $wi) = @_;
    my $r;
    my $x = $thisop->{args}->[1];
    if ( $x == 7 ) {
        $r = "    sxtb $reg[$wo], $reg[$wi]\n";
    } elsif ( $x == 15 ) {
        $r = "    sxth $reg[$wo], $reg[$wi]\n";
    } elsif ( $x < 31 ) {
        $x = 31 - $x;
        $r = "    lsls r0, $reg[$wi], $x\n".
             "    asrs $reg[$wo], r0, $x\n";
    }
    return $r;
}

sub out_st {
    my ($thisop, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    strb $reg[$wi0], [$reg[$wi1]]\n";
    } elsif ($s == 1) {
        $r = "    strh $reg[$wi0], [$reg[$wi1]]\n";
    } elsif ($s == 2) {
        $r = "    str $reg[$wi0], [$reg[$wi1]]\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	$r = "    movs r0, #0\n".
 	     "    str $reg[$wi0], [$reg[$wi1]]\n".
 	     "    str r0, [$reg[$wi1], #4]\n";
    }
    return $r;
}

sub out_ste {
    my ($thisop, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $x = $thisop->{args}->[2];
    my $r;
    if ($x < 0 || $x > (31 << $s)) {
	if ($x >= 0 && $x < 255) {
	    $r = "    movs r0, #$x\n";
	} else {
	    $r = "    ldr r0, = ".($x & 0xffffffff)."\n";
	}
	if ($s == 0) {
	    $r .= "    strb $reg[$wi0], [$reg[$wi1], r0]\n";
	} elsif ($s == 1) {
	    $r .= "    strh $reg[$wi0], [$reg[$wi1], r0]\n";
	} elsif ($s == 2) {
	    $r .= "    str $reg[$wi0], [$reg[$wi1], r0]\n";
	} else {
	    main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	    $r .= "    mov r12, r1\n".
                  "    movs r1, #0\n".
		  "    str $reg[$wi0], [$reg[$wi1], r0]\n".
		  "    str r1, [$reg[$wi1], r0]\n".
                  "    mov r1, r12\n";
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

sub out_sti {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    strb $reg[$wi0], [$reg[$wi1]]\n";
    } elsif ($s == 1) {
        $r = "    strh $reg[$wi0], [$reg[$wi1]]\n";
    } elsif ($s == 2) {
        $r = "    str $reg[$wi0], [$reg[$wi1]]\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	$r = "    movs r0, #0\n".
 	     "    str $reg[$wi0], [$reg[$wi1]]\n".
 	     "    str r0, [$reg[$wi1], #4]\n";
    }
    $r .= "    mov $reg[$wo], $reg[$wi1]\n" if ( $wi1 != $wo );
    $r .= "    adds $reg[$wo], #".(1 << $s)."\n";
    return $r;
}

sub out_std {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r;
    $r .= "    mov $reg[$wo], $reg[$wi1]\n" if ( $wi1 != $wo );
    $r .= "    subs $reg[$wo], #".(1 << $s)."\n";
    if ($s == 0) {
        $r .= "    strb $reg[$wi0], [$reg[$wo]]\n";
    } elsif ($s == 1) {
        $r .= "    strh $reg[$wi0], [$reg[$wo]]\n";
    } elsif ($s == 2) {
        $r .= "    str $reg[$wi0], [$reg[$wo]]\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
	$r .= "    movs r0, #0\n".
	      "    str $reg[$wi0], [$reg[$wo]]\n".
	      "    str r0, [$reg[$wo], #4]\n";
    }
    return $r;
}

sub out_ld {
    my ($thisop, $wo, $wi) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    ldrb $reg[$wo], [$reg[$wi]]\n";
    } elsif ($s == 1) {
        $r = "    ldrh $reg[$wo], [$reg[$wi]]\n";
    } elsif ($s == 2) {
        $r = "    ldr $reg[$wo], [$reg[$wi]]\n";
    } else {
        main::warning($thisop, "64 bit load truncated to 32 bits.\n");
	$r = "    ldr $reg[$wo], [$reg[$wi]]\n";
    }
    return $r;
}

sub out_lde {
    my ($thisop, $wo, $wi) = @_;
    my $s = $thisop->{width};
    my $x = $thisop->{args}->[2];
    my $r;
    if ($x < 0 || $x > (31 << $s)) {
	if ($x >= 0 && $x < 255) {
	    $r = "    movs r0, #$x\n";
	} else {
	    $r = "    ldr r0, = ".($x & 0xffffffff)."\n";
	}
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

sub out_ldi {
    my ($thisop, $wo0, $wo1, $wi) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    ldrb $reg[$wo0], [$reg[$wi]]\n";
    } elsif ($s == 1) {
        $r = "    ldrh $reg[$wo0], [$reg[$wi]]\n";
    } elsif ($s == 2) {
        $r = "    ldr $reg[$wo0], [$reg[$wi]]\n";
    } else {
        main::warning($thisop, "64 bit load truncated to 32 bits.\n");
	$r = "    ldr $reg[$wo0], [$reg[$wi]]\n";
    }
    $r .= "    mov $reg[$wo1], $reg[$wi]\n" if ( $wi != $wo1 );
    $r .= "    adds $reg[$wo1], #".(1 << $s)."\n";
    return $r;
}

sub out_cst {
    my ($thisop, $wo) = @_;
    my $r;
    my $x = ($thisop->{args}->[1] << $thisop->{args}->[2]) & 0xffffffff;
    if ( $thisop->{width} >= 2 ) {
        main::warning($thisop, "64 bit constant truncated to 32 bits.\n");
    }
    return "    ldr $reg[$wo], = $x\n";
}

sub out_gaddr {
    my ($thisop, $wo) = @_;
    return "    ldr $reg[$wo], = $thisop->{args}->[1]\n";
}

sub out_laddra {
    out_gaddr( @_ );
}

sub out_laddrr {
    out_gaddr( @_ );
}

sub out_data {
    my ($thisop) = @_;

    my $r;

    if ($thisop->{width}) {
        $r = sprintf("    .balign %u ;", 1 << $thisop->{width});
    }

    $r .= join('', map {
                      sprintf("    .byte 0x%02x ;", $_);
                    } main::repack_data( $thisop ) )."\n";

    return $r;
}

sub write {
    main::write_asm( scalar @reg );
}

return 1;

