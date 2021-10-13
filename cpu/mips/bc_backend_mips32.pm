
package bc_backend_mips32;

use strict;

our @reg = ( '$8', '$9', '$10', '$11', '$12', '$13', '$14', '$15' );
our $caller_saved = 0xff; # vm working regs are caller saved
our $max_op_regs = (scalar @reg) - 2;

sub out_begin {
    my ( $b ) = @_;
    return "    .section .rodata,\"a\"\n".
	   "    .globl ${main::bc_name}_bytecode\n".
           "    .balign 4\n".
           "${main::bc_name}_bytecode:\n".
	   # struct bc_descriptor_s
	   "    .long 1f\n".
	   "    .long _${main::bc_name}_bytecode\n".
	   "    .long $main::bcflags\n".

           "    .section .text,\"ax\",\@progbits\n".
           "    .set noat\n".
           "    .globl _${main::bc_name}_bytecode\n".
           "    .func _${main::bc_name}_bytecode\n".
           "    .type _${main::bc_name}_bytecode, \%function\n".
           "_${main::bc_name}_bytecode:\n".
           "    addiu   \$sp, \$sp, -24\n".
           "    sw      \$31, 20(\$sp)\n".
           # vm regs array
           "    sw      \$17, 16(\$sp)\n".
           "    move    \$17, \$4\n".
	   # jump to vm start
           "    lw      \$at, ".(16 * 4)."(\$17)\n".
           "    jr      \$at\n".
           "1:\n";
}

sub out_eof {
    return "    .balign 4\n".  # end
           "Lbytecode_end:\n".
           "    lw      \$31, 20(\$sp)\n".
           "    lw      \$17, 16(\$sp)\n".
           "    addiu   \$sp, \$sp, 24\n".
           "    jr      \$31\n".
           "    .endfunc\n".
           "    .size _${main::bc_name}_bytecode, . - _${main::bc_name}_bytecode\n";
}

sub out_load {
    my ($b, $ri, $wo) = @_;
    return "    lw $reg[$wo], ".($ri * 4)."(\$17)\n";
}

sub out_store {
    my ($b, $ro, $wi) = @_;
    return "    sw $reg[$wi], ".($ro * 4)."(\$17)\n";
}

sub out_custom {
    my ($thisop) = @_;
    my $op = $thisop->{code} | $thisop->{op}->{code};
    return "    ori \$v0, \$0, $op\n".
           "    .set noreorder\n".
           "    bal Lbytecode_end\n".
           # resume address
           "    sw \$31, ".(16 * 4)."(\$17)\n".
           "    .set reorder\n";
}

sub out_custom_cond {
    my ($thisop) = @_;
    my $op = $thisop->{code} | $thisop->{op}->{code};
    return # skip amount
           "    ori \$at, \$0, 1f - 2f\n".
           "    sb \$at, ".(18 * 4)."(\$17)\n".
           "    ori \$v0, \$0, $op\n".
           "    .set noreorder\n".
           "    bal Lbytecode_end\n".
           # resume address
           "    sw \$31, ".(16 * 4)."(\$17)\n".
           "    .set reorder\n".
           "2:";
}

sub out_mode {
    my ($thisop) = @_;
    return "    addiu \$v0, \$0, ".$thisop->{args}->[0]."\n".
           "    sb \$v0, ".(18 * 4 + 1)."(\$17)\n";
}

sub out_end {
    return "    move \$v0, \$0\n".
           "    sw \$v0, ".(16 * 4)."(\$17)\n".
           "    b Lbytecode_end\n";
}

sub out_abort {
    return "    addiu \$v0, \$0, 3\n".
           "    b Lbytecode_end\n";
}

sub parse_dump {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;
}

sub out_dump {
    my ($thisop) = @_;
    return "    li \$a1, 1\n".
           "    move \$a0, \$17\n".
           "    .set noreorder\n".
           "    jal bc_dump\n".
           "    sw \$31, ".(16 * 4)."(\$17)\n".
           "    .set reorder\n";
}

sub out_nop {
}

sub out_die {
    my ($thisop) = @_;
    return "    jal abort\n";
}

sub out_trace {
}

sub out_add8 {
    my ($thisop, $wo, $wi) = @_;
    return "    addiu $reg[$wo], $reg[$wi], $thisop->{args}->[1]\n";
}

sub out_cst8 {
    my ($thisop, $wo) = @_;
    return "    li $reg[$wo], $thisop->{args}->[1]\n";
}

sub out_jmp8 {
    my ($thisop) = @_;
    return "    b $thisop->{args}->[0]\n";
}

sub parse_call {
    my ($thisop) = @_;
    $thisop->{wbin} = 1;
}

sub out_call {
    my ($thisop, $wi) = @_;
    return "    .set noreorder\n".
           "    jalr $reg[$wi]\n".
           "    sw \$31, ".($thisop->{out}->[0] * 4)."(\$17)\n".
           "    .set reorder\n"
           ;
}

sub out_call8 {
    my ($thisop) = @_;
    return "    .set noreorder\n".
           "    bal $thisop->{args}->[1]\n".
           "    sw \$31, ".($thisop->{out}->[0] * 4)."(\$17)\n".
           "    .set reorder\n"
           ;
}

sub out_calla {
    my ($thisop) = @_;
    return "    .set noreorder\n".
           "    bal $thisop->{args}->[1]\n".
           "    sw \$31, ".($thisop->{out}->[0] * 4)."(\$17)\n".
           "    .set reorder\n"
}

sub out_callr {
    return out_calla(shift);
}

sub out_jmpa {
    my ($thisop) = @_;
    return "    j $thisop->{args}->[0]\n";
}

sub out_jmpr {
    return out_jmpa(shift);
}

sub out_ret {
    my ($thisop, $wi) = @_;
    return "    jr $reg[$wi]\n";
}

sub out_jmp {
    return out_ret( @_ );
}

sub parse_pick {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;
}

sub out_pick {
    my ($thisop, @w) = @_;

    return "    addiu \$a0, \$17, ".($thisop->{packout_reg} * 4)."\n".
           "    li \$a1, ".($thisop->{args}->[1])."\n".
           "    jal bc_pick\n";
}

sub parse_place {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;
}

sub out_place {
    my ($thisop, @w) = @_;

    return "    addiu \$a0, \$17, ".($thisop->{packout_reg} * 4)."\n".
           "    li \$a1, ".($thisop->{args}->[1])."\n".
           "    jal bc_place\n";
}

sub out_pack {
    my ($thisop, @wi) = @_;

    my $sym = $main::packops{$thisop->{name}};
    return "# pack: nothing to do\n" if !$sym;

    if ( $thisop->{count} > 6 || $sym =~ /swap/ ) {
        return "    move \$a0, \$17\n".
               "    li \$a1, ".($thisop->{packout_reg} + $thisop->{count} * 16)."\n".
               "    jal $sym\n";
    } else {
        my $r;
        for ( my $i = 0; $i < $thisop->{count}; $i++ ) {
            my %op = (
                'bc_pack_op8' => sub {
                    $r .= "    sb $reg[$wi[$i]], ".(4 * $thisop->{packout_reg} + $i)."(\$17)\n";
                }, 'bc_pack_op16' => sub {
                    $r .= "    sh $reg[$wi[$i]], ".(4 * $thisop->{packout_reg} + $i * 2)."(\$17)\n";
                });
            $op{$sym}->();
        }
        return $r;
    }
}

sub out_unpack {
    my ($thisop, @wi) = @_;

    my $sym = $main::packops{$thisop->{name}};
    return "# unpack: nothing to do\n" if !$sym;

    if ( $thisop->{count} > 6 || $sym =~ /swap/ ) {
        return "    move \$a0, \$17\n".
               "    li \$a1, ".($thisop->{packin_reg} + $thisop->{count} * 16)."\n".
               "    jal $sym\n";
    } else {
        my $r;
        for ( my $i = 0; $i < $thisop->{count}; $i++ ) {
            my %op = (
                'bc_unpack_op8' => sub {
                    $r .= "    lbu $reg[$wi[$i]], ".(4 * $thisop->{packin_reg} + $i)."(\$17)\n";
                }, 'bc_unpack_op16' => sub {
                    $r .= "    lhu $reg[$wi[$i]], ".(4 * $thisop->{packin_reg} + $i * 2)."(\$17)\n";
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
    } elsif ( $thisop->{count} > $max_op_regs || $sym =~ /swap/ ) {
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
    } elsif ( $thisop->{count} > $max_op_regs || $sym =~ /swap/ ) {
        # use function call
        $thisop->{reloadout} = (1 << $thisop->{count}) - 1;
        $thisop->{clobber} = $caller_saved;
    }
}

sub parse_swap {
    my ($thisop) = @_;

    if ( ($thisop->{name} =~ /le$/ && $main::backend_endian eq 'little') ||
         ($thisop->{name} =~ /be$/ && $main::backend_endian eq 'big') ) {
        $thisop->{nop} = 1;
    } else {
        # use function call
        $thisop->{flushin} = 1;
        $thisop->{reloadout} = 1;
        $thisop->{clobber} = $caller_saved;
    }
}

sub out_swap {
    my ($thisop) = @_;

    return "    # swap: nothing to do\n" unless $thisop->{flushin};

    die unless $thisop->{name} =~ /^swap(\d\d)/;

    return "    move \$a0, \$17\n".
           "    li \$a1, ".($thisop->{in}->[0] + 16)."\n".
           "    jal bc_swap_op$1\n";
}

sub out_loop {
    my ($thisop, $wi) = @_;
    my $wo = $wi;
    if ($thisop->{target}->{addr} < $thisop->{addr}) {
        return "    addiu $reg[$wo], $reg[$wi], -1\n".
               "    sw $reg[$wo], ".($thisop->{in}->[0] * 4)."(\$17)\n".
               "    bne $reg[$wo], \$0, $thisop->{args}->[1]\n";
    } else {
        return
               "    beq $reg[$wi], \$0, $thisop->{args}->[1]\n".
               "    addiu $reg[$wo], $reg[$wi], -1\n";
    }
}

sub out_eq {
    my ($thisop, $wi0, $wi1) = @_;
    return "    bne $reg[$wi0], $reg[$wi1], 1f\n";
}

sub out_eq0 {
    my ($thisop, $wi0) = @_;
    return "    bne $reg[$wi0], \$0, 1f\n";
}

sub out_neq {
    my ($thisop, $wi0, $wi1) = @_;
    return "    beq $reg[$wi0], $reg[$wi1], 1f\n";
}

sub out_neq0 {
    my ($thisop, $wi0) = @_;
    return "    beq $reg[$wi0], \$0, 1f\n";
}

sub out_lt {
    my ($thisop, $wi0, $wi1) = @_;
    return "    sltu \$at, $reg[$wi0], $reg[$wi1]\n".
           "    beq  \$at, \$0, 1f\n";
}

sub out_lteq {
    my ($thisop, $wi0, $wi1) = @_;
    return "    sltu \$at, $reg[$wi1], $reg[$wi0]\n".
           "    bne  \$at, \$0, 1f\n";
}

sub out_lts {
    my ($thisop, $wi0, $wi1) = @_;
    return "    slt \$at, $reg[$wi0], $reg[$wi1]\n".
           "    beq  \$at, \$0, 1f\n";
}

sub out_lteqs {
    my ($thisop, $wi0, $wi1) = @_;
    return "    slt \$at, $reg[$wi1], $reg[$wi0]\n".
           "    bne  \$at, \$0, 1f\n";
}

sub out_add {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    addu $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_sub {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    subu $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_neg {
    my ($thisop, $wo, $wi0) = @_;
    return "    subu $reg[$wo], \$0, $reg[$wi0]\n";
}

sub out_mov {
    my ($thisop, $wo, $wi) = @_;
    return "    move $reg[$wo], $reg[$wi]\n";
}

sub out_mul {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    multu $reg[$wi0], $reg[$wi1]\n".
           "    mflo $reg[$wo]\n";
}

sub out_div {
    my ($thisop, $wo0, $wo1, $wi0, $wi1) = @_;
    return "    divu $reg[$wi0], $reg[$wi1]\n".
           "    mflo $reg[$wo0]\n".
           "    mfhi $reg[$wo1]\n";
}

sub out_or {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    or $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_xor {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    xor $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_and {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    and $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_not {
    my ($thisop, $wo, $wi0) = @_;
    return "    nor $reg[$wo], \$0, $reg[$wi0]\n";
}

sub out_msbs {
    my ($thisop, $wo, $wi0) = @_;
    return "    clz $reg[$wo], $reg[$wi0]\n".
           "    xori $reg[$wo], $reg[$wo], 31\n";
}

sub parse_ccall {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;
}

sub out_ccall {
    my ($thisop, $wo, $wi0) = @_;
    return "    move \$a0, \$17\n".
           "    jalr $reg[$wi0]\n".
           "    move $reg[$wo], \$v0\n";
}

sub parse_rand {
    my ($thisop) = @_;
    $thisop->{clobber} = $caller_saved;
}

sub out_rand {
    my ($thisop, $wo) = @_;

    return "    jal rand_64\n".
           "    move $reg[$wo], \$v0\n";
}

sub out_shl {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    sllv $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_shr {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    srlv $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_sha {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    return "    srav $reg[$wo], $reg[$wi0], $reg[$wi1]\n";
}

sub out_tstc {
    my ($thisop, $wi) = @_;
    my $x = $thisop->{args}->[1];
    if ($x > 15) {
        return "    srl \$at, $reg[$wi], $x\n".
               "    andi \$at, 1\n".
               "    bne \$at, \$0, 1f\n";
    } else {
        return "    andi \$at, $reg[$wi], (1 << $x)\n".
               "    bne \$at, \$0, 1f\n";
    }
}

sub out_tsts {
    my ($thisop, $wi) = @_;
    my $x = $thisop->{args}->[1];
    if ($x > 15) {
        return "    srl \$at, $reg[$wi], $x\n".
               "    andi \$at, 1\n".
               "    beq \$at, \$0, 1f\n";
    } else {
        return "    andi \$at, $reg[$wi], ".(1 << $x)."\n".
               "    beq \$at, \$0, 1f\n";
    }
}

sub out_bitc {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    return "    li \$at, ".(0xffffffff ^ (1 << $x))."\n".
	   "    and $reg[$wo], $reg[$wi], \$at\n";
}

sub out_bits {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    if ($x > 15) {
        return "    lui \$at, ".(1 << ($x - 16))."\n".
               "    or $reg[$wo], \$at\n";
    } else {
	return "    ori $reg[$wo], $reg[$wi], ".(1 << $x)."\n";
    }
}

sub out_shil {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    return "    sll $reg[$wo], $reg[$wi], $x\n";
}

sub out_shir {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    return "    srl $reg[$wo], $reg[$wi], $x\n";
}

sub out_shia {
    my ($thisop, $wo, $wi) = @_;
    my $x = $thisop->{args}->[1];
    return "    sra $reg[$wo], $reg[$wi], $x\n";
}

sub out_extz {
    my ($thisop, $wo, $wi) = @_;
    my $r;
    my $x = $thisop->{args}->[1];
    if ( $x < 16 ) {
        $r = "    andi $reg[$wo], $reg[$wi], ".((1 << ($x + 1)) - 1)."\n";
    } elsif ( $x < 31 ) {
        $x = 31 - $x;
        $r = "    sll \$at, $reg[$wi], $x\n".
             "    srl $reg[$wo], \$at, $x";
    }
    return $r;
}

sub out_exts {
    my ($thisop, $wo, $wi) = @_;
    my $r;
    my $x = $thisop->{args}->[1];
    if ( $x < 31 ) {
        $x = 31 - $x;
        $r = "    sll \$at, $reg[$wi], $x\n".
             "    sra $reg[$wo], \$at, $x";
    }
    return $r;
}

sub out_st {
    my ($thisop, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    sb $reg[$wi0], ($reg[$wi1])\n";
    } elsif ($s == 1) {
        $r = "    sh $reg[$wi0], ($reg[$wi1])\n";
    } elsif ($s == 2) {
        $r = "    sw $reg[$wi0], ($reg[$wi1])\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r = "    sw $reg[$wi0], ($reg[$wi1])\n".
                 "    sw \$0, 4($reg[$wi1])\n";
        } else {
            $r = "    sw $reg[$wi0], 4($reg[$wi1])\n".
                 "    sw \$0, ($reg[$wi1])\n";
        }
    }
    return $r;
}

sub out_ste {
    my ($thisop, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    sb $reg[$wi0], ($thisop->{args}->[2])($reg[$wi1])\n";
    } elsif ($s == 1) {
        $r = "    sh $reg[$wi0], ($thisop->{args}->[2])($reg[$wi1])\n";
    } elsif ($s == 2) {
        $r = "    sw $reg[$wi0], ($thisop->{args}->[2])($reg[$wi1])\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r = "    sw $reg[$wi0], ($thisop->{args}->[2])($reg[$wi1])\n".
                 "    sw \$0, (4 + $thisop->{args}->[2])($reg[$wi1])\n";
        } else {
            $r = "    sw $reg[$wi0], (4 + $thisop->{args}->[2])($reg[$wi1])\n".
                 "    sw \$0, ($thisop->{args}->[2])($reg[$wi1])\n";
        }
    }
    return $r;
}

sub out_sti {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    sb $reg[$wi0], ($reg[$wi1])\n";
    } elsif ($s == 1) {
        $r = "    sh $reg[$wi0], ($reg[$wi1])\n";
    } elsif ($s == 2) {
        $r = "    sw $reg[$wi0], ($reg[$wi1])\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r = "    sw $reg[$wi0], ($reg[$wi1])\n".
                 "    sw \$0, 4($reg[$wi1])\n";
        } else {
            $r = "    sw $reg[$wi0], 4($reg[$wi1])\n".
                 "    sw \$0, ($reg[$wi1])\n";
        }
    }
    $r .= "    addiu $reg[$wo], $reg[$wi1], ".(1 << $s)."\n";
    return $r;
}

sub out_std {
    my ($thisop, $wo, $wi0, $wi1) = @_;
    my $s = $thisop->{width};
    my $r = "    addiu $reg[$wo], $reg[$wi1], -".(1 << $s)."\n";
    if ($s == 0) {
        $r .= "    sb $reg[$wi0], ($reg[$wo])\n";
    } elsif ($s == 1) {
        $r .= "    sh $reg[$wi0], ($reg[$wo])\n";
    } elsif ($s == 2) {
        $r .= "    sw $reg[$wi0], ($reg[$wo])\n";
    } else {
        main::warning($thisop, "64 bit store truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r .= "    sw $reg[$wi0], ($reg[$wo])\n".
                  "    sw \$0, 4($reg[$wi1])\n";
        } else {
            $r .= "    sw $reg[$wi0], 4($reg[$wo])\n".
                  "    sw \$0, ($reg[$wi1])\n";
        }
    }
    return $r;
}

sub out_ld {
    my ($thisop, $wo, $wi) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    lbu $reg[$wo], ($reg[$wi])\n";
    } elsif ($s == 1) {
        $r = "    lhu $reg[$wo], ($reg[$wi])\n";
    } elsif ($s == 2) {
        $r = "    lw $reg[$wo], ($reg[$wi])\n";
    } else {
        main::warning($thisop, "64 bit load truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r = "    lw $reg[$wo], ($reg[$wi])\n";
        } else {
            $r = "    lw $reg[$wo], 4($reg[$wi])\n";
        }
    }
    return $r;
}

sub out_lde {
    my ($thisop, $wo, $wi) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    lbu $reg[$wo], ($thisop->{args}->[2])($reg[$wi])\n";
    } elsif ($s == 1) {
        $r = "    lhu $reg[$wo], ($thisop->{args}->[2])($reg[$wi])\n";
    } elsif ($s == 2) {
        $r = "    lw $reg[$wo], ($thisop->{args}->[2])($reg[$wi])\n";
    } else {
        main::warning($thisop, "64 bit load truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r = "    lw $reg[$wo], ($thisop->{args}->[2])($reg[$wi])\n";
        } else {
            $r = "    lw $reg[$wo], ($thisop->{args}->[2] + 4)($reg[$wi])\n";
        }
    }
    return $r;
}

sub out_ldi {
    my ($thisop, $wo0, $wo1, $wi) = @_;
    my $s = $thisop->{width};
    my $r;
    if ($s == 0) {
        $r = "    lbu $reg[$wo0], ($reg[$wi])\n";
    } elsif ($s == 1) {
        $r = "    lhu $reg[$wo0], ($reg[$wi])\n";
    } elsif ($s == 2) {
        $r = "    lw $reg[$wo0], ($reg[$wi])\n";
    } else {
        main::warning($thisop, "64 bit load truncated to 32 bits.\n");
        if ( $main::backend_endian eq "little" ) {
            $r = "    lw $reg[$wo0], ($reg[$wi])\n";
        } else {
            $r = "    lw $reg[$wo0], 4($reg[$wi])\n";
        }
    }
    $r .= "    addiu $reg[$wo1], $reg[$wi], ".(1 << $s)."\n";
    return $r;
}

sub out_cst {
    my ($thisop, $wo) = @_;
    my $r;
    my $x = $thisop->{args}->[1] << $thisop->{args}->[2];
    if ( $thisop->{width} >= 2 ) {
        main::warning($thisop, "64 bit constant truncated to 32 bits.\n");
    }
    if ( !($x & 0xffff0000) ) {
        $r = "    li $reg[$wo], $x\n";
    } else {
        $r = "    lui $reg[$wo], ".(($x & 0xffff0000) >> 16)."\n";
        if ($x & 0x0000ffff) {
            $r .= "    ori $reg[$wo], ".($x & 0xffff)."\n";
        }
    }
    return $r;
}

sub out_gaddr {
    my ($thisop, $wo) = @_;
    return "    la $reg[$wo], $thisop->{args}->[1]\n";
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

