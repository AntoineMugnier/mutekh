
package bc_backend_bytecode;

use strict;
 
sub out_begin {
    my ( $b, $opcount ) = @_;
    return "    .section .rodata,\"a\"\n".
           "    .globl $main::bc_name\n".
           "$main::bc_name:\n".
	   # struct bc_descriptor_s
	   "    .short 0x0000\n".
	   "    .short $opcount\n".
	   "    .long _$main::bc_name\n".
	   "    .long bc_run_vm\n".
	   "_$main::bc_name:\n";
}

sub out_eof {
    return "    .hword 0\n". # end op
	   "    .hword 0\n". # end op
	   "    .size $main::bc_name, . - $main::bc_name\n";
}

sub word
{
    my ($w) = @_;
    return sprintf("    .balign 2 ; .byte 0x%02x ; .byte 0x%02x ;", $w & 0xff, ($w >> 8) & 0xff);
#    return sprintf(" .hword %u ;", $w & 0xffff);
}

sub out_custom {
    my ($thisop) = @_;
    my $code = $thisop->{op}->{code} | $thisop->{code};
    return word($code);
}

sub out_custom_cond {
    my ($thisop) = @_;
    my $code = $thisop->{op}->{code} | $thisop->{code};
    return word($code);
}

sub fmt0
{
    my ($thisop, $v, $r) = @_;
    return word( $thisop->{op}->{code} | (($v & 0xff) << 4) | ($r & 0xf) );
}

sub fmt1
{
    my ($thisop, $rs, $rd) = @_;
    return word( $thisop->{op}->{code} | (($rs & 0xf) << 4) | ($rd & 0xf) );
}

sub fmt2
{
    my ($thisop, $b, $r) = @_;
    return word( $thisop->{op}->{code} | (($b & 0x3f) << 4) | ($r & 0xf) );
}

sub fmt3
{
    my ($thisop, $s, $a, $r) = @_;
    return word( $thisop->{op}->{code} | ($s << 9) | (($a & 0xf) << 4) | ($r & 0xf) );
}

sub out_end {
    return fmt0( shift, 0, 0 );
}

sub out_dump {
    return fmt0( shift, 0, 1 );
}

sub out_abort {
    return fmt0( shift, 0, 2 );
}

sub out_nop {
    return fmt0( shift, 0, 4 );
}

sub out_trace {
    my ($thisop) = @_;
    return fmt0( $thisop, 0, ($thisop->{args}->[1] << 1) | ($thisop->{args}->[0] << 0) );
}

sub out_add8 {
    my ($thisop) = @_;
    return fmt0( $thisop, $thisop->{args}->[1], $thisop->{out}->[0] );
}

sub out_cst8 {
    my ($thisop) = @_;
    return fmt0( $thisop, $thisop->{args}->[1], $thisop->{out}->[0] );
}

sub out_jmp8 {
    my ($thisop) = @_;
    return fmt0( $thisop, $thisop->{disp}, 0 );
}

sub out_call8 {
    my ($thisop) = @_;
    return fmt0( $thisop, $thisop->{disp}, $thisop->{lr} );
}

sub out_call32 {
    my ($thisop) = @_;
    my $a = $thisop->{target}->{addr} - 1;
    return fmt3( $thisop, 0, 0, $thisop->{lr} ).
           word( $a >> 16 ).
           word( $a );
}

sub out_jmp32 {
    my ($thisop) = @_;
    my $a = $thisop->{target}->{addr} - 1;
    return fmt3( $thisop, 0, 0, 0 ).
           word( $a >> 16 ).
           word( $a );
}

sub out_ret {
    my ($thisop) = @_;
    return fmt0( $thisop, 0, $thisop->{in}->[0] );
}

sub out_loop {
    my ($thisop) = @_;
    return fmt0( $thisop, $thisop->{disp}, $thisop->{in}->[0] );
}

sub out_eq {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[1] );
}

sub out_eq0 {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[0] );
}

sub out_neq {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[1] );
}

sub out_neq0 {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[0] );
}

sub out_lt {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[1] );
}

sub out_lteq {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[1] );
}

sub out_add {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_sub {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_neg {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_mov {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_mul {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_or {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_xor {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_and {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_andn {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_not {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_ccall {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_shl {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_shr {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_msbs {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_tstc {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_tsts {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_bitc {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_bits {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_shil {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_shir {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_extz {
    my ($thisop) = @_;
    return fmt2( $thisop, 31 - $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_exts {
    my ($thisop) = @_;
    return fmt2( $thisop, 31 - $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_st {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_ste {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[1], $thisop->{in}->[0] ).
           word( $thisop->{args}->[2] );
}

sub out_sti {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_std {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_ld {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_lde {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[0], $thisop->{out}->[0] ).
           word( $thisop->{args}->[2] );
}

sub out_ldi {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_cst {
    my ($thisop) = @_;

    my $res = fmt3( $thisop, $thisop->{width},
                    $thisop->{args}->[2] >> 2,
                    $thisop->{out}->[0] );

    my $x = $thisop->{args}->[1];

    if ( $thisop->{width} == 1 ) {
        $res .= word( $x );
    } elsif ( $thisop->{width} == 2 ) {
        $res .= word( $x >> 16 );
        $res .= word( $x );
    } elsif ( $thisop->{width} == 3 ) {
        $res .= word( $x >> 48 );
        $res .= word( $x >> 32 );
        $res .= word( $x >> 16 );
        $res .= word( $x );
    } else {
        die;
    }

    return $res;
}

sub out_laddr {
    my ($thisop) = @_;

    my $res = fmt3( $thisop, $thisop->{width}, 0,
                    $thisop->{out}->[0] );

    my $x = $thisop->{target}->{addr} - 1;

    if ( $thisop->{width} == 1 ) {
        $res .= word( $x );
    } elsif ( $thisop->{width} == 2 ) {
        $res .= word( $x >> 16 );
        $res .= word( $x );
    } else {
        die;
    }

    return $res;
}

sub out_gaddr {
    my ($thisop) = @_;

    return fmt3( $thisop, 0, 0, $thisop->{out}->[0] ).
           "    .long $thisop->{args}->[1] ;";
}

sub out_data {
    my ($thisop) = @_;

    return word( $thisop->{args}->[0] );
}

sub write {
    main::write_bc();
    main::write_addr();
}

return 1;

