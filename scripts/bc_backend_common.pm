
package bc_backend_common;

use strict;

our $word = sub { die "instruction word output handler not installed"; };

sub out_custom {
    my ($thisop) = @_;
    my $code = $thisop->{op}->{code} | $thisop->{code};
    return $word->($code);
}

sub out_custom_cond {
    my ($thisop) = @_;
    my $code = $thisop->{op}->{code} | $thisop->{code};
    return $word->($code);
}

sub fmt0
{
    my ($thisop, $v, $r) = @_;
    return $word->( $thisop->{op}->{code} | (($v & 0xff) << 4) | ($r & 0xf) );
}

sub fmt1
{
    my ($thisop, $rs, $rd) = @_;
    return $word->( $thisop->{op}->{code} | (($rs & 0xf) << 4) | ($rd & 0xf) );
}

sub fmt2
{
    my ($thisop, $b, $r) = @_;
    return $word->( $thisop->{op}->{code} | (($b & 0x3f) << 4) | ($r & 0xf) );
}

sub fmt3
{
    my ($thisop, $s, $a, $r) = @_;
    return $word->( $thisop->{op}->{code} | ($s << 10) | (($a & 0xf) << 4) | ($r & 0xf) );
}

sub fmt4
{
    my ($thisop, $c, $o, $r) = @_;
    return $word->( $thisop->{op}->{code} | ($c << 8) | (($o & 0xf) << 4) | ($r & 0xf) );
}

sub check_imm
{
    my ($thisop, $a, $signed) = @_;

    my $w = $thisop->{width};
    my $m1 = -1 << (16 + 16 * $w);

    my $m2 =  1 << (15 + 16 * $w);
    $a ^= $m1 if ( $signed && ( $m2 & $a ) );

    main::error($thisop, sprintf("`%s' range exceeded : 0x%x.\n", $thisop->{name}, $a))
        if ( $a & $m1 );

    return $a;
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

sub out_die {
    return fmt0( shift, 0, 3 );
}

sub out_nop {
    return fmt0( shift, 0, 4 );
}

sub out_trace {
    my ($thisop) = @_;
    return fmt0( $thisop, 0, $thisop->{args}->[0]);
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
    if ( my $d = ($thisop->{disp} - 2) >> 1 ) {
        return fmt0( $thisop, $d, 0 );
    } else {
        # nop
        return $word->(0x0004);
    }
}

sub out_call8 {
    my ($thisop) = @_;
    return fmt0( $thisop, ($thisop->{disp} - 2) >> 1, $thisop->{out}->[0] );
}

sub out_calljmp {
    my ($thisop, $a, $signed, $reg) = @_;

    my $w = $thisop->{width};
    $a = check_imm( $thisop, $a, $signed );
    my $d = $thisop->{disp} - 2;

    if ($d <= 127*2 && $d > -128*2) {
        main::warning($thisop, "8 bits jump could be used instead.\n")
    } elsif ($w > 0 && $d <= 32767 && $d > -32768) {
        main::warning($thisop, "16 bits jump could be used instead.\n")
    }

    my $r = fmt3( $thisop, 0, 0, $reg );

    $r .= $word->( $a );
    $r .= $word->( $a >> 16 ) if ( $w > 0 );

    return $r;
}

sub out_callr {
    my ($thisop) = @_;
    return out_calljmp( $thisop, $thisop->{disp} - 2, 1, $thisop->{out}->[0] );
}

sub out_calla {
    my ($thisop) = @_;
    return out_calljmp( $thisop, $thisop->{target}->{addr} - 2, 0, $thisop->{out}->[0] );
}

sub out_jmpr {
    my ($thisop) = @_;
    return out_calljmp( $thisop, $thisop->{disp} - 2, 1, 0 );
}

sub out_jmpa {
    my ($thisop) = @_;
    return out_calljmp( $thisop, $thisop->{target}->{addr} - 2, 0, 0 );
}

sub out_call {
    my ($thisop) = @_;
    return fmt0( $thisop, 0, $thisop->{in}->[0] );
}

sub out_ret {
    my ($thisop) = @_;
    return fmt0( $thisop, 0, $thisop->{in}->[0] );
}

sub out_jmp {
    my ($thisop) = @_;
    return fmt0( $thisop, 0, $thisop->{in}->[0] );
}

sub out_loop {
    my ($thisop) = @_;
    return fmt0( $thisop, (($thisop->{disp} - 2) >> 1) & 0x7f, $thisop->{in}->[0] );
}

our %packops = (
    'pack8' => 0,
    'pack16le' => 1,
    'pack16be' => 2,
    'unpack16le' => 3,
    'unpack16be' => 4,
    'swap16le' => 5,
    'swap16be' => 6,
    'swap16' => 7,
    'unpack8' => 8,
    'pack32le' => 9,
    'pack32be' => 10,
    'unpack32le' => 11,
    'unpack32be' => 12,
    'swap32le' => 13,
    'swap32be' => 14,
    'swap32' => 15
    );

sub out_pack {
    my ($thisop) = @_;
    return fmt4( $thisop, $thisop->{count} - 1, $packops{$thisop->{name}}, $thisop->{in}->[0] );
}

sub out_unpack {
    my ($thisop) = @_;
    return fmt4( $thisop, $thisop->{count} - 1, $packops{$thisop->{name}}, $thisop->{out}->[0] );
}

sub out_swap {
    my ($thisop) = @_;
    return fmt4( $thisop, 0, $packops{$thisop->{name}}, $thisop->{in}->[0] );
}

sub out_pick {
    my ($thisop) = @_;
    my $r = $thisop->{packin_reg};
    return $word->( $thisop->{op}->{code} | ($r & 0xf)).
        $word->( $thisop->{args}->[1] );
}

sub out_place {
    return out_pick( shift );
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
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_lteq {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_lts {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_lteqs {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{in}->[0] );
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
    if ( $thisop->{in}->[0] != $thisop->{out}->[0] ) {
        return fmt1( $thisop, $thisop->{in}->[0], $thisop->{out}->[0] );
    } else {
        # nop
        return $word->(0x0004);
    }
}

sub out_mul {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_div {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_or {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_rand {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{out}->[0], $thisop->{out}->[0] );
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
    return fmt1( $thisop, $thisop->{in}->[0], $thisop->{in}->[0] );
}

sub out_shl {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_shr {
    my ($thisop) = @_;
    return fmt1( $thisop, $thisop->{in}->[1], $thisop->{out}->[0] );
}

sub out_sha {
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

sub out_shia {
    my ($thisop) = @_;
    return fmt2( $thisop, $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_extz {
    my ($thisop) = @_;
    return fmt2( $thisop, 31 - $thisop->{args}->[1], $thisop->{in}->[0] );
}

sub out_exts {
    my ($thisop) = @_;
    my $r = $thisop->{in}->[0];
    my $b = $thisop->{args}->[1];
    my %code = ( 7 => 0x4200, 15 => 0x4300, 31 => 0x4400 );
    return $word->( $code{$b} | ($r << 4) | $r );
}

sub out_st {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[1], $thisop->{in}->[0] );
}

sub out_ste {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[1], $thisop->{in}->[0] ).
           $word->( $thisop->{args}->[2] );
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
           $word->( $thisop->{args}->[2] );
}

sub out_ldi {
    my ($thisop) = @_;
    return fmt3( $thisop, $thisop->{width}, $thisop->{in}->[0], $thisop->{out}->[0] );
}

sub out_cst {
    my ($thisop) = @_;

    my $w = $thisop->{width};
    my $a = $thisop->{args}->[1];

    if ($w == 0 && ($a & 0xffffffffffff0000) == 0xffffffffffff0000) {
        $a &= 0xffff;
        $w |= 2;
    } elsif ($w == 1 && ($a & 0xffffffff00000000) == 0xffffffff00000000) {
        $a &= 0xffffffff;
        $w |= 2;
    }

    my $r = fmt3( $thisop, $w,
                  $thisop->{args}->[2] >> 3,
                  $thisop->{out}->[0] );

    $r .= $word->( $a );
    $r .= $word->( $a >> 16 ) if ( $w & 1 );

    return $r;
}

sub out_mode {
    my ($thisop) = @_;
    my $m = $thisop->{args}->[0];
    return fmt3($thisop, $m >> 4, 0, $m & 15);
}

sub out_laddr_ {
    my ($thisop, $a, $signed) = @_;

    my $op = 0;
    my $w = $thisop->{width};

    # may embbed bit 31 of laddr16 when in sandbox mode
    if (($main::bcflags & 0x02000000) && $w == 0 &&
        ($a & 0xffff0000) == 0x80000000) {
        $a -= 0x80000000;
        $op = 4;
    }

    $a = check_imm( $thisop, $a, $signed );

    my $r = fmt3( $thisop, $w, $op,
                  $thisop->{out}->[0] );

    $r .= $word->( $a );
    $r .= $word->( $a >> 16 ) if ( $w > 0 );

    return $r;
}

sub out_laddra {
    my ($thisop) = @_;
    return out_laddr_( $thisop, $thisop->{target}->{addr}, 0 );
}

sub out_laddrr {
    my ($thisop) = @_;
    return out_laddr_( $thisop, $thisop->{disp}, 1 );
}
