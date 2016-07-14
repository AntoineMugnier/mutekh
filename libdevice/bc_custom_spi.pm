
package bc_custom_spi;

main::custom_op('spi_nodelay',        0,      0x0300 );
main::custom_op('spi_deadline',       1,      0x0340, \&parse_reg );
main::custom_op('spi_delay',          1,      0x0380, \&parse_reg );
main::custom_op('spi_timestamp',      1,      0x03c0, \&parse_reg );

main::custom_op('spi_yield',          0,      0x0020 );
main::custom_op('spi_yield_delay',    1,      0x00a0, \&parse_reg );
main::custom_op('spi_yield_deadline', 1,      0x0060, \&parse_reg );

main::custom_cond_op('spi_yieldc',          0,   0x0000 );
main::custom_cond_op('spi_yieldc_delay' ,   1,   0x0080, \&parse_reg );
main::custom_cond_op('spi_yieldc_deadline', 1,   0x0040, \&parse_reg );

main::custom_op('spi_wait',           0,      0x0200 );
main::custom_op('spi_wait_delay',     1,      0x0280, \&parse_reg );
main::custom_op('spi_wait_deadline',  1,      0x0240, \&parse_reg );

main::custom_op('spi_width',          2,      0x0400, \&parse_width  );
main::custom_op('spi_brate',          1,      0x0480, \&parse_reg );

main::custom_op('spi_rd',             3,      0x1000, \&parse_rd );
main::custom_op('spi_wr',             3,      0x2000, \&parse_wr );
main::custom_op('spi_swp',            4,      0x3000, \&parse_swp );

main::custom_op('spi_pad',            2,      0x4000, \&parse_pad );
main::custom_op('spi_rdm',            3,      0x4400, \&parse_xxm );
main::custom_op('spi_wrm',            3,      0x4800, \&parse_xxm );
main::custom_op('spi_swpm',           4,      0x4c00, \&parse_swpm );

main::custom_op('spi_gpiomode',       2,      0x0800, \&parse_gpio_mode );
main::custom_op('spi_gpioget',        2,      0x0a00, \&parse_gpio_get );
main::custom_op('spi_gpioset',        2,      0x0c00, \&parse_gpio_set );

our %csops1 = (
    'CS_END'   => 0,
    'CS_PULSE' => 0,
    'CS_START' => 1,
    'CS_CONTINUE' => 1,
    );

our %csops2 = (
    'CS_NOP' => 0,
    'CS_DESELECT'  => 1,
    'CS_END'   => 2,
    'CS_PULSE' => 2,
    'CS_START' => 3,
    'CS_CONTINUE' => 3,
    );

sub check_csop
{
    my ( $thisop, $argidx, $ops ) = @_;
    my $expr = $thisop->{args}->[$argidx];
    my $op = $ops->{$expr};
    die "$thisop->{line}: bad chip select operation, expected: ".join(', ', keys %{$ops})."\n"
        unless defined $op;
    return $op;
}

sub parse_reg
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    $thisop->{in}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_width
{
    my $thisop = shift;
    my $w = main::check_reg( $thisop, 0, 1, 32 ) - 1;
    my $o = main::check_num( $thisop, 1, 0, 1 );
    $thisop->{code} |= ( $o << 5 ) | $w;
}

sub parse_rd
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    my $l = main::check_num( $thisop, 1, 1, 16 );
    $thisop->{packout_reg} = $r;
    $thisop->{packout_bytes} = $l;
    my $cs = check_csop( $thisop, 2, \%csops1 );
    die "$thisop->{line}: out of range byte array\n"
        if ($r * 4 + $l > 16 * 4);
    $thisop->{code} |= ($cs << 14) | (($l - 1) << 8) | ($r << 4);
}

sub parse_wr
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    my $l = main::check_num( $thisop, 1, 1, 16 );
    $thisop->{packin_reg} = $r;
    $thisop->{packin_bytes} = $l;
    my $cs = check_csop( $thisop, 2, \%csops1 );
    die "$thisop->{line}: out of range byte array\n"
        if ($r * 4 + $l > 16 * 4);
    $thisop->{code} |= ($cs << 14) | (($l - 1) << 8) | ($r << 0);
}

sub parse_swp
{
    my $thisop = shift;
    my $wr = main::check_reg( $thisop, 0 );
    my $rd = main::check_reg( $thisop, 1 );
    my $l = main::check_num( $thisop, 2, 1, 16 );
    $thisop->{packin_reg} = $wr;
    $thisop->{packin_bytes} = $l;
    $thisop->{packout_reg} = $rd;
    $thisop->{packout_bytes} = $l;
    my $cs = check_csop( $thisop, 3, \%csops1 );
    die "$thisop->{line}: out of range read byte array\n"
        if ($rd * 4 + $l > 16 * 4);
    die "$thisop->{line}: out of range write byte array\n"
        if ($wr * 4 + $l > 16 * 4);
    $thisop->{code} |= ($cs << 14) | (($l - 1) << 8) | ($wr << 4) | $rd;
}

sub parse_pad
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    $thisop->{in}->[0] = $r;
    my $cs = check_csop( $thisop, 1, \%csops2 );
    $thisop->{code} |= ($cs << 8) | $r;
}

sub parse_xxm
{
    my $thisop = shift;
    my $ra = main::check_reg( $thisop, 0 );
    my $rl = main::check_reg( $thisop, 1 );
    $thisop->{in}->[0] = $ra;
    $thisop->{in}->[1] = $rl;
    my $cs = check_csop( $thisop, 2, \%csops2 );
    $thisop->{code} |= ($cs << 8) | ($ra << 4) | $rl;
}

sub parse_swpm
{
    my $thisop = shift;
    my $rar = main::check_reg( $thisop, 0 );
    my $raw = main::check_reg( $thisop, 1 );
    die "$thisop->{line}: read and write address registers must be a contiguous pair\n"
        if ($raw ^ 1 != $rar);
    my $rl = main::check_reg( $thisop, 2 );
    $thisop->{in}->[0] = $rar;
    $thisop->{in}->[1] = $raw;
    $thisop->{in}->[2] = $rl;
    my $cs = check_csop( $thisop, 3, \%csops2 );
    $thisop->{code} |= ($cs << 8) | ($rar << 4) | $rl;
}

sub parse_gpio_set
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 31 );
    my $r = main::check_reg( $thisop, 1 );
    $thisop->{in}->[0] = $r;
    $thisop->{code} |= ($i << 4) | $r;
}

sub parse_gpio_get
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 31 );
    my $r = main::check_reg( $thisop, 1 );
    $thisop->{out}->[0] = $r;
    $thisop->{code} |= ($i << 4) | $r;
}

sub parse_gpio_mode
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 31 );
    my $m = main::check_num( $thisop, 0, 0, 15 );
    $thisop->{code} |= ($a << 4) | $m;
}

return 1;

