
package bc_custom_i2c;

main::custom_op('i2c_nodelay',            0,    0x0000 );
main::custom_op('i2c_delay',              1,    0x0010, \&parse_reg );

main::custom_cond_op('i2c_wait',          0,    0x0080 );
main::custom_cond_op('i2c_wait_delay',    1,    0x0090, \&parse_reg );

main::custom_op('i2c_yield',              0,    0x0100 );
main::custom_op('i2c_yield_delay',        1,    0x0110, \&parse_reg );

main::custom_cond_op('i2c_yieldc',        0,    0x0180 );
main::custom_cond_op('i2c_yieldc_delay',  1,    0x0190, \&parse_reg );

main::custom_op('i2c_addr_set',           1,    0x0710, \&parse_reg );
main::custom_op('i2c_addr_get',           1,    0x0700, \&parse_rego );

main::custom_op('i2c_gpioset',            2,    0x0400, \&parse_gpio_set );
main::custom_op('i2c_gpioget',            2,    0x0500, \&parse_gpio_get );
main::custom_op('i2c_gpiomode',           2,    0x0600, \&parse_gpio_mode );

main::custom_op('i2c_rdm',                3,    0xc100, \&parse_xxm );
main::custom_op('i2c_wrm',                3,    0xc000, \&parse_xxm );
main::custom_op('i2c_rdr',                3,    0xe100, \&parse_rdr );
main::custom_op('i2c_wrr',                3,    0xe000, \&parse_wrr );
main::custom_cond_op('i2c_rdmc',          3,    0xd100, \&parse_xxm );
main::custom_cond_op('i2c_wrmc',          3,    0xd000, \&parse_xxm );
main::custom_cond_op('i2c_rdrc',          3,    0xf100, \&parse_rdr );
main::custom_cond_op('i2c_wrrc',          3,    0xf000, \&parse_wrr );

sub check_op
{
    our %ops = (
        'CONTINUOUS'      => 2,
        'CONTINUOUS_SYNC' => 10,
        'STOP'            => 12,
        'RESTART'         => 6,
        'RESTART_SYNC'    => 14,
        );

    my ( $thisop, $argidx ) = @_;
    my $expr = $thisop->{args}->[$argidx];
    my $op = $ops{$expr};
    die "$thisop->{line}: bad transfer operation, expected: ".join(', ', keys %ops)."\n"
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

sub parse_rego
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    $thisop->{out}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_xxm
{
    my $thisop = shift;
    my $ra = main::check_reg( $thisop, 0 );
    my $rl = main::check_reg( $thisop, 1 );
    $thisop->{in}->[0] = $ra;
    $thisop->{in}->[1] = $rl;
    my $e = check_op( $thisop, 2 );
    $thisop->{code} |= ( $e << 8 ) | ($ra << 4) | $rl;
}

sub parse_rdr
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    my $l = main::check_num( $thisop, 1, 1, 16 );
    $thisop->{packout_reg} = $r;
    $thisop->{packout_bytes} = $l;
    my $e = check_op( $thisop, 2 );
    $thisop->{code} |= ( $e << 8 ) | ($r << 4) | ($l - 1);
}

sub parse_wrr
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    my $l = main::check_num( $thisop, 1, 1, 16 );
    $thisop->{packin_reg} = $r;
    $thisop->{packin_bytes} = $l;
    my $e = check_op( $thisop, 2 );
    $thisop->{code} |= ( $e << 8 ) | ($r << 4) | ($l - 1);
}

sub parse_gpio_set
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 15 );
    my $r = main::check_reg( $thisop, 1 );
    $thisop->{in}->[0] = $r;
    $thisop->{code} |= ($i << 4) | $r;
}

sub parse_gpio_get
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 15 );
    my $r = main::check_reg( $thisop, 1 );
    $thisop->{out}->[0] = $r;
    $thisop->{code} |= ($i << 4) | $r;
}

sub parse_gpio_mode
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 15 );
    my $m = main::check_num( $thisop, 0, 0, 15 );
    $thisop->{code} |= ($a << 4) | $m;
}

return 1;

