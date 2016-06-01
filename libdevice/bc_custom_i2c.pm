
package bc_custom_i2c;

#    instruction         params        opcode                  format
# -------------------------------------------------------------------
#
#     generic instructions              0--- ---- ---- ----
#
#     delay               r             1000 0011 10-- rrrr
#     nodelay                           1000 0011 00-- ----
#
#     yield                             1000 0000 001- ----
#     yield_delay         r             1000 0000 101- rrrr
#     yieldc                            1000 0000 000- ----
#     yieldc_delay        r             1000 0000 100- rrrr
#
#     wait                              1000 0010 00-- ----
#     wait_delay          r             1000 0010 10-- rrrr
#
#     addr_set            r             1001 0--- ---- rrrr
#     addr_get            r             1001 1--- ---- rrrr
#
#     rdm                 ra, rl, s     1010 00ss aaaa llll
#     wrm                 ra, rl, s     1010 01ss aaaa llll
#
#     rdr                 r, l, s       1010 10ss rrrr llll
#     wrr                 r, l, s       1010 11ss rrrr llll
#
#     rdmc                ra, rl, s     1011 00ss aaaa llll
#     wrmc                ra, rl, s     1011 01ss aaaa llll
#
#     rdrc                r, l, s       1011 10ss rrrr 0lll
#     wrrc                r, l, s       1011 11ss rrrr 0lll
#
#     gpioset             i, r          1100 iiii iiii rrrr
#     gpioget             i, r          1101 iiii iiii rrrr
#     gpiomode            i, m          1110 iiii iiii mmmm
#

main::custom_op('i2c_delay',              1,    0x0380, \&parse_reg );
main::custom_op('i2c_nodelay',            0,    0x0300 );

main::custom_op('i2c_yield',              0,    0x0020 );
main::custom_op('i2c_yield_delay',        1,    0x00a0, \&parse_reg );

main::custom_cond_op('i2c_yieldc',        0,    0x0000 );
main::custom_cond_op('i2c_yieldc_delay',  1,    0x0080, \&parse_reg );

main::custom_cond_op('i2c_wait',          0,    0x0200 );
main::custom_cond_op('i2c_wait_delay',    1,    0x0280, \&parse_reg );

main::custom_op('i2c_addr_set',           1,    0x1000, \&parse_reg );
main::custom_op('i2c_addr_get',           1,    0x1800, \&parse_reg );

main::custom_op('i2c_rdm',                3,    0x2000, \&parse_xxm );
main::custom_op('i2c_wrm',                3,    0x2400, \&parse_xxm );

main::custom_op('i2c_rdr',                3,    0x2800, \&parse_xxr );
main::custom_op('i2c_wrr',                3,    0x2c00, \&parse_xxr );

main::custom_cond_op('i2c_rdmc',          3,    0x3000, \&parse_xxm );
main::custom_cond_op('i2c_wrmc',          3,    0x3400, \&parse_xxm );

main::custom_cond_op('i2c_rdrc',          3,    0x3800, \&parse_xxr );
main::custom_cond_op('i2c_wrrc',          3,    0x3c00, \&parse_xxr );

main::custom_op('i2c_gpioset',            2,    0x4000, \&parse_gpio );
main::custom_op('i2c_gpioget',            2,    0x5000, \&parse_gpio );
main::custom_op('i2c_gpiomode',           2,    0x6000, \&parse_gpio_mode );

sub parse_reg
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    $thisop->{code} |= $r;
}

sub parse_xxm
{
    my $thisop = shift;
    my $ra = main::check_reg( $thisop, 0 );
    my $rl = main::check_reg( $thisop, 1 );
    my $s = main::check_num( $thisop, 2, 0, 2 );
    $thisop->{code} |= ( $s << 8 ) | ($ra << 4) | $rl;
}

sub parse_xxr
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    my $l = main::check_num( $thisop, 1, 0, 8 );
    my $s = main::check_num( $thisop, 2, 0, 2 );
    $thisop->{code} |= ( $s << 8 ) | ($r << 4) | $l;
}

sub parse_gpio
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 255 );
    my $r = main::check_reg( $thisop, 1 );
    $thisop->{code} |= ($i << 4) | $r;
}

sub parse_gpio_mode
{
    my $thisop = shift;
    my $i = main::check_num( $thisop, 0, 0, 255 );
    my $m = main::check_num( $thisop, 0, 0, 15 );
    $thisop->{code} |= ($a << 4) | $m;
}

return 1;

