
package bc_custom_spi;

#    instruction         params        opcode                  format
# -------------------------------------------------------------------
#
#    generic instructions              0--- ---- ---- ----
#
#    nodelay                           1000 0011 00-- ----
#    deadline            r             1000 0011 01-- rrrr
#    delay               r             1000 0011 10-- rrrr
#    timestamp           r             1000 0011 11-- rrrr
#
#    yield                             1000 0000 001- ----
#    yield_delay         r             1000 0000 101- rrrr
#    yield_deadline      r             1000 0000 011- rrrr
#    yieldc                            1000 0000 000- ----
#    yieldc_delay        r             1000 0000 100- rrrr
#    yieldc_deadline     r             1000 0000 010- rrrr
#
#    wait                cs            1000 0010 00cc ----
#    wait_delay          r, cs         1000 0010 10cc rrrr
#    wait_deadline       r, cs         1000 0010 01cc rrrr
#
#    setcs               cs            1000 0010 11cc ----
#
#    width               w, o          1000 0100 00ow wwww
#    brate               r             1000 0100 10-- rrrr
#
#    write               r             1000 1000 rrrr 1111
#    swp                 wr, rd        1000 1000 rrrr rrrr
#    swpl                wr, rd, l     1000 1lll rrrr rrrr
#
#    pad                 r             1001 0000 ---- rrrr
#
#    rdm[8,16,32]        ra, r         1001 01ss aaaa rrrr
#    wrm[8,16,32]        ra, r         1001 10ss aaaa rrrr
#    swpm[8,16,32]       ra, r         1001 11ss aaaa rrrr
#
#    gpioset             i, r          1010 iiii iiii rrrr
#    gpioget             i, r          1011 iiii iiii rrrr
#    gpiomode            i, m          1100 iiii iiii mmmm
#

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

main::custom_op('spi_wait',           1,      0x0200, \&parse_cs );
main::custom_op('spi_wait_delay',     2,      0x0280, \&parse_reg_cs );
main::custom_op('spi_wait_deadline',  2,      0x0240, \&parse_reg_cs );

main::custom_op('spi_setcs',          1,      0x02c0, \&parse_cs );

main::custom_op('spi_width',          2,      0x0400, \&parse_width  );
main::custom_op('spi_brate',          1,      0x0480, \&parse_reg );

main::custom_op('spi_write',          1,      0x080f, \&parse_write );
main::custom_op('spi_swp',            2,      0x0800, \&parse_swp );
main::custom_op('spi_swpl',           3,      0x0800, \&parse_swpl );

main::custom_op('spi_pad',            1,      0x1000, \&parse_reg );

main::custom_op('spi_rdm8',           2,      0x1400, \&parse_rdm );
main::custom_op('spi_rdm16',          2,      0x1500, \&parse_rdm );
main::custom_op('spi_rdm32',          2,      0x1700, \&parse_rdm );

main::custom_op('spi_rdm8',           2,      0x1400, \&parse_xxm );
main::custom_op('spi_rdm16',          2,      0x1500, \&parse_xxm );
main::custom_op('spi_rdm32',          2,      0x1700, \&parse_xxm );

main::custom_op('spi_wrm8',           2,      0x1800, \&parse_xxm );
main::custom_op('spi_wrm16',          2,      0x1900, \&parse_xxm );
main::custom_op('spi_wrm32',          2,      0x1b00, \&parse_xxm );

main::custom_op('spi_swpm8',          2,      0x1c00, \&parse_xxm );
main::custom_op('spi_swpm16',         2,      0x1d00, \&parse_xxm );
main::custom_op('spi_swpm32',         2,      0x1f00, \&parse_xxm );

main::custom_op('spi_gpioset',        2,      0x2000, \&parse_gpio );
main::custom_op('spi_gpioget',        2,      0x3000, \&parse_gpio );
main::custom_op('spi_gpiomode',       2,      0x4000, \&parse_gpio_mode );

sub parse_reg
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    $thisop->{code} |= $r;
}

sub parse_cs
{
    my $thisop = shift;
    my $cs = main::check_num( $thisop, 0, 0, 3 );
    $thisop->{code} |= ( $cs << 4 );
}

sub parse_reg_cs
{
    my $thisop = shift;
    my $r = main::check_reg( $thisop, 0 );
    my $cs = main::check_num( $thisop, 1, 0, 3 );
    $thisop->{code} |= ( $cs << 4 ) | $r;
}

sub parse_width
{
    my $thisop = shift;
    my $w = main::check_reg( $thisop, 0, 1, 32 ) - 1;
    my $o = main::check_num( $thisop, 1, 0, 1 );
    $thisop->{code} |= ( $o << 5 ) | $w;
}

sub parse_write
{
    my $thisop = shift;
    my $wr = main::check_reg( $thisop, 0 );
    $thisop->{code} |= ($wr << 4);
}

sub parse_swp
{
    my $thisop = shift;
    my $wr = main::check_reg( $thisop, 0 );
    my $rd = main::check_reg( $thisop, 1 );
    $thisop->{code} |= ($wr << 4) | $rd;
}

sub parse_swpl
{
    my $thisop = shift;
    my $wr = main::check_reg( $thisop, 0 );
    my $rd = main::check_reg( $thisop, 1 );
    my $l = main::check_num( $thisop, 2, 1, 8 ) - 1;
    $thisop->{code} |= ($l << 8) | ($wr << 4) | $rd;
}

sub parse_xxm
{
    my $thisop = shift;
    my $a = main::check_reg( $thisop, 0 );
    my $r = main::check_reg( $thisop, 1 );
    $thisop->{code} |= ($a << 4) | $r;
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

