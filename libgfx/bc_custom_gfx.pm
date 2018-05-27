package bc_custom_gfx;

# *  gfx_disp       n                1000 1000 00nn 0000   : copy the drawable surface to the display at 0,0
# *  gfx_disp_at    xy, n            1000 1000 01nn xxxx   : copy the drawable surface to the display at specified position ans size. size is in r15
#
# *  gfx_surface    p, d, n          1001 00nn pppp dddd   : setup surface `n'. `p' is the pointer to surface data and `d' is the descriptor
# *  gfx_tilemap    d, n             1001 01-- --nn dddd   : setup the current tile map. `d' is the tile descriptor and `n' is the source surface
# *  gfx_attr_l8    a                1001 1000 aaaa aaaa   : select the current drawing attribute from a 8 bit greyscale value
#    gfx_attr_rgb8  a                1001 1001 aaaa aaaa   : select the current drawing color from a 8 bits RRRGGGBB value
#    gfx_pick_at    xy               1001 1010 --nn xxxx   : pick the color from a surface and set the current attribute
# *  gfx_swap       n, N             1001 1011 nnNN ----   : exchange index of surfaces n and N
#    gfx_pick       x                1001 11nn xxxx xxxx   : pick the color from a surface and set the current attribute.
#
# *  gfx_circle     xy, q            1010 0-00 qqqq xxxx   : draw a circle on the current drawable. q is a quadrants, radius is r15
# *  gfx_circle_f   xy, q            1010 0-01 qqqq xxxx   : draw a filled circle on the current drawable. q is a quadrants, radius is r15
# *  gfx_circle_i   xy, q            1010 0-10 qqqq xxxx   : draw a inverted filled circle on the current drawable. q is a quadrants, radius is r15
# *  gfx_arc_cw     xy, a            1010 10-0 xxxx aaaa   : draw an arc with center and radius on the current drawable. g are begin and end angles, radius is r15, c is CW|CCW
# *  gfx_arc_ccw    xy, a            1010 10-1 xxxx aaaa   : draw an arc with center and radius on the current drawable. g are begin and end angles, radius is r15, c is CW|CCW
#    gfx_xyarc_cw   xy, XY           1010 11-0 xxxx XXXX   : draw an arc between xy and XY on the current drawable. radius is r15, c is CW|CCW
#    gfx_xyarc_ccw  xy, XY           1010 11-1 xxxx XXXX   : draw an arc between xy and XY on the current drawable. radius is r15, c is CW|CCW

# *  gfx_line       xy, XY           1011 000- xxxx XXXX   : draw a line on the current drawable
# *  gfx_point      xy               1011 000- xxxx xxxx   : draw a point (same as gfx_line xy, xy)
# *  gfx_tile       xy, tile         1011 0010 xxxx tttt   : draw a single tile on the current drawable, xy is tile top left corner
# *  gfx_tilec      xy, tile         1011 0011 xxxx tttt   : draw a single tile on the current drawable, xy is tile center
# *  gfx_rect       xy, XY           1011 0100 xxxx XXXX   : draw a rectangle on the current drawable. f is filled
# *  gfx_rect_r     xy, XY           1011 0110 xxxx XXXX   : draw a rectangle with round corners on the current drawable. f is filled, radius is r15
# *  gfx_rect_f     xy, XY           1011 0101 xxxx XXXX   : draw a filled rectangle on the current drawable. f is filled
# *  gfx_rect_fr    xy, XY           1011 0111 xxxx XXXX   : draw a filed rectangle with round corners on the current drawable. f is filled, radius is r15
# *  gfx_tilestr    xy, str, r15, dir 1011 10dd xxxx ssss   : draw a string on the current drawable. Use tile map as font. string len is in r15.
# *  gfx_tilestrc   xy, str, r15, dir 1011 11dd xxxx ssss   : draw a centered string on the current drawable. Use tile map as font. string len is in r15.
#    gfx_plotx      xy, scale, buf
#
#    gfx_copy                        1100 000- ---- ----   : copy from source to drawable of the same size
# *  gfx_clear_l8   attr             1100 0010 aaaa aaaa   : fill the current drawable with the specified pixel attribute
#    gfx_clear_rgb8 attr             1100 0011 aaaa aaaa   : fill the current drawable with the specified pixel attribute
#
# *  gfx_blit       xy, XY, s        1100 10ss xxxx XXXX   : blit from source surface s at XY to drawable surface at xy. rectangle size is r15. Whole source when r15 is 0.
# *  gfx_blit_o     xy, XY           1100 11-- xxxx XXXX   : blit from drawable surface at XY to drawable surface at xy. rectangle size is r15. Blit areas are allowed to overlap.
#
# *  gfx_addxi      xy, +/-v         1101 0svv vvvv vvxx   : add a signed constant value to the integral part of the x component of a vector
# *  gfx_addyi      xy, +/-v         1101 1svv vvvv vvxx   : add a signed constant value to the integral part of y component of a vector
# *  gfx_addv       xy, XY           1110 0000 xxxx XXXX   : add vectors and store result in xy
# *  gfx_subv       xy, XY           1110 0001 xxxx XXXX   : subtract vectors and store result in xy
# *  gfx_negx       XY               1110 0010 -001 XXXX   : compute (-x, y) of vector
# *  gfx_negy       XY               1110 0010 -010 XXXX   : compute (x, -y) of vector
# *  gfx_negv       XY               1110 0010 -011 XXXX   : compute (-x, -y) of vector
# *  gfx_swpv       XY               1110 0010 -100 XXXX   : compute (y, x) of vector
# *  gfx_negx_swpv  XY               1110 0010 -101 XXXX   : compute (y, -x) of vector
# *  gfx_negy_swpv  XY               1110 0010 -110 XXXX   : compute (-y, x) of vector
# *  gfx_negv_swpv  XY               1110 0010 -111 XXXX   : compute (-y, -x) of vector
# *  gfx_mul        a, b             1110 0011 aaaa bbbb   : multiply signed Q27.5 values

# *  gfx_mulxy      XY, r            1110 0100 rrrr XXXX   : multiply both x and y components by a Q27.5 value from register
# *  gfx_addx       XY, rx           1110 0101 rrrr XXXX   : add a Q27.5 value from a register to the x component
# *  gfx_addy       XY, ry           1110 0110 rrrr XXXX   : add a Q27.5 value from a register to the y component
# *  gfx_divxy      XY, r            1110 0111 rrrr XXXX   : divide both x and y components by a Q27.5 value from register

# *  gfx_unpack     rx, ry, XY       1110 1000 rrr0 XXXX   : move both x and y unsigned components to two contiguous registers
# *  gfx_unpacks    rx, ry, XY       1110 1000 rrr1 XXXX   : move both x and y signed components to two contiguous registers
# *  gfx_packx0     XY, rx           1110 1001 rrr0 XXXX   : set the x component from a register and clear the y component
# *  gfx_packx      XY, rx           1110 1001 rrr1 XXXX   : set the x component from a register
# *  gfx_pack0y     XY, ry           1110 1010 rrr0 XXXX   : set the y component from a register and clear the x component
# *  gfx_packy      XY, ry           1110 1010 rrr1 XXXX   : set the y component from a register
# *  gfx_pack       XY, rx, ry       1110 1011 rrr- XXXX   : set both the x and y components from two contiguous registers

# *  gfx_size       xy, s            1110 1100 00nn xxxx   : get surface size
# *  gfx_hypot      xy               1110 1100 0100 xxxx   : compute distance (Q27.5) from a signed vector
# *  gfx_sqrt       a                1110 1100 0101 aaaa   : compute square root of a Q27.5 value
# *  gfx_sincos     a, r15           1110 1100 0110 aaaa   : compute a sin,cos signed vector scaled by r15 (Q27.5) from angle (Q9.5)
# *  gfx_sin        a, r15           1110 1100 1000 aaaa   : compute a sin (Q27.5) scaled by r15 (Q27.5) from angle (Q9.5)
# *  gfx_cos        a, r15           1110 1100 1001 aaaa   : compute a cos (Q27.5) scaled by r15 (Q27.5) from angle (Q9.5)

#    gfx_mreset                      1111 000- ---- ----   : reset the current transform matrix
#    gfx_mpush                       1111 001- ---- ----   : push the current transform matrix (a 4 entries ring is used)
#    gfx_mpop                        1111 010- ---- ----   : pop the current transform matrix
#    gfx_translate  vect             1111 011- ---- vvvv   : apply a translation to the current matrix in degrees by r[v]
#    gfx_rot        angle            1111 100- ---- aaaa   : apply a rotation of r[a] degrees to the current matrix
#    gfx_rotc       angle            1111 101a aaaa aaaa   : apply a constant rotation to the current matrix in degrees
#    gfx_scale      vect             1111 110- ---- vvvv   : apply a 2d scale to the current matrix by r[v]
#    gfx_apply      xy, XY           1111 111- xxxx XXXX   : apply the current matrix to the position XY and store result in xy
#
#  vector register format:
#
#    xxxxxyyy yyXXXYYY YYYYYYYY XXXXXXXX
#        |     |  |  |        |        \----- x integral low part
#        |     |  |  |        \-------------- y integral low part
#        |     |  |  \----------------------- y integral high part
#        |     |  \-------------------------- x integral high part
#        |     \----------------------------- y fractional part
#        \----------------------------------- x fractional part
#
#  register format of a surface descriptor:
#    wwwwwwww wwwwhhhh hhhhhhhh zzzfffff
#                                  \--------- pixel format
#                               \------------ compression (0:none, rle, lzo, ..)
#                  \------------------------- height
#    \--------------------------------------- width
#
#  register format of a tile descriptor:
#    wwwwwwww wwwwhhhh hhhhhhhh oooooooo
#                               \------------ index of first tile
#                 \-------------------------- tile height
#    \--------------------------------------- tile width

sub parse_attr
{
    my $thisop = shift;

    my $arg0 = main::check_num( $thisop, 0, 0, 255 );

    $thisop->{code} |= $arg0;
}

sub parse_1reg
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );

    $thisop->{code} |= $arg0;
}

sub parse_1reg_r15
{
    my $thisop = shift;
    main::check_reg( $thisop, 1, 15, 15 );
    parse_1reg( $thisop );
}

sub parse_2reg
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_reg( $thisop, 1 );

    $thisop->{code} |= ($arg0 << 4) | $arg1;
}

sub parse_tilestr
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_reg( $thisop, 1 );
    my $arg2 = main::check_reg( $thisop, 2 );
    my $arg3 = main::check_num( $thisop, 3, 0, 3 );

    die "$thisop->{line}: string length must be r15\n"
        unless ($arg2 == 15);

    $thisop->{code} |= ($arg3 << 8) | ($arg0 << 4) | $arg1;
}

sub parse_1reg_2
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );

    $thisop->{code} |= ($arg0 << 4) | $arg0;
}

sub parse_2reg_s
{
    my $thisop = shift;

    my $a = main::check_reg( $thisop, 0 );
    my $b = main::check_reg( $thisop, 1 );
    my $n = main::check_num( $thisop, 2, 0, 3 );

    $thisop->{code} |= ($n << 8) | ($a << 4) | $b;
}

sub parse_addi
{
    my $thisop = shift;

    my $a = main::check_reg( $thisop, 0, 0, 3 );
    my $n = main::check_num( $thisop, 1, -256, 255 );

    $thisop->{code} |= (($n & 0x1ff) << 2) | $a;
}

sub parse_s
{
    my $thisop = shift;

    my $n = main::check_num( $thisop, 0, 0, 3 );

    $thisop->{code} |= $n << 4;
}

sub parse_swap
{
    my $thisop = shift;

    my $a = main::check_num( $thisop, 0, 0, 3 );
    my $b = main::check_num( $thisop, 1, 0, 3 );

    $thisop->{code} |= ($a << 4) | ($b << 6);
}

sub parse_disp_at
{
    parse_2reg_s( $thisop );

    die "$thisop->{line}: same register can't be used for position and size\n"
        unless ($thisop->{code} & 0xff);

    $thisop->{code} |= ($n << 8) | ($xy << 4) | $s;
}

sub parse_1reg_s
{
    my $thisop = shift;

    my $d = main::check_reg( $thisop, 0 );
    my $n = main::check_num( $thisop, 1, 0, 3 );

    $thisop->{code} |= ($n << 4) | $d;
}

sub parse_1reg_s_r15
{
    my $thisop = shift;
    main::check_reg( $thisop, 2, 15, 15 );
    parse_1reg_s( $thisop );
}

sub parse_attr
{
    my $thisop = shift;

    my $arg0 = main::check_num( $thisop, 0, 0, 255 );

    $thisop->{code} |= $arg0;
}

sub parse_circle
{
    my $thisop = shift;

    my $xy = main::check_reg( $thisop, 0 );
    my $oct = main::check_num( $thisop, 1, 1, 15 );

    $thisop->{code} |= ($oct << 4) | $xy;
}

sub parse_unpack
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_reg( $thisop, 1 );
    my $arg2 = main::check_reg( $thisop, 2 );

    die "$thisop->{line}: contiguous destination registers expected\n"
        if ( $arg1 - 1 != $arg0 || ( $arg0 & 1 ) );

    $thisop->{code} |= ($arg0 << 4) | $arg2;
}

sub parse_packy
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_reg( $thisop, 1 );

    die "$thisop->{line}: odd source register expected\n"
        unless ( $arg1 & 1 );

    $thisop->{code} |= (($arg1 & 14) << 4) | $arg0;
}

sub parse_packx
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_reg( $thisop, 1 );

    die "$thisop->{line}: even source register expected\n"
        if ( $arg1 & 1 );

    $thisop->{code} |= ($arg1 << 4) | $arg0;
}

sub parse_pack
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_reg( $thisop, 1 );
    my $arg2 = main::check_reg( $thisop, 2 );

    die "$thisop->{line}: contiguous source registers expected\n"
        if ( $arg2 - 1 != $arg1 || ( $arg1 & 1 ) );

    $thisop->{code} |= ($arg1 << 4) | $arg0;
}


main::custom_op('gfx_disp',      1, 0x0800, \&parse_s );
main::custom_op('gfx_disp_at',   3, 0x0800, \&parse_1reg_s_r15 );

main::custom_op('gfx_surface',   3, 0x1000, \&parse_2reg_s );
main::custom_op('gfx_tilemap',   2, 0x1400, \&parse_1reg_s );
main::custom_op('gfx_attr_l8',   1, 0x1800, \&parse_attr );
main::custom_op('gfx_attr_rgb8', 1, 0x1900, \&parse_attr );
main::custom_op('gfx_swap',      2, 0x1b00, \&parse_swap );

main::custom_op('gfx_circle',    2, 0x2000, \&parse_circle );
main::custom_op('gfx_circle_f',  2, 0x2100, \&parse_circle );
main::custom_op('gfx_circle_i',  2, 0x2200, \&parse_circle );

main::custom_op('gfx_arc_cw',    2, 0x2800, \&parse_2reg );
main::custom_op('gfx_arc_ccw',   2, 0x2900, \&parse_2reg );
main::custom_op('gfx_xyarc_cw',  2, 0x2c00, \&parse_2reg );
main::custom_op('gfx_xyarc_ccw', 2, 0x2d00, \&parse_2reg );
main::custom_op('gfx_line',      2, 0x3000, \&parse_2reg );
main::custom_op('gfx_point',     1, 0x3000, \&parse_1reg_2 );
main::custom_op('gfx_tile',      2, 0x3200, \&parse_2reg );
main::custom_op('gfx_tilec',     2, 0x3300, \&parse_2reg );
main::custom_op('gfx_rect',      2, 0x3400, \&parse_2reg );
main::custom_op('gfx_rect_r',    2, 0x3600, \&parse_2reg );
main::custom_op('gfx_rect_f',    2, 0x3500, \&parse_2reg );
main::custom_op('gfx_rect_fr',   2, 0x3700, \&parse_2reg );
main::custom_op('gfx_tilestr',   4, 0x3800, \&parse_tilestr );
main::custom_op('gfx_tilestrc',  4, 0x3c00, \&parse_tilestr );

main::custom_op('gfx_clear',     1, 0x4200, \&parse_attr );
main::custom_op('gfx_blit',      3, 0x4800, \&parse_2reg_s );
main::custom_op('gfx_blit_o',    2, 0x4c00, \&parse_2reg );

main::custom_op('gfx_addxi',     2, 0x5000, \&parse_addi );
main::custom_op('gfx_addyi',     2, 0x5800, \&parse_addi );
main::custom_op('gfx_addv',      2, 0x6000, \&parse_2reg );
main::custom_op('gfx_subv',      2, 0x6100, \&parse_2reg );
main::custom_op('gfx_negx',      1, 0x6210, \&parse_1reg );
main::custom_op('gfx_negy',      1, 0x6220, \&parse_1reg );
main::custom_op('gfx_negv',      1, 0x6230, \&parse_1reg );
main::custom_op('gfx_swpv',      1, 0x6240, \&parse_1reg );
main::custom_op('gfx_negx_swpv', 1, 0x6250, \&parse_1reg );
main::custom_op('gfx_negy_swpv', 1, 0x6260, \&parse_1reg );
main::custom_op('gfx_negv_swpv', 1, 0x6270, \&parse_1reg );

main::custom_op('gfx_mul',       2, 0x6300, \&parse_2reg );

main::custom_op('gfx_mulxy',     2, 0x6400, \&parse_2reg );
main::custom_op('gfx_addx',      2, 0x6500, \&parse_2reg );
main::custom_op('gfx_addy',      2, 0x6600, \&parse_2reg );
main::custom_op('gfx_divxy',     2, 0x6700, \&parse_2reg );

main::custom_op('gfx_unpack',    3, 0x6800, \&parse_unpack );
main::custom_op('gfx_unpacks',   3, 0x6810, \&parse_unpack );
main::custom_op('gfx_packx',     2, 0x6910, \&parse_packx );
main::custom_op('gfx_packx0',    2, 0x6900, \&parse_packx );
main::custom_op('gfx_packy',     2, 0x6a10, \&parse_packy );
main::custom_op('gfx_pack0y',    2, 0x6a00, \&parse_packy );
main::custom_op('gfx_pack',      3, 0x6b00, \&parse_pack );

main::custom_op('gfx_size',      2, 0x6c00, \&parse_1reg_s );
main::custom_op('gfx_hypot',     1, 0x6c40, \&parse_1reg );
main::custom_op('gfx_sqrt',      1, 0x6c50, \&parse_1reg );
main::custom_op('gfx_sincos',    2, 0x6c60, \&parse_1reg_r15 );
main::custom_op('gfx_sin',       2, 0x6c80, \&parse_1reg_r15 );
main::custom_op('gfx_cos',       2, 0x6c90, \&parse_1reg_r15 );

return 1;
