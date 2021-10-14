
package bc_custom_print;

sub parse_1reg
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    $thisop->{in}->[0] = $arg0;

    $thisop->{code} |= $arg0;
}

sub parse_loadstr
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_num( $thisop, 1, 1, 32 );

    $thisop->{packout_reg} = $arg0;
    $thisop->{packout_bytes} = $arg1;
    $thisop->{code} |= $arg0 | ($arg1 << 4);
}

sub parse_checkstr
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_num( $thisop, 1, 1, 32 );

    $thisop->{packin_reg} = $arg0;
    $thisop->{packin_bytes} = $arg1;
    $thisop->{code} |= $arg0 | ($arg1 << 4);
}

main::custom_op('printi', 1, 0x1000, \&parse_1reg );
main::custom_op('prints', 1, 0x2000, \&parse_1reg );
main::custom_cond_op('skipodd', 1, 0x3000, \&parse_1reg );
main::custom_op('loadstr', 2, 0x4000, \&parse_loadstr );
main::custom_op('checkstr', 2, 0x5000, \&parse_checkstr );
main::custom_op('printh', 2, 0x6000, \&parse_checkstr );

return 1;

