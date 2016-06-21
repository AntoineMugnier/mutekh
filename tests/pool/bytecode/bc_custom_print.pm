
package bc_custom_print;

sub parse_1reg
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );

    $thisop->{code} |= $arg0;
}

sub parse_1reg1imm
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );
    my $arg1 = main::check_num( $thisop, 1, 1, 32 );

    $thisop->{code} |= $arg0 | ($arg1 << 4);
}

main::custom_op('printi', 1, 0x1000, \&parse_1reg );
main::custom_op('prints', 1, 0x2000, \&parse_1reg );
main::custom_cond_op('skipodd', 1, 0x3000, \&parse_1reg );
main::custom_op('loadstr', 2, 0x4000, \&parse_1reg1imm );
main::custom_op('checkstr', 2, 0x5000, \&parse_1reg1imm );

return 1;

