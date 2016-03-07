
package bc_custom_print;

sub parse_1reg
{
    my $thisop = shift;

    my $arg0 = main::check_reg( $thisop, 0 );

    $thisop->{code} |= $arg0;
}

main::custom_op('printi', 1, 0x1000, \&parse_1reg );
main::custom_op('prints', 1, 0x2000, \&parse_1reg );
main::custom_cond_op('skipodd', 1, 0x3000, \&parse_1reg );

return 1;

