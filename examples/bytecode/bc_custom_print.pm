
package bc_custom_print;

# parse operand of our custom instruction
sub parse_1reg
{
    my $thisop = shift;

    # first operand must be register
    my $arg0 = main::check_reg( $thisop, 0 );

    # merge the 4 bits register index in the 16 bits opcode
    $thisop->{code} |= $arg0;
}

# declare our custom integer printing instruction
main::custom_op('printi', 1, 0x1000, \&parse_1reg );

return 1;

