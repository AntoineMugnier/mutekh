package bc_custom_matrix_keyboard;

main::custom_op('wait_row',        0, 0x0000);
main::custom_op('wait_refresh',    0, 0x0100);
main::custom_op('column_strobe',   1, 0x1000, \&parse_reg_in);
main::custom_op('columns_release', 0, 0x1100);
main::custom_op('rows_get',        1, 0x1200, \&parse_reg_out);
main::custom_op('wait_change',     1, 0x1300, \&parse_reg_in);
main::custom_op('pressed_reset',   0, 0x2000);
main::custom_op('pressed_set',     1, 0x2100, \&parse_reg_in);
main::custom_op('pressed_done',    0, 0x2200);

sub parse_reg_in
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 0);
    $thisop->{in}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_reg_out
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 0);
    $thisop->{out}->[0] = $r;
    $thisop->{code} |= $r;
}

return 1;
