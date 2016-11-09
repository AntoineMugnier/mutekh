package bc_custom_matrix_keyboard;

main::custom_op('wait_row',        0, 0x0000);
main::custom_op('wait_refresh',    0, 0x0100);
main::custom_op('column_strobe',   1, 0x1000, \&parse_reg_in);
main::custom_op('columns_release', 0, 0x1100);
main::custom_op('rows_get',        1, 0x1200, \&parse_reg_out);
main::custom_op('wait_change',     0, 0x1300);
main::custom_op('notify',          1, 0x1400, \&parse_mode);
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

our %modes = (
    'none'    => 0,
    'press'   => 1,
    'release' => 2,
    );

sub parse_mode
{
    my $thisop = shift;
    my $expr = $thisop->{args}->[0];
    my $d = $modes{$expr};
    die "$thisop->{line}: bad notify operation '$expr', expected: ".join(', ', keys %modes)."\n"
        unless defined $d;
    $thisop->{code} |= $d;
}

return 1;
