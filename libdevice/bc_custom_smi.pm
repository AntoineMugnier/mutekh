package bc_custom_smi;

main::custom_op     ('yield',            1, 0x0000, \&parse_literal8);
main::custom_op     ('reg_set',          2, 0x0100, \&parse_reg_io);
main::custom_op     ('reg_get',          2, 0x0200, \&parse_reg_io);
main::custom_op     ('prtad',            1, 0x0300, \&parse_reg_in);
main::custom_op     ('devad',            1, 0x0400, \&parse_reg_in);

sub parse_literal8
{
    my $thisop = shift;
    my $i = main::check_num($thisop, 0, 0, 255);
    $thisop->{code} |= $i;
}

sub parse_reg_io
{
    my $thisop = shift;
    my $d = main::check_reg($thisop, 0);
    my $s = main::check_reg($thisop, 1);
    $thisop->{out}->[0] = $d;
    $thisop->{in}->[0] = $s;
    $thisop->{code} |= $d << 4;
    $thisop->{code} |= $s;
}

sub parse_reg_out
{
    my $thisop = shift;
    my $d = main::check_reg($thisop, 0);
    $thisop->{out}->[0] = $d;
    $thisop->{code} |= $d << 4;
}

sub parse_reg_in
{
    my $thisop = shift;
    my $s = main::check_reg($thisop, 0);
    $thisop->{in}->[0] = $s;
    $thisop->{code} |= $s;
}

return 1;
