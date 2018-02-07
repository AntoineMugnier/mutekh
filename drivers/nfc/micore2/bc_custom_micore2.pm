package bc_custom_micore2;

main::custom_op     ('wait_ms',           1, 0x0000, \&parse_delay);
main::custom_op     ('irq_pending_clear', 0, 0x1000);
main::custom_cond_op('on_irq_timeout',    1, 0x1000, \&parse_delay);
main::custom_op     ('reg_get',           2, 0x2000, \&parse_reg_value_out);
main::custom_op     ('reg_set',           2, 0x2400, \&parse_reg_value_in);
main::custom_op     ('reg_or',            2, 0x2800, \&parse_reg_value_in);
main::custom_op     ('reg_andn',          2, 0x2C00, \&parse_reg_value_in);
main::custom_op     ('fifo_read',         2, 0x3000, \&parse_addr_size);
main::custom_op     ('fifo_write',        2, 0x3100, \&parse_addr_size);
main::custom_op     ('fifo_read_r',       2, 0x3200, \&parse_packout_size);
main::custom_op     ('fifo_write_r',      2, 0x3300, \&parse_packin_size);
main::custom_op     ('sleep_until_rq',    0, 0x4000);
main::custom_op     ('rq_done',           1, 0x4100, \&parse_reg_in);
main::custom_cond_op('rq_next_then',      1, 0x5000, \&parse_reg_out);
main::custom_op     ('resetn',            1, 0x6000, \&parse_bin_in);
main::custom_op     ('print',             1, 0x7000, \&parse_reg_in);

sub parse_reg_out
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 0);
    $thisop->{out}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_reg_in
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 0);
    $thisop->{in}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_addr_size
{
    my $thisop = shift;
    my $addr = main::check_reg($thisop, 0);
    my $size = main::check_reg($thisop, 1);
    $thisop->{in}->[0] = $addr;
    $thisop->{in}->[1] = $size;
    $thisop->{code} |= $addr << 4;
    $thisop->{code} |= $size;
}

sub parse_packin_size
{
    my $thisop = shift;
    my $pack = main::check_reg($thisop, 0);
    my $size = main::check_num($thisop, 1, 1, 16);
    $thisop->{packin_reg} = $pack;
    $thisop->{packin_bytes} = $size;
    die "$thisop->{line}: out of range byte array\n"
        if ($pack * 4 + $size > 16 * 4);
    $thisop->{code} |= $pack << 4;
    $thisop->{code} |= $size - 1;
}

sub parse_packout_size
{
    my $thisop = shift;
    my $pack = main::check_reg($thisop, 0);
    my $size = main::check_num($thisop, 1, 1, 16);
    $thisop->{packout_reg} = $pack;
    $thisop->{packout_bytes} = $size;
    die "$thisop->{line}: out of range byte array\n"
        if ($pack * 4 + $size > 16 * 4);
    $thisop->{code} |= $pack << 4;
    $thisop->{code} |= $size - 1;
}

sub parse_reg_value_in
{
    my $thisop = shift;
    my $micore_regno = main::check_num($thisop, 0, 0, 0x3f);
    my $value_reg = main::check_reg($thisop, 1);
    $thisop->{in}->[0] = $value_reg;
    $thisop->{code} |= $micore_regno << 4;
    $thisop->{code} |= $value_reg;
}

sub parse_reg_value_out
{
    my $thisop = shift;
    my $micore_regno = main::check_num($thisop, 0, 0, 0x3f);
    my $value_reg = main::check_reg($thisop, 1);
    $thisop->{out}->[0] = $value_reg;
    $thisop->{code} |= $micore_regno << 4;
    $thisop->{code} |= $value_reg;
}

sub parse_delay
{
    my $thisop = shift;
    my $d = main::check_num($thisop, 0, 0, 2047 * 5);
    $thisop->{code} |= ($d + 4) / 5;
}

sub parse_bin_in
{
    my $thisop = shift;
    my $d = main::check_num($thisop, 0, 0, 1);
    $thisop->{code} |= $d;
}

return 1;
