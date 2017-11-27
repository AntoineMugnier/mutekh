package bc_custom_ble_sm;

main::custom_op     ('packet_alloc',      0, 0x0000);
main::custom_op     ('packet_wait',       0, 0x0100);
main::custom_op     ('packet_left',       1, 0x0200, \&parse_reg_out);
main::custom_op     ('packet_send',       0, 0x0300);
main::custom_op     ('ltk_get',           1, 0x1000, \&parse_reg_out);
main::custom_op     ('irk_get',           1, 0x1100, \&parse_reg_out);
main::custom_op     ('peer_id_get',       1, 0x1200, \&parse_reg_out);
main::custom_op     ('ediv_get',          1, 0x1300, \&parse_reg_out);
main::custom_op     ('peer_csrk_get',     1, 0x1400, \&parse_reg_out);
main::custom_op     ('peer_ltk_set',      1, 0x1500, \&parse_reg_in);
main::custom_op     ('peer_id_set',       1, 0x1600, \&parse_reg_in);
main::custom_op     ('peer_ediv_set',     1, 0x1700, \&parse_reg_in);
main::custom_op     ('peer_irk_set',      1, 0x1800, \&parse_reg_in);
main::custom_op     ('peer_csrk_set',     1, 0x1900, \&parse_reg_in);
main::custom_op     ('rng_read',          2, 0x2000, \&parse_reg_rng);
main::custom_op     ('conf_calc',         2, 0x2100, \&parse_reg_conf);
main::custom_op     ('stk_compute',       0, 0x2200);
main::custom_op     ('pairing_success',   0, 0x3000);
main::custom_op     ('encryption_wait',   0, 0x3100);
main::custom_op     ('bonding_success',   0, 0x3200);
main::custom_op     ('failure',           0, 0x3300, \&parse_failure);

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

sub parse_failure
{
    my $thisop = shift;
    my $err = main::check_num($thisop, 0, 0x1, 0xe);
    $thisop->{code} |= $err;
}

sub parse_reg_rng
{
    my $thisop = shift;
    my $dest = main::check_reg($thisop, 0);
    my $size = main::check_num($thisop, 1, 1, 16);
    $thisop->{out}->[0] = $dest;
    $thisop->{code} |= $dest;
    $thisop->{code} |= ($size - 1) << 4;
}

sub parse_reg_conf
{
    my $thisop = shift;
    my $conf_dst = main::check_reg($thisop, 0);
    my $rand_src = main::check_reg($thisop, 1);
    $thisop->{in}->[0] = $rand_src;
    $thisop->{out}->[0] = $conf_dst;
    $thisop->{code} |= $rand_src;
    $thisop->{code} |= $conf_dst << 4;
}

return 1;
