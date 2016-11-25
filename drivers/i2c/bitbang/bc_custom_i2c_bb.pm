package bc_custom_i2c_bb;

main::custom_op     ('wait',             0, 0x0000);
main::custom_op     ('txn_next_wait',    0, 0x2000);
main::custom_op     ('txn_type_get',     1, 0x2100, \&parse_reg_out);
main::custom_op     ('txn_saddr_get',    1, 0x2200, \&parse_reg_out);
main::custom_cond_op('txn_byte_pop',     1, 0x2300, \&parse_reg_out);
main::custom_op     ('txn_byte_push',    1, 0x2400, \&parse_reg_in);
main::custom_op     ('txn_byte_is_last', 1, 0x2500, \&parse_reg_out);
main::custom_op     ('txn_byte_is_done', 1, 0x2600, \&parse_reg_out);
main::custom_op     ('txn_done',         1, 0x2700, \&parse_int_in);
main::custom_op     ('io_set',           2, 0x4000, \&parse_io_val_in);
main::custom_op     ('io_get',           2, 0x4040, \&parse_io_reg_out);
main::custom_op     ('io_mode',          2, 0x4080, \&parse_io_mode);
main::custom_op     ('yield',            0, 0x6000);

our %ios = (
    'SCL'  => 0,
    'SDA'  => 1,
    );

our %levels = (
    'LOW'    => 0,
    'HIGH'   => 1,
    );

sub parse_int_in
{
    my $thisop = shift;
    my $expr = $thisop->{args}->[0];
    my $i = main::check_num($thisop, 0, 0, 255);
    $thisop->{code} |= $i;
}

sub parse_io_mode
{
    my $thisop = shift;

    my $mode = main::check_num($thisop, 1, 0, 31);

    my $io = $thisop->{args}->[0];
    my $ion = $ios{$io};

    die "Bad IO name $io" unless defined $ion;

    $thisop->{code} |= $mode;
    $thisop->{code} |= $ion << 5;
}

sub parse_io_val_in
{
    my $thisop = shift;
    my $expr = $thisop->{args}->[1];
    my $d = $levels{$expr};
    my $io = $thisop->{args}->[0];
    my $ion = $ios{$io};

    die "Bad IO name $io" unless defined $ion;

    if ( defined $d ) {
        $thisop->{code} |= 0x10;
        $thisop->{code} |= $d;
    } else {
        my $r = main::check_reg($thisop, 1);
        $thisop->{in}->[0] = $r;
        $thisop->{code} |= $r;
    }

    $thisop->{code} |= $ion << 5;
}

sub parse_io_reg_out
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 1);
    my $io = $thisop->{args}->[0];
    my $ion = $ios{$io};

    die "Bad IO name $io" unless defined $ion;

    $thisop->{out}->[0] = $r;
    $thisop->{code} |= $r;
    $thisop->{code} |= $ion << 5;
}

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

return 1;
