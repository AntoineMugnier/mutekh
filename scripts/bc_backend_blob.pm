
package bc_backend_blob;

use base 'bc_backend_common';

use strict;

$bc_backend_common::word = sub
{
    my ($w) = @_;
    return pack("v", $w & 0xffff);
};

sub out_data {
    my ($thisop) = @_;

    return pack("C*", main::repack_data( $thisop ) );
}

sub out_gaddr {
    my ($thisop) = @_;

    main::error($thisop, "not allowed in blob bytecode\n")
}

sub write {
    my $this = shift;

    open(OUT, ">$main::fout") || die "unable to open output file `$main::fout'.\n";

    print OUT pack( "V", $main::bcflags );
    my $addr = 0;

    foreach my $thisop (@main::src) {

        if ( my $pad = $addr < $thisop->{addr} ) {
            print OUT pack("C", 0) x $pad;
        } elsif ( $addr > $thisop->{addr} ) {
            die;
        }

        my $hname = 'out_'.$thisop->{op}->{backend};

        print OUT $this->can( $hname )->( $thisop );

        $addr += $thisop->{bytes};
    }

    close( OUT );

    main::write_addr();
}

return 1;

