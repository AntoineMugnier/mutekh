
package bc_backend_blob;

use base 'bc_backend_common';

use strict;

$bc_backend_common::word = sub
{
    my ($w) = @_;
    return pack("v", $w);
};

sub out_gaddr {
    my ($thisop) = @_;

    main::error($thisop, "not allowed in blob bytecode\n")
}

sub write {
    my $this = shift;

    open(OUT, ">$main::fout") || die "unable to open output file `$main::fout'.\n";

    print OUT pack( "vv", 0, $main::last_addr );

    foreach my $thisop (@main::src) {
        my $hname = 'out_'.$thisop->{op}->{backend};
        print OUT $this->can( $hname )->( $thisop );
    }

    close( OUT );

    main::write_addr();
}

return 1;

