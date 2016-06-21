
package bc_backend_mips32r2;

use File::Basename;
use lib dirname (__FILE__);
use base 'bc_backend_mips32';

use strict;

our @reg = @bc_backend_mips32::reg;

sub out_exts {
    my ($thisop, $wo, $wi) = @_;
    my $r;
    my $x = $thisop->{args}->[1];
    if ( $x == 7 ) {
        $r = "    seb $reg[$wo], $reg[$wi]\n";
    } elsif ( $x == 15 ) {
        $r = "    seh $reg[$wo], $reg[$wi]\n";
    } elsif ( $x < 31 ) {
        $x = 31 - $x;
        $r = "    sll \$at, $reg[$wi], $x\n".
             "    sra $reg[$wo], \$at, $x";
    }
    return $r;
}

return 1;

