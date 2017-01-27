
package bc_backend_bytecode;

use base 'bc_backend_common';

use strict;

sub addr {
    my $x = shift;
    return "    .".(1 << $main::backend_width)."byte $x\n";
}

sub out_begin {
    my ( $opcount ) = @_;
    return "    .section .rodata,\"a\"\n".
           "    .globl ${main::bc_name}_bytecode\n".
           "    .balign ".(1 << $main::backend_width)."\n".
           "${main::bc_name}_bytecode:\n".
	   # struct bc_descriptor_s
	   addr( "_${main::bc_name}_bytecode" ).
	   addr( "bc_run_vm" ).
	   "    .2byte 0x0000\n".
	   "    .2byte $opcount\n".
	   "_${main::bc_name}_bytecode:\n";
}

sub out_eof {
    return "    .2byte 0\n". # end op
	   "    .2byte 0\n". # end op
	   "    .size ${main::bc_name}_bytecode, . - ${main::bc_name}_bytecode\n";
}

$bc_backend_common::word = sub
{
    my ($w) = @_;
    return sprintf("    .balign 2 ; .byte 0x%02x ; .byte 0x%02x ;", $w & 0xff, ($w >> 8) & 0xff);
};

sub out_gaddr {
    my ($thisop) = @_;

    return bc_backend_common::fmt3( $thisop, 0, 0, $thisop->{out}->[0] ).
           addr( $thisop->{args}->[1] );
}

sub write {
    my $this = shift;

    open(OUT, ">$main::fout") || die "unable to open output file `$main::fout'.\n";

    print OUT out_begin( $main::last_addr );

    foreach my $thisop (@main::src) {

        foreach my $l ( @{$thisop->{labels}} ) {
            if ( $l->{export} ) {
                print OUT "$l->{name}:\n";
                print OUT "     .globl $l->{name}\n";
            };
        }

        my $hname = 'out_'.$thisop->{op}->{backend};

        printf OUT "    %-20s  # %s %s\n",
            $this->can( $hname )->( $thisop ),
            $thisop->{name}, join(', ', @{$thisop->{args}});
    }

    print OUT out_eof();

    close( OUT );

    main::write_addr();
}

return 1;

