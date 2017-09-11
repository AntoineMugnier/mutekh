
package bc_backend_bytecode;

use base 'bc_backend_common';

use strict;

sub addr {
    my $x = shift;
    return "    .".(1 << $main::backend_width)."byte $x\n";
}

sub out_begin {

    return "    .section .rodata,\"a\"\n".
           "    .globl ${main::bc_name}_bytecode\n".
           "    .globl _${main::bc_name}_bytecode\n".
           "    .globl _${main::bc_name}_bytecode_end\n".
           "    .balign ".(1 << $main::backend_width)."\n".
           "${main::bc_name}_bytecode:\n".
	   # struct bc_descriptor_s
	   addr( "_${main::bc_name}_bytecode" ).
	   addr( $main::bcflags & 0x02000000 ? "bc_run_sandbox" : "bc_run_vm" ).
	   "    .4byte $main::bcflags\n".
	   "_${main::bc_name}_bytecode:\n";
}

sub out_eof {
    return "    .2byte 0\n". # end op
	   "    .2byte 0\n". # end op
	   "_${main::bc_name}_bytecode_end:\n".
	   "    .size ${main::bc_name}_bytecode, . - ${main::bc_name}_bytecode\n";
}

$bc_backend_common::word = sub
{
    my ($w) = @_;
    return sprintf("    .balign 2 ; .byte 0x%02x ; .byte 0x%02x ;", $w & 0xff, ($w >> 8) & 0xff);
};

sub out_data {
    my ($thisop) = @_;

    return join('', map {
                      sprintf("    .byte 0x%02x ;", $_);
                    } main::repack_data( $thisop ) );
}

sub out_gaddr {
    my ($thisop) = @_;

    return bc_backend_common::fmt3( $thisop, 0, 0, $thisop->{out}->[0] ).
           addr( $thisop->{args}->[1] );
}

sub write {
    my $this = shift;

    open(OUT, ">$main::fout") || die "unable to open output file `$main::fout'.\n";

    print OUT out_begin();
    my $addr = 0;

    foreach my $thisop (@main::src) {

        if ( my $pad = $addr < $thisop->{addr} ) {
            printf OUT ("    .byte 0 ;" x $pad)." # padding\n";
        } elsif ( $addr > $thisop->{addr} ) {
            die;
        }

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

        $addr += $thisop->{bytes};
    }

    print OUT out_eof();

    close( OUT );

    main::write_addr();
}

return 1;

