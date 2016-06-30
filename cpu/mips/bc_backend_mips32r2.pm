
package bc_backend_mips32r2;

use File::Basename;
use lib dirname (__FILE__);
use base 'bc_backend_mips32';

use strict;

our @reg = @bc_backend_mips32::reg;

sub out_pack_ {
    my ($thisop, @w) = @_;

    my $sym = $main::packops{$thisop->{name}};
    return "# pack: nothing to do\n" if !$sym;

    if ( $thisop->{count} > 6 ) {
        return "    move \$a0, \$17\n".
               "    li \$a1, ".($thisop->{reg} + $thisop->{count} * 16)."\n".
               "    jal $sym\n";
    } else {
        my $r;
        for ( my $i = 0; $i < $thisop->{count}; $i++ ) {
            my %op = (
                'bc_pack_op8' => sub {
                    $r .= "    sb $reg[$w[$i]], ".(4 * $thisop->{reg} + $i)."(\$17)\n";
                }, 'bc_unpack_op8' => sub {
                    $r .= "    lbu $reg[$w[$i]], ".(4 * $thisop->{reg} + $i)."(\$17)\n";
                }, 'bc_pack_op16' => sub {
                    $r .= "    sh $reg[$w[$i]], ".(4 * $thisop->{reg} + $i * 2)."(\$17)\n";
                }, 'bc_unpack_op16' => sub {
                    $r .= "    lhu $reg[$w[$i]], ".(4 * $thisop->{reg} + $i * 2)."(\$17)\n";
                }, 'bc_swap_pack_op16' => sub {
                    $r .= "    wsbh \$at, $reg[$w[$i]]\n";
                    $r .= "    sh \$at, ".(4 * $thisop->{reg} + $i * 2)."(\$17)\n";
                }, 'bc_unpack_swap_op16' => sub {
                    $r .= "    lhu \$at, ".(4 * $thisop->{reg} + $i * 2)."(\$17)\n";
                    $r .= "    wsbh $reg[$w[$i]], \$at\n";
                }, 'bc_swap_pack_op32' => sub {
                    $r .= "    wsbh \$at, $reg[$w[$i]]\n";
                    $r .= "    rotr \$at, \$at, 16\n";
                    $r .= "    sw \$at, ".(4 * $thisop->{reg} + $i * 4)."(\$17)\n";
                }, 'bc_unpack_swap_op32' => sub {
                    $r .= "    lw \$at, ".(4 * $thisop->{reg} + $i * 4)."(\$17)\n";
                    $r .= "    wsbh \$at, \$at\n";
                    $r .= "    rotr $reg[$w[$i]], \$at, 16\n";
                });
            $op{$sym}->();
        }
        return $r;
    }
}

sub parse_pack {
    my ($thisop) = @_;

    my $sym = $main::packops{$thisop->{name}};

    if ( !$sym ) {
        # nop
        $thisop->{in} = [];
    } elsif ( $thisop->{count} > $bc_backend_mips32::max_op_regs ) {
        # use function call
        $thisop->{flushin} = (1 << $thisop->{count}) - 1;
        $thisop->{clobber} = $bc_backend_mips32::caller_saved;
    } else {
        # prevent overwritting of packed data
        $thisop->{wbin} = (1 << $thisop->{count}) - 1;
    }
}

sub out_pack {
    return out_pack_( @_ );
}

sub parse_unpack {
    my ($thisop) = @_;

    my $sym = $main::packops{$thisop->{name}};

    if ( !$sym ) {
        # nop
        $thisop->{out} = [];
    } elsif ( $thisop->{count} > $bc_backend_mips32::max_op_regs ) {
        # use function call
        $thisop->{reloadout} = (1 << $thisop->{count}) - 1;
        $thisop->{clobber} = $bc_backend_mips32::caller_saved;
    }
}

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

sub parse_swap {
    my ($thisop) = @_;

    if ( ($thisop->{name} =~ /le$/ && $main::backend_endian eq 'little') ||
         ($thisop->{name} =~ /be$/ && $main::backend_endian eq 'big') ) {
        # nop
        $thisop->{in} = [];
        $thisop->{out} = [];
    }
}

sub out_swap {
    my ($thisop, $wo, $wi) = @_;

    return "# swap: nothing to do\n" unless defined $wi;

    my $r = "    wsbh $reg[$wo], $reg[$wi]\n";
    if ( $thisop->{name} =~ /^swap32/ ) {
        $r .= "    rotr $reg[$wo], $reg[$wo], 16\n";
    }
    return $r;
}

sub out_unpack {
    return out_pack_( @_ );
}

return 1;

