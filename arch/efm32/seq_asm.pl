#    This file is part of MutekH.
#    
#    MutekH is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation; version 2.1 of the
#    License.
#    
#    MutekH is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.
#    
#    You should have received a copy of the GNU Lesser General Public
#    License along with MutekH; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
#    02110-1301 USA.
#
#    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2017
#    Copyright Nicolas Pouillon <nipo@ssji.net>, 2017

#    Assembler tool for the EFR32 radio sequencer

use strict;

use seq_isa;

our @src;
our %labels;
our $fout;
our $fin;

my $baseaddr = 0x21000000;
my $racaddr = 0x40084000;

sub usage
{
    die "seq_asm.pl [-o output] [-b baseaddr] input\n";
}

while (defined $ARGV[0]) {
    my $opt = shift @ARGV;
    if ($opt !~ /^-/ && !defined $fin ) {
	$fin = $opt;
    } elsif ($opt eq '-o') {
        $fout = shift @ARGV;
    } elsif ($opt eq '-b') {
        $baseaddr = oct(shift @ARGV);
    } else {
	usage();
    }
}

if ( !defined $fin ) {
    usage();
}

if ( !defined $fout ) {
    $fout = $fin;
    $fout =~ s/\.\w+$//;
    $fout .= '.bin';
}

open( FOUT, ">:raw", $fout ) or die "unable to open output file.\n";
open( FIN, "<", $fin ) or die "unable to open input file.\n";

sub warning
{
    my ( $loc, $msg ) = @_;
    if ( ref $loc ) {
        $loc = "$loc->{file}:$loc->{line}: ";
    } else {
        $loc .= ": ";
    }
    print STDERR $loc.$msg;
}

sub error
{
    my ( $loc, $msg ) = @_;
    if ( ref $loc ) {
        $loc = "$loc->{file}:$loc->{line}: ";
    } else {
        $loc .= ": ";
    }
    die $loc.$msg;
}

sub args_split
{
    return map { s/^\s*|\s*$//g; $_ } split(/,/, shift);
}

sub check_reg
{
    my ( $thisop, $argidx, $count, $mem ) = @_;
    my $reg = $thisop->{args}->[$argidx];

    if ( $mem && $reg !~ s/^\[(.*)\]$/$1/ ) {
	error($thisop, "expected memory access as operand $argidx of `$thisop->{name}'.\n");
    }

    my $r = $seq_isa::reg_by_name{$reg};

    if ( !defined $r ) {
	error($thisop, "expected register as operand $argidx of `$thisop->{name}'.\n");
    }

    if ( $r >= $count ) {
	error($thisop, "can not use register `$reg' as operand $argidx of `$thisop->{name}'.\n");
    }

    return $r;
}

sub check_num
{
    my ( $thisop, $argidx, $min, $max, $mem ) = @_;
    my $expr = $thisop->{args}->[$argidx];

    if ( $mem && $expr !~ s/^\[(.*)\]$/$1/ ) {
	error($thisop, "expected memory access as operand $argidx of `$thisop->{name}'.\n");
    }

    my $bit = sub {
        my $x = shift;
        error($thisop, "bitpos() expects a power of 2.\n") if !$x or ($x & ($x - 1));
        return log2($x);
    };

    our $num = qr/(?>[-+]?\b\d+\b)/xs;

    while (1) {
        next if ($expr =~ s/'(.)'/ord($1)/ge);
        next if ($expr =~ s/\s*([-+]?)(0[Xx][a-fA-F0-9]+)\s*/$1.hex($2)/ge);
	next if ($expr =~ s/bitpos\(($num)\)/$bit->($1)/ge);
	next if ($expr =~ s/\(\s*($num)\s*\)/$1/ge);
	next if ($expr =~ s/($num)\s*\*\s*($num)/int($1)*int($2)/ge);
	next if ($expr =~ s/($num)\s*\/\s*($num)/int($2) ? int(int($1)\/int($2)) : 0/ge);
	next if ($expr =~ s/($num)\s*%\s*($num)/int($2) ? int($1)%int($2) : 0/ge);
	next if ($expr =~ s/($num)\s*\+\s*($num)/int($1)+int($2)/ge);
	next if ($expr =~ s/($num)\s*-\s*($num)/int($1)-int($2)/ge);
	next if ($expr =~ s/($num)\s*>>\s*($num)/int($1)>>int($2)/ge);
	next if ($expr =~ s/($num)\s*<<\s*($num)/int($1)<<int($2)/ge);
	next if ($expr =~ s/($num)\s*&\s*($num)/int($1)&int($2)/ge);
	next if ($expr =~ s/($num)\s*\|\s*($num)/int($1)|int($2)/ge);
	next if ($expr =~ s/($num)\s*\^\s*($num)/int($1)^int($2)/ge);
	last;
    }

    if ( $expr !~ /^($num)$/ ) {
        error($thisop, "expected number as operand $argidx of `$thisop->{name}', got `$expr'.\n");
	$expr = 0;
    }

    $expr = int($expr);

    if ( ( ( defined $max ) && $expr > $max ) ||
         ( ( defined $min ) && $expr < $min ) ) {
        error($thisop, "operand $argidx of `$thisop->{name}' is out of range [$min, $max].\n");
    }

    $thisop->{args}->[$argidx] = $expr;
    return $expr;
}

sub check_label
{
    my ( $thisop, $argidx ) = @_;
    my $lbl = $thisop->{args}->[$argidx];

    if ( $lbl !~ /^[a-zA-Z_]\w*$/ ) {
        error($thisop, "expected label as operand $argidx of `$thisop->{name}'.\n");
    }

    my $l = $labels{$lbl};

    if ( not defined $l ) {
        error($thisop, "undefined label `$lbl' used with `$thisop->{name}'.\n");
    }

    $l->{used}++;

    return $l;
}

sub parse
{
    my @lbls;
    my $line;
    my $file = "<STDIN>.bc";
    my $loc;

    foreach (<FIN>) {

      $line++;

      if (/^# (\d+) "(.*)"\s*$/) {
          # cpp location
          $line = $1 - 1;
          $file = $2;
          next;
      }

      s/#.*$//;
      s/\/\/.*$//;
      next if (/^\s*\/\//);

      $loc = "$file:$line";

      foreach my $l ( split(/;/) ) {

          next if ($l =~ /^\s*$/);

	  if ($l =~ /^\s*(\w+):\s*$/) {
	      my $name = $1;
	      error($loc, "label `$name' already defined\n")
		  if defined $labels{$name};
	      my $l = { name => $name };
	      $labels{$name} = $l;
	      push @lbls, $l;
	      next;
	  }

	  if ($l =~ /^\s*\.baseaddr\s+(\w+)/) {
	      $baseaddr = oct($1);
	      next;
	  }

	  if ($l =~ /^\s*\.(\w+)/) {
	      error($loc, "bad directive syntax `.$1'\n");
	  }

	  if ($l =~ /^\s*(\w[.\w]*)\b\s*(.*?)\s*$/) {
	      my $opname = $1;
	      my @args = args_split($2);
	      my $thisop = {
		  src => "$1 $2",
		  name => $opname,
		  file => $file,
		  line => $line,
		  args => [ @args ],
		  in => [],
		  out => [],
		  labels => [ @lbls ],
	      };
	      push @src, $thisop;
	      @lbls = ();
	      next;
	  }

	  error($loc, "syntax error: `$l'\n");
      }
    }
}

sub assemble
{
    my $addr = $baseaddr;

    my %encode_ctx = (
	racaddr => $racaddr,
	check_reg => \&check_reg,
	check_num => \&check_num,
	check_label => \&check_label,
	error => \&error,
	warning => \&warning,
    );

    foreach my $thisop (@src) {

	my $op = $seq_isa::ops_by_name{$thisop->{name}};

	error($thisop, "unknown instruction `$thisop->{name}'\n")
	    unless defined $op;

	error($thisop, "can not encode instruction `$thisop->{name}'\n")
	    unless defined $op->{encode};

	warning($thisop, "use of reserved instruction `$thisop->{name}'\n")
	    if $op->{reserved};

	$thisop->{op} = $op;
	$thisop->{addr} = $addr;
        foreach my $l ( @{$thisop->{labels}} ) {
	    $l->{addr} = $addr;
	}

	$addr += $op->{len};
    }

    my $c = 0;

    foreach my $thisop (@src) {
	my $op = $thisop->{op};
	$op->{encode}->( \%encode_ctx, $thisop );
	die "BAD binary len for $thisop->{loc} $op->{name}\n"
	    unless $op->{len} == scalar @{$thisop->{bin}};
	$c += scalar @{$thisop->{bin}};

	print FOUT pack("C*", @{$thisop->{bin}});
    }

    #    printf STDERR "binary size: %u bytes\n", $c;
}

parse();
assemble();

close( FIN );
close( FOUT );
