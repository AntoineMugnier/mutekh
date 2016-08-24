
use strict;

our $instdir;
our @src;
our %labels;
our %defs;
our $backend_endian = "little";
our $backend_width = 0;		# 8 << 2 bits
our $backend_name = "bytecode";
our $bc_name = "mybytecode";
our $fout = "bc.out";
our $fheader;
our $last_addr;
our $modpath = $ENV{MUTEK_BYTECODE_PATH};
our %srcaddr;

BEGIN {
    use Cwd;
    my $script = "".__FILE__;
    $script = getcwd."/$script" if ($script !~ /^\//);
    $script = readlink $script while ( -l $script );
    $script =~ /(.+)\/([^\/]+)/;
    $instdir = $1;
    push @INC, $1;
}

while (defined $ARGV[0]) {
    my $opt = shift @ARGV;
    if ($opt eq '-o') {
        $fout = shift @ARGV;
        next;
    }
    if ($opt eq '-b') {
        $backend_name = shift @ARGV;
        next;
    }
    if ($opt eq '-e') {
        $backend_endian = shift @ARGV;
        next;
    }
    if ($opt eq '-h') {
        $fheader = shift @ARGV;
        next;
    }
    if ($opt eq '-n') {
        $bc_name = shift @ARGV;
        next;
    }
    if ($opt eq '-p') {
        $modpath .= ':'.(shift @ARGV);
        next;
    }
    if ($opt eq '-w') {
        $backend_width = shift @ARGV;
        next;
    }
    die "bad command line option `$opt'\n";
}

die "missing -w option\n" if !$backend_width;
$defs{_SIZEOF_PTR} = 1 << $backend_width;

sub log2
{
    return int (log(shift) / log(2));
}

our @warnings;

sub warning
{
    my ( $loc, $msg ) = @_;
    if ( ref $loc ) {
        push @warnings, { obj => $loc, msg => $msg };
    } else {
        print STDERR $loc.": ".$msg;
    }
}

sub warnings_print
{
    foreach my $w ( sort { $a->{obj}->{line} <=> $b->{obj}->{line} } @warnings ) {
        print STDERR "$w->{obj}->{file}:$w->{obj}->{line}: $w->{msg}";
    }
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

sub check_reg
{
    my ( $thisop, $argidx ) = @_;
    my $reg = $thisop->{args}->[$argidx];

    if ( $reg !~ /^%(\d+)$/ || $1 > 15 ) {
        error($thisop, "expected register as operand $argidx of `$thisop->{name}'.\n");
    }

    return $1;
}

sub check_num
{
    my ( $thisop, $argidx, $min, $max ) = @_;
    my $expr = $thisop->{args}->[$argidx];

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

    if ( not $l->{export} ) {
        $thisop->{args}->[$argidx] = "L$lbl";
    }

    $thisop->{disp} = $l->{addr} - $thisop->{addr} - 1;

    $l->{used}++;
    return $l;
}

sub check_label8
{
    my ( $thisop, $argidx, $bound ) = @_;
    my $l = check_label( $thisop, $argidx );

    if ($thisop->{disp} > $bound - 1 || $thisop->{disp} < -$bound) {
        error($thisop, "jump target out of range for `$thisop->{name}'.\n");
    }

    return $l;
}

sub parse_trace
{
    my $thisop = shift;

    check_num($thisop, 0, 0, 1);
    check_num($thisop, 1, 0, 1);
}

sub parse_cmp1
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
}

sub parse_cmp2
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 1);
    push @{$thisop->{in}}, check_reg($thisop, 0);

    if ( $thisop->{in}->[0] == $thisop->{in}->[1] ) {
        error($thisop, "compare to the same register is not allowed\n");
    }
}

sub parse_alu
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    push @{$thisop->{out}}, check_reg($thisop, 0);
}

sub parse_alu1
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{out}}, check_reg($thisop, 0);
}

sub parse_alu2
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    push @{$thisop->{out}}, check_reg($thisop, 0);

    if ( $thisop->{in}->[0] == $thisop->{in}->[1] ) {
        error($thisop, "the register same register can not be used twice\n");
    }
}

sub parse_mov
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 1);
    push @{$thisop->{out}}, check_reg($thisop, 0);

    if ( $thisop->{in}->[0] == $thisop->{out}->[0] ) {
        error($thisop, "the register same register can not be used twice\n");
    }
}

sub parse_add8
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{out}}, check_reg($thisop, 0);
    check_num($thisop, 1, -128, 127);
}

sub parse_cst8
{
    my $thisop = shift;

    push @{$thisop->{out}}, check_reg($thisop, 0);
    check_num($thisop, 1, 0, 255);
}

sub parse_cst
{
    my $thisop = shift;

    push @{$thisop->{out}}, check_reg($thisop, 0);

    if ( check_num($thisop, 2, 0, 56) % 8 ) {
        error($thisop, "cst shift must be a multiple of 8\n");
    }

    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : 2;

    check_num( $thisop, 1, 0, 0xffffffffffffffff >> (64 - (8 << $thisop->{width})) );
}

sub parse_data
{
    my $thisop = shift;

    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : 2;

    check_num( $thisop, 0, 0, 0xffffffffffffffff >> (64 - (8 << $thisop->{width})) );
}

sub parse_laddr
{
    my $thisop = shift;

    $thisop->{name} =~ /^[a-z]+(\d+)/ ;
    $thisop->{width} = $1 ? log2($1) - 3 : 2;

    push @{$thisop->{out}}, check_reg($thisop, 0);
    $thisop->{target} = check_label($thisop, 1);
}

sub parse_gaddr
{
    my $thisop = shift;

    if ( $thisop->{args}->[1] !~ /^[a-zA-Z_]\w*$/ ) {
        error($thisop, "expected symbol name as operand 1 of `gaddr'.\n");
    }

    push @{$thisop->{out}}, check_reg($thisop, 0);
}

sub parse_bitop
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{out}}, check_reg($thisop, 0);

    check_num($thisop, 1, 0, 31);
}

sub parse_tst
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);

    check_num($thisop, 1, 0, 31);
}

sub parse_jmp8
{
    my $thisop = shift;

    $thisop->{target} = check_label8($thisop, 0, 128);

    if ( $thisop->{disp} == 0 ) {
	error($thisop, "jmp can not have a zero displacement\n");
    }
}

sub parse_call8
{
    my $thisop = shift;

    $thisop->{lr} = check_reg($thisop, 0);
    $thisop->{target} = check_label8($thisop, 1, 128);

    if ( $thisop->{lr} == 0 ) {
	error($thisop, "call8 can not modify register 0\n");
    }

    if ( $thisop->{disp} == 0 ) {
	error($thisop, "call8 can not have a zero displacement\n");
    }
}

sub parse_call32
{
    my $thisop = shift;

    $thisop->{lr} = check_reg($thisop, 0);
    $thisop->{target} = check_label($thisop, 1);

    if ( $thisop->{lr} == 0 ) {
	error($thisop, "call32 can not modify register 0\n");
    }
}

sub parse_jmp32
{
    my $thisop = shift;

    $thisop->{target} = check_label($thisop, 0);
}

sub parse_noarg
{
}

sub parse_ret
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
}

sub parse_loop
{
    my $thisop = shift;

    push @{$thisop->{out}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 0);
    $thisop->{target} = check_label8($thisop, 1, 64);
}

our $le = ($main::backend_endian eq 'little');
die unless $le or $main::backend_endian eq 'big';

our %packops = (
    'pack8' =>      'bc_pack_op8',
    'unpack8' =>    'bc_unpack_op8',
    'pack16le' =>   $le ? 'bc_pack_op16'        : 'bc_swap_pack_op16',
    'pack16be' =>   $le ? 'bc_swap_pack_op16'   : 'bc_pack_op16',
    'pack32le' =>   $le ? undef                 : 'bc_swap_pack_op32',
    'pack32be' =>   $le ? 'bc_swap_pack_op32'   : undef,
    'unpack16le' => $le ? 'bc_unpack_op16'      : 'bc_unpack_swap_op16',
    'unpack16be' => $le ? 'bc_unpack_swap_op16' : 'bc_unpack_op16',
    'unpack32le' => $le ? undef                 : 'bc_unpack_swap_op32',
    'unpack32be' => $le ? 'bc_unpack_swap_op32' : undef,
);

our %packbytes = (
    'pack8' =>      1,
    'unpack8' =>    1,
    'pack16le' =>   2,
    'pack16be' =>   2,
    'pack32le' =>   4,
    'pack32be' =>   4,
    'unpack16le' => 2,
    'unpack16be' => 2,
    'unpack32le' => 4,
    'unpack32be' => 4,
);

sub parse_pack
{
    my $thisop = shift;

    my $r = check_reg($thisop, 0);
    my $count = check_num($thisop, 1, 1, 8);
    if ($r + $count > 16) {
        error($thisop, "out of range register\n");
    }
    for (my $i = 0; $i < $count; $i++) {
        push @{$thisop->{in}}, $r + $i;
    }
    $thisop->{count} = $count;
    $thisop->{packout_reg} = $r;
    $thisop->{packout_bytes} = $count * $packbytes{$thisop->{name}};
}

sub parse_unpack
{
    my $thisop = shift;

    my $r = check_reg($thisop, 0);
    my $count = check_num($thisop, 1, 1, 8);
    if ($r + $count > 16) {
        error($thisop, "out of range register\n");
    }
    for (my $i = 0; $i < $count; $i++) {
        push @{$thisop->{out}}, $r + $i;
    }
    $thisop->{count} = $count;
    $thisop->{packin_reg} = $r;
    $thisop->{packin_bytes} = $count * $packbytes{$thisop->{name}};
}

sub parse_ld
{
    my $thisop = shift;

    push @{$thisop->{out}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : $backend_width;
}

sub parse_st
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : $backend_width;
}

sub parse_lde
{
    my $thisop = shift;

    push @{$thisop->{out}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    check_num($thisop, 2, -32768, 32767);
    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : $backend_width;
}

sub parse_ste
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    check_num($thisop, 2, -32768, 32767);
    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : $backend_width;
}

sub parse_ldi
{
    my $thisop = shift;

    push @{$thisop->{out}}, check_reg($thisop, 0);
    push @{$thisop->{out}}, check_reg($thisop, 1);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : $backend_width;
}

sub parse_sti
{
    my $thisop = shift;

    push @{$thisop->{in}}, check_reg($thisop, 0);
    push @{$thisop->{out}}, check_reg($thisop, 1);
    push @{$thisop->{in}}, check_reg($thisop, 1);
    $thisop->{name} =~ /^[a-z]+(\d+)/;
    $thisop->{width} = $1 ? log2($1) - 3 : $backend_width;
}

sub load_module
{
    my ($mod_name, $prefix) = @_;
    my $mod = $prefix.'_'.$mod_name;

    foreach my $dir ( $instdir, split(/:/, $modpath ) ) {
	next if not -d $dir;
        my $fname = "$dir/$mod.pm";
        next if not -f $fname;
        require $fname;
        return $mod;
    }

    die "unable to find module `$mod.pm' in MUTEK_BYTECODE_PATH\n";
}

my @custom;

sub args_split
{
    return map { s/^\s*|\s*$//g; $_ } split(/,/, shift);
}

sub regs_mask_parse
{
    my ($loc, $class, $str, $maskref, $alias) = @_;
    my $mask = 0;
    our $alias_id;

    foreach my $reg (args_split($str)) {
        error($loc, "bad register `$reg'\n")
            if ( $reg !~ /^%(\d+)\s*((?:\s*\b[_a-z]\w*)*)\s*$/ || $1 > 15 );

        foreach ( split(/\s+/, $2) ) {
            error($loc, "register alias can't be used with .$class\n")
                unless defined $alias;
            error($loc, "register alias `$_' already defined\n")
                if defined $alias->{$_};
            $alias->{$_} = {
                name => $_,
                reg => $1,
                id => $alias_id++,
                class => $class
            };
        }

        error($loc, "$class register \%$1 used more than once\n")
            if ($$maskref & (1 << $1));
        $$maskref |= 1 << $1;
    }
}

my $global_regmask = 0;
my $global_const_regmask = 0;
my %global_regalias;

sub parse
{
    my @lbls;
    my $line;
    my $file = "<STDIN>.bc";
    my $loc;
    my $func;

    foreach (<STDIN>) {

      $line++;

      if (/^# (\d+) "(.*)"\s*$/) {
          # cpp location
          $line = $1 - 1;
          $file = $2;
          next;
      }

      next if (/^\s*#/);
      next if (/^\s*\/\//);

      $loc = "$file:$line";

      foreach my $l ( split(/;/) ) {

          next if ($l =~ /^\s*$/);

        if ($l =~ /^\s*(\w+):\s*$/) {
            my $name = $1;
            error($loc, "label `$name' already defined\n") if defined $labels{$name};
	    my $l = { name => $name };
            $labels{$name} = $l;
	    push @lbls, $l;
            next;
        }
        if ($l =~ /^\s*\.define\s+(\w+)\s+(.*?)\s*$/) {
            error($loc, "expr already defined\n") if defined $defs{$1};
            $defs{$1} = $2;
            next;
        }
        if ($l =~ /^\s*\.func\s+(\w+)\s*$/) {
            my $name = $1;
            error($loc, "label `$name' already defined\n") if defined $labels{$name};
            error($loc, "nested function\n") if defined $func;
            error($loc, "label before a function\n") if scalar @lbls;
	    my $l = { file => $file,
                      line => $line,
                      name => $name,
                      func => 1,
                      input => 0,
                      output => 0,
                      clobber => 0,
                      preserve => 0,
                      regalias => { }
            };
            $labels{$name} = $l;
	    push @lbls, $l;
            $func = $l;
            next;
        }
        if ($l =~ /^\s*\.endfunc\s*$/) {
            error($loc, "no function to end\n") if !defined $func;
            error($loc, "label at end of function\n") if scalar @lbls;
            # declare global aliases for function input/output regs
            while (my ($k, $v) = each(%{$func->{regalias}})) {
                $global_regalias{$func->{name}.':'.$k} = $v
                    unless ($func->{temp} & (1 << $v));
            }
            $func = undef;
            next;
        }
        if ($l =~ /^\s*\.entry\s+(.*?)\s*$/) {
            error($loc, "entry point inside a function\n") if defined $func;
            error($loc, "missing label before entry point\n") if !scalar @lbls;
            my $l = @lbls[-1];
            $l->{export} = 1;
            $l->{used}++;
            regs_mask_parse($loc, 'entry', $1, \$l->{entry}, undef);
            next;
        }
        if ($l =~ /^\s*\.(global|const)\s+(.*?)\s*$/) {
            error($loc, "global register declared inside a function\n") if defined $func;
            my $regs;
            regs_mask_parse($loc, $1, $2, \$regs, \%global_regalias);
            $global_regmask |= $regs;
            $global_const_regmask |= $regs if $1 eq 'const';
            next;
        }
        if ($l =~ /^\s*\.(input|output|clobber|preserve)\s+(.*?)\s*$/) {
            error($loc, "$1 regs not inside a function\n") if !defined $func;
            regs_mask_parse($loc, $1, $2, \$func->{$1}, $func->{regalias} );
            next;
        }
        if ($l =~ /^\s*\.backend\s+(\w+)\s*$/) {
            $backend_name = $1;
            next;
        }
        if ($l =~ /^\s*\.name\s+(\w+)\s*$/) {
            $bc_name = $1;
            next;
        }
        if ($l =~ /^\s*\.custom\s+([\w.]+)\s*$/) {
	    push @custom, $1;
            next;
        }
        if ($l =~ /^\s*\.export\s+(\w+)\s*$/) {
	    my $l = $labels{$1};
            error($loc, "undefined label `$1'\n") if not defined $l;
            $l->{export} = 1;
            $l->{used}++;
            next;
        }
        if ($l =~ /^\s*\.(\w+)/) {
            error($loc, "bad directive syntax `.$1'\n");
        }
        if ($l =~ /^\s*(\.?\w+)\b\s*(.*?)\s*$/) {
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
                regs => [ (0) x 16 ],
                labels => [ @lbls ],
                func => $func,
            };
            push @src, $thisop;
            @lbls = ();
            next;
        }

        error($loc, "syntax error: `$l'\n");
      }
    }

    error($loc, "function not ended\n") if defined $func;
    if ( scalar @lbls ) {
        error($loc, "found trailing label\n");
    }
}

parse();

my $backend = load_module($backend_name, "bc_backend");

sub _multi_keys # for easy init of multiple hash keys with same value
{
    my $val = pop;
    return map { ( $_, $val ) } @_;
}

our %asm = (
    'end' => {
        words => 1, code => 0x0000, argscnt => 0,
        parse => \&parse_noarg, backend => ('end'),
	flushregs => 1, op_tail => 1, op_ret => 1
    },
    'dump'  => {
        words => 1, code => 0x0001, argscnt => 0,
        parse => \&parse_noarg, backend => ('dump'),
        flushregs => 1,
    },
    'abort'  => {
        words => 1, code => 0x0002, argscnt => 0,
        parse => \&parse_noarg, backend => ('abort'),
        op_tail => 1
    },
    'die'  => {
        words => 1, code => 0x0003, argscnt => 0,
        parse => \&parse_noarg, backend => ('die'),
        op_tail => 1
    },
    'nop'  => {
        words => 1, code => 0x0004, argscnt => 0,
        parse => \&parse_noarg, backend => ('nop'),
    },
    'trace'  => {
        words => 1, code => 0x0008, argscnt => 2,
        parse => \&parse_trace, backend => ('trace'),
    },
    'add8'  => {
        words => 1, code => 0x0000, argscnt => 2,
        parse => \&parse_add8, backend => ('add8'),
    },
    'cst8'  => {
        words => 1, code => 0x1000, argscnt => 2,
        parse => \&parse_cst8, backend => ('cst8'),
    },
    'jmp8'  => {
        words => 1, code => 0x2000, argscnt => 1,
        parse => \&parse_jmp8, backend => ('jmp8'),
        flushregs => 1, op_jmp => 1, op_tail => 1
    },
    'call8'  => {
        words => 1, code => 0x2000, argscnt => 2,
        parse => \&parse_call8, backend => ('call8'),
        flushregs => 1, reloadregs => 1, op_call => 1,
    },
    'jmp32'  => {
        words => 3, code => 0x7010, argscnt => 1,
        parse => \&parse_jmp32, backend => ('jmp32'),
        flushregs => 1, op_jmp => 1, op_tail => 1
    },
    'call32'  => {
        words => 3, code => 0x7010, argscnt => 2,
        parse => \&parse_call32, backend => ('call32'),
        flushregs => 1, reloadregs => 1, op_call => 1,
    },
    'ret' => {
        words => 1, code => 0x2000, argscnt => 1,
        parse => \&parse_ret, backend => ('ret'),
        flushregs => 1, op_tail => 1, op_ret => 1
    },
    'jmp' => {  # same implementation as ret, different semantics
        words => 1, code => 0x2000, argscnt => 1,
        parse => \&parse_ret, backend => ('ret'),
        flushregs => 1, op_tail => 1
    },
    'loop' => {
        words => 1, code => 0x3000, argscnt => 2,
        parse => \&parse_loop, backend => ('loop'),
        flushregs => 1, op_jmp => 1
    },
    _multi_keys( 'pack8' => 'pack16le' => 'pack16be' =>
                 'pack32le' => 'pack32be' => {
        words => 1, code => 0x3800, argscnt => 2,
        parse => \&parse_pack, backend => ('pack'),
        nocond => 1
    }),
    _multi_keys( 'unpack8' => 'unpack16le' => 'unpack16be' =>
                 'unpack32le' => 'unpack32be' => {
        words => 1, code => 0x3800, argscnt => 2,
        parse => \&parse_unpack, backend => ('unpack'),
        nocond => 1
    }),
    _multi_keys( 'swap16le' => 'swap16be' => 'swap16' =>
                 'swap32le' => 'swap32be' => 'swap32' => {
        words => 1, code => 0x3800, argscnt => 1,
        parse => \&parse_alu1, backend => ('swap'),
    }),
    'eq'  => {
        words => 1, code => 0x4000, argscnt => 2,
        parse => \&parse_cmp2, backend => ('eq'),
        op_cond => 1,
    },
    'eq0'  => {
        words => 1, code => 0x4000, argscnt => 1,
        parse => \&parse_cmp1, backend => ('eq0'),
        op_cond => 1,
    },
    'neq'  => {
        words => 1, code => 0x4100, argscnt => 2,
        parse => \&parse_cmp2, backend => ('neq'),
        op_cond => 1,
    },
    'neq0'  => {
        words => 1, code => 0x4100, argscnt => 1,
        parse => \&parse_cmp1, backend => ('neq0'),
        op_cond => 1,
    },
    'lt'  => {
        words => 1, code => 0x4200, argscnt => 2,
        parse => \&parse_cmp2, backend => ('lt'),
        op_cond => 1,
    },
    'lteq'  => {
        words => 1, code => 0x4300, argscnt => 2,
        parse => \&parse_cmp2, backend => ('lteq'),
        op_cond => 1,
    },
    'add' => {
        words => 1, code => 0x4400, argscnt => 2,
        parse => \&parse_alu2, backend => ('add')
    },
    'sub' => {
        words => 1, code => 0x4500, argscnt => 2,
        parse => \&parse_alu2, backend => ('sub')
    },
    'neg' => {
        words => 1, code => 0x4500, argscnt => 1,
        parse => \&parse_alu1, backend => ('neg')
    },
    'mul32' => {
        words => 1, code => 0x4700, argscnt => 2,
        parse => \&parse_alu, backend => ('mul')
    },
    'or32' => {
        words => 1, code => 0x4800, argscnt => 2,
        parse => \&parse_alu2, backend => ('or')
    },
    'xor32' => {
        words => 1, code => 0x4900, argscnt => 2,
        parse => \&parse_alu, backend => ('xor')
    },
    'and32' => {
        words => 1, code => 0x4a00, argscnt => 2,
        parse => \&parse_alu2, backend => ('and')
    },
    'ccall' => {
        words => 1, code => 0x4b00, argscnt => 2,
        parse => \&parse_alu, backend => ('ccall'),
        flushregs => 1,
    },
    'shl32' => {
        words => 1, code => 0x4c00, argscnt => 2,
        parse => \&parse_alu, backend => ('shl'),
    },
    'shr32' => {
        words => 1, code => 0x4d00, argscnt => 2,
        parse => \&parse_alu, backend => ('shr'),
    },
    'andn32' => {
        words => 1, code => 0x4e00, argscnt => 2,
        parse => \&parse_alu2, backend => ('andn')
    },
    'not32' => {
        words => 1, code => 0x4e00, argscnt => 1,
        parse => \&parse_alu1, backend => ('not')
    },
    'mov' => {
        words => 1, code => 0x4f00, argscnt => 2,
        parse => \&parse_mov, backend => ('mov'),
        op_mov => 1
    },
    'msbs32' => {
        words => 1, code => 0x4f00, argscnt => 1,
        parse => \&parse_alu1, backend => ('msbs')
    },
    'tst32c' => {
        words => 1, code => 0x5000, argscnt => 2,
        parse => \&parse_tst, backend => ('tstc'),
        op_cond => 1,
    },
    'tst32s' => {
        words => 1, code => 0x5200, argscnt => 2,
        parse => \&parse_tst, backend => ('tsts'),
        op_cond => 1,
    },
    'bit32c' => {
        words => 1, code => 0x5400, argscnt => 2,
        parse => \&parse_bitop, backend => ('bitc'),
    },
    'bit32s' => {
        words => 1, code => 0x5600, argscnt => 2,
        parse => \&parse_bitop, backend => ('bits'),
    },
    'shi32l' => {
        words => 1, code => 0x5800, argscnt => 2,
        parse => \&parse_bitop, backend => ('shil'),
    },
    'shi32r' => {
        words => 1, code => 0x5a00, argscnt => 2,
        parse => \&parse_bitop, backend => ('shir'),
    },
    'exts' => {
        words => 1, code => 0x5e00, argscnt => 2,
        parse => \&parse_bitop, backend => ('exts'),
    },
    'extz' => {
        words => 1, code => 0x5c00, argscnt => 2,
        parse => \&parse_bitop, backend => ('extz'),
    },
    _multi_keys( 'ld' => 'ld8' => 'ld16' => 'ld32' => 'ld64' => {
        words => 1, code => 0x6000, argscnt => 2,
        parse => \&parse_ld, backend => ('ld'),
    }),
    _multi_keys( 'ldi' => 'ld8i' => 'ld16i' => 'ld32i' => 'ld64i' => {
        words => 1, code => 0x6100, argscnt => 2,
        parse => \&parse_ldi, backend => ('ldi'),
    }),
    _multi_keys( 'lde' => 'ld8e' => 'ld16e' => 'ld32e' => 'ld64e' => {
        words => 2, code => 0x7100, argscnt => 3,
        parse => \&parse_lde, backend => ('lde'),
    }),
    _multi_keys( 'st' => 'st8' => 'st16' => 'st32' => 'st64' => {
        words => 1, code => 0x6800, argscnt => 2,
        parse => \&parse_st, backend => ('st'),
    }),
    _multi_keys( 'sti' => 'st8i' => 'st16i' => 'st32i' => 'st64i' => {
        words => 1, code => 0x6900, argscnt => 2,
        parse => \&parse_sti, backend => ('sti'),
    }),
    _multi_keys( 'std' => 'st8d' => 'st16d' => 'st32d' => 'st64d' => {
        words => 1, code => 0x7800, argscnt => 2,
        parse => \&parse_sti, backend => ('std'),
    }),
    _multi_keys( 'ste' => 'st8e' => 'st16e' => 'st32e' => 'st64e' => {
        words => 2, code => 0x7900, argscnt => 3,
        parse => \&parse_ste, backend => ('ste'),
    }),
    'cst16' => {
        words => 2, code => 0x7210, argscnt => 3,
        parse => \&parse_cst, backend => ('cst'),
    },
    'cst32' => {
        words => 3, code => 0x7410, argscnt => 3,
        parse => \&parse_cst, backend => ('cst'),
    },
    'cst64' => {
        words => 5, code => 0x7610, argscnt => 3,
        parse => \&parse_cst, backend => ('cst'),
    },
    'laddr16' => {
        words => 2, code => 0x7200, argscnt => 2,
        parse => \&parse_laddr, backend => ('laddr'),
    },
    'laddr32' => {
        words => 3, code => 0x7400, argscnt => 2,
        parse => \&parse_laddr, backend => ('laddr'),
    },
    'gaddr' => {
        words => 1 + (4 << $backend_width) / 8, code => 0x7000, argscnt => 2,
        parse => \&parse_gaddr, backend => ('gaddr'),
    },
    'data16' => {
        words => 1, argscnt => 1,
        parse => \&parse_data, backend => ('data'),
        op_data => 1
    },
);

load_module($_, "bc_custom") foreach (@custom);

sub custom_op
{
    my ( $name, $args, $code, $parse ) = @_;

    $parse ||= sub {};

    my $op = {
        words => 1, code => 0x8000 | $code,
        argscnt => $args,
        parse => sub {
            my $thisop = shift;
            $parse->( $thisop );

            # prevent load of instruction input register
            $thisop->{flushin} = (1 << scalar @{$thisop->{in}}) - 1;
        },
        backend => 'custom',
        flushregs => 1,
        reloadregs => 1,
    };

    $asm{$name} = $op;

    return $op;
}

sub custom_cond_op
{
    my ( $name, $args, $code, $parse ) = @_;

    my $op = custom_op( $name, $args, $code, $parse );

    $op->{backend} = 'custom_cond';
    $op->{op_cond} = 1;

    return $op;
}

sub parse_args
{
    my $addr = 0;
    my $prevop;

    foreach my $thisop (@src) {

	my $op = $asm{$thisop->{name}};

	error($thisop, "unknown instruction `$thisop->{name}'\n")
	    unless defined $op;

	error($thisop, "bad operand count for `$thisop->{name}'\n")
	    if (defined $op->{argscnt}) && $op->{argscnt} != @{$thisop->{args}};

	error($thisop, "multi-word instruction after conditional\n")
	    if $prevop && $prevop->{op}->{op_cond} && $op->{words} > 1;

        error($thisop, "instruction can not be conditional\n")
	    if $prevop && $prevop->{op}->{op_cond} && ($op->{op_cond} || $op->{nocond});

	$thisop->{op} = $op;
	$thisop->{addr} = $addr;

        # bit mask of arguments which require a register write back
        # rather than a register load, updated by the backend parse
        # handler
        $thisop->{flushin} = 0;

        # bit mask of arguments which are written back by the instruction,
        # ignored when flushin is set for the argument
        $thisop->{wbin} = 0;

        # bit mask of arguments which require a register reload rather
        # than an output register allocation, updated by the backend
        # parse handler
        $thisop->{reloadout} = 0;

        # bit mask of clobbered cpu registers
        $thisop->{clobber} = 0;

        $srcaddr{$addr} = $thisop;
        foreach my $l ( @{$thisop->{labels}} ) {
	    $l->{addr} = $addr;
	}

	$addr += $op->{words} + $thisop->{words};
        $prevop = $thisop;
    }

    error($prevop, "conditional at end of program\n")
        if ( $prevop && $prevop->{op}->{op_cond} );

    $last_addr = $addr;

    foreach my $thisop (@src) {

	# substitute defs in args
 	my $r = sub {
	    my $s = shift;
	    return defined $defs{$s} ? $defs{$s} : $s;
	};
 	my $a = sub {
            my $s = shift;
            my $t;
            my $g = $global_regalias{$s};
            if ( my $f = $thisop->{func} ) {
                $t = $f->{regalias}->{$s};
                if ( defined $t ) {
                    warning($thisop, "not using global register alias `$s'\n")
                        if ( defined $g );
                } else {
                    $t = $g;
                }
            } elsif ( defined $g ) {
                $t = $g;
            }
            error($thisop, "undefined register alias `$s'\n")
                unless ( defined $t );
	    return '%'.$t->{reg};
	};
	foreach my $arg (@{$thisop->{args}}) {
	    $arg =~ s/%([a-z_][\w:]*)\b/$a->($1)/ge;
	    $arg =~ s/\b([a-zA-Z_]\w*)\b/$r->($1)/ge;
	}

	$thisop->{op}->{parse}->( $thisop );

        my $opbackend = $thisop->{op}->{backend};
        if (my $bparse = $backend->can('parse_'.$opbackend)) {
            $bparse->( $thisop );
        }
    }
}

parse_args();

sub write_addr
{
    open(OUT, ">$fout.adr") || die "unable to open output file `$fout.adr'.\n";

    foreach my $thisop (@src) {

        foreach my $l ( @{$thisop->{labels}} ) {
            print OUT "$l->{name}:\n";
        }

        printf OUT " %4u   %-15s %s\n", $thisop->{addr}, $thisop->{src};
    }

    close( OUT );
}

sub write_bc
{
    open(OUT, ">$fout") || die "unable to open output file `$fout'.\n";

    print OUT $backend->out_begin( $last_addr );

    foreach my $thisop (@src) {

        foreach my $l ( @{$thisop->{labels}} ) {
            if ( $l->{export} ) {
                print OUT "$l->{name}:\n";
                print OUT "     .globl $l->{name}\n";
            };
        }

        # print STDERR $thisop->{name}."\n";
        printf OUT "    %-20s  # %s %s\n",
          $backend->can('out_'.$thisop->{op}->{backend})->( $thisop ),
          $thisop->{name}, join(', ', @{$thisop->{args}});
    }

    print OUT $backend->out_eof();

    close( OUT );
}

sub write_header
{
    open(OUT, ">$fheader") || die "unable to open output file `$fheader'.\n";

    print OUT "
#ifndef __BC_DEFS_${bc_name}__
#define __BC_DEFS_${bc_name}__
#include <mutek/bytecode.h>
";

    # emit defs for global registers
    foreach my $a ( keys %global_regalias ) {
        my $r = $global_regalias{$a};

        if ($r->{class} eq "const") {
            print OUT "#define ".uc($bc_name)."_BCCONST_".uc($a)." ".$r->{reg}."\n";
        }

        if ($r->{class} eq "global") {
            print OUT "#define ".uc($bc_name)."_BCGLOBAL_".uc($a)." ".$r->{reg}."\n";
        }
    }

    # emit defs for the exported label/function
    foreach my $l ( values %labels ) {
        next unless $l->{export};
        print OUT "extern bytecode_entry_t $l->{name};\n";

        next unless $l->{func};
        my @inputs;

        # input and output function registers
        foreach my $a ( keys %{$l->{regalias}} ) {
            my $r = $l->{regalias}->{$a};

            if ($r->{class} eq "output") {
                print OUT "#define ".uc($l->{name})."_BCOUT_".uc($a)." ".$r->{reg}."\n";
            }

            if ($r->{class} eq "input") {
                print OUT "#define ".uc($l->{name})."_BCIN_".uc($a)." ".$r->{reg}."\n";
                push @inputs, $r;
            }
        }

        # for use with the bc_set_regs_va function
        print OUT "#define ".uc($l->{name})."_BCARGS(";
        print OUT join(', ', map { $_->{name} } sort { $a->{id} <=> $b->{id} } @inputs);
        printf OUT ") 0x%x, ", $l->{input};
        print OUT join(', ', map { '('.$_->{name}.')' } sort { $a->{reg} <=> $b->{reg} } @inputs);
        print OUT "\n";
    }

    print OUT "#endif
";

    close( OUT );
}


sub write_asm
{
    my $wcnt = shift;		# number of cpu working registers
    my %w2r;			# cpu to vm register mapping
    my %r2w;			# vm to cpu register mapping
    my %wb;			# writeback vm register flags
    my @wfifo;			# cpu register allocation fifo

    die unless $wcnt >= 4;

    open(OUT, ">$fout") || die "unable to open output file `$fout'.\n";

    my $wreg_flush = sub {
        my $w = shift;
        # write back the vm regs previously associated to the cpu working reg
        foreach my $r (keys %{$w2r{$w}}) {
            if ($wb{$r}) {
                print OUT $backend->out_store($r, $r2w{$r});
                $wb{$r} = 0;
            }
            delete $r2w{$r};
        }
    };

    my $reg_alloc = sub {
        my $res = shift @wfifo;
        push @wfifo, $res;

        # print "  #alloc $res\n";
        $wreg_flush->($res);
        return $res;
    };

    my $reg_use = sub {
        # move register to the end of allocation fifo
        my $w = shift;
        @wfifo = (( grep { $_ != $w } @wfifo ), $w);
        # print "  #hot $w\n";
    };

    my $reg_flush = sub {
        my $r = shift;
        if ( $wb{$r} ) {
            my $w = $r2w{$r};
            print OUT $backend->out_store($r, $w);
            $wb{$r} = 0;
        }
    };

    my $regs_flush = sub {
        while (my ($r, $d) = each(%wb)) {
            $reg_flush->( $r );
        }
    };

    my $reg_reload = sub {
        my $r = shift;
        die "register $r not flushed" if $wb{$r};
        my $w = $r2w{$r};
        delete $r2w{$r};
        $w2r{$w} = {};
    };

    my $regs_reload = sub {
        while (my ($r, $d) = each(%wb)) {
            die "register $r not flushed" if $d;
        }
        for (my $w = 0; $w < $wcnt; $w++) {
            $w2r{$w} = {};
            $wfifo[$w] = $w;
        }
        %r2w = ();
    };

    $regs_reload->();

    my $cond;

    print OUT $backend->out_begin();

    foreach my $thisop (@src) {

        # while (my ($w, $l) = each(%w2r)) {
        #     foreach my $r (keys %$l) {
        #         die "$r $w" unless $r2w{$r} == $w;
        #     }
        # }
	#
        # while (my ($r, $w) = each(%r2w)) {
        #     die "$r $w" unless $w2r{$w}->{$r};
        # }

        my $op = $thisop->{op};
        my $opbackend = $op->{backend};

        my $update = $backend->can('update_'.$opbackend);
        $update->($thisop) if $update;

        my @wregin;
        my @wregout;

        # skip flush of vm regs that will be clobbered by the called function
        if ( $op->{op_call} && !defined $cond ) {
            my $l = $thisop->{target};
            if ( $l->{func} ) {
                my $m = ( $l->{output} | $l->{clobber} ) & ~$l->{input};
                for (my $i = 0; $i < 16; $i++) {
                    $wb{$i} = 0 if ( ( $m >> $i ) & 1 );
                }
            }
        }

        # skip flush of vm regs which are clobbered by the returning function
        if ( $op->{op_ret} && !defined $cond ) {
            if ( my $l = $thisop->{func} ) {
                for (my $i = 0; $i < 16; $i++) {
                    $wb{$i} = 0 if ( ( ( $l->{clobber} ) >> $i ) & 1 );
                }
            }
        }

        my $lbl_refs;
        $lbl_refs += $_->{used} foreach ( @{$thisop->{labels}} );

        # if the instruction may read any vm regs directly from memory
        # or if we are the target of a jump, then flush all cpu working regs
        if ( $op->{flushregs} || $lbl_refs ) {
            $regs_flush->();
        }

        # write labels
        if ( $lbl_refs ) {
            error($thisop, "label inside conditional\n") if defined $cond;

            foreach my $l (@{$thisop->{labels}}) {
                if ( $l->{export} ) {
                    print OUT "$l->{name}:\n";
                    print OUT "    .globl $l->{name}\n";
                } else {
                    print OUT "L$l->{name}:\n";
                }
            }

            $regs_reload->();
        }

        print OUT '  # '.$thisop->{name}.' ';
        print OUT "$_, " foreach (@{$thisop->{args}});
        print OUT " # L$thisop->{line}\n";
        my $inst;

        goto done if $thisop->{nop};

        # map instruction input vm registers to cpu registers
        for (my $j = 0; $j < scalar @{$thisop->{in}} ; $j++) {
          my $i = $thisop->{in}->[$j];
          if ( ($thisop->{flushin} >> $j) & 1 ) {
            # flush the input register before the instruction
            $reg_flush->($i);
            push @wregin, -1;
          } else {
            my $iw = $r2w{$i};
            if (not defined $iw) {
                # allocate new cpu register and load vm reg
                my $new = $reg_alloc->();
                $r2w{$i} = $new;
                $w2r{$new} = { $i => 1 };
                $wb{$i} = 0;
                print OUT $backend->out_load($i, $new);
            } else {
                # the instruction will take care of register write back
                $wb{$i} = 0 if ( ($thisop->{wbin} >> $j) & 1 );
                # keep cpu register hot
                $reg_use->($iw);
            }
            push @wregin, $r2w{$i};
          }
        }

        # handle mov using copy on write unless involved in a conditional
        if ( $op->{op_mov} && !defined $cond ) {
            my $i = $thisop->{in}->[0];
            my $o = $thisop->{out}->[0];

            my $iw = $r2w{$i};
            my $ow = $r2w{$o};
            # print "  #cow $iw $ow\n";
            delete $w2r{$ow}->{$o} if ( defined $ow );
            $r2w{$o} = $iw;
            $w2r{$iw}->{$o} = 1;
            $wb{$o} = 1;
            goto done;
        }

        # if the instruction write vm regs directly in memory,
        # don't mess with output registers mapping
        if ( $op->{reloadregs} ) {
            $regs_reload->();
            goto reload;
        }

        # flush cpu registers which are clobbered by the instruction
        for (my $j = 0; (1 << $j) <= $thisop->{clobber} ; $j++) {
            if ( ( $thisop->{clobber} >> $j ) & 1 ) {
                # print OUT "  #clobber $j\n";
                $wreg_flush->($j);
                $w2r{$j} = {};
            }
        }

        # map instruction output vm registers to cpu registers
        for (my $j = 0; $j < scalar @{$thisop->{out}} ; $j++) {
          my $o = $thisop->{out}->[$j];
          if ( ($thisop->{reloadout} >> $j) & 1 ) {
            # force reload of this register after the instruction
            $reg_reload->($o);
            push @wregout, -1;
          } else {
            my $ow = $r2w{$o};
            if (not defined $ow) {
                # allocated new cpu register
                my $new = $reg_alloc->();
                $r2w{$o} = $new;
                $w2r{$new} = { $o => 1 };
            } else {
                # handle copy on write
                my @k = keys %{$w2r{$ow}};
                if (@k > 1) {
                    my $new = $reg_alloc->();
                    $r2w{$o} = $new;
                    $w2r{$new} = { $o => 1 };
                    delete $w2r{$ow}->{$o};
                }
            }
            $wb{$o} = 1;
            push @wregout, $r2w{$o};
          }
        }

      reload:
        # get cpu instruction string
        $inst = $backend->can('out_'.$opbackend)->( $thisop, @wregout, @wregin );

      done:

        if ( $op->{op_cond} ) {

            if ( $op->{reloadregs} ) {
                # emit conditional now if reload has been forced
                print OUT $inst;
                $cond = "";
            } else {
                # keep conditional code for the next instruction so
                # that register mapping changes are never skipped
                $cond = $inst;
            }
        } else {
            # emit conditional + instruction
            print OUT $cond.$inst;

            if ( defined $cond ) {
                if ( $cond eq "" ) {
                    # flush registers again if the conditional
                    # instruction is able to modify registers
                    $regs_flush->();
                    $regs_reload->();
                }
                print OUT "1:\n";
            }
            $cond = undef;
        }

    }

    print OUT $backend->out_eof();

    close( OUT );
}

sub check_regs
{
     use constant REGVAL_PACK =>  15;
     use constant REGVAL_UNDEF => 16;
     use constant REGVAL_VALUE => 32;

     my @entries;
     my %entries_done;

     my $exc; $exc = sub {
         my ( $pc, $regs, $func, $last ) = @_;

         my $wr_mask = 0;
         my $rd_mask = 0;

         while ( 1 ) {
             my $thisop = $srcaddr{$pc};

             # print STDERR $pc, " ", $op->{backend}, " ", @$regs, "\n";

             if ( !$thisop ) {
                 warning($last, "jump out of bytecode to pc=$pc\n");
                 last;
             }

             my $op = $thisop->{op};

             if ( $thisop->{func} != $func ) {
                 warning($last, "execution across function boundary\n");
                 $func = $thisop->{func};
                 if ( $func ) {
                     push @entries, $func unless ( $entries_done{$func}++ );
                     last;
                 }
             }

             if ( $op->{op_data} ) {
                 warning($thisop, "execution of data\n");
                 last;
             }

             # skip if the instruction has already been checked
             # with a similar registers state
             my $update;
             for (my $i = 0; $i < 16; $i++) {
                 $update |= ($regs->[$i] & ~$thisop->{regs}->[$i]);
             }
             last if ( !$update );

             my @in = @{$thisop->{in}};
             my @out = @{$thisop->{out}};
             my @clobber;

             my $l = $thisop->{target};
             my $tailcall = $func && $op->{op_jmp} && $l->{func};

             # check function call
             if ( $op->{op_call} || $tailcall ) {

                 $regs->[$thisop->{lr}] = REGVAL_VALUE;

                 # check how registers are affected by the function call
                 if ( $l->{func} ) {
                     for (my $i = 0; $i < 16; $i++) {
                         warning($thisop, "possibly undefined value in register %$i used as parameter to function `$l->{name}'\n")
                             if ( ( $l->{input} >> $i ) & 1) && ( $regs->[$i] & REGVAL_UNDEF );

                         $regs->[$i] = REGVAL_VALUE if ( ( $l->{output} >> $i ) & 1);
                         push @clobber, $i if ( ( $l->{clobber} >> $i ) & 1);

                         if ( $func ) {
                             warning($thisop, "nested call to function `$l->{name}' returns a value in register %$i, not declared in function `$func->{name}'\n")
                                 if ( ( ( ~($func->{clobber} | $func->{output}) & $l->{output} ) >> $i ) & 1 );
                             warning($thisop, "nested call to function `$l->{name}' clobbers register %$i, not declared in function `$func->{name}'\n")
                                 if ( ( ( ~$func->{clobber} & $l->{clobber} ) >> $i ) & 1 );
                         }
                     }

                     if ( $l != $func ) {
                         $wr_mask |= $l->{clobber} | $l->{output};
                         $rd_mask |= $l->{input};
                     }

                     # schedule checking of the called function
                     push @entries, $l unless ( $entries_done{$l}++ );

                 } elsif ( $func ) {
                     warning($thisop, "call target is not a function, unable to track registers state\n");
                     for (my $i = 0; $i < 16; $i++) {
                         $regs->[$i] = REGVAL_UNDEF;
                     }
                 }
             }

             # check function return
             if ( ( $func && $op->{op_ret} ) || $tailcall ) {
                 for (my $i = 0; $i < 16; $i++) {
                     next unless ( $func->{output} >> $i ) & 1;
                     warning($thisop, "possibly undefined value in register %$i used as return value of function `$func->{name}' \n")
                         if ( $regs->[$i] & REGVAL_UNDEF );
                     warning($thisop, "possible use of packed data in register %$i as return value of function `$func->{name}' \n")
                         if ( $regs->[$i] & REGVAL_PACK );
                 }
                 $rd_mask |= $func->{output};
             }

             # check instruction input registers
             foreach my $in ( @in ) {
                 warning($thisop, "instruction uses the undeclared register %$in\n")
                 if ( $func && !( ( ( $func->{input} | $func->{preserve} |
                                      $func->{clobber} | $global_regmask ) >> $in ) & 1 ) );

                 warning($thisop, "use of possibly undefined value in register %$in\n")
                     if ( $regs->[$in] & REGVAL_UNDEF );

                 warning($thisop, "possible presence of packed data in register %$in, value is expected\n")
                     if ( $regs->[$in] & REGVAL_PACK );

                 $rd_mask |= 1 << $in;
             }

             for (my $i = 0; $i < $thisop->{packin_bytes}; $i += 4) {
                 my $r = ($thisop->{packin_reg} * 4 + $i) / 4;
                 my $d = $thisop->{packin_bytes} - $i;
                 $d = 4 if $d > 4;
                 if ( $regs->[$r] & REGVAL_VALUE ) {
                     warning($thisop, "possible presence of value in register %$r, packed data is expected\n")
                 } elsif ( ( $regs->[$r] & REGVAL_PACK ) < ( REGVAL_PACK >> (4 - $d) ) ) {
                     warning($thisop, "packed data size mismatch in register %$r\n");
                 }
             }

             # check instruction output registers
             foreach my $out ( @out ) {
                 $regs->[$out] = REGVAL_VALUE;
                 warning($thisop, "instruction writes to the undeclared register %$out\n")
                     if ( $func && !( ( ( $func->{output} | $func->{preserve} |
                                          $func->{clobber} | $global_regmask ) >> $out ) & 1 ) );
                 warning($thisop, "instruction writes to the constant global register %$out\n")
                     if ( ( ( $global_const_regmask ) >> $out ) & 1 );
                 $wr_mask |= 1 << $out;
             }

             for (my $i = 0; $i < $thisop->{packout_bytes}; $i += 4) {
                 my $r = ($thisop->{packout_reg} * 4 + $i) / 4;
                 my $d = $thisop->{packout_bytes} - $i;
                 $d = 4 if $d > 4;
                 $regs->[$r] = REGVAL_PACK >> (4 - $d);

                 warning($thisop, "instruction writes to the undeclared register %$r\n")
                     if ( $func && !( ( ( $func->{output} | $func->{preserve} |
                                          $func->{clobber} | $global_regmask ) >> $r ) & 1 ) );
                 $wr_mask |= 1 << $r;
             }

             foreach my $clob ( @clobber ) {
                 $regs->[$clob] = REGVAL_UNDEF;
                 $wr_mask |= 1 << $clob;
             }

             # update the set of possible register states for this instruction
             for (my $i = 0; $i < 16; $i++) {
                 $thisop->{regs}->[$i] |= $regs->[$i];
             }

             # explore branch target
             if ( $op->{op_cond} ) {
                 my ( $rd, $wr ) = $exc->( $pc + $op->{words} + 1, [ @$regs ], $func, $thisop );
                 $rd_mask |= $rd;
                 $wr_mask |= $wr;
             } elsif ( ( $op->{op_call} && !$l->{func} ) ||
                       ( $op->{op_jmp} && !$tailcall ) ) {
                 my ( $rd, $wr ) = $exc->( $thisop->{target}->{addr}, [ @$regs ], $func, $thisop );
                 $rd_mask |= $rd;
                 $wr_mask |= $wr;
             }

             # explore next instruction
             if ( $op->{op_tail} ) {
                 last;
             } else {
                 $pc += $op->{words};
                 $last = $thisop;
             }
         }

         return ( $rd_mask, $wr_mask );
     };

     # schedule analysis of exported labels
     foreach my $l ( values %labels ) {
         next unless $l->{export};
         next if $entries_done{$l}++;
         push @entries, $l
     }

     # perform analysis
     while ( my $l = pop @entries ) {

         # compute mask of initially defined registers
         my $mask;
         if ( $l->{func} ) {
             $mask = $global_regmask | $l->{input};
         } else {
             $mask = $global_regmask | $l->{entry};
         }

         my @regs;

         # setup initial register state
         for (my $i = 0; $i < 16; $i++) {
             @regs[$i] = ($mask >> $i) & 1 ? REGVAL_VALUE : REGVAL_UNDEF;
         }

         # start program exploration from this entry point
         my $func = $l->{func} ? $l : undef;
         my ( $rd_mask, $wr_mask ) = $exc->( $l->{addr}, \@regs, $func, $l );

         # emit some post exploration warnings
         if ( $func ) {
             for (my $i = 0; $i < 16; $i++) {
                 my $m = ( 1 << $i );
                 warning($func, "function `$func->{name}' register %$i is declared with incompatible roles\n")
                     if ( $func->{output} & $func->{clobber} & $m ) ||
                        ( $func->{output} & $func->{preserve} & $m ) ||
                        ( $func->{clobber} & $func->{preserve} & $m );
                 warning($func, "function `$func->{name}' register %$i is already declared global\n")
                     if ( ( $func->{output} | $func->{input} | $func->{clobber} | $func->{preserve} ) & $global_regmask & $m );
                 warning($func, "register %$i is declared as input of function `$func->{name}' but never used\n")
                     if ( $func->{input} & ~$rd_mask & $m );
                 warning($func, "register %$i is declared as output of function `$func->{name}' but never written\n")
                     if ( $func->{output} & ~($wr_mask | $func->{input}) & $m );
                 warning($func, "register %$i is declared clobbered in function `$func->{name}' but never written\n")
                     if ( $func->{clobber} & ~$wr_mask & $m );
             }
         }
     }

     # report unreachable code
     foreach my $thisop ( @src ) {
         my $op = $thisop->{op};
         if ( !$op->{op_data} && !$thisop->{regs}->[0] ) {
             warning($thisop, "instruction is unreachable\n");
         }
     }
}

check_regs();

$backend->write();
write_header() if defined $fheader;

warnings_print();

exit 0;

