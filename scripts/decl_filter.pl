# This script can be used as a filter between the C preprocessor and
# an assembler. It relies on an input containing preprocessor
# linemarkers in order to split between C declarations from header
# files and assembly file content. The C declarations are filtered out
# so that the assembler is not confused.
#
# When the --filter-gnuasm option is used, content of the global asm()
# statements present in the header files are unstringified and pasted
# in the output.
#
# When the --parse-decl option is used, C declarations are parsed.
# This way information about enum as well as structure fields offset
# and size can be used in the assembly code. The following expressions
# are replaced in the output by integer values:
#
#    _const(enum_value_name)
#    _sizeof(struct_name)
#    _sizeof(struct_name, field.field...)
#    _offsetof(struct_name, field.field...)
#    _offsetof(struct_name, field.field..., expected_field_size)
#
# All other arguments are used to invoke the compiler in order to
# parse C declarations (when --parse-decl is used).

use strict;

my $compiler;
my $gnuasm;
my $parsedecl;

foreach my $opt (@ARGV) {
    if ($opt =~ /^-o(.*)/) {
    } elsif ($opt eq '-save-temps') {

    } elsif ($opt eq '--filter-gnuasm') {
        $gnuasm++;

    } elsif ($opt eq '--parse-decl') {
        $parsedecl++;

    } else {
        $compiler .= " $opt";
    }
}

our $cdecl_code;
our @asm_code;
use IPC::Open2;

sub parse_global_gnuc_asm
{
    my ( $str, $cpploc ) = @_;

    our %cstr_escape = (
        '0' => "\0",
        't' => "\t",
        'n' => "\n",
        'r' => "\r",
        '"' => '"',
        '\\' => '\\',
        );

    our $e_quoted = qr/ (?> (?:[^"\\]|\\[0"\\rnt])* ) /xs;

    while ($str =~ /\basm \s* \( ( (?: "$e_quoted" | \s+)+ ) \) \s* ;/gxs) {
        my $s = $1;
        my $t;

        if ( $cpploc ) {
            push @asm_code, $cpploc;
            $cpploc = undef;
        }

        while ( $s =~ /\s*"($e_quoted)"/g ) {
            my $r = $1;
            $r =~ s/\\(.)/$cstr_escape{$1}/ge;
            $t .= $r;
        }
        # print STDERR "ASM $t\n";
        push @asm_code, $t;
    }

}

sub split_input
{
    my $line;
    my $file = "<STDIN>";
    my $loc;
    my $last;

    foreach my $str (<STDIN>) {

      $line++;

      if ($str =~ /^# (\d+) "(.*)"/) {

          if ( $file ne $2 ) {
              # print STDERR $last if $last =~ /\basm\b/;
              parse_global_gnuc_asm($last, $str) if ($gnuasm);
              $last = "";
          }

          # cpp location
          $line = $1 - 1;
          $file = $2;
      } else {
          $last .= $str;
      }

      $loc = "$file:$line";
      if ( $file =~ /\.[ch]$/ ) {
          $cdecl_code .= $str;
      } else {
          push @asm_code, $str;
      }

    }
}

sub cmd_pipe
{
    my ($command, $in) = @_;

    my ($pipe_out, $pipe_in);
    my $pid = open2($pipe_out, $pipe_in, $command);
    print $pipe_in $in;
    close $pipe_in;
    my @result = <$pipe_out>;
    close $pipe_out;
    waitpid($pid, 0);
    die "error status $? for command: $command\n" if $?;
    return @result;
}

my %enums;
my %consts;
my %structs;
my %types;
my $err_count;

sub stabs_error
{
    my $err = shift;
    #print STDERR "stabs parse error: $err\n";
    $err_count++;
}

our $e_name = qr/ [_a-zA-Z][\w]* /xs;
our $e_fields = qr/ [_a-zA-Z][\w\.]* /xs;

sub parse_stabs
{
    my @stabs = @_;

    our $e_type_number = qr/ (?> \-?\d+ | \(\d+,\d+\) ) /xs;

    my $str;

    foreach my $l ( @stabs ) {

        next unless ( $l =~ /^\s*\.stabs\s+"([^"\\]*)(\\\\)?" /xs );

        # handle stabs string continuation
        $str .= $1;
        next if ( $2 );

        if ( $str =~ / ($e_name) : T$e_type_number = e ([^;]*); /xs ) {
            my ( $name, $desc ) = ( $1, $2 );
            # print STDERR "enum: $name\n";

            my $dict = {};
            foreach (split(/,/, $desc)) {
                if (/ ($e_name) : (-?\d+) /xs) {
                    $dict->{$1} = $2;
                    $consts{$1} = $2;
                    # print STDERR "  $1 = $2\n";
                }
            }

            my $enum = {
                name => $name,
                dict => $dict,
            };

            $enums{$name} = $enum;

        } elsif ( $str =~ / ($e_name) : T($e_type_number) = [su](\d+) ([^"]*) /xs ) {
            my ( $name, $tnum, $sizeof, $content ) = ( $1, $2, $3, $4 );
            #print STDERR $str;
            #print STDERR "struct $name : $sizeof\n";

            my ( $t, $p );

            # type parser
            $t = sub {
                my $r = { };
                # print STDERR "$content\n";
                if ( $content =~ /^ ($e_type_number) = /xs ) {
                    $content = $';
                    #print STDERR "newtype $1 {\n";
                    $r = $t->();
                    $r->{tnum} = $1;
                    $types{$1} = $r;
                    #print STDERR "}\n";
                } elsif ( $content =~ /^ ($e_type_number) /xs ) {
                    $r->{tnum} = $1;
                    $content = $';
                    #print STDERR "type\n";
                    return $r;
                } elsif ( $content =~ /^ \* /xs ) {
                    # pointer
                    $content = $';
                    #print STDERR "pointer {\n";
                    $t->();
                    #print STDERR "}\n";
                } elsif ( $content =~ /^ [su](\d+) /xs ) {
                    # struct/union
                    $content = $';
                    #print STDERR "struct {\n";
                    my $sf = $p->();
                    my $dict = {};
                    $dict->{$_->{name}} = $_ foreach ( @$sf );
                    $r->{struct} = {
                        fields => $sf,
                        sizeof => $1,
                        dict => $dict
                    };
                    #print STDERR "}\n";
                } elsif ( $content =~ /^ a /xs ) {
                    # array
                    $content = $';
                    $t->(); # type of index
                    $t->(); # type of elements
                    #print STDERR "array\n";
                } elsif ( $content =~ /^ r /xs ) {
                    # subrange
                    $content = $';
                    $t->();
                    unless ( $content =~ /^;(-?\d+);(-?\d+);/xs ) {
                        stabs_error("subrange $content");
                    }
                    $content = $';
                    #print STDERR "array\n";
                } elsif ( $content =~ /^ x (.) $e_name : /xs ) {
                    # cross
                    $content = $';
                    #print STDERR "cross\n";
                } else {
                    stabs_error("type $content");
                }
                return $r;
            };

            # struct/union content parser
            $p = sub {
                my $r = [];
                while (1) {
                    # print STDERR "$content\n";
                    if ( $content =~ /^;/ ) {
                        $content = $';
                        last;
                    } elsif ( $content =~ /^ ($e_name) ? : | \s : /xs ) {
                        my $name = $1;
                        $content = $';

                        my $type = $t->( );

                        if ( $content =~ /^,(\d+),(\d+);/xs ) {
                            $content = $';
                            my ( $bitoff, $bitsize ) = ( $1, $2 );

                            if ( defined $name ) {
                                push @$r, {
                                    name => $name,
                                    bitsize => $bitsize,
                                    bitoff => $bitoff,
                                    tnum => $type->{tnum}
                                };
                            }

                            if ( $type->{struct} ) {
                                foreach my $sf (@{$type->{struct}->{fields}}) {
                                    $sf->{bitoff} += $bitoff;
                                    push @$r, $sf;
                                }
                            }

                            next;
                        }
                    }
                    stabs_error("field $content");
                    last;
                }
                return $r;
            };

            my $fields = $p->();

            my $dict = {};
            foreach my $sf (@$fields) {
                #printf STDERR "  $sf->{name}, %u %u\n", $sf->{bitoff} / 8, $sf->{bitsize} / 8;
                $dict->{$sf->{name}} = $sf;
            }

            my $struct = {
                name => $name,
                sizeof => $sizeof,
                fields => $fields,
                dict => $dict
            };

            $types{$tnum} = { struct => $struct };
            $structs{$name} = $struct;
        }

        $str = "";
    }
}

sub process_asm
{
    my $line;
    my $file = "<STDIN>";
    my $loc;

    my $const = sub {
        my ( $name ) = @_;
        my $c = $consts{$name};
        die "$loc: undefined constant identifier `$name'.\n" unless defined $c;
        return $c;
    };

    my $sizeof_struct = sub {
        my ( $name ) = @_;
        my $s = $structs{$name};
        die "$loc: undefined struct `$name'.\n" unless defined $s;
        return $s->{sizeof};
    };

    my $offsetof = sub {
        my ( $struct, $path, $size ) = @_;
        my $s = $structs{$struct};
        die "$loc: undefined struct `$struct'.\n" unless defined $s;
        my $bitoff = 0;
        my $f;
        foreach my $field ( split /\./, $path ) {
            $f = $s->{dict}->{$field};
            die "$loc: undefined field `$path' in struct `$struct'.\n" unless defined $f;
            die "$loc: field `$field' is not byte aligned.\n" if ($f->{bitoff} % 8) || ($f->{bitsize} % 8);
            $bitoff += $f->{bitoff};
            my $type = $types{$f->{tnum}};
            $s = $type->{struct};
        }
        my $fs = $f->{bitsize} / 8;
        die "$loc: field `$f->{name}' size doesn't match expected $size $fs.\n" if ($size && $size != $fs);
        return $bitoff / 8;
    };

    my $sizeof_field = sub {
        my ( $struct, $path ) = @_;
        my $s = $structs{$struct};
        die "$loc: undefined struct `$struct'.\n" unless defined $s;
        my $f;
        foreach my $field ( split /\./, $path ) {
            $f = $s->{dict}->{$field};
            die "$loc: undefined field `$path' in struct `$struct'.\n" unless defined $f;
            die "$loc: field `$field' is not byte aligned.\n" if ($f->{bitoff} % 8) || ($f->{bitsize} % 8);
            my $type = $types{$f->{tnum}};
            $s = $type->{struct};
        }
        return $f->{bitsize} / 8;
    };

    foreach my $str (@asm_code) {

        $line++;

        if ($str =~ /^# (\d+) "(.*)"/) {
            # cpp location
            $line = $1 - 1;
            $file = $2;

        } else {
            $loc = "$file:$line";

            # sizeof(struct foo, field)
            $str =~ s/\b _sizeof\s*\((?:struct\s*)? \s* ($e_name) \s* , \s* ($e_name) \s*\)/$sizeof_field->($1, $2)/gxse;

            # sizeof(struct foo)
            $str =~ s/\b _sizeof\s*\(\s* struct \s+ ($e_name) \s*\)/$sizeof_struct->($1)/gxse;

            # const(foo_enum_value)
            $str =~ s/\b _const\s*\(\s* ($e_name) \s*\)/$const->($1)/gxse;

            # offsetof(struct foo, field, size)
            $str =~ s/\b _offsetof\s*\((?:struct\s*)? \s* ($e_name) \s* , \s* ($e_fields) \s* , \s* (\d+) \s* \)/$offsetof->($1, $2, $3)/gxse;

            # offsetof(struct foo, field)
            $str =~ s/\b _offsetof\s*\((?:struct\s*)? \s* ($e_name) \s* , \s* ($e_fields) \s*\)/$offsetof->($1, $2, 0)/gxse;
        }
    }
}

split_input();

if ( $parsedecl ) {
    parse_stabs( grep {/^\s*\.stab\w\b/} cmd_pipe("$compiler -x c - -gstabs -S -o -", $cdecl_code) );
    process_asm();
}

print foreach (@asm_code);

