#!/usr/bin/env perl
#
# This software is released into the public domain.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Alexandre Becoulet <alexandre.becoulet@free.fr>, 2014

use strict;
use List::Util qw(max min);
use POSIX qw(strtol);

my $longhelp = 
'
This script parses C header files looking for enums and build some
compact string descriptors from enums. The descriptors contain nul
terminated entry names and associated integer values in compressed
form.

A new header file is produced which contains generated descriptors
along with C macros. One macro is used to flag an enum. Other macros
can then be used to iterate over enum entries from a descriptor
string.

Sample content of in.h:

  #include "test.h"

  ENUM_DESCRIPTOR(test_e, strip:MY_, lower);

  enum test_e
  {
      MY_FOO = 1,
      MY_BAR = 42
  };

Sample content of test.c:

  #include <stdio.h>
  #include "in.h"
  #include "test.h"
  
  void enum_print(const char *desc)
  {
      ENUM_FOREACH(enum test_e, desc, {
          printf("%s %u\n", name, value);
      });
  }

  int main()
  {
      static const char d[] = ENUM_DESC_TEST_E;
      enum_print(d);
  }

Generate and compile:

  $ enum.pl -o test.h in.h
  $ cc test.c
  $ ./a.out
  foo 1
  bar 42
';

# enum descriptor string format:
#
#   enum_header_byte ( value_header_byte ( constant )? name \0 )+ \0
#
# enum_header_byte:
#
#   xxxxxxxx
#   |
#   \--- enum contains values which can be ored together
#   
# value_header_byte:
#   
#   xxx lllll
#    |    |
#    |    \---- min(name length, 31), 0: update value and skip
#    |
#    \--- 000 32bits unsigned constant follows
#         001 16bits unsigned constant follows
#         010 8bits unsigned constant follows
#         011 8bits shifted constant follows:
#             abbbcccc : a ? ~(c << (b*4)) : (c << (b*4))
#         100 8bits signed offset follows
#         101 no constant, previous + 1  (1 if first entry)
#         110 no constant, previous - 1  (-1 if first entry)
#         111 no constant, previous << 1 (0 if first entry)

my $fileout;

if (not defined @ARGV[0]) {
    die "usage: enum.pl [ -h ] [ -o output.h ] input1.h input1.h ... \n";
}

if (@ARGV[0] eq '-h') {
    die $longhelp;
}

if (@ARGV[0] eq '-o') {
    shift @ARGV;
    $fileout = shift @ARGV;
}

my $out = "
/*
 * Generated file, do not edit!
 */

#include <stdint.h>

#ifndef ENUM_DESCRIPTOR
# define ENUM_DESCRIPTOR(...)
#endif

#ifndef ENUM_FOREACH
# define ENUM_FOREACH(type, desc, ...)        		\\
do {                                          		\\
  uint8_t *_d = (void*)(desc);                		\\
  uint8_t _len, _hdr, flags = *_d++;          		\\
  type value = 0;                             		\\
                                              		\\
  (void)flags;                                 		\\
  while ((_hdr = *_d++))                      		\\
    {                                         		\\
      if (!(_hdr & 0x80))                     		\\
        value = 0;                            		\\
      switch (_hdr >> 5)                      		\\
        {                                     		\\
        case 0:                               		\\
          value = (_d[0] << 8) | _d[1];       		\\
          _d += 2;                            		\\
        case 1:                               		\\
          value = (value << 8) | *_d++;       		\\
        case 2:                               		\\
          value = (value << 8) | *_d++;       		\\
          break;                              		\\
        case 3: {                             		\\
          int8_t x = *_d++;                   		\\
          value = (int32_t)(int8_t)(x >> 7) ^ 		\\
            ((x & 0xf) << ((x & 0x70) >> 2)); 		\\
          break;                              		\\
        }                                     		\\
        case 4:                               		\\
          value += (int32_t)(int8_t)*_d++;    		\\
          break;                              		\\
        case 5:                               		\\
          value++;                            		\\
          break;                              		\\
        case 6:                               		\\
          value--;                            		\\
          break;                              		\\
        case 7:                               		\\
          value <<= 1;                        		\\
          break;                              		\\
        }                                     		\\
      if ((_len = _hdr & 0x1f) == 0)          		\\
        continue;                             		\\
      const char *name = (void*)_d;           		\\
      { __VA_ARGS__ }                         		\\
      for (_d += _len; *_d++; )               		\\
        ;                                     		\\
    }                                         		\\
} while (0)
#endif

#ifndef ENUM_FOREACH_NAME
# define ENUM_FOREACH_NAME(desc, ...)   		\\
do {                                          		\\
  uint8_t *_d = (void*)(desc);                		\\
  uint8_t _len, _hdr, flags = *_d++;          		\\
                                              		\\
  (void)flags;                                 		\\
  while ((_hdr = *_d++))                      		\\
    {                                         		\\
      _d += (0x00011124 >> ((_hdr >> 3) & 0x1c)) & 0xf; \\
      if ((_len = _hdr & 0x1f) == 0)          		\\
        continue;                             		\\
      const char *name = (void*)_d;           		\\
      { __VA_ARGS__ }                         		\\
      for (_d += _len; *_d++; )               		\\
        ;                                     		\\
    }                                         		\\
} while (0)
#endif

#ifndef ENUM_FLAGS
# define ENUM_FLAGS(desc) (*desc)
# define ENUM_FLAGS_OR 0x80
# define ENUM_FLAGS_EMPTY 0x40
#endif
";

foreach my $filein (@ARGV) {

    my %enums;
    my $state = 0;
    my $enum;
    my $val;
    my $lnum;

    open(IN, "<$filein") or die "unable to open `$filein'\n";

    foreach my $line (<IN>) {
        $lnum++;

        if ($state == 0) {
            if ($line =~ /^\s*enum\s+(\w+)\s*(\{)?\s*$/) {
                my $e = $enums{$1};
                if (defined $e) {
                    $state = defined $2 ? 2 : 1;
                    $val = 0;
                    $e->{defined}++;
                    $enum = $e;
                }
                next;
            } elsif ($line =~ /^\s*ENUM_DESCRIPTOR\s*\((\w+)\s*(?:,\s*([^)]+?))?\)\s*;\s*$/) {
                my $opts = {};
                foreach my $opt (split /[ ,]+/, $2) {
                    $opt =~ /(\w+)(?::(\w+))?/;
                    $opts->{$1} = defined $2 ? $2 : 1;
                }
                my $e = { 'name' => $1,
                          'vals' => [],
                          'opts' => $opts,
                          'mlnum' => $lnum };
                $enums{$1} = $e;
                next;
            }
        }

        if ($state == 1) {
            if ($line =~ /^\s*\{\s*$/) {
                $state = 2;
                next;
            } else {
                die "error:$lnum: enum parse error\n";
            }
        }

        if ($state == 2) {
            if ($line =~ /^\s*(\w+)\b\s*(?:=\s*(\w+))?,?\s*$/) {
                my $name = $1;
                if (defined $2) {
                    $val = $2;
                }
                if (my $s = $enum->{opts}->{strip}) {
                    $name =~ s/^$s//;
                }
                if (my $s = $enum->{opts}->{lower}) {
                    $name = lc($name);
                } elsif (my $s = $enum->{opts}->{upper}) {
                    $name = uc($name);
                } elsif (my $s = $enum->{opts}->{cap}) {
                    $name = lc($name);
                    $name =~ s/(^|_)(\w)/uc($2)/ge;
                }
                my $e = {
                    name => $name,
                    val => strtol($val, 0),
                };
                push @{$enum->{vals}}, $e;
                # print "$enum->{name} $1 $val\n";
                $val++;
            } elsif ($line =~ /^\s*\};\s*$/) {
                $state = 0;
            } elsif ($line =~ /^\s*\/\*.*\*\/\s*$/) {
            } elsif ($line =~ /^\s*\/\*/) {
                $state = 3;
            } else {
                die "error:$lnum: enum parse error\n";
            }
        }

        if ($state == 3) {
            if ($line =~ /\*\/\s*$/) {
                $state = 2;
            } elsif ($line =~ /\*\//) {
                die "error:$lnum: unable to handle comment end\n";
            }
            next;
        }

    }

    foreach my $name (keys %enums) {
        my $e = %enums{$name};
        if (!$e->{defined}) {
            die "error:$e->{mlnum}: enum `$e->{name}' never defined\n";
        }

        my $flags = 0;

        if ($e->{opts}->{or}) {
            $flags |= 0x80;
        }
        if ($e->{opts}->{empty}) {
            $flags |= 0x40;
        }

        $out .= "\n#define ENUM_DESC_".uc($name)." \\\n";
        $out .=  "\"". sprintf "\\x%02x", $flags;

        my $previous = 0;
        my $first = 1;

        foreach my $v (@{$e->{vals}}) {

            my $h = min(32, length $v->{name});
            my $cst;
            my $cstl;

            my $diff = $v->{val} - $previous;
            my $nb = (0x11111111 & $v->{val}) |
                     (0x11111111 & ($v->{val} >> 1)) |
                     (0x11111111 & ($v->{val} >> 2)) |
                     (0x11111111 & ($v->{val} >> 3));
            my $nnb = (0x11111111 & ~$v->{val}) |
                     (0x11111111 & (~$v->{val} >> 1)) |
                     (0x11111111 & (~$v->{val} >> 2)) |
                     (0x11111111 & (~$v->{val} >> 3));

            if ($v->{val} == $previous + 1) {
                $h |= 0b10100000;
            } elsif ($v->{val} == $previous - 1) {
                $h |= 0b11000000;
            } elsif ($v->{val} == $previous << 1) {
                $h |= 0b11100000;
            } elsif ($diff <= 127 && $diff >= -128) {
                    $h |= 0b10000000;
                    $cst = $diff & 0xff;
                    $cstl = 1;
            } elsif (($nb & ($nb - 1)) == 0) {
                    $h |= 0b01100000;
                    my $shift = log($v->{val} & ~($v->{val} - 1)) / log(2);
                    $cst = ($shift >> 2 << 4) | ($v->{val} >> ($shift & 0xfc));
                    $cstl = 1;
            } elsif (($nnb & ($nnb - 1)) == 0) {
                    $h |= 0b01100000;
                    my $shift = log(~$v->{val} & ~(~$v->{val} - 1)) / log(2);
                    $cst = ($shift >> 2 << 4) | (~$v->{val} >> ($shift & 0xfc)) | 0x80;
                    $cstl = 1;
            } else {
                    $cst = $v->{val};
                    if ($cst & 0xffffffffffff0000) {
                        $h |= 0b00000000;
                        $cstl = 4;
                    } elsif ($cst & 0xffffffffffffff00) {
                        $h |= 0b00100000;
                        $cstl = 2;
                    } else {
                        $h |= 0b01000000;
                        $cstl = 1;
                    }
            }

            $previous = $v->{val};
            $out .= sprintf "\\x%02x", $h;

            while ($cstl--) {
                $out .= sprintf "\\x%02x", ($cst >> ($cstl * 8)) & 0xff;
            }

            $out .= sprintf "\"\"%s\\0", $v->{name};
            $first = 0;
        }
        $out .=  "\\0\";\n";

    }

    close(IN);
}

if (defined $fileout) {
    open(OUT, ">$fileout") or die "unable to open `$fileout'\n";
} else {
    open(OUT, ">&STDOUT")
}
print OUT $out;

