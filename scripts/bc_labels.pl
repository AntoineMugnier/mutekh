#!/bin/perl

#
#     This file is part of MutekH.
#     
#     MutekH is free software; you can redistribute it and/or modify it
#     under the terms of the GNU Lesser General Public License as
#     published by the Free Software Foundation; version 2.1 of the
#     License.
#     
#     MutekH is distributed in the hope that it will be useful, but
#     WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#     
#     You should have received a copy of the GNU Lesser General Public
#     License along with MutekH; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
#     02110-1301 USA.
#
#     Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014
#

my $longhelp = "
 This script computes relative and absolute branch targets in the
 mutek bytecode embedded in a C source file. The bytecode
 declarations must be of the form:
 
   static const bc_opcode_t foo[] = {
      ....
   };

   static const bc_opcode_t bar[] =
   {
      ....
   };

 Labels in the bytecode must be declared like this:
 
     BC_INSTRUCTION(.....),
   /* label:name */
     BC_INSTRUCTION(.....),
 
 Branch targets values will be inserted or adjusted when the proper
 comment is used inside bytecode instruction parameters:
 
     BC_JMP(...,  -18 /* :name */),       /* relative */
     BC_JMPL(..., -18 /* :name */),       /* relative */
     BC_LOOP(..., -18 /* :name */),       /* relative */

     BC_CALL(...,  42 /* :name */),       /* absolute - 1 */
     BC_CST16(..., 42 /* :name */),       /* absolute - 1 */

     BC_ANY(... 42 /* abs:name */ ...),   /* absolute */

 Branch targets are also updated inside macros defined like this:
 
     #define MACRO(........)  \\
       BC_INSTRUCTION(.....), \\
       /* label:name */       \\
       BC_INSTRUCTION(.....),

 When the macro is used in the middle of an other bytecode chunk, its
 computed bytecode size will be used to properly compute branch
 targets.

 Label names are local to the macro or declaration they appear in.

 Absolute branch targets can also be adjusted outside bytecode,
 in the C source code, by specifying the name of the bytecode
 declaration they appear in:

      /* change the bytecode entry point */
      bc_set_reg(ctx, 15, /* :foo:name */);

";

use strict;
use File::Copy;

my $err = 0;

if (not defined @ARGV[0])
{
    print STDERR "usages: bc_labels.pl file.c\n";
    print STDERR "        bc_labels.pl input_file.c output_file.c\n";
    print STDERR "        bc_labels.pl input_file.c stdout\n";
    print STDERR "        bc_labels.pl help\n";
    exit 2;
}

if (@ARGV[0] eq "help")
{
    print STDERR $longhelp;
    exit 2;    
}

my $in = @ARGV[0];
open(IN, "<$in") || die "unable to open input file `$in'.\n";
my @src = <IN>;
close(IN);

my $state = 0;

sub parse
{
    my ( $filter, $filter2, $reset, $end ) = @_;

    my $lnum = 0;
    my $name;
    foreach my $line (@src)
    {
	if ($state == 0 && $line =~ /^\s*static\s+const\s+bc_opcode_t\s+(\w+)\[\]\s*=\s*(\{)?\s*$/) {

            $name = $1;
	    if ($2) {
		$state = 2;
		$reset->($lnum, undef, $name);
	    } else {
		$state = 1;
	    }
	} elsif ($state == 0 && $line =~ /^\s*#define\s+(\w*).*\\/) {
	    $state = 3;
	    $reset->($lnum, $1);
        } elsif ($state == 0) {
            $filter->($line, $lnum);
	} elsif ($state == 1 && $line =~ /^\s*\{/) {
	    $state = 2;
	    $reset->($lnum, undef, $name);
	} elsif ($state == 2) {
	    if ($line =~ /^\s*\}/) {
		$state = 0;
	    } else {
		$filter2->($line, $lnum);
	    }
	} elsif ($state == 3) {
	    $line =~ /^(.*)(\\)\s*$/;
	    $filter2->($line, $lnum);
	    $state = 0 if !$2;
	}

	$lnum++;
    }

    $end->($lnum);
}

my %blocks;
my %macros;
my $block;
my $addr;

my %opcodes = (
    "BC_END" => 1,    "BC_DUMP" => 1,  "BC_ABORT" => 1,  "BC_TRACE" => 1,
    "BC_ADD8" => 1,   "BC_CST8" => 1,
    "BC_JMP" => 1,    "BC_JMPL" => 1,  "BC_LOOP" => 1,   "BC_CALL" => 3,
    "BC_LT" => 1,     "BC_LTEQ" => 1,  "BC_EQ" => 1,     "BC_NEQ" => 1,
    "BC_MOV" => 1,    "BC_ADD" => 1,   "BC_SUB" => 1,
    "BC_MUL" => 1,    "BC_CCALL" => 1, "BC_OR" => 1,      "BC_XOR" => 1,
    "BC_AND" => 1,    "BC_SHL" => 1,   "BC_SHR" => 1,     "BC_SHRA" => 1,
    "BC_TSTC" => 1,   "BC_TSTS" => 1,
    "BC_BITS" => 1,   "BC_BITC" => 1,  "BC_SHIL" => 1,   "BC_SHIR" => 1,
    "BC_LD8" => 1,    "BC_LD16" => 1,  "BC_LD32" => 1,    "BC_LD64" => 1,
    "BC_LD8I" => 1,   "BC_LD16I" => 1, "BC_LD32I" => 1,   "BC_LD64I" => 1,
    "BC_LD8E" => 2,   "BC_LD16E" => 2, "BC_LD32E" => 2,   "BC_LD64E" => 2,
    "BC_ST8" => 1,    "BC_ST16" => 1,  "BC_ST32" => 1,    "BC_ST64" => 1,
    "BC_ST8I" => 1,   "BC_ST16I" => 1, "BC_ST32I" => 1,   "BC_ST64I" => 1,
    "BC_ST8D" => 1,   "BC_ST16D" => 1, "BC_ST32D" => 1,   "BC_ST64D" => 1,
    "BC_ST8E" => 2,   "BC_ST16E" => 2, "BC_ST32E" => 2,   "BC_ST64E" => 2,
    "BC_LDPTR" => 1,  "BC_LDPTRI" => 1,"BC_STPTR" => 1,   "BC_STPTRI" => 1,
    "BC_STPTRD" => 1, "BC_CST16" => 2, "BC_CST16X" => 2,
    "BC_CST32" => 3,   "BC_CST64" => 5,
    "BC_CUSTOM" => 1,
    );

parse( sub {
       },
       sub {
           my ( $line, $lnum, $name ) = @_;

	   if ( $line =~ /^\s*\/\*\s*label:\s*(\w+)\s*\d*\s*\*\// )
	   {
	       if (defined $block->{labels}->{$1})
               {
                   print STDERR "error:".$in.":".($lnum+1).":label `$1' defined multiple times.\n";
                   $err = 1;
               }
               else
               {
                   $block->{labels}->{$1} = $addr;
               }
	   }
	   elsif ( $line =~ /\/\* \s*size:\s*(\d+)\s* \*\//x )
	   {
	       $addr += $1;
	   }
	   elsif ( $line =~ /^\s*(BC_\w+)\(.*?\)/ && $opcodes{$1})
	   {
	       $addr += $opcodes{$1};
	   }
	   elsif ( $line =~ /^\s*(\w+)\(/ )
	   {
	       if (not defined $macros{$1})
	       {
		   print STDERR $in.":".($lnum+1).":unknown instruction/macro `$1', assuming size is 1.\n";
		   $addr++;
	       } else {
		   $addr += $macros{$1}->{size};
	       }
	   }
	   elsif ( $line =~ /^\s+\\?$/ )
	   {
	       return;
	   }
	   elsif ( $line =~ /^\s*\/\*.*\*\/\s*\\?$/ || $line =~ /\s*\/\/ /)
	   {
	       return;
	   }
	   else
	   {
	       print STDERR "error:".$in.":".($lnum+1).":unable to parse bytecode source line:\n";
	       print STDERR "    $line";
	       $err = 1;
	   }

	   $block->{lines}->{$lnum} = $addr;
       }, 
       sub {
           my ( $lnum, $macro, $name ) = @_;

	   $block->{size} = $addr if ( $block );

	   $block = { labels => {}, lines => {} };
	   $blocks{$lnum} = $block;
	   $blocks{$name} = $block if defined $name;
	   $macros{$macro} = $block if defined $macro;
	   $addr = 0;
       },
       sub {
           my ( $lnum ) = @_;

	   $block->{size} = $addr if ( $block );
       }
    );

sub label_addr
{
    my ( $lbl, $lnum, $bname ) = @_;

    my $b = $block;

    if (defined $bname)
    {
        $b = $blocks{$bname};
        if (not defined $b)
        {
            print STDERR "error:".$in.":".($lnum+1).":undefined block `$bname'.\n";
            $err = 1;
            return;
        }
    }

    if (not defined $b->{labels}->{$lbl})
    {
	print STDERR "error:".$in.":".($lnum+1).":undefined label `$lbl'.\n";	
	$err = 1;
    }

    return $block->{labels}->{$lbl};
}

sub label_goto
{
    my ( $lbl, $lnum ) = @_;
    if (not defined $block->{labels}->{$lbl})
    {
	print STDERR "error:".$in.":".($lnum+1).":undefined label `$lbl'.\n";	
	$err = 1;
    }

    return $block->{labels}->{$lbl} - $block->{lines}->{$lnum};
}

sub label_call
{
    my ( $lbl, $lnum ) = @_;
    return label_addr($lbl, $lnum) - 1;
}

parse( sub {
           my ( $line, $lnum ) = @_;
	   $line =~ s/ (\s*)-?\d*\s* \/ \* \s*:(\w+):(\w+) /$1.label_addr($3,$lnum,$2).' \/* :'.$2.':'.$3/gex;
	   @src[$lnum] = $line;
       },
       sub {
           my ( $line, $lnum ) = @_;

	   if ( $line =~ /^(\s*\bBC_(?:JMPL?|JMP|LOOP)\b.*?) -?\d*\s* \/\* \s*:(\w+)(.*)$/x ) {

               my $d = label_goto($2, $lnum);

               $line = $1.$d.' /* :'.$2.$3."\n";

               if ($d > 127 || $d < -128)
               {
                   print STDERR "error:".$in.":".($lnum+1).":displacement exceed 8 bits signed value.\n";
                   $err = 1;
               }

           } else {

               unless ( $line =~ s/ (\s*\bBC_(?:CALL|CST\d\d)\b.*?)           -?\d*\s* \/\* \s*:(\w+) /$1.label_call($2,$lnum).' \/* :'.$2/gex ) {
                   $line =~ s/ (\s*)-?\d*\s* \/ \* \s*abs:(\w+) /$1.label_addr($2,$lnum).' \/* abs:'.$2/gex;
               }
           }

	   $line =~ s/^(\s*\/\*\s*label:\s*)(\w+)\s*\d*\s*(\s*\*\/)/$1.$2.' '.label_addr($2,$lnum).' '.$3/gex;

	   @src[$lnum] = $line;
       }, 
       sub {
           my ( $lnum, $macro ) = @_;
	   $block = $blocks{$lnum};
       },
       sub {
       }
    );

if (!$err) {
    my $out = defined @ARGV[1] ? @ARGV[1] : @ARGV[0];

    if ($out eq "stdout")
    {
        print @src;
    }
    else
    {
        copy($out, $out.'~') or die "unable to create $out~" if -e $out;
        open(OUT, ">$out") || die "unable to open output file `$out'.\n";
        print OUT @src;
        close(OUT);
    }
}

exit $err;
