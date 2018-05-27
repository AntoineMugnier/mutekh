#!/usr/bin/perl

# Copyright (C) 2009 Alexandre Becoulet
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

use strict;
use integer;

my $region_state = 0;
my $block_state = 0;
my $count = 0;
my $loc = 1;

my $fname_in = @ARGV[0];
my $fname_out = @ARGV[1];

open ( INFILE, "<".$fname_in ) or die "error: unable to open `$fname_in' input file\n";

my $content = do { local $/; <INFILE> };

close ( INFILE );

open ( OUTFILE, ">".$fname_out ) or die "error: unable to open `$fname_out' output file\n";

sub error
{
    my ( $msg ) = @_;

    close ( OUTFILE );
    unlink( $fname_out );
    die $msg;
}

sub process_fornum
{
    my ( $var, $begin, $end, $text ) = @_;
    my $res;

    for (my $i = $begin; $i < $end; $i++)
    {
	my $t = $text;
	$t =~ s/%$var%/$i/g;
	$res .= $t;
    }

    return $res;
}

sub process_for
{
    my ( $var, $list, $text ) = @_;
    my $res;

    foreach my $i (split(/[\s,\\]+/, $list))
    {
	my $t = $text;
	$i = '' if ($i eq '##'); 
	$t =~ s/%$var%/$i/g;
	$res .= $t;
    }

    return $res;
}

while ($content =~ s/\#\s*fornum \s+ (\w+) \s+ (\d+) \s+ (\d+)\s*?\n
              ((?:(?!\#\s*for)(?!\#\s*endfor)[^\n]*\n)*)
              \#\s*endfornum\s*?\n
             /&process_fornum($1,$2,$3,$4)/gesx){}

while ($content =~ s/\#\s*for \s+ (\w+) \s+ ([#\w\s\\]+)(?!\\)\n
              ((?:(?!\#\s*for)(?!\#\s*endfor)[^\n]*\n)*)
              \#\s*endfor\s*?\n
             /&process_for($1,$2,$3)/gesx){}

print OUTFILE
"
/* -*- buffer-read-only: 1 ; -*- */

/*
 * Generated file. ANY CHANGES WILL BE LOST!
 */

# 1 \"$fname_in\"
";

foreach my $line ( split ( "\n", $content ) )
{
    if ($line =~ /^\/\*\s*backslash-region-end\b/)
    {
	error( "$fname_in:$loc: error: no backslash region to terminate\n" ) if (not $region_state);
	$region_state = 0;
    }

    $line =~ s/\s*$//;
    $line =~ s/(^|[^\t]+)(\t+)/$1." " x (length($2) * 8 - (length($1) & 7))/ge;

    $block_state = 0 if ($line eq "");

    if ($region_state || $block_state)
    {
	$line =~ s/[\s\\]*$//;

	if ( $line !~ /^\s*\#\s*define\b/ ) {
	    print STDERR "$fname_in:$loc: warning: backslash region doesn't start with #define\n"
		if ( $count == 1 );
	} else {
	    print STDERR "$fname_in:$loc: warning: #define in backslash region\n"
		if ( $count > 1 );
	}

	my $tab_count = 9 - (length($line) / 8);

	if (length($line) < 72)
	{
	    $line .= "\t" x (9 - (length($line) / 8)) . "\\";
	}
	else
	{
	    $line .= " \\";
	}

	$count++;
    }

    if ($line =~ /^\/\*\s*backslash-region-begin/)
    {
	error( "$fname_in:$loc: error: unterminated backslash region\n" ) if ($region_state || $block_state);
	$region_state = 1;
	$count = 1;
    } 

    elsif ($line =~ /^\/\*\s*backslash-block/)
    {
	error( "$fname_in:$loc: error: unterminated backslash region\n" ) if ($region_state || $block_state);
	$block_state = 1;
	$count = 1;
    }

    print OUTFILE $line."\n";
    $loc++;
}

error( "$fname_in:$loc: error: unterminated backslash region at end of file\n" ) if ($region_state);

close ( OUTFILE );

exit 0;

