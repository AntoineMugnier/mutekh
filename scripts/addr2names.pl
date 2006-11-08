#!/usr/bin/perl

# Batch address to file/function name converter.

if (not @ARGV)
{
    die "
 This script filters its standard input to convert all addresses of
 the form f:0x12345678 in source code location. Debug informations are
 found in binary file given on command line.

 This tool has been designed to be used with a kernel image output when
 CONFIG_COMPILE_INSTRUMENT and tracing are enabled.
 \n"
}

my $binary = @ARGV[0];

if (not -f $binary)
{
    die "`$binary' file not found\n";
}

foreach my $line(<STDIN>)
{
    while ($line =~ /f:(0x[0-9A-Fa-f]+)/)
    {
	my $addr = $1;
	my $ref = `echo $addr | addr2line -f -s -e $binary`;
	$ref =~ s/\n/ /g;
	$ref = sprintf("%-50s", $ref);

	$line =~ s/f:$addr/$ref/g;
    }

    print $line;
}

