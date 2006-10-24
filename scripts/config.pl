#!/usr/bin/perl

use strict;

my %config_opts;

# set description

sub cmd_desc
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{desc}}, @args);
}

# add dependency

sub cmd_depend
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{depend}}, "@args");
}

# set default value

sub cmd_default
{
    my ($location, $opts, @args) = @_;
    my $value = "@args";

    print "warning:".$location.": default value redefined for `".$$opts{name}."'\n" if ($$opts{default});

    $$opts{default} = $value;
}

# add exclusion

sub cmd_exclude
{
    my ($location, $opts, @args) = @_;

    print "warning:".$location.": exclusion list redefined for `".$$opts{name}."'\n" if ($$opts{exclude});

    $$opts{exclude} = "@args";
}

sub cmd_mandatory
{
    my ($location, $opts, @args) = @_;
}

sub cmd_require
{
    my ($location, $opts, @args) = @_;

    print "warning:".$location.": requirement list redefined for `".$$opts{name}."'\n" if ($$opts{require});

    $$opts{require} = "@args";
}

my %config_cmd =
(
 "exclude" => \&cmd_exclude,
 "default" => \&cmd_default,
 "depend" => \&cmd_depend,
 "mandatory" => \&cmd_mandatory,
 "require" => \&cmd_require,
 "desc" => \&cmd_desc,
 "description" => \&cmd_desc
);

# process source and configuration files

sub process_file
{
    my ($file) = @_;

    if (open(FILE, "< ".$file))
    {
	my $state = 0;
	my $lnum = 0;
	my $name;
	my $opts;

	foreach my $line (<FILE>)
	{
	    $lnum++;

	    next if ($line =~ /^\s*$/);

	    # catch %config blocks start and end

	    if ($line =~ /^\s*%config\s+([^\s]+)/)
	    {
		if ($1 eq "end")
		{
		    die "error:$file:$lnum: unexpected `%config end'\n" if (not $state);
		    $$opts{default} = "undefined" if (not defined $$opts{default});
		    $state = 0;
		}
		else
		{
		    die "error:$file:$lnum: unexpected `%config', previous `".
			$name."' block not terminated\n" if ($state);

		    if ($opts = $config_opts{$1})
		    {
			die "error:$file:$lnum: `".$name."' block already defined ".
			    "at `".$$opts{location}."'\n";
		    }

		    $name = $1;
		    $opts = $config_opts{$1} = {};
		    $$opts{location} = "$file:$lnum";
		    $$opts{name} = $name;
		    $$opts{depend} = [];
		    $$opts{desc} = [];
		    $state = 1;
		}

		next;
	    }

	    # process %config blocks content

	    if ($state)
	    {
		$line =~ s/^[ \t]*//g;
		my @line_l = split(/\s+/, $line);

		# get pointer on function for command
		if (my $func_ptr = $config_cmd{@line_l[0]})
		{
		    # call command function
		    $func_ptr -> ("$file:$lnum", $opts, @line_l[1..@line_l - 1]);
		}
		else
		{
		    # error if unknow function
		    die "error:$file:$lnum: unknown command `".@line_l[0]."' in %config block\n";
		}
	    }
	}

	if ($state)
	{
	    die "error:$file:$lnum: unexpected end of file\n";
	}

	close(FILE);
    }
}

sub explore
{
    my ($dir) = @_;

    foreach my $ent (<$dir/*>)
    {
	# remove leading ./ from path
	$ent =~ s/^\.\///;

	if (-d $ent)
	{
	    explore($ent);
	}
	else
	{
	    if (($ent =~ /\.c$/) || ($ent =~ /\.h$/) || ($ent =~ /\.config$/))
	    {
		process_file($ent);
	    }
	}
    }

}

sub process_config_depend
{
    my ($orig) = @_;

    if (defined $$orig{depend_result})
    {
	return $$orig{depend_result};
    }

    foreach my $dep_and (@{$$orig{depend}})
    {
	my @deps_or = split(/\s+/, $dep_and);
	my $flag = 0;

	foreach my $dep (@deps_or)
	{
	    my $opt = $config_opts{$dep};

	    if ($opt)
	    {
		if ($$opt{value} ne "undefined")
		{
		    $flag += process_config_depend($opt);
		}
	    }
	    else
	    {
		print "warning: unknown `".$dep."' dependency ignored for `".$$orig{name}.
		    "' defined at `".$$orig{location}."'\n";
	    }
	}

	# return 0 if dependency not ok
	if (not $flag)
	{
	    print "warning: `".$$orig{name}."' will be disabled due to unmet dependencies\n";
	    $$orig{depend_result} = 0;
	    return 0;
	}
    }

    $$orig{depend_result} = 1;
    return 1;
}

sub process_config_exclude
{
    my ($orig) = @_;

    foreach my $dep (split(/\s+/, $$orig{exclude}))
    {
	my $opt = $config_opts{$dep};

	if ($opt)
	{
	    if ($$opt{value} ne "undefined")
	    {
		die "error: `".$$orig{name}."' defined at `".$$orig{location}.
		    "' prevents `".$dep."' from being defined\n";
	    }
	}
	else
	{
	    print "warning: unknown `".$dep."' exclusion ignored for `".$$orig{name}.
		"' defined at `".$$orig{location}."'\n";
	}
    }
}

sub process_config_require
{
    my ($orig) = @_;

    foreach my $dep (split(/\s+/, $$orig{require}))
    {
	my $opt = $config_opts{$dep};

	if ($opt)
	{
	    if ($$opt{value} eq "undefined")
	    {
		die "error: `".$$orig{name}."' defined at `".$$orig{location}.
		    "' require `".$dep."' to be defined\n";
	    }
	}
	else
	{
	    print "warning: unknown `".$dep."' requierment ignored for `".$$orig{name}.
		"' defined at `".$$orig{location}."'\n";
	}
    }
}

sub process_config
{
    my $changed = 0;

    # set default values

    foreach my $opt (values %config_opts)
    {
	if (not defined $$opt{value})
	{
	    $$opt{value} = $$opt{default};
	    $changed = 1;
	}

	if (not @{$$opt{desc}})
	{
	    print "warning:".$$opt{location}.": missing description for `".$$opt{name}."'\n";
	}
    }

    # check exclusion

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_exclude($opt);
	}
    }

    # check dependencies

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    if (not process_config_depend($opt))
	    {
		$$opt{value} = "undefined";
		$changed = 1;
	    }
	}
    }

    # check require

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_require($opt);
	}
    }

    return $changed;
}

sub read_header
{
    my ($file) = @_;

    if (open(FILE, "<".$file))
    {
	my $lnum = 0;

	foreach my $line (<FILE>)
	{
	    $lnum++;

	    next if ($line =~ /^\s*$/);

	    if ($line =~ /^\s*\#define\s+([^\s]+)\s+([^\s]*)/)
	    {
		my $opt = $config_opts{$1};

		if (not $opt)
		{
		    print "warning:$file:$lnum: unknown configuration token `$1'\n";
		}
		else
		{
		    $$opt{"value"} = $2;
		    next;
		}
	    }

	    if ($line =~ /^\s*\#undef\s+([^\s]+)/)
	    {
		my $opt = $config_opts{$1};

		if (not $opt)
		{
		    print "warning:$file:$lnum: unknown configuration token `$1'\n";
		}
		else
		{
		    $$opt{"value"} = "undefined";
		    next;
		}
	    }
	}

	close(FILE);
    }
}

sub write_header
{
    my ($file) = @_;

    if (open(FILE, ">".$file))
    {
	foreach my $opt (values %config_opts)
	{
	    my @desc = @{$$opt{"desc"}};

	    print FILE "\n/* "."@desc"." */\n\n";

	    if ($$opt{value} eq "undefined")
	    {
		print FILE "#undef ".$$opt{name}."\n";
		next;
	    }

	    if ($$opt{value} eq "defined")
	    {
		print FILE "#define ".$$opt{name}."\n";
		next;
	    }

	    print FILE "#define ".$$opt{name}." ".$$opt{value}."\n";
	}
    }
}

sub main
{
    explore(".");
    read_header("config.h");

    if (process_config())
    {
	write_header("config.h");
    }
}

main;
exit 0;

