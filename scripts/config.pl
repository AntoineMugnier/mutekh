#!/usr/bin/perl

use strict;

my %config_opts;
my $err_flag = 0;

sub text80
{
    my ($msg) = @_;
    my $res;
    my $len;

    foreach my $word (split(/\s+/, $msg))
    {
	if ($len + 1 + length($word) >= 80)
	{
	    $res .= "\n".$word;
	    $len = length($word);
	}
	else
	{
	    if ($res)
	    {
		$len += length($word) + 1;
		$res .= " ".$word;
	    }
	    else
	    {
		$len += length($word);
		$res .= $word;
	    }
	}
    }

    return $res;
}

sub text_list
{
    return "  * ".join("\n  * ", @_)."\n";
}

sub error
{
    my ($msg, @list) = @_;

    print text80("error:".$msg)."\n";

    print text_list(@list) if (@list);

    $err_flag = 1;
}

sub warning
{
    my ($msg, @list) = @_;

    print text80("warning:".$msg)."\n";

    print text_list(@list) if (@list);
}

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

# add definition

sub cmd_provide
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{provide}}, "@args");
}

# set default value

sub cmd_default
{
    my ($location, $opts, @args) = @_;
    my $value = "@args";

    warning($location.": default value redefined for `".$$opts{name}." token'") if ($$opts{default});

    $$opts{default} = $value;
}

# add exclusion

sub cmd_exclude
{
    my ($location, $opts, @args) = @_;

    warning($location.": exclusion list redefined for `".$$opts{name}." token'") if ($$opts{exclude});

    $$opts{exclude} = "@args";
}

sub cmd_flags
{
    my ($location, $opts, @args) = @_;

    foreach my $flag (@args)
    {
	if (defined $$opts{$flag})
	{
	    warning($location.": unable to set flag `".$flag."' for `".$$opts{name}." token'");
	}
	else
	{
	    $$opts{$flag} = 1;
	}
    }
}

sub cmd_require
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{require}}, "@args");
}

my %config_cmd =
(
 "exclude" => \&cmd_exclude,
 "default" => \&cmd_default,
 "depend" => \&cmd_depend,
 "flags" => \&cmd_flags,
 "require" => \&cmd_require,
 "provide" => \&cmd_provide,
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
	my $blocks = 0;
	my $name;
	my $opts;

	foreach my $line (<FILE>)
	{
	    $lnum++;

#	    last if (($lnum > 50) and ($blocks == 0));

	    next if ($line =~ /^\s*$/);

	    # catch %config blocks start and end

	    if ($line =~ /^\s*%config\s+([^\s]+)/)
	    {
		if ($1 eq "end")
		{
		    if (not $state)
		    {
			error("$file:$lnum: unexpected `%config end'");
			next;
		    }
		    $$opts{default} = "undefined" if (not defined $$opts{default});
		    $state = 0;
		}
		else
		{
		    if ($state)
		    {
			error("$file:$lnum: unexpected `%config', previous `".
			      $name."' block not terminated");
			$state = 0;
		    }

		    if ($opts = $config_opts{$1})
		    {
			error("$file:$lnum: `".$name."' block already declared ".
			      "at `".$$opts{location}."'");
			next;
		    }

		    $blocks++;
		    $name = $1;
		    $opts = $config_opts{$1} = {};
		    $$opts{location} = "$file:$lnum";
		    $$opts{name} = $name;
		    $$opts{depend} = [];
		    $$opts{require} = [];
		    $$opts{desc} = [];
		    $$opts{provide} = [];
		    $state = 1;
		}

		next;
	    }

	    # process %config blocks content

	    if ($state)
	    {
		$line =~ s/^\s*//g;
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
		    error("$file:$lnum: unknown command `".@line_l[0]."' in `%config' block");
		}
	    }
	}

	if ($state)
	{
	    error("$file:$lnum: unexpected end of file, `%config end' expected");
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

	if ((-d $ent) && (! -l $ent))
	{
#	    print $ent."\n";
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
    my $res = 1;
    my ($orig) = @_;

    if (defined $$orig{depend_result})
    {
	return $$orig{depend_result};
    }

    foreach my $dep_and (@{$$orig{depend}})
    {
	my @deps_and = split(/\s+/, $dep_and);
 	my $flag = 0;

	foreach my $dep (@deps_and)
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
		warning(" undeclared `".$dep."' token ignored in dependency list for `".
			$$orig{name}."' token declared at `".$$orig{location}."'");
	    }
	}

	# return 0 if dependency not ok
	if (not $flag)
	{
	    warning(" `".$$orig{name}."' declared at `".$$orig{location}.
		    "' will be undefined due to unmet dependencies. ".
		    "At least one of the tokens below should be defined:", @deps_and) if (not $$orig{nowarn});
	    $res = 0;
	}
    }

    $$orig{depend_result} = $res;
    return $res;
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
		error(" `".$$orig{name}."' token declared at `".$$orig{location}.
		      "' is defined and prevents `".$dep."' from being defined");
	    }
	}
	else
	{
	    warning(" undeclared `".$dep."' token ignored in exclusion list for `".
		$$orig{name}."' declared at `".$$orig{location}."'");
	}
    }
}

sub process_config_provide
{
    my ($orig) = @_;

    foreach my $rule (@{$$orig{provide}})
    {
	$rule =~ /^([^\s=]+)=?([^\s]*)$/;
	my $dep = $1;
	my $val = $2;
	my $opt = $config_opts{$dep};

	if ($opt)
	{
	    if ($$opt{value} eq "undefined")
	    {
		if ($val)
		{
		    $$opt{value} = $val;
		}
		else
		{
		    $$opt{value} = "defined";	    
		}
		process_config_provide($opt);
	    }
	    else
	    {
		warning(" `".$$orig{name}."' token declared at `".$$orig{location}.
			"' provide already defined token `".$dep."'") if (not $$orig{nowarn});
	    }
	}
	else
	{
	    warning(" undeclared `".$dep."' token ignored in provide list for `".
		$$orig{name}."' declared at `".$$orig{location}."'");
	}
    }
}

sub process_config_require
{
    my ($orig) = @_;

    foreach my $dep_and (@{$$orig{require}})
    {
	my @deps_and = split(/\s+/, $dep_and);
 	my $flag = 0;

	foreach my $rule (@deps_and)
	{
	    $rule =~ /^([^\s=]+)=?([^\s]*)$/;
	    my $dep = $1;
	    my $val = $2;

	    my $opt = $config_opts{$dep};

	    if ($opt)
	    {
		if ((not $val and ($$opt{value} ne "undefined")) or
		    ($val and $$opt{value} eq $val))
		{
		    $flag = 1;
		}
	    }
	    else
	    {
		warning(" undeclared `".$dep."' token ignored in requirement list for `".
		    $$orig{name}."' declared at `".$$orig{location}."'");
	    }
	}

	if (not $flag)
	{
	    error(" `".$$orig{name}."' declared at `".$$orig{location}.
		  "' is defined but has unmet requirements. ".
		  "At least one of the tokens below must be defined:", @deps_and);
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
	    warning($$opt{location}.": missing description for `".$$opt{name}."' token");
	}
    }

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_provide($opt);
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

    # check require and mandatory

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_require($opt);
	}

	if ($$opt{mandatory} and ($$opt{value} eq "undefined"))
	{
	    error(" `".$$opt{name}."' token must be defined");
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
		     warning("$file:$lnum: unknown configuration token `$1'");
		}
		else
		{
		    if ($2)
		    {
			$$opt{"value"} = $2;
		    }
		    else
		    {
			$$opt{"value"} = "defined";
		    }
		    next;
		}
	    }

	    if ($line =~ /^\s*\#undef\s+([^\s]+)/)
	    {
		my $opt = $config_opts{$1};

		if (not $opt)
		{
		    warning("$file:$lnum: unknown configuration token `$1'");
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
	foreach my $optname (sort keys %config_opts)
	{
	    my $opt = $config_opts{$optname};
	    my @desc = @{$$opt{"desc"}};

	    next if $$opt{noexport} or $$opt{noheader};

	    print FILE ("\n".
			text80("/* "."@desc"." */").
			"\n");

	    if ($$opt{value} eq "undefined")
	    {
		print FILE "#undef  ".$$opt{name}."\n";
		next;
	    }

	    if ($$opt{value} eq "defined")
	    {
		print FILE "#define ".$$opt{name}."\n";
		next;
	    }

	    print FILE "#define ".$$opt{name}." ".$$opt{value}."\n";
	}

	close(FILE);
    }
    else
    {
	error(" unable to open `$file' to write configuration");
    }
}

sub write_makefile
{
    my ($file) = @_;

    if (open(FILE, ">".$file))
    {
	foreach my $opt (values %config_opts)
	{
	    next if $$opt{noexport} or $$opt{nomakefile};

	    print FILE $$opt{name}."=".$$opt{value}."\n";
	}

	close(FILE);
    }
    else
    {
	error(" unable to open `$file' to write configuration");
    }
}

sub main
{
    my $changed;

    explore(".");
    exit 1 if $err_flag;

    read_header("config.h");
    exit 1 if $err_flag;

    $changed = process_config();
    exit 1 if $err_flag;

    write_header("config.h") if ($changed or not -f "config.h");
    write_makefile("config.mk") if ($changed or not -f "config.mk");
}

main;
exit 0;

