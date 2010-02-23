#!/usr/bin/perl

#
#     This file is part of MutekH.
#
#     MutekH is free software; you can redistribute it and/or modify it
#     under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.
#
#     MutekH is distributed in the hope that it will be useful, but
#     WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with MutekH; if not, write to the Free Software Foundation,
#     Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
#
#     Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
#

use strict;
use Cwd;
use File::Basename;
use Term::ANSIColor;

my @config_files;
my @output_files;
my %config_opts;
my $err_flag = 0;

my %param_h = (
	       "input" => "myconfig",
	       "build" => "default",
	       );

my %sec_types;
my %sec_types_req;
my %used_build = ( "default" => 1 );

my %init_env = ( %ENV );		# initial environment

sub error
{
    my ($msg, @list) = @_;
    my $tlist = join(", ", @list) if (@list);

    print STDERR color('bold red')."error:".color('clear red')."$msg$tlist\n".color('reset');
    $err_flag = 1;
}

sub warning
{
    my ($msg, @list) = @_;
    my $tlist = join(", ", @list) if (@list);

    print STDERR color('bold yellow')."warning:".color('reset')."$msg$tlist\n";
}

sub check_rule
{
    my ($orig, $rule) = @_;

    $rule =~ /^([^\s=<>]+)([=<>]*)([^\s]*)$/;
    my $dep = $1;
    my $op = $2;
    my $val = $3;

    my $opt = $config_opts{$dep};

    if (not $opt)
    {
	warning($$orig{location}.": `".$$orig{name}."' refers to undeclared token `".$dep."', ignored.");
	return 0;
    }

    if (not $op)
    {
	return 1 if ($$opt{value} ne "undefined");
    }
    elsif ($op eq "=")
    {
	return 1 if ($$opt{value} eq $val);
    }
    elsif  ($op eq ">")
    {
	return 1 if ($$opt{value} > $val);
    }
    elsif  ($op eq "<")
    {
	return 1 if ($$opt{value} < $val);
    }
    else
    {
	error($$orig{location}.": bad operator in assertion for `".$$opt{name}."' token");
    }

    return 0;
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

# add weak dependency

sub cmd_parent
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{parent}}, @args);
}

# add definition

sub cmd_provide
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{provide}}, @args);
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

    push(@{$$opts{exclude}}, @args);
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

sub cmd_suggest
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{suggest}}, "@args");
}

sub cmd_single
{
    my ($location, $opts, @args) = @_;

    push(@{$$opts{single}}, "@args");
}

sub cmd_fallback
{
    my ($location, $opts, $arg) = @_;

    if (defined $$opts{fallback})
    {
	error($location.": fallback token already declared for `".$$opts{name}."' token");
    }
    else
    {
	$$opts{fallback} = $arg;
    }
}

my %config_cmd =
(
 "exclude" => \&cmd_exclude,
 "default" => \&cmd_default,
 "depend" => \&cmd_depend,
 "parent" => \&cmd_parent,
 "flags" => \&cmd_flags,
 "require" => \&cmd_require,
 "single" => \&cmd_single,
 "suggest" => \&cmd_suggest,
 "fallback" => \&cmd_fallback,
 "provide" => \&cmd_provide,
 "desc" => \&cmd_desc,
 "description" => \&cmd_desc
);

##
## parses source and configuration files and process %config blocks
##

my %processed;

sub process_file
{
    my ($file) = @_;

    # skip already processed files
    $file = Cwd::realpath($file);

    return if ($processed{$file});
    $processed{$file} = 1;

    # process file

    if (open(FILE, "< ".$file))
    {
	my $state = 0;
	my $lnum = 0;
	my $blocks = 0;
	my $name;
	my $opts;
	my $module;

	foreach my $line (<FILE>)
	{
	    $lnum++;

	    next if ($line =~ /^[ \t]*(\#.*)?$/);

	    # catch %config blocks start and end

	    if ($line =~ /^\s*%module\s+(.+?)\s+$/)
	    {
		$module = $1;
		next;
	    }

	    if ($line =~ /^\s*%config\s+(\S+)/)
	    {
		if ($1 eq "end")
		{
		    if (not $state)
		    {
			error("$file:$lnum: unexpected `%config end'");
			next;
		    }

		    # set token default attribute to `undefined' if no default set

		    if (not defined $$opts{default})
		    {
			$$opts{default} = "undefined";
		    }

		    $state = 0;
		}
		else
		{
		    if ($state)
		    {
			error("$file:$lnum: unexpected `%config', previous `".
			      $1."' block not terminated");
			$state = 0;
		    }

		    if ($opts = $config_opts{$1})
		    {
			error("$file:$lnum: `".$1."' block already declared ".
			      "at `".$$opts{location}."'");
			next;
		    }

		    $blocks++;
		    $name = $1;
		    $opts = $config_opts{$1} = {};
		    $$opts{location} = "$file:$lnum";
		    $$opts{name} = $name;
		    $$opts{depend} = [];
		    $$opts{parent} = [];
		    $$opts{require} = [];
		    $$opts{suggest} = [];
		    $$opts{single} = [];
		    $$opts{desc} = [];
		    $$opts{provide} = [];
		    $$opts{provided_by} = [];
		    $$opts{exclude} = [];
		    $$opts{provided_count} = 0;
		    $$opts{module} = $module;
		    $state = 1;

		}

		next;
	    }

	    # process %config blocks content

	    if ($state)
	    {
		$line =~ s/^\s*//g;
		$line =~ s/\$\((\w+)\)/$init_env{$1}/ge;

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


##
## explores all subdirectories to find configuration token declarations
##

sub explore
{
    my ($dir) = @_;

    foreach my $ent (<$dir/*>)
    {
	# remove leading ./ from path
	$ent =~ s/^\.\///;

	if ((-d $ent) && (! -l $ent))
	{
	    explore($ent);
	}
	else
	{
	    if ($ent =~ /\.config$/)
	    {
		my $rp = Cwd::realpath($ent);
		push @config_files, $rp;
		$init_env{CONFIGPATH} = dirname($rp);
		process_file($ent);
	    }
	}
    }
}

##
## recursively checks all dependency of a defined token
##

my %depend_cache;

sub process_config_depend
{
    my $res = 1;
    my ( $orig ) = @_;

    if (defined $depend_cache{$$orig{name}."depend"})
    {
	return $depend_cache{$$orig{name}."depend"};
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
		warning($$orig{location}.": `".$$orig{name}."' depends on undeclared token `".$dep."', ignored.");
	    }
	}

	# return 0 if dependency not ok
	if (not $flag)
	{
		warning($$orig{vlocation}.": `".$$orig{name}."' token will be undefined ".
			"due to unmet dependencies; dependencies list is: ",
			@deps_and);

		if (my $fb = $$orig{fallback})
		{
		    $fb =~ /^([^\s=]+)=?([^\s]*)$/;
		    my $dep = $1;
		    my $val = $2 ? $2 : "defined";
		    my $opt = $config_opts{$dep};

		    if ($opt)
		    {
			if ($$opt{value} eq "undefined")
			{
			    warning("using `".$fb."' as fall-back definition for `".$$orig{name}."'.");
			    $$opt{value} = $val;
			    process_config_provide($opt);
			}
			else
			{
			    warning("`".$dep."' fall-back token for `".$$orig{name}."' has already been defined, good.");
			}
		    }
		}

	    $res = 0;
	}
    }

    $depend_cache{$$orig{name}."depend"} = $res;

    return $res;
}

##
## at least one parent must be defined
##

sub process_config_parent
{
    my $res = 1;
    my ($orig) = @_;

    if (defined $depend_cache{$$orig{name}."parent"})
    {
	return $depend_cache{$$orig{name}."parent"};
    }

    foreach my $dep (@{$$orig{parent}})
    {
	my $opt = $config_opts{$dep};
	$res = 0;

	if ($opt)
	{
	    if ($$opt{value} ne "undefined")
	    {
		$res = process_config_parent($opt);
	    }
	}
	else
	{
	    warning($$orig{location}.": `".$$orig{name}."' has undeclared parent token `".$dep."', ignored.");
	}

	last if $res;
    }

    $depend_cache{$$orig{name}."parent"} = $res;

    return $res;
}

##
## checks exclusion list of a defined token
##

sub process_config_exclude
{
    my ($orig) = @_;

    foreach my $dep (@{$$orig{exclude}})
    {
	my $opt = $config_opts{$dep};

	if ($opt)
	{
	    if ($$opt{value} ne "undefined")
	    {
		error($$orig{vlocation}.": `".$$orig{name}."' and ".
		      " `".$dep."' (".$$opt{vlocation}.") can not be defined at the same time");
	    }
	}
	else
	{
	    warning($$orig{location}.": `".$$orig{name}."' excludes undeclared token `".$dep."', ignored.");
	}
    }
}

##
## checks requirement list of a defined token
##

sub process_config_require
{
    my ($orig) = @_;

    foreach my $dep_and (@{$$orig{require}})
    {
	my @deps_and = split(/\s+/, $dep_and);
 	my $flag = 0;

	foreach my $rule (@deps_and)
	{
	    $flag = 1 if (check_rule($orig, $rule));
	}

	if (not $flag)
	{
	    error($$orig{vlocation}.": `".$$orig{name}."' token is defined ".
		  "but has unmet requirements; requirements list is: ",
		  @deps_and);
	}
    }
}

##
## checks suggestion list of a defined token
##

sub process_config_suggest
{
    my ($orig) = @_;

    foreach my $dep_and (@{$$orig{suggest}})
    {
	my @deps_and = split(/\s+/, $dep_and);
 	my $flag = 0;

	foreach my $rule (@deps_and)
	{
	    $flag = 1 if (check_rule($orig, $rule));
	}

	if (not $flag)
	{
	    warning($$orig{vlocation}.": `".$$orig{name}."' token is defined ".
		    "and suggests this configuration: ",
		    @deps_and);
	}
    }
}

##
## process single definition constraints and add to exclude lists
##

sub process_config_single
{
    my ($orig) = @_;

    foreach my $dep_and (@{$$orig{single}})
    {
	my @deps_and = split(/\s+/, $dep_and);

	foreach my $r1 (@deps_and)
	{
	    foreach my $r2 (@deps_and)
	    {
		next if ($r1 eq $r2);

		my $opt = $config_opts{$r1};
		push(@{$$opt{exclude}}, $r2);
	    }
	}
    }
}

##
## defines all tokens provided by a defined token
##

sub process_config_provide
{
    my ($orig) = @_;

    return if ($$orig{provide_done});
    $$orig{provide_done} = 1;

    foreach my $rule (@{$$orig{provide}})
    {
	$rule =~ /^([^\s=]+)=?([^\s]*)$/;
	my $dep = $1;
	my $val = $2 ? $2 : "defined";
	my $opt = $config_opts{$dep};

	my $concat = 0;

	if ($val =~ /^\+(.*)/)
	{
	    $val = $1;
	    $concat = 1;
	}

	if ($val eq "undefined")
	{
	    error($$orig{location}.": `".$$orig{name}."' token provides `".
		  $dep."' with `undefined' value");
	    next;
	}

	if ($opt)
	{
	    if ($$opt{value} eq "undefined")
	    {		
		$$opt{value} = $val;
		$$opt{vlocation} = $$orig{location};
	    }
	    else
	    {
		# if value start with a '+' it must be concatened
		if ($concat)
		{
		    $$opt{value} .= " " . $val;
		}
		elsif ($val ne $$opt{value})
		{
		    error($$orig{location}.": `".$$orig{name}."' token provides ".
			  "already defined token `".$dep."' with a different value;".
			  " previous value definition was at ".$$opt{vlocation});
		}

		# prevent previously defined tokens from being undefined by unprovide
		if (not $$opt{provided_count})
		{
		    $$opt{provided_count}++;
		}

	    }
	    $$opt{provided_count}++;

	    process_config_provide($opt);
	}
	else
	{
	    warning($$orig{location}.": `".$$orig{name}."' provides undeclared token `".$dep."', ignored.");
	}
    }
}

##
## cancels/removes provided tokens
##

sub process_config_unprovide
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
	    $$opt{provided_count}--;

	    # if value start with a '+' it must be deleted from string
	    if ($val =~ /^\+(.*)/)
	    {
		$$opt{value} =~ s/\b$1\b//;
	    }
	    elsif ($$opt{provided_count} < 1)
	    {
		$$opt{value} = "undefined";
	    }
	}
    }
}

##
## set provided_by list for all tokens
##

sub set_provided_by
{
    foreach my $opt (values %config_opts)
    {
	foreach my $rule (@{$$opt{provide}})
	{
	    $rule =~ /^([^\s=]+)=?([^\s]*)$/;
	    my $dep = $1;

	    if (my $opt2 = $config_opts{$dep})
	    {
		push(@{$$opt2{provided_by}}, $$opt{name});
	    }
	}
    }
}

##
## replace token names in values by value
##

sub preprocess_values
{
    foreach my $opt (values %config_opts)
    {
	my $value = $$opt{value};
	my $token;

	# replace configuration token names by values

	while  ($token = $config_opts{$value})
	{
	    $value = $$token{value};
	}

	# normalize numerical value

# 	if ($value =~ /^0x[0-9a-fA-F]+$/)
# 	{
# 	    $value = sprintf "0x%x", hex $value;
# 	} elsif ($value =~ /^0[0-7]+$/) {
# 	    $value = sprintf "0x%x", oct $value;
# 	} elsif ($value =~ /^\d+$/) {
# 	    $value = sprintf "0x%x", $value;
# 	}

	$$opt{value} = $value;
    }
}

##
## sets token values
##

sub set_config
{
    # set default values

    foreach my $opt (values %config_opts)
    {
	if (not defined $$opt{value})
	{
	    $$opt{value} = $$opt{default};
	    $$opt{vlocation} = $$opt{location};
	}
    }

    # provides all tokens

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_provide($opt);
	}
    }

    foreach my $opt (values %config_opts)
    {
	process_config_single($opt);
    }
}

##
## checks all configuration constraints
##

sub check_config
{
    my $changed = 0;

    foreach my $opt (values %config_opts)
    {
	my $name = $$opt{name};

	if (not @{$$opt{desc}})
	{
	    warning($$opt{location}.": missing description for `".$$opt{name}."' token");
	}

	if (uc($name) ne $name)
	{
	    warning($$opt{location}.": `".$name."' is not strictly upper case");
	}

	if (not (lc($name) =~ /^config_/))
	{
	    warning($$opt{location}.": `".$name."' has no `CONFIG_' prefix");
	}

	if (my $fb = $$opt{fallback})
	{
	    $fb =~ /^([^\s=]+)/;

	    if (not $config_opts{$1})
	    {
		warning($$opt{location}.": `".$$opt{name}."' fall-back to undeclared token `".$1."'");
	    }

	    if ($config_opts{$1} eq $opt)
	    {
		error($$opt{location}.": `".$$opt{name}."' fall-back to self");
	    }
	}
    }

    # checks and adjusts dependencies

    do
    {
	$changed = 0;
	%depend_cache = ();

	foreach my $opt (values %config_opts)
	{
	    if ($$opt{value} ne "undefined")
	    {
		if (not process_config_parent($opt) or
		    not process_config_depend($opt)
		    )
		{
		    $$opt{value} = "undefined";
		    process_config_unprovide($opt);
		    $changed = 1;
		    last;
		}
	    }
	}
    }
    until (not $changed);

    # check update exclusion lists

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_exclude($opt);
	}
    }

    # checks require and mandatory

    foreach my $opt (values %config_opts)
    {
	if ($$opt{value} ne "undefined")
	{
	    process_config_require($opt);
	    process_config_suggest($opt);
	}

	if ($$opt{mandatory} and ($$opt{value} eq "undefined"))
	{
	    error($$opt{vlocation}.": `".$$opt{name}."' token can not be undefined");
	}
    }
}

sub read_myconfig
{
    my ( $file, $section ) = @_;

    my $cd = dirname(Cwd::realpath($file));

    push @config_files, Cwd::realpath($file);

    $init_env{CONFIGSECTION} = 'common';
    $init_env{CONFIGPATH} = $cd;

    if (open(FILE, "<".$file))
    {
	my $lnum = 0;
	my @ignore = ( 0 );
	my @cur_sections = ( ["common"] );

	foreach my $line (<FILE>)
	{
	    $lnum++;

	    # skip empty lines and comment lines
	    next if ($line =~ /^[ \t]*(\#.*)?$/);

	    # replace env variables
	    $line =~ s/\$\((\w+)\)/$init_env{$1}/ge;

	    if ($line =~ /^\s* %(sub)?section \s+ ([*\w\d\s-]+)/x)
	    {
		my $s = $1;	# subsection ?
		my @sections = split(/\s+/, $2);
		my $i = 1;

		if (!$s || !@ignore[1]) {

		    foreach my $p (@sections) {

			my $p_ = $p;
			$p_ =~ s/\*/[\\w\\d]\+/g;

			foreach (split(/:/, $section)) {
			    if ( $_ =~ /^$p_$/ ) {
				$i = 0;
				$used_build{$_} = 1;
				$init_env{CONFIGSECTION} = $_ if ( !$s );
			    }
			}

			last if !$i;
		    }
		}

		if ( $s ) {
		    unshift @ignore, $i;
		    unshift @cur_sections, [ @sections ];
		} else {
		    @ignore = ( $i );
		    @cur_sections = [ @sections ];
		}
		next;
	    }

	    if ($line =~ /^\s* %else\b/x)
	    {
		@ignore[0] = !@ignore[0] if !@ignore[1];
		@cur_sections[0] = [];
		next;
	    }

	    if ($line =~ /^\s* %end\b/x)
	    {
		if ( scalar @ignore < 2 ) {
		    if ( scalar @ignore == 1 ) {
			$line = "%common";
		    } else {
			error( "$file:$lnum: unbalanced %end.");
		    }
		} else {
		    shift @ignore;
		    shift @cur_sections;
		}
		next;
	    }

	    if ($line =~ /^\s* %common\b/x)
	    {
		$init_env{CONFIGSECTION} = 'common';
		@cur_sections = ( ["common"] );
		@ignore = ( 0 );
		next;
	    }

	    if ($line =~ /^\s* %types \s+ (\w[\w\d]*\b\s*)+$/x)
	    {
		foreach my $t (split(/\s+/, $1)) {
		    if (!@ignore[0]) {
			error( "$file: multiple `$t' section types in use" ) if ($sec_types{$t} == 1);
			$sec_types{$t}++;
		    }

		    my $r = $sec_types_req{$t};
		    $r = [] if ( ! $r );
		    push @$r, $_ foreach (@{@cur_sections[0]});
		    $sec_types_req{$t} = $r;
		}
		next;
	    }

	    next if @ignore[0];

	    if ($line =~ /^\s* %set \s+ (\w[\w\d]*) \s+ (.*?) \s*$/x)
	    {
		$init_env{$1} = $2;
		next;
	    }

	    if ($line =~ /^\s* %error \s+ (.*)$/x)
	    {
		error("$file:$lnum: $1");
		next;
	    }

	    if ($line =~ /^\s* %die \s+ (.*)$/x)
	    {
		error("$1");
		next;
	    }

	    if ($line =~ /^\s* %warning \s+ (.*)$/x)
	    {
		warning("$1");
		next;
	    }

	    if ($line =~ /^\s* %notice \s+ (.*)$/x)
	    {
		print STDERR color('green')."notice:".color('reset')."$1\n";
		next;
	    }

	    if ($line =~ /^\s* %requiretypes \s+ (.+) $/x)
	    {
		foreach my $t (split(/\s+/, $1)) {
		    next if ($sec_types{$t});

		    my $r = $sec_types_req{$t};
		    if ( !$r ) {
			error("$file:$lnum: required `$t' type is never defined");
		    } else {
			error("no `$t' section in use, candidate sections are: ", @$r);
		    }
		}
		next;
	    }

	    if ($line =~ /^\s* %include \s+ (\S+)/x)
	    {
		my $f = $1;
		$f = "$cd/$f" unless $f =~ /^\//;
		read_myconfig( $f, $section );
		next;
	    }

	    if ($line =~ /^\s* (\w[\w\d]*) (?: \s+(\S+) )?/x)
	    {
		my $opt = $config_opts{$1};
        my $val = $2;

		if (not $opt)
		{
		     warning("$file:$lnum: undeclared configuration token `$1', ignored");
		}
		else
		{
		    if ($$opt{nodefine})
		    {
			error("$file:$lnum: `".$$opt{name}."' token can not be defined directly;".
			      " it must be provided by defining other appropriate token(s) instead.");
		    }

		    if (defined $val)
		    {
                if ($val =~ /^\+(.*)/)
                {
                    $$opt{value} .= " ".$1;
                }
                else
                {
                    $$opt{value} = $val;
                }
		    }
		    else
		    {
			$$opt{value} = "defined";
		    }

		    $$opt{vlocation} = "$file:$lnum";
		}
		next;
	    }

	    warning("$file:$lnum: bad line format, ignored");
	}

	close(FILE);
    }
    else
    {
	error("unable to open/read `".$file."' configuration input file");
    }
}

sub write_header
{
    my ($file) = @_;

	push @output_files, $file;

    if (open(FILE, ">".$file))
    {
	print FILE ("/*\n".
		    " * This file has been generated by the configuration script.\n".
		    " */\n\n");

	foreach my $opt (values %config_opts)
	{
	    next if $$opt{noexport} or $$opt{noheader};

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

sub write_doc_header
{
    my ($file) = @_;

	push @output_files, $file;

    if (open(FILE, ">".$file))
    {
	print FILE ("/** \@file \@hidden\n".
		    " * This file has been generated by the configuration script.\n".
		    " */\n\n");

	foreach my $opt (values %config_opts)
	{
	    my @desc = @{$$opt{"desc"}};

	    next if $$opt{noexport} or $$opt{noheader};

	    my $pwd = Cwd::realpath($ENV{PWD});
	    my $loc = $$opt{location};

	    $loc =~ s/^$pwd\///;

	    print FILE
"/**
    @desc 
";
	    print FILE "   \@module {".$$opt{module}."}\n" if defined $$opt{module};

	    print FILE"
    \@list
     \@item Declared in \@sourcelink $loc.
     \@item Default value is $$opt{default}.
    \@end list
";

	    sub doc_list
	    {
		my ( $listref, $title ) = @_;
		if (my @list = @{$listref})
		{
		    print FILE "\n    $title:\n    \@list\n";
		    foreach my $dep_and (@list)
		    {
			my @l;
			foreach ( split(/\s+/, $dep_and) ) {
			    s/^\w[\w\d]*/$& /;
			    push @l, "\@ref \#$_";
			}
			print FILE "     \@item ".join(" or ", @l)."\n";
		    }
		    print FILE "    \@end list\n";
		}
	    }

            doc_list($$opt{depend}, "This token depends on");
            doc_list($$opt{require}, "This token require definition of");
            doc_list($$opt{suggest}, "Defining this token suggest use of");
            doc_list($$opt{exclude}, "This token can not be defined along with");
            doc_list($$opt{provide}, "Defining this token will also provide");
            doc_list($$opt{provided_by}, "This token is provided along with");

	    if (my $fb = $$opt{fallback})
	    {
		print FILE "\n  Fallback token : \@ref \#$fb\n";
	    }

	    print FILE "*/\n#define ".$$opt{name}."\n\n";
	    next;
	}

	close(FILE);
    }
    else
    {
	error(" unable to open `$file' to write documentation");
    }
}

sub write_depmakefile
{
    my ($file) = @_;

    if (open(FILE, ">".$file))
    {
	print FILE ("##\n".
		    "## This file has been generated by the configuration script.\n".
		    "##\n\n");

	print FILE $_." \\\n" foreach (@output_files);
	print FILE ":";
	print FILE " \\\n\t".$_ foreach (@config_files);
	print FILE "\n";

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

	push @output_files, $file;

    if (open(FILE, ">".$file))
    {
	print FILE ("##\n".
		    "## This file has been generated by the configuration script.\n".
		    "##\n");

	print FILE ("\n# configuration options\n\n");
	foreach my $opt (values %config_opts)
	{
	    next if $$opt{noexport} or $$opt{nomakefile};

	    print FILE $$opt{name}."=".$$opt{value}."\n";
	}

	print FILE ("\n# configuration variables\n\n");
	foreach my $var (keys %init_env)
	{
	    next if $ENV{$var} eq $init_env{$var};

	    print FILE "BUILD_$var=".$init_env{$var}."\n";
	}

	close(FILE);
    }
    else
    {
	error(" unable to open `$file' to write configuration");
    }
}

sub write_m4
{
    my ($file) = @_;

	push @output_files, $file;

    if (open(FILE, ">".$file))
    {
	foreach my $opt (values %config_opts)
	{
	    next if $$opt{noexport};

	    print FILE "m4_define(".$$opt{name}.", `".$$opt{value}."')\n";
	}

	close(FILE);
    }
    else
    {
	error(" unable to open `$file' to write configuration");
    }
}

sub write_py
{
    my ($file) = @_;

	push @output_files, $file;

    if (open(FILE, ">".$file))
    {
	foreach my $opt (values %config_opts)
	{
	    next if $$opt{noexport};

	    print FILE $$opt{name}." = '".$$opt{value}."'\n";
	}

	close(FILE);
    }
    else
    {
	error(" unable to open `$file' to write configuration");
    }
}

sub tokens_list
{
    printf("\n    %-40s %s \n", "Configuration token name", "Declare location");
    print ("="x79, "\n\n");

    foreach my $name (sort keys %config_opts)
    {
	my $opt = $config_opts{$name};
	my $attr;

	if (not ($param_h{list} eq "all"))
	{
	    next if ($$opt{nodefine});

	    # hide entry if all parents are disabled
	    if (my @list = @{$$opt{parent}})
	    {
		my	$flag = 0;

		foreach my $n (@list)
		{
		    my $o = $config_opts{$n};
		
		    $flag = 1 if ($$o{value} ne "undefined");
		}

		next if (not $flag);
	    }
	}

	if ($$opt{value} eq "undefined") {
	    $attr = " ";
	} elsif ($$opt{provided_count}) {
	    $attr = "p";
	} elsif ($$opt{mandatory}) {
	    $attr = "m";
	} elsif ($$opt{value} ne "defined") {
	    $attr = "v";
	} else {
	    $attr = "+";
	}

	printf(" %s  %-40s (%s)\n", $attr, $name, $$opt{location});
    }

    print("\n    (+) defined, (p) provided, (m) mandatory, (v) value.\n\n");
}

sub tokens_info
{
    my ($name) = @_;
    my $opt = $config_opts{$name};

    if (not $opt)
    {
	print "No `".$name."' token declared\n";
	return;
    }

    printf("
==================== %s ====================

", $name);

    if (my @desc = @{$$opt{"desc"}})
    {
	print text80("@desc", "  ")."\n";
    }
    else
    {
	print "(no description)\n";
    }

    print("\n  This token is mandatory and can not be undefined.\n") if $$opt{mandatory};

    print("\n  This token can not be defined directly by user.\n") if $$opt{nodefine};

    printf("
  declared at   :  %s
  assigned at   :  %s
  default value :  %s
  current value :  %s
", $$opt{location}, $$opt{vlocation},
	   $$opt{default}, $$opt{value});

    if (my $fb = $$opt{fallback})
    {
	printf("  fall-back def :  %s\n", $fb);
    }

    if (my @list = @{$$opt{depend}})
    {
	print "\n  depends on :\n\n";

	foreach my $dep_and (@list)
	{
	    print text80(join(" or ", split(/\s+/, $dep_and)),
			 "      ", "    * ")."\n";
	}
    }

    if (my @list = @{$$opt{require}})
    {
	print "\n  require :\n\n";

	foreach my $dep_and (@list)
	{
	    print text80(join(" or ", split(/\s+/, $dep_and)),
			 "      ", "    * ")."\n";
	}
    }

    if (my @list = @{$$opt{suggest}})
    {
	print "\n  suggest :\n\n";

	foreach my $dep_and (@list)
	{
	    print text80(join(" or ", split(/\s+/, $dep_and)),
			 "      ", "    * ")."\n";
	}
    }

    if (my @list = @{$$opt{exclude}})
    {
	print "\n  excludes :\n\n";

	foreach my $dep (@list)
	{
	    print text80(join(" or ", split(/\s+/, $dep)),
			 "      ", "    * ")."\n";
	}
    }

    if (my @list = @{$$opt{provide}})
    {
	print "\n  provides :\n\n";

	foreach my $dep (@list)
	{
	    print text80(join(" or ", split(/\s+/, $dep)),
			 "      ", "    * ")."\n";
	}
    }

    if (my @list = @{$$opt{provided_by}})
    {
	print "\n  may be provided by :\n\n";

	foreach my $dep (@list)
	{
	    print text80(join(" or ", split(/\s+/, $dep)),
			 "      ", "    * ")."\n";
	}
    }

    print "\n";
}

sub main
{
    foreach my $param (@ARGV)
    {
	error " bad command line parameter `$param'"
	    if (! ($param =~ /--([^=]+)=?(.*)/));
	
	my $name = $1;
	my $value = $2;

	$name =~ s/-/_/g;

	if ($value)
	{
	    $param_h{$name} = $value;
	}
	else
	{
	    $param_h{$name} = 1;
	}
    }

    if (not @ARGV or $param_h{help})
    {
	print "
Usage: config.pl [options]

	--input=file[:...]  Set build configuration file list (myconfig).
	--build=name[:...]  Set build configuration enabled section names.

	--header=file       Output configuration header in `file'.
	--docheader=file    Output header with documentation tags in `file'.
	--makefile=file     Output configuration makefile variables in `file'.
	--m4=file           Output configuration m4 macro definitions in `file'.
	--python=file       Output configuration python definitions in `file'.

	--depmakefile=file  Output config files depend makefile `file'.

	--check             Check configuration constraints without output.
	--list[=all]        Display configuration tokens list.
	--info=token        Display informations about `token'.

	--arch-cpu          Output arch-cpu couple

	--path=dir[:...]    Set list of directories to explore, default is \$PWD
";
	return;
    }

    if ($param_h{path})
    {
	explore($_) foreach (split(/:/, $param_h{path}))
    }
    else
    {
	explore($ENV{PWD});
    }

    exit 1 if $err_flag;

    if ($param_h{docheader})
    {
	write_doc_header($param_h{docheader});
	return;
    }

    read_myconfig( $_, $param_h{build} )
	foreach (split(/:/, $param_h{input}));

    delete $init_env{CONFIGSECTION};
    delete $init_env{CONFIGPATH};

    foreach (split(/:/, $param_h{build})) {
	error("build section name `$_' never considered in configuration file") if ( !$used_build{$_} );
    }

    exit 1 if $err_flag;

    set_config();
    preprocess_values();

    if ($param_h{list})
    {
	check_config();
	tokens_list();
	return;
    }

    if ($param_h{info})
    {
	check_config();
	set_provided_by();
	tokens_info($param_h{info});
	return;
    }

    if ($param_h{arch_cpu})
    {
	check_config();
	set_provided_by();
	my $arch = $config_opts{CONFIG_ARCH_NAME};
	my $cpu = $config_opts{CONFIG_CPU_NAME};
	print $$arch{value}."-".$$cpu{value}."\n";
	return;
    }

    if ($param_h{header} or $param_h{makefile} or
	$param_h{m4} or $param_h{check})
    {
	exit 1 if $err_flag;

	check_config();
	exit 1 if $err_flag;

	write_header($param_h{header}) if $param_h{header};
	write_makefile($param_h{makefile}) if $param_h{makefile};
	write_m4($param_h{m4}) if $param_h{m4};
	write_py($param_h{python}) if $param_h{python};
	write_depmakefile($param_h{depmakefile}) if $param_h{depmakefile};
	return;
    }
}

main;
exit 0;


# Local Variables:
# tab-width: 8
# basic-offset: 4
# indent-tabs-mode: t
# End:
