#  This file is part of MutekH.
#
#  MutekH is free software; you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License as
#  published by the Free Software Foundation; version 2.1 of the
#  License.
#
#  MutekH is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this program.  If not, see
#  <http://www.gnu.org/licenses/>.
#
#  Copyright (c) 2016, Nicolas Pouillon, <nipo@ssji.net>

package bc_custom_hd44780;

main::custom_op('wait', 1, 0x0000, \&parse_delay);
main::custom_op('data', 1, 0x1000, \&parse_reg);
main::custom_op('e',    1, 0x2000, \&parse_int);
main::custom_op('rs',   1, 0x2010, \&parse_int);
main::custom_op('next', 1, 0x3000, \&parse_regout);

sub parse_reg
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 0);
    $thisop->{in}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_regout
{
    my $thisop = shift;
    my $r = main::check_reg($thisop, 0);
    $thisop->{out}->[0] = $r;
    $thisop->{code} |= $r;
}

sub parse_delay
{
    my $thisop = shift;
    my $d = main::check_num($thisop, 0, 0, 2047);
    $thisop->{code} |= $d;
}

sub parse_int
{
    my $thisop = shift;
    my $d = main::check_num($thisop, 0, 0, 1);
    $thisop->{code} |= $d;
}

return 1;
