#!/usr/bin/perl -w
#
# Copyright (C) 2009 Motorola, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307, USA
#

use File::Basename;

my $argc = @ARGV;

if ($argc < 2) {
	print "Usage: gcc_warn_filter <git_top_dir> <filter_file>\n";
	exit(1);
}

my $top = $ARGV[0];
my $filter_file = $ARGV[1];
my $dbg = $ARGV[2];
my $file_warn;
my $file_failed;
my $failed = 0;


print "GCC warning chk...\n";

while (<STDIN>) {
	chop;

	$file_warn = chk_pattern($_);
	if ($file_warn eq "0") {
		next;
	}

	if (!open(FILE, $filter_file)) {
		print STDERR"Can not open the file: $filter_file";
		exit(1);
	}

	# In case the filter list is empty
	$file_failed = $file_warn;
	$file_failed =~ s/\:[0-9]+//;

	while (defined ($eachline =<FILE>)) {
     		chomp $eachline;
		$file_failed = chk_filter($file_warn, $eachline, $top);

		# Already in  filter list
		if ($file_failed eq "0") {
			last;
		}
	}

	if ($file_failed ne "0") {
		print STDERR"$_ \n";

		if ( -e $file_failed) {
			DBG("touch $file_failed \n");
			system "touch $file_failed";
		} else {
			print STDERR"$file_failed not exist\n";
		}

		$failed = 1;
	}
        close FILE;
}

if ($failed ne "0") {
	print STDERR"Unexpected GCC warning found!\n";
	exit(1);
}

exit(0);

sub DBG {
	my ($dbg_msg) = @_;
	if (!$dbg || ($dbg ne "1")) {
		return;
	}

	print $dbg_msg;
}

#
# Check if the gcc messages meet below pattern
# xxxx/xxx/xxx/yyy.c:111:warning:mmmm
#
sub chk_pattern {
	my ($full_msg) = @_;

	my @array = split/\:/, $full_msg;
	my $file = $array[0];
	my $line = $array[1];
	my $warn = $array[2];
	my $msg = $array[3];

	if (!$file || !$line || !$warn) {
		return 0;
	}


	# Only handle warning for .c and .h files
	if (!($file =~ /\.c/) && !($file =~ /\.h/)) {
		return 0;
	}

	# The 2nd/3rd field should be line# and "warning"
	if (!($line =~ /[0-9]/) || !($warn =~ /warning/)) {
		return 0;
	}

	if ($msg =~ /unused variable/) {
		return 0;
	}

	if ($msg =~ /defined but not used/) {
		return 0;
	}

	$file = $file . "\:" . $line;
	return $file;
}

sub chk_filter {
	my ($file, $filter, $topdir) = @_;

	$_file = $file;
	# Get relative file path
	$_file =~ s/$topdir\///;

	if (!($filter =~ /\:[0-9]/)) {
		# none-line# based rule
		$_file =~ s/\:[0-9]+//;
	}

	if (!($filter =~ /\.c/) && !($filter =~ /\.h/)) {
		# folder based rule
	 	$_file = dirname($_file);
	}

	if ($_file eq $filter) {
		DBG("$_file filter out\n");
		return 0;
	}

	$file =~ s/\:[0-9]+//;
	return $file;
}

