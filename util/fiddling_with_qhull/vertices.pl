#!/usr/bin/perl

# This script is an ad-hoc utility that converts output of qvoronoi program (or
# its input) to Matlab m-file which can be used to load the data into the Matlab
# environment
#
# Use it like this:
#	cat data | ./vertices.pl > /tmp/input_data.m
#	cat data | ../../qhull/bin/qvoronoi o | ./vertices.pl > /tmp/output_data.m

$DEBUG = 0;

$_ = <>;	# just ignore the first line
$_ = <>;	# second line represents number of vertices, facets and number 1
/(\d+)( (\d+) (\d+))?/;
my $pCount = $1;	# number of vertices (points)
my $fCount = $3;	# number of facets
my @x, @y;
print "File claims to contain $pCount points.\n" if $DEBUG;

foreach $i (0..$pCount-1) {
	$_ = <>;
	if (m/(-?\d+.\d+)\s+(-?\d+.\d+)/) {
		my $x = $1;
		my $y = $2;
		$x[$i] = $x;
		$y[$i] = $y;
		print "$i -- has it: ($x, $y)\n" if $DEBUG;
	} else {
		print STDERR "invalid data on line $i:\n$_";
		exit;
	}
}

# Output points:
print "% Points:\n\n";
print "X = [ ...\n" . join(", ", @x) . "; ...\n";
print join(", ", @y) . "];\n";

# Finish, if file contains only points definition
if (eof) {
	print "N = X;\n";
	exit
}

print "File claims to contain $fCount facets.\n" if $DEBUG;

my @region;
foreach $i (0..$fCount-1) {
	$_ = <>;
	/^(\d+)(.*)/;
	$_ = $2;
	my $n = $1;
	my $j = 0;
	my @vertices;
	while (/(-?\d+)/g) {
		if ($1 >= 0) {
			$vertices[$j] = $1;
			$j++;
		}
	}

	$region[$i] = \@vertices;
}

# Output facets:
print "\n\n% Lines (facets, there is " . ($#region+1) . " of them):\n\n";
print "L = { ";
for ( $i = 0; $i <= $#region ; $i++) {
	my @ix = @{$region[$i]};
	@ix = (@ix, $ix[0]);

	print "[ ";
	print join(",", @x[@ix]) ."; ...\n";
	print "\t" . join(",", @y[@ix]) . "], ...\n"
}
print "};\n";

