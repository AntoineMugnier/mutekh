
my $c = 40;

print "int calc(int n)
{
  int i;
";

print "  int ".join(", ", map( { "r$_ = n" } (0..$c) ) ).";\n";


print "  for (i = 0; i < 256; i++) {\n";

for (my $i = 0; $i <= $c; $i++) {
    print "    r$i = 1 ";

    for (my $j = 0; $j < $c; $j++) {
        print " ^ (r$j";
#        print rand() > .5 ? (rand() > .5 ? '*' : '%') : (rand() > .5 ? '+' : '/');
        print rand() > .5 ? (rand() > .5 ? '*' : '*') : (rand() > .5 ? '+' : '+');
        print int(rand() * (1 << 31)).")";
    }

    print ";\n";

}

    print "  }
";

print "  return ".join(" ^ ", map( { "r$_" } (0..$c) ) ).";\n";
#print join("\n", map( { "  printf(\"%x \", r$_);" } (0..$c) ) ).";\n";

    print "}

int main(int argc, char **argv)
{
    printf(\"%x\\n\", calc(atoi(argv[1])));
}

";
