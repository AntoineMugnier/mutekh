require 'rubygems'

require 'mtest/test'

module MTest

class MakefileParser

    def parse!(fname)
        tset = []
        t    = nil

        File.open(fname).each do |line|
            line.chomp!

            if /^TEST_(D|N)_(?<name>[^:]+):/ =~ line
                t = Test.new(name)
            end

            t.add_action :finished if t

            if t and line.empty?
                tset << t
                t = nil
            end
        end
        tset
    end
end

end # End of module MTest

if __FILE__ == $0
    def usage
        puts("usage: #{File.basename $0} MAKEFILE")
        exit 1
    end

    if ARGV.length < 1
        usage
    end

    p = MTest::MakefileParser.new
    tset = p.parse(ARGV[0])
    puts("info: #{tset.length} tests")
end
