require 'rubygems'

module MTest

class ChangesetParser

    def parse!(fname)
        cset = {}
        File.open(fname).each do |line|
            line.chomp!
            if /^(?<key>[^:]+):(?<value>.*)$/ =~ line
                cset[key.to_sym] = value
            end
        end
        cset
    end

end

end # ENd of module MTest
