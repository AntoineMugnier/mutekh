require 'rubygems'
require 'set'

module MTest

class Validator
    def initialize(tests)
        @tests = tests
    end

    def validate(path)
        raise InvalidArgument, "invalid path #{path}" unless File.exists? path
        @tests.each do |t|
            [ "N", "D" ].each do |tok|
                t.validate :finished if File.exists? File.join(path, "TEST_#{tok}_#{t.name}")
            end
        end
        @tests
    end
end

end # End of module MTest
