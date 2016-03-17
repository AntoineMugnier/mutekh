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
            t.validate :finished if File.exists? File.join(path, "TEST_#{t.name}")
        end
        @tests
    end
end

end # End of module MTest
