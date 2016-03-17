require 'rubygems'
require 'date'

module MTest

class Test
    def initialize(name)
        @name    = name
        @actions = {}
    end

    attr_reader :name

    def add_action(action)
        @actions[action] = false unless @actions.has_key? action
    end

    def has_action?(action)
        @actions.has_key? action
    end

    def each(&block)
        @actions.each &block
    end

    def validate(action)
        return unless @actions.has_key? action
        @actions[action] = true
    end

    def is_validated?(action)
        @actions.has_key? action and @actions[action]
    end

    def success?
        r = true
        @actions.each do |action, ok|
            r = r && ok
        end
        r
    end

    def each_success(&block)
        @actions.each do |action, ok|
            block.call(action) if ok
        end
    end

    def each_failure(&block)
        @actions.each do |action, ok|
            block.call(action) unless ok
        end
    end

    def to_s
        "<MakefileTest name:#{@name} success:#{success?}>"
    end
end

class Run
    def initialize(name, tests, changesets={})
        @name   = name
        @tests  = tests
        @status = true

        @changesets = changesets

        @when = DateTime.strptime(name, '%Y%m%d-%H%M%S')

        @tests.each { |t| @status &&= t.success? }
    end

    attr_reader :name, :when, :tests, :changesets

    def success?
        @status
    end

    def each(&block)
        @tests.each &block
    end

    def each_success(&block)
        @tests.each do |t|
            block.call(t) if t.success?
        end
    end

    def count_success
        count = 0
        each_success { |t| count += 1 }
        count
    end

    def each_failure(&block)
        @tests.each do |t|
            block.call(t) unless t.success?
        end
    end

    def count_failure
        count = 0
        each_failure { |t| count += 1 }
        count
    end

    def count
        @tests.length
    end
end

end # End of module MTest
