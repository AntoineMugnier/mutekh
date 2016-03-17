#!/usr/bin/env ruby

require 'digest/sha1'
require 'em-irc'
require 'logger'
require 'mkfifo'

module Helpers

class FifoReader < EventMachine::Connection
    def initialize(client)
        @client = client
        @buf    = ""
    end

    def notify_readable
        data = @io.readpartial(4096)
        @buf << data

        pos = data.index("\n")
        if pos
            @client.message '#domus', @buf[0..pos-1]
            @buf = @buf[pos+1..-1]
        end
    end

    def unbind(*args)
    end
end

end # End of module Helpers

class IRCLogger < EventMachine::IRC::Client

    def initialize(options={})
        super(options)

        options = {
            :name  => "test-logger.fifo",
            :where => Dir.getwd
        }.merge!(options)

        fname = File.join(options[:where], options[:name])
        @fifo = File.open(fname, "r+")

        on :connect do
            log Logger::INFO, "Connected"
            nick 'thebug'
            join '#domus'
        end
    end

    def run!
        stop = false
        while not stop do
            EM.epoll
            EM.run do
                trap("TERM") { stop = true; EM::stop }
                trap("INT") { stop = true; EM::stop }
                connect
                watched = EM.watch(@fifo, Helpers::FifoReader, self)
                watched.notify_readable = true
                log Logger::INFO, "Starting IRC logger..."
            end
            sleep 5
        end
        log Logger::INFO, "Stopping IRC logger"
        @logger.close if @logger
    end
end

if __FILE__ == $0
    pname  = ARGV[0] || "test-logger.fifo"
    logger = IRCLogger.new :host => 'irc.ssji.net', :port => 6697, :ssl => true, :name => pname
    logger.run!
end
