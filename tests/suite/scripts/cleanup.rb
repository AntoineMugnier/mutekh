#!/usr/bin/env ruby

require 'fileutils'

# Clean up outputs directory
def cleanup_directory(path, keep)
    puts(">> Entering directory #{path}")

    entries = Dir.glob("#{path}/*")
    entries.sort! do |a,b|
        File.new(a).ctime <=> File.new(b).ctime
    end

    return if keep >= entries.length

    entries[0..-(keep+1)].each do |e|
        puts("  Deleting `#{e}'")
        FileUtils.remove_entry(e, true)
    end
    puts
end

puts("Cleaning up!")

KEEP = 20
cleanup_directory File.join(ENV['HOME'], "outputs"), KEEP
cleanup_directory File.join(ENV['HOME'], "www", "runs"), KEEP
cleanup_directory File.join(ENV['HOME'], "www", "logs"), KEEP

# Clean up /tmp/plot-*
puts(">> Entering directory /tmp")
Dir.glob("/tmp/plot-*.dat").each { |e| puts("  Deleting `#{e}'"); File.delete e }

puts("Done!")

