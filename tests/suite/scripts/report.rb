#!/usr/bin/env ruby

LIBPATH = File.join(File.dirname(__FILE__), "lib", "ruby")
$LOAD_PATH << LIBPATH unless $LOAD_PATH.include? LIBPATH

require 'optparse'
require 'erb'
require 'digest/sha1'

require 'mtest/parser'
require 'mtest/validator'
require 'mtest/changeset'

options = {}
parser  = OptionParser.new do |opts|
    opts.banner = "Usage: report.rb [options]"

    opts.separator ""
    opts.separator "Options:"

    opts.on("-r", "--results path", String, "results path") do |p|
        options[:results] = File.absolute_path p if File.exists? p
    end

    opts.on("-d", "--output-dir path", String, "output directory path") do |p|
        options[:output] = File.absolute_path p if File.exists? p
    end

    opts.separator ""
end

begin
    parser.parse!(ARGV)
rescue OptionParser::ParseError
    $stderr.puts($!)
    $stderr.puts
    $stderr.puts(parser.help)
    exit 1
end

if options[:results].nil?
    $stderr.puts("error: missing results path")
    exit(1)
end

options[:output] ||= Dir.getwd

runs = []
Dir.open(options[:results]).each do |e|
    dpath = File.join(options[:results], e)

    if /^run-(?<token>.*)$/ =~ e
        mkpath = File.join(dpath, "tests-#{token}.mk")
        next unless File.exists? mkpath

        mparser = MTest::MakefileParser.new
        tests = mparser.parse!(mkpath).sort { |a,b| a.name <=> b.name }

        tvalid = MTest::Validator.new(tests)
        vtests = tvalid.validate(dpath)

        changesets = {}
        changesets_path = File.join(dpath, "changesets")
        if File.exists? changesets_path
            cparser = MTest::ChangesetParser.new
            changesets = cparser.parse!(changesets_path)
        end

        runs << MTest::Run.new(token, vtests, changesets)
    end
end

class Graph
    def render(options={})
        @runs = options[:runs] || []
        @path = options[:path] || "output.png"

        outfile = "/tmp/plot-#{Digest::SHA1.hexdigest rand.to_s}.dat"
        out     = File.open(outfile, "w")
        out.write("#Date;Success;Failed\n")

        ymax = 0
        @runs.each do |r|
            buf = ""
            buf += "#{r.when.strftime("%d-%m-%Y %H:%M")};"
            buf += "#{r.count_success};"
            buf += "#{r.count_failure}\n"
            out.write(buf)

            if r.count > ymax
                ymax = r.count
            end
        end
        out.close()

        ymax += 100 - (ymax % 100)

        commands = %Q(
            set terminal png size 1024,480 font "Arial" 9
            set output "#{@path}"

            set style data histograms
            set style histogram rowstacked
            set style fill solid 1.0 border -1

            set grid

            set boxwidth 0.5

            set style line 10 lt rgb "#61f200"
            set style line 11 lt rgb "#ff7575"

            set xlabel "Test runs" offset 0,-5.5
            set xtics rotate by 60 offset -6,-5.8 out

            set yrange [0:#{ymax}]
            set ytics 20 nomirror
            set ylabel "Number of tests"

            unset key

            set datafile separator ";"
            plot '#{outfile}' using 2:xtic(1) t "Success" ls 10, '' using 3 t "Failed" ls 11
        )

        plot(commands)
    end

private
    def plot(commands)
        IO.popen("gnuplot", "w") { |io| io.puts commands }
    end
end

class Renderer

    def initialize
        @all = DATA.read
    end

    def render(hash={})
        hash.each { |name, value| self.instance_variable_set("@#{name.to_s}", value) }
        ERB.new(get_template("layout")).result(binding)
    end

    def show(tmpl)
        tmpl = get_template(tmpl.to_s)
        ERB.new(tmpl).result(binding)
    end

private

    def get_template(name)
        @all.split(/@@#{name}/, 2).last.split(/@@.*/, 2).first.strip
    end
end

runs.sort! { |a, b| b.when <=> a.when }

# Render graph
graph = Graph.new
graph.render :runs => runs.reverse, :path => File.join(options[:output], "results.png")

# Render html
renderer = Renderer.new

index_html = renderer.render :page => :index, :title => "MutekH Automated Test System", :runs => runs
File.open(File.join(options[:output], "index.html"), "w") do |f|
    f.write(index_html)
    f.close()
end

# Create directory for test run individual report (if necessary)
run_html_dir = File.join(options[:output], "runs")
Dir.mkdir run_html_dir unless File.exists? run_html_dir

runs.each do |r|
    html = renderer.render :page => :run, :title => "MutekH Automated Test System", :record => r

    run_html_file = File.join(run_html_dir, "#{r.name}.html")
    #next if File.exists? run_html_file

    File.open(run_html_file, "w") do |f|
        f.write(html)
        f.close()
    end
end

puts "#{runs.first.count_success} passed, #{runs.first.count_failure} failed"

exit (runs.empty? or runs.first.success? ? 0 : 1)

__END__
@@layout

<!DOCTYPE html>
<html>
    <head>
        <title><%= @title %></title>
        <link rel="stylesheet"
              href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.5/css/bootstrap.min.css" />

        <link rel="stylesheet"
              href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.5/css/bootstrap-theme.min.css" />
    </head>
    <body>
    <%= show @page %>
    </body>
</html>

@@index

<section id="header">
    <div class="container" style="margin-bottom: 30px;">
        <div class="row col-md-12 text-center">
            <h1 class="big">Mutekh Test Automation System</h1>
            <% if not @runs.empty? %>
            <h4>Last run on <%= @runs.first.when.strftime("%d-%m-%Y %H:%M:%S") %></h4>
            <% end %>
        </div>
    </div>
</section>

<section id="graph">

<div class="container" style="margin-bottom: 20px;">
    <div class="row text-center col-md-12">
        <h2>Summary</h2>
        <h4>Results of test runs for the MutekH repository</h4>
    </div>
    <div class="row col-md-12 text-center">
        <img src="results.png" alt="Your browser does not support SVG" />
    </div>
</div>

</section>

<section id="summary">

<div class="container">
    <div class="row">
        <div class="col-md-6">
            <div class="row text-center">
                <h4>Latest test runs (<%= @runs.length %>)</h4>
            </div>
            <table class="table">
                <tr>
                    <th class="text-right">#</th><th>Date</th><th>Name</th><th>Changeset</th>
                    <th class="text-center" style="color: green;">Passed</th>
                    <th class="text-center" style="color:red;">Failed</th>
                </tr>
                <% id = @runs.length %>
                <% for run in @runs %>
                <tr <% if not run.success? %>class="danger"<% end %>>
                    <th class="text-right"><%= id %></th>
                    <td><%= run.when.strftime("%d-%m-%Y %H:%M") %></td>
                    <td><a href="runs/<%= run.name %>.html">
                        <%= run.name.length > 30 ?"#{run.name[0,30]}..." : run.name %>
                    </a></td>
                    <td>
                        <a href="http://www.mutekh.org/hg/mutekh/rev/<%= run.changesets[:mutekh] %>">
                            <%= run.changesets[:mutekh] %>
                        </a>
                    </td>
                    <td class="text-center"
                        <% if run.count_failure == 0 %>style="color: green; font-weight: bold;"<% end %>>
                        <%= run.count_success %>
                    </td>
                    <td class="text-center"
                        <% if run.count_failure > 0 %>style="color: red; font-weight: bold;"<% end %>>
                        <%= run.count_failure %>
                    </td>
                </tr>
                <% id -= 1 %>
                <% end %>
            </table>
        </div>
        <div class="col-md-6">
            <% fail_count = @runs.empty? ? 0 : @runs.first.count_failure %>
            <div class="row text-center">
                <h4>Latest failed tests (<%= fail_count %>)</h4>
            </div>
            <% if fail_count == 0 %>
                <div class="row text-center">
                    <h4 style="color: green;">All tests passed!</h4>
                </div>
            <% else %>
            <table class="table">
                <tr><th>Name</th></tr>
                <% @runs.first.each_failure do |t| %>
                <tr>
                    <td>
                        <a href="runs/<%= @runs.first.name %>.html#<%= t.name %>">
                            <% if t.name.length > 60 %>
                                <%= t.name[0,60] %>...
                            <% else %>
                                <%= t.name %>
                            <% end %>
                        </a>
                    </td>
                </tr>
                <% end %>
            </table>
            <% end %>
        </div>
    </div>
</div>

</section>

@@run

<section id="title">
    <div class="container" style="margin-bottom: 30px;">
        <div class="row col-md-12 text-center">
            <h1>Run <%= @record.name %> (<%= @record.count %> tests)</h1>
            <a href="../">Return to summary</a>
        </div>
    </div>
</section>

<% if not @record.changesets.empty? %>
<section id="changesets">
    <div class="container" style="margin-bottom: 20px;">
        <div class="row col-md-12">
            <h4>Mercurial changesets</h4>
        </div>
        <div class="row col-md-6">
            <table class="table">
                <tr><th>Repository</th><th>Changeset value</th></tr>
                <% @record.changesets.each do |name, value| %>
                <tr>
                    <td><%= name.to_s %></td>
                    <td>
                        <a href="http://www.mutekh.org/hg/<%= name.to_s %>/rev/<%= value %>">
                            <%= value %>
                        </a>
                    </td>
                </tr>
                <% end %>
            </table>
        </div>
    </div>
</section>
<% end %>

<section id="tests">

<div class="container">
    <div class="row">
        <div class="col-md-12">
            <h2 class="text-center">Failed tests (<%= @record.count_failure %>)</h2>
            <table class="table">
                <tr>
                    <th>Name</th>
                    <th class="text-center">Status</th>
                    <th class="text-center" colspan="4">Logs</th>
                </tr>
                <% @record.each_failure do |t| %>
                <tr <% if not t.success? %>class="danger"<% end %>>
                    <td style="min-width: 80px;">
                        <a name="<%= t.name %>"></a><%= t.name %>
                    </td>
                    <% if t.success? %>
                        <td class="text-center" style="color: green;">OK</td>
                    <% else %>
                        <td class="text-center" style="color: red;">FAILED</td>
                    <% end %>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}_Configure0.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>_Configure0.log">configure</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}_Build1.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>_Build1.log">build</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}.out.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>.out.log">env log</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}_Execute2.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>_Execute2.log">execute</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                </tr>
                <% end %>
            </table>
        </div>
    </div>
    <div class="row">
        <div class="col-md-12">
            <h2 class="text-center">Passed tests (<%= @record.count_success %>)</h2>
            <table class="table">
                <tr>
                    <th>Name</th>
                    <th class="text-center">Status</th>
                    <th class="text-center" colspan="4">Logs</th>
                </tr>
                <% @record.each_success do |t| %>
                <tr <% if not t.success? %>class="danger"<% end %>>
                    <td style="min-width: 80px;">
                        <a name="<%= t.name %>"></a><%= t.name %>
                    </td>
                    <% if t.success? %>
                        <td class="text-center" style="color: green;">OK</td>
                    <% else %>
                        <td class="text-center" style="color: red;">FAILED</td>
                    <% end %>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}_Configure0.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>_Configure0.log">configure</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}_Build1.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>_Build1.log">build</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}.out.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>.out.log">env log</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                    <td style="min-width: 50px;" class="text-center">
                        <% if File.exists? File.join(ENV['HOME'], 'www', 'logs', @record.name, "TEST_#{t.name}_Execute2.log") %>
                            <a href="../logs/<%= @record.name %>/TEST_<%= t.name %>_Execute2.log">execute</a>
                        <% else %>
                            n/a
                        <% end %>
                    </td>
                </tr>
                <% end %>
            </table>
        </div>
    </div>
</div>

</section>
