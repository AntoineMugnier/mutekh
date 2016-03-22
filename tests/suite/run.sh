#!/bin/bash

# Mandatory directories that must exist in $destdir
MANDATORY_DIRS="outputs www mutekh soclib"

# Selected tests from mutekh/tests/pool
SELECTED_POOL_TESTS=""

# IRC logger fifo name
IRC_LOGGER_FIFO_NAME="logger.pipe"

###############################################################################

here=$(basename `readlink -f $0`)
destdir=$PWD

# Setup environment
. $here/setup-env.sh

# Setup mtest tool
export PYTHONPATH

PYTHONPATH=$PYTHONPATH:$here/../lib/python
MTEST="python -m mt.generator"

# Print header
echo
echo "########################################################################"
echo "# Test run begins on `date "+%d-%m-%Y %H:%M:%S"`                       #"
echo "########################################################################"
echo

# Check mandatory directories
echo "INFO:: checking mandatory directories"
for d in $MANDATORY_DIRS; do
  if [ ! -d $destdir/$md ]; then
    echo "ERROR:: missing directory $destdir/$d."
    exit 1
  fi
done

# Run tests
pushd $destdir/mutekh 2>&1 >/dev/null

# Update sources
echo "INFO:: updating mutekh repository"
(hg pull -u && hg purge) || exit 1

# Dump changesets
echo "INFO:: extract changesets from mecurial repositories"
changeset_mutekh=`hg id | cut -d' ' -f1 | sed 's/\+//g'`
changeset_mutekh_last=""

# Check changes from last run
if [ -f $destdir/outputs/latest/changesets ]; then
    changeset_mutekh_last=`cat $destdir/outputs/latest/changesets | grep mutekh | cut -d':' -f2`

    if [ "$changeset_mutekh_last" = "$changeset_mutekh" ]; then
        echo "INFO:: No changes in mutekh and tests repositories"
        exit 0
    fi
fi

# Notify commits on mutekh
#if [ -n "$changeset_mutekh_last" ]; then
#    commits="$(hg log -r $changeset_mutekh_last..tip --template '{author}:{rev}')"
#    for c in $commits; do
#        author="$(echo $c | cut -d':' -f1 | cut -d' ' -f1-2)"
#        changeset="$(hg id -r `echo $c | cut -d':' -f2` | cut -d' ' -f1 | sed 's/\+//g')"
#
#        irc_log "New commit => repo:mutekh changeset:$changeset ($author)"
#        irc_log "  url: http://www.mutekh.org/hg/mutekh/rev/$changeset"
#        sync
#    done
#fi

# Prepare mutekh repository
echo "INFO:: prepare mutekh repository"
rm -f tests.mk
suffix=`date +%Y%m%d-%H%M%S`

# Generate test makefile
echo "INFO:: generate test run makefile"
enabled_pool_tests=( $SELECTED_POOL_TESTS )
if [ -f tests/pool/enabled.list ]; then
    enabled_pool_tests=( $SELECTED_POOL_TESTS $(cat tests/pool/enabled.list) )
fi

mtest_args="${enabled_pool_tests[@]/#/tests/pool/}"
$MTEST $mtest_args

[ $? -ne 0 ] && exit 1

# Create test run output directory
output_dir="$destdir/outputs/run-$suffix"
echo "INFO:: Create output directory '$output_dir'"
mkdir -p $output_dir

# Copy previous
cp -a $destdir/lastest/TESTD_* $output_dir/
rm -f $output_dir/*.out

# Create 'latest' symlink
(cd $destdir/outputs; rm -f latest; ln -s `basename $output_dir` latest)

# Compute vcpu number
echo "INFO:: Compute the number of available vcpus"
ncpus=`cat /proc/cpuinfo | grep -c ^processor`

# Update the makefile
echo "INFO:: Adapt the test run makefile"
cat tests.mk | \
    sed -e "s|make -j|make -j2 BUILD_DIR=$output_dir|g" \
        -e "s|> TEST_|> $output_dir/TEST_|g" \
        -e "s|\./TEST_\([^\ ]\+\)\.out >|TEST_\1.out >|g" \
        -e "s|TEST_\([^\ ]\+\)\.iso|$output_dir/TEST_\1.iso|g" \
        -e "s| TEST_\([^\ ]\+\)\.out| $output_dir/TEST_\1.out|g" \
        -e "s| obj-\([^\ ]\+\)| $output_dir/obj-\1|g" \
        -e "s|TEST_\([^\ ]\+\)_Execute2.log >>|$output_dir/TEST_\1_Execute2.log >>|g" \
        -e "s|TEST_\([^\ ]\+\)\.bochs\.log|$output_dir/TEST_\1.bochs.log|g" \
        -e "s|touch TEST_\([^\ ]\+\)|touch $output_dir/TEST_\1|g" \
    > tests-${suffix}.mk
rm tests.mk

# Run tests
make -j${ncpus} -f tests-${suffix}.mk -k

# Save the generated makefile
mv tests-${suffix}.mk $output_dir

# Save changesets
cat <<END > $output_dir/changesets
mutekh:$changeset_mutekh
END

# Copy logs
mkdir -p $destdir/www/logs/$suffix
mv $output_dir/*.log $destdir/www/logs/$suffix/

popd 2>&1 >/dev/null
# End of test run

# Load ruby environment
. $HOME/.rvm/scripts/rvm

# Make reports
$here/scripts/report.rb -r $destdir/outputs -d $destdir/www > $output_dir/report.txt

if [ $? -ne 0 ]; then
    irc_log "Test run ends, changeset '$changeset_mutekh', see results at http://mutekh-tests.ssji.net/runs/${suffix}.html"
    irc_log "  Results: `cat $output_dir/report.txt`"
fi

# Cleanup obj directories
for d in `find $output_dir -name "obj-*" -type d`; do
    echo "INFO:: Deleting directory $d..."
    rm -rf $d
done

# Sync with public hosting
rsync -avz --delete $destdir/www/ nsa@ssh.ssji.net:public_html/mutekh-tests.ssji.net/

# Print footer
echo
echo "########################################################################"
echo "# Test run ends on `date "+%d-%m-%Y %H:%M:%S"`                         #"
echo "########################################################################"
