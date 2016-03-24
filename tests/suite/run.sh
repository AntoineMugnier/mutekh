#!/bin/bash

# Mandatory directories that must exist in $destdir
MANDATORY_DIRS="outputs www mutekh soclib"

# Selected tests from mutekh/tests/pool
SELECTED_POOL_TESTS=""

# IRC logger fifo name
IRC_LOGGER_FIFO_NAME="logger.pipe"

###############################################################################

here=$(dirname `readlink -f $0`)
destdir=$PWD

# Setup environment
. $here/setup-env.sh

# Setup mtest tool
export PYTHONPATH

if [ -z $PYTHONPATH ]; then
PYTHONPATH=$here/../lib/python
else
PYTHONPATH=$PYTHONPATH:$here/../lib/python
fi
MTEST="python -m mt.generator"

run_command()
{
    keep=0
    if [ "$1" == "keep" ]; then
        keep=1
        shift
    fi

    $*
    result=$?
    if [ $keep -eq 0 -a $result -ne 0 ]; then
        echo "ERROR: Failed to run command '$*'."
        exit $result
    fi
    return $result
}

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
#(hg pull -u && hg purge) || exit 1
run_command hg pull -u
run_command hg purge

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

# Prepare mutekh repository
echo "INFO:: Changes found. Preparing the MutekH repository..."
run_command rm -f tests.mk

# Generate test makefile
echo "INFO:: Generating test run makefile..."
enabled_pool_tests=( $SELECTED_POOL_TESTS )
if [ -f tests/pool/enabled.list ]; then
    enabled_pool_tests=( $SELECTED_POOL_TESTS $(cat tests/pool/enabled.list | grep -v "^#") )
fi

mtest_args="${enabled_pool_tests[@]/#/tests/pool/}"
$MTEST $mtest_args

[ $? -ne 0 ] && exit 1

# Update the makefile
echo "INFO:: Adapting the test run makefile..."
suffix=`date +%Y%m%d-%H%M%S`
that_run_dir="$destdir/outputs/run-$suffix"
cat tests.mk | \
    sed -e "s|make -j|make -j2 BUILD_DIR=$that_run_dir|g" \
        -e "s|> TEST_|> $that_run_dir/TEST_|g" \
        -e "s|\./TEST_\([^\ ]\+\)\.out >|TEST_\1.out >|g" \
        -e "s|TEST_\([^\ ]\+\)\.iso|$that_run_dir/TEST_\1.iso|g" \
        -e "s| TEST_\([^\ ]\+\)\.out| $that_run_dir/TEST_\1.out|g" \
        -e "s| obj-\([^\ ]\+\)| $that_run_dir/obj-\1|g" \
        -e "s|TEST_\([^\ ]\+\)_Execute2.log >>|$that_run_dir/TEST_\1_Execute2.log >>|g" \
        -e "s|TEST_\([^\ ]\+\)\.bochs\.log|$that_run_dir/TEST_\1.bochs.log|g" \
        -e "s|touch TEST_\([^\ ]\+\)|touch $that_run_dir/TEST_\1|g" \
    > tests-${suffix}.mk
run_command rm -f tests.mk

# Create test run output directory
echo "INFO:: Creating output directory '$that_run_dir'..."
mkdir -p $that_run_dir

# Copy previous
echo "INFO:: Copying latest test run state..."
if [ -e $destdir/outputs/latest ]; then
    last_run_dep_tests=$(find $destdir/outputs/latest -name "TEST_D_*")
    if [ -n $last_run_dep_tests ]; then
        cp -a $destdir/outputs/latest/TEST_D_* $that_run_dir/
        rm -f $that_run_dir/*.out
    fi
fi

# Create 'latest' symlink
echo "INFO:: Linking the current test run..."
(cd $destdir/outputs; run_command rm -f latest; run_command ln -s `basename $that_run_dir` latest)

# Compute vcpu number
echo "INFO:: Computing the number of available vcpus..."
ncpus=`cat /proc/cpuinfo | grep -c ^processor`
ncpus=1

# Run tests
echo "INFO:: Running tests..."
run_command keep make -j${ncpus} -f tests-${suffix}.mk -k

# Save the generated makefile
run_command mv tests-${suffix}.mk $that_run_dir

# Save changesets
cat <<END > $that_run_dir/changesets
mutekh:$changeset_mutekh
END

# Copy logs
echo "INFO:: Copying logs..."
run_command mkdir -p $destdir/www/logs/$suffix
for f in $that_run_dir/*.log; do
    destfile=$(echo `basename $f` | sed 's/TEST_\(D\|N\)/TEST/g')
    echo "  - $f -> $destdir/www/logs/$suffix/$destfile"
    run_command cp $f $destdir/www/logs/$suffix/$destfile
done

popd 2>&1 >/dev/null
# End of test run

# Load ruby environment
. $HOME/.rvm/scripts/rvm

# Make reports
echo "INFO:: Generating the final report..."
$here/scripts/report.rb -r $destdir/outputs -d $destdir/www > $that_run_dir/report.txt

# Cleanup obj directories
for d in `find $that_run_dir -name "obj-*" -type d`; do
    echo "INFO:: Deleting directory $d..."
    rm -rf $d
done

# Sync with public hosting
rsync -avz --delete $destdir/www/ nsa@ssh.ssji.net:public_html/mutekh-tests.ssji.net/
rsync -avz --delete $destdir/www/. nsa@ssh.ssji.net:/srv/web-ssl/mutekh.org/www/testsuite/.

# Print footer
echo
echo "########################################################################"
echo "# Test run ends on `date "+%d-%m-%Y %H:%M:%S"`                         #"
echo "########################################################################"
