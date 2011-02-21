#!/bin/sh
#
# Usage: $0 /path/to/python-script args-to-script
#
# Try out different pythons from the most recent ones I basically dont trust
# default "python" command, it is often the antic python-1.5.x on redhat
# enterprise :'(

# Note to self: when dropping support for python2.4, move soclib_cc_main.py to
# soclib_cc/main.py, -m soclib_cc.main is accepted since 2.5.

for py in python2.7 python2.6 python2.5 python2.4 python ; do
	if ($py -c "import sys;sys.exit(0)" 2>&1) >/dev/null ; then
        pyargs=
        if [ "$py" = "python2.6" ] ; then
            pyargs='-3'
        fi
		exec $py $pyargs $*
	fi
done

echo "No python found, this is not good..."
exit 2
