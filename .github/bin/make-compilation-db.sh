#!/bin/bash
set -u

TMPDIR="${TMPDIR:-/tmp}"
readonly BUILD_OUT=${TMPDIR}/build.out

make clean > /dev/null 2>&1
BEAR_COMMAND_SEPARATOR="--"
if bear --version | grep "2\.4" ; then
    # Older versions of bear did not have the -- separator. The github actions
    # run on an Ubuntu which still has the 2.4 version.
    BEAR_COMMAND_SEPARATOR=""
fi
time bear ${BEAR_COMMAND_SEPARATOR} make -C src all test-binaries > ${BUILD_OUT} 2>&1

if [ $? -ne 0 ]; then
    cat ${BUILD_OUT}
    echo "Build failure."
    exit 1
fi
echo "done."
exit 0
