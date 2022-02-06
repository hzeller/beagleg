#!/bin/bash

set -u

TMPDIR="${TMPDIR:-/tmp}"

readonly PARALLEL_COUNT=$(nproc)
readonly FILES_PER_INVOCATION=5

readonly FILES_TO_PROCESS=${TMPDIR}/clang-tidy-files.list
readonly TIDY_OUT=${TMPDIR}/clang-tidy.out

# Using clang-tidy-11 as it is the last one still checking for
# google-runtime-references, non-const references in parameters - which is
# the preferred style in this project.
readonly CLANG_TIDY=clang-tidy-11
hash ${CLANG_TIDY} || exit 2  # make sure it is installed.

echo ::group::Build compilation database

$(dirname $0)/make-compilation-db.sh
if [ $? -ne 0 ]; then
    echo "::error::Can't generate compilation database."
    exit 1
fi

find src -name "*.h" -o -name "*.cc" > ${FILES_TO_PROCESS}

echo "::endgroup::"

echo "::group::Run ${CLANG_TIDY} on $(wc -l < ${FILES_TO_PROCESS}) files"

cat ${FILES_TO_PROCESS} \
    | xargs -P${PARALLEL_COUNT} -n ${FILES_PER_INVOCATION} -- \
            ${CLANG_TIDY} --quiet 2>/dev/null \
            > ${TIDY_OUT}.tmp

# Only modify at the end, so we can inspect the file manually while a new run
# of this script is in progress.
mv ${TIDY_OUT}.tmp ${TIDY_OUT}

cat ${TIDY_OUT}

echo "::endgroup::"

if [ -s ${TIDY_OUT} ]; then

    # Tidy results were non-empty. Put a summary into a separate group.
    echo "::group::Summary"
    # Give a nice overview of how many counts of which problem found.
    sed 's|\(.*\)\(\[[a-zA-Z.-]*\]$\)|\2|p;d' < ${TIDY_OUT} | sort | uniq -c | sort -n
    echo "::endgroup::"

    echo "::error::There were clang-tidy warnings. Please fix"
    exit 1
fi

echo "No clang-tidy complaints.😎"
exit 0
