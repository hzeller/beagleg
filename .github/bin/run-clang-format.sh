#!/bin/bash

FORMAT_OUT=${TMPDIR:-/tmp}/clang-format-diff.out

# Run on all files except the ones that are generated.
# TODO: have generator scripts run clang-format on their
#   output as well as last step, then we can exclude less
#   files here.
find src -name "*.h" -o -name "*.cc"  \
    | xargs clang-format -i \
            --style="{BasedOnStyle: Google,\
                      ContinuationIndentWidth: 2, \
                      IndentPPDirectives: None,\
                      AllowShortCaseLabelsOnASingleLine: true,\
                      AlignConsecutiveBitFields: true,\
                      AlignConsecutiveMacros: true,\
                      IndentCaseLabels: false}"

# Check if we got any diff, then print it out in in the CI.
# TODO: make these suggested diffs in the pull request.
git diff > ${FORMAT_OUT}

if [ -s ${FORMAT_OUT} ]; then
    echo "== There were changes running the formatter =="
    cat ${FORMAT_OUT}
    echo "To locally fix, run .github/bin/run-clang-format.sh then commit and push."
    exit 1
fi

exit 0
