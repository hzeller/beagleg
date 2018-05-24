#!/bin/bash
##

if [ $# -lt 1 ] ; then
    echo "Usage: $0 <gcode-files>"
    exit 1
fi

#OPTIONAL_VALGRIND="valgrind --error-exitcode=5"
IMAGE_SIZE="24%"
PARAMS="-q -w1200 -t16 -Viso -g10"

# For now, let's use the configuration in which each axis has the same steps/mm.
# The step-speed-different.config _should_ look the same in the output, but
# currently the output is clearly broken, so let's not use it initially to
# first fix what is broken alrady.
# If config is commented out entirely, we only get the gcode output without
# colored machine move.
BEAGLEG_CONFIG="-c testdata/step-speed-same.config"
#BEAGLEG_CONFIG="-c testdata/step-speed-different.config"

GCODE2PS=$(dirname $0)/gcode2ps
TEST_OUT_DIR=$(dirname $0)/test-out
OUT_HTML=$TEST_OUT_DIR/test.html

mkdir -p $TEST_OUT_DIR
rm -f $OUT_HTML
while [ $# -ne 0 ] ; do
    GCODE_FILE=$1
    BASENAME=$(basename $GCODE_FILE .gcode)
    $OPTIONAL_VALGRIND $GCODE2PS $PARAMS -o $TEST_OUT_DIR/${BASENAME}.ps $BEAGLEG_CONFIG -s -T2 $GCODE_FILE -C "${BASENAME}"
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 5 ] ; then
	ERRMSG="<span style='color:#ffff00; font-weight:bold; background-color:#ff0000'> Got valgrind errors </span>"
    elif [ $EXIT_CODE -ne 0 ]; then
	ERRMSG="<span style='color:#ffff00; font-weight:bold; background-color:#ff0000'> Got execution errors; exit=$EXIT_CODE </span>"
    else
	ERRMSG=""
    fi
    # create PNG
    gs -q -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop -dBATCH -dNOPAUSE -sDEVICE=png16m -sOutputFile=$TEST_OUT_DIR/${BASENAME}.png $TEST_OUT_DIR/${BASENAME}.ps

    cat <<EOF >> $OUT_HTML
<div style="width:${IMAGE_SIZE}; float:left; border:1px solid #ccc; margin:2px; background-color:#ccc;">
<a href="../testdata/${BASENAME}.gcode" style="text-align:center; font-weight:bold;">${BASENAME}</a>
<a href='${BASENAME}.png'><img src='${BASENAME}.png' width='100%' title='${BASENAME}'></a>
$ERRMSG
</div>
EOF
    shift  # next file
done

echo
echo "Output is in $(realpath $OUT_HTML)"
