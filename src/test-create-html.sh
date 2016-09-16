#!/bin/bash
##

if [ $# -lt 1 ] ; then
    echo "Usage: $0 <gcode-files>"
    exit 1
fi

IMAGE_SIZE="24%"
THRESHOLD_ANGLE="-t16"

# For now, let's use the configuration in which each axis has the same steps/mm.
# The step-speed-different.config _should_ look the same in the output, but
# currently the output is clearly broken, so let's not use it initially to
# first fix what is broken alrady.
BEAGLEG_CONFIG=testdata/step-speed-same.config
#BEAGLEG_CONFIG=testdata/step-speed-different.config

GCODE2PS=$(dirname $0)/gcode2ps
TEST_OUT_DIR=$(dirname $0)/test-out
OUT_HTML=$TEST_OUT_DIR/test.html

mkdir -p $TEST_OUT_DIR
rm -f $OUT_HTML
while [ $# -ne 0 ] ; do
    GCODE_FILE=$1
    BASENAME=$(basename $GCODE_FILE .gcode)
    $GCODE2PS $THRESHOLD_ANGLE -o $TEST_OUT_DIR/${BASENAME}.ps -c $BEAGLEG_CONFIG -s -T2 $GCODE_FILE
    if [ $? -ne 0 ] ; then
	ERRMSG="<span style='color:#ff0000; font-weight:bold;'>Got gcode errors</span>"
    else
	ERRMSG=""
    fi
    # create PNG
    gs -q -r144 -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop -dBATCH -dNOPAUSE -sDEVICE=png16m -sOutputFile=$TEST_OUT_DIR/${BASENAME}.png $TEST_OUT_DIR/${BASENAME}.ps

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
