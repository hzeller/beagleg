#!/bin/bash
# Dropping permisisons to this after hardware is initialized
PERMISSION_DROP=daemon:daemon

VARDIR=/var/beagleg
mkdir -p $VARDIR
chown $PERMISSION_DROP $VARDIR

# The port we're listening for gcode commands.
PORT=4444

# Status server port.
STATUS_PORT=4445

# Machine configuration of motors and switches etc.
CONFIG=/etc/beagleg.config

# Permanent storage of variables used in GCode
PARAMS_FILE=$VARDIR/beagleg.params

# Logfile
LOGFILE=$VARDIR/beagleg.log

# Machine control binary. Default 'make install' location is in /usr/local/bin
MACHINE_CONTROL=/usr/local/bin/machine-control

# We do need the uio-pruss module to be loaded.
# TODO: figure out what in systemd the preferred 'After' target is to
# have his already taken care of.
modprobe uio_pruss

exec ${MACHINE_CONTROL} --priv=${PERMISSION_DROP} --config=${CONFIG} --port=${PORT} --status-server=${STATUS_PORT} --logfile=${LOGFILE} --param=${PARAMS_FILE}
