#!/usr/bin/env bash
#
# Script to launch recording daemon
#
# configuration located as recorderd.conf (see CONFIG below)

DAEMON="recorderd"                            # this one is a separate script just to perform writing
DAEMON_HOME="$(dirname $(realpath $0))"
CONFIG="$DAEMON_HOME/$DAEMON.conf"
EXECUTABLE="$DAEMON_HOME/$DAEMON"
PIDFILE="/var/run/$DAEMON.pid"

# shellcheck source=/dev/null
[ -r "$CONFIG" ] && . "$CONFIG"
# shellcheck source=/dev/null
[ -r "/etc/default/$DAEMON" ] && . "/etc/default/$DAEMON"

# nice place for common functions
function loggy() {
    logger -s -t recorderd "$@"
}

function start() {
    loggy "validating status to start $EXECUTABLE"
    [ -d "$RECORDER_FOLDER" ] || mkdir -p "$RECORDER_FOLDER"
    fstrim --verbose "$RECORDER_FOLDER"

    # basically, SERIAL_DEVICE needs to be checked
    if [ -n "$SERIAL_DEVICE" ]; then
        loggy "serial device $SERIAL_DEVICE to be used"
    else
        loggy "ERROR: there is no SERIAL_DEVICE specified to use"
        echo "FAIL"
        return "1"
    fi
    # and now require properly appears setup serial
    while [ ! -e "$SERIAL_DEVICE" ]; do
        loggy "pending serial device $SERIAL_DEVICE up and running"
        sleep 1
    done
    if [ -n "$SERIAL_OPTIONS" ]; then
        sleep 1
        if stty -F $SERIAL_DEVICE $SERIAL_OPTIONS; then
            loggy "successfully setup $SERIAL_DEVICE for options $SERIAL_OPTIONS"
        else
            loggy "ERROR: cannot setup $SERIAL_DEVICE device using stty -F $SERIAL_DEVICE $SERIAL_OPTIONS"
            echo "FAIL"
            return "1"
        fi
    else
        loggy "there is no SERIAL_OPTIONS set - defaults will be used"
    fi

    # construct args
    DAEMON_ARGS="$SERIAL_DEVICE $RECORDER_FOLDER $RECORDER_FILE $RECORDER_FILE_TAG"

    # shellcheck disable=SC2086 # we need the word splitting
    start-stop-daemon -S -v -b -m -p "$PIDFILE" -x "$EXECUTABLE" -- $DAEMON_ARGS
    status=$?
    if [ "$status" -eq 0 ]; then
        loggy "successfully run $EXECUTABLE $DAEMON_ARGS"
        echo "OK"
    else
        loggy "ERROR: failed to launch $EXECUTABLE $DAEMON_ARGS"
        echo "FAIL"
    fi
    return "$status"
}

function stop() {
    loggy "stopping $DAEMON using PID $(cat $PIDFILE)"
    start-stop-daemon -K -q -p "$PIDFILE"
    status=$?
    if [ "$status" -eq 0 ]; then
            rm -f "$PIDFILE"
            echo "OK"
    else
            echo "FAIL"
    fi
    return "$status"
}

function restart() {
    stop
    sleep 1
    start
}

case "$1" in
    start|stop|restart)
            "$1";;
    reload)
            # Restart, since there is no true "reload" feature.
            restart;;
    *)
            echo "Usage: $0 {start|stop|restart}"
            exit 1
esac
