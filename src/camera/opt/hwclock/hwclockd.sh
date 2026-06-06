#!/usr/bin/env bash
#
# Script to launch daemon to read and ajust time and not become closer to 1970
# Installation
# ln -s /opt/hwclock/hwclockd.sh /etc/init.d/S20hwclockd
#
# configuration located as gpstimed.conf (see CONFIG below)

DAEMON="fake-hwclockd"
DAEMON_HOME="$(dirname $(realpath $0))"
EXECUTABLE="$DAEMON_HOME/$DAEMON"
PIDFILE="/var/run/$DAEMON.pid"
DAEMON_ARGS="save"

function loggy() {
    logger -s -t $DAEMON "$@"
}

function start() {
    loggy "using $EXECUTABLE to restore current time"
    $EXECUTABLE load
    loggy "starting $EXECUTABLE using $DAEMON_ARGS"
    # shellcheck disable=SC2086 # we need the word splitting
    # background execution must be not used as daemon uses stdout
    start-stop-daemon -S -v -m -p "$PIDFILE" -x /usr/bin/setsid -- /usr/bin/setsid $EXECUTABLE $DAEMON_ARGS
    status=$?
    if [ "$status" -eq 0 ]; then
            echo "OK"
    else
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
