#!/usr/bin/env bash
#
# Script to launch camerad daemon, installation
# ln -s /opt/camera/camerad.sh /etc/init.d/S99camerad
#
# configuration located as camera.conf (see CONFIG below)

DAEMON="camerad"                            # this one is a separate script just to perform writing
DAEMON_HOME="$(dirname $(realpath $0))"
CONFIG="$DAEMON_HOME/$DAEMON.conf"
EXECUTABLE="$DAEMON_HOME/$DAEMON"
PIDFILE="/var/run/$DAEMON.pid"

# shellcheck source=/dev/null
[ -r "$CONFIG" ] && . "$CONFIG"
# shellcheck source=/dev/null
[ -r "/etc/default/$DAEMON" ] && . "/etc/default/$DAEMON"


function check_pc_connected() {
    local entry="$1"
    local stopper="$2"
    local value="$(cat $entry)"

    loggy "value in $entry is $value, looking for $stopper"
    [ "$value" == "$stopper" ] && exit 0
}


function start() {
    loggy "validating status to start $EXECUTABLE"
    # pessimistic approach to indicate no writing happened
    led_control "none" "0"
    check_pc_connected "$ANDROID_USB_STATE" "CONFIGURED"
    check_pc_connected "$CARRIER_USB_STATE" "1"
    # now turn lights on to show all working
    led_control "none" "1"
    [ -d "$VIDEO_FOLDER" ] || mkdir -p "$VIDEO_FOLDER"
    fstrim --verbose "$VIDEO_FOLDER"
    # shellcheck disable=SC2086 # we need the word splitting
    start-stop-daemon -S -v -b -m -p "$PIDFILE" -x "$EXECUTABLE" -- $EXECUTABLE $DAEMON_ARGS
    status=$?
    if [ "$status" -eq 0 ]; then
        sleep 5 # allow to start properly before continue
        led_control "mmc1" "1"
        echo "OK"
    else
        led_control "none" "0"
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
