#! /bin/bash

set -e

start() {
    if [[ -z "$HOME" ]]; then
        export HOME=/root
    fi
    source /opt/ros/humble/setup.bash
    source /rmcs_install/setup.bash
    nohup ros2 launch rmcs_bringup rmcs.launch.py &> rmcs.launch.out 2>&1 & echo $! > /tmp/rmcs.pid
}

stop() {
    if [ -f "/tmp/rmcs.pid" ]; then
        pkill -P "$(cat /tmp/rmcs.pid)"
    fi
}

check_running() {
    if [ -f "/tmp/rmcs.pid" ]; then
        if ps -p "$(cat /tmp/rmcs.pid)" > /dev/null
        then
            echo 'Fatal: RMCS daemon is still running!'
            exit 1
        fi
    fi
}

case "$1" in
    start)
        check_running
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop
        start
        ;;
    *)
        echo 'Usage: /etc/init.d/rmcs {start|stop|restart}'
        exit 1
        ;;
esac

exit 0