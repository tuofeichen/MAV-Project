#!/bin/sh -x
# Modified version of catkin template/python_distutils_install.sh.in

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

/usr/bin/env \
    "/usr/bin/python" \
    "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/mavlink/pymavlink/setup.py" \
    build --build-base "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/build/mavlink/pymavlink/pybuild" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/tuofeichen/SLAM/MAV-Project/catkin_ws/devel"
