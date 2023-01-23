#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/pavan/hsr2/catkin_ws/src/navigation/base_local_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pavan/hsr2/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pavan/hsr2/catkin_ws/install/lib/python3/dist-packages:/home/pavan/hsr2/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pavan/hsr2/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/pavan/hsr2/catkin_ws/src/navigation/base_local_planner/setup.py" \
     \
    build --build-base "/home/pavan/hsr2/catkin_ws/build/navigation/base_local_planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pavan/hsr2/catkin_ws/install" --install-scripts="/home/pavan/hsr2/catkin_ws/install/bin"
