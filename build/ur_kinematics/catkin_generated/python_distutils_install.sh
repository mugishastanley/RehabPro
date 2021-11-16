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

echo_and_run cd "/home/adminuser/ws_moveit3/src/universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/adminuser/ws_moveit3/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/adminuser/ws_moveit3/install/lib/python2.7/dist-packages:/home/adminuser/ws_moveit3/build/ur_kinematics/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/adminuser/ws_moveit3/build/ur_kinematics" \
    "/usr/bin/python2" \
    "/home/adminuser/ws_moveit3/src/universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/adminuser/ws_moveit3/build/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/adminuser/ws_moveit3/install" --install-scripts="/home/adminuser/ws_moveit3/install/bin"
