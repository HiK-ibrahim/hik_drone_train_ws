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

echo_and_run cd "/home/hik/Masaüstü/ros/hik_drone_train_ws/src/hector_quadrotor_noetic/geographic_info/geodesy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hik/Masaüstü/ros/hik_drone_train_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hik/Masaüstü/ros/hik_drone_train_ws/install/lib/python3/dist-packages:/home/hik/Masaüstü/ros/hik_drone_train_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hik/Masaüstü/ros/hik_drone_train_ws/build" \
    "/usr/bin/python3" \
    "/home/hik/Masaüstü/ros/hik_drone_train_ws/src/hector_quadrotor_noetic/geographic_info/geodesy/setup.py" \
    egg_info --egg-base /home/hik/Masaüstü/ros/hik_drone_train_ws/build/hector_quadrotor_noetic/geographic_info/geodesy \
    build --build-base "/home/hik/Masaüstü/ros/hik_drone_train_ws/build/hector_quadrotor_noetic/geographic_info/geodesy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hik/Masaüstü/ros/hik_drone_train_ws/install" --install-scripts="/home/hik/Masaüstü/ros/hik_drone_train_ws/install/bin"
