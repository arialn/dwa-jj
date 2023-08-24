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

echo_and_run cd "/home/z_lin/mapless/src/geometry2/tf2_kdl"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/z_lin/mapless/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/z_lin/mapless/install/lib/python2.7/dist-packages:/home/z_lin/mapless/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/z_lin/mapless/build" \
    "/usr/bin/python2" \
    "/home/z_lin/mapless/src/geometry2/tf2_kdl/setup.py" \
     \
    build --build-base "/home/z_lin/mapless/build/geometry2/tf2_kdl" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/z_lin/mapless/install" --install-scripts="/home/z_lin/mapless/install/bin"
