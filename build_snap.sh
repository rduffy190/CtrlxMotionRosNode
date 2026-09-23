#!/bin/bash
#!/bin/bash
set -e

rm -rf install/
mkdir -p install/app
rm -rf build/
rm -rf log/

source /opt/ros/jazzy/setup.bash
colcon build


if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

snapcraft clean
snapcraft pack --build-for=amd64 --verbosity=verbose