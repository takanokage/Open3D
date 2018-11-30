#!/bin/sh

# get the name of the upper level directory
NAME=$(bash -c 'basename $(cd .. ; pwd)')
TAG=no_deps
CONTAINER_NAME=${NAME}_${TAG}
DOCKERFILE=Dockerfile_$TAG

# stop the container if it's already running
docker container stop -t 0 $CONTAINER_NAME

# delete the previous container
docker image rm $NAME:$TAG

# build the image
docker image build -t $NAME:$TAG -f ../$DOCKERFILE ..

# run the container
docker container run \
       --rm \
       -d \
       -t \
       -h $CONTAINER_NAME \
       --name $CONTAINER_NAME \
       $NAME:$TAG

# attach to the container, clone & build & install Open3d
docker container exec -it -w /root $CONTAINER_NAME bash -c '\
    echo && \
    git clone https://github.com/IntelVCL/Open3D.git open3d && \
    cd open3d && \
    echo && \
    echo building... && \
    mkdir -p build && \
    cd build && \
    cmake .. -DBUILD_EIGEN3=ON \
             -DBUILD_GLEW=ON \
             -DBUILD_GLFW=ON \
             -DBUILD_JPEG=ON \
             -DBUILD_JSONCPP=ON \
             -DBUILD_PNG=ON \
             -DBUILD_UNIT_TESTS=ON \
             -DCMAKE_BUILD_TYPE=Release && \
    echo && \
    make -j && \
    echo && \
    echo running the unit tests... && \
    ./bin/unitTests'

# stop the container
docker container stop -t 0 $CONTAINER_NAME

# display images in order to check image size
docker image ls
