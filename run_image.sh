xhost +


docker run -it --rm --name l4d \
    --mount type=bind,source="$(pwd)",target=/app \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --privileged \
     l4dead 
