docker run -it --rm --name l4d \
    --mount type=bind,source="$(pwd)"/data,target=/app \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    l4dead:latest