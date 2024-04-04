```bash
docker pull ubuntu:22.04
```

```bash
xhost + &&
docker run -it --rm --net=host --name=autoware-container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ubuntu:22.04
```
