```bash
docker pull ubuntu:22.04
```

```bash
xhost + &&
docker run -it --rm --net=host --name=autoware-container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/autoware:/root/autoware \
    -v $HOME/autoware_map:/root/autoware_map \
    -v $HOME/autoware_data:/root/autoware_data \
    ubuntu:22.04
```
