
```bash
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
mkdir src
vcs import src < autoware.repos
```

```bash
docker pull ubuntu:22.04
```

```bash
xhost + &&
docker run -it --net=host --name=autoware-container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    autoware:setup-dev-env
```
