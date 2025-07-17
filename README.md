# Tamarindo Docker Installation

## Installation Steps

1. Place the `Dockerfile` and the `.sh` script in the **same folder**.
2. Open a terminal in that folder and run the following commands:

```bash
# Build the Docker image
docker build -t argos3-tuttifrutti .

# Run the container with X11 forwarding and necessary privileges
docker run -it \
    --name het-tuttifrutti-container \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $HOME/.Xauthority:/root/.Xauthority \
    -v $HOME/argos3-tuttifrutti/AutoMoDe-tuttifrutti:/home/robotmaster/AutoMoDe-tuttifrutti \
    -v $HOME/argos3-tuttifrutti/heterogenity_habanero_loopfunctions:/home/robotmaster/experiments-loop-functions \
    --net=host \
    --privileged \
    argos3-tuttifrutti /bin/bash
