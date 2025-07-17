FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Update and install system dependencies
# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    sudo \
    bash \
    nano \
    pkg-config \
    liblua5.3-dev \
    lua5.3 \
    libboost-all-dev \
    libfreeimage-dev \
    libfreeimageplus-dev \
    libeigen3-dev \
    libxml2-dev \
    libtinyxml-dev \
    libcurl4-openssl-dev \
    libqwt-qt5-dev \
    qt5-default \
    libqt5opengl5-dev \
    libglew-dev \
    libglu1-mesa-dev \
    libgl1-mesa-dev \
    libglfw3-dev \
    freeglut3-dev \
    libxi-dev \
    libxmu-dev \
    x11-apps \
    curl \
    ca-certificates \
    xz-utils \
    && rm -rf /var/lib/apt/lists/*



# Create a non-root user (optional)
RUN useradd -ms /bin/bash robotmaster
USER robotmaster
WORKDIR /home/robotmaster

# Copy your installation script
COPY --chown=robotmaster:robotmaster install_tuttifrutti.sh .

# Make sure it's executable
RUN chmod +x install_tuttifrutti.sh

# Run the install script
RUN bash ./install_tuttifrutti.sh

# Optional: Setup environment when container starts
RUN echo "source /home/robotmaster/argos3-installation/tuttifrutti/argos3-dist/bin/setup_argos3" >> /home/robotmaster/.bashrc

# Default shell
CMD ["/bin/bash"]
