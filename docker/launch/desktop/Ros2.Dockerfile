ARG BASEIMAGE
FROM $BASEIMAGE

#========================================
# ENVIRONMENT VARIABLES
#========================================
ENV APT_INSTALL="apt-get install -y --no-install-recommends"
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm256-color
ENV force_color_prompt=yes

#========================================
# BASE PACKAGES
#========================================
RUN cat /etc/resolv.conf
RUN cat /etc/apt/sources.list
RUN rm -rf  /var/lib/apt/lists/* \
    && apt-get update \
    && $APT_INSTALL \
    apt-transport-https \
    ca-certificates \
    gnupg \
    software-properties-common \
    wget \
    apt-utils \
    && apt-get update \
    && $APT_INSTALL \
    build-essential \
    net-tools \
    cmake \
    curl \
    git \
    vim \
    tmux \
    python3-pip \
    libassimp-dev \
    xorg-dev \
    libbotan-2-dev \
    libglu1-mesa-dev 



#========================================
# GUI DEPENDENCIES
#========================================

#ENV NODE_VERSION=16.13.0
#RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
#ENV NVM_DIR=/root/.nvm
#RUN . "$NVM_DIR/nvm.sh" && nvm install ${NODE_VERSION}
#RUN . "$NVM_DIR/nvm.sh" && nvm use v${NODE_VERSION}
#RUN . "$NVM_DIR/nvm.sh" && nvm alias default v${NODE_VERSION}
#ENV PATH="/root/.nvm/versions/node/v${NODE_VERSION}/bin/:${PATH}"
#RUN node --version
#RUN npm --version
RUN apt-get update
RUN apt-get install -y ca-certificates curl gnupg
RUN mkdir -p /etc/apt/keyrings
RUN curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg
ENV NODE_MAJOR=18
RUN echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list
RUN apt-get update
RUN apt-get install nodejs -y
RUN pip3 install \
    flask \
    flask_socketio \
    eventlet \
    python-socketio \
    websocket-client \
    opencv-python \
    scipy \
    bluerobotics-ping \
    ouster-sdk \
    rosbags \
    pyproj 
RUN pip3 install "numpy<1.24"

ENV NODE_OPTIONS=--openssl-legacy-provider
#Fix rviz2 black screen issue
RUN add-apt-repository ppa:kisak/kisak-mesa && apt upgrade -y

#========================================
# FILE STRUCTURE
#========================================
RUN mkdir /home/packages
ENV SOURCE_DIR="/home/packages" 
WORKDIR ${SOURCE_DIR}
#VOLUME ["${SOURCE_DIR}"]
#ADD ../../NVIDIA-OptiX-SDK-7.3.0-linux64-x86_64.sh ${SOURCE_DIR}

#========================================
# STEAM AND DEPENDENCIES
#========================================
WORKDIR ${SOURCE_DIR}
RUN git clone https://github.com/utiasASRL/lgmath.git 
RUN mkdir ${SOURCE_DIR}/lgmath/build
WORKDIR ${SOURCE_DIR}/lgmath/build
RUN cmake -DUSE_AMENT=OFF ..
RUN cmake --build .
RUN cmake --install .

WORKDIR ${SOURCE_DIR}
RUN git clone https://github.com/utiasASRL/steam.git 
RUN mkdir ${SOURCE_DIR}/steam/build
WORKDIR ${SOURCE_DIR}/steam/build
RUN cmake -DUSE_AMENT=OFF ..
RUN cmake --build .
RUN cmake --install .

#========================================
# EMBREE AND DEPENDENCIES
#========================================
#
#WORKDIR ${SOURCE_DIR}
#RUN git clone https://github.com/embree/embree.git
#RUN mkdir ${SOURCE_DIR}/embree/build
#WORKDIR ${SOURCE_DIR}/embree/build
#RUN cmake ..
#RUN make -j 8
#RUN make install
#========================================
# RMAGINE 
#========================================
# This is still in my custom branch of rocker because you 
# only build this after cuda is installed
#WORKDIR ${SOURCE_DIR}/rmagine/build
#RUN make install

#
#========================================
# ENTER
#========================================
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]
