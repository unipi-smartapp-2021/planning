FROM ros:noetic

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y git && \
    #apt-get install -y build-essential && \
    apt-get install -y python3-catkin-tools && \
    apt-get install -y python3-scipy && \
    apt-get install -y python3-matplotlib

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

RUN catkin_make