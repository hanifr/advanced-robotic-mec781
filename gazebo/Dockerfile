FROM osrf/ros:noetic-desktop-full

# Install VNC server and desktop environment
RUN apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    tigervnc-xorg-extension \
    xfce4 \
    xfce4-terminal \
    && rm -rf /var/lib/apt/lists/*

# Set up VNC password
RUN mkdir -p /root/.vnc && \
    echo "password" | vncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd

# Create startup script
RUN echo '#!/bin/bash \n\
vncserver :1 -geometry 1280x800 -depth 24 -localhost no \n\
tail -f /root/.vnc/*:1.log' > /start-vnc.sh && \
    chmod +x /start-vnc.sh

# Expose VNC port
EXPOSE 5901

CMD ["/start-vnc.sh"]