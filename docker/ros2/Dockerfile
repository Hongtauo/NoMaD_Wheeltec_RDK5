# Build from the project root. Requires the project bind mount at runtime.
# This is an environment image; project code, weights and results stay external.
FROM osrf/ros:humble-desktop-full-jammy

ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
    && DEBIAN_FRONTEND=noninteractive dpkg-reconfigure tzdata

COPY docker/ros2 /opt/nomad-setup
RUN bash /opt/nomad-setup/install_dependencies.sh --dependencies-only
RUN printf '\nsource /opt/ros/humble/setup.bash\n' >> /root/.bashrc

# The existing development container uses editable installs. In rebuilt images,
# these paths expose the same mounted source trees without copying model weights.
ENV PYTHONPATH=/workspace/SocialEmbodiedAgent/visualnav-transformer/train:/workspace/SocialEmbodiedAgent/diffusion_policy
WORKDIR /workspace/SocialEmbodiedAgent
CMD ["bash"]
