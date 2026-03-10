# The devcontainer should use the developer target and run as root with podman
# or docker with user namespaces.
FROM ghcr.io/diamondlightsource/ubuntu-devcontainer:noble AS developer

# Add GitHub CLI repository and system dependencies for the developer/build environment
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    && curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
    -o /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
    > /etc/apt/sources.list.d/github-cli.list \
    && apt-get update -y && apt-get install -y --no-install-recommends \
    gh \
    graphviz \
    && rm -rf /var/lib/apt/lists/*

# The build stage installs the context into the venv
FROM developer AS build

# Change the working directory to the `app` directory
# and copy in the project
WORKDIR /app
COPY . /app
RUN chmod o+wrX .

# Tell uv sync to install python in a known location so we can copy it out later
ENV UV_PYTHON_INSTALL_DIR=/python

# Sync the project without its dev dependencies
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-editable --no-dev


# The runtime stage copies the built venv into a runtime container
FROM ubuntu:noble AS runtime

# Add apt-get system dependecies for runtime here if needed
# RUN apt-get update -y && apt-get install -y --no-install-recommends \
#     some-library \
#     && apt-get dist-clean

# Copy the python installation from the build stage
COPY --from=build /python /python

# Copy the environment, but not the source code
COPY --from=build /app/.venv /app/.venv

# Copy the robot data files needed at runtime
COPY --from=build /app/robots /app/robots

ENV PATH=/app/.venv/bin:$PATH

# change this entrypoint if it is not the same as the repo
ENV ROBOT_DIR=/app/robots/Meca500-R3
ENTRYPOINT ["robot-arm-sim"]
CMD ["sh", "-c", "robot-arm-sim simulate ${ROBOT_DIR}"]
