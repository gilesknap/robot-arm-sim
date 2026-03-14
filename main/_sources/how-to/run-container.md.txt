# Run in a container

Pre-built containers with robot-arm-sim and its dependencies already
installed are available on [Github Container Registry](https://ghcr.io/gilesknap/robot-arm-sim).

## Starting the simulator

To pull the container and launch the simulator with the built-in robots:

```bash
$ docker run -p 8080:8080 ghcr.io/gilesknap/robot-arm-sim:latest
```

Open `http://localhost:8080` in your browser to access the simulator.

To get a released version, use a numbered tag instead of `latest`:

```bash
$ docker run -p 8080:8080 ghcr.io/gilesknap/robot-arm-sim:0.1.0
```

## Checking the version

```bash
$ docker run ghcr.io/gilesknap/robot-arm-sim:latest --version
```

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_DIR` | `/app/robots` | Path to the directory containing robot folders inside the container |
| `CLUSTER_MODE` | *(unset)* | Set to `1` to enable cluster mode (disables the Stop button, enables health checks) |

## Using your own robot data

Mount a local directory of robot folders into the container and override
`ROBOT_DIR`:

```bash
$ docker run -p 8080:8080 \
    -v /path/to/my/robots:/data/robots \
    -e ROBOT_DIR=/data/robots \
    ghcr.io/gilesknap/robot-arm-sim:latest
```

## Kubernetes deployment

For multi-user deployment on Kubernetes with Helm, see
{doc}`/how-to/deploy-kubernetes`.
