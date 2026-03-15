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
| `CLUSTER_MODE` | *(unset)* | Set to `1` to hide the Stop Simulator button (also auto-detected when `KUBERNETES_SERVICE_HOST` is set) |

The `/healthz` health-check endpoint is always available regardless of
`CLUSTER_MODE`.

## Using your own robot data

Mount a local directory of robot folders into the container and override
the default command:

```bash
$ docker run -p 8080:8080 \
    -v /path/to/my/robots:/data/robots \
    ghcr.io/gilesknap/robot-arm-sim:latest \
    simulate /data/robots
```

## Kubernetes deployment

For multi-user deployment on Kubernetes with Helm, see
{doc}`/how-to/deploy-kubernetes`.
