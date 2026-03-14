# Deploy on Kubernetes

This guide covers deploying robot-arm-sim on Kubernetes using the included
Helm chart for multi-user access.

## Prerequisites

- A Kubernetes cluster (any provider — local k3s, EKS, GKE, etc.)
- [Helm](https://helm.sh/docs/intro/install/) 3.x
- `kubectl` configured for your cluster

## Install with Helm

The Helm chart is in the `helm/robot-arm-sim/` directory of the repository.

```bash
$ git clone https://github.com/gilesknap/robot-arm-sim.git
$ helm install robot-sim ./robot-arm-sim/helm/robot-arm-sim
```

This creates a Deployment, a Service, an optional Ingress, and configures
health checks.

## Configuration

Override defaults by passing `--set` flags or a custom values file:

```bash
$ helm install robot-sim ./helm/robot-arm-sim \
    --set image.tag=0.1.0 \
    --set replicaCount=3 \
    --set service.type=LoadBalancer
```

Or with a values file:

```bash
$ helm install robot-sim ./helm/robot-arm-sim -f my-values.yaml
```

### values.yaml reference

| Key | Default | Description |
|-----|---------|-------------|
| `replicaCount` | `1` | Number of pod replicas |
| `image.repository` | `ghcr.io/gilesknap/robot-arm-sim` | Container image |
| `image.tag` | `latest` | Image tag |
| `image.pullPolicy` | `Always` | Kubernetes pull policy |
| `service.type` | `ClusterIP` | Service type (`ClusterIP`, `LoadBalancer`, `NodePort`) |
| `service.port` | `80` | External service port |
| `ingress.enabled` | `true` | Enable Ingress resource |
| `ingress.className` | `nginx` | Ingress class name |
| `ingress.host` | `claude-robots.diamond.ac.uk` | Ingress hostname |
| `ingress.tls` | `true` | Enable TLS on Ingress |
| `containerPort` | `8080` | Port the simulator listens on inside the container |
| `resources.requests.cpu` | `250m` | CPU request |
| `resources.requests.memory` | `256Mi` | Memory request |
| `resources.limits.cpu` | `2` | CPU limit |
| `resources.limits.memory` | `1Gi` | Memory limit |

## Cluster mode

The Helm chart automatically sets the `CLUSTER_MODE=1` environment variable.
In cluster mode:

- The **Stop Simulator** button is hidden to prevent accidental shutdown of a
  shared instance.
- The `/healthz` endpoint is available for liveness and readiness probes.

Cluster mode is also activated automatically when the `KUBERNETES_SERVICE_HOST`
environment variable is present (set by Kubernetes in every pod).

## Health checks

The deployment includes both probes against the `/healthz` endpoint:

| Probe | Path | Initial delay | Period |
|-------|------|---------------|--------|
| Liveness | `/healthz` | 10 s | 30 s |
| Readiness | `/healthz` | 5 s | 10 s |

## Resource tuning

The default resource limits work well for a single robot with moderate
mesh complexity. For robots with high-polygon STL files or many concurrent
users, increase the memory limit:

```yaml
resources:
  requests:
    cpu: 500m
    memory: 512Mi
  limits:
    cpu: "4"
    memory: 2Gi
```

## Accessing the simulator

Once deployed, get the external URL:

```bash
$ kubectl get svc robot-sim
```

For `ClusterIP` services, use `kubectl port-forward`:

```bash
$ kubectl port-forward svc/robot-sim 8080:80
```

Then open `http://localhost:8080`.

If Ingress is enabled, access the simulator at the configured hostname
(e.g. `https://claude-robots.diamond.ac.uk`).

## Uninstalling

```bash
$ helm uninstall robot-sim
```
