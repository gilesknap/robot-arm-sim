# Running Claude Code Autonomously in Containers

This document describes how to run Claude Code in a fully autonomous mode
(no human in the loop) inside an isolated container. For the interactive
devcontainer pattern where a human reviews prompts, see
{doc}`claude-safety`.

```{contents}
:local:
:depth: 2
```

---

## Overview

When Claude runs unattended — in CI pipelines, scheduled tasks, or
long-running autonomous sessions — the permission prompt system cannot
provide safety because nobody is there to review. Instead, the
**container itself** becomes the security boundary:

- A dedicated throwaway container with only the credentials Claude needs
- Network access restricted to an allowlist of hosts
- `--dangerously-skip-permissions` enables full autonomy inside the sandbox

This is cleaner than layering security on top of a VS Code devcontainer,
which injects its own credential helpers and SSH agents that then need to
be suppressed.

---

## Container setup

Run Claude in a dedicated container with a scoped GitHub PAT as the only
credential. Use Podman (or Docker) to create an ephemeral environment:

```bash
podman run --rm \
  -e ANTHROPIC_API_KEY \
  -e GH_TOKEN=<scoped-pat> \
  -e HTTPS_PROXY=http://host.containers.internal:8888 \
  -e HTTP_PROXY=http://host.containers.internal:8888 \
  -e NO_PROXY=localhost,127.0.0.1 \
  -v $(pwd):/workspace:Z \
  <image> \
  claude --dangerously-skip-permissions -p "do the task"
```

Key properties:

- **Ephemeral** — `--rm` destroys the container after the task completes
- **No SSH agent** — no keys are mounted or forwarded
- **No VS Code credential helper** — this is not a devcontainer, so there
  is no OAuth token injection
- **Scoped PAT** — the `GH_TOKEN` is a fine-grained PAT with access only
  to the repositories Claude needs (see {doc}`claude-safety` Layer 3 for
  PAT setup)

---

## Restricting network access with a filtering proxy

`--network=none` would block all network access including the GitHub API,
so it is too restrictive. Instead, run a filtering forward proxy on the
host that only allows connections to specific domains.

### Why not iptables?

- GitHub's IP ranges change frequently — maintaining CIDR allowlists is
  fragile
- Rootless Podman cannot manipulate iptables without `--cap-add=NET_ADMIN`
- Domain-based filtering at the proxy layer is simpler and more
  maintainable

### tinyproxy setup

Install tinyproxy on the host and create a configuration that denies all
traffic by default, then allowlists specific domains.

**`/etc/tinyproxy/tinyproxy.conf`:**

```ini
Port 8888
Listen 127.0.0.1

# Default deny — only domains in the filter list are allowed
FilterDefaultDeny Yes
Filter "/etc/tinyproxy/allow.list"
```

**`/etc/tinyproxy/allow.list`** (regex patterns, one per line):

```text
^api\.github\.com$
^github\.com$
^uploads\.github\.com$
```

Add any other domains Claude needs (e.g., `^pypi\.org$`,
`^files\.pythonhosted\.org$` for pip installs).

Start the proxy:

```bash
tinyproxy -c /etc/tinyproxy/tinyproxy.conf
```

### Connecting the container to the proxy

Rootless Podman containers can reach the host via
`host.containers.internal`. The proxy environment variables in the
`podman run` command above route all HTTP/HTTPS traffic through tinyproxy:

```bash
-e HTTPS_PROXY=http://host.containers.internal:8888
-e HTTP_PROXY=http://host.containers.internal:8888
-e NO_PROXY=localhost,127.0.0.1
```

Any connection attempt to a domain not in the allowlist will be rejected
by tinyproxy.

---

## Comparison with the interactive pattern

| Aspect | Interactive devcontainer | Autonomous container |
|--------|------------------------|---------------------|
| Human in the loop | Yes — reviews prompts | No |
| Permission model | allow/deny/prompt rules | `--dangerously-skip-permissions` |
| Security boundary | Harness rules + scoped PAT | Container + network policy |
| VS Code integration | Yes (credential helper present) | No (standalone container) |
| Network restriction | None (accepted risk) | Proxy allowlist |
| Lifetime | Long-lived development session | Ephemeral per-task |

---

## Checklist

Before running Claude autonomously:

1. Fine-grained PAT created and scoped to required repositories only
2. tinyproxy running on host with domain allowlist
3. Container image includes `claude`, `gh`, and project dependencies
4. Branch protection rules enabled on the target repository
5. No SSH keys, Docker sockets, or broad tokens mounted into the container
