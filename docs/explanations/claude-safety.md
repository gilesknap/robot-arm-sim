# Securing Claude Code in Devcontainers

This document describes a practical security pattern for running Claude Code
autonomously in VS Code devcontainers. The goal is maximum autonomy with
minimum blast radius — Claude should be able to read, write, build, test, and
push code without constant permission prompts, while limiting the damage a
prompt injection could cause.

The pattern was developed for this project but is designed to be reusable
across any Claude-enabled repository.

```{contents}
:local:
:depth: 2
```

---

## Threat model

Claude Code operates with broad permissions to be productive. The primary
threat is **prompt injection** — malicious instructions embedded in content
Claude processes (GitHub issue bodies, fetched web pages, `CLAUDE.md` files in
cloned repos, dependency READMEs). A successful injection could instruct Claude
to:

- Exfiltrate code or credentials to an external server
- Push malicious code to repositories
- SSH into remote machines
- Modify its own permission rules to remove safety checks
- Install malicious packages

The key insight is that **Claude Code's permission rules (`.claude/settings.json`)
are enforced by the harness, not by the LLM**. Claude cannot "decide" to ignore
them. However, the pattern matching on Bash commands is fragile — a
sophisticated injection can craft commands that achieve the same effect while
evading the patterns. Therefore:

- **Permission rules** are useful as speed bumps for accidental misuse
- **Server-side enforcement** (GitHub branch protection, scoped tokens,
  container boundaries) is the real security boundary
- **Removing assets from the container** (SSH keys, broad tokens) eliminates
  entire classes of attack

---

## Layer 1: Devcontainer isolation

Claude Code must only run inside a devcontainer, and the container must not
have SSH keys available. A `UserPromptSubmit` hook in `.claude/settings.json`
checks both conditions on every prompt:

```json
"hooks": {
  "UserPromptSubmit": [{
    "hooks": [{
      "type": "command",
      "command": "if [ -z \"$REMOTE_CONTAINERS\" ]; then echo 'BLOCKED: ...'; exit 2; fi; if ssh-add -l 2>/dev/null; then echo 'BLOCKED: SSH agent is available...'; exit 2; fi"
    }]
  }]
}
```

The first check verifies the `$REMOTE_CONTAINERS` environment variable is
set (only present inside a VS Code devcontainer). The second check verifies
that `ssh-add -l` **fails** — if it succeeds, SSH keys are reachable and the
session is blocked. This catches cases where `SSH_AUTH_SOCK` was not cleared
or was re-enabled.

The devcontainer provides:

- **Write access limited to `/workspaces/`** — all repos, all git-backed
- **Read-only mounts** for organisation code that Claude should reference but
  not modify
- **No Docker socket** — Claude cannot escape to the host via Docker
- **No raw SSH keys** — see [Layer 4](#layer-4-git-network-operations)

---

## Layer 2: Permission rules in settings.json

`.claude/settings.json` defines three permission tiers:

### Allow (auto-execute)

```json
"allow": [
  "Read", "Edit", "Write", "Bash(*)",
  "WebSearch", "WebFetch(*)"
]
```

These run without prompting. `Bash(*)` is intentionally broad — restricting
it would cripple Claude's ability to build, test, and run code.

### Deny (blocked entirely)

```json
"deny": [
  "Edit(*.claude/settings.json)",
  "Write(*.claude/settings.json)",
  "Edit(*.claude/settings.local.json)",
  "Write(*.claude/settings.local.json)",
  "Bash(*settings.json*)",
  "Bash(*settings.local.json*)",
  "Bash(git push*)",
  "Bash(git fetch*)",
  "Bash(git pull*)",
  "Bash(git clone*)",
  "Bash(git remote*)",
  "Bash(git ls-remote*)"
]
```

**Settings file protection is critical.** Without deny rules on settings
files, a prompt injection could edit `.claude/settings.json` to remove all
`prompt` rules, then proceed unrestricted. The harness checks deny rules
*before* executing the tool call that would modify them, so this is a
bootstrap lock that cannot be self-removed.

**Git network commands are denied, not prompted.** VS Code's devcontainer
credential helper injects the host's broad GitHub OAuth token into every
container. Any `git push/fetch/pull` command authenticates with this token,
bypassing the scoped PAT in Layer 3. These commands are denied rather than
prompted because prompt rules offer an "always allow in this project" button
— users will inevitably click it, silently re-opening the OAuth token leak.
Claude should use `gh` CLI commands instead (see
[Layer 4](#layer-4-git-network-operations)), which authenticate with the
scoped fine-grained PAT.

### Prompt (require user confirmation)

```json
"prompt": [
  "Bash(git reset --hard*)",
  "Bash(ssh *)", "Bash(scp *)", "Bash(rsync *)", "Bash(sftp *)",
  "Bash(telnet *)", "Bash(mail *)", "Bash(sendmail *)",
  "Bash(wget --post* *)",
  "Bash(gh pr merge*)",
  "Bash(gh gist create*)",
  "Bash(gh api *)",
  "Bash(pip install*)",
  "Bash(npm install*)",
  "Bash(uv add*)"
]
```

These are **speed bumps, not security boundaries**. A sophisticated injection
can evade pattern matching (e.g., `bash -c "ssh ..."` bypasses
`Bash(ssh *)`). But they catch accidental misuse and naive injection attempts.

The rules gate:
- **Destructive git operations** — hard reset
- **Network escape** — SSH, SCP, mail, POST requests
- **Destructive GitHub operations** — merging PRs, creating gists, raw API calls
- **Supply chain** — installing arbitrary packages

---

## Layer 3: Scoped GitHub/GitLab tokens

The default `gh` OAuth token grants access to **all organisations and
repositories**. Replace it with a fine-grained Personal Access Token (PAT)
scoped to only the repositories Claude needs.

### GitHub fine-grained PAT setup

1. GitHub → Settings → Developer settings → Fine-grained personal access tokens
2. **Repository access:** "Only select repositories" — pick the repos for this
   devcontainer
3. **Permissions:**
   - Contents: Read and write
   - Pull requests: Read and write
   - Issues: Read and write
   - Metadata: Read (required)
   - **Do NOT grant:** Administration, Actions/workflow, or org-level permissions
4. **Expiration:** 30–90 days (rotate regularly)
5. Authenticate in the devcontainer: `gh auth login`

### GitLab PAT setup

1. GitLab → Preferences → Access Tokens
2. Scope to specific projects/groups
3. Authenticate: `git config --global credential.helper store` and use the
   token for HTTPS operations

### Persisting tokens across container rebuilds

Use a named Docker volume per devcontainer so you only authenticate once:

```json
"mounts": [
  "source=gh-auth-${localWorkspaceFolderBasename},target=/root/.config/gh,type=volume"
]
```

Each devcontainer gets its own volume with its own scoped PAT. A compromised
container can only access its own repository, not others.

---

(layer-4-git-network-operations)=

## Layer 4: Git network operations — use `gh`, not `git`

### The problem

VS Code devcontainers automatically inject an HTTPS credential helper into
the container's `~/.gitconfig`. This helper proxies the host's VS Code GitHub
OAuth token — the same broad token that VS Code uses for PRs, settings sync,
and other GitHub features. The injection happens at container start and cannot
be disabled via `devcontainer.json`.

This means that **any `git push`, `git fetch`, or `git pull` command
authenticates with the host's full GitHub OAuth token**, which has access to
all repositories and organisations the user can reach — completely bypassing
the fine-grained PAT set up in Layer 3.

SSH agent forwarding is also disabled (`SSH_AUTH_SOCK: ""`), but the VS Code
credential helper is the more insidious leak because it is invisible and
cannot be opted out of.

Overriding the credential helper via gitconfig or env vars is not a reliable
fix: any file-based config change can be reverted by an attacker from inside
the container.

### The compromise

Since the VS Code credential helper cannot be removed, we take a different
approach: **gate all network git operations behind prompt rules** and direct
Claude to use `gh` CLI commands instead.

The `gh` CLI authenticates with the fine-grained PAT from `gh auth login`
(Layer 3), not the VS Code credential helper. Key commands:

- **Push and create PR:** `gh pr create` (pushes the branch via the GitHub API)
- **View remote state:** `gh pr list`, `gh pr view`, `gh repo view`
- **Fetch PR content:** `gh pr checkout`

The deny rules in Layer 2 block `git push`, `git fetch`, `git pull`,
`git clone`, `git remote`, and `git ls-remote`. Deny was chosen over prompt
because prompt rules offer an "always allow" button that users will
inevitably click, silently re-opening the OAuth token leak. Deny rules
cannot be bypassed through the UI. The pattern matching itself is still
evadable by a sufficiently crafted bash command — this remains a **speed
bump, not a hard boundary** — but deny removes the most likely path to
accidental re-enablement.

### Why we removed the git HTTPS rewrite

Earlier versions of this configuration used `GIT_CONFIG_COUNT` env vars to
rewrite `git@github.com:` URLs to `https://github.com/`. This was intended to
force git through the scoped PAT. However, the VS Code credential helper
means HTTPS git operations still use the broad OAuth token, so the rewrite
provided no security benefit. Many organisations use `insteadOf` rules in
their host gitconfig to map HTTPS to SSH for normal development workflows;
the env var overrides conflicted with these. Removing them simplifies the
configuration and avoids confusing URL rewrite interactions.

### What remains

- **SSH agent forwarding is disabled** (`SSH_AUTH_SOCK: ""`) — SSH-based
  access is unavailable
- **`gh` CLI uses the scoped PAT** — all GitHub operations Claude should
  perform go through `gh`
- **`git` network commands require confirmation** — a speed bump that catches
  accidental use of the broad OAuth token

---

## Layer 5: Accepted risks

Some risks are accepted as pragmatic trade-offs:

### Network egress (data exfiltration)

The container has unrestricted internet access. A sophisticated injection
could exfiltrate code via `curl` or `WebFetch`. This risk is accepted because:

- The workspace contains only code, all backed up in git
- Read-only organisation mounts contain code only, no secrets
- SSH keys are not present (agent forwarding is disabled)
- GitHub/GitLab tokens are scoped and short-lived
- Network allowlisting would restrict the developer's interactive use of the
  container

### Bash pattern evasion

The `prompt` rules use pattern matching that can be bypassed by wrapping
commands in `bash -c "..."`, scripts, or other indirection. This is accepted
because:

- The deny rules on settings files prevent self-escalation
- Server-side enforcement (branch protection, scoped tokens) provides the
  real boundary
- The prompt rules still catch the majority of accidental and naive cases

---

## Complete devcontainer.json snippet

Combine all layers into your devcontainer configuration:

```json
{
  "remoteEnv": {
    "SSH_AUTH_SOCK": ""
  },
  "mounts": [
    "source=gh-auth-${localWorkspaceFolderBasename},target=/root/.config/gh,type=volume"
  ]
}
```

And the `.claude/settings.json` template is in this project's
`.claude/settings.json`.

---

## Summary

| Layer | What it protects against | Enforcement |
|-------|------------------------|-------------|
| Devcontainer | Host filesystem access | Container isolation |
| Deny rules on settings | Self-escalation of permissions | Harness (bootstrap lock) |
| Prompt rules | Accidental destructive commands | Harness (speed bump) |
| Scoped PAT | Access to unrelated repos/orgs | GitHub/GitLab server-side |
| No SSH agent | SSH to remote machines, git via SSH | Container config |
| `gh` over `git` for network ops | VS Code OAuth token leak | Harness (speed bump) + convention |
