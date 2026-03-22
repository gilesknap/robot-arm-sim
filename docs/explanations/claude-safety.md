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

Claude Code must only run inside a devcontainer. A `UserPromptSubmit` hook
in `.claude/settings.json` enforces this:

```json
"hooks": {
  "UserPromptSubmit": [{
    "hooks": [{
      "type": "command",
      "command": "if [ -z \"$REMOTE_CONTAINERS\" ]; then echo 'BLOCKED: ...'; exit 2; fi"
    }]
  }]
}
```

The devcontainer provides:

- **Write access limited to `/workspaces/`** — all repos, all git-backed
- **Read-only mounts** for organisation code that Claude should reference but
  not modify
- **No Docker socket** — Claude cannot escape to the host via Docker
- **No raw SSH keys** — see [Layer 4](#layer-4-ssh-agent-and-git-transport)

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
  "Bash(*settings.local.json*)"
]
```

**This is critical.** Without deny rules on settings files, a prompt injection
could edit `.claude/settings.json` to remove all `prompt` rules, then proceed
unrestricted. The harness checks deny rules *before* executing the tool call
that would modify them, so this is a bootstrap lock that cannot be
self-removed.

### Prompt (require user confirmation)

```json
"prompt": [
  "Bash(git push --force *)",
  "Bash(git push -f *)",
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
- **Destructive git operations** — force push, hard reset
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

## Layer 4: SSH agent and git transport

### The problem

VS Code automatically forwards the host's SSH agent into devcontainers. This
means Claude can use your SSH keys to:

- Push to **any** git repo your keys have access to (bypassing PAT scoping)
- SSH into **any** remote machine your keys are authorised on

The keys themselves aren't in the container (agent forwarding means the key
material stays on the host), so they can't be exfiltrated. But they can be
**used live** during the session.

### The fix

Disable SSH agent forwarding and force git to use HTTPS:

```json
"remoteEnv": {
  "SSH_AUTH_SOCK": ""
},
"postCreateCommand": "git config --global url.'https://github.com/'.insteadOf 'git@github.com:' && git config --global url.'https://gitlab.diamond.ac.uk/'.insteadOf 'git@gitlab.diamond.ac.uk:'"
```

This ensures:
- **No SSH agent** in the container — SSH is simply unavailable
- **Git uses HTTPS** — authenticated by the scoped PAT, not SSH keys
- **You can still SSH from your host terminal** when needed — outside Claude's
  reach

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
  ],
  "postCreateCommand": "git config --global url.'https://github.com/'.insteadOf 'git@github.com:' && git config --global url.'https://gitlab.diamond.ac.uk/'.insteadOf 'git@gitlab.diamond.ac.uk:'"
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
| Git HTTPS rewrite | Bypassing PAT via SSH git remotes | Git config |
