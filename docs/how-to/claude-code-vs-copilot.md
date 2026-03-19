# Migrating from Claude Code to GitHub Copilot

This guide covers how to transfer the AI-assisted development features in this
project's `.claude/` directory to GitHub Copilot, and the trade-offs between the
two harnesses.

## What Lives in `.claude/`

| Asset | Purpose |
|-------|---------|
| `CLAUDE.md` (repo root) | Project conventions, critical rules, safety constraints |
| `settings.json` | Permissions (allow/prompt lists), devcontainer enforcement hook |
| `settings.local.json` | Machine-specific permissions (uv, git, gh CLI) |
| 14 skill files across 8 skills | Domain-specific multi-step workflows |
| Memory system (`~/.claude/projects/.../memory/`) | Persistent cross-session knowledge |
| MCP integration (`claude-in-chrome`) | Browser automation for visual refinement |

## What Transfers Directly

### Skills (90% Compatible)

GitHub Copilot adopted the same `SKILL.md` format and explicitly supports the
`.claude/skills/` directory as a valid skill location. The existing skill
directory structure works as-is with Copilot CLI, Copilot coding agent, and
VS Code agent mode.

Minor adjustments needed:

- Add YAML frontmatter to skills that lack it. Copilot uses the `name` and
  `description` fields in the frontmatter to decide when to auto-load a skill.
  Skills without frontmatter (e.g. `control-simulator/SKILL.md`,
  `center-on-axis/SKILL.md`) should be updated.
- Multi-file skills (e.g. `make-robot/01-analyze-stls.md` through
  `04-generate-verify.md`) work the same way -- the `SKILL.md` references
  sub-files and the agent loads them when needed.

**References:**
- [GitHub Docs: Creating Agent Skills](https://docs.github.com/en/copilot/how-tos/use-copilot-agents/coding-agent/create-skills)
- [VS Code: Agent Skills](https://code.visualstudio.com/docs/copilot/customization/agent-skills)
- [GitHub Docs: About Agent Skills](https://docs.github.com/en/copilot/concepts/agents/about-agent-skills)

### Hooks (Structurally Compatible, Syntax Differs)

Copilot supports the same lifecycle events with the same concepts:

| Claude Code Event | Copilot Equivalent | Notes |
|---|---|---|
| `UserPromptSubmit` | `UserPromptSubmit` | Same name and semantics |
| `PreToolUse` | `PreToolUse` | Can allow/deny/modify tool calls |
| `PostToolUse` | `PostToolUse` | Can inject context or block |
| Exit code 2 = block | Exit code 2 = block | Same convention |

Copilot reads hooks from `.github/hooks/*.json` files (for team-shared hooks)
or `.claude/settings.json` (for compatibility). However, **tool name matchers
are parsed but not applied** -- so permission patterns like
`Bash(git push --force *)` will not enforce in Copilot. The devcontainer
enforcement hook works since it only uses exit codes.

**References:**
- [VS Code: Agent Hooks](https://code.visualstudio.com/docs/copilot/customization/hooks)
- [SmartScope: Copilot Hooks Guide](https://smartscope.blog/en/generative-ai/github-copilot/github-copilot-hooks-guide/)

### Instructions (Manual Copy)

`CLAUDE.md` maps to `.github/copilot-instructions.md`. Copy the content
directly; the format is identical (natural language in Markdown). Copilot also
supports path-scoped instructions via `.github/instructions/*.instructions.md`
with `applyTo` glob patterns.

**References:**
- [GitHub Docs: Custom Instructions](https://docs.github.com/copilot/customizing-copilot/adding-custom-instructions-for-github-copilot)

## What Needs Significant Adaptation

### Permissions Model -- No Equivalent

The fine-grained `allow`/`prompt` permission lists in `settings.json` (e.g.
allowing `Bash(uv run:*)` but prompting for `Bash(git push --force *)`) have
no direct Copilot equivalent:

- **Coding agent**: runs in a sandboxed cloud VM -- no local system access
- **VS Code agent mode**: uses VS Code's built-in workspace trust model
- **Copilot CLI**: has a simpler allow/deny model without glob-pattern
  granularity

Approximate this with `PreToolUse` hooks that inspect tool inputs and return
`permissionDecision: "deny"`.

### Memory System -- Different Architecture

| | Claude Code | Copilot |
|---|---|---|
| Storage | Local files you control | Cloud-hosted, managed by GitHub |
| Structure | Typed (user/feedback/project/reference) with frontmatter | Unstructured facts auto-discovered |
| Control | Full read/write/delete via file operations | View/delete via github.com settings |
| Scope | Per-project directory | Per-repository or per-user |
| Portability | Git-trackable, shareable | Locked to GitHub account |

The `memo` skill (which promotes learnings from memory into skills) has no
Copilot equivalent. Put those learnings directly into skills or
`copilot-instructions.md` instead.

**References:**
- [GitHub Docs: Copilot Memory](https://docs.github.com/en/copilot/how-tos/use-copilot-agents/copilot-memory)

### MCP Integrations -- Partially Available

The `claude-in-chrome` MCP server for browser automation is critical to the
`visual-refine` and `control-simulator` skills. Copilot in VS Code supports MCP
servers, but:

- The **coding agent** (cloud-based) has limited MCP support -- it runs in a
  sandboxed VM without access to your local browser
- **Copilot CLI** supports MCP via plugins, but `claude-in-chrome` specifically
  targets Claude Code's MCP protocol

Verify that any MCP server works with Copilot's MCP implementation before
relying on it.

### Sub-agent Orchestration

The `make-robot` skill explicitly launches parallel sub-agents (e.g.
`01-analyze-stls` + `02-research-specs` in parallel). Claude Code's `Agent`
tool with `subagent_type` is a first-class feature. Copilot has built-in
specialised agents (Explore, Task, Code Review, Plan) but custom sub-agent
orchestration within skills is less mature -- skills are instructions the agent
follows, not programmatic orchestration.

## Advantages and Disadvantages

### Claude Code Advantages

1. **Deeper agentic autonomy** -- Plans and executes multi-file, multi-step
   workflows with explicit sub-agent orchestration. The `make-robot` pipeline
   is a showcase for this.
2. **Transparent, file-based configuration** -- Everything is inspectable,
   version-controllable, and diffable.
3. **Fine-grained permissions** -- Glob-pattern tool permissions allow
   selective command approval.
4. **MCP ecosystem** -- 300+ integrations including browser automation,
   critical for visual refinement.
5. **1M token context** -- With Opus 4.6 the full context window is available.
6. **Terminal-native** -- No IDE dependency; works directly in devcontainers.

### GitHub Copilot Advantages

1. **Multi-model choice** -- Run Claude Opus 4.6, Sonnet 4.6, GPT-5.3-Codex,
   or Gemini 3 Pro. Pick the best model per task without switching tools.
2. **Async cloud agent** -- The coding agent runs in a cloud VM, opens PRs
   autonomously. Assign a GitHub issue to Copilot and walk away.
3. **IDE integration** -- Inline completions, chat panel, code review, and
   agent mode all in VS Code.
4. **Cost** -- Copilot Pro ($10/mo) or Pro+ ($39/mo) includes the coding agent
   and premium model requests. Claude Code Pro is $20/mo with usage caps.
5. **Team features** -- Organisation-wide policies, knowledge bases, and shared
   instructions via `.github/`.
6. **Native GitHub integration** -- Issues, PRs, code search, and repository
   context are first-class.
7. **Skills interoperability** -- `.claude/skills/` work in Copilot too, so
   skills are portable across both systems.

### Why Claude Code Is the Better Fit for This Project

This project relies heavily on:

- **MCP browser automation** for visual mesh alignment
- **Sub-agent orchestration** for the make-robot pipeline
- **Fine-grained permissions** enforcing devcontainer-only operation
- **Structured memory** for cross-session learning

These are Claude Code's unique strengths. A hybrid approach is viable: use
Copilot's coding agent for issue triage and routine PRs (it reads the shared
`.claude/skills/`), and Claude Code for interactive visual refinement and
robot-building pipelines.

## Migration Checklist

If migrating (or running both side by side):

1. Copy `CLAUDE.md` to `.github/copilot-instructions.md`
2. Add YAML frontmatter (`name`, `description`) to skills that lack it
3. Skills stay in `.claude/skills/` -- Copilot reads this directory natively
4. Convert hooks to `.github/hooks/*.json` format
5. Re-implement permissions as `PreToolUse` hook scripts
6. Export key memories into skills or instructions (memory is not portable)
7. Test `visual-refine` -- highest-risk workflow due to MCP dependency
8. Keep Claude Code available for browser automation workflows
