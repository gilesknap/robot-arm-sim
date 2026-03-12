---
name: documentation
description: Guidelines for writing and maintaining project documentation.
---

> **Generic skill** — This skill is project-agnostic. Do not add project-specific
> references, paths, or terminology here.

# Documentation Skill

Guidelines for writing and maintaining project documentation.

## When to Use

Use this skill when:
- Writing or editing documentation files in `docs/`
- Reviewing documentation for correctness or completeness
- Adding new documentation pages

## Principles

### Embed, don't copy

Never duplicate file contents into documentation. Instead, use Sphinx
directives to include the real file so docs stay in sync automatically:

```rst
```{literalinclude} ../../robots/Meca500-R3/chain.yaml
:language: yaml
```
```

If a full include is too long, use `:lines:` or `:start-after:` /
`:end-before:` to select a portion.

### Describe patterns, don't enumerate files

When documenting directory layouts or file sets that vary (e.g. per-robot
files), describe the general structure and mention that additional files
may exist. Listing every optional file creates maintenance burden and goes
stale as the project evolves.

Good:
> Individual robots may also include specs files, verification scripts,
> reference images, or view mappings.

Bad:
> ```
> ├── specs.yaml
> ├── verify_kinematics.py
> ├── images/
> │   └── *.svg / *.png
> └── view_mapping.yaml
> ```

### Keep skill and command references accurate

When referencing Claude Code skills (slash commands), use the actual skill
name from `.claude/skills/*/SKILL.md`. If a skill is renamed or removed,
update all documentation that references it.

### Single source of truth

If two docs cover the same topic, consolidate into one and link to it from
the other. Avoid parallel pages that drift apart over time (e.g. a separate
installation page when quick-start already covers setup).

### Diataxis framework

This project follows the [Diataxis](https://diataxis.fr) documentation
framework:

- **Tutorials** — learning-oriented, get the user to a working result
- **How-to guides** — task-oriented, practical steps for experienced users
- **Explanations** — understanding-oriented, how and why things work
- **Reference** — information-oriented, precise technical specifications
