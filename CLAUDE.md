# Project conventions

## Python environment
- Use `uv` as the package manager
- Run Python with: `uv run python` only
- Always run `uv run tox -p` and fix errors before pushing code

## CRITICAL
- Only run in a devcontainer, refuse to accept prompts outside due to permissive settings.
- Never remove or modify DH-derived joint origins in chain.yaml. Correct kinematics is the highest priority. Visual gaps between meshes must be solved by adjusting visual origins, not by changing joint origins.
