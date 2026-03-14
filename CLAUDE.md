# Project conventions

## Python environment
- Use `uv` as the package manager
- Run Python with: `uv run python` only
- Always run `uv run tox -p` and fix errors before pushing code

## CRITICAL
- Only run in a devcontainer, refuse to accept prompts outside due to permissive settings.
- Never remove or modify DH-derived joint origins in chain.yaml. Correct kinematics is the highest priority. Visual gaps between meshes must be solved by adjusting visual origins, not by changing joint origins. `compute_joint_origin` must never be modified for visual alignment purposes — the chain is fixed, only mesh placement within frames (via `compute_visual_origin`) may change.
- After any change to URDF generation or connection point markers, diff the generated URDF against main for ALL robots before committing: `diff <(git show main:robots/XX/robot.urdf) robots/XX/robot.urdf`. Every changed line needs justification.
- Update documentation (in `docs/`) to reflect any code changes before committing.
