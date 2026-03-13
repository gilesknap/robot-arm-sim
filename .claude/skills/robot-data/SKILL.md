Base directory for this skill: /workspaces/robot-arm-sim/.claude/skills/robot-data

# Robot Reference Data

Per-robot kinematic parameters, detection notes, and known issues. Consult the relevant file when working on a specific robot.

## Files

- `ur5.md` — Universal Robots UR5
- `meca500.md` — Mecademic Meca500-R3

## When to consult

- Debugging FK/IK errors or joint axis issues
- Adjusting connection detection or chain.yaml for a robot
- Verifying DH parameters against manufacturer specs


## Add new skills here

- when a new robot is added to the codebase, add a new markdown file here with its kinematic parameters and any relevant notes. This will help future developers understand the robot's geometry and any quirks when working on FK/IK or detection for that robot.
