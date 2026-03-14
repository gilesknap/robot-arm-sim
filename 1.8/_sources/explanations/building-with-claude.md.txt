# Building a Robot Simulator with Claude

This is the story of how robot-arm-sim was built from scratch using
[Claude Code](https://claude.com/claude-code) as a pair-programming partner.
It is aimed at anyone interested in learning how to use Claude to build
something ambitious — a multi-stage pipeline that analyses CAD meshes, reasons
about robot kinematics, generates industry-standard URDF files, and renders an
interactive 3D simulator in the browser.

The project went through several distinct phases: an initial design
conversation, a first implementation sprint, a difficult debugging cycle where
the generated robot didn't match reality, and a final visual-tuning phase
using reference photos and browser automation. Each phase taught different
lessons about how to collaborate with Claude effectively.

Every prompt shown below is **verbatim** — exactly what was typed into Claude
Code. The responses are summarised, but the key decisions and reasoning are
preserved.

```{contents}
:local:
:depth: 2
```

---

## First, Some General Principles

The specific files mentioned here are for Claude Code, but the same features are at least partially available in other harnesses such as Github Copilot.

1. Make your conversation interactive:
   - discuss your requirements, don't give solutions
   - ask for feedback and suggestions
   - ask if a task is feasible before committing to it
   - verify technical choices with Claude's knowledge
   - ask "can you think of anything else?" after the core design is settled
   - suggest that Claude interviews you to clarify requirements before writing code
1. Write the plan down before implementing:
   - Iterate in plan mode first
   - When ready, clear the context and start writing code — agent performance is best with clear planning and an empty context.
1. Create skills:
   - When you have spent effort working on a problem and have a solution, ask Claude to write a skill that captures the debugging methodology, not just the fix. This way you can reuse the same process for future problems.
   - See examples from this project in `.claude/skills/`
   - Skills are only loaded when the agent sees that they might be needed (or you exlicity invoke with e.g. /build-robot). So they aren't taking up context space all the time, but they are there when you need them.
1. Look after memory:
   - Claude Code has auto memory that is triggered on events like commits.
   - ask Claude to remember useful things at other time too.
   - its good to say remember useful tips from this context before clearing a context.
   - See the memo skill in this project which makes this easy
     - it remembers tips from the context
     - then it trims down memory to only currently useful things
     - it also moves relevant points up into the project's skills or CLAUDE.md for retention in the repository.
1. When Claude is guessing at something that can be computed, write code instead. e.g. for geometric problems where small errors can compound.
1. Manage permissions carefully.
   - giving more permission allows improved flow
   - but includes risks
   - this project uses quite permissive settings, but only runs in a devcontainer to mitigate risks. It won't run outside of a devcontainer and it asks for permission on commands that may escape from the container.
   - See `.claude/settings.json` in this project
1. Use `CLAUDE.md` for durable project instructions:
   - Checked into the repo, so every session and every contributor gets the same baseline.
   - Good for build commands, coding conventions, and constraints that Claude should always follow.
   - See this project's `CLAUDE.md` for an example.
   - Keep it less than 40 lines or so as it is loaded into context at the start of every session.
1. Give permission for effort and parallelism:
   - "Use as much effort as needed" and "make it multi-agent" unlock Claude's ability to work on multiple files simultaneously.
   - Without this, Claude tends to be conservative and sequential.

## Phase 1: The design conversation

The project started with nothing but a set of STL mesh files for a Meca500-R3
collaborative robot arm and a blank Python project scaffold. The first ten
prompts were a rapid-fire design conversation that shaped the entire
architecture before any code was written.

### Prompt 1 — Understanding the input

> **what is the best way for you to load stl files into context and understand
> the shapes they describe**

Claude explained that ASCII STL can be read as text, but binary STL (the
common format) needs parsing. The recommended approach: use `trimesh` to
extract geometry properties (bounding box, dimensions, volume, centre of mass)
and render multi-angle views to PNG for visual inspection. This set the
pattern for the whole project — give Claude structured data it can reason
about, not raw binary.

### Prompt 2 — Ambition check

> **I want you to be able to do a deep analysis of the models and infer how
> the parts fit together. Do you think you are up to that?**

This is a useful prompting pattern: **ask Claude whether a task is feasible
before committing to it**. Claude confirmed it could identify geometric
features (cylindrical surfaces for joints, flat mating faces, holes and
shafts) and analyse adjacency between sequentially-named parts to infer
assembly relationships. This gave confidence to proceed.

### Prompt 3 — Project structure

> **OK we have a python project here. I'd like to change argparse to typer and
> have some sub commands. The firs subcommand is to parse binary stl files (but
> leave open the possibilty of other formats). This parsing should create an
> output that is fully understandable to claude.**

Two important decisions here. First, the choice of Typer for the CLI — a
concrete technology steer from the human. Second, the key design insight:
**the analysis output should be "fully understandable to Claude"**. This meant
structured YAML with human-readable text descriptions, not just numeric data.
Every design choice downstream follows from this principle.

### Prompt 4 — The assembly reasoning skill

> **So now I want to write a claude skill that uses the above to reason about
> the parts it can see. I want the skill to infer how the parts fit together
> and where the pivot points are (to make a robot arm - this will always be the
> target, although we will want to add other robot descriptions later)**

A Claude "skill" is a markdown file that gives Claude specialised instructions
for a particular task. This prompt established the core architecture: **split
computation from reasoning**. Python code handles the precise geometry; a
Claude skill handles the semantic understanding of how parts form a robot.

### Prompt 5 — Output format

> **Next I want that skill to output a robot description that can be fed into
> a python based simulator. You mentioned a format for that earlier - is there
> a standard format that this could use?**

Claude recommended URDF (Unified Robot Description Format), the industry
standard supported by PyBullet, MuJoCo, Drake, and ROS. This was a case of
**letting Claude pick the standard** rather than prescribing one — Claude's
training data includes extensive robotics knowledge, so it was well-placed to
make this choice.

### Prompt 6 — The simulator

> **nice. lets write the intermediate files down in the robot folder. Then lets
> have an additional entrypoint into the python project 'simulate'. This will
> faithfully render the robot in 3d - with the render including the details
> from the original stl files. The GUI will also supply sliders that can rotate
> each of the joints of the robot so that the simulation is animated
> accordingly.**

This prompt defined the full pipeline in one sentence:
`analyze → skill → URDF → simulate`. The key decision was keeping
intermediate files in the robot folder so each stage is independently
re-runnable.

### Prompt 7 — GUI framework

> **OK I'm not aware of those so would need to take your steer. I'd quite like
> to use nicegui for the overall GUI so that we can make it beautiful and work
> in a browser. Do any of those libraries fit well into that framework?**

A good example of **combining a human preference with Claude's knowledge**.
The human wanted NiceGUI and a browser-based UI; Claude confirmed that
NiceGUI's `ui.scene` uses Three.js which loads STL natively, making it an
excellent fit. No heavy 3D libraries needed.

### Prompt 8 — Making it awesome

> **can you think of anything else that would make this project awesome, or
> improve on any of the choices above?**

Always worth asking. Claude proposed coordinate frame visualisation, joint
limit feedback, camera presets, inverse kinematics, collision highlighting,
and trajectory recording. Not all of these were implemented, but they shaped
the roadmap.

### Prompt 9 — Confirming the pipeline

> **That all sounds very nice - I definitely intended for the URDF bit to
> happen outside of sim.**

A short confirmation prompt. These are important — they prevent Claude from
going down the wrong path on a key architectural decision.

### Prompt 10 — Documentation and planning

> **one more thing. I'm hoping that this would be a nice demo of how to set up
> a project with claude (assuming it works!). Write a document /prompt.md
> before you start that lists the prompts above, plus a precis of your
> responses. And write the plan you are about to make into /plan.md. One final
> thing - you can use as much effort as needed to do a good job, make it multi
> agent if that helps.**

This prompt did three things: (1) created the documentation trail that makes
this story possible, (2) asked Claude to write a detailed implementation plan
before coding, and (3) gave explicit permission to use multi-agent execution
for parallelism. The resulting plan is reproduced in
{doc}`implementation-plan`.

### Prompt 11 — Web search fallback

> **lets add to the skill that it can search for manufacturers info online if
> the stl files are not enough to complete the task.**

A practical addition. STL files don't contain joint limits or DH parameters,
so the assembly-reasoning skill needed the ability to search for manufacturer
datasheets. This turned out to be essential for getting accurate joint limits.

### Prompt 12 — Package management

> **please use uv - there is already a venv and add all dependencies into
> pyproject.toml**

A concrete technology decision from the human. Short prompts like this are
effective when the intent is clear.

---

## Phase 2: First implementation

Claude executed the plan from `plan.md` using multi-agent parallelism. The
commit history tells the story:

```
a5d0904 Add analyze/simulate pipeline with Typer CLI (phases 1-4)
300c296 Add Meca500-R3 analysis output (YAML + renders)
6c736d5 Add Meca500-R3 URDF generated from analysis + manufacturer specs
```

In a single sweep, Claude implemented:

- The Typer CLI with `analyze` and `simulate` subcommands
- The mesh parser abstraction and STL parser using trimesh
- Geometric feature detection (flat faces, cylindrical surfaces)
- Off-screen rendering with pyrender and EGL
- YAML output with human-readable text descriptions
- The NiceGUI web application with Three.js 3D scene and joint sliders
- URDF loading, forward kinematics, and real-time joint control
- The assembly-reasoning skill

The simulator launched and showed a robot. But there was a problem.

---

## Phase 3: When the robot doesn't match reality

### Prompt 13 — The comparison that changed everything

> **we need to have another go at the urdf. See comparison between the sim
> using the urdf and a real image of the robot in a similar pose. There are
> still multiple issues. Can you think of a way to improve the
> assembly-reasoning skill to get perfect results.**

This was the turning point. When the simulator was compared against a photo of
the real Meca500-R3, parts were floating, disconnected, and misaligned. The
assembly-reasoning skill had been **guessing** 3D offsets from text
descriptions — and small errors compounded along the 6-joint chain.

Claude proposed the key architectural insight:

> **Connection point detection (finding bore/shaft centres) is a well-defined
> geometric computation that Python can do precisely. Kinematic chain reasoning
> (which parts connect, joint types, axes) requires semantic understanding that
> Claude does well. Split the problem accordingly.**

This led to plan 2, reproduced in {doc}`connection-point-plan`.

### Prompt 14 — Implementing the fix

> **Implement the following plan: Fix URDF Assembly via Programmatic Connection
> Point Detection.**

Claude implemented all seven steps from plan 2:

- A `ConnectionPoint` dataclass and detection module that slices cross-sections
  along cylindrical axes and fits circles to find bore centres
- A `urdf_generator.py` that reads `chain.yaml` plus analysis connection
  points to compute exact transforms
- A `generate` CLI subcommand
- A rewritten assembly-reasoning skill that outputs `chain.yaml` instead of
  raw URDF

The new pipeline:
```
analyze → connection_points → skill writes chain.yaml → generate → URDF → simulate
```

This was a significant lesson: **when Claude is guessing at something that can
be computed, write code to compute it instead**. The skill went from writing
raw URDF XML with hand-estimated offsets to writing a simple topology file,
while Python handled the precise geometry.

### Prompt 15 — Documentation

> **Update the readme.md to describe this project and give details of how to
> start the simulator and how to add a new robot.**

After a big implementation push, it's worth pausing to document. Claude
rewrote the README with the pipeline overview, installation instructions,
and a step-by-step guide for adding new robots.

---

## Phase 4: Visual tuning with reference images

Even with programmatic connection point detection, the robot still had visual
issues — meshes not quite aligned, gaps at joints, parts pointing the wrong
direction. This phase used a completely different approach: **iterative visual
comparison against photographs**.

### Prompt 16 — Creating the tuning skill

> **The URDF is still not quite right — still has a few issues with joint
> location and orientation. You mentioned a plan to pose the simulator and
> compare with images online. Can you prepare a new skill to do that and
> refine the URDF.**

Claude created the `visual-urdf-tuning` skill — a structured workflow for:

1. Searching for reference photos of the real robot
2. Launching the simulator and capturing screenshots
3. Comparing side-by-side to diagnose issues
4. Editing `chain.yaml`, regenerating, and iterating

This established a pattern: **when precision matters, create a skill that
encodes the debugging methodology**, not just the fix.

### Prompt 17 — Interactive debugging session

> **Load assembly-reasoning and visual-urdf-tuning skills, then use labels in
> the simulator to talk through where joint mapping is still incorrect and
> update the URDF accordingly.**

This was the longest and most interactive session. Claude used browser
automation to control the simulator, toggle labels, set camera angles, and
capture screenshots. Key discoveries:

- **Hot reload doesn't work for URDF changes** — the simulator process must be
  fully restarted. This was captured in the skill for future sessions.
- **Camera control via JavaScript** is safer than mouse dragging, which can
  accidentally move joint sliders. This led to the `control-simulator` skill.
- **Working base-to-tip is essential** — fixing a misalignment at joint 2
  automatically fixed apparent issues at joints 3, 4, and 5.

The session also created the `refine-with-image` skill — a comprehensive
10-step workflow for visual URDF refinement that any future session can follow.

### Prompt 18 — The wrist connection breakthrough

> **open a blank window in chrome**

> **/refine-with-image**

> **see image. keep iterating until you can match the pose and all looks
> perfect.**

The user provided a reference photo of the real Meca500-R3 held in someone's
hand. Claude analysed it, launched the simulator, matched the camera angle,
and started comparing.

The critical discovery: **A5 (the wrist) was connecting to the top of A3_4
instead of its front face**. The A3_4 part is L-shaped — it has a bore at the
bottom (for J3, Z-axis rotation) and a bore on its front face (for J4, X-axis
rotation). The URDF generator was using the Z-axis bore for both connections.

The user provided the key insight:

> **part a5 needs to be attached to the front of a3_4 and have a wrist
> rotation there. in the current view you can see a circle on a3_4 that
> defines where they should meet and the centre of rotation. The face of a5
> currently touching the top of a3_4 has a similar circle that should meet
> it.**

This led to three changes:

1. Move joint_4's origin to match A3_4's distal bore position (the front-face
   bore at `[62, 0, 76]mm` instead of the top at `[0, 0, 120]mm`)
2. Remove the `origin_rpy` frame rotation — instead of rotating the child
   frame and using Z-axis rotation, use `axis: [1, 0, 0]` directly
3. Remove manual `visual_xyz` shifts that were compensating for the wrong
   connection point

After these changes, the robot matched the reference photo.

---

## Lessons learned

### For prompting Claude on complex projects

1. **Start with a design conversation before writing code.** The first 10
   prompts wrote zero code but established the entire architecture. This
   prevented major rewrites later.

2. **Ask Claude if a task is feasible before committing.** Prompt 2 ("Do you
   think you are up to that?") gave Claude a chance to flag risks and set
   expectations.

3. **Give concrete technology steers where you have preferences** (Typer,
   NiceGUI, uv) **and let Claude choose where you don't** (URDF, trimesh).

4. **Ask "can you think of anything else?"** after the core design is
   settled. Claude's suggestions for camera presets, collision detection, and
   trajectory recording enriched the project roadmap.

5. **Write the plan down before implementing.** Prompt 10 asked for both
   `prompt.md` and `plan.md`. This created accountability and made it possible
   to review the plan before execution.

6. **Give permission for effort and parallelism.** "Use as much effort as
   needed, make it multi agent if that helps" unlocked Claude's ability to
   work on multiple files simultaneously.

### For splitting work between human and AI

7. **Split computation from reasoning.** The biggest improvement came from
   recognising that connection point detection is a computation (Python should
   do it) while kinematic chain topology is reasoning (Claude should do it).

8. **When Claude is guessing at something computable, write code instead.**
   The assembly-reasoning skill went from guessing xyz offsets to writing a
   simple topology file. Precision improved dramatically.

9. **Create skills that encode debugging methodology.** The `refine-with-image`
   and `visual-urdf-tuning` skills don't just fix one bug — they capture a
   repeatable process for future sessions.

10. **Human visual judgement is essential for visual tasks.** Claude can
    control the simulator, capture screenshots, and make programmatic
    comparisons, but the user's observation that "A5 needs to attach to the
    front of A3_4" was the breakthrough that no amount of automated analysis
    would have found.

### For iterative debugging

11. **Work base-to-tip.** Errors compound along a kinematic chain. Always fix
    the base link first.

12. **One change at a time.** Adjust one parameter, regenerate, compare. It's
    slower but you always know what fixed (or broke) things.

13. **Capture lessons in skills and memory**, not just in commit messages. A
    commit message is read once; a skill is used in every future session.

14. **Don't fight the tools.** When hot reload didn't work for URDF changes,
    the team didn't try to make it work — they documented the restart
    requirement in the skill and moved on.

---

## The full commit history

For reference, here is the complete sequence of commits that built the project:

```
d5f4b45 initial commit
831202c planning phase
a5d0904 Add analyze/simulate pipeline with Typer CLI (phases 1-4)
300c296 Add Meca500-R3 analysis output (YAML + renders)
6c736d5 Add Meca500-R3 URDF generated from analysis + manufacturer specs
60d1fad Fix simulator UI: slider events, lighting, layout, and camera
eab4327 2nd iteration on URDF, using a real image to sim compare
e264c9d Improve simulator UI: lighting, layout, grid, and slider labels
a1278db more prompting
288fb99 Add programmatic connection point detection and URDF generation pipeline
c8fb4fa Add visual-urdf-tuning skill for image-based URDF refinement
a5425f8 Update prompt.md with recent conversation history
85c2aa9 Visual URDF tuning: close mesh gaps and improve skill with simulator control docs
f2a9bce update permissions for autonomous refinement in devcontainer
fc0db34 Move skills into subfolders for slash command support
93fe00c Add callout labels, stop button, and multi-tab fixes to simulator
76805c5 Update project config: devcontainer hook, skill docs, workspace paths
e256f30 Visual upgrades: PBR metal materials, HDRI env map, lighter background, UI polish
f0d0da1 Visual joint alignment tuning, camera control and refine-with-image skills
8a90c50 fixing joint locations
99f0029 fixed joint 3
6fd9afb further visual refinement
397e7e9 Fix wrist connection: A5 attaches to front bore of A3_4
```

## Related documents

- {doc}`chrome-connector-workflow` — a detailed walkthrough of using the
  Chrome connector and camera skills to debug the end-effector
- {doc}`implementation-plan` — the original plan written before any code
- {doc}`connection-point-plan` — the plan that fixed the URDF assembly
- {doc}`architecture` — how the final system fits together
