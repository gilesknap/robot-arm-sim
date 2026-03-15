[![CI](https://github.com/gilesknap/robot-arm-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/gilesknap/robot-arm-sim/actions/workflows/ci.yml)
[![Coverage](https://codecov.io/gh/gilesknap/robot-arm-sim/branch/main/graph/badge.svg)](https://codecov.io/gh/gilesknap/robot-arm-sim)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

# robot-arm-sim

Analyze robot arm STL meshes, generate URDF models, and simulate them interactively in the browser.

Source          | <https://github.com/gilesknap/robot-arm-sim>
:---:           | :---:
Docker          | `docker run ghcr.io/gilesknap/robot-arm-sim:latest`
Documentation   | <https://gilesknap.github.io/robot-arm-sim>
Releases        | <https://github.com/gilesknap/robot-arm-sim/releases>

## AI Coding Assistant Showcase

I built this project from concept to completion in 4 days assisted by Claude Code. I had no prior experience of robots, URDF, or 3D graphics programming.

For some details of how I use Claude and what prompts got this project started, see [Building a Robot Simulator with Claude](https://gilesknap.github.io/robot-arm-sim/main/explanations/building-with-claude.html)).

## Pipeline

```
STL files → analyze → chain.yaml → generate → URDF → simulate
```

## Quick start

```bash
git clone https://github.com/gilesknap/robot-arm-sim.git
cd robot-arm-sim
uv sync
uv run robot-arm-sim simulate robots/Meca500-R3/
```

Opens a browser at `http://localhost:8080` with a 3D model and joint sliders.)

![simulator image](https://raw.githubusercontent.com/gilesknap/robot-arm-sim/main/docs/images/simulator.png)

<!-- README only content. Anything below this line won't be included in index.md -->


See <https://gilesknap.github.io/robot-arm-sim> for full documentation.
