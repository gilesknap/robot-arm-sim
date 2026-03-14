# CLI reference

robot-arm-sim provides three subcommands corresponding to the three pipeline
stages. Run any command with `--help` for built-in usage.

## Global options

```
robot-arm-sim [OPTIONS] COMMAND [ARGS]
```

| Option | Description |
|--------|-------------|
| `--version`, `-v` | Show version and exit |
| `--install-completion` | Install shell completion for the current shell |
| `--show-completion` | Show shell completion script |
| `--help` | Show help and exit |

---

## `analyze`

Analyze STL files and generate per-part YAML analysis + renders.

```bash
$ robot-arm-sim analyze ROBOT_DIR [--override-manual]
```

| Argument / Option | Description |
|-------------------|-------------|
| `ROBOT_DIR` | Path to robot directory. Must contain a `stl_files/` subdirectory with one STL file per link. |
| `--override-manual` | Re-detect all connection points, discarding manual placements. |

**Output:** creates an `analysis/` directory inside `ROBOT_DIR` with:

- One YAML file per part (geometry, connection points, features)
- A `summary.yaml` listing all parts with role hints
- Multi-angle renders of each part

**Example:**

```bash
$ robot-arm-sim analyze robots/Meca500-R3/
```

---

## `generate`

Generate a URDF file from a kinematic chain specification and analysis data.

```bash
$ robot-arm-sim generate ROBOT_DIR CHAIN_FILE [--output PATH]
```

| Argument / Option | Description |
|-------------------|-------------|
| `ROBOT_DIR` | Path to robot directory. Must contain an `analysis/` subdirectory. |
| `CHAIN_FILE` | Path to the `chain.yaml` kinematic chain specification. |
| `--output PATH` | Output URDF path. Default: `<ROBOT_DIR>/robot.urdf`. |

**Example:**

```bash
$ robot-arm-sim generate robots/Meca500-R3/ robots/Meca500-R3/chain.yaml
```

```bash
$ robot-arm-sim generate robots/UR5/ robots/UR5/chain.yaml --output /tmp/ur5.urdf
```

---

## `simulate`

Launch the interactive 3D simulator in the browser.

```bash
$ robot-arm-sim simulate ROBOTS_DIR [--port PORT]
```

| Argument / Option | Description |
|-------------------|-------------|
| `ROBOTS_DIR` | Directory containing one or more robot folders (each with a `robot.urdf`). |
| `--port PORT` | Port for the web UI. Default: `8080`. |

The simulator auto-discovers all robots under `ROBOTS_DIR` that contain a
`robot.urdf` file. If multiple robots are found, a dropdown selector appears
in the UI.

To choose which robot is selected by default, create a `.default` file in
`ROBOTS_DIR` containing the robot folder name (e.g. `Meca500-R3`). If the
file is absent or names an unknown robot, the first robot alphabetically is
used.

**Example:**

```bash
$ robot-arm-sim simulate robots/Meca500-R3/
```

```bash
$ robot-arm-sim simulate robots/ --port 9090
```

The simulator serves the web UI at `http://localhost:<PORT>` and opens the
browser automatically. STL files are served at `/stl/<robot_name>/` paths.
A health-check endpoint is available at `/healthz`.
