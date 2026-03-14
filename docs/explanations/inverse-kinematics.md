# Inverse kinematics solver

How the custom damped least-squares IK solver works, why it was built, and
how it relates to the rest of the simulator.

## Why a custom solver

The original implementation used [ikpy](https://github.com/Phylliade/ikpy),
a pure-Python IK library. It was replaced with a custom solver for two
reasons:

1. **URDF integration** — the solver reads joint origins, axes, and limits
   directly from the `URDFRobot` model, avoiding the need to re-parse the
   URDF into ikpy's chain format and maintain two kinematic representations.
2. **Control** — a purpose-built solver allows tuning convergence parameters
   (damping, tolerances, iteration count) and handling 6-DOF position +
   orientation targets without wrapper code.

The solver lives in `src/robot_arm_sim/simulate/ik_solver.py`.

## Algorithm: damped least-squares

The solver uses the **damped least-squares** (DLS) method, also known as the
Levenberg–Marquardt method applied to IK.

### Problem statement

Given a target end-effector pose (4×4 homogeneous transform) and the current
joint angles, find joint angle updates **dq** that drive the end-effector
toward the target.

### Iteration

On each iteration the solver:

1. **Evaluates FK** to get the current end-effector pose and all link
   transforms.
2. **Computes the 6-vector error**:
   - Position error: `e_pos = p_target - p_current` (3 elements, metres)
   - Orientation error: `e_rot = axis_angle(R_target × R_current^T)`
     (3 elements, radians)
3. **Builds the 6×N geometric Jacobian** analytically from the URDF joint
   chain (see below).
4. **Solves for joint updates** using the DLS formula:

   ```
   dq = J^T (J J^T + λ²I)^{-1} e
   ```

   where λ is the damping factor.
5. **Updates joint angles** and clamps to joint limits.
6. **Checks convergence** — if position error < `pos_tol` and rotation
   error < `rot_tol`, returns the solution.

If the solver does not converge within `max_iter` iterations, it returns
`None`.

### Default parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_iter` | 100 | Maximum iterations |
| `pos_tol` | 1e-4 m (0.1 mm) | Position convergence threshold |
| `rot_tol` | 1e-3 rad (~0.057°) | Orientation convergence threshold |
| `damping` | 0.05 | Damping factor λ |

## Geometric Jacobian construction

The Jacobian `J` is a 6×N matrix where N is the number of active
(revolute or continuous) joints. For each joint *i*:

- **z_i** — the joint axis in world coordinates, computed by transforming
  the URDF axis vector through the parent link transform and joint origin.
- **p_i** — the joint pivot position in world coordinates.
- **p_ee** — the end-effector position.

The Jacobian columns are:

```
J[:3, i] = z_i × (p_ee - p_i)    (linear velocity contribution)
J[3:, i] = z_i                    (angular velocity contribution)
```

This is the standard geometric Jacobian for revolute joints.

## The role of damping

The undamped pseudo-inverse `J^T (J J^T)^{-1}` becomes numerically unstable
near **kinematic singularities** — configurations where the robot loses a
degree of freedom (e.g. fully extended arm, aligned wrist axes).

Adding λ²I to the denominator ensures the matrix is always invertible at
the cost of slightly less accurate steps. The default λ = 0.05 provides a
good balance: convergence is barely affected in normal configurations, but
the solver remains stable through singular regions.

## Convergence behaviour

For typical 6-DOF robots (Meca500-R3, UR5) reaching targets within their
workspace, the solver converges in 5–30 iterations. Targets near workspace
boundaries or requiring large joint-angle changes from the initial guess may
take more iterations or fail to converge.

The solver is called on every IK slider change in the simulator UI, so
the current joint angles always serve as a nearby initial guess, giving
fast incremental convergence as the user drags sliders.

## Orientation error computation

The orientation error is extracted from the rotation error matrix
`R_err = R_target × R_current^T` using the axis-angle decomposition:

- **Small-angle case** (trace > 3 - ε): uses the linear approximation
  from the skew-symmetric part, avoiding numerical issues with `arccos`.
- **General case**: computes `angle = arccos((trace - 1) / 2)` and extracts
  the axis from the skew-symmetric components.

## Relationship to FK

The IK solver evaluates forward kinematics internally on every iteration
to compute both the Jacobian (which needs all link transforms) and the
current end-effector pose (to measure error). A correct URDF — with joint
origins derived from DH parameters — is therefore essential for accurate IK.

See {doc}`/explanations/kinematics_and_urdf_generation` for how DH
parameters, connection geometry, and FK interact.
