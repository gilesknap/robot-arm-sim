# UR5 Reference Data

- Kinematics verified: all 6 joints 0mm error at DH zero config
- DH params: d1=89.159, a2=425, a3=392.25, d4=109.15, d5=94.65, d6=82.3 (mm)
- Additional offsets: shoulder_offset=135.85mm, elbow_offset=-119.7mm
- DH zero config = arm horizontal (+X), arm-up = J2 at -pi/2
- Needs origin_rpy=[0, pi/2, 0] at J2 and J4 for DH alpha rotations
- Joint axes: Z-Y-Y-Y-Z-Y (shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3)
- 7 STL files: base, shoulder, upperarm, forearm, wrist1, wrist2, wrist3 (collision meshes)
- Bore auto-detection unreliable on wrist parts: finds lateral motor bores, not kinematic bores — use Edit Bores mode for manual assignment
