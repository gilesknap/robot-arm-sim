# 3. Keep connection point auto-detection despite low success rate

## Status

Accepted

## Context

The connection point auto-detection pipeline (bore centering, gap-closing, axis
snapping) currently works well on only a small fraction of joints across all
tested robots — roughly 3 joints on the UR5 and few if any on the newer robots
(Kinova Gen3, FANUC LR Mate 200iD, ABB IRB 2400). For the majority of joints
the auto-detected connections place parts incorrectly, and users must enter
Edit Connections mode to fix them manually or use the new Remove Connections
button to switch to pure `visual_xyz` placement.

Given this low hit rate, we considered removing the connection point and
auto-snapping code entirely and switching to a purely manual workflow where
humans position every part via `visual_xyz` offsets.

## Decision

We will keep the connection point auto-detection pipeline and all associated
UI (Edit Connections mode, centering modes, Save & Rebuild) alongside the new
Remove Connections escape hatch.

Two reasons:

1. **AI showcase.** This project serves as a demonstration of what AI-assisted
   tooling can build. The auto-detection pipeline — geometry classification,
   bore fitting, gap closure — is a meaningful example of algorithmic work
   generated and iterated on with Claude. Removing it would reduce the
   project's value as a showcase.

2. **Future visual-AI alignment.** We plan to train Claude to use visual
   analysis (via browser screenshots and Chrome automation) to achieve a much
   higher rate of automatic part alignment. The existing pipeline provides the
   infrastructure — connection point data model, URDF generation passes,
   Edit Connections UI — that a future visual-AI loop would build on. Keeping
   it now avoids having to re-create it later.

## Consequences

- The Remove Connections button provides an escape hatch for robots where
  auto-detection is unhelpful, so the low success rate does not block users.
- The codebase retains complexity (connection point models, multi-pass URDF
  generation, centering modes) that is not yet paying for itself on most
  robots. This is an intentional trade-off.
- Future work on visual-AI alignment can build on the existing pipeline
  rather than starting from scratch.
- We should track which robots and joints auto-detection works for, to
  measure improvement as the pipeline evolves.
