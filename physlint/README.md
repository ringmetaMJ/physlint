# contract_check.py

This started as a hand-written test for one bug in tensaur/drone and turned into a check I could point at
any C environment. The question it asks is simple: when the policy's action gets turned into a motor
command, does that conversion use a parameter that was randomized at reset and that the policy can't see?
In the drone that was the hover RPM, recomputed every episode from the randomized mass and thrust constant,
so action 0 always meant "hover" in sim no matter what the sim had drawn. The firmware uses fixed constants,
so on the real drone it doesn't.

It uses libclang. It finds fields that get assigned from a random call, follows them through simple copies,
throws out anything that ends up in the observation buffer, then traces the action through each function
that touches it and reports any command line that depends on what's left. Lines that compute state or
derivatives get labelled PLANT and skipped, bare calls into an integrator get labelled CALL and skipped.
You get file, line, the parameter names, and the function they were read in.

    python3 physlint/contract_check.py <dir> [--entry file.h] [-I dir] [-Dmacro=value] [--verbose]

You'll need `pip install libclang`. `--entry` is for directories with more than one root file, `-D` is for
things like CUDA qualifiers, `--verbose` also shows the PLANT and CALL lines.

I ran it on eight environments on 2026-09-17, giving it only the directory:

| target | hidden randomized fields | flags | found |
|---|---|---|---|
| tensaur/drone `73aa519` | 13 | 1 | `dronelib.h:271`, target RPM depends on mass, k_thrust, gravity via `rpm_hover` |
| PufferLib 5.0 `ocean/drone` | 27 | 1 | `physics.h:356`, same thing through the vectorized `Paramsv` copy |
| `ocean/robot_arm` | 0 | 0 | randomizes object pose and joint config, but the policy sees both |
| `ocean/double_pendulum` | 0 | 0 | randomizes initial state, observed |
| `ocean/cartpole` | 0 | 0 | same |
| `ocean/whisker_racer` | 1 | 0 | randomizes the track, nothing on the action path |
| `ocean/matsci` | 0 | 0 | observed |
| `ocean/squared_continuous` | 0 | 0 | nothing randomized |

So two hits, both the bug from tensaur/drone#35, one of them in the 5.0 env the maintainer is porting to.
Six clean.

Things to know before trusting it:

- MAPPING vs PLANT is decided by what the left-hand side is called. Look at the verbose output if a flag
  seems off.
- In a big monolithic step function the slice pulls in more than it should. The robot arm came out clean
  because nothing there is hidden from the policy, not because the slice is tight.
- "Observed" means some function that writes to `observations` or `obs` reads the field. If the buffer is
  called something else, or built in another file, it'll miss that.
- It only looks at the sim. It can tell you the mapping depends on a hidden parameter. Whether the firmware
  makes the same assumption you still have to check yourself, like I did for the drone.
- C only.
