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

I ran it over every environment in PufferLib's `ocean/` directory on 2026-09-17, 85 in total, plus the
tensaur/drone repo the whole thing started from. It got the directory and nothing else.

| result | count | which |
|---|---|---|
| true positive | 2 | tensaur/drone `dronelib.h:271` and PufferLib's own `ocean/drone` at `physics.h:356`, both the bug in tensaur/drone#35 |
| false positive | 2 | both in `osrs`, a 2,100-function game sim, where the slice through a local reaches a boss spawn direction |
| clean | 83 | everything else, including `robot_arm`, `double_pendulum`, `cartpole`, `impulse_wars` |

Before the last two rules went in (randomization only counts on a reset path, test and benchmark files
are skipped) it produced 120 false positives, most of them from game randomness drawn during the step and
from a benchmark helper that fills the action buffer with noise. The true positive never moved.

Things to know before trusting it:

- MAPPING vs PLANT is decided by what the left-hand side is called. Look at the verbose output if a flag
  seems off.
- In a big monolithic step function the slice pulls in more than it should. That's where the two osrs
  false positives come from, and the robot arm came out clean because nothing there is hidden from the
  policy, not because the slice is tight.
- Only randomization on a reset or init path counts. If an environment draws its physical parameters
  somewhere with an unusual name, they'll be missed.
- "Observed" means some function that writes to `observations` or `obs` reads the field. If the buffer is
  called something else, or built in another file, it'll miss that.
- It only looks at the sim. It can tell you the mapping depends on a hidden parameter. Whether the firmware
  makes the same assumption you still have to check yourself, like I did for the drone.
- C only.
