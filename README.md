# physlint

A tool for checking that a simulator and the code deployed from it actually agree, and that a policy
isn't leaning on something the simulator gets wrong. Right now it's stage 1: one environment, one harness,
one report. There's no framework yet, on purpose.

Status: the finding is reproduced from source, and a second reviewer re-derived it independently from
the public commit and the published thrust curve. Filed as tensaur/drone#35 (https://github.com/tensaur/drone/issues/35). We're waiting on the authors.

What's here:

- `drone/harness.c`, `drone/REPORT.md`, `drone/ISSUE_AS_POSTED.md`, for tensaur/drone, the PufferLib
  first-party drone environment.

The checks fall into three families, based on what the first target taught us:

1. Contract checks (static). Does anything on the action path depend on a randomized parameter that
   the deployed side can't know? Do the sim constants match the airframe that's actually flown? Does the
   deployed code even compile against the sim header it's supposed to share?
2. Trajectory checks (dynamic). Actuator limits, energy budget, clamps that quietly remove energy,
   integrator artefacts, contact impulse limits.
3. Reward checks. Penalty terms that go positive, shaping that doesn't telescope, per-step pay after
   arriving early at the goal, rewards that depend on being in a bad state.

The one rule we took away from stage 1: check the contract before the trajectory. In the first target,
the failures that mattered were mismatches between sim and deployment, not physics errors.

Every finding comes with a file and line, a number, and a way to reproduce it.

Parts of the reading and drafting are delegated to agents. The harness runs and the numbers are checked by hand.
