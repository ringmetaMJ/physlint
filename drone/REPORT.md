# physlint report: tensaur/drone

Target: https://github.com/varviotoni/drone, a mirror of tensaur/drone, commit `73aa519` (2026-09-09).

Status: the finding is reproduced from source, and a second reviewer re-derived it independently from the
public commit and the published brushless thrust curve, with numbers matching to two decimals. Filed as https://github.com/tensaur/drone/issues/35. We're waiting
on the authors.

Files read in full: `env/dronelib.h`, `env/drone.h`, `env/task_hover.h`, `env/task_race.h`, `env/binding.c`,
`env/drone.c`, `controller/src/out_of_tree_controller.c`, `controller/src/dronelib.h`, `config/drone.ini`,
`docs/summit-26.pdf`.

Method: read the code first, then run a physics-only harness (`harness.c`) against it. No RL and no policy
anywhere in this.

Reproduce with `cc -O2 -I<repo>/env -o harness harness.c -lm && ./harness`. It's seeded, so it's deterministic.

CONFIRMED below means reproduced numerically or by exact code path. REJECTED means we tested it and it doesn't happen.

## F1. CONFIRMED. Randomization is cancelled at the hover point by the action mapping

`compute_derivatives` maps action `a` to a target RPM:
`min_rpm(params) + (a+1)/2 * (max_rpm - min_rpm(params))`,
with `min_rpm = 2*rpm_hover(params) - max_rpm` and `rpm_hover = sqrt(m g / (4 k_thrust))`
(`dronelib.h:226-235, 266-272`). `params` holds the episode's randomized mass, thrust constant and gravity
(`init_drone`, `dronelib.h:237-251`, dr = 0.05 from `drone.h:87`). So action 0 gives exact hover thrust in
every episode, no matter what was drawn.

The firmware uses the same function with `dr = 0` (`out_of_tree_controller.c:26-30, 50`), which means the
nominal constants. On hardware, action 0 only hovers if the real airframe happens to match them.

Harness, 200,000 draws at dr = 0.05:
- sim: vertical acceleration at action 0 is 0.000 m/s^2 in every episode
- firmware mapping on the same randomized airframes: mean +0.09, sd 0.41, range [-0.93, +1.21] m/s^2

So the policy never sees the one offset it has to correct first on hardware. Randomizing mass, k_thrust and
gravity gives no training signal at all along the hover-offset axis. It still changes the dynamics away from
hover, thrust-to-weight and response for instance, so it isn't useless, but the slide-31 line "we don't know
these exactly, we don't need to" doesn't hold for the parameters that set the trim.

Either of two fixes restores the signal: map actions with the nominal constants in sim too, so the randomized
airframe shows up as a trim error the policy has to learn, or randomize the mapping constants separately from
the true ones. Both are one-line changes in `compute_derivatives`.

## F6. CONFIRMED with a measured curve. The constants are for a different airframe than the one flown

`dronelib.h:17-28` cites arplaboratory/learning-to-fly for its constants: 27 g, k_thrust 3.16e-10, max 21,702
RPM. That's a brushed Crazyflie 2.x. It gives a max modelled thrust of 0.595 N (60.7 gf), a thrust-to-weight of
2.25 and a 150 ms motor time constant. The hardware actually flown is a Crazyflie 2.1 Brushless (slide 6, and
`configure-firmware` defaults to `cf21bl`).

The brushless has since been measured properly by Graefe, Scherer, Hoenig and Trimpe in "How to Model Your
Crazyflie Brushless" (ICRA 2026, arXiv 2603.05944, code released): 44 g with guards, 40 g without, a 50 ms motor
time constant, a steady-state command gain of K = 2900 rad/s, and per-motor thrust against PWM duty ratio
fitted as the cubic `-0.2301 p^3 + 0.5618 p^2 - 0.0433 p` N.

That curve checks out against two other sources. At full PWM it gives 118 gf total, against about 120 gf on the
Bitcraze datasheet. At PWM 0.667 it gives 0.153 N per motor, against 0.145 N from the utiasDSL/drone-models
`cf21B_500` fit evaluated at K*0.667. So the Graefe cubic, the Bitcraze total at full throttle and the utiasDSL
fit at two-thirds throttle agree within 5% on the one number that matters here, which is thrust at the
firmware's action-0 command.

Here's what that means at action 0 on the real drone. The firmware sends PWM ratio 14476/21702 = 0.667
(`out_of_tree_controller.c:132-138`), which on the brushless is 0.611 N of total thrust.
- 34 g (Bitcraze, legs): action 0 climbs at +8.2 m/s^2. Hover is at PWM 0.481, action -0.560.
- 40 g (Graefe, no guards): +5.5 m/s^2. Hover at PWM 0.523, action -0.432.
- 44 g (Graefe, guards): +4.1 m/s^2. Hover at PWM 0.550, action -0.351.

So on the airframe that's actually flown, action 0 is a hard climb at half of gravity or more, and hover sits
somewhere between action -0.35 and -0.56. Per F1, the policy trained on a trim of exactly 0 in every episode,
and the ±5% randomization never got anywhere near the trim point either.

An earlier version of this report assumed RPM was linear in PWM up to the brushed max RPM and got 0.5 to
1.7 m/s^2 with an uncertain sign. That assumption turned out to be wrong by a factor of 3 to 5, so it's withdrawn.

The same paper backs this up from a different direction. They needed mass ±10% and thrust ±20% randomization
for transfer, and they report that "rotational dynamics are less sensitive to low domain randomization than
the vertical dynamics", which they put down to "slight discrepancies in mass and thrust between the model and
the real quadcopter". That's exactly the vertical trim axis F1 says this sim never exercises.

One caveat on the size. The curve is at nominal battery voltage, and the OOT controller skips battery
compensation (`out_of_tree_controller.c:126-138` writes motor ratios directly). A 20% thrust loss from sag still
leaves a clear climb at 34 g but brings the 44 g case close to trim. So the sign is solid for light
configurations; how big it is at the heaviest depends on charge.

One more thing in the same vein: motor lag is 50 ms measured against 150 ms in sim, so the real motors respond
about 3x faster than the policy was trained to expect. Not a trim issue, but the same class of mismatch.

## F2. CONFIRMED. Deliberate omission, matters by task. Zero drag, hard box clamp that removes energy

`BASE_B_DRAG = 0` and `BASE_K_ANG_DAMP = 0` (`dronelib.h:30-31`). The only speed limit is a per-axis clamp at
20 m/s after each RK4 substep (`move_drone`, `dronelib.h:367`). Speed can reach 20*sqrt(3) = 34.6 m/s.

In the harness, level full throttle hits the clamp in 1.81 s, in a grid that's only 10 m tall. Once there, the
clamp quietly removes about 6.6 W of kinetic energy per second (384 J over a pathological 60 s run).

In the hover task this never comes up, since the target is 5 m away and you're out of bounds at +1 m. In the
race task the rings span the whole 20 m grid, and a policy can use drag-free flight the real vehicle doesn't
have. For scale, the identified brushless model has linear drag of 0.0215 N.s/m in xy, which at 5 m/s is
0.107 N, a quarter of the 43.4 g airframe's weight, against zero in sim. Slide 30 says leaving drag out was
intentional, so the only thing to add is where it bites: race, and any approach faster than a few m/s. Not hover.

## F3. REJECTED. "Negative RPM feeds torque without thrust"

Thrust clamps rpm at 0 (`dronelib.h:279`). Torque uses the same clamped thrusts (`296-300`). Stored RPM is
clamped to [0, max] after every substep (`369-370`). Targets are always in [min_rpm, max_rpm], and the
first-order lag cannot overshoot. Harness: the minimum intermediate pre-clamp RPM over 2,000 bang-bang
rollouts is 8,584. Never negative. The guard at line 279 is dead code, not a bug.

## F4. NEGLIGIBLE. "Velocity exceeds the bound inside substeps"

It's true the clamp is applied after integration, but the largest possible change per 2 ms substep is
(T_max/m) * dt = 0.024 m/s. The harness worst case over 2,000 bang-bang rollouts started at the clamp was
0.045 m/s, or 0.23% of max_vel. Not worth fixing.

## F5. CONFIRMED by inspection. The committed firmware controller does not compile against the committed header

`controller/src/dronelib.h` is byte-identical to `env/dronelib.h`. The prototype is
`init_drone(Drone*, unsigned int*, float)`, unchanged since 2026-01-12. `out_of_tree_controller.c:50` calls
`init_drone(&drone, 0.0f)` with two arguments, and calling a prototyped C function with too few arguments is a
hard error. So either the hardware demos were built from an uncommitted tree, or the controller has been broken
since the task-interface refactor on 2026-05-23. I couldn't verify by building the firmware, no toolchain here.

## Not findings

- The rewards are potential-based: `alpha_dist*(prev_dist - dist)` and `alpha_shaping*(curr - prev)` both
  telescope, so oscillating toward the target earns nothing net. Any claim of "oscillation farming" is wrong.
- Randomizing mass and k_thrust independently at 5% is fine. They're independent in reality too.
- Gravity is randomized by 1% as a stand-in for mass, which is harmless. It is randomized unconditionally
  though (`dronelib.h:247`), so `init_drone(..., dr=0)` isn't deterministic, and the firmware, if it compiled,
  would boot with a random ±1% gravity baked into its mapping constants.

## What this says about the linter

Three of the five confirmed items aren't physics violations that a trajectory checker would ever catch.
They're contract mismatches: sim action mapping against firmware action mapping (F1), sim constants against
the airframe (F6), sim API against firmware API (F5). A linter that only watches states would miss the biggest
sim-to-real hole in this repo. So the first check worth building is a static one: does anything on the action
path depend on a randomized parameter the deployment side can't know? The trajectory checks (F2 to F4) come after.

Last caveat: none of this was tested with a trained policy or on hardware. It's what the code does.
