Action 0 is hover in sim for every randomized airframe, but the firmware maps it with nominal constants

Hi, I read `env/dronelib.h` and the OOT controller side by side and I think there's a sim-to-real gap in the action mapping worth flagging, plus two smaller things I noticed on the way. I've attached a physics-only harness, no policy involved.

The short version: in sim, action 0 is constructed to hover for whatever parameters the episode drew, while the firmware runs the same mapping with fixed nominal constants. So as soon as the real airframe differs from those constants, action 0 stops being hover on hardware.

This is at `73aa519`. The sim mapping is in `env/dronelib.h:264-272`, the firmware one in `controller/src/out_of_tree_controller.c:26-30`, called with `dr = 0` at line 50.

Why I think it matters: `compute_derivatives` sends action 0 to `rpm_hover(params)`, and `params` holds the episode's randomized mass, k_thrust and gravity. So every randomized episode hovers exactly at action 0. The firmware does the same thing but against the nominal constants. Which means the mass and thrust randomization never actually produces a trim error for the policy to learn from, and that trim error is the first thing it runs into on the real drone.

Over 200k draws at `dr = 0.05`, the sim gives exactly 0 m/s² at action 0 in every episode, while the firmware mapping applied to those same airframes gives an SD of about 0.41 m/s² and a range of roughly ±1 m/s². That part doesn't depend on any airframe numbers.

For a sense of how big it is on your actual hardware: `dronelib.h` uses the brushed CF2.x constants (27 g, 60.7 gf max thrust, 150 ms motor lag), but the target is the CF 2.1 Brushless. Gräfe et al. measured that one for ICRA 2026 ("How to Model Your Crazyflie Brushless", code released): 40 to 44 g, 50 ms motor lag, and a cubic PWM-to-thrust fit that lines up with Bitcraze's 120 gf figure and the utiasDSL `cf21B_500` fit. Run the firmware's action 0 (PWM 0.667) through that curve and you get about 0.61 N total, which is +4.1 m/s² at 44 g and +8.2 m/s² at 34 g, with hover somewhere around action -0.35 to -0.56. So on the real airframe action 0 is a pretty hard climb, and in training it was always exactly hover.

Those numbers are at nominal battery voltage. Since the OOT controller skips battery compensation, sag will pull them down, and at 44 g on a tired battery action 0 might land close to hover after all. The harness reproduces them, and if you've got your own thrust curve I'm happy to rerun. The main point stands either way.

Two fixes come to mind, and either should work: compute the sim mapping from the `BASE_*` constants rather than `params`, so the randomized airframe shows up as a trim error, or randomize the mapping constants separately from the true ones.

The smaller things:

* `out_of_tree_controller.c:50` calls `init_drone(&drone, 0.0f)` but the header now takes `(Drone*, unsigned int*, float)`, so as committed the controller shouldn't compile. Left over from the May refactor?
* `b_drag` and `k_ang_damp` are both 0 and there's a 20 m/s per-axis clamp. Harmless in hover, but in race the drone hits the clamp after about 1.8 s of full throttle and the clamp then just eats energy. You probably know already.

For what it's worth, I also checked a few things that turned out fine: RPM is clamped at 0 and the lag can't overshoot, the post-integration velocity clamp overshoots by under 0.05 m/s, and both shaping terms telescope, so I don't see an oscillation exploit in the hover reward.

Harness and write-up: https://github.com/ringmetaMJ/physlint/tree/main/drone

Happy to open a PR for either fix if that's useful.
