// physlint stage-1 harness for tensaur/drone env/dronelib.h
// Reproduces four candidate findings numerically. No RL, no policy: physics only.
#include <stdio.h>
#include <string.h>
#include "dronelib.h"

// --- helper: hover thrust accel error when action==0 is interpreted with OTHER params ---
static float accel_at_action0(const Params* map_p, const Params* true_p) {
    // rpm commanded for action 0 under mapping params (what the firmware does with BASE constants)
    float rpm0 = rpm_hover(map_p);
    // thrust produced by TRUE airframe at that rpm, minus true weight
    float T = 4.0f * true_p->k_thrust * rpm0 * rpm0;
    return T / true_p->mass - true_p->gravity;  // m/s^2, 0 means hover
}

// --- instrumented RK4 to observe intermediate (pre-clamp) states ---
static float g_min_tmp_rpm = 1e9f, g_max_tmp_vel = 0.f;
static void rk4_step_instrumented(State* state, Params* params, float* actions, float dt) {
    StateDerivative k1, k2, k3, k4; State tmp;
    compute_derivatives(state, params, actions, &k1);
    step(state, &k1, dt*0.5f, &tmp);
    for (int i=0;i<4;i++) if (tmp.rpms[i] < g_min_tmp_rpm) g_min_tmp_rpm = tmp.rpms[i];
    compute_derivatives(&tmp, params, actions, &k2);
    step(state, &k2, dt*0.5f, &tmp);
    for (int i=0;i<4;i++) if (tmp.rpms[i] < g_min_tmp_rpm) g_min_tmp_rpm = tmp.rpms[i];
    compute_derivatives(&tmp, params, actions, &k3);
    step(state, &k3, dt, &tmp);
    for (int i=0;i<4;i++) if (tmp.rpms[i] < g_min_tmp_rpm) g_min_tmp_rpm = tmp.rpms[i];
    compute_derivatives(&tmp, params, actions, &k4);
    float dt6 = dt/6.0f;
    state->pos.x += (k1.vel.x+2*k2.vel.x+2*k3.vel.x+k4.vel.x)*dt6;
    state->pos.y += (k1.vel.y+2*k2.vel.y+2*k3.vel.y+k4.vel.y)*dt6;
    state->pos.z += (k1.vel.z+2*k2.vel.z+2*k3.vel.z+k4.vel.z)*dt6;
    state->vel.x += (k1.v_dot.x+2*k2.v_dot.x+2*k3.v_dot.x+k4.v_dot.x)*dt6;
    state->vel.y += (k1.v_dot.y+2*k2.v_dot.y+2*k3.v_dot.y+k4.v_dot.y)*dt6;
    state->vel.z += (k1.v_dot.z+2*k2.v_dot.z+2*k3.v_dot.z+k4.v_dot.z)*dt6;
    state->quat.w += (k1.q_dot.w+2*k2.q_dot.w+2*k3.q_dot.w+k4.q_dot.w)*dt6;
    state->quat.x += (k1.q_dot.x+2*k2.q_dot.x+2*k3.q_dot.x+k4.q_dot.x)*dt6;
    state->quat.y += (k1.q_dot.y+2*k2.q_dot.y+2*k3.q_dot.y+k4.q_dot.y)*dt6;
    state->quat.z += (k1.q_dot.z+2*k2.q_dot.z+2*k3.q_dot.z+k4.q_dot.z)*dt6;
    state->omega.x += (k1.w_dot.x+2*k2.w_dot.x+2*k3.w_dot.x+k4.w_dot.x)*dt6;
    state->omega.y += (k1.w_dot.y+2*k2.w_dot.y+2*k3.w_dot.y+k4.w_dot.y)*dt6;
    state->omega.z += (k1.w_dot.z+2*k2.w_dot.z+2*k3.w_dot.z+k4.w_dot.z)*dt6;
    for (int i=0;i<4;i++) state->rpms[i] += (k1.rpm_dot[i]+2*k2.rpm_dot[i]+2*k3.rpm_dot[i]+k4.rpm_dot[i])*dt6;
    quat_normalize(&state->quat);
    float m = fmaxf(fabsf(state->vel.x), fmaxf(fabsf(state->vel.y), fabsf(state->vel.z)));
    if (m > g_max_tmp_vel) g_max_tmp_vel = m;
}

int main(void) {
    unsigned int rng = 42;
    const float DR = 0.05f;   // drone.h reset_agent_base uses 0.05f

    // ---------- F1: action-0 hover leak (sim recentres on randomized params; firmware uses BASE) ----------
    Params base; { Drone d; unsigned int r0 = 1; init_drone(&d, &r0, 0.0f); base = d.params; base.gravity = BASE_GRAVITY; /* init_drone randomizes gravity +-1% even at dr=0 */ }
    int N = 200000; double sum=0, sumsq=0; float amin=1e9f, amax=-1e9f; int sink=0;
    for (int n=0;n<N;n++) {
        Drone d; init_drone(&d, &rng, DR);
        float a_sim = accel_at_action0(&d.params, &d.params);   // sim: mapping uses true params
        float a_fw  = accel_at_action0(&base, &d.params);       // firmware: mapping uses BASE
        if (fabsf(a_sim) > 1e-4f) { printf("UNEXPECTED: sim action0 accel %g\n", a_sim); }
        sum += a_fw; sumsq += (double)a_fw*a_fw; if (a_fw<amin) amin=a_fw; if (a_fw>amax) amax=a_fw; if (a_fw<0) sink++;
    }
    double mean = sum/N, sd = sqrt(sumsq/N - mean*mean);
    printf("F1 action-0 hover leak (DR=%.2f, N=%d)\n", DR, N);
    printf("   sim:      action 0 -> vertical accel = 0.000 m/s^2 in EVERY episode (mapping uses randomized mass/k_thrust)\n");
    printf("   firmware: action 0 -> vertical accel mean %+.3f, sd %.3f, range [%+.3f, %+.3f] m/s^2 (%.1f%% of draws sink)\n",
           mean, sd, amin, amax, 100.0*sink/N);
    printf("   i.e. the policy never sees the mass/thrust error it must correct on hardware.\n");
    // airframe mismatch illustration: constants are brushed CF2.x (27 g). Brushless mass filled from datasheet at report time.
    for (float m_true = 0.027f; m_true <= 0.0421f; m_true += 0.005f) {
        Params t = base; t.mass = m_true;
        printf("   if true mass = %.1f g with BASE k_thrust: action 0 -> %+.2f m/s^2 (hover needs action %+.3f)\n",
               m_true*1000, accel_at_action0(&base, &t),
               2.0f*(rpm_hover(&t)-rpm_min_for_centered_hover(&base))/(base.max_rpm-rpm_min_for_centered_hover(&base)) - 1.0f);
    }

    // ---------- F2: zero aerodynamic drag -> velocity reaches hard box clamp ----------
    { Drone d; unsigned int r=7; init_drone(&d, &r, 0.0f);
      float act[4]={1,1,1,1}; // full throttle, level: pure vertical accel = Tmax/m - g
      float Tmax = 4*d.params.k_thrust*d.params.max_rpm*d.params.max_rpm;
      printf("F2 zero drag: b_drag=%g, k_ang_damp=%g. Tmax=%.3f N, T/W=%.2f, level full-throttle accel=%.2f m/s^2\n",
             d.params.b_drag, d.params.k_ang_damp, Tmax, Tmax/(d.params.mass*d.params.gravity), Tmax/d.params.mass-d.params.gravity);
      int steps=0; double ke_removed=0; int clamp_events=0; int printed=0;
      while (steps < 100*60) { // up to 60 s at 100 Hz
        // replicate move_drone but measure energy discarded by the clamp
        clamp4(act,-1,1);
        for (int s=0;s<ACTION_SUBSTEPS;s++) {
          rk4_step(&d.state,&d.params,act,DT);
          float v2_before = dot3(d.state.vel,d.state.vel);
          Vec3 before = d.state.vel;
          clamp3(&d.state.vel,-d.params.max_vel,d.params.max_vel);
          clamp3(&d.state.omega,-d.params.max_omega,d.params.max_omega);
          for (int i=0;i<4;i++) d.state.rpms[i]=clampf(d.state.rpms[i],0,d.params.max_rpm);
          float v2_after = dot3(d.state.vel,d.state.vel);
          if (v2_after < v2_before - 1e-9f) { clamp_events++; ke_removed += 0.5*d.params.mass*(v2_before-v2_after); }
          (void)before;
        }
        steps++;
        if (clamp_events>0 && !printed) { printf("   vertical velocity hit the %.0f m/s clamp after %.2f s of full throttle (grid is only %.0f m tall)\n", d.params.max_vel, steps*ACTION_DT, 2*GRID_Z); printed=1; }
      }
      printf("   over 60 s: kinetic energy silently removed by the clamp = %.3f J (drone KE at clamp = %.3f J)\n", ke_removed, 0.5*d.params.mass*d.params.max_vel*d.params.max_vel);
    }

    // ---------- F3: candidate 'negative RPM in intermediates' and F4: 'velocity exceeds bound inside substeps' ----------
    { unsigned int r=99; int rollouts=2000, T=200; float worst_over=0;
      for (int k=0;k<rollouts;k++) {
        Drone d; init_drone(&d,&r,DR);
        d.state.vel = (Vec3){ rndf(-20,20,&r), rndf(-20,20,&r), rndf(-20,20,&r) };
        for (int t=0;t<T;t++) {
          float act[4]; for(int i=0;i<4;i++) act[i] = (rndf(0,1,&r)<0.5f)? -1.f : 1.f; // bang-bang worst case
          for (int s=0;s<ACTION_SUBSTEPS;s++) {
            rk4_step_instrumented(&d.state,&d.params,act,DT);
            float over = g_max_tmp_vel - d.params.max_vel; if (over>worst_over) worst_over=over; g_max_tmp_vel=0;
            clamp3(&d.state.vel,-d.params.max_vel,d.params.max_vel);
            clamp3(&d.state.omega,-d.params.max_omega,d.params.max_omega);
            for (int i=0;i<4;i++) d.state.rpms[i]=clampf(d.state.rpms[i],0,d.params.max_rpm);
          }
        }
      }
      printf("F3 negative RPM: min intermediate (pre-clamp) rpm over %d bang-bang rollouts = %.1f  (candidate %s)\n", rollouts, g_min_tmp_rpm, g_min_tmp_rpm<0? "CONFIRMED":"REJECTED");
      printf("F4 velocity bound inside substeps: worst pre-clamp exceedance = %.4f m/s = %.3f%% of max_vel (candidate %s)\n",
             worst_over, 100*worst_over/BASE_MAX_VEL, worst_over > 0.01f*BASE_MAX_VEL ? "MATERIAL":"NEGLIGIBLE");
    }

    // ---------- F6: trim offset on the real brushless, measured PWM->thrust curve ----------
    // Per-motor thrust vs PWM duty ratio, cubic fit from Graefe, Scherer, Hoenig, Trimpe, "How to Model Your
    // Crazyflie Brushless", ICRA 2026, released code (environment/quadcopter.py). Cross-checked: gives 118 gf
    // total at full PWM (datasheet ~120 gf) and 0.153 N/motor at 2/3 PWM vs 0.145 N from the utiasDSL
    // rpm->thrust fit at the paper's K=2900 rad/s command gain.
    { float ratio0 = rpm_hover(&base)/base.max_rpm;   // what the firmware sends at action 0 (0.667)
      #define T_MOTOR(p) (-0.23009526f*(p)*(p)*(p) + 0.56176458f*(p)*(p) - 0.0433191f*(p))
      float T0 = 4*T_MOTOR(ratio0);
      printf("F6 measured brushless curve: full PWM = %.0f gf total; firmware action 0 -> PWM %.3f -> %.3f N total\n", 4*T_MOTOR(1.0f)/base.gravity*1000, ratio0, T0);
      float masses[3] = {0.034f, 0.040f, 0.044f}; const char* names[3]={"34 g (Bitcraze, legs)","40 g (Graefe et al., no guards)","44 g (Graefe et al., guards)"};
      for (int i=0;i<3;i++) {
        float m=masses[i], Tm=m*base.gravity/4, lo=0, hi=1;
        for (int k=0;k<60;k++){ float mid=0.5f*(lo+hi); if (T_MOTOR(mid)<Tm) lo=mid; else hi=mid; }
        float ph=0.5f*(lo+hi);
        float act = 2.0f*(ph*base.max_rpm - rpm_min_for_centered_hover(&base))/(base.max_rpm - rpm_min_for_centered_hover(&base)) - 1.0f;
        printf("   %s: action 0 -> %+.2f m/s^2; hover at PWM %.3f -> action %+.3f (trained on exactly 0)\n", names[i], T0/m - base.gravity, ph, act);
      }
      printf("   Graefe et al. motor lag: T = 50 ms (sim k_mot = 150 ms). Their DR for transfer: mass +-10%%, thrust +-20%%; sim uses +-5%%.\n");
    }
    return 0;
}
