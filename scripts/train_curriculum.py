"""Reverse + difficulty curriculum to teach PPO the landing skill, then harden.

The bottleneck on this task is *discovering* the precise soft-touchdown maneuver:
from a 12 m start it is almost never stumbled upon, so PPO collapses to hovering.
We fix that by starting episodes just above the pad on calm air (touchdown is
trivially and frequently experienced), then gradually raising both the start
difficulty (init_scale) and the disturbance level, warm-starting throughout.

    python scripts/train_curriculum.py

Saves models/ppo_hard.zip (final) and per-stage checkpoints in models/.
"""

from __future__ import annotations

import os
import sys
from functools import partial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# (difficulty, init_scale, steps, ent_coef)
STAGES = [
    ("calm", 0.12, 400_000, 0.010),
    ("calm", 0.45, 350_000, 0.010),
    ("moderate", 0.75, 450_000, 0.007),
    ("hard", 1.00, 1_400_000, 0.005),
]
N_ENVS = 8
STEP_PENALTY = 0.08


def main() -> int:
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from rocketsim.envs import make_landing_env

    os.makedirs("models", exist_ok=True)

    def venv_for(difficulty, init_scale):
        fn = partial(
            make_landing_env, difficulty, step_penalty=STEP_PENALTY, init_scale=init_scale
        )
        return make_vec_env(fn, n_envs=N_ENVS, vec_env_cls=SubprocVecEnv)

    model = None
    for i, (difficulty, init_scale, steps, ent) in enumerate(STAGES):
        venv = venv_for(difficulty, init_scale)
        if model is None:
            model = PPO(
                "MlpPolicy",
                venv,
                n_steps=1024,
                batch_size=2048,
                gae_lambda=0.95,
                gamma=0.99,
                ent_coef=ent,
                # PPO was finding a good policy then destroying it with an
                # over-large update (ep_rew climbed to +4 then collapsed to
                # -157). target_kl is a trust region: it stops an update once
                # the policy has moved too far, preventing that collapse. A
                # slightly lower constant LR adds margin (a global decay would
                # wrongly zero-out the LR by the final curriculum stage).
                learning_rate=3e-4,
                target_kl=0.06,
                policy_kwargs=dict(net_arch=[256, 256]),
                verbose=1,
            )
        else:
            model.set_env(venv)
            model.ent_coef = ent
        print(f"\n=== STAGE {i+1}/{len(STAGES)}: {difficulty} init_scale={init_scale} "
              f"steps={steps} ent={ent} ===", flush=True)
        model.learn(total_timesteps=steps, progress_bar=False, reset_num_timesteps=(i == 0))
        out = f"models/ppo_stage{i+1}_{difficulty}.zip"
        model.save(out)
        print(f"STAGE {i+1} saved {out}", flush=True)
        venv.close()

    model.save("models/ppo_hard.zip")
    print("saved models/ppo_hard.zip", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
