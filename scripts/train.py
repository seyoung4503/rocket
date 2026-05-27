"""Train an RL landing policy and compare it to the PID baseline.

    python scripts/train.py --difficulty hard --steps 800000 --n-envs 8

Saves the model to models/<algo>_<difficulty>.zip. Evaluate with:
    python scripts/evaluate.py --policy models/ppo_hard.zip --difficulty hard
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--difficulty", default="hard", choices=["calm", "moderate", "hard"])
    ap.add_argument("--algo", default="ppo", choices=["ppo", "sac"])
    ap.add_argument("--steps", type=int, default=800_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--out", default=None)
    ap.add_argument("--init-from", default=None, help="warm-start from a saved model (curriculum)")
    ap.add_argument("--ent-coef", type=float, default=0.005, help="PPO entropy coef (exploration)")
    ap.add_argument("--step-penalty", type=float, default=0.08, help="per-step penalty (anti-hover)")
    ap.add_argument("--init-scale", type=float, default=1.0, help="reverse curriculum: ease of start (0..1)")
    ap.add_argument("--residual", action="store_true", help="residual RL: learn a correction on top of PID")
    ap.add_argument("--residual-scale", type=float, default=0.4, help="max residual correction magnitude")
    args = ap.parse_args()

    from functools import partial

    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from rocketsim.envs import make_landing_env

    # picklable for SubprocVecEnv workers (spawn) — make_landing_env is importable
    env_fn = partial(
        make_landing_env,
        args.difficulty,
        step_penalty=args.step_penalty,
        init_scale=args.init_scale,
        residual=args.residual,
        residual_scale=args.residual_scale,
    )

    out = args.out or f"models/{args.algo}_{args.difficulty}.zip"
    os.makedirs("models", exist_ok=True)

    if args.algo == "ppo":
        venv = make_vec_env(env_fn, n_envs=args.n_envs, vec_env_cls=SubprocVecEnv)
        if args.init_from:
            print(f"warm-starting PPO from {args.init_from}")
            model = PPO.load(args.init_from, env=venv)
            model.ent_coef = args.ent_coef
        else:
            model = PPO(
                "MlpPolicy",
                venv,
                n_steps=1024,
                batch_size=2048,
                gae_lambda=0.95,
                gamma=0.99,
                ent_coef=args.ent_coef,
                learning_rate=3e-4,
                target_kl=0.06,  # trust region: avoid catastrophic policy collapse
                policy_kwargs=dict(net_arch=[256, 256]),
                verbose=1,
            )
    else:
        env = env_fn()
        if args.init_from:
            print(f"warm-starting SAC from {args.init_from}")
            model = SAC.load(args.init_from, env=env)
        else:
            model = SAC(
                "MlpPolicy",
                env,
                learning_rate=3e-4,
                buffer_size=300_000,
                batch_size=512,
                gamma=0.99,
                policy_kwargs=dict(net_arch=[256, 256]),
                verbose=1,
            )

    ckpt = CheckpointCallback(
        save_freq=max(1, 100_000 // args.n_envs),
        save_path="models/checkpoints",
        name_prefix=f"{args.algo}_{args.difficulty}",
    )
    print(f"training {args.algo} on '{args.difficulty}' for {args.steps} steps -> {out}")
    model.learn(total_timesteps=args.steps, progress_bar=False, callback=ckpt)
    model.save(out)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
