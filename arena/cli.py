"""Command line:  python -m arena <command>   (or `cakeai-arena <command>` after pip install)

    fight <opponent>     one match with video          python -m arena fight hspin
    tournament           the full fight card           python -m arena tournament -j 3
    play                 you drive, CakeAI fights      python -m arena play --robot flipper
    roster               list opponents
    check                verify markers / heading on the rendered camera image
"""
import argparse
import sys

from .pilots import ROSTER, TACTICS


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(prog="python -m arena", description="CakeAI Arena — 1.5 kg robot combat simulator")
    sub = ap.add_subparsers(dest="cmd", required=True)

    f = sub.add_parser("fight", help="run one match (video + JSON in out/)")
    f.add_argument("opponent", nargs="?", default="vspin", choices=list(ROSTER))
    f.add_argument("--secs", type=float, default=60.0, help="match length (KO ends it earlier)")
    f.add_argument("--seed", type=int, default=0)
    f.add_argument("--tactic", choices=TACTICS, help="override the pilot's tactic")
    f.add_argument("--flip", action="store_true", help="CakeAI starts upside down (marker ID 1)")
    f.add_argument("--fps", type=int, default=60, help="overhead camera frame rate")
    f.add_argument("--latency", type=int, default=2, help="camera->motor latency, frames")
    f.add_argument("--no-video", action="store_true")
    f.add_argument("--out", default=None, help="output folder (default: ./out)")

    t = sub.add_parser("tournament", help="run the full fight card in parallel")
    t.add_argument("--secs", type=float, default=60.0)
    t.add_argument("-j", "--jobs", type=int, default=3)
    t.add_argument("--no-video", action="store_true")
    t.add_argument("--out", default=None)

    p = sub.add_parser("play", help="drive the opponent yourself (needs pygame)")
    p.add_argument("--robot", default="flipper", choices=list(ROSTER))
    p.add_argument("--secs", type=float, default=120.0)
    p.add_argument("--fps", type=int, default=30)

    sub.add_parser("roster", help="list opponents")
    sub.add_parser("check", help="verify ArUco detection + heading on rendered frames")

    a = ap.parse_args(argv)
    if a.cmd == "fight":
        from .runner import OUT, run_match
        run_match(a.opponent, secs=a.secs, seed=a.seed, flip=a.flip, tactic=a.tactic, fps=a.fps,
                  video=not a.no_video, out_dir=a.out or OUT, latency_frames=a.latency)
    elif a.cmd == "tournament":
        from .runner import OUT, tournament
        tournament(secs=a.secs, jobs=a.jobs, video=not a.no_video, out_dir=a.out or OUT)
    elif a.cmd == "play":
        from .play import play
        play(a.robot, secs=a.secs, fps=a.fps)
    elif a.cmd == "roster":
        for o in ROSTER.values():
            print(f"{o.key:13s} {o.name:11s} robot={o.robot:8s} tactic={o.tactic:7s} {o.blurb}")
    elif a.cmd == "check":
        from .checks import check_markers
        sys.exit(0 if check_markers() else 1)


if __name__ == "__main__":
    main()
