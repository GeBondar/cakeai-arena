"""Run matches (with video) and tournaments."""
import json
import math
import os
import time

from .match import Match
from .pilots import ROSTER
from .video import Broadcast, VideoSink

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out")


def match_tag(opponent, seed, flip, tactic=None):
    return f"{opponent}{'-' + tactic if tactic else ''}_s{seed}{'_flip' if flip else ''}"


def run_match(opponent="vspin", secs=60.0, seed=0, flip=False, tactic=None, fps=60, video=True,
              video_fps=30, out_dir=OUT, latency_frames=2, quiet=False):
    """Play one match. Returns the result dict (also written to <out_dir>/result_<tag>.json)."""
    kw = dict(latency_frames=latency_frames)
    if flip:
        kw["cake_pose"] = (-0.65, -0.65, math.radians(45), True)
    mt = Match(opponent, tactic=tactic, secs=secs, fps=fps, seed=seed, **kw)
    tag = match_tag(opponent, seed, flip, tactic)
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"fight_{tag}.mp4")
    bc = sink = None
    title_s = 2.5
    if video:
        bc = Broadcast(mt)
        sink = VideoSink(path, bc.W, bc.H, video_fps)
        o = mt.opp
        card = bc.card([f"CakeAI  vs  {o.name}",
                        f"{o.name}: {o.blurb} · pilot tactic: {o.tactic}" + (" · CakeAI starts UPSIDE DOWN" if flip else ""),
                        "Autonomous 1.5 kg robot · one overhead camera · ArUco + motion detection",
                        "CakeAI-V1 logic in the loop: pixels -> AI -> Arduino protocol -> motors"])
        for _ in range(int(title_s * video_fps)):
            sink.write(card)
    every = max(1, round(fps / video_fps))
    t0, k, last = time.time(), 0, None
    while True:
        done = mt.step()
        if bc and k % every == 0:
            last = bc.frame()
            sink.write(last)
        k += 1
        if done:
            break
    res = mt.finish()
    res["flip_start"] = flip
    res["wall_s"] = round(time.time() - t0, 1)
    if bc:
        for _ in range(int(1.0 * video_fps)):
            sink.write(last)
        a, b = res["cake"], res["opp"]
        end = bc.card([f"WINNER: {res['winner']}  ({res['reason']})",
                       f"CakeAI: HP {a['hp']:.0f} · weapon hits {a['weapon_hits']} · {a['energy_J']:.0f} J delivered · flips {a['flips']}",
                       f"{b['name']}: HP {b['hp']:.0f} · weapon hits {b['weapon_hits']} · rams {b['rams']} · flips {b['flips']}",
                       f"match time {res['t']:.1f} s"], base=last)
        for _ in range(int(3.5 * video_fps)):
            sink.write(end)
        sink.close()
        bc.close()
        res["video"] = os.path.relpath(path, out_dir)
        res["video_offset_s"] = title_s           # match time 0 starts here in the video
    mt.close()
    with open(os.path.join(out_dir, f"result_{tag}.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    if not quiet:
        print(f"[{tag}] winner {res['winner']} ({res['reason']}) after {res['t']:.1f} s "
              f"| CakeAI HP {res['cake']['hp']:.0f} vs {res['opp']['name']} HP {res['opp']['hp']:.0f} | {res['wall_s']} s wall")
    return res


DEFAULT_CARD = [("vspin", 0, False), ("hspin", 0, False), ("wedge", 0, False), ("flipper", 0, False),
                ("vspin-orbit", 1, False), ("wedge-zigzag", 2, False), ("flipper", 3, True)]


def tournament(card=None, secs=60.0, jobs=3, video=True, out_dir=OUT):
    """Run a fight card in parallel processes, then write results.md / results.json / all_fights.mp4."""
    import subprocess
    import sys
    from concurrent.futures import ThreadPoolExecutor
    card = card or DEFAULT_CARD
    out_dir = os.path.abspath(out_dir)
    root =os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def one(job):
        opp, seed, flip = job
        cmd = [sys.executable, "-m", "arena", "fight", opp, "--secs", str(secs), "--seed", str(seed), "--out", out_dir]
        cmd += ["--flip"] if flip else []
        cmd += [] if video else ["--no-video"]
        p = subprocess.run(cmd, cwd=root, capture_output=True, text=True, encoding="utf-8", errors="replace")
        rp = os.path.join(out_dir, f"result_{match_tag(opp, seed, flip)}.json")
        if p.returncode or not os.path.exists(rp):
            print(f"[FAIL] {opp} seed {seed}\n{p.stderr[-3000:]}")
            return None
        print(p.stdout.strip().splitlines()[-1])
        with open(rp, encoding="utf-8") as f:
            return json.load(f)

    os.makedirs(out_dir, exist_ok=True)
    with ThreadPoolExecutor(jobs) as ex:
        results = [r for r in ex.map(one, card) if r]
    with open(os.path.join(out_dir, "results.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    rows = ["| # | Opponent | Start | Winner | How | Time, s | CakeAI HP | Opp HP | CakeAI hits | Energy, J | Flips by opp |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for i, r in enumerate(results, 1):
        c, o = r["cake"], r["opp"]
        rows.append(f"| {i} | {o['name']} ({ROSTER[r['opponent']].blurb}) | {'upside down' if r['flip_start'] else 'normal'} | "
                    f"**{r['winner']}** | {r['reason']} | {r['t']} | {c['hp']:.0f} | {o['hp']:.0f} | {c['weapon_hits']} | "
                    f"{c['energy_J']:.0f} | {o['flips']} |")
    with open(os.path.join(out_dir, "results.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(rows) + "\n")
    print("\n".join(rows))
    if video:
        from .video import ffmpeg_exe
        exe = ffmpeg_exe()
        lst = os.path.join(out_dir, "concat.txt")
        with open(lst, "w", encoding="utf-8") as f:
            for r in results:
                f.write(f"file '{os.path.join(out_dir, r['video']).replace(os.sep, '/')}'\n")
        if exe:
            subprocess.run([exe, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-c", "copy",
                            "-movflags", "+faststart", os.path.join(out_dir, "all_fights.mp4")], check=False)
        os.remove(lst)
    return results
