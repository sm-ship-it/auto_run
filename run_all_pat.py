#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import itertools
import re
import subprocess
import sys
import time
from pathlib import Path


PARAM_E_START_RE = re.compile(r"^\s*param\s*:\s*E\s*:\s*c\s*:=")


def parse_network_prefix(network: str, start_step: int) -> tuple[str, int]:
    """
    div_autorun_multi.py と同じ考え方で、
    cost239-1 のような指定なら stem=cost239, step=1 として扱う。
    """
    if re.search(r"-\d+$", network):
        stem, step_text = network.rsplit("-", 1)
        return stem, int(step_text)
    return network, start_step


def load_representative_undirected_edges(dat_path: Path) -> list[tuple[int, int]]:
    """
    dat ファイルの param : E : c := ブロックからリンクを読む。

    双方向故障前提なので、(1,2) と (2,1) が両方あっても
    物理リンクとしては1本だけ採用する。

    ただし、元プログラム側では fault_links が dat の E に
    存在する向きでないとエラーになるため、
    dat に最初に現れた向きを代表として使う。
    """
    edges: list[tuple[int, int]] = []
    seen_undirected: set[tuple[int, int]] = set()

    in_e_block = False

    with dat_path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()

            if not in_e_block:
                if PARAM_E_START_RE.match(line):
                    in_e_block = True
                continue

            if line == ";":
                break

            if not line:
                continue

            parts = line.split()
            if len(parts) < 2:
                continue

            u = int(parts[0])
            v = int(parts[1])
            key = tuple(sorted((u, v)))

            if key not in seen_undirected:
                seen_undirected.add(key)
                edges.append((u, v))

    if not edges:
        raise ValueError(f"dat ファイルから E を読み取れませんでした: {dat_path}")

    return edges


def edge_to_text(edge: tuple[int, int]) -> str:
    return f"{edge[0]}-{edge[1]}"


def fault_pattern_to_text(pattern: tuple[tuple[int, int], ...]) -> str:
    return ",".join(edge_to_text(e) for e in pattern)


def safe_name(text: str) -> str:
    """
    出力ディレクトリ名に使いやすい形に変換する。
    例:
      1-2      -> f_1-2
      1-2,5-6  -> f_1-2__5-6
    """
    return "f_" + text.replace(",", "__")


def parse_cplex_solution_time(line: str) -> float:
    m = re.search(r"Solution time\s*=\s*([0-9]+(?:\.[0-9]+)?)\s*sec\.", line)
    if m:
        return float(m.group(1))
    return 0.0


def sum_cplex_time_in_output_dir(output_dir: Path) -> float:
    total = 0.0
    if not output_dir.exists():
        return total

    for path in output_dir.rglob("*.log"):
        if path.name.endswith("-1.log"):
            continue
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f:
                for raw in f:
                    total += parse_cplex_solution_time(raw)
        except OSError:
            continue

    return total


def parse_lmax_from_line(line: str) -> float:
    m = re.search(r"^\s*Lmax\s+([0-9]+(?:\.[0-9]+)?)\s*$", line)
    if m:
        return float(m.group(1))
    return 0.0


def sum_lmax_in_output_dir(output_dir: Path, exclude_log_name: str | None = None) -> float:
    total = 0.0
    if not output_dir.exists():
        return total

    for path in output_dir.rglob("*.log"):
        if exclude_log_name is not None and path.name == exclude_log_name:
            continue
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f:
                for raw in f:
                    total += parse_lmax_from_line(raw)
        except OSError:
            continue

    return total


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="div_autorun_multi.py を、1箇所故障・2箇所故障の全パターンで実行する"
    )

    p.add_argument(
        "network",
        help="元プログラムに渡す network。例: cost239 または cost239-1"
    )

    p.add_argument(
        "--program",
        default="div_autorun_multi.py",
        help="実行対象の Python プログラム"
    )

    p.add_argument(
        "--python",
        default=sys.executable,
        help="使用する Python コマンド。省略時は現在の Python"
    )

    p.add_argument(
        "--work-dir",
        default=str(Path(__file__).resolve().parent),
        help="元プログラムの --work-dir に渡す作業ディレクトリ"
    )

    p.add_argument(
        "--input-dir-name",
        default="input",
        help="元プログラムの --input-dir-name に渡す値"
    )

    p.add_argument(
        "--output-root-name",
        default="batch_output",
        help="各パターンの出力ディレクトリを作る親ディレクトリ名"
    )

    p.add_argument(
        "--start-step",
        type=int,
        default=1,
        help="初期 step 番号。network に cost239-1 のように step が含まれる場合はそちらを優先"
    )

    p.add_argument(
        "--summary-csv",
        default="batch_summary.csv",
        help="各パターンの実行結果をまとめる CSV"
    )

    p.add_argument(
        "--dry-run",
        action="store_true",
        help="コマンドを表示するだけで実行しない"
    )

    p.add_argument(
        "--continue-on-error",
        action="store_true",
        help="途中でエラーが出ても残りのパターンを実行する"
    )

    return p


def main() -> int:
    argv = sys.argv[1:]

    if "--" in argv:
        sep = argv.index("--")
        wrapper_argv = argv[:sep]
        target_args = argv[sep + 1:]
    else:
        wrapper_argv = argv
        target_args = []

    args = build_arg_parser().parse_args(wrapper_argv)

    work_dir = Path(args.work_dir).resolve()
    program = Path(args.program)

    if not program.is_absolute():
        program = work_dir / program
    program = program.resolve()

    if not program.exists():
        raise FileNotFoundError(f"実行対象プログラムが見つかりません: {program}")

    stem, initial_step = parse_network_prefix(args.network, args.start_step)

    input_dir = work_dir / args.input_dir_name
    initial_dat = input_dir / f"{stem}-{initial_step}.dat"

    if not initial_dat.exists():
        raise FileNotFoundError(f"初期 dat ファイルが見つかりません: {initial_dat}")

    edges = load_representative_undirected_edges(initial_dat)

    patterns: list[tuple[tuple[int, int], ...]] = []
    patterns.extend(itertools.combinations(edges, 1))
    patterns.extend(itertools.combinations(edges, 2))

    output_root = work_dir / args.output_root_name
    output_root.mkdir(parents=True, exist_ok=True)

    summary_csv = work_dir / args.summary_csv
    status_csv = work_dir / "batch_status.csv"

    print(f"program      = {program}")
    print(f"work_dir     = {work_dir}")
    print(f"initial dat  = {initial_dat}")
    print(f"links        = {len(edges)}")
    print(f"patterns     = {len(patterns)}")
    print(f"output root  = {output_root}")
    print(f"summary csv  = {summary_csv}")
    print(f"status csv   = {status_csv}")
    print("")

    with status_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "fault_links",
            "fault_count",
            "returncode",
            "output_dir",
            "python_time_sec",
            "cplex_time_sec",
            "lmax_sum",
            "inspection_time_sec",
            "total_time_sec",
        ])

        for idx, pattern in enumerate(patterns, start=1):
            fault_links = fault_pattern_to_text(pattern)
            fault_count = len(pattern)

            # 各パターンの中間ファイルが上書きされないように、
            # パターンごとに別 output ディレクトリを使う。
            output_dir_name = f"{args.output_root_name}/{safe_name(fault_links)}"

            cmd = [
                args.python,
                str(program),
                args.network,
                fault_links,
                "--work-dir",
                str(work_dir),
                "--input-dir-name",
                args.input_dir_name,
                "--output-dir-name",
                output_dir_name,
                "--start-step",
                str(args.start_step),
                "--result-csv",
                str(summary_csv),
            ]

            cmd.extend(target_args)

            print(f"[{idx}/{len(patterns)}] fault_links={fault_links}")
            print(" ".join(cmd))

            if args.dry_run:
                returncode = 0
                python_time_sec = 0.0
                cplex_time_sec = 0.0
                inspection_time_sec = 0.0
                total_time_sec = 0.0
                base_solution_time = 0.0
            else:
                start = time.perf_counter()
                result = subprocess.run(cmd)
                end = time.perf_counter()
                returncode = result.returncode
                python_time_sec = end - start
                print(f"python execution time: {python_time_sec:.3f} sec")

                output_dir_path = work_dir / output_dir_name
                
                t_cplex_start = time.perf_counter()
                cplex_time_sec = sum_cplex_time_in_output_dir(output_dir_path)
                t_cplex_end = time.perf_counter()
                cplex_read_time = t_cplex_end - t_cplex_start
                
                exclude_log_name = f"{args.network}.log"
                t_lmax_start = time.perf_counter()
                lmax_sum = sum_lmax_in_output_dir(output_dir_path, exclude_log_name=exclude_log_name)
                t_lmax_end = time.perf_counter()
                lmax_read_time = t_lmax_end - t_lmax_start
                
                inspection_time_sec = lmax_sum / 200_000
                total_time_sec = python_time_sec + cplex_time_sec + inspection_time_sec
                print(f"cplex execution time sum: {cplex_time_sec:.3f} sec")
                print(f"cplex log read time: {cplex_read_time:.3f} sec")
                print(f"lmax log read time: {lmax_read_time:.3f} sec")
                print(f"inspection time sum: {inspection_time_sec:.6f} sec")
                print(f"total time: {total_time_sec:.3f} sec")

            writer.writerow([
                fault_links,
                fault_count,
                returncode,
                output_dir_name,
                f"{python_time_sec:.3f}",
                f"{cplex_time_sec:.3f}",
                f"{lmax_sum:.3f}",
                f"{inspection_time_sec:.6f}",
                f"{total_time_sec:.3f}",
            ])
            f.flush()

            if returncode != 0:
                print(f"ERROR: failed fault_links={fault_links}, returncode={returncode}", file=sys.stderr)
                if not args.continue_on_error:
                    return returncode

            print("")

    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())