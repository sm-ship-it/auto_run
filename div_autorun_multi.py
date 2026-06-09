#!/usr/bin/env python3
"""
複数故障対応のネットワーク分割・検査自動実行スクリプト

概要:
- 1回目の分割結果(= 初期 dat/log/dst/pos/seq)は既に与えられている前提
- 2回目以降の分割を自動で繰り返す
- CPLEX 実行には obj_ILP.sh と mod ファイルを利用する
- 各ステップで画像と seq を生成し，故障を含むルートだけを次段へ送る
- 複数ルートで故障が出た場合は論文フローに従って
  重複リンクの全検査と，各故障ルートの並列再分割/全検査を行う

注意:
- 既存の cplexvis_multi.py / stepper_ni_multi.py / gen-data_multi.py / keiro.py
  は参考実装であり，本スクリプトはそれらを内部で再実装する
- 入力の dst.csv は 3列 CSV (u,v,cost) を前提とする
- log は CPLEX の display solution variable - の出力を前提とする
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import subprocess
import sys
from collections import Counter, defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


# =========================
# データ構造
# =========================

@dataclass(frozen=True, order=True)
class Edge:
    """有向リンク (u -> v)"""
    u: int
    v: int

    def rev(self) -> "Edge":
        return Edge(self.v, self.u)

    def key(self) -> str:
        return f"{self.u}-{self.v}"

    def as_tuple(self) -> tuple[int, int]:
        return (self.u, self.v)

    def undir_tuple(self) -> tuple[int, int]:
        """物理リンクとしての無向表現"""
        return tuple(sorted((self.u, self.v)))


@dataclass
class RouteInfo:
    """1本の検査ルートを表す"""
    route_id: int
    depot: int | None
    node_seq: list[int]
    edges: list[Edge]

    @property
    def edge_count(self) -> int:
        return len(self.edges)

    @property
    def node_set(self) -> set[int]:
        return set(self.node_seq)


@dataclass
class StepFiles:
    """あるステップに対応する入出力ファイル群"""
    stem: str
    step: int
    base_dir: Path

    @property
    def prefix(self) -> str:
        return f"{self.stem}-{self.step}"

    @property
    def dat(self) -> Path:
        return self.base_dir / f"{self.prefix}.dat"

    @property
    def log(self) -> Path:
        return self.base_dir / f"{self.prefix}.log"

    @property
    def dst(self) -> Path:
        return self.base_dir / f"{self.prefix}_dst.csv"

    @property
    def pos(self) -> Path:
        return self.base_dir / f"{self.prefix}_pos.txt"

    @property
    def seq(self) -> Path:
        return self.base_dir / f"{self.prefix}_seq.csv"

    @property
    def all_png(self) -> Path:
        return self.base_dir / f"{self.prefix}_all_routes.png"

    def route_png(self, route_id: int) -> Path:
        return self.base_dir / f"{self.prefix}_route_{route_id}.png"


@dataclass
class WorkItem:
    """分割木上の1つの検査対象サブネット"""
    stem: str
    parent_stem: str | None
    current_step: int
    fault_edges: set[Edge]
    inspection_count: int = 0
    history: list[str] = field(default_factory=list)


@dataclass
class RunContext:
    base_dir: Path
    mod_file: Path
    ilp_sh: Path
    next_auto_step: int
    generated_images: list[Path] = field(default_factory=list)
    generated_logs: list[Path] = field(default_factory=list)
    verbose: bool = True
    no_image: bool = False

    def log(self, msg: str) -> None:
        if self.verbose:
            print(msg, flush=True)


# =========================
# 基本ユーティリティ
# =========================

EDGE_RE = re.compile(r"^\s*x\((\d+),(\d+),(\d+)\)\s+([0-9Ee+\-.]+)\s*$")
DP_RE = re.compile(r"^\s*DP\((\d+),(\d+)\)\s+([0-9Ee+\-.]+)\s*$")
PARAM_RE = re.compile(r"^\s*param\s+(\w+)\s*:=\s*(\d+)")


def parse_edge_token(token: str) -> Edge:
    parts = token.strip().split("-")
    if len(parts) != 2:
        raise ValueError(f"故障リンク指定が不正です: {token}")
    return Edge(int(parts[0]), int(parts[1]))


def parse_fault_edges(text: str) -> set[Edge]:
    edges = set()
    for token in text.split(","):
        token = token.strip()
        if token:
            edges.add(parse_edge_token(token))
    if not edges:
        raise ValueError("故障リンクが1本も指定されていません")
    return edges


def read_dat_params(dat_path: Path) -> dict[str, int]:
    params: dict[str, int] = {}
    with dat_path.open("r", encoding="utf-8") as f:
        for line in f:
            m = PARAM_RE.match(line)
            if m:
                params[m.group(1)] = int(m.group(2))
    return params


def load_pos(pos_path: Path) -> dict[int, tuple[float, float]]:
    nodepos: dict[int, tuple[float, float]] = {}
    with pos_path.open("r", encoding="utf-8") as f:
        for idx, line in enumerate(f, start=1):
            x, y = map(float, line.strip().split())
            nodepos[idx] = (x, y)
    return nodepos


def load_dst_simple(dst_path: Path) -> list[tuple[int, int, int]]:
    rows: list[tuple[int, int, int]] = []
    with dst_path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if len(row) < 3:
                raise ValueError(f"dst ファイルの列数が不足しています: {dst_path} / {row}")
            rows.append((int(row[0]), int(row[1]), int(row[2])))
    return rows


def build_detour_map(dst_path: Path) -> dict[str, list[int]]:
    """
    cplexvis_multi.py 互換:
    dst.csv が単純 3 列のときは detour 情報なし.
    7列目以降を持つ場合は '存在しないリンク' の展開経路として読む.
    """
    detdata: dict[str, list[int]] = {}
    with dst_path.open("r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f]
    if not lines:
        return detdata

    # 先頭行がヘッダっぽくなければ 3 列CSVとみなす
    first = lines[0].split(",")
    if len(first) == 3 and all(part.strip().lstrip("-").isdigit() for part in first[:3]):
        return detdata

    detedgename = None
    for line in lines[1:]:
        dfbuf = [x for x in line.strip().split(",")]
        if len(dfbuf) < 2:
            continue
        detedges = [int(k) for k in dfbuf[6:] if k != ""] if len(dfbuf) >= 7 else []
        if dfbuf[1] == "":
            if detedgename is not None:
                detdata[detedgename] += detedges
        else:
            detdata[dfbuf[1]] = detedges
            detedgename = dfbuf[1]
    return detdata


def compute_optimal_division_from_edge_count(edge_count_undirected: int) -> int:
    """keiro.py の式を関数化"""
    if edge_count_undirected <= 1:
        return 1
    best_a = 2
    best_cost = None
    for a in range(2, edge_count_undirected + 1):
        k = 0
        while math.ceil(edge_count_undirected / (a ** k)) > a:
            k += 1
        cost = a * k + math.ceil(edge_count_undirected / (a ** k))
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_a = a
    return best_a


def undir_edge_count(dst_rows: Iterable[tuple[int, int, int]]) -> int:
    undirected = {tuple(sorted((u, v))) for u, v, _ in dst_rows}
    return len(undirected)

def load_dat_edges(dat_path: Path) -> set[Edge]:
    edges: set[Edge] = set()
    in_e_block = False

    with dat_path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not in_e_block:
                if line.startswith("param : E :"):
                    in_e_block = True
                continue

            if line == ";":
                break
            if not line:
                continue

            parts = line.split()
            if len(parts) < 2:
                continue
            edges.add(Edge(int(parts[0]), int(parts[1])))

    return edges

# =========================
# CPLEX log / route 復元
# =========================


def parse_log_routes(log_path: Path) -> tuple[dict[int, int | None], dict[int, list[Edge]]]:
    depot_by_k: dict[int, int | None] = {}
    edges_by_k: dict[int, list[Edge]] = defaultdict(list)

    with log_path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            m1 = EDGE_RE.match(line)
            if m1:
                i, j, k, val = int(m1.group(1)), int(m1.group(2)), int(m1.group(3)), float(m1.group(4))
                if abs(val - 1.0) < 1e-9:
                    edges_by_k[k].append(Edge(i, j))
                continue
            m2 = DP_RE.match(line)
            if m2:
                node, k, val = int(m2.group(1)), int(m2.group(2)), float(m2.group(3))
                if abs(val - 1.0) < 1e-9:
                    depot_by_k[k] = node
    for k in edges_by_k:
        depot_by_k.setdefault(k, None)
    return depot_by_k, edges_by_k


def build_single_route(edge_list: list[Edge], depot: int | None) -> list[int]:
    """cplexvis_multi.py 相当。Hierholzer 法で 1 本の連続路に復元する。"""
    if not edge_list:
        return []

    outdeg = defaultdict(int)
    indeg = defaultdict(int)
    adj: dict[int, list[tuple[int, int]]] = defaultdict(list)
    nodes: set[int] = set()

    for idx, e in enumerate(edge_list):
        adj[e.u].append((e.v, idx))
        outdeg[e.u] += 1
        indeg[e.v] += 1
        nodes.add(e.u)
        nodes.add(e.v)

    used = [False] * len(edge_list)

    start = None
    for n in nodes:
        if outdeg[n] - indeg[n] == 1:
            start = n
            break
    if start is None:
        if depot is not None and outdeg[depot] > 0:
            start = depot
        else:
            start = edge_list[0].u

    stack = [start]
    route: list[int] = []
    while stack:
        v = stack[-1]
        while adj[v] and used[adj[v][-1][1]]:
            adj[v].pop()
        if adj[v]:
            to, eid = adj[v].pop()
            if not used[eid]:
                used[eid] = True
                stack.append(to)
        else:
            route.append(stack.pop())
    route.reverse()

    if sum(used) != len(edge_list):
        print("warning: not all edges were consumed in build_single_route", file=sys.stderr)

    return route


def reconstruct_routes(files: StepFiles) -> list[RouteInfo]:
    depot_by_k, edges_by_k = parse_log_routes(files.log)
    detour_map = build_detour_map(files.dst)
    routes: list[RouteInfo] = []

    for route_id in sorted(edges_by_k):
        raw_edges = edges_by_k[route_id]
        depot = depot_by_k.get(route_id)
        base_route = build_single_route(raw_edges, depot)

        expanded_nodes: list[int] = []
        if base_route:
            expanded_nodes = [base_route[0]]
            for a, b in zip(base_route[:-1], base_route[1:]):
                key = f"{a}-{b}"
                if key in detour_map and detour_map[key]:
                    expanded_nodes.extend(detour_map[key][1:])
                else:
                    expanded_nodes.append(b)

        expanded_edges = [Edge(a, b) for a, b in zip(expanded_nodes[:-1], expanded_nodes[1:])]
        routes.append(RouteInfo(route_id=route_id, depot=depot, node_seq=expanded_nodes, edges=expanded_edges))

    return routes


def save_seq_file(files: StepFiles, routes: list[RouteInfo]) -> None:
    with files.seq.open("w", encoding="utf-8", newline="") as f:
        for route in routes:
            f.write(",".join(map(str, route.node_seq)) + "\n")


# =========================
# 画像出力
# =========================


def save_route_figures(files: StepFiles, routes: list[RouteInfo], ctx: RunContext) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import networkx as nx
    except Exception as e:  # pragma: no cover
        raise RuntimeError(f"画像出力に必要な matplotlib/networkx の読み込みに失敗しました: {e}")

    nodepos = load_pos(files.pos)
    dst_rows = load_dst_simple(files.dst)
    base_edges = [(u, v) for u, v, _ in dst_rows]

    colorl = ["orange", "skyblue", "lawngreen", "hotpink", "yellow", "violet", "gold", "cyan"]

    graph = nx.MultiDiGraph()
    kgraphs: list[tuple[int, nx.MultiDiGraph]] = []

    for idx, route in enumerate(routes):
        color = colorl[idx % len(colorl)]
        kg = nx.MultiDiGraph()
        for e in route.edges:
            graph.add_edge(e.u, e.v, color=color)
            kg.add_edge(e.u, e.v, color=color)

        checkedges = list(kg.edges()) + [(b, a) for a, b in kg.edges()]
        for e in base_edges:
            if e not in checkedges:
                kg.add_edge(e[0], e[1], color="gainsboro")
        kgraphs.append((route.route_id, kg))

    # 全ルート図
    plt.figure(figsize=(8, 6))
    if graph.number_of_edges() > 0:
        edge_colors = [edge_attr["color"] for *_, edge_attr in graph.edges(data=True)]
        nx.draw(graph, nodepos, with_labels=True, edge_color=edge_colors,
                node_size=300, width=2, alpha=0.7, arrowsize=10)
    else:
        nx.draw(nx.DiGraph(), nodepos, with_labels=True)
    plt.title("All routes")
    plt.savefig(files.all_png, dpi=300, bbox_inches="tight")
    plt.close()
    ctx.generated_images.append(files.all_png)

    # 各ルート図
    for route_id, kg in kgraphs:
        plt.figure(figsize=(8, 6))
        edge_colors = [edge_attr["color"] for *_, edge_attr in kg.edges(data=True)]
        nx.draw(kg, nodepos, with_labels=True, edge_color=edge_colors,
                node_size=300, width=2, alpha=1.0, arrowsize=20)
        plt.title(f"Route k={route_id}")
        out = files.route_png(route_id)
        plt.savefig(out, dpi=300, bbox_inches="tight")
        plt.close()
        ctx.generated_images.append(out)


# =========================
# サブネット生成
# =========================


def choose_reachable_depot(route: RouteInfo, allowed_depots: set[int]) -> int | None:
    candidates = sorted(route.node_set & allowed_depots)
    return candidates[0] if candidates else None


def write_pos_subset(parent_pos: Path, child_pos: Path, kept_old_nodes: list[int]) -> dict[int, int]:
    old_to_new = {old: idx for idx, old in enumerate(kept_old_nodes, start=1)}
    lines = parent_pos.read_text(encoding="utf-8").splitlines()
    selected = [lines[old - 1] for old in kept_old_nodes]
    child_pos.write_text("\n".join(selected) + ("\n" if selected else ""), encoding="utf-8")
    return old_to_new


def build_route_subproblem(
    parent_files: StepFiles,
    child_files: StepFiles,
    route: RouteInfo,
    allowed_depots: set[int],
    ctx: RunContext,
) -> tuple[dict[int, int], int | None, set[int]]:
    """
    1つのルートから次段サブネットを作る。
    既存 stepper_ni_multi.py は selected_route 1本だけ切り出すが、
    ここでは論文フロー用に RouteInfo を直接使って柔軟に生成する。
    """
    kept_nodes = sorted(route.node_set)
    old_to_new = write_pos_subset(parent_files.pos, child_files.pos, kept_nodes)

    route_undir = {tuple(sorted(e.as_tuple())) for e in route.edges}
    dst_rows = load_dst_simple(parent_files.dst)
    new_rows: list[tuple[int, int, int]] = []
    for u, v, c in dst_rows:
        if u in old_to_new and v in old_to_new and tuple(sorted((u, v))) in route_undir:
            new_rows.append((old_to_new[u], old_to_new[v], c))

    with child_files.dst.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(new_rows)

    chosen_old_depot = choose_reachable_depot(route, allowed_depots)
    chosen_new_depot = old_to_new[chosen_old_depot] if chosen_old_depot is not None else None

    reachable_old_depots = sorted(set(kept_nodes) & allowed_depots)
    reachable_new_depots = {old_to_new[n] for n in reachable_old_depots}

    ctx.log(f"  - child dst saved: {child_files.dst.name} ({len(new_rows)} directed edges)")
    ctx.log(f"  - child pos saved: {child_files.pos.name} ({len(kept_nodes)} nodes)")
    return old_to_new, chosen_new_depot, reachable_new_depots


def write_dat_file(
    dat_path: Path,
    dst_rows: list[tuple[int, int, int]],
    node_num: int,
    division_num: int,
    depot_nodes: set[int] | None = None,
) -> None:
    if depot_nodes is None or not depot_nodes:
        depot_nodes = set(range(1, node_num + 1))

    lines: list[str] = []
    lines.append(f"param N := {node_num} ;")
    lines.append(f"param T := {division_num} ;")
    lines.append(f"param link_num := {undir_edge_count(dst_rows) * 2} ;")
    lines.append("")
    lines.append("param : E : c :=")
    for u, v, c in dst_rows:
        lines.append(f"{u}\t{v}\t{c}")
    lines.append(";")
    lines.append("")
    lines.append("param : d :=")
    for i in range(1, node_num + 1):
        lines.append(f"{i}\t{1 if i in depot_nodes else 0}")
    lines.append(";")
    lines.append("")
    lines.append("end ;")
    dat_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# =========================
# 故障判定ロジック
# =========================


def route_contains_any_fault(route: RouteInfo, fault_edges: set[Edge]) -> bool:
    """
    故障は物理リンクの双方向で同時に起きている前提。
    そのため，ルートが (u,v) または (v,u) のどちらか一方を通れば
    その故障リンクを検出できるものとして判定する。
    """
    route_undir = {e.undir_tuple() for e in route.edges}
    fault_undir = {e.undir_tuple() for e in fault_edges}
    return any(e in route_undir for e in fault_undir)


def route_fault_subset(route: RouteInfo, fault_edges: set[Edge]) -> set[Edge]:
    """
    そのルートで検出可能な故障リンク集合を返す。
    戻り値はユーザー指定の向きを保つが，判定自体は無向で行う。
    """
    route_undir = {e.undir_tuple() for e in route.edges}
    return {e for e in fault_edges if e.undir_tuple() in route_undir}


def same_topology_as_parent(route: RouteInfo, parent_dst_rows: list[tuple[int, int, int]]) -> bool:
    """
    Step4-1a 用の「分割前と同じネットワーク構造」判定。
    厳密な図同型ではなく、親ネットワークの無向リンク集合と
    ルートが通過した無向リンク集合が一致したら同じ構造とみなす。
    """
    parent_undir = {tuple(sorted((u, v))) for u, v, _ in parent_dst_rows}
    route_undir = {tuple(sorted(e.as_tuple())) for e in route.edges}
    return route_undir == parent_undir


def overlapping_edges(routes: list[RouteInfo]) -> set[tuple[int, int]]:
    """
    複数ルート間で重複している物理リンク（無向）を返す。
    (u,v) と (v,u) は同じ重複リンクとして扱う。
    """
    counter = Counter()
    for route in routes:
        for e in {edge.undir_tuple() for edge in route.edges}:
            counter[e] += 1
    return {e for e, c in counter.items() if c >= 2}


def remove_overlaps_from_route(route: RouteInfo, overlaps: set[tuple[int, int]]) -> RouteInfo:
    kept_edges = [e for e in route.edges if e.undir_tuple() not in overlaps]
    if not kept_edges:
        return RouteInfo(route.route_id, route.depot, [], [])

    # エッジから連続する node_seq を再構成
    node_seq = build_single_route(kept_edges, route.depot)
    if not node_seq:
        node_seq = [kept_edges[0].u, kept_edges[0].v]
        for e in kept_edges[1:]:
            node_seq.append(e.v)
    rebuilt_edges = [Edge(a, b) for a, b in zip(node_seq[:-1], node_seq[1:])]
    return RouteInfo(route.route_id, route.depot, node_seq, rebuilt_edges)


# =========================
# 外部コマンド
# =========================


def run_ilp(files: StepFiles, ctx: RunContext) -> None:
    cmd = ["sh", str(ctx.ilp_sh), str(ctx.mod_file), str(files.dat), str(files.log)]
    # ctx.log(f"[ILP] {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=str(ctx.base_dir), capture_output=True, text=True)

# エラー時だけ内容を表示する
    if result.returncode != 0:
        if result.stdout:
            print(result.stdout, end="")
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)

        raise RuntimeError(f"obj_ILP.sh が失敗しました: returncode={result.returncode}")       
    ctx.generated_logs.append(files.log)


# =========================
# メイン再帰処理
# =========================


def inspect_all_links_count(target_edges: Iterable[Edge]) -> int:
    return len({e.undir_tuple() for e in target_edges})


def next_child_stem(parent_stem: str, route_id: int, ctx: RunContext) -> tuple[str, int]:
    step = ctx.next_auto_step
    ctx.next_auto_step += 1
    return f"{parent_stem}_r{route_id}", step


def process_work_item(item: WorkItem, ctx: RunContext, allowed_depots_orig: set[int]) -> int:
    files = StepFiles(item.stem, item.current_step, ctx.base_dir)
    ctx.log("")
    ctx.log(f"=== Processing subnet: stem={item.stem}, step={item.current_step} ===")
    ctx.log(f"fault edges in this subnet: {sorted(e.key() for e in item.fault_edges)}")

    if not files.dat.exists() or not files.log.exists() or not files.dst.exists() or not files.pos.exists():
        raise FileNotFoundError(f"必要ファイルが不足しています: {files.prefix}.*")

    # 与えられた初回ステップでは log をそのまま使う。再帰下では log を新規に解く。
    routes = reconstruct_routes(files)
    save_seq_file(files, routes)
    if not getattr(ctx, "no_image", False):
        save_route_figures(files, routes, ctx)

    dat_params = read_dat_params(files.dat)
    div_num = dat_params.get("T")
    if div_num is None:
        raise ValueError(f"dat ファイルから param T を取得できません: {files.dat}")

    # Step3: 各ルートの正常性検査 → ルート数だけ検査回数加算
    item.inspection_count += len(routes)
    ctx.log(f"Step3: route inspections +{len(routes)} (subtotal={item.inspection_count})")

    hit_routes = [r for r in routes if route_contains_any_fault(r, item.fault_edges)]
    if not hit_routes:
        raise RuntimeError(f"故障を含むルートが見つかりませんでした: {files.seq}")

    parent_dst_rows = load_dst_simple(files.dst)

    # Step4-1: 1ルートのみ故障
    if len(hit_routes) == 1:
        route = hit_routes[0]
        route_faults = route_fault_subset(route, item.fault_edges)
        ctx.log(f"Step4-1: only route k={route.route_id} detected faults {sorted(e.key() for e in route_faults)}")

        if same_topology_as_parent(route, parent_dst_rows):
            extra = inspect_all_links_count(route.edges)
            item.inspection_count += extra
            ctx.log(f"Step4-1a: same topology as parent -> all-link inspection +{extra}")
            return item.inspection_count
        
        # Step4-1bの処理
        if route.edge_count <= div_num:
            if route.edge_count == 1:
                ctx.log(
                    f"Step4-1b: route edge_count=1 <= division={div_num} "
                    "-> single-link route, no additional all-link inspection +0"
                )
                return item.inspection_count

            extra = inspect_all_links_count(route.edges)
            item.inspection_count += extra
            ctx.log(f"Step4-1b: route edge_count={route.edge_count} <= division={div_num} -> all-link inspection +{extra}")
            return item.inspection_count
        child_stem, child_step = next_child_stem(item.stem, route.route_id, ctx)
        child_files = StepFiles(child_stem, child_step, ctx.base_dir)
        old_to_new, chosen_new_depot, reachable_new_depots = build_route_subproblem(files, child_files, route, allowed_depots_orig, ctx)
        if chosen_new_depot is None:
            ctx.log("Step4-2c: 到達可能デポなし → 全リンク検査へ移行")
            add = inspect_all_links_count(route.edges)
            item.inspection_count += add
            ctx.log(f"Step5: all-link inspection +{add} (subtotal={item.inspection_count})")
            return item.inspection_count

        child_dst_rows = load_dst_simple(child_files.dst)
        child_undir = undir_edge_count(child_dst_rows)
        next_div = compute_optimal_division_from_edge_count(undir_edge_count(child_dst_rows))
        write_dat_file(child_files.dat, child_dst_rows, node_num=len(load_pos(child_files.pos)),
                    division_num=next_div, depot_nodes=reachable_new_depots)
        ctx.log(f"Step1: optimal division = {next_div} for undirected edge count {child_undir}")
        run_ilp(child_files, ctx)

        child_faults = {Edge(old_to_new[e.u], old_to_new[e.v]) for e in route_faults if e.u in old_to_new and e.v in old_to_new}
        child_item = WorkItem(
            stem=child_stem,
            parent_stem=item.stem,
            current_step=child_step,
            fault_edges=child_faults,
            inspection_count=0,
            history=item.history + [f"{item.stem}:{route.route_id}"],
        )
        return item.inspection_count + process_work_item(child_item, ctx, allowed_depots_orig)

    # Step4-2: 複数ルートで故障
    ctx.log(f"Step4-2: multiple hit routes = {[r.route_id for r in hit_routes]}")
    total = item.inspection_count

    overlaps = overlapping_edges(hit_routes)
    if overlaps:
        total += len(overlaps)
        ctx.log(f"Step4-2a: overlap edges found -> all-link inspection on overlaps +{len(overlaps)}")
        ctx.log(f"  overlaps = {[f'{u}-{v}' for (u, v) in sorted(overlaps)]}")
    else:
        ctx.log("Step4-2a: no overlap edges")

    # 重複リンク削除後に各ルートを個別処理
    for route in hit_routes:
        ctx.log(
            f"Step4-2 loop: parent subnet stem={item.stem}, step={item.current_step}, now handling route k={route.route_id}"
        )
        cleaned = remove_overlaps_from_route(route, overlaps)

        if not cleaned.edges:
            ctx.log(f"  route k={route.route_id}: route vanished after removing overlaps; faults should be on overlaps only")
            continue

        reachable_depot = choose_reachable_depot(cleaned, allowed_depots_orig)
        if reachable_depot is None:
            # ctx.log(f"{cleaned.edges}")
            extra = inspect_all_links_count(cleaned.edges)
            total += extra
            ctx.log(f"  Step4-2c: route k={route.route_id} has no reachable depot -> all-link inspection +{extra}")
            continue

        # Step4-2c から Step4-1a に戻る扱い
        child_stem, child_step = next_child_stem(item.stem, route.route_id, ctx)
        child_files = StepFiles(child_stem, child_step, ctx.base_dir)
        old_to_new, chosen_new_depot, reachable_new_depots = build_route_subproblem(files, child_files, cleaned, allowed_depots_orig, ctx)
        ctx.log(
            f"  route k={route.route_id}: descend into child subnet stem={child_stem}, step={child_step}"
        )
        if chosen_new_depot is None:
            extra = inspect_all_links_count(cleaned.edges)
            total += extra
            ctx.log(f"  route k={route.route_id}: depot remap failed -> all-link inspection +{extra}")
            continue

        child_dst_rows = load_dst_simple(child_files.dst)
        child_undir = undir_edge_count(child_dst_rows)

        if same_topology_as_parent(cleaned, parent_dst_rows) or len(cleaned.edges) <= div_num or child_undir <= 1:
            extra = inspect_all_links_count(cleaned.edges)
            total += extra
            ctx.log(f"  route k={route.route_id}: direct all-link inspection +{extra}")
            continue

        next_div = compute_optimal_division_from_edge_count(child_undir)
        write_dat_file(child_files.dat, child_dst_rows, node_num=len(load_pos(child_files.pos)),
                       division_num=next_div, depot_nodes=reachable_new_depots)
        ctx.log(f"  Step1(route k={route.route_id}): optimal division = {next_div} for undirected edge count {child_undir}")
        run_ilp(child_files, ctx)
        child_faults = {
            Edge(old_to_new[e.u], old_to_new[e.v])
            for e in item.fault_edges
            if e.u in old_to_new and e.v in old_to_new
        }
        child_item = WorkItem(
            stem=child_stem,
            parent_stem=item.stem,
            current_step=child_step,
            fault_edges=child_faults,
            inspection_count=0,
            history=item.history + [f"{item.stem}:{route.route_id}"],
        )
        total += process_work_item(child_item, ctx, allowed_depots_orig)
        ctx.log(
            f"Step4-2 loop resume: back to parent subnet stem={item.stem}, step={item.current_step} "
            f"after finishing child from route k={route.route_id}"
        )


    return total


# =========================
# CLI
# =========================


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="複数故障対応の分割・検査自動実行")
    p.add_argument("network", help="ベース名。例: cost239-1 を与えると cost239-1.dat/log/dst/pos を初期入力として使う")
    p.add_argument("fault_links", help="故障リンク。例: 1-2,5-6。双方向故障として扱い、どちら向きの通過でも検出する")
    p.add_argument("--mod", default="uiwn_1_md.mod", help="CPLEX/GLPK 用 .mod ファイル名")
    p.add_argument("--ilp-sh", default="obj_ILP.sh", help="ILP 実行シェルスクリプト")
    p.add_argument("--work-dir", default=str(Path(__file__).resolve().parent), help="この python / sh / mod が置かれた作業ディレクトリ")
    p.add_argument("--input-dir-name", default="input", help="初期入力ファイルを置くディレクトリ名")
    p.add_argument("--output-dir-name", default="output", help="途中経過を含む出力ファイルを置くディレクトリ名")
    p.add_argument("--allowed-depots", default="", help="デポにできる元ネットワーク上のノード。例: 2,11。省略時は全ノード可")
    p.add_argument("--start-step", type=int, default=1, help="初期データの step 番号。通常は 1")
    p.add_argument("--next-auto-step", type=int, default=2, help="自動生成する次ステップ番号の開始値")
    p.add_argument("--quiet", action="store_true", help="進捗表示を抑制")
    p.add_argument("--no-image", action="store_true", help="画像生成を行わない")

    p.add_argument(
        "--result-csv",
        default="",
        help=(
            "検査結果を書き込むCSVファイル。"
            "省略時は output_dir/<stem>_multi_result.csv に書き込む"
        ),
    )

    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    work_dir = Path(args.work_dir).resolve()
    input_dir = (work_dir / args.input_dir_name).resolve()
    output_dir = (work_dir / args.output_dir_name).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    mod_file = (work_dir / args.mod).resolve()
    ilp_sh = (work_dir / args.ilp_sh).resolve()

    if not mod_file.exists():
        raise FileNotFoundError(f"mod ファイルが見つかりません: {mod_file}")
    if not ilp_sh.exists():
        raise FileNotFoundError(f"obj_ILP.sh が見つかりません: {ilp_sh}")
    if not input_dir.exists():
        raise FileNotFoundError(f"input ディレクトリが見つかりません: {input_dir}")

    # network 引数は通常 "cost239" を想定するが、
    # ユーザーが "cost239-1" のような prefix をそのまま渡しても扱えるようにする。
    if re.search(r"-\d+$", args.network):
        initial_stem = args.network.rsplit("-", 1)[0]
        initial_step = int(args.network.rsplit("-", 1)[1])
    else:
        initial_stem = args.network
        initial_step = args.start_step

    input_initial = StepFiles(initial_stem, initial_step, input_dir)
    required_inputs = [input_initial.dat, input_initial.log, input_initial.dst, input_initial.pos]
    missing = [str(p) for p in required_inputs if not p.exists()]
    if missing:
        raise FileNotFoundError("初期入力ファイルが不足しています: " + ", ".join(missing))

    dat_edges = load_dat_edges(input_initial.dat)
    fault_edges = parse_fault_edges(args.fault_links)

    invalid_fault_edges = [e for e in sorted(fault_edges) if e not in dat_edges]
    if invalid_fault_edges:
        raise ValueError(
            "指定された故障リンクは初期 dat ファイルの E に存在しません: "
            + ", ".join(e.key() for e in invalid_fault_edges)
        )

    # 初期入力は output にコピーして，その後の処理は output 側だけで進める
    output_initial = StepFiles(initial_stem, initial_step, output_dir)
    shutil.copy2(input_initial.dat, output_initial.dat)
    shutil.copy2(input_initial.log, output_initial.log)
    shutil.copy2(input_initial.dst, output_initial.dst)
    shutil.copy2(input_initial.pos, output_initial.pos)
    if input_initial.seq.exists():
        shutil.copy2(input_initial.seq, output_initial.seq)

    allowed_depots_orig: set[int]
    if args.allowed_depots.strip():
        allowed_depots_orig = {int(x.strip()) for x in args.allowed_depots.split(",") if x.strip()}
    else:
        initial_pos = load_pos(output_initial.pos)
        allowed_depots_orig = set(initial_pos.keys())

    ctx = RunContext(
        base_dir=output_dir,
        mod_file=mod_file,
        ilp_sh=ilp_sh,
        next_auto_step=args.next_auto_step,
        generated_images=[],
        generated_logs=[],
        verbose=not args.quiet,
        no_image=args.no_image,
    )

    if args.no_image:
        ctx.log("画像生成をスキップします")

    ctx.log(f"work_dir = {work_dir}")
    ctx.log(f"input_dir = {input_dir}")
    ctx.log(f"output_dir = {output_dir}")
    ctx.log(f"initial prefix = {output_initial.prefix}")
    ctx.log(f"faults = {sorted(e.key() for e in fault_edges)}")
    ctx.log(f"allowed depots (original network labels) = {sorted(allowed_depots_orig)}")

    total = process_work_item(
        WorkItem(
            stem=output_initial.stem,
            parent_stem=None,
            current_step=output_initial.step,
            fault_edges=fault_edges,
        ),
        ctx,
        allowed_depots_orig,
    )

    print("")
    print("================ RESULT ================")
    print(f"total inspection count: {total}")
    print("generated images:")
    for p in ctx.generated_images:
        print(f"  {p.relative_to(output_dir)}")
    print("generated logs:")
    for p in ctx.generated_logs:
        print(f"  {p.relative_to(output_dir)}")

    if args.result_csv.strip():
        result_csv = Path(args.result_csv)

        # 相対パスで指定された場合は work_dir 基準にする
        if not result_csv.is_absolute():
            result_csv = work_dir / result_csv
    else:
        result_csv = output_dir / f"{output_initial.stem}_multi_result.csv"

    # 親ディレクトリがなければ作成
    result_csv.parent.mkdir(parents=True, exist_ok=True)

    # 新規作成時だけヘッダを書く
    write_header = not result_csv.exists()

    with result_csv.open("a", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)

        if write_header:
            writer.writerow([
                "network",
                "fault_links",
                "fault_count",
                "total_inspection_count",
                "output_dir",
            ])

        writer.writerow([
            args.network,
            ";".join(sorted(e.key() for e in fault_edges)),
            len(fault_edges),
            total,
            str(output_dir),
        ])

    print(f"result csv appended: {result_csv}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        raise
