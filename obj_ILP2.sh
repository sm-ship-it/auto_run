#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./make_multiobj_lp_and_cplex.sh <model.mod> <data.dat> <logfile.log>
#
# Example:
#   ./make_multiobj_lp_and_cplex.sh uiwn_2_md.mod cost239-1.dat cplex.log
#
# Output:
#   <data filename without extension>_multiobj.lp
#   Example: cost239-1_multiobj.lp

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <model.mod> <data.dat> <logfile.log>" >&2
  exit 1
fi

MOD_FILE="$1"
DAT_FILE="$2"
LOG_FILE="$3"

DAT_BASE="$(basename "$DAT_FILE")"
DAT_NAME="${DAT_BASE%.*}"
OUT_LP="${DAT_NAME}_multiobj.lp"

TMP_LP="${OUT_LP}.glpk.tmp"
OBJ_TMP="${OUT_LP}.obj.tmp"
EDGES_TMP="${OUT_LP}.edges.tmp"
CPLEX_TMP_LOG="${LOG_FILE}.cplex.tmp"

cleanup() {
  rm -f "$TMP_LP" "$OBJ_TMP" "$EDGES_TMP" "$CPLEX_TMP_LOG"
}
trap cleanup EXIT

if ! command -v glpsol >/dev/null 2>&1; then
  echo "ERROR: glpsol command was not found." >&2
  exit 1
fi

if ! command -v cplex >/dev/null 2>&1; then
  echo "ERROR: cplex command was not found." >&2
  exit 1
fi

# ログファイルを初期化
: > "$LOG_FILE"

# 古いLPを削除
rm -f "$OUT_LP" "$TMP_LP"
rm -f clone*.log 2>/dev/null || true

# 1. GLPKで通常のLPを生成
{
  echo "========== GLPK: generate LP =========="
  echo "model: $MOD_FILE"
  echo "data : $DAT_FILE"
  echo "tmp LP: $TMP_LP"
  echo
} >> "$LOG_FILE"

glpsol --model "$MOD_FILE" --data "$DAT_FILE" --check --wlp "$TMP_LP" 2>&1 | tee -a "$LOG_FILE"


# 2. datからTを読む
T=$(
  awk '
    $1 == "param" && $2 == "T" {
      for (i = 1; i <= NF; i++) {
        if ($i == ":=") {
          v = $(i+1)
          gsub(/;/, "", v)
          print v
          exit
        }
      }
    }
  ' "$DAT_FILE"
)

if [ -z "${T:-}" ]; then
  echo "ERROR: cannot read T from $DAT_FILE" | tee -a "$LOG_FILE" >&2
  exit 1
fi

case "$T" in
  ''|*[!0-9]*)
    echo "ERROR: T is not a positive integer: $T" | tee -a "$LOG_FILE" >&2
    exit 1
    ;;
esac

if [ "$T" -le 0 ]; then
  echo "ERROR: T must be positive: $T" | tee -a "$LOG_FILE" >&2
  exit 1
fi

# 3. datからEを読む
awk '
  /^[[:space:]]*param[[:space:]]*:[[:space:]]*E[[:space:]]*:/ {
    inE = 1
    next
  }
  inE && /^[[:space:]]*;/ {
    exit
  }
  inE && NF >= 2 {
    if ($1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+$/) {
      print $1, $2
    }
  }
' "$DAT_FILE" > "$EDGES_TMP"

if [ ! -s "$EDGES_TMP" ]; then
  echo "ERROR: cannot read E links from $DAT_FILE" | tee -a "$LOG_FILE" >&2
  exit 1
fi

# 4. 複数目的関数の先頭部分を作る
{
  echo "Minimize multi-objectives"
  echo "OBJ1: Priority=2 Weight=1 AbsTol=0.0 RelTol=0.0"

  while read -r i j; do
    k=1
    while [ "$k" -le "$T" ]; do
      echo "+ 0.5 x_or($i,$j,$k)"
      k=$((k + 1))
    done
  done < "$EDGES_TMP"

  echo "OBJ2: Priority=1 Weight=1 AbsTol=0.0 RelTol=0.0"
  echo "+ Lmax"
  echo ""
  echo "Subject To"
} > "$OBJ_TMP"

# 5. LPの Minimize ～ Subject To を差し替える
awk -v objfile="$OBJ_TMP" '
  BEGIN {
    while ((getline line < objfile) > 0) {
      obj = obj line "\n"
    }
    close(objfile)
    skipping = 0
  }
  /^[[:space:]]*Minimize[[:space:]]*$/ {
    printf "%s", obj
    skipping = 1
    next
  }
  skipping && /^[[:space:]]*Subject To[[:space:]]*$/ {
    skipping = 0
    next
  }
  !skipping {
    print
  }
' "$TMP_LP" > "$OUT_LP"

EDGE_COUNT=$(wc -l < "$EDGES_TMP" | tr -d ' ')
TERM_COUNT=$((EDGE_COUNT * T))

{
  echo
  echo "========== multi-objective LP created =========="
  echo "created LP: $OUT_LP"
  echo "T = $T"
  echo "|E| = $EDGE_COUNT"
  echo "OBJ1 terms = $TERM_COUNT"
  echo
} >> "$LOG_FILE"

# 6. CPLEXでLPを読み込み、最適化し、解を表示
{
  echo "========== CPLEX: optimize =========="
  echo "read LP: $OUT_LP"
  echo
} >> "$LOG_FILE"

cplex -c \
  "set logfile $CPLEX_TMP_LOG" \
  "set output clonelog -1" \
  "read $OUT_LP" \
  "optimize" \
  "display solution variable -" 2>&1 | tee -a "$LOG_FILE"

cleanup() {
  rm -f "$TMP_LP" "$OBJ_TMP" "$EDGES_TMP" "$CPLEX_TMP_LOG"
}
trap cleanup EXIT

{
  echo
  echo "finished"
  echo "LP : $OUT_LP"
  echo "LOG: $LOG_FILE"
} | tee -a "$LOG_FILE"
