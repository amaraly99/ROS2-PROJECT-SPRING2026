#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# cpu_sampler.sh — sample one process's CPU% + MEM% at a fixed rate
# until it exits. Replaces the old inline sampler that logged 0.0 by
# grabbing a zombie/wrapper PID and using ps lifetime-average %cpu.
#
#   cpu_sampler.sh <match_pattern> <out_csv> [interval_sec]
#
# <match_pattern>  passed to `pgrep -f` (e.g. the node executable name).
# Writes CSV:  epoch,pid,pcpu,pmem,rss_kb
#
# %cpu is INSTANTANEOUS, computed from /proc/<pid>/stat (utime+stime)
# deltas over the sample interval, normalised to ONE core (top/ps
# convention — a fully-busy single thread reads ~100%, multi-threaded
# work across cores can exceed 100%).
#
# PID resolution is robust: skips zombies (state Z) and processes with
# zero resident memory (launch wrappers / defunct), so it locks onto the
# real, live node.
# ─────────────────────────────────────────────────────────────────
set -u

PATTERN="${1:?usage: cpu_sampler.sh <match_pattern> <out_csv> [interval_sec]}"
OUT="${2:?out csv path required}"
INTERVAL="${3:-1}"

CLK=$(getconf CLK_TCK 2>/dev/null || echo 100)
MEMTOTAL=$(awk '/^MemTotal:/{print $2}' /proc/meminfo 2>/dev/null || echo 0)

# CPU jiffies (utime+stime) for a pid. Parse AFTER the last ')' so a comm
# field containing spaces/parens can't shift the columns: in the remainder
# field 1=state ... field 12=utime, field 13=stime.
cpu_jiffies() {
    local rest
    rest=$(sed 's/.*) //' "/proc/$1/stat" 2>/dev/null) || return 1
    awk '{print $12+$13}' <<<"$rest"
}

proc_state() { sed 's/.*) //' "/proc/$1/stat" 2>/dev/null | awk '{print $1}'; }
proc_rss_kb() { awk '/^VmRSS:/{print $2}' "/proc/$1/status" 2>/dev/null; }
proc_comm()  { cat "/proc/$1/comm" 2>/dev/null; }

# Pick the live node by matching /proc/<pid>/comm — the ACTUAL executable
# name, not the full command line. This is what makes the sampler robust:
# pgrep -f also matches any SHELL whose argv contains the node name (the
# sampler itself, the bench script, an ssh wrapper). comm for those is
# "bash"/"sh", so they're excluded. comm is truncated to 15 chars by the
# kernel, so we accept "$PATTERN" == "$comm" (exact) or a truncation prefix.
resolve_pid() {
    local p st rss comm
    for p in $(pgrep -f "$PATTERN" 2>/dev/null); do
        [[ "$p" == "$$" ]] && continue          # never match ourselves
        comm=$(proc_comm "$p")
        [[ -n "$comm" && ( "$PATTERN" == "$comm" || "$PATTERN" == "$comm"* ) ]] || continue
        st=$(proc_state "$p")
        [[ -z "$st" || "$st" == Z* ]] && continue
        rss=$(proc_rss_kb "$p")
        [[ -z "$rss" || "$rss" -eq 0 ]] && continue
        echo "$p"; return 0
    done
    return 1
}

# Wait up to ~10s for the process to appear.
PID=""
for _ in $(seq 1 20); do
    PID=$(resolve_pid) && [[ -n "$PID" ]] && break
    sleep 0.5
done

echo "epoch,pid,pcpu,pmem,rss_kb" > "$OUT"
if [[ -z "${PID:-}" ]]; then
    echo "cpu_sampler: no live PID matched '$PATTERN' — nothing sampled" >&2
    exit 1
fi
echo "cpu_sampler: sampling pid $PID ('$PATTERN') every ${INTERVAL}s" >&2

prev_j=$(cpu_jiffies "$PID"); prev_t=$(date +%s.%N)
while kill -0 "$PID" 2>/dev/null; do
    sleep "$INTERVAL"
    cur_j=$(cpu_jiffies "$PID") || break
    cur_t=$(date +%s.%N)
    rss=$(proc_rss_kb "$PID"); rss=${rss:-0}

    pcpu=$(awk -v dj="$((cur_j - prev_j))" -v clk="$CLK" \
               -v t0="$prev_t" -v t1="$cur_t" \
        'BEGIN{ dt=t1-t0; if(dt<=0){print "0.0"} else {printf "%.1f",(dj/clk)/dt*100} }')
    pmem=$(awk -v r="$rss" -v m="$MEMTOTAL" \
        'BEGIN{ if(m<=0){print "0.0"} else {printf "%.2f", r/m*100} }')

    echo "$(date +%s.%N),$PID,$pcpu,$pmem,$rss" >> "$OUT"
    prev_j=$cur_j; prev_t=$cur_t
done