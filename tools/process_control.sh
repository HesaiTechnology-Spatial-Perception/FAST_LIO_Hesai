#!/usr/bin/env bash
# Children are started by launch_process.py in their own process groups.
process_alive() {
    local pid="$1" stat_file record state parent group rest
    kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null || return 1
    # kill -0 also succeeds for zombies. Minimal container PID 1 processes may
    # leave exited grandchildren unreaped; those cannot receive another signal.
    [[ -d /proc/self ]] || return 0
    if { IFS= read -r record < "/proc/$pid/stat"; } 2>/dev/null; then
        read -r state parent group rest <<< "${record##*) }"
        [[ "$state" == Z || "$state" == X ]] || return 0
    fi
    # The leader can exit before a live descendant. Inspect the whole owned
    # group before allowing wait/cleanup to finish, ignoring only dead tasks.
    for stat_file in /proc/[0-9]*/stat; do
        { IFS= read -r record < "$stat_file"; } 2>/dev/null || continue
        read -r state parent group rest <<< "${record##*) }"
        if [[ "$group" == "$pid" && "$state" != Z && "$state" != X ]]; then
            return 0
        fi
    done
    return 1
}

stop_process() {
    local pid="$1" int_seconds="${2:-30}" term_seconds="${3:-5}"
    local signal grace attempt forced=false code=0
    [[ -n "$pid" ]] || return 0
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 2
    for signal in INT TERM KILL; do
        process_alive "$pid" || break
        [[ "$signal" == INT ]] || forced=true
        kill -"$signal" -- "-$pid" 2>/dev/null || kill -"$signal" "$pid" 2>/dev/null || true
        case "$signal" in INT) grace="$int_seconds";; TERM) grace="$term_seconds";; KILL) grace=1;; esac
        for ((attempt=0; attempt<grace*10; attempt++)); do
            process_alive "$pid" || break
            sleep 0.1
        done
    done
    if process_alive "$pid"; then
        printf '[ERROR] Process group %s did not exit after KILL; cleanup will continue.\n' "$pid" >&2
        return 124
    fi
    # Never wait while a process/group is still alive (including uninterruptible I/O).
    wait "$pid" 2>/dev/null || code=$?
    if [[ "$forced" == true ]]; then
        printf '[ERROR] Process group %s required forced termination.\n' "$pid" >&2
        return 124
    fi
    return "$code"
}

wait_live() {
    local mapper="$1" driver="$2" code=0
    while kill -0 "$mapper" 2>/dev/null && kill -0 "$driver" 2>/dev/null; do
        sleep 0.2
    done
    if ! kill -0 "$driver" 2>/dev/null; then
        wait "$driver" || code=$?
        printf '[ERROR] Live driver stopped (exit %s); see driver.log.\n' "$code" >&2
        [[ "$code" != 0 ]] || code=1
        return "$code"
    fi
    wait "$mapper"
}
