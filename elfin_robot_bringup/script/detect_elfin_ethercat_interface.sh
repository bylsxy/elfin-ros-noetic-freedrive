#!/usr/bin/env bash

# Print the one Ethernet interface whose EtherCAT slaves match this E05.

set -euo pipefail

SLAVEINFO=${SLAVEINFO:-/opt/ros/noetic/bin/slaveinfo}
EXPECTED_HANS_ID='Man: 0000001a ID: 50440200 Rev: 05132016'
EXPECTED_IO_ID='Man: 00201911 ID: 10003201 Rev: 00000001'
LOCK_FILE=/run/lock/elfin5-hardware.lock

if (( EUID != 0 )); then
    echo "Run this detector as root so slaveinfo can open a raw EtherCAT socket." >&2
    exit 77
fi

for command_name in flock ip timeout; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command is missing: $command_name" >&2
        exit 69
    fi
done

# A second SOEM master can reconfigure the same slaves and invalidate the
# running driver's PDO data. The launcher already owns this lock and marks
# that inherited context; every standalone probe must acquire it itself.
if [[ ${ELFIN_HARDWARE_LOCK_HELD:-0} != 1 ]]; then
    exec 8>"$LOCK_FILE"
    if ! flock -n 8; then
        echo "Refusing to probe while the Elfin hardware stack owns $LOCK_FILE." >&2
        exit 75
    fi
fi

if [[ ! -x "$SLAVEINFO" ]]; then
    echo "slaveinfo is unavailable at $SLAVEINFO" >&2
    exit 69
fi

declare -a candidates=()
declare -a matches=()

for interface_path in /sys/class/net/*; do
    interface=${interface_path##*/}

    [[ "$interface" == lo ]] && continue
    [[ -e "$interface_path/device" ]] || continue
    [[ -d "$interface_path/wireless" ]] && continue
    [[ "$(<"$interface_path/type")" == 1 ]] || continue
    [[ -r "$interface_path/carrier" ]] || continue
    [[ "$(<"$interface_path/carrier")" == 1 ]] || continue

    # Never probe a normal LAN interface. The robot link has carrier but no
    # routed IPv4 address and no default route.
    if ip -o -4 address show dev "$interface" scope global | grep -q .; then
        continue
    fi
    if ip -4 route show default dev "$interface" | grep -q .; then
        continue
    fi

    candidates+=("$interface")
done

if (( ${#candidates[@]} == 0 )); then
    echo "No safe EtherCAT candidate found: every physical Ethernet link is down or carries normal IP traffic." >&2
    exit 69
fi

for interface in "${candidates[@]}"; do
    echo "Checking EtherCAT identities on $interface..." >&2
    output="$(timeout --signal=INT --kill-after=2s 12s "$SLAVEINFO" "$interface" 2>&1 || true)"

    configured_count=$(sed -n 's/^\([0-9][0-9]*\) slaves found and configured\.$/\1/p' <<<"$output")
    slave_count=$(grep -c '^Slave:' <<<"$output" || true)
    hans_names=$(grep -c '^ Name:Hans Robot$' <<<"$output" || true)
    hans_ids=$(grep -c "^ $EXPECTED_HANS_ID$" <<<"$output" || true)
    io_names=$(grep -c '^ Name:F2838x CPU1 EtherCAT Slave$' <<<"$output" || true)
    io_ids=$(grep -c "^ $EXPECTED_IO_ID$" <<<"$output" || true)

    if [[ "$configured_count" == 4 && "$slave_count" == 4 && "$hans_names" == 3 && "$hans_ids" == 3 && "$io_names" == 1 && "$io_ids" == 1 ]]; then
        matches+=("$interface")
    fi
done

if (( ${#matches[@]} == 0 )); then
    echo "No interface matched the expected E05 chain (3 Hans Robot modules plus 1 F2838x I/O slave)." >&2
    exit 69
fi

if (( ${#matches[@]} > 1 )); then
    echo "Refusing to choose between multiple matching EtherCAT interfaces: ${matches[*]}" >&2
    exit 69
fi

printf '%s\n' "${matches[0]}"
