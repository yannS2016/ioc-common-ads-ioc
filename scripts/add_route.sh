#!/bin/bash
# Usage: add_route.sh [plc-hostname] [ip-regex]

if [ $# -ne 2 ]; then
    echo "Usage: $0 [plc hostname] [ip regex]" > /dev/stderr
    exit 1
fi

CONDA_BIN=${CONDA_BIN=/cds/group/pcds/pyps/conda/py39/envs/pcds-5.4.1/bin/conda}
HOST_VARS=${HOST_VARS=/cds/group/pcds/tcbsd/twincat-bsd-ansible/host_vars}

if [ ! -f "$CONDA_BIN" ]; then
    echo "Conda unavailable. Skipping route addition." > /dev/stderr
    exit 1
fi

set -e

add_route() {
    local plc_host
    local ioc_hostname
    local ioc_host_ip

    plc_host=$1
    ioc_hostname=$2
    ioc_host_ip=$3
    ioc_host_net_id="${ioc_host_ip}.1.1"

    if [ -z "$plc_host" ]; then
        echo "PLC hostname unspecified" > /dev/stderr
        exit 1
    fi

    if [ -z "$ioc_host_ip" ]; then
        echo "IOC IP address unspecified" > /dev/stderr
        exit 1
    fi

    if [ -z "$ioc_hostname" ]; then
        echo "IOC hostname unspecified" > /dev/stderr
        exit 1
    fi

    echo "Running ads-async to add a route to ${ioc_hostname} for IOC host ${ioc_hostname} (${ioc_host_ip})..."
    set -x
    $CONDA_BIN run ads-async route \
        --route-name="${ioc_hostname}" \
        "${plc_host}" \
        "${ioc_host_net_id}" \
        "${ioc_host_ip}"
    { set +x; } 2> /dev/null
    echo "Done."
}

find_ioc_ip() {
    local ipaddr
    local regex
    regex=$1

    if [ -z "$regex" ]; then
        echo "IOC IP match regex unspecified" > /dev/stderr
        exit 1
    fi

    for ipaddr in $(hostname -I); do
        if [[ "$ipaddr" =~ ${regex} ]]; then
            echo "$ipaddr"
            return
        fi
    done
}

check_host_vars() {
    local ip_or_hostname
    local host_info
    local hostname

    ip_or_hostname=$1

    if [ ! -d "$HOST_VARS" ]; then
        echo "Host vars unavailable. Cannot check if we should skip add_route.sh"
        return 0
    fi

    host_info="$(getent hosts "$ip_or_hostname")"
    rval=$?
    if [ $rval -ne 0 ]; then
        echo "Error getting host information, cannot add_route."
        return 1
    fi
    hostname="$(echo "$host_info" | cut -d " " -f 2- | tr -d " " | cut -d "." -f 1)"
    if [ -d "$HOST_VARS/$hostname" ]; then
        # PLCs managed in our HOST_VARS folder have been set up via standard procedure
        # which means this automatic route script is unlikely to work (with timeout)
        echo "Host likely has a real admin password, skipping add route to save time."
        return 1
    else
        echo "Add route likely to work, trying..."
        return 0
    fi
}


plc_host=$1
host_ip_regex=$2
if ! check_host_vars "$plc_host"; then
    exit 0
fi
ioc_hostname=$(hostname -s)
ioc_ip=$(find_ioc_ip "${host_ip_regex}")

add_route "${plc_host}" "${ioc_hostname}" "${ioc_ip}"
