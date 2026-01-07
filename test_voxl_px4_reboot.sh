#!/bin/bash
#
# Reboot stress test for voxl-px4 systemd service
#

# Default connection method
use_ssh=false
ssh_host=""
ssh_pass=""

# Usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -s, --ssh <host>      Use SSH instead of ADB (e.g., --ssh 192.168.1.10)"
    echo "  -p, --password <pwd>  SSH password (default: voxl123, requires sshpass)"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                              # Use ADB (default)"
    echo "  $0 --ssh 192.168.1.10           # SSH with default password"
    echo "  $0 -s 192.168.1.10 -p mypass    # SSH with custom password"
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--ssh)
            use_ssh=true
            ssh_host="$2"
            shift 2
            ;;
        -p|--password)
            ssh_pass="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# Validate SSH host if SSH mode is enabled
if $use_ssh && [ -z "$ssh_host" ]; then
    echo "Error: SSH mode requires a host address"
    usage
fi

# Check for sshpass if password is provided
if [ -n "$ssh_pass" ]; then
    if ! command -v sshpass &> /dev/null; then
        echo "Error: sshpass is required for password authentication"
        echo "Install with: sudo apt install sshpass"
        exit 1
    fi
fi

# Add root@ prefix if not already present
if $use_ssh && [[ "$ssh_host" != *@* ]]; then
    ssh_host="root@$ssh_host"
fi

# Default password for SSH if not specified
if $use_ssh && [ -z "$ssh_pass" ]; then
    ssh_pass="voxl123"
fi

# Build SSH command prefix
ssh_cmd="ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no"
if [ -n "$ssh_pass" ]; then
    ssh_cmd="sshpass -p '$ssh_pass' $ssh_cmd"
fi

# Helper functions for remote commands
remote_shell() {
    if $use_ssh; then
        eval "$ssh_cmd \"$ssh_host\" \"$@\""
    else
        adb shell "$@"
    fi
}

remote_reboot() {
    if $use_ssh; then
        # Use nohup with delayed reboot to avoid SSH session hanging
        eval "$ssh_cmd \"$ssh_host\" \"nohup bash -c 'sleep 1; reboot' > /dev/null 2>&1 &\""
    else
        adb shell reboot
    fi
}

wait_for_device() {
    if $use_ssh; then
        # Extract just the hostname/IP from user@host
        host_only="${ssh_host#*@}"
        echo "Waiting for $host_only to become reachable..."
        # Build wait command with optional sshpass
        # Note: BatchMode=yes conflicts with sshpass, only use it for key-based auth
        if [ -n "$ssh_pass" ]; then
            wait_cmd="sshpass -p '$ssh_pass' ssh -o ConnectTimeout=2 -o StrictHostKeyChecking=no"
        else
            wait_cmd="ssh -o ConnectTimeout=2 -o StrictHostKeyChecking=no -o BatchMode=yes"
        fi
        while ! eval "$wait_cmd \"$ssh_host\" exit" 2>/dev/null; do
            sleep 1
        done
    else
        adb wait-for-device
    fi
}

# Counters
successes=0
failures=0
iterations=0

# Trap Ctrl+C for clean exit
cleanup() {
    echo ""
    echo "========================================"
    echo "Test interrupted by user"
    echo "========================================"
    echo "Final Results:"
    echo "  Iterations: $iterations"
    echo "  Successes:  $successes"
    echo "  Failures:   $failures"
    if [ $iterations -gt 0 ]; then
        success_rate=$(echo "scale=1; $successes * 100 / $iterations" | bc)
        echo "  Success Rate: ${success_rate}%"
    fi
    echo "========================================"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "========================================"
echo "voxl-px4 Reboot Stress Test"
if $use_ssh; then
    echo "Connection: SSH ($ssh_host)"
else
    echo "Connection: ADB"
fi
echo "Press Ctrl+C to stop"
echo "========================================"
echo ""

while true; do
    ((iterations++))
    echo "--- Iteration $iterations ---"

    # Reboot the target
    echo "Rebooting target..."
    remote_reboot
    sleep 2

    # Wait for device to come back
    echo "Waiting for device..."
    wait_for_device

    # Wait for voxl-px4 service to start
    echo "Waiting 8 seconds for voxl-px4 service to start..."
    sleep 8

    # Check the service status
    status_line=$(remote_shell systemctl status voxl-px4 | grep "Active: ")

    if echo "$status_line" | grep -q "active (running)"; then
        ((successes++))
        echo "Result: SUCCESS - voxl-px4 is running"
    else
        ((failures++))
        echo "Result: FAILURE - $status_line"
        exit -1
    fi

    # Print tally
    echo ""
    echo "Tally: $successes successes / $failures failures (out of $iterations iterations)"
    echo ""
done
