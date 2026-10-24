#!/bin/sh
# setup_can.sh - bring up both CAN interfaces for Lab 3 (run with sudo after every boot)
#
# Bus A = MCP2515 on CE0 (spi0.0) -> canA,  bus B = MCP2515 on CE1 (spi0.1) -> canB.
# The kernel numbers the interfaces (can0, can1) in probe order, which can swap between
# boots (one boot gave can0 to CE1), so this script finds each interface by its SPI device
# and renames it. cockpit.py expects canA / canB.
#
# One-time, in /boot/firmware/config.txt, then reboot:
#   dtparam=spi=on
#   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
#   dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24
# One-time packages:
#   sudo apt install can-utils python3-can python3-lgpio
#
# Exit status is non-zero if a module is missing (check its wiring and reboot).

find_iface() {              # $1 = SPI device (spi0.0): print the net interface bound to it
    for d in /sys/class/net/*; do
        [ -e "$d/device" ] || continue
        if [ "$(basename "$(readlink -f "$d/device")")" = "$1" ]; then
            basename "$d"
            return 0
        fi
    done
    return 1
}

rc=0
for pair in "spi0.0 canA" "spi0.1 canB"; do
    set -- $pair
    spi=$1
    want=$2
    cur=$(find_iface "$spi") || {
        echo "ERROR: no CAN interface on $spi ($want): module not probed - check CS, 3V3, GND and the" >&2
        echo "       shared SCK/MOSI/MISO wires, then reboot (dmesg | grep mcp251)" >&2
        rc=1
        continue
    }
    ip link set "$cur" down 2>/dev/null || true
    if [ "$cur" != "$want" ]; then
        ip link set "$cur" name "$want" || { echo "ERROR: cannot rename $cur to $want" >&2; rc=1; continue; }
    fi
    # restart-ms: automatic bus-off recovery (R4.8 recovery within 100 ms)
    ip link set "$want" type can bitrate 500000 restart-ms 100
    # txqueuelen 4: few frames can pile up while a jumper is pulled
    ip link set "$want" txqueuelen 4
    ip link set "$want" up
    echo "$spi -> $want"
    ip -details link show "$want" | grep -E "state|bitrate"
done
exit $rc
