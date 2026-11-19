#!/bin/sh
# Which bus does steering call the Pi's canB? Takes the Pi's canB down for ~2 s and
# reads steering's own bus-health bits (HB_N2 byte 2) on canA. Run with sudo.
ip link set canB down
sleep 1
timeout 1 candump canA,101:7FF > /tmp/bus_check.log 2>&1
ip link set canB up
echo "steering bus-health byte2 while the Pi's canB was down:"
awk '{print $6}' /tmp/bus_check.log | sort | uniq -c
