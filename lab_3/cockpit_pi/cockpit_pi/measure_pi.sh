#!/bin/sh
# measure_pi.sh - the two Part 3.3 Pi-stage runs, 70 s each (handout: at least a minute)
#   sudo sh measure_pi.sh            # needs the proxy streaming at --interval-ms 20
# Keep the scope on UDP_RX (GPIO17, pin 11) and CMD_TX (GPIO27, pin 13) during both
# runs and save a capture / CSV of each. latency_idle.csv and latency_stress.csv are
# the software cross-check (python3 ../analysis/pi_latency.py latency_stress.csv --cockpit).
set -e
RT=${RT:---rt}

echo "== run 1: idle =="
python3 cockpit.py $RT --duration 70 --latency-csv latency_idle.csv < /dev/null

echo "== run 2: under stress-ng (sudo apt install stress-ng) =="
stress-ng --cpu 4 --io 2 --vm 1 --vm-bytes 256M --timeout 80s >/dev/null 2>&1 &
STRESS=$!
sleep 2
python3 cockpit.py $RT --duration 70 --latency-csv latency_stress.csv < /dev/null
wait $STRESS || true
echo "done: latency_idle.csv latency_stress.csv"
