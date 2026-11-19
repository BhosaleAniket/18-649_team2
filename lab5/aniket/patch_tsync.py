import sys
p = sys.argv[1]; s = open(p).read()
# 1) age check: a matched RX entry must be from the last 3 REF periods
old = "static bool find_rx(const tsync_t *t, int b, uint8_t cycle, bool want_relayed, uint64_t *local)\n{\n    for (int i = 0; i < 4; i++) {\n        if (t->rx[b][i].used && t->rx[b][i].cycle == cycle && t->rx[b][i].relayed == want_relayed) {"
new = "static uint64_t g_rx_now;\nstatic bool find_rx(const tsync_t *t, int b, uint8_t cycle, bool want_relayed, uint64_t *local)\n{\n    for (int i = 0; i < 4; i++) {\n        if (t->rx[b][i].used && t->rx[b][i].cycle == cycle && t->rx[b][i].relayed == want_relayed &&\n            g_rx_now - t->rx[b][i].local < 3u * t->period_us) {"
assert old in s; s = s.replace(old, new)
old2 = "    int b = bus - 1;\n    uint8_t cycle = d[2];\n"
assert old2 in s; s = s.replace(old2, old2 + "    g_rx_now = rx_local;\n", 1)
# 2) leader alternates the stamp bus: A on even cycles, B on odd (falls back to the other)
old3 = "    for (int b = 0; b < 2; b++) {\n        if (t->stamp_valid[b] && t->stamp_cycle[b] == prev) {"
new3 = "    for (int k = 0; k < 2; k++) {\n        int b = (k + (t->cycle & 1)) & 1;\n        if (t->stamp_valid[b] && t->stamp_cycle[b] == prev) {"
if "ALT" in sys.argv:
    assert old3 in s; s = s.replace(old3, new3)
open(p, "w").write(s)
