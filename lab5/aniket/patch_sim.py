import sys
p = sys.argv[1]
s = open(p).read()
i = s.index('failure%s)')
i = s.rindex('\n', 0, i) + 1
exp = r'''
    /* ---------------- EXPERIMENT: follower off bus A, leader on both ---------------- */
    printf("X. Pi leads on both buses, drivetrain off bus A for 60 s\n");
    reset_world();
    boot(1, 1, 0); boot(2, 1, 0); boot(3, 1, 0);
    run_until(5000000);
    uint32_t p3 = N[3].ts.steps + N[3].ts.slews, p2 = N[2].ts.steps + N[2].ts.slews;
    run_until(15000000);
    printf("      healthy 10 s: sync pairs N2 %u, N3 %u (500 REFs sent)\n",
           N[2].ts.steps + N[2].ts.slews - p2, N[3].ts.steps + N[3].ts.slews - p3);
    N[3].wire[0] = false;
    measure_spread = true; spread_max = 0;
    p3 = N[3].ts.steps + N[3].ts.slews; p2 = N[2].ts.steps + N[2].ts.slews;
    run_until(75000000);
    printf("      N3 off A 60 s: sync pairs N2 %u, N3 %u (3000 REFs sent); spread max %.0f us\n",
           N[2].ts.steps + N[2].ts.slews - p2, N[3].ts.steps + N[3].ts.slews - p3, spread_max);
'''
open(p, 'w').write(s[:i] + exp + s[i:])
