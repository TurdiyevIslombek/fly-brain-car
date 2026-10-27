# Run logs, Oct 24, 2026

All 12 runs from Oct 24 are here. One CSV per run; one row per control step.
Columns: distances in 5 directions (cm), eye input (Hz), steering and moonwalker firing (Hz),
wheel commands, `reflex` (1 = the stuck reflex drove, not the brain), camera values (stage 3),
`vcc_mv` (the Arduino's own 5 V reading), and events.

| Session | File | Length | Notes |
|---|---|---|---|
| 16:10 (not in the video) | stage1_20261024_161017 | 56.5 s | |
| | stage2_20261024_161115 | 13.6 s | |
| | stage3_20261024_161128 | 6.3 s | |
| | stage3_swapped_20261024_161135 | 8.5 s | |
| | stage1_swapped_20261024_161144 | 4.7 s | |
| 16:34 (in the video) | stage1_20261024_163442 | 73.3 s | |
| | stage2_20261024_163616 | 84.2 s | the stage 2 numbers in the README |
| | stage3_20261024_163809 | 26.0 s | |
| | stage3_20261024_163915 | 43.3 s | |
| | stage3_20261024_164029 | 55.0 s | |
| | stage3_20261024_164146 | 1.0 s | |
| | stage3_swapped_20261024_164147 | 50.8 s | eyes swapped |

The README reports the runs in the video that are longer than 20 s. Stage 1 has no brain, so swapping its eyes changes nothing. Share of control steps driven by the fly circuit,
all runs: stage 2: 72.7% (84 s), 100% (14 s); stage 3: 78.1%, 63.4%, 74.4% (in the video), 70.7% (6 s), 100% (1 s);
stage 3 swapped: 60.7% (in the video), 61.8% (8.5 s).
