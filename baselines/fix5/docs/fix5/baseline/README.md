# FIX4 immutable baseline for FIX5

Captured before firmware modifications from tag `fix4_baseline` (`89bafa2385fc0dfa512d89b6cce4209bab8fa6fc`).
The only preparatory changes were host observers, strict GCC flags and new test runners; controller C behaviour and original physical equations / pass thresholds were unchanged.

* Original units: **94/94**.
* Standard geometry: **34/34**.
* Original high-friction pressure: **9/10**. `pressure_square_-1` stops safely at 22.64 s, reason 9, 0.488 laps. It remains a recorded failure.
* Fixed seeds 10500–10529: **27/30** success, **3** protection STOP, **0** uncontrolled timeout. These assumptions do not prove physical-car stability.
* TRACK front deviation in the 30 seeds: median 10.91 mm, 90th percentile 12.73 mm, maximum 13.8 mm. Total completion/stop time: median 42.82 s, maximum 44.06 s. Straight correction sign flips: median 50, maximum 73.
* GCC syntax: 18 project C units and 63 references; six original compile-only variants pass. ARMCC and physical-car validation not run.

GCC 14.2.0, C99, `-O2 -g -Wall -Wextra -Wdouble-promotion -Werror -ffp-contract=off`. No fast-math or native architecture flags. Float fields are hexadecimal and compare exactly (zero tolerance), integers / enum fields exactly. Timestamp, logging sequence and trace indexes are excluded. Gzip JSONL contains 11,293 unit control calls, 70,500 geometry/pressure calls and 60,534 random calls. `metadata.json` fixes compressed-file SHA256; do not regenerate these golden files to fit algorithm changes.

Commands:

```sh
python scripts/fix5_golden.py compare
python scripts/fix5_stage.py step1
python scripts/fix5_matrix.py
```

Stage comparison preserves every previously successful scenario and rejects new uncontrolled timeout, new protection STOP in normal successful scenarios, or TRACK lateral deterioration exceeding the predeclared allowance `max(2 mm, 25% of baseline)`. A time ratio above 1.30 requires explanation; time alone is not a regression gate. Random disturbances are generated before each run so early stopping cannot change future random draws.
