"""Run the original testcase manifest with a golden observer per scenario."""
import argparse
import json
from pathlib import Path
import unittest
from native_api import set_scenario

ROOT=Path(__file__).resolve().parents[1]

def flatten(suite):
    for item in suite:
        if isinstance(item,unittest.TestSuite):yield from flatten(item)
        else:yield item

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--manifest',type=Path)
    parser.add_argument('--write-manifest',type=Path); args=parser.parse_args()
    if args.manifest:
        names=json.loads(args.manifest.read_text())
        tests=[unittest.defaultTestLoader.loadTestsFromName(name) for name in names]
        cases=list(flatten(unittest.TestSuite(tests)))
    else:
        cases=list(flatten(unittest.defaultTestLoader.discover(str(ROOT/'tests'),'test_*.py')))
    if args.write_manifest:args.write_manifest.write_text(json.dumps([t.id() for t in cases],indent=2))
    result=unittest.TextTestRunner(verbosity=2)
    aggregate=unittest.TestResult()
    for testcase in cases:
        set_scenario(testcase.id())
        outcome=result.run(testcase)
        aggregate.testsRun+=outcome.testsRun
        aggregate.failures.extend(outcome.failures); aggregate.errors.extend(outcome.errors)
        aggregate.skipped.extend(outcome.skipped)
    summary={'total':aggregate.testsRun,'failed':len(aggregate.failures),'errors':len(aggregate.errors),
        'skipped':len(aggregate.skipped),'passed':aggregate.testsRun-len(aggregate.failures)-len(aggregate.errors)-len(aggregate.skipped)}
    print('FIX5_UNITS_SUMMARY '+json.dumps(summary),flush=True)
    return int(not aggregate.wasSuccessful())

if __name__=='__main__':raise SystemExit(main())
