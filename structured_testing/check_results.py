import argparse
import yaml
import os
import sys


def getResults(yaml_file):
    with open(yaml_file, "r") as stream:
        results_lst = yaml.safe_load(stream)

    if os.stat(yaml_file).st_size == 0:
        raise Exception("Provided results file is empty")

    tests_pass = True
    for observer in results_lst:
        if observer["status"] == 'FAIL':
            tests_pass = False

    return tests_pass


def main(simResultsFile):
    test_status = getResults(simResultsFile)

    if test_status:
        print("All tests pass")
        sys.exit(0)
    else:
        print("At least one integration test has failed")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "simResultsFile", help="The .simresults file showing results of previous simulation"
    )
    args = parser.parse_args()
    main(args.simResultsFile)
