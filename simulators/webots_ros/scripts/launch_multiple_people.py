#!/usr/bin/env python3
import subprocess
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--people_number', type=int)
    args, unknown = parser.parse_known_args()
    people_number = args.people_number
    print("PERSON NUMBER", args)
    if not people_number:
        print("Person number required")
        exit()
    processes = []
    script_name = "people_controller_c.py"
    package_folder = os.path.dirname(os.path.abspath(__file__))
    for i in range(people_number) :
        processes.append(subprocess.Popen(["python3", package_folder + "/" +script_name] + ["--number", str(i + 1)]))

    for process in processes:
        process.wait()