#!/usr/bin/python3

import yaml
import sys

def main():

    if len(sys.argv) == 5:
        file_path = sys.argv[1]
        variant = sys.argv[2]
        repo_name = sys.argv[4]
    else:
        return

    with open(file_path, "r") as file:

        try:
            data = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)

        properties = data[repo_name]

        architecture = properties['architecture']

        url = properties['source']

        refs = properties['git_refs']

        amd = False
        arm = False
        gitman = False

        if "arm64" in architecture:
            arm = True

        if "amd64" in architecture:
            amd = True

        if variant == "stable":
            ref = refs['stable']
        elif variant == "unstable":
            ref = refs['unstable']
        else:
            ref = refs['testing']

        try:
            gitman = bool(properties['gitman'])
        except:
            pass

        print("{} {} {} {}, {}, {}".format(repo_name, url, ref, gitman, amd, arm))

if __name__ == '__main__':
    main()
