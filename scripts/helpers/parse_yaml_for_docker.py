#!/usr/bin/python3

import yaml
import sys

def main():

    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        return

    with open(file_path, "r") as file:

        try:
            data = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)

        for package in data:

            properties = data[package]

            architecture = properties['architecture']

            url = properties['source']

            docker = False
            amd = False
            arm = False

            if "arm64" in architecture:
                arm = True

            if "amd64" in architecture:
                amd = True

            refs = properties['git_refs']

            try:
                docker = bool(properties['generate_docker_image'])
            except:
                pass

            print("{} {} {} {} {}".format(package, url, docker, amd, arm))

if __name__ == '__main__':
    main()
