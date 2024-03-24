#!/bin/python3

import sys
import re
import os

if len(sys.argv) == 1:
    print("Fatal: No arguments provided. Please provide the IP of the remote host.")
    sys.exit(1)
elif len(sys.argv) > 2:
    print(
        "Fatal: Too many arguments provided. Please provide only the IP of the remote host."
    )
    sys.exit(1)

ssh_path = os.path.join(os.getenv("HOME"), ".ssh", "config")
try:
    with open(ssh_path, "r") as file:
        config = file.read()
        updated_config, updated_count = re.subn(
            r"Host remote\n    HostName [^\n]+",
            f"Host remote\n    HostName {sys.argv[1]}",
            config,
        )
    if updated_count == 0:
        print("Fatal: Cannot find any place to modify in SSH config.")
        sys.exit(1)
    with open(ssh_path, "w") as file:
        file.write(updated_config)
except FileNotFoundError:
    print("Fatal: SSH config file not found.")
    sys.exit(1)
