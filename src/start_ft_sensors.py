#!/usr/bin/env python3

import subprocess

def main():
    # List of commands to run as subprocesses
    commands = [
        ["rosrun", "netft_utils", "netft_node", f"--address=192.168.56.6", f"--sensor_nr=1"],
        ["rosrun", "netft_utils", "netft_node", f"--address=192.168.56.5", f"--sensor_nr=2"]
    ]

    processes = []
    
    # Start each command as a subprocess
    for cmd in commands:
        print(f"Starting subprocess: {' '.join(cmd)}")
        p = subprocess.Popen(cmd)
        processes.append(p)

    # Wait for all subprocesses to complete
    for p in processes:
        p.wait()

    


if __name__ == "__main__":
    main()