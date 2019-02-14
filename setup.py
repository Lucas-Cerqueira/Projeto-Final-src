#! /usr/bin/env python
import os
import sys
from util import run_command, fatal, CommandError

def main():
    import os
    print(os.getcwd())
    try:
        run_command(['cp -r src/* ns-3-dev/src/'], shell=True)
        run_command(['cp -r scripts/* ns-3-dev/scratch/.'], shell=True)
        if (not os.path.isdir('./output')):
            run_command(['mkdir', 'output'])
    except CommandError:
        print ("Run 'download.py' before running this script.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
