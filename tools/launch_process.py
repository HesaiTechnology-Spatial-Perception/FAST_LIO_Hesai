#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Start one owned process group with normal signals, including on Ubuntu 20.04."""
import os
import signal
import sys

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("Usage: launch_process.py PROGRAM [ARG ...]")
    os.setsid()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.SIG_DFL)
    os.execvp(sys.argv[1], sys.argv[1:])
