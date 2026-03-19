#!/usr/bin/env python3

import datetime
import os
import signal
import subprocess
import sys


def main():
	timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
	output_name = f"imu_data_{timestamp}"

	cmd = [
		"rosbag",
		"record",
		"/imu/data",
		"--duration=3h",
		"-O",
		output_name,
	]

	print("Start recording topic /imu/data")
	print(f"Output: {output_name}.bag")
	print("Auto stop after 3 hours")

	process = subprocess.Popen(cmd)

	def handle_signal(signum, _frame):
		print(f"\nReceive signal {signum}, stopping recorder...")
		if process.poll() is None:
			process.send_signal(signal.SIGINT)

	signal.signal(signal.SIGINT, handle_signal)
	signal.signal(signal.SIGTERM, handle_signal)

	return_code = process.wait()
	if return_code == 0:
		print("Recording finished")
	else:
		print(f"Recorder exited with code: {return_code}")

	return return_code


if __name__ == "__main__":
	sys.exit(main())
