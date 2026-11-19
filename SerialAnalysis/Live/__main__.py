"""Run the Python recorder without a notebook."""

import argparse
import time

from .recorder import LiveRecorder


def main() -> None:
    parser = argparse.ArgumentParser(description="Record SerialAnalysis board signals")
    parser.add_argument("--port", required=True, help="COM3 or /dev/ttyACM0")
    parser.add_argument("--output", default="sessions")
    parser.add_argument("--duration-min", type=float, default=30)
    args = parser.parse_args()
    with LiveRecorder(args.port, args.output, args.duration_min) as recorder:
        print(f"Recording to {recorder.session_dir} (Ctrl+C to stop)")
        last_status = time.monotonic()
        try:
            while not recorder.done:
                recorder.poll()
                if time.monotonic() - last_status >= 10:
                    print(recorder.stats())
                    last_status = time.monotonic()
        except KeyboardInterrupt:
            pass
        print(f"Finished: {recorder.session_dir}")


if __name__ == "__main__":
    main()
