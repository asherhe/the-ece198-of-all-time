"""Run the Python recorder without a notebook."""

import argparse
import time

from .recorder import LiveRecorder


def main() -> None:
    parser = argparse.ArgumentParser(description="Record SerialAnalysis board signals")
    parser.add_argument("--port", required=True, help="COM3 or /dev/ttyACM0")
    parser.add_argument("--output", default="sessions")
    parser.add_argument("--duration-min", type=float, default=30)
    parser.add_argument("--status-interval", type=float, default=5, help="Seconds between rate reports")
    args = parser.parse_args()
    if args.status_interval <= 0:
        parser.error("--status-interval must be positive")
    with LiveRecorder(args.port, args.output, args.duration_min) as recorder:
        print(f"Recording to {recorder.session_dir} (Ctrl+C to stop)")
        last_status = time.monotonic()
        last_counts = (0, 0, 0)
        try:
            while not recorder.done:
                recorder.poll()
                now = time.monotonic()
                if now - last_status >= args.status_interval:
                    elapsed = now - last_status
                    counts = (recorder.ecg_samples, recorder.accel_samples, recorder.optical_samples)
                    rates = [round((count - previous) / elapsed, 1)
                             for count, previous in zip(counts, last_counts)]
                    print(f"rates Hz ECG/accel/optical={rates} stats={recorder.stats()}", flush=True)
                    last_counts, last_status = counts, now
        except KeyboardInterrupt:
            pass
        print(f"Finished: {recorder.session_dir}")


if __name__ == "__main__":
    main()
