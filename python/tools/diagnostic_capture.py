#!/usr/bin/env python3
"""Capture AirBridge live TCE data to the Chromebook for diagnostics.

This is deliberately a development tool, not a clinical/long-term recorder.
It consumes AirBridge's existing /events/live SSE stream, writes the 25 Hz
pressure/flow/leak samples to CSV, polls /api/status, and lets the operator
type free-form ground-truth markers during controlled tests.

No third-party Python packages are required.
"""

from __future__ import annotations

import argparse
import base64
import csv
import getpass
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SAMPLE_PERIOD_S = 0.040  # TCE is 25 Hz
STATUS_PERIOD_S = 5.0
SUMMARY_PERIOD_S = 2.0


def iso_now(epoch: float | None = None) -> str:
    if epoch is None:
        epoch = time.time()
    return datetime.fromtimestamp(epoch, tz=timezone.utc).astimezone().isoformat(timespec="milliseconds")


class Capture:
    def __init__(self, base_url: str, username: str, password: str, out_dir: Path) -> None:
        self.base_url = base_url.rstrip("/")
        token = base64.b64encode(f"{username}:{password}".encode("utf-8")).decode("ascii")
        self.headers = {
            "Authorization": f"Basic {token}",
            "User-Agent": "cpap-diagnostic-capture/1",
        }
        self.out_dir = out_dir
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.last_seq: int | None = None
        self.latest: deque[tuple[float, float, float, float]] = deque(maxlen=100)
        self.sample_rows = 0
        self.gaps = 0

        out_dir.mkdir(parents=True, exist_ok=True)

        self.samples_fp = (out_dir / "samples.csv").open("w", newline="", buffering=1)
        self.samples = csv.writer(self.samples_fp)
        self.samples.writerow([
            "sample_time",
            "received_time",
            "seq",
            "mkp_raw",
            "rfl_raw",
            "lyk_raw",
            "pressure_cmH2O",
            "flow_L_min",
            "leak_L_min",
        ])

        self.markers_fp = (out_dir / "markers.csv").open("w", newline="", buffering=1)
        self.markers = csv.writer(self.markers_fp)
        self.markers.writerow(["time", "seq", "note"])

        self.status_fp = (out_dir / "status.jsonl").open("w", buffering=1)

        meta = {
            "started_at": iso_now(),
            "source": self.base_url,
            "sample_rate_hz": 25,
            "notes": (
                "Temporary diagnostic capture from AirBridge /events/live. "
                "Sample timestamps are estimated from SSE batch receipt time; "
                "raw TCE values are retained for exact replay."
            ),
            "scaling": {
                "pressure_cmH2O": "MKP / 50",
                "flow_L_min": "RFL / 500 L/s * 60",
                "leak_L_min": "LYK / 50 L/s * 60",
            },
        }
        (out_dir / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")

    def request(self, path: str, *, accept: str | None = None, timeout: float | None = None):
        headers = dict(self.headers)
        if accept:
            headers["Accept"] = accept
        req = urllib.request.Request(self.base_url + path, headers=headers)
        if timeout is None:
            return urllib.request.urlopen(req)
        return urllib.request.urlopen(req, timeout=timeout)

    def run_status(self) -> None:
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                with self.request("/api/status", accept="application/json", timeout=5) as resp:
                    body = resp.read().decode("utf-8")
                    payload: Any = json.loads(body)
                    record = {"captured_at": iso_now(), "status": payload}
                    self.status_fp.write(json.dumps(record, separators=(",", ":")) + "\n")
            except urllib.error.HTTPError as exc:
                if exc.code == 401:
                    print("\nStatus polling: authentication failed (HTTP 401).", file=sys.stderr)
                    self.stop.set()
                    return
                print(f"\nStatus polling HTTP error: {exc}", file=sys.stderr)
            except Exception as exc:
                print(f"\nStatus polling error: {exc}", file=sys.stderr)

            remaining = STATUS_PERIOD_S - (time.monotonic() - started)
            self.stop.wait(max(0.2, remaining))

    def run_markers(self) -> None:
        # stdin blocking is fine in a daemon thread; Ctrl+C in main stops capture.
        while not self.stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if line == "":
                return
            note = line.strip()
            if not note:
                continue
            with self.lock:
                seq = "" if self.last_seq is None else self.last_seq
            self.markers.writerow([iso_now(), seq, note])
            self.markers_fp.flush()
            print(f"[MARK] {note} (seq={seq})", flush=True)

    def run_summary(self) -> None:
        while not self.stop.wait(SUMMARY_PERIOD_S):
            cutoff = time.time() - 2.0
            with self.lock:
                rows = [row for row in self.latest if row[0] >= cutoff]
                seq = self.last_seq
                count = self.sample_rows
                gaps = self.gaps
            if not rows:
                print(f"[LIVE] no TCE samples yet | rows={count} gaps={gaps}", flush=True)
                continue
            pressures = [r[1] for r in rows]
            flows = [r[2] for r in rows]
            leaks = [r[3] for r in rows]
            print(
                "[LIVE] "
                f"P={sum(pressures)/len(pressures):5.2f} cmH2O  "
                f"Leak avg/max={sum(leaks)/len(leaks):5.1f}/{max(leaks):5.1f} L/min  "
                f"Flow min/max={min(flows):6.1f}/{max(flows):6.1f} L/min  "
                f"seq={seq} rows={count} gaps={gaps}",
                flush=True,
            )

    def handle_live_payload(self, payload: dict[str, Any], received_epoch: float) -> None:
        samples = payload.get("samples")
        end_seq = payload.get("seq")
        if not isinstance(samples, list) or not isinstance(end_seq, int) or not samples:
            return

        n = len(samples)
        end_seq &= 0xFFFF

        with self.lock:
            if self.last_seq is not None:
                delta = (end_seq - self.last_seq) & 0xFFFF
                if delta != n:
                    self.gaps += 1
                    print(
                        f"\n[GAP] previous end seq={self.last_seq}, current={end_seq}, "
                        f"expected delta={n}, got {delta}",
                        file=sys.stderr,
                        flush=True,
                    )
            self.last_seq = end_seq

        # AirBridge emits a batch after the newest sample is available. Estimate
        # the last sample at receipt time and place preceding samples at 40 ms.
        for i, sample in enumerate(samples):
            if not (isinstance(sample, list) and len(sample) >= 3):
                continue
            try:
                mkp, rfl, lyk = int(sample[0]), int(sample[1]), int(sample[2])
            except (TypeError, ValueError):
                continue
            seq = (end_seq - (n - 1 - i)) & 0xFFFF
            sample_epoch = received_epoch - (n - 1 - i) * SAMPLE_PERIOD_S
            pressure = mkp / 50.0
            flow_l_min = (rfl / 500.0) * 60.0
            leak_l_min = (lyk / 50.0) * 60.0

            self.samples.writerow([
                iso_now(sample_epoch),
                iso_now(received_epoch),
                seq,
                mkp,
                rfl,
                lyk,
                f"{pressure:.3f}",
                f"{flow_l_min:.3f}",
                f"{leak_l_min:.3f}",
            ])
            with self.lock:
                self.latest.append((sample_epoch, pressure, flow_l_min, leak_l_min))
                self.sample_rows += 1

    def run_live(self) -> None:
        print(f"Capturing from {self.base_url}/events/live")
        print(f"Files: {self.out_dir}")
        print("Type a marker such as 'normal', 'intentional leak', 'mask off', or 'mask on' and press Enter.")
        print("Press Ctrl+C to stop.\n")

        while not self.stop.is_set():
            try:
                with self.request("/events/live", accept="text/event-stream") as resp:
                    if resp.status != 200:
                        raise RuntimeError(f"SSE returned HTTP {resp.status}")
                    data_lines: list[str] = []
                    for raw in resp:
                        if self.stop.is_set():
                            return
                        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        if line == "":
                            if data_lines:
                                text = "\n".join(data_lines)
                                data_lines.clear()
                                try:
                                    payload = json.loads(text)
                                except json.JSONDecodeError:
                                    continue
                                if isinstance(payload, dict):
                                    self.handle_live_payload(payload, time.time())
                            continue
                        if line.startswith("data:"):
                            data_lines.append(line[5:].lstrip())
            except urllib.error.HTTPError as exc:
                if exc.code == 401:
                    print("Authentication failed (HTTP 401). Check AirBridge username/password.", file=sys.stderr)
                    self.stop.set()
                    return
                print(f"SSE HTTP error: {exc}; reconnecting...", file=sys.stderr)
            except KeyboardInterrupt:
                self.stop.set()
                return
            except Exception as exc:
                if not self.stop.is_set():
                    print(f"SSE connection error: {exc}; reconnecting...", file=sys.stderr)

            self.stop.wait(1.0)

    def close(self) -> None:
        self.stop.set()
        for fp in (self.samples_fp, self.markers_fp, self.status_fp):
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture AirBridge live pressure/flow/leak to local diagnostic files."
    )
    parser.add_argument("--host", default="airbridge.local", help="AirBridge host or IP")
    parser.add_argument("--user", default="admin", help="AirBridge web username")
    parser.add_argument(
        "--output",
        type=Path,
        help="Output directory (default: captures/YYYYMMDD-HHMMSS)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    host = args.host
    if not host.startswith(("http://", "https://")):
        host = "http://" + host

    password = os.environ.get("AIRBRIDGE_PASSWORD")
    if password is None:
        password = getpass.getpass(f"Password for {args.user}@{args.host}: ")

    out_dir = args.output or Path("captures") / datetime.now().strftime("%Y%m%d-%H%M%S")
    capture = Capture(host, args.user, password, out_dir)

    status_thread = threading.Thread(target=capture.run_status, name="status", daemon=True)
    marker_thread = threading.Thread(target=capture.run_markers, name="markers", daemon=True)
    summary_thread = threading.Thread(target=capture.run_summary, name="summary", daemon=True)
    status_thread.start()
    marker_thread.start()
    summary_thread.start()

    try:
        capture.run_live()
    except KeyboardInterrupt:
        pass
    finally:
        capture.close()

    print(
        f"\nStopped. Captured {capture.sample_rows} samples; "
        f"detected {capture.gaps} sequence gaps. Files are in {capture.out_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
