#!/usr/bin/env python3
"""GUI viewer for Spencer Board 98-byte telemetry packets.

Open either the raw binary SD-card log or a text file containing hexadecimal bytes.
The viewer displays one packet at a time with the complete packet hex and a table
of decoded fields based on the packet layout in SpencerBoardCode.ino.
"""

import argparse
import datetime as _dt
import pathlib
import re
import struct
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

PACKET_SIZE = 98
LINE_TERMINATED_RECORD_SIZE = PACKET_SIZE + 2

FIELD_SPECS = [
    ("Callsign", 0, 6, "6s", "ASCII callsign"),
    ("GPS Latitude", 6, 4, "<i", "degrees = raw / 1e7"),
    ("GPS Longitude", 10, 4, "<i", "degrees = raw / 1e7"),
    ("GPS Altitude", 14, 4, "<i", "millimeters"),
    ("GPS NED North Velocity", 18, 4, "<i", "millimeters/second"),
    ("GPS NED Down Velocity", 22, 4, "<i", "millimeters/second"),
    ("GPS NED East Velocity", 26, 4, "<i", "millimeters/second"),
    ("GPS Unix Epoch", 30, 4, "<I", "seconds since 1970-01-01 UTC"),
    ("BMP Temperature", 34, 4, "<f", "degrees C"),
    ("BMP Pressure", 38, 4, "<f", "pascals"),
    ("Magnetometer X", 42, 2, "<h", "raw int16"),
    ("Magnetometer Y", 44, 2, "<h", "raw int16"),
    ("Magnetometer Z", 46, 2, "<h", "raw int16"),
    ("Accel X", 48, 4, "<f", "m/s^2"),
    ("Accel Y", 52, 4, "<f", "m/s^2"),
    ("Accel Z", 56, 4, "<f", "m/s^2"),
    ("Gyro Z", 60, 4, "<f", "rad/s"),
    ("Gyro Y", 64, 4, "<f", "rad/s"),
    ("Gyro X", 68, 4, "<f", "rad/s"),
    ("IMU Temperature", 72, 4, "<f", "degrees C"),
    ("Reserved", 76, 22, None, "padding/reserved bytes"),
]

HEX_TEXT_RE = re.compile(rb"^[0-9a-fA-F\s]+$")


def _format_hex(data: bytes) -> str:
    return " ".join(f"{byte:02x}" for byte in data)


def _format_packet_hex(packet: bytes) -> str:
    lines = []
    for offset in range(0, len(packet), 16):
        chunk = packet[offset : offset + 16]
        lines.append(f"{offset:04x}: {_format_hex(chunk)}")
    return "\n".join(lines)


def _format_decoded_value(name: str, raw_value):
    if name == "Callsign":
        return raw_value.rstrip(b"\x00").decode("ascii", errors="replace")
    if name == "GPS Latitude" or name == "GPS Longitude":
        return f"{raw_value} ({raw_value / 10_000_000:.7f}°)"
    if name == "GPS Altitude":
        return f"{raw_value} mm ({raw_value / 1000:.3f} m)"
    if "Velocity" in name:
        return f"{raw_value} mm/s ({raw_value / 1000:.3f} m/s)"
    if name == "GPS Unix Epoch":
        if raw_value == 0:
            return "0 (unset)"
        timestamp = _dt.datetime.fromtimestamp(raw_value, tz=_dt.timezone.utc)
        return f"{raw_value} ({timestamp.isoformat()})"
    if isinstance(raw_value, float):
        return f"{raw_value:.7g}"
    return str(raw_value)


def decode_packet(packet: bytes):
    rows = []
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"Expected {PACKET_SIZE} bytes, got {len(packet)} bytes")

    for name, offset, size, fmt, notes in FIELD_SPECS:
        raw_bytes = packet[offset : offset + size]
        if fmt is None:
            raw = _format_hex(raw_bytes)
            decoded = raw if any(raw_bytes) else "all zeros"
        else:
            raw = struct.unpack_from(fmt, packet, offset)[0]
            decoded = _format_decoded_value(name, raw)
        rows.append(
            {
                "offset": offset,
                "size": size,
                "field": name,
                "hex": _format_hex(raw_bytes),
                "decoded": decoded,
                "notes": notes,
            }
        )
    return rows


def _looks_like_hex_text(data: bytes) -> bool:
    stripped = b"".join(data.split())
    return bool(stripped) and len(stripped) % 2 == 0 and bool(HEX_TEXT_RE.match(data))


def _split_records(data: bytes):
    if len(data) == 0:
        return []
    if len(data) % PACKET_SIZE == 0:
        return [data[i : i + PACKET_SIZE] for i in range(0, len(data), PACKET_SIZE)]
    if len(data) % LINE_TERMINATED_RECORD_SIZE == 0:
        packets = []
        for offset in range(0, len(data), LINE_TERMINATED_RECORD_SIZE):
            record = data[offset : offset + LINE_TERMINATED_RECORD_SIZE]
            if record[PACKET_SIZE:] != b"\r\n":
                break
            packets.append(record[:PACKET_SIZE])
        if len(packets) * LINE_TERMINATED_RECORD_SIZE == len(data):
            return packets
    raise ValueError(
        f"Input contains {len(data)} bytes after decoding. Expected a multiple of "
        f"{PACKET_SIZE} bytes, or {LINE_TERMINATED_RECORD_SIZE}-byte records with CRLF."
    )


def load_packets(path: pathlib.Path):
    data = path.read_bytes()
    if _looks_like_hex_text(data):
        data = bytes.fromhex(data.decode("ascii"))
    packets = _split_records(data)
    if not packets:
        raise ValueError("No packets found in the selected file")
    return packets


class TelemetryPacketViewer(tk.Tk):
    def __init__(self, initial_path=None):
        super().__init__()
        self.title("Spencer Board Telemetry Packet Viewer")
        self.geometry("1100x720")
        self.minsize(880, 560)

        self.packets = []
        self.current_index = 0
        self.path_var = tk.StringVar(value="No file loaded")
        self.packet_var = tk.StringVar(value="Packet 0 of 0")
        self.goto_var = tk.StringVar(value="1")

        self._build_ui()
        if initial_path is not None:
            self.open_path(pathlib.Path(initial_path))

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Button(top, text="Open log…", command=self.open_file).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.path_var, padding=(10, 0)).pack(side=tk.LEFT, fill=tk.X, expand=True)

        nav = ttk.Frame(self, padding=(8, 0, 8, 8))
        nav.pack(fill=tk.X)
        ttk.Button(nav, text="◀ Previous", command=self.previous_packet).pack(side=tk.LEFT)
        ttk.Button(nav, text="Next ▶", command=self.next_packet).pack(side=tk.LEFT, padx=(6, 12))
        ttk.Label(nav, textvariable=self.packet_var).pack(side=tk.LEFT)
        ttk.Label(nav, text="Go to packet:", padding=(18, 0, 4, 0)).pack(side=tk.LEFT)
        goto_entry = ttk.Entry(nav, textvariable=self.goto_var, width=8)
        goto_entry.pack(side=tk.LEFT)
        goto_entry.bind("<Return>", lambda _event: self.go_to_packet())
        ttk.Button(nav, text="Go", command=self.go_to_packet).pack(side=tk.LEFT, padx=(4, 0))

        panes = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        hex_frame = ttk.LabelFrame(panes, text="Packet hex", padding=6)
        self.hex_text = tk.Text(hex_frame, wrap=tk.NONE, font=("TkFixedFont", 10), height=12)
        hex_y = ttk.Scrollbar(hex_frame, orient=tk.VERTICAL, command=self.hex_text.yview)
        hex_x = ttk.Scrollbar(hex_frame, orient=tk.HORIZONTAL, command=self.hex_text.xview)
        self.hex_text.configure(yscrollcommand=hex_y.set, xscrollcommand=hex_x.set)
        self.hex_text.grid(row=0, column=0, sticky="nsew")
        hex_y.grid(row=0, column=1, sticky="ns")
        hex_x.grid(row=1, column=0, sticky="ew")
        hex_frame.rowconfigure(0, weight=1)
        hex_frame.columnconfigure(0, weight=1)
        panes.add(hex_frame, weight=1)

        table_frame = ttk.LabelFrame(panes, text="Decoded fields", padding=6)
        columns = ("offset", "size", "field", "hex", "decoded", "notes")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings")
        headings = {
            "offset": "Offset",
            "size": "Bytes",
            "field": "Field",
            "hex": "Hex",
            "decoded": "Decoded value",
            "notes": "Notes / units",
        }
        widths = {"offset": 62, "size": 55, "field": 170, "hex": 210, "decoded": 220, "notes": 180}
        for column in columns:
            self.table.heading(column, text=headings[column])
            self.table.column(column, width=widths[column], anchor=tk.W)
        table_y = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.table.yview)
        table_x = ttk.Scrollbar(table_frame, orient=tk.HORIZONTAL, command=self.table.xview)
        self.table.configure(yscrollcommand=table_y.set, xscrollcommand=table_x.set)
        self.table.grid(row=0, column=0, sticky="nsew")
        table_y.grid(row=0, column=1, sticky="ns")
        table_x.grid(row=1, column=0, sticky="ew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)
        panes.add(table_frame, weight=3)

    def open_file(self):
        filename = filedialog.askopenfilename(
            title="Open telemetry log",
            filetypes=(
                ("Telemetry logs", "*.bin *.dat *.log *.hex *.txt"),
                ("All files", "*"),
            ),
        )
        if filename:
            self.open_path(pathlib.Path(filename))

    def open_path(self, path: pathlib.Path):
        try:
            packets = load_packets(path)
        except Exception as exc:
            messagebox.showerror("Could not load telemetry log", str(exc))
            return
        self.packets = packets
        self.current_index = 0
        self.path_var.set(f"{path} ({len(packets)} packets)")
        self.show_packet()

    def previous_packet(self):
        if not self.packets:
            return
        self.current_index = max(0, self.current_index - 1)
        self.show_packet()

    def next_packet(self):
        if not self.packets:
            return
        self.current_index = min(len(self.packets) - 1, self.current_index + 1)
        self.show_packet()

    def go_to_packet(self):
        if not self.packets:
            return
        try:
            requested = int(self.goto_var.get())
        except ValueError:
            messagebox.showwarning("Invalid packet number", "Enter a whole packet number.")
            return
        if requested < 1 or requested > len(self.packets):
            messagebox.showwarning(
                "Packet out of range",
                f"Enter a packet number from 1 to {len(self.packets)}.",
            )
            return
        self.current_index = requested - 1
        self.show_packet()

    def show_packet(self):
        if not self.packets:
            self.packet_var.set("Packet 0 of 0")
            return
        packet = self.packets[self.current_index]
        self.packet_var.set(f"Packet {self.current_index + 1} of {len(self.packets)}")
        self.goto_var.set(str(self.current_index + 1))

        self.hex_text.configure(state=tk.NORMAL)
        self.hex_text.delete("1.0", tk.END)
        self.hex_text.insert(tk.END, _format_packet_hex(packet))
        self.hex_text.configure(state=tk.DISABLED)

        self.table.delete(*self.table.get_children())
        for row in decode_packet(packet):
            self.table.insert(
                "",
                tk.END,
                values=(
                    row["offset"],
                    row["size"],
                    row["field"],
                    row["hex"],
                    row["decoded"],
                    row["notes"],
                ),
            )


def main():
    parser = argparse.ArgumentParser(description="View Spencer Board telemetry packets")
    parser.add_argument("path", nargs="?", help="Optional telemetry log path to open on startup")
    args = parser.parse_args()
    app = TelemetryPacketViewer(args.path)
    app.mainloop()


if __name__ == "__main__":
    main()
