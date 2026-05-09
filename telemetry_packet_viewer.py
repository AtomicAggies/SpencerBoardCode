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
RECORD_FORMAT_AUTO = "auto"
RECORD_FORMAT_RAW = "raw"
RECORD_FORMAT_LF = "lf"
RECORD_FORMAT_CR = "cr"
RECORD_FORMAT_CRLF = "crlf"
RECORD_FORMAT_LABELS = {
    "Auto detect": RECORD_FORMAT_AUTO,
    "Raw 98-byte packets": RECORD_FORMAT_RAW,
    "LF/newline terminated (98 + 0A)": RECORD_FORMAT_LF,
    "CR terminated (98 + 0D)": RECORD_FORMAT_CR,
    "CRLF terminated (98 + 0D0A)": RECORD_FORMAT_CRLF,
}
RECORD_FORMAT_DISPLAY = {value: label for label, value in RECORD_FORMAT_LABELS.items()}
RECORD_TERMINATORS = {
    RECORD_FORMAT_RAW: b"",
    RECORD_FORMAT_LF: b"\n",
    RECORD_FORMAT_CR: b"\r",
    RECORD_FORMAT_CRLF: b"\r\n",
}
MAX_AUTO_PAYLOAD_SIZE = 256


VALIDITY_FLAGS = {
    "gps": (1 << 0, "GPS"),
    "bmp": (1 << 1, "BMP"),
    "magnetometer": (1 << 2, "Magnetometer"),
    "inertial": (1 << 3, "Inertial/IMU"),
}

I2C_STATUS_MESSAGES = {
    0: "success",
    1: "data too long for transmit buffer",
    2: "received NACK on address",
    3: "received NACK on data",
    4: "other I2C error",
    5: "timeout",
    0xFE: "short Wire.write",
}

FIELD_SPECS = [
    ("Callsign", 0, 6, "6s", "ASCII callsign", None),
    ("Packet Counter", 6, 2, "<H", "uint16, rolls over after 65535", None),
    ("Validity Flags", 8, 1, "<B", "bit 0 GPS, bit 1 BMP, bit 2 mag, bit 3 IMU", None),
    ("GPS Latitude", 9, 4, "<i", "degrees = raw / 1e7", "gps"),
    ("GPS Longitude", 13, 4, "<i", "degrees = raw / 1e7", "gps"),
    ("GPS Altitude", 17, 4, "<i", "millimeters", "gps"),
    ("GPS NED North Velocity", 21, 4, "<i", "millimeters/second", "gps"),
    ("GPS NED Down Velocity", 25, 4, "<i", "millimeters/second", "gps"),
    ("GPS NED East Velocity", 29, 4, "<i", "millimeters/second", "gps"),
    ("GPS Unix Epoch", 33, 4, "<I", "seconds since 1970-01-01 UTC", "gps"),
    ("BMP Temperature", 37, 4, "<f", "degrees C", "bmp"),
    ("BMP Pressure", 41, 4, "<f", "pascals", "bmp"),
    ("Magnetometer X", 45, 2, "<h", "raw int16", "magnetometer"),
    ("Magnetometer Y", 47, 2, "<h", "raw int16", "magnetometer"),
    ("Magnetometer Z", 49, 2, "<h", "raw int16", "magnetometer"),
    ("Accel X", 51, 4, "<f", "m/s^2", "inertial"),
    ("Accel Y", 55, 4, "<f", "m/s^2", "inertial"),
    ("Accel Z", 59, 4, "<f", "m/s^2", "inertial"),
    ("Gyro Z", 63, 4, "<f", "rad/s", "inertial"),
    ("Gyro Y", 67, 4, "<f", "rad/s", "inertial"),
    ("Gyro X", 71, 4, "<f", "rad/s", "inertial"),
    ("IMU Temperature", 75, 4, "<f", "degrees C", "inertial"),
    ("Previous I2C Bytes Written", 79, 1, "<B", "previous framed I2C send", None),
    ("Previous I2C Status", 80, 1, "<B", "previous Wire.endTransmission/status", None),
    ("Reserved", 81, 17, None, "padding/reserved bytes", None),
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


def _format_validity_flags(raw_value: int) -> str:
    enabled = [label for bit, label in VALIDITY_FLAGS.values() if raw_value & bit]
    if not enabled:
        enabled_text = "no sensors valid"
    else:
        enabled_text = ", ".join(enabled)
    return f"0x{raw_value:02x} ({enabled_text})"


def _format_decoded_value(name: str, raw_value):
    if name == "Callsign":
        return raw_value.rstrip(b"\x00").decode("ascii", errors="replace")
    if name == "Packet Counter":
        return str(raw_value)
    if name == "Validity Flags":
        return _format_validity_flags(raw_value)
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
    if name == "Previous I2C Status":
        status_message = I2C_STATUS_MESSAGES.get(raw_value, "unknown status")
        return f"{raw_value} ({status_message})"
    if isinstance(raw_value, float):
        return f"{raw_value:.7g}"
    return str(raw_value)


def _field_validity_text(validity: int, sensor_key) -> str:
    if sensor_key is None:
        return "n/a"
    flag, label = VALIDITY_FLAGS[sensor_key]
    return f"Yes ({label})" if validity & flag else f"No ({label})"


def decode_packet(packet: bytes):
    rows = []
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"Expected {PACKET_SIZE} bytes, got {len(packet)} bytes")

    validity = struct.unpack_from("<B", packet, 8)[0]
    for name, offset, size, fmt, notes, sensor_key in FIELD_SPECS:
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
                "valid": _field_validity_text(validity, sensor_key),
                "notes": notes,
            }
        )
    return rows


def _looks_like_hex_text(data: bytes) -> bool:
    stripped = b"".join(data.split())
    return bool(stripped) and len(stripped) % 2 == 0 and bool(HEX_TEXT_RE.match(data))


def _build_truncation_warning(payload_size: int, discarded_chunks):
    if payload_size == PACKET_SIZE:
        return None

    extra_byte_count = payload_size - PACKET_SIZE
    if all(not any(chunk) for chunk in discarded_chunks):
        extra_description = "all discarded bytes were 00"
    else:
        extra_description = "some discarded bytes were non-zero"
    return (
        f"Detected {payload_size}-byte payloads and truncated "
        f"{extra_byte_count} extra byte(s) from each record; {extra_description}."
    )


def _split_raw_records(data: bytes):
    if len(data) % PACKET_SIZE != 0:
        raise ValueError(
            f"Input contains {len(data)} bytes after decoding, which is not a "
            f"multiple of {PACKET_SIZE} for {RECORD_FORMAT_DISPLAY[RECORD_FORMAT_RAW]}."
        )
    return [data[i : i + PACKET_SIZE] for i in range(0, len(data), PACKET_SIZE)], []


def _split_terminated_records(data: bytes, record_format: str, payload_size: int):
    terminator = RECORD_TERMINATORS[record_format]
    record_size = payload_size + len(terminator)
    if len(data) % record_size != 0:
        raise ValueError(
            f"Input contains {len(data)} bytes after decoding, which is not a "
            f"multiple of {record_size} for {RECORD_FORMAT_DISPLAY[record_format]}."
        )

    packets = []
    discarded_chunks = []
    for offset in range(0, len(data), record_size):
        payload_end = offset + payload_size
        record_terminator = data[payload_end : payload_end + len(terminator)]
        if record_terminator != terminator:
            raise ValueError(
                f"Record at byte offset {offset} does not end with "
                f"the expected {terminator.hex()} terminator after "
                f"{payload_size} payload bytes."
            )
        packets.append(data[offset : offset + PACKET_SIZE])
        discarded_chunks.append(data[offset + PACKET_SIZE : payload_end])

    warnings = []
    truncation_warning = _build_truncation_warning(payload_size, discarded_chunks)
    if truncation_warning is not None:
        warnings.append(truncation_warning)
    return packets, warnings


def _payload_sizes_to_try(data_length: int, terminator_length: int):
    max_payload_size = min(MAX_AUTO_PAYLOAD_SIZE, data_length - terminator_length)
    return range(PACKET_SIZE, max_payload_size + 1)


def _split_records_for_format(data: bytes, record_format: str):
    if record_format == RECORD_FORMAT_RAW:
        return _split_raw_records(data)

    errors = []
    terminator_length = len(RECORD_TERMINATORS[record_format])
    for payload_size in _payload_sizes_to_try(len(data), terminator_length):
        try:
            return _split_terminated_records(data, record_format, payload_size)
        except ValueError as exc:
            errors.append(str(exc))

    raise ValueError(
        f"Could not parse {RECORD_FORMAT_DISPLAY[record_format]} using payload "
        f"sizes from {PACKET_SIZE} through "
        f"{min(MAX_AUTO_PAYLOAD_SIZE, len(data) - terminator_length)} bytes. "
        f"Last error: {errors[-1] if errors else 'input is too short'}"
    )


def _split_records(data: bytes, record_format: str = RECORD_FORMAT_AUTO):
    if len(data) == 0:
        return [], record_format, []
    if record_format != RECORD_FORMAT_AUTO:
        packets, warnings = _split_records_for_format(data, record_format)
        return packets, record_format, warnings

    format_order = (
        RECORD_FORMAT_CRLF,
        RECORD_FORMAT_LF,
        RECORD_FORMAT_CR,
        RECORD_FORMAT_RAW,
    )
    errors = []
    for candidate_format in format_order:
        try:
            packets, warnings = _split_records_for_format(data, candidate_format)
        except ValueError as exc:
            errors.append(f"{RECORD_FORMAT_DISPLAY[candidate_format]}: {exc}")
            continue
        return packets, candidate_format, warnings

    raise ValueError(
        "Could not auto-detect packet record format. Tried raw 98-byte packets, "
        "LF-terminated records, CR-terminated records, and CRLF-terminated "
        "records. Details: " + "; ".join(errors)
    )


def load_packets(path: pathlib.Path, record_format: str = RECORD_FORMAT_AUTO):
    data = path.read_bytes()
    if _looks_like_hex_text(data):
        data = bytes.fromhex(data.decode("ascii"))
    packets, detected_format, warnings = _split_records(data, record_format)
    if not packets:
        raise ValueError("No packets found in the selected file")
    return packets, detected_format, warnings


class TelemetryPacketViewer(tk.Tk):
    def __init__(self, initial_path=None):
        super().__init__()
        self.title("Spencer Board Telemetry Packet Viewer")
        self.geometry("1200x720")
        self.minsize(940, 560)

        self.packets = []
        self.current_index = 0
        self.path_var = tk.StringVar(value="No file loaded")
        self.packet_var = tk.StringVar(value="Packet 0 of 0")
        self.goto_var = tk.StringVar(value="1")
        self.record_format_var = tk.StringVar(value="Auto detect")
        self.current_record_format = RECORD_FORMAT_AUTO
        self.warning_var = tk.StringVar(value="")

        self._build_ui()
        if initial_path is not None:
            self.open_path(pathlib.Path(initial_path))

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Record format:").pack(side=tk.LEFT)
        format_selector = ttk.Combobox(
            top,
            textvariable=self.record_format_var,
            values=tuple(RECORD_FORMAT_LABELS.keys()),
            state="readonly",
            width=31,
        )
        format_selector.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="Open log…", command=self.open_file).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.path_var, padding=(10, 0)).pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.warning_label = ttk.Label(
            self,
            textvariable=self.warning_var,
            foreground="dark orange",
            padding=(8, 0, 8, 6),
        )
        self.warning_label.pack(fill=tk.X)

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
        columns = ("offset", "size", "field", "hex", "decoded", "valid", "notes")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings")
        headings = {
            "offset": "Offset",
            "size": "Bytes",
            "field": "Field",
            "hex": "Hex",
            "decoded": "Decoded value",
            "valid": "Valid?",
            "notes": "Notes / units",
        }
        widths = {
            "offset": 62,
            "size": 55,
            "field": 170,
            "hex": 210,
            "decoded": 230,
            "valid": 105,
            "notes": 190,
        }
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
        selected_format = RECORD_FORMAT_LABELS[self.record_format_var.get()]
        try:
            packets, detected_format, warnings = load_packets(path, selected_format)
        except Exception as exc:
            messagebox.showerror("Could not load telemetry log", str(exc))
            return
        self.packets = packets
        self.current_record_format = detected_format
        self.current_index = 0
        self.path_var.set(
            f"{path} ({len(packets)} packets, {RECORD_FORMAT_DISPLAY[detected_format]})"
        )
        self.warning_var.set("  ".join(warnings))
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
                    row["valid"],
                    row["notes"],
                ),
            )


def main():
    parser = argparse.ArgumentParser(description="View Spencer Board telemetry packets")
    parser.add_argument("path", nargs="?", help="Optional telemetry log path to open on startup")
    parser.add_argument(
        "--record-format",
        choices=(RECORD_FORMAT_AUTO, *RECORD_TERMINATORS.keys()),
        default=RECORD_FORMAT_AUTO,
        help="Packet record format to use when opening the optional startup path.",
    )
    args = parser.parse_args()
    app = TelemetryPacketViewer()
    if args.record_format != RECORD_FORMAT_AUTO:
        app.record_format_var.set(RECORD_FORMAT_DISPLAY[args.record_format])
    if args.path is not None:
        app.open_path(pathlib.Path(args.path))
    app.mainloop()


if __name__ == "__main__":
    main()
