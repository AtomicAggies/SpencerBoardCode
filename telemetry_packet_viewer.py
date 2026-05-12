#!/usr/bin/env python3
"""GUI viewer for Spencer Board telemetry SD logs.

Supports length-prefixed records (byte 0 = total record size including itself),
matching TelemetryData.h (max 76 bytes for the current struct). LoRa does not
include this byte; SD / I2C binary logs do.

Also supports legacy fixed 98-byte records for older logs.
"""

import argparse
import datetime as _dt
import pathlib
import re
import struct
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# Must match TelemetryData.h (wireLength + rest; full struct).
TELEMETRY_WIRE_LENGTH_MAX = 76
LEGACY_PACKET_SIZE = 98

RECORD_FORMAT_AUTO = "auto"
RECORD_FORMAT_LEN_RAW = "len_raw"
RECORD_FORMAT_LEN_LF = "len_lf"
RECORD_FORMAT_LEN_CR = "len_cr"
RECORD_FORMAT_LEN_CRLF = "len_crlf"
RECORD_FORMAT_FIXED98_RAW = "fixed98_raw"
RECORD_FORMAT_FIXED98_LF = "fixed98_lf"
RECORD_FORMAT_FIXED98_CR = "fixed98_cr"
RECORD_FORMAT_FIXED98_CRLF = "fixed98_crlf"

RECORD_FORMAT_LABELS = {
    "Auto detect": RECORD_FORMAT_AUTO,
    "Raw length-prefixed (byte0=len, concatenated)": RECORD_FORMAT_LEN_RAW,
    "LF after each record (len + payload + 0A)": RECORD_FORMAT_LEN_LF,
    "CR after each record (len + payload + 0D)": RECORD_FORMAT_LEN_CR,
    "CRLF after each record (len + payload + 0D0A)": RECORD_FORMAT_LEN_CRLF,
    "Legacy raw 98-byte packets": RECORD_FORMAT_FIXED98_RAW,
    "Legacy LF (98 + 0A)": RECORD_FORMAT_FIXED98_LF,
    "Legacy CR (98 + 0D)": RECORD_FORMAT_FIXED98_CR,
    "Legacy CRLF (98 + 0D0A)": RECORD_FORMAT_FIXED98_CRLF,
}
RECORD_FORMAT_DISPLAY = {value: label for label, value in RECORD_FORMAT_LABELS.items()}

LEGACY_TERMINATORS = {
    RECORD_FORMAT_FIXED98_LF: b"\n",
    RECORD_FORMAT_FIXED98_CR: b"\r",
    RECORD_FORMAT_FIXED98_CRLF: b"\r\n",
}
LEN_TERMINATORS = {
    RECORD_FORMAT_LEN_LF: b"\n",
    RECORD_FORMAT_LEN_CR: b"\r",
    RECORD_FORMAT_LEN_CRLF: b"\r\n",
}

LEGACY_DECODE_FORMATS = frozenset(
    {
        RECORD_FORMAT_FIXED98_RAW,
        RECORD_FORMAT_FIXED98_LF,
        RECORD_FORMAT_FIXED98_CR,
        RECORD_FORMAT_FIXED98_CRLF,
    }
)

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

# Current TelemetryData (wireLength first). Offsets include byte 0.
FIELD_SPECS = [
    ("Wire length", 0, 1, "<B", "total bytes in this record on SD/I2C, including this byte", None),
    ("Packet Counter", 1, 2, "<H", "uint16, rolls over after 65535", None),
    ("Validity Flags", 3, 1, "<B", "bit 0 GPS, bit 1 BMP, bit 2 mag, bit 3 IMU", None),
    ("GPS Latitude", 4, 4, "<i", "degrees = raw / 1e7", "gps"),
    ("GPS Longitude", 8, 4, "<i", "degrees = raw / 1e7", "gps"),
    ("GPS Altitude", 12, 4, "<i", "millimeters", "gps"),
    ("GPS NED North Velocity", 16, 4, "<i", "millimeters/second", "gps"),
    ("GPS NED Down Velocity", 20, 4, "<i", "millimeters/second", "gps"),
    ("GPS NED East Velocity", 24, 4, "<i", "millimeters/second", "gps"),
    ("GPS Unix Epoch", 28, 4, "<I", "seconds since 1970-01-01 UTC", "gps"),
    ("BMP Temperature", 32, 4, "<f", "degrees C", "bmp"),
    ("BMP Pressure", 36, 4, "<f", "pascals", "bmp"),
    ("Magnetometer X", 40, 2, "<h", "raw int16", "magnetometer"),
    ("Magnetometer Y", 42, 2, "<h", "raw int16", "magnetometer"),
    ("Magnetometer Z", 44, 2, "<h", "raw int16", "magnetometer"),
    ("Accel X", 46, 4, "<f", "m/s^2", "inertial"),
    ("Accel Y", 50, 4, "<f", "m/s^2", "inertial"),
    ("Accel Z", 54, 4, "<f", "m/s^2", "inertial"),
    ("Gyro Z", 58, 4, "<f", "rad/s", "inertial"),
    ("Gyro Y", 62, 4, "<f", "rad/s", "inertial"),
    ("Gyro X", 66, 4, "<f", "rad/s", "inertial"),
    ("IMU Temperature", 70, 4, "<f", "degrees C", "inertial"),
    ("Previous I2C Bytes Written", 74, 1, "<B", "previous framed I2C send", None),
    ("Previous I2C Status", 75, 1, "<B", "previous Wire.endTransmission/status", None),
]

# Legacy 98-byte layout (no leading wireLength).
LEGACY_FIELD_SPECS = [
    ("Packet Counter", 0, 2, "<H", "uint16, rolls over after 65535", None),
    ("Validity Flags", 2, 1, "<B", "bit 0 GPS, bit 1 BMP, bit 2 mag, bit 3 IMU", None),
    ("GPS Latitude", 3, 4, "<i", "degrees = raw / 1e7", "gps"),
    ("GPS Longitude", 7, 4, "<i", "degrees = raw / 1e7", "gps"),
    ("GPS Altitude", 11, 4, "<i", "millimeters", "gps"),
    ("GPS NED North Velocity", 15, 4, "<i", "millimeters/second", "gps"),
    ("GPS NED Down Velocity", 19, 4, "<i", "millimeters/second", "gps"),
    ("GPS NED East Velocity", 23, 4, "<i", "millimeters/second", "gps"),
    ("GPS Unix Epoch", 27, 4, "<I", "seconds since 1970-01-01 UTC", "gps"),
    ("BMP Temperature", 31, 4, "<f", "degrees C", "bmp"),
    ("BMP Pressure", 35, 4, "<f", "pascals", "bmp"),
    ("Magnetometer X", 39, 2, "<h", "raw int16", "magnetometer"),
    ("Magnetometer Y", 41, 2, "<h", "raw int16", "magnetometer"),
    ("Magnetometer Z", 43, 2, "<h", "raw int16", "magnetometer"),
    ("Accel X", 45, 4, "<f", "m/s^2", "inertial"),
    ("Accel Y", 49, 4, "<f", "m/s^2", "inertial"),
    ("Accel Z", 53, 4, "<f", "m/s^2", "inertial"),
    ("Gyro Z", 57, 4, "<f", "rad/s", "inertial"),
    ("Gyro Y", 61, 4, "<f", "rad/s", "inertial"),
    ("Gyro X", 65, 4, "<f", "rad/s", "inertial"),
    ("IMU Temperature", 69, 4, "<f", "degrees C", "inertial"),
    ("Previous I2C Bytes Written", 73, 1, "<B", "previous framed I2C send", None),
    ("Previous I2C Status", 74, 1, "<B", "previous Wire.endTransmission/status", None),
    ("Reserved", 75, 23, None, "padding/reserved bytes", None),
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
    if name == "Packet Counter":
        return str(raw_value)
    if name == "Validity Flags":
        return _format_validity_flags(raw_value)
    if name == "Wire length":
        return str(raw_value)
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


def _decode_with_specs(packet: bytes, field_specs, validity_offset: int):
    rows = []
    if len(packet) > validity_offset:
        validity = struct.unpack_from("<B", packet, validity_offset)[0]
    else:
        validity = 0
    for name, offset, size, fmt, notes, sensor_key in field_specs:
        if offset >= len(packet):
            break
        if offset + size > len(packet):
            raw_bytes = packet[offset:]
            rows.append(
                {
                    "offset": offset,
                    "size": len(raw_bytes),
                    "field": name,
                    "hex": _format_hex(raw_bytes),
                    "decoded": "(truncated — packet ends before field completes)",
                    "valid": _field_validity_text(validity, sensor_key),
                    "notes": notes,
                }
            )
            break
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


def decode_packet(packet: bytes):
    """Length-prefixed TelemetryData (variable total size, max TELEMETRY_WIRE_LENGTH_MAX)."""
    if len(packet) < 2:
        raise ValueError("Packet too short (need at least wireLength + 1 byte)")
    wire_len = packet[0]
    if wire_len != len(packet):
        raise ValueError(
            f"wireLength byte is {wire_len} but buffer length is {len(packet)}"
        )
    if wire_len < 2 or wire_len > TELEMETRY_WIRE_LENGTH_MAX:
        raise ValueError(
            f"wireLength {wire_len} out of range (2..{TELEMETRY_WIRE_LENGTH_MAX})"
        )
    return _decode_with_specs(packet, FIELD_SPECS, validity_offset=3)


def decode_legacy_packet(packet: bytes):
    if len(packet) != LEGACY_PACKET_SIZE:
        raise ValueError(
            f"Expected legacy {LEGACY_PACKET_SIZE} bytes, got {len(packet)} bytes"
        )
    return _decode_with_specs(packet, LEGACY_FIELD_SPECS, validity_offset=2)


def _looks_like_hex_text(data: bytes) -> bool:
    stripped = b"".join(data.split())
    return bool(stripped) and len(stripped) % 2 == 0 and bool(HEX_TEXT_RE.match(data))


def _split_len_prefixed_raw(data: bytes):
    packets = []
    offset = 0
    while offset < len(data):
        if offset + 1 > len(data):
            raise ValueError(f"Incomplete length prefix at file offset {offset}")
        wire_len = data[offset]
        if wire_len < 2 or wire_len > TELEMETRY_WIRE_LENGTH_MAX:
            raise ValueError(
                f"Invalid wireLength {wire_len} at file offset {offset} "
                f"(expected 2..{TELEMETRY_WIRE_LENGTH_MAX})"
            )
        end = offset + wire_len
        if end > len(data):
            raise ValueError(
                f"Incomplete record at offset {offset}: need {wire_len} bytes, "
                f"only {len(data) - offset} available"
            )
        packets.append(data[offset:end])
        offset = end
    if offset != len(data):
        raise ValueError(f"Trailing {len(data) - offset} byte(s) after last record")
    return packets, []


def _split_len_prefixed_terminated(data: bytes, terminator: bytes):
    packets = []
    warnings = []
    offset = 0
    tl = len(terminator)
    while offset < len(data):
        if offset + 1 > len(data):
            raise ValueError(f"Incomplete length prefix at file offset {offset}")
        wire_len = data[offset]
        if wire_len < 2 or wire_len > TELEMETRY_WIRE_LENGTH_MAX:
            raise ValueError(f"Invalid wireLength {wire_len} at file offset {offset}")
        end = offset + wire_len
        if end + tl > len(data):
            raise ValueError(f"Incomplete record or missing terminator at offset {offset}")
        if data[end : end + tl] != terminator:
            raise ValueError(
                f"Missing expected terminator {terminator!r} after record at offset {offset}"
            )
        packets.append(data[offset:end])
        offset = end + tl
    if offset != len(data):
        raise ValueError("Trailing data after last terminated record")
    return packets, warnings


def _split_fixed98_raw(data: bytes):
    if len(data) % LEGACY_PACKET_SIZE != 0:
        raise ValueError(
            f"Input length {len(data)} is not a multiple of {LEGACY_PACKET_SIZE}"
        )
    return [
        data[i : i + LEGACY_PACKET_SIZE] for i in range(0, len(data), LEGACY_PACKET_SIZE)
    ], []


def _split_fixed98_terminated(data: bytes, record_format: str):
    terminator = LEGACY_TERMINATORS[record_format]
    payload_size = LEGACY_PACKET_SIZE
    record_size = payload_size + len(terminator)
    if len(data) % record_size != 0:
        raise ValueError(
            f"Input length {len(data)} is not a multiple of {record_size} "
            f"for legacy terminated format"
        )
    packets = []
    discarded_chunks = []
    for offset in range(0, len(data), record_size):
        payload_end = offset + payload_size
        if data[payload_end : payload_end + len(terminator)] != terminator:
            raise ValueError(f"Bad terminator at offset {offset}")
        packets.append(data[offset : offset + payload_size])
        discarded_chunks.append(data[offset + payload_size : payload_end])

    warnings = []
    extra = payload_size - LEGACY_PACKET_SIZE
    if extra != 0:
        pass
    if any(any(chunk) for chunk in discarded_chunks):
        warnings.append(
            "Legacy format: non-zero bytes between 98-byte payload and terminator "
            "(unexpected)."
        )
    return packets, warnings


def _split_records_for_format(data: bytes, record_format: str):
    if record_format == RECORD_FORMAT_LEN_RAW:
        return _split_len_prefixed_raw(data)
    if record_format in LEN_TERMINATORS:
        return _split_len_prefixed_terminated(data, LEN_TERMINATORS[record_format])
    if record_format == RECORD_FORMAT_FIXED98_RAW:
        return _split_fixed98_raw(data)
    if record_format in LEGACY_TERMINATORS:
        return _split_fixed98_terminated(data, record_format)
    raise ValueError(f"Unknown record format {record_format!r}")


def _split_records(data: bytes, record_format: str = RECORD_FORMAT_AUTO):
    if len(data) == 0:
        return [], record_format, []
    if record_format != RECORD_FORMAT_AUTO:
        packets, warnings = _split_records_for_format(data, record_format)
        return packets, record_format, warnings

    format_order = (
        RECORD_FORMAT_LEN_CRLF,
        RECORD_FORMAT_LEN_LF,
        RECORD_FORMAT_LEN_CR,
        RECORD_FORMAT_LEN_RAW,
        RECORD_FORMAT_FIXED98_RAW,
        RECORD_FORMAT_FIXED98_CRLF,
        RECORD_FORMAT_FIXED98_LF,
        RECORD_FORMAT_FIXED98_CR,
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
        "Could not auto-detect packet record format. Tried length-prefixed "
        "(CRLF/LF/CR/raw), then legacy fixed 98-byte layouts. Details: "
        + "; ".join(errors)
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
        self.decode_as_legacy = False
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
            width=48,
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
        self.decode_as_legacy = detected_format in LEGACY_DECODE_FORMATS
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
        try:
            if self.decode_as_legacy:
                rows = decode_legacy_packet(packet)
            else:
                rows = decode_packet(packet)
        except ValueError as exc:
            messagebox.showerror("Decode error", str(exc))
            return
        for row in rows:
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
    all_formats = (
        RECORD_FORMAT_AUTO,
        RECORD_FORMAT_LEN_RAW,
        RECORD_FORMAT_LEN_LF,
        RECORD_FORMAT_LEN_CR,
        RECORD_FORMAT_LEN_CRLF,
        RECORD_FORMAT_FIXED98_RAW,
        RECORD_FORMAT_FIXED98_LF,
        RECORD_FORMAT_FIXED98_CR,
        RECORD_FORMAT_FIXED98_CRLF,
    )
    parser = argparse.ArgumentParser(description="View Spencer Board telemetry packets")
    parser.add_argument("path", nargs="?", help="Optional telemetry log path to open on startup")
    parser.add_argument(
        "--record-format",
        choices=all_formats,
        default=RECORD_FORMAT_AUTO,
        help="Packet record format when opening the optional startup path.",
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
