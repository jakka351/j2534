#!/usr/bin/env python3
"""
j2534_send.py — Minimal J2534 sender/receiver for Linux (Raspberry Pi)

- Loads either `libj2534.so` or `j2534.so` (whichever you built/installed)
- Opens the first J2534 device (PassThruOpen)
- Connects on CAN or ISO15765 at the requested bitrate (PassThruConnect)
- Sends ONE message (PassThruWriteMsgs)
- Reads responses for N milliseconds (PassThruReadMsgs)
- Disconnects and closes

USAGE EXAMPLES
--------------
# Raw CAN, 500 kbit, 11-bit ID 0x7DF, data 02 01 00, wait 1000ms for replies:
sudo python3 j2534_send.py --protocol can --baud 500000 --id 0x7DF --data 02 01 00 --wait 1000

# Raw CAN, 500 kbit, 29-bit ID 0x18DB33F1 (set --ext), read for 2s:
sudo python3 j2534_send.py --protocol can --baud 500000 --id 0x18DB33F1 --ext --data 02 10 01 --wait 2000

# ISO-TP (ISO15765) session (some devices require filters; this tries a permissive pass filter):
sudo python3 j2534_send.py --protocol iso15765 --baud 500000 --id 0x7E0 --data 02 10 01 --wait 2000
"""
import argparse
import ctypes as ct
import os
import sys
from typing import List

# ---------- J2534 constants (subset) ----------
STATUS_NOERROR          = 0x00

PROTOCOL_CAN            = 0x00000005
PROTOCOL_ISO15765       = 0x00000006

# TxFlags / RxStatus bits (subset)
CAN_29BIT_ID            = 0x00000100

# IOCTLs (subset)
# Not all stacks require these; ISO15765 often needs a filter to pass responses.
SET_CONFIG                    = 0x00000001
GET_CONFIG                    = 0x00000002
START_MSG_FILTER              = 0x00000003
STOP_MSG_FILTER               = 0x00000004
READ_VBATT                    = 0x00000005
FIVE_BAUD_INIT                = 0x00000006
FAST_INIT                     = 0x00000007
CLEAR_TX_BUFFER               = 0x00000008
CLEAR_RX_BUFFER               = 0x00000009
CLEAR_PERIODIC_MSGS           = 0x0000000A
CLEAR_MSG_FILTERS             = 0x0000000B
CLEAR_FUNCT_MSG_LOOKUP_TABLE  = 0x0000000C
ADD_TO_FUNCT_MSG_LOOKUP_TABLE = 0x0000000D

# Filter types
PASS_FILTER             = 0x00000001
BLOCK_FILTER            = 0x00000002
FLOW_CONTROL_FILTER     = 0x00000003

# Config parameters (subset)
DATA_RATE               = 0x00000101

# SAE J2534 PASSTHRU_MSG structure (use fixed 32-bit fields)
class PASSTHRU_MSG(ct.Structure):
    _fields_ = [
        ("ProtocolID",     ct.c_uint32),
        ("RxStatus",       ct.c_uint32),
        ("TxFlags",        ct.c_uint32),
        ("Timestamp",      ct.c_uint32),
        ("DataSize",       ct.c_uint32),
        ("ExtraDataIndex", ct.c_uint32),
        ("Data",           ct.c_ubyte * 4128),
    ]

class SCONFIG(ct.Structure):
    _fields_ = [("Parameter", ct.c_uint32), ("Value", ct.c_uint32)]

class SCONFIG_LIST(ct.Structure):
    _fields_ = [("NumOfParams", ct.c_uint32), ("ConfigPtr", ct.POINTER(SCONFIG))]

def load_j2534() -> ct.CDLL:
    """
    Try loading libj2534 from common locations:
    - J2534_LIB env var path
    - libj2534.so (system)
    - j2534.so (local)
    """
    candidates: List[str] = []
    envp = os.getenv("J2534_LIB")
    if envp:
        candidates.append(envp)
    candidates.extend(["libj2534.so", "j2534.so", "./libj2534.so", "./j2534.so"])
    last_err = None
    for p in candidates:
        try:
            return ct.CDLL(p)
        except OSError as e:
            last_err = e
    raise SystemExit(f"Could not load J2534 library (tried {candidates}): {last_err}")

def set_prototypes(dll: ct.CDLL):
    # unsigned long -> use 32-bit types for Linux/ARM consistency
    u32 = ct.c_uint32
    ptr = ct.POINTER
    voidp = ct.c_void_p

    dll.PassThruOpen.argtypes      = [voidp, ptr(u32)]
    dll.PassThruOpen.restype       = u32
    dll.PassThruClose.argtypes     = [u32]
    dll.PassThruClose.restype      = u32
    dll.PassThruConnect.argtypes   = [u32, u32, u32, u32, ptr(u32)]
    dll.PassThruConnect.restype    = u32
    dll.PassThruDisconnect.argtypes= [u32]
    dll.PassThruDisconnect.restype = u32
    dll.PassThruReadMsgs.argtypes  = [u32, ptr(PASSTHRU_MSG), ptr(u32), u32]
    dll.PassThruReadMsgs.restype   = u32
    dll.PassThruWriteMsgs.argtypes = [u32, ptr(PASSTHRU_MSG), ptr(u32), u32]
    dll.PassThruWriteMsgs.restype  = u32
    dll.PassThruStartMsgFilter.argtypes = [u32, u32, ptr(PASSTHRU_MSG), ptr(PASSTHRU_MSG), ptr(PASSTHRU_MSG), ptr(u32)]
    dll.PassThruStartMsgFilter.restype  = u32
    dll.PassThruIoctl.argtypes     = [u32, u32, voidp, voidp]
    dll.PassThruIoctl.restype      = u32
    try:
        dll.PassThruReadVersion.argtypes = [u32, ct.c_char_p, ct.c_char_p, ct.c_char_p]
        dll.PassThruReadVersion.restype  = u32
    except AttributeError:
        pass

def hex_or_int(s: str) -> int:
    s = s.strip()
    return int(s, 16) if s.lower().startswith("0x") else int(s)

def parse_bytes(xs: List[str]) -> bytes:
    out = bytearray()
    for t in xs:
        if t == "": continue
        out.append(hex_or_int(t) & 0xFF)
    return bytes(out)

def pack_can_msg(protocol: int, can_id: int, data: bytes, extended: bool, id_le=True) -> PASSTHRU_MSG:
    m = PASSTHRU_MSG()
    m.ProtocolID = protocol
    m.RxStatus   = 0
    m.TxFlags    = CAN_29BIT_ID if extended else 0
    m.Timestamp  = 0
    # J2534 CAN: first 4 bytes = CAN ID (endianness varies by vendor; try little-endian first)
    if id_le:
        id_bytes = can_id.to_bytes(4, "little")
    else:
        id_bytes = can_id.to_bytes(4, "big")
    payload = id_bytes + data
    m.DataSize = len(payload)
    m.ExtraDataIndex = 0
    m.Data[0:len(payload)] = payload
    return m

def print_msg(prefix: str, m: PASSTHRU_MSG, id_le=True):
    ds = int(m.DataSize)
    raw = bytes(m.Data[:ds])
    if ds >= 4:
        can_id_le = int.from_bytes(raw[:4], "little")
        can_id_be = int.from_bytes(raw[:4], "big")
        can_id = can_id_le if id_le else can_id_be
        payload = raw[4:]
        ext = bool(m.RxStatus & CAN_29BIT_ID or m.TxFlags & CAN_29BIT_ID)
        print(f"{prefix} id=0x{can_id:08X} ext={ext} len={len(payload)} data={' '.join(f'{b:02X}' for b in payload)}")
    else:
        print(f"{prefix} raw={raw.hex()}")

def main():
    ap = argparse.ArgumentParser(description="Minimal J2534 sender/receiver (CAN / ISO15765)")
    ap.add_argument("--protocol", choices=["can","iso15765"], default="can")
    ap.add_argument("--baud", type=int, default=500000, help="bitrate (e.g., 500000)")
    ap.add_argument("--id", type=hex_or_int, default=0x7DF, help="CAN arbitration ID")
    ap.add_argument("--ext", action="store_true", help="Use 29-bit (extended) ID")
    ap.add_argument("--data", nargs="*", default=["02","01","00"], help="Payload bytes (space-separated, hex like 0x01 or bare 01/1)")
    ap.add_argument("--wait", type=int, default=1000, help="read window after send (ms)")
    ap.add_argument("--id-be", action="store_true", help="Store CAN ID big-endian instead of little-endian (vendor quirk)")
    args = ap.parse_args()

    dll = load_j2534()
    set_prototypes(dll)

    # Open device
    dev_id = ct.c_uint32(0)
    st = dll.PassThruOpen(None, ct.byref(dev_id))
    if st != STATUS_NOERROR:
        sys.exit(f"PassThruOpen failed: 0x{st:08X}")
    try:
        # Optional version read
        try:
            dll_ver = ct.create_string_buffer(128)
            fw_ver  = ct.create_string_buffer(128)
            api_ver = ct.create_string_buffer(128)
            if dll.PassThruReadVersion(dev_id.value, dll_ver, fw_ver, api_ver) == STATUS_NOERROR:
                print(f"DLL:{dll_ver.value.decode()} FW:{fw_ver.value.decode()} API:{api_ver.value.decode()}")
        except Exception:
            pass

        # Connect
        protocol = PROTOCOL_CAN if args.protocol == "can" else PROTOCOL_ISO15765
        chan_id = ct.c_uint32(0)
        st = dll.PassThruConnect(dev_id.value, protocol, 0, ct.c_uint32(args.baud), ct.byref(chan_id))
        if st != STATUS_NOERROR:
            # Some stacks want DATA_RATE via SET_CONFIG (esp. ISO15765). Try politely:
            cfg = (SCONFIG * 1)()
            cfg[0].Parameter = DATA_RATE
            cfg[0].Value     = args.baud
            cfg_list = SCONFIG_LIST(1, ct.cast(ct.pointer(cfg), ct.POINTER(SCONFIG)))
            dll.PassThruIoctl(dev_id.value, SET_CONFIG, ct.byref(cfg_list), None)
            st = dll.PassThruConnect(dev_id.value, protocol, 0, ct.c_uint32(args.baud), ct.byref(chan_id))
            if st != STATUS_NOERROR:
                raise SystemExit(f"PassThruConnect failed: 0x{st:08X}")

        # Clear buffers
        dll.PassThruIoctl(chan_id.value, CLEAR_RX_BUFFER, None, None)
        dll.PassThruIoctl(chan_id.value, CLEAR_TX_BUFFER, None, None)

        # If ISO15765, try a permissive PASS_FILTER to let replies through
        if protocol == PROTOCOL_ISO15765:
            filter_id = ct.c_uint32(0)
            mask = PASSTHRU_MSG(); patt = PASSTHRU_MSG(); flow = PASSTHRU_MSG()
            # Let everything pass (mask 0, pattern 0) — some stacks require at least *something*
            st = dll.PassThruStartMsgFilter(chan_id.value, PASS_FILTER,
                                            ct.byref(mask), ct.byref(patt), None, ct.byref(filter_id))
            # Ignore errors; many libs allow read without filters

        # Build and send ONE frame
        data = parse_bytes(args.data)
        msg = pack_can_msg(protocol, args.id, data, args.ext, id_le=not args.id_be)
        n = ct.c_uint32(1)
        st = dll.PassThruWriteMsgs(chan_id.value, ct.byref(msg), ct.byref(n), 100)
        if st != STATUS_NOERROR or n.value != 1:
            raise SystemExit(f"PassThruWriteMsgs failed: 0x{st:08X}, wrote={n.value}")
        print_msg("TX", msg, id_le=not args.id_be)

        # Read window
        if args.wait > 0:
            buf_count = 64
            arr = (PASSTHRU_MSG * buf_count)()
            n = ct.c_uint32(buf_count)
            st = dll.PassThruReadMsgs(chan_id.value, arr, ct.byref(n), ct.c_uint32(args.wait))
            if st == STATUS_NOERROR and n.value > 0:
                for i in range(n.value):
                    print_msg("RX", arr[i], id_le=not args.id_be)
            else:
                print(f"RX: no messages within {args.wait} ms (status=0x{st:08X})")

        # Done
        dll.PassThruDisconnect(chan_id.value)
    finally:
        dll.PassThruClose(dev_id.value)

if __name__ == "__main__":
    main()
