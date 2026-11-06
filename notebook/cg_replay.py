"""Helper untuk replay dataset ConnectorGuard-HDC ke FPGA (format beat sama dengan rtl/cg_replay_top.sv)."""
from pathlib import Path
import numpy as np
import pandas as pd

KIND_EMPTY, KIND_SAMPLE, KIND_CMD = 0, 1, 2
END_MARKER = 0xE0F0E0F0


def pack_stimulus(dataset_dir):
    """raw_input.csv + commands.csv -> array uint32 (N, 4); satu baris = satu beat 128 bit."""
    d = Path(dataset_dir)
    raw = pd.read_csv(d / "connectorguard_raw_input.csv")
    cmd = pd.read_csv(d / "connectorguard_commands.csv")
    cmd["addr"] = cmd["addr_hex"].map(lambda x: int(x, 16))
    cmd["data"] = cmd["data_hex"].map(lambda x: int(x, 16))
    by_slot = {}
    for r in cmd.itertuples():
        by_slot.setdefault(int(r.after_slot), []).append((r.addr, r.data))

    u16 = lambda col: raw[col].to_numpy(np.int64).astype(np.uint16).astype(np.uint32)
    i, v, t, ta = u16("i"), u16("v"), u16("t"), u16("tamb")
    ires = raw["ires"].to_numpy(np.int64).astype(np.uint32) & 0xFFFF
    valid = raw["valid"].to_numpy(np.int64)

    beats = []
    def add_cmds(slot):
        for addr, data in by_slot.get(slot, []):
            beats.append((data & 0xFFFFFFFF, addr, 0, KIND_CMD))
    add_cmds(-1)
    for s in range(len(raw)):
        if valid[s]:
            beats.append((int(i[s]) | (int(v[s]) << 16), int(t[s]) | (int(ta[s]) << 16), int(ires[s]), KIND_SAMPLE))
        else:
            beats.append((0, 0, 0, KIND_EMPTY))
        add_cmds(s)
    return np.array(beats, dtype=np.uint32)


STATUS_FIELDS = {  # bit layout of STATUS (0x01) in cg_m9_regs.sv
    "state": (0, 2), "hard_trip": (2, 1), "fsm_trip": (3, 1), "sensor_fault": (4, 1), "stale": (5, 1),
    "arc": (6, 1), "cfg_locked": (7, 1), "contactor": (8, 1), "cfg_wr_err": (9, 1), "cmd_rej": (10, 1),
    "m5b_ready": (11, 1), "m6_ready": (12, 1), "train_active": (13, 1), "m6_cls": (14, 2),
    "m8_cause": (16, 5), "cls_trained": (21, 3), "m5b_ovr": (24, 1), "m6_ovr": (25, 1),
}


def decode_log(buf):
    """buf: array uint32 (n, 8) hasil S2MM -> DataFrame per window (marker akhir dibuang)."""
    a = np.asarray(buf, dtype=np.uint32).reshape(-1, 8)
    ok = ~((a[:, 0] == 0xFFFFFFFF) & (a[:, 7] == END_MARKER))
    a = a[ok]
    df = pd.DataFrame({"window": a[:, 0] >> 16, "last_slot": a[:, 0] & 0xFFFF})
    for name, (lo, n) in STATUS_FIELDS.items():
        df[name] = (a[:, 1] >> lo) & ((1 << n) - 1)
    df["r_comp"] = a[:, 2] & 0xFFFF
    df["th_idx"] = a[:, 2] >> 16
    df["e_band"] = a[:, 3] & 0xFFFFFF
    df["d_adapt"] = a[:, 4] & 0xFFFF
    df["d_frozen"] = a[:, 4] >> 16
    df["health"] = a[:, 5] & 0xFF
    df["cp_duty"] = (a[:, 5] >> 8) & 0xFF
    df["clk"] = a[:, 6]
    return df


COMPARE_COLS = ["state", "hard_trip", "contactor", "cp_duty", "health", "m8_cause", "cfg_locked",
                "train_active", "sensor_fault", "r_comp", "th_idx", "e_band", "d_adapt", "d_frozen", "m6_cls"]


def compare(log, dataset_dir):
    """Bandingkan log hardware dengan output RTL hasil simulasi di dataset (validation/rtl_output.csv)."""
    ref = pd.read_csv(Path(dataset_dir) / "validation" / "rtl_output.csv")
    ref = ref.rename(columns={"contactor": "contactor"})
    ref["arc"] = ref["arc_confirmed"]
    n = min(len(ref), len(log))
    bad = {}
    for c in COMPARE_COLS + ["arc"]:
        if c == "arc":
            continue
        diff = (log[c].to_numpy()[:n] != ref[c].to_numpy()[:n])
        if diff.any():
            bad[c] = np.flatnonzero(diff)[:10] + 1
    arc_diff = log["arc"].to_numpy()[:n] != ref["arc"].to_numpy()[:n]
    if arc_diff.any():
        bad["arc"] = np.flatnonzero(arc_diff)[:10] + 1
    return {"windows_hw": len(log), "windows_ref": len(ref), "mismatch": bad}
