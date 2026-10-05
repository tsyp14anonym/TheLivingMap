"""32-byte beacon v2 with EGOCENTRIC directions (no GPS, no absolute coordinates inside the building).
Every beacon says: 'I am H metres ahead / R metres to the right of my parent beacon (in the parent's heading),
on floor F. The thing I found is T metres ahead / U metres to the right of ME.'  Positions are rebuilt by chaining hops."""
import struct, hmac, hashlib, math
from dataclasses import dataclass

MAGIC = 0xB6
KEY = b"living-map-demo-key"
TYPES = {"REPEATER": 0, "FIRE": 1, "GAS": 2, "HUMAN": 3, "ANIMAL": 4, "DEBRIS": 5, "BEACON_LOST": 6, "STAIRS": 7}
TYPE_NAMES = {v: k for k, v in TYPES.items()}
FLAG_HUMAN_NEEDED = 1   # human_intervention_required
FLAG_VERIFIED = 2
#      magic id  org seq t0  tpf flr n  hdg hopF hopR tgtF tgtR parent temp gas mirror_id mirror_seq
_FMT = ">B  H   B   H   I   B   B   B  B   b    b    b    b    H      h    H   H         B".replace(" ", "")
BODY = struct.calcsize(_FMT)
PACKET_SIZE = BODY + 5
assert PACKET_SIZE == 32, PACKET_SIZE

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8): crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
def _mac(data: bytes) -> bytes: return hmac.new(KEY, data, hashlib.sha256).digest()[:3]
def _clip(v, lo, hi): return max(lo, min(hi, int(round(v))))

# ---- egocentric geometry: heading theta (rad, CCW from +x); forward=(cos,sin); right=(sin,-cos) ----
def quant_heading(theta): return int(round((theta % (2 * math.pi)) / (2 * math.pi) * 256)) % 256
def heading_rad(hq): return hq * 2 * math.pi / 256
def to_body(frm, hq, to):
    """world offset frm->to expressed as (metres ahead, metres to the right) for a beacon with quantised heading hq."""
    th = heading_rad(hq); dx, dy = to[0] - frm[0], to[1] - frm[1]
    return dx * math.cos(th) + dy * math.sin(th), dx * math.sin(th) - dy * math.cos(th)
def from_body(frm, hq, ahead, right):
    th = heading_rad(hq)
    return frm[0] + ahead * math.cos(th) + right * math.sin(th), frm[1] + ahead * math.sin(th) - right * math.cos(th)
def describe(ahead, right):
    """Human readable: '4.0 m ahead, 3.0 m left'."""
    fb = f"{abs(ahead):.1f} m {'ahead' if ahead >= 0 else 'behind'}"; lr = f"{abs(right):.1f} m {'right' if right >= 0 else 'left'}"
    return f"{fb}, {lr}"

@dataclass
class Beacon:
    bid: int; origin: int; seq: int; t0: int; type: int; prio: int; flags: int; floor: int; n: int; heading: int
    hop_f: int; hop_r: int; tgt_f: int; tgt_r: int; parent: int; temp_dc: int; gas: int; mirror_id: int = 0; mirror_seq: int = 0

def make_beacon(bid, origin, seq, t0, type_name, prio, flags, floor, n, heading_q, hop, tgt, parent, temp=25.0, gas=0.0, mirror=(0, 0)):
    """hop = (ahead, right) metres from the PARENT beacon; tgt = (ahead, right) metres from THIS beacon (its own heading)."""
    return Beacon(bid, origin, seq & 0xFFFF, _clip(t0, 0, 2**32 - 1), TYPES[type_name], prio, flags, floor, n, heading_q,
                  _clip(hop[0] * 10, -128, 127), _clip(hop[1] * 10, -128, 127), _clip(tgt[0] * 10, -128, 127), _clip(tgt[1] * 10, -128, 127),
                  parent, _clip(temp * 10, -32768, 32767), _clip(gas, 0, 65535), mirror[0], mirror[1] & 0xFF)

def pack(b: Beacon) -> bytes:
    tpf = (b.type & 0xF) << 4 | (b.prio & 3) << 2 | (b.flags & 3)
    body = struct.pack(_FMT, MAGIC, b.bid, b.origin, b.seq, b.t0, tpf, b.floor, b.n, b.heading, b.hop_f, b.hop_r, b.tgt_f, b.tgt_r,
                       b.parent, b.temp_dc, b.gas, b.mirror_id, b.mirror_seq)
    body += struct.pack(">H", crc16(body)); return body + _mac(body)

def unpack(data: bytes) -> Beacon:
    if len(data) != PACKET_SIZE: raise ValueError("bad length")
    body, crc, mac = data[:BODY], data[BODY:BODY + 2], data[BODY + 2:]
    if struct.unpack(">H", crc)[0] != crc16(body): raise ValueError("bad CRC")
    if not hmac.compare_digest(mac, _mac(body + crc)): raise ValueError("bad MAC")
    (magic, bid, origin, seq, t0, tpf, floor, n, hdg, hf, hr, tf, tr, parent, temp, gas, mid, mseq) = struct.unpack(_FMT, body)
    if magic != MAGIC: raise ValueError("bad magic")
    return Beacon(bid, origin, seq, t0, tpf >> 4, (tpf >> 2) & 3, tpf & 3, floor, n, hdg, hf, hr, tf, tr, parent, temp, gas, mid, mseq)

# ---- two-clock priority: confidence decays, waiting time raises queue priority ----
P_BASE = {0: 10.0, 1: 6.0, 2: 3.0, 3: 1.0}
W_VICTIM = {"HUMAN": 3.0, "ANIMAL": 1.0}
H_HAZARD = {"GAS": 10.0, "FIRE": 6.0}
LAMBDA_PER_MIN = {"FIRE": 0.10, "GAS": 0.10, "HUMAN": 0.01, "ANIMAL": 0.01, "DEBRIS": 0.0, "REPEATER": 0.0, "BEACON_LOST": 0.0, "STAIRS": 0.0}
BETA = 0.01
def u0(type_name, prio, n): return P_BASE[prio] * W_VICTIM.get(type_name, 1.0) * max(n, 1 if type_name in ("HUMAN", "ANIMAL", "FIRE", "GAS") else 1) + H_HAZARD.get(type_name, 0.0)
def confidence(type_name, age_s): return math.exp(-LAMBDA_PER_MIN[type_name] * max(age_s, 0.0) / 60.0)
def queue_score(type_name, prio, n, age_s, wait_s): return u0(type_name, prio, n if type_name in ("HUMAN", "ANIMAL") else 1) * confidence(type_name, age_s) * (1.0 + BETA * max(wait_s, 0.0))


# ---- continuous priority pi in [0, +inf): LOWER = MORE URGENT, and it DROPS with time so that nothing waits forever ----
BASE_PI = {"HUMAN": 3.0, "FIRE": 5.0, "GAS": 6.0, "ANIMAL": 10.0, "DEBRIS": 30.0, "BEACON_LOST": 60.0, "STAIRS": 80.0, "REPEATER": 100.0}
CLASS_FACTOR = {0: 0.15, 1: 1.0, 2: 1.5, 3: 2.0}     # the Writer's 2-bit class: 0 = critical ... 3 = routine
AGING = 0.004                                        # per second: after ~3 min a waiting event is about half as 'heavy'
def priority(type_name, cls, n, age_s):
    pi0 = BASE_PI[type_name] * CLASS_FACTOR[cls] / max(n, 1)
    return pi0 * math.exp(-AGING * max(age_s, 0.0))
