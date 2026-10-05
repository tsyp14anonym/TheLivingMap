"""Priority 0 goes out at once (with its context). Others flush on 120 s OR size OR high score."""
from .beacon_protocol import queue_score

class AdaptiveQueue:
    def __init__(self, send, interval=120.0, max_len=8, theta=20.0, immediate=False):
        self.send, self.interval, self.max_len, self.theta, self.immediate = send, interval, max_len, theta, immediate
        self.pending, self.last_flush = [], 0.0

    def _q(self, r, now):
        return queue_score(r["type"], r["prio"], r["n"], now - r["t0"], now - r["enq_t"])

    def push(self, rec, now, context_fn=None):
        rec["enq_t"] = now
        if self.immediate: self.send([rec], "IMMEDIATE"); return                 # no batching: every beacon goes to the Command Post as soon as the van has it
        if rec["prio"] == 0 or rec.get("escalate"):                       # bypass + carry the context needed to read it
            ctx = [p for p in self.pending if context_fn and context_fn(rec, p)]
            for p in ctx: self.pending.remove(p)
            self.send(ctx + [rec], "P0_IMMEDIATE")
            return
        self.pending.append(rec)
        self.tick(now)

    def tick(self, now):
        if not self.pending: return
        why = None
        if now - self.last_flush >= self.interval: why = "BATCH_120S"
        elif len(self.pending) >= self.max_len: why = "ADAPTIVE_SIZE"
        elif max(self._q(r, now) for r in self.pending) >= self.theta: why = "ADAPTIVE_SCORE"
        if why:
            batch = sorted(self.pending, key=lambda r: -self._q(r, now))
            self.pending, self.last_flush = [], now
            self.send(batch, why)
