"""Payload sensor adapter. ACK confirms FC command, not physical release.
External feedback must be atomically replaced; monotonic clock is that of same Pi.
"""
import json, time
from pathlib import Path
import config as C
from serial_comm import FlightError
class PayloadFeedback:
    def read(self,slot,color,*,present):
        try:
            raw=json.loads(Path(C.PAYLOAD_FEEDBACK_FILE).read_text(encoding='utf-8'))
            stamp=raw['sample_monotonic_s']; event=raw['event_id']; item=raw['slots'][str(slot)]
            if not C.number(stamp) or not 0<=time.monotonic()-stamp<=0.5: raise ValueError('stale feedback')
            if type(event)!=int or item['color']!=color or item['present'] is not present: return None
            return event
        except (OSError,ValueError,KeyError,TypeError) as exc: raise FlightError('Payload feedback invalid: '+str(exc)) from exc
class MockPayloadFeedback:
    def __init__(self): self.event=1; self.released=set(); self.jammed=False
    def read(self,slot,color,*,present):
        if ((slot not in self.released)==present): return self.event
        return None
    def release(self,slot):
        if not self.jammed: self.released.add(slot); self.event+=1
