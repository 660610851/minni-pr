"""Dependency-free command lease used on robot and in unit tests."""
import math

MAX_LINEAR = 0.2
MAX_ANGULAR = 0.2

def clamp(a,lo,hi):return max(lo,min(hi,a))


class CommandLease:
    """Robot-local input lease. Expired/future commands yield zero."""
    def __init__(self,timeout=.4):self.timeout=timeout;self.last=-math.inf;self.v=self.w=0.
    def accept(self,v,w,receipt,age):
        if not all(math.isfinite(a) for a in (v,w,receipt,age)) or not -.1<=age<=self.timeout:
            self.last=-math.inf;return False
        self.v=clamp(v,0.,MAX_LINEAR);self.w=clamp(w,-MAX_ANGULAR,MAX_ANGULAR);self.last=receipt;return True
    def output(self,now):
        return (self.v,self.w) if 0<=now-self.last<=self.timeout else (0.,0.)
