
class _M:
    def __init__(self, **kw):
        for k,v in kw.items(): setattr(self,k,v)
    def __getattr__(self, k):
        if k.startswith('__'): raise AttributeError(k)
        v=_M(); setattr(self,k,v); return v

class Takeoff:
    class Request(_M): pass
class GoTo:
    class Request(_M): pass
class Land:
    class Request(_M): pass
