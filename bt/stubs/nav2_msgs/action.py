
class _M:
    def __init__(self, **kw):
        for k,v in kw.items(): setattr(self,k,v)
    def __getattr__(self, k):
        if k.startswith('__'): raise AttributeError(k)
        v=_M(); setattr(self,k,v); return v

class NavigateToPose:
    class Goal(_M): pass
