

class Myclass:
    def __init__(self, a):
        self.a = a

    @classmethod
    def class_method(cls, a_value):
        return Myclass(a_value)


mc = Myclass.class_method(2)
print mc.a
