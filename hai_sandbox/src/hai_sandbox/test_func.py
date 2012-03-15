class aa:
    def __init__(self):
        pass

    def run(self):
        self.change_detect(self.a, ['hello'])

    def a(self, b):
        print b
    
    def change_detect(self, f, args):
        print len(args), args.__class__
        print 'before'
        f(*args)
        print 'after'

m = aa()
m.run()
