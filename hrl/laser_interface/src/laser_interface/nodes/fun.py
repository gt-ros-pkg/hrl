########################################################################################
# Functional Programmizers
# Set of functions to support functional programming
########################################################################################
def split(func, list):
    """ 
        Like filter but returns both elements that satisfy the given condition and those 
        that do not.
    """
    true_items = []
    false_items = []
    for el in list:
        if func(el):
            true_items.append(el)
        else:
            false_items.append(el)

    return (true_items, false_items)

def chain(list_of_lists):
    """ Like itertools.chain except accept a list (potentially iterator) as input """
    for l in list_of_lists:
        for el in l:
            yield el

def rm_exception(f):
   """Remove exceptions from given functions, but caller has to check for None"""
   def w(*x,**y):
      try:
         return f(*x,**y)
      except BaseException, e:
         return None
   return w


def iter(f):
   """Turn given function into an iterator assuming that it returns None at termination"""
   def w(*x,**y):
      while 1:
         result = f(*x,**y)
         if (result != None):
            yield result
         else:
            raise StopIteration
   return w


def eiter(f):
   """Like iterize but handles function that throw exceptions"""
   return iter(rm_exception(f))


def repeat(f, arg):
   while True:
      yield apply(f,arg)


def iterate(f, arg):
   """ Keeps applying f on its own results """
   last_arg = arg
   while True:
      val = apply(f, last_arg)
      last_arg = [val]
      yield val


def truncate(iter, n):
   for i in xrange(n):
      yield iter.next()


def within_eps(iter, eps, last_val=None):
   a = None
   if last_val == None:
      a = iter.next()
   else:
      a = last_val

   b = iter.next()
   if (abs (a - b) <= eps):
      return b
   else:
      return within_eps(iter, eps, last_val = b)


def collect(iter):
   return map(None, iter)


def points_of_mat(mat):
   num_el = mat.shape[1]
   #print "mat.shape", mat.shape, mat.shape[0], mat.shape[1]
   for i in xrange(num_el):
      yield mat[:,i]


########################################################################################
##
## Collection of functions copied over from Python docs:
##    http://docs.python.org/lib/itertools-recipes.html
##
########################################################################################
import itertools as it

def take(n, seq):
    return list(islice(seq, n))

def enumerate(iterable):
    return izip(count(), iterable)

def tabulate(function):
    "Return function(0), function(1), ..."
    return imap(function, count())

def iteritems(mapping):
    return izip(mapping.iterkeys(), mapping.itervalues())

def nth(iterable, n):
    "Returns the nth item"
    return list(islice(iterable, n, n+1))

def all(seq, pred=None):
    "Returns True if pred(x) is true for every element in the iterable"
    for elem in ifilterfalse(pred, seq):
        return False
    return True

def any(seq, pred=None):
    "Returns True if pred(x) is true for at least one element in the iterable"
    for elem in ifilter(pred, seq):
        return True
    return False

def no(seq, pred=None):
    "Returns True if pred(x) is false for every element in the iterable"
    for elem in ifilter(pred, seq):
        return False
    return True

def quantify(seq, pred=None):
    "Count how many times the predicate is true in the sequence"
    return sum(imap(pred, seq))

def padnone(seq):
    """Returns the sequence elements and then returns None indefinitely.

    Useful for emulating the behavior of the built-in map() function.
    """
    return it.chain(seq, repeat(None))

def ncycles(seq, n):
    "Returns the sequence elements n times"
    return it.chain(*repeat(seq, n))

def dotproduct(vec1, vec2):
    return sum(imap(operator.mul, vec1, vec2))

def flatten(listOfLists):
    return list(it.chain(*listOfLists))

def repeatfunc(func, times=None, *args):
    """Repeat calls to func with specified arguments.
    
    Example:  repeatfunc(random.random)
    """
    if times is None:
        return starmap(func, repeat(args))
    else:
        return starmap(func, repeat(args, times))

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    try:
        b.next()
    except StopIteration:
        pass
    return izip(a, b)

def grouper(n, iterable, padvalue=None):
    "grouper(3, 'abcdefg', 'x') --> ('a','b','c'), ('d','e','f'), ('g','x','x')"
    return izip(*[it.chain(iterable, repeat(padvalue, n-1))]*n)
