import sqlite3
import numpy as np
import pickle as pk

#tables will be of the form
## Table given a topic name - /some/path/to/topic_a => ?
# Time Message

def create_raw_table(topic_name, cur):
    cur.execute("CREATE TABLE raw_data (id text PRIMARY KEY," \
                    + "surface_id integer, "\
                    + "surface_height real, " \
                    + "unrotated_points matrix, " \
                    + "normal matrix, " \
                    + "labels matrix, " \
                    + "image text, "  \
                    + "intensities matrix, "  \
                    + "points matrix)")


def pickle_adapter(some_obj):
    return pk.dumps(some_obj)

def pickle_converter(pkl_str):
    try:
        if len(pkl_str) == 0:
            print "pickle_converter: WARNING -- encountered string of length 0"
            return None
        return pk.loads(pkl_str)
    except EOFError, e:
        import pdb
        pdb.set_trace()
        print 'error'
    except ValueError, e:
        import pdb
        pdb.set_trace()
        print 'error'
        return None

