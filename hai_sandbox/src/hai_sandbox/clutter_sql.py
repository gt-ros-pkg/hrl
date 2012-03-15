import sqlite3
import numpy as np
import pickle as pk
import hrl_lib.util as ut

def create_raw_table(cur):
    cur.execute("CREATE TABLE raw_data (id text PRIMARY KEY," \
                    + "surface_id integer, "\
                    + "surface_height real, " \
                    + "unrotated_points matrix, " \
                    + "normal matrix, " \
                    + "labels matrix, " \
                    + "image text, "  \
                    + "intensities matrix, "  \
                    + "points matrix)")


def matrix_adapter(some_mat):
    return pk.dumps(some_mat)

def matrix_converter(matrix_string):
    try:
        if len(matrix_string) == 0:
            print "matrix_converter: WARNING -- encountered string of length 0"
            return np.matrix([])
        return pk.loads(matrix_string)
    except EOFError, e:
        import pdb
        pdb.set_trace()
        print 'error'
    except ValueError, e:
        import pdb
        pdb.set_trace()
        print 'error'
        return np.matrix([])


def store_raw_data_from_individual_files(cur, basedir='clutter_db'):
    import clutter_util as cu
    scans = cu.list_files('pkl', basedir)
    print '>> starting'
    for filename in scans:
        store_raw_data_from_file(filename)
    print '>> done.'

def store_raw_data_from_file(filename):
    print '>> loading and storing', filename
    r = ut.load_pickle(filename)
    cur.execute('INSERT INTO raw_data (id, surface_id, surface_height, unrotated_points, normal, labels, image, intensities, points) '\
                + 'values (?, ?, ?, ?, ?, ?, ?, ?, ?)',\
                (r['id'],     r['surface_id'],        r['surface_height'], r['unrotated_points'], 
                 r['normal'], np.matrix(r['labels']), r['image_name'],     r['intensities'],    r['points']))

def db_iterator(name=None, con=None, cur=None):
    started = False
    if name != None:
        con, cur = start(name)
        started = True

    data = cur.execute("SELECT * from raw_data")
    for i, r in enumerate(data):
        #print '>>', i
        d = convert_record_to_dict(r)
        yield d

    if started:
        cur.close()
        con.close()

def convert_record_to_dict(record):
    keys = ["id", "surface_id", "surface_height", "unrotated_points", "normal", "labels", "image", "intensities", "points"]
    a = dict()
    for i, k in enumerate(keys):
        a[k] = record[i]
    return a

def start(name):
    sqlite3.register_adapter(np.matrix, matrix_adapter)
    sqlite3.register_converter('matrix', matrix_converter)
    con = sqlite3.connect(name, detect_types=sqlite3.PARSE_DECLTYPES)
    cur = con.cursor()
    return con, cur

def create_db(name):
    con, cur = start(name)
    create_raw_table(cur)
    store_raw_data_from_individual_files(cur)
    con.commit()
    cur.close()
    con.close()
    print '>> db closed.'

def select(name):
    con, cur = start(name)
    selected = cur.execute('SELECT id, surface_id FROM raw_data')
    id, surface_id = zip(*selected)
    print "print len(set(id))"
    print len(set(id))
    print id
    print "print len(set(surface_id))"
    print len(set(surface_id))
    print surface_id
    
def recreate_table(name):
    con, cur = start(name)
    cur.execute('DROP TABLE bkup')
    create_bkup(cur)
    cur.execute('INSERT INTO bkup SELECT id, surface_id, surface_height, unrotated_points, normal, labels, image, points FROM raw_data')
    cur.execute('DROP TABLE raw_data')
    create_raw_table(cur)
    cur.execute('INSERT INTO raw_data SELECT id, surface_id, surface_height, unrotated_points, normal, labels, image, points FROM bkup')
    cur.execute('DROP TABLE bkup')
    con.commit()
    cur.close()
    con.close()

def test_converter_adapter():
    con, cur = start(':memory:')
    cur.execute("create table test(m matrix)")
    cur.execute("insert into test(m) values (?)", (np.matrix([1,2,3.])))
    cur.execute("select m from test")
    print "with declared types:", cur.fetchone()
    cur.close()
    con.close()


if __name__ == '__main__':
    name = 'clutter_database.sqlite'
    #select(name)
    # create_db(name)
    con, cur = start(name)
    #result = cur.execute('SELECT * FROM raw_data WHERE id="2009Nov09_234352"')
    #cur.execute('DELETE FROM raw_data WHERE id="2009Nov12_182507"')
    #store_raw_data_from_file('clutter_db/hai_2009Nov09_234352.pkl')
    result = cur.execute('SELECT * FROM raw_data')
    for i, r in enumerate(result):
        scan = convert_record_to_dict(r)
        print i, scan['id'], scan['points'].shape, np.matrix(scan['labels']).shape, scan['intensities'].shape, scan['unrotated_points'].shape,
        print scan['normal'].shape,
        pts = np.concatenate((scan['points'], np.matrix(scan['labels']), scan['intensities']), 0)
        print pts.shape

    #con.commit()
    cur.close()
    con.close()














#def create_raw_table_old(cur):
#    cur.execute("CREATE TABLE raw_data (scan_id integer PRIMARY KEY AUTOINCREMENT," \
#                    + "surface_id integer, "\
#                    + "surface_height real, " \
#                    + "unrotated_points matrix, " \
#                    + "normal matrix, " \
#                    + "labels matrix, " \
#                    + "image text, "  \
#                    + "points matrix, " \
#                    + "id text)")
#
#def create_bkup(cur):
#    cur.execute("CREATE TABLE bkup ("\
#                    + "surface_id integer, "\
#                    + "surface_height real, " \
#                    + "unrotated_points matrix, " \
#                    + "normal matrix, " \
#                    + "labels matrix, " \
#                    + "image text, "  \
#                    + "points matrix, " \
#                    + "id text)")
