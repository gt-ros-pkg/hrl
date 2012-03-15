import rospy
import hrl_lib
import yaml
import os
node_name = "overhead_grasping"

ARM = 0 # right arm

PRESSURE_LIST =["r_finger_periph_pressure", 
                "r_finger_pad_pressure", 
                "l_finger_periph_pressure",
                "l_finger_pad_pressure"]

class FileOperations():
    def __init__(self):
        grep = os.popen("rospack find pr2_overhead_grasping|grep pr2_overhead_grasping")
        self.package_loc = grep.readlines()[0].rstrip()
    
    def load_pickle(self, fn):
        return hrl_lib.util.load_pickle(self.package_loc + "//pickles//" + fn)

    def save_pickle(self, p, fn):
        hrl_lib.util.save_pickle(p, self.package_loc + "//pickles//" + fn)

    def get_pickle_name(self, fn):
        return self.package_loc + "//pickles//" + fn

    def file_exists(self, fn):
        return os.path.exists(self.package_loc + "//pickles//" + fn)
    
    def get_plot_name(self, directory, gfn):
        filename = (self.package_loc + "//plots//" + directory + "//" +
                    gfn.split(".")[0].split("//")[-1] + ".pdf")
        return filename

    def load_coll_times(self, loc):
        ct_index_fn = (self.package_loc + "//pickles//" + 
                       loc + "//collision_times.yaml")
        stream = file(ct_index_fn, "r")
        coll_times = yaml.load(stream)
        return coll_times

    def save_coll_times(self, coll_times, loc):
        ct_index_fn = (self.package_loc + "//pickles//" + 
                       loc + "//collision_times.yaml")
        stream = file(ct_index_fn, "w")
        yaml.dump(coll_times, stream)

    def load_yaml_file(self, fn):
        ct_index_fn = (self.package_loc + "//yaml//" + fn)
        stream = file(ct_index_fn, "r")
        coll_times = yaml.load(stream)
        return coll_times

    def make_directories(self, directory):
        try:
            os.mkdir(self.package_loc + "//plots")
        except:
            pass
        try:
            os.mkdir(self.package_loc + "//plots//" + directory)
        except:
            pass
        try:
            os.mkdir(self.package_loc + "//pickles")
        except:
            pass
        try:
            os.mkdir(self.package_loc + "//pickles//collision_data")
        except:
            pass
        try:
            os.mkdir(self.package_loc + "//pickles//collision_data//" + directory)
        except:
            pass
        try:
            os.mkdir(self.package_loc + "//arff_files")
        except:
            pass
        try:
            os.mkdir(self.package_loc + "//pickles//classifiers")
        except:
            pass
 

def log(*strs):
    prstrs = ""
    for s in strs:
        prstrs += str(s) + " "
    rospy.loginfo(node_name + ": " + prstrs)

def err(*strs):
    prstrs = ""
    for s in strs:
        prstrs += str(s) + " "
    rospy.logerr(node_name + ": " + prstrs)

def wait_for_key(ki, key='c'):
    ch = None
    while not rospy.is_shutdown() and ch != key:
        ch = ki.getch()
        rospy.sleep(0.1)
    log("CONTINUING")
