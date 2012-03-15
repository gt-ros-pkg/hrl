##
# Representation of a dataset, operations that can be performed on it, and quantities that can be calculated from it
import numpy as np

class Dataset:

    ##    
    # inputs coded as numpy array, column vectors
    # outputs also as numpy array, column vectors
    def __init__(self, inputs, outputs):
        self.inputs  = inputs
        self.outputs = outputs
        self.metadata = []
        assert(inputs.shape[1] == outputs.shape[1])

    def num_examples(self):
        return self.inputs.shape[1]

    def num_attributes(self):
        return self.inputs.shape[0]

    def add_attribute_descriptor(self, descriptor):
        self.metadata.append(descriptor)
        #self.metadata[descriptor.name] = descriptor

    def append(self, another_dataset):
        if self.inputs != None:
            self.inputs  = np.concatenate((self.inputs, another_dataset.inputs), axis=1)
        else:
            self.inputs = another_dataset.inputs

        if self.outputs != None:
            self.outputs = np.concatenate((self.outputs, another_dataset.outputs), axis=1)
        else:
            self.outputs = another_dataset.outputs

class AttributeDescriptor:
    def __init__(self, name, extent):
        self.name = name
        self.extent = extent


###############################################################################
# Operations on datasets
###############################################################################

##
# Splits up a dataset based on value in a particular attribute
# @param attribute attribute to split on
# @param split_point value in that attribute to split on
def split_continuous(dataset, attribute, split_point):
    selected_attribute = dataset.inputs[attribute, :]
    leq_bool           = selected_attribute <= split_point
    _, leq_col         = np.where(leq_bool)

    #print 'leq_col', leq_col
    if leq_col.shape[1] > 0:
        leq_dataset        = Dataset(dataset.inputs[:, leq_col.A[0]], dataset.outputs[:, leq_col.A[0]])
    else:
        leq_dataset        = Dataset(np.matrix([]), np.matrix([]))

    _, gt_col          = np.where(~leq_bool)
    if gt_col.shape[1] > 0:
        gt_dataset         = Dataset(dataset.inputs[:, gt_col.A[0]], dataset.outputs[:, gt_col.A[0]])
    else:
        gt_dataset         = Dataset(np.matrix([]), np.matrix([]))
    
    ret_sets = []
    if leq_dataset.num_examples() > 0:
        ret_sets.append(leq_dataset)
    if gt_dataset.num_examples() > 0:
        ret_sets.append(gt_dataset)
    return ret_sets

##
# Makes bootstrap samples
#
# @param dataset Dataset object
# @param number_samples number of bootstrap set to generate
# @param points_per_sample number of points in each sample
# @return an iterator over bootstrap samples
def bootstrap_samples(dataset, number_samples, points_per_sample):
    in_bags, out_bags = [], []
    for i in xrange(number_samples):
        selected_pts     = np.random.randint(0, dataset.inputs.shape[1], points_per_sample)
        n_selected_pts = np.setdiff1d(range(dataset.inputs.shape[1]), selected_pts)
        selected_inputs  = dataset.inputs[:, selected_pts]
        selected_outputs = dataset.outputs[:, selected_pts]
        n_selected_inputs  = dataset.inputs[:, n_selected_pts]
        n_selected_outputs = dataset.outputs[:, n_selected_pts]
        #print 'Dataset.bootstrap count', i
        in_bags.append(Dataset(selected_inputs, selected_outputs))
        out_bags.append(Dataset(n_selected_inputs, n_selected_outputs))
    return in_bags, out_bags


###############################################################################
# Common quantities calculated from datasets
###############################################################################
##
# Returns unique values represented by an attribute
#
# ex.  unique_values(np.matrix([1, 2, 3, 4, 4, 4, 5]), 0)
#      returns [1,2,3,4,5]
#
# @param data nxm matrix where each column is a data vector
# @param attribute_number row to find unique values in
def unique_values(data, attribute_number=0):
    values   = dict()
    for instance_idx in xrange(data.shape[1]):
        values[data[attribute_number, instance_idx]] = True
    k = values.keys()
    k.sort()
    return k

##
# Caculates the discrete entropy of a data vector
#
# @param data 1xn matrix of discrete values
def entropy_discrete(data):
    values = unique_values(data)
    #for each output class calculate
    def calc_class_entropy(value):
        number_in_class = np.sum(data == value)
        num_examples = data.shape[1]
        percentage_in_class = (number_in_class / float(num_examples))
        return -percentage_in_class * np.log2(percentage_in_class)
    return np.sum(map(calc_class_entropy, values))

##
# Calculates entropy in a dataset's output labels
#
# @param dataset
def dataset_entropy_discrete(dataset):
    return entropy_discrete(dataset.outputs[0,:])






