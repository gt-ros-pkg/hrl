# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
import numpy as np
import itertools as it
import functools as ft
import time

class Dataset:
    def __init__(self, inputs, outputs):
        """
            inputs coded as numpy array, column vectors
            outputs also as numpy array, column vectors
        """
        self.inputs  = inputs
        self.outputs = outputs
        assert(inputs.shape[1] == outputs.shape[1])

    def num_examples(self):
        return self.inputs.shape[1]

    def num_attributes(self):
        return self.inputs.shape[0]

    def split_continuous(self, attribute, split_point):
        selected_attribute = self.inputs[attribute, :]
        leq_bool           = selected_attribute <= split_point
        _, leq_col         = np.where(leq_bool)

        #print 'leq_col', leq_col
        if leq_col.shape[1] > 0:
            leq_dataset        = Dataset(self.inputs[:, leq_col.A[0]], self.outputs[:, leq_col.A[0]])
        else:
            leq_dataset        = Dataset(np.matrix([]), np.matrix([]))

        _, gt_col          = np.where(~leq_bool)
        if gt_col.shape[1] > 0:
            gt_dataset         = Dataset(self.inputs[:, gt_col.A[0]], self.outputs[:, gt_col.A[0]])
        else:
            gt_dataset         = Dataset(np.matrix([]), np.matrix([]))
        
        ret_sets = []
        if leq_dataset.num_examples() > 0:
            ret_sets.append(leq_dataset)
        if gt_dataset.num_examples() > 0:
            ret_sets.append(gt_dataset)
        return ret_sets

    def unique_values(self, attribute_number, set='input'):
        if set == 'input':
            examples = self.inputs
        else:
            examples = self.outputs

        values   = dict()
        for instance_idx in xrange(examples.shape[1]):
            values[examples[attribute_number, instance_idx]] = True
        k = values.keys()
        k.sort()
        return k

    def bootstrap_samples(self, number_samples, points_per_sample):
        for i in xrange(number_samples):
            selected_pts     = np.random.randint(0, self.inputs.shape[1], points_per_sample)
            selected_inputs  = self.inputs[:, selected_pts]
            selected_outputs = self.outputs[:, selected_pts]
            #print 'Dataset.bootstrap count', i
            yield Dataset(selected_inputs, selected_outputs)

    def entropy_discrete(self):
        values = self.unique_values(0, 'output')
        #print 'entropy_discrete: values', values
        #for each output class calculate
        def calc_class_entropy(value):
            number_in_class     = np.sum(self.outputs[0,:] == value)
            percentage_in_class = (number_in_class / float(self.num_examples()))
            return -percentage_in_class * np.log2(percentage_in_class)
        return np.sum(map(calc_class_entropy, values))

    def append(self, another_dataset):
        self.inputs  = np.concatenate((self.inputs, another_dataset.inputs), axis=1)
        self.outputs = np.concatenate((self.outputs, another_dataset.outputs), axis=1)

class LinearDimReduceDataset(Dataset):
    def __init__(self, inputs, outputs):
        Dataset.__init__(self, inputs, outputs)

    def set_projection_vectors(self, vec):
        '''
            projection vectors are assumed to be columnwise
        '''
        self.projection_basis = vec
        print 'LinearDimReduceDataset: projection_basis', vec.shape

    def reduce(self, data_points):
        return self.projection_basis.T * data_points

    def reduce_input(self):
        '''
           reduce dimensionality of this dataset
        '''
        self.inputs =  self.projection_basis.T * self.inputs

def binary_less_than(attribute, threshold, input_vec):
    return input_vec[attribute,0] <= threshold

def binary_greater_than(attribute, threshold, input_vec):
    return input_vec[attribute,0] > threshold

def create_binary_tests(attribute, threshold):
    return [ft.partial(binary_less_than, attribute, threshold), 
            ft.partial(binary_greater_than, attribute, threshold)]

def mode_exhaustive(set):
    '''
       Finds the mode of a given set
    '''

    #Count, store in dictionary
    mdict = dict()
    for s in set:
        has_stored = False
        for k in mdict.keys():
            if k == s:
                mdict[k] = 1+mdict[k]
                has_stored = True
        if not has_stored:
            mdict[s] = 1

    #Find the key with maximum votes
    max_key   = None
    max_count = -1
    for k in mdict.keys():
        if mdict[k] > max_count:
            max_key   = k
            max_count = mdict[k]
    #print 'mode_exhaustive: ', mdict
    return max_key, mdict

def min_entropy_split(dataset):
    '''
        Find the split that produces subsets with the minimum combined entropy
        return splitting attribute & splitting point for that attribute
    '''
    #print 'in min_entropy_split'
    # Assume inputs are continuous, and are column vectors.
    hypotheses     = []
    entropies      = []
    # For each attribute find the best split point.
    for attribute in xrange(dataset.num_attributes()):
        values = dataset.unique_values(attribute)
        #Iterate over the possible values of split & calculate entropy for each split.
        for split_point in values:
            def calc_entropy(data_set):
                num_points = data_set.num_examples()
                return (num_points / float(dataset.num_examples())) * data_set.entropy_discrete()
            split_entropy = map(calc_entropy, dataset.split_continuous(attribute, split_point))
            hypotheses.append((attribute, split_point))
            entropies.append(sum(split_entropy))
    # Select the attribute split pair that has the lowest entropy.
    entropies                              = np.matrix(entropies)
    min_idx                                = np.argmin(entropies)
    return hypotheses[min_idx]

def random_subset(subset_size, total_size):
    #print 'in random_subset'
    assert(subset_size <= total_size)
    occupancy = np.matrix(np.zeros((1, total_size)))
    while occupancy.sum() < subset_size:
        occupancy[0, np.random.randint(0, total_size)] = 1
    rows, columns = np.where(occupancy > 0)
    return columns.A[0]

def split_random_subset(subset_size, total_size):
    assert(subset_size <= total_size)
    occupancy = np.matrix(np.zeros((1, total_size)))
    while occupancy.sum() < subset_size:
        occupancy[0, np.random.randint(0, total_size)] = 1
    bool_sel                = occupancy > 0
    rows, columns_subset    = np.where(bool_sel)
    rows, columns_remaining = np.where(np.invert(bool_sel))
    return columns_subset.A[0], columns_remaining.A[0]

def random_subset_split(num_subset, dataset):
    ''' splitter in decision tree '''
    #print 'in random_subset_split'
    #print 'num_subset', num_subset, dataset, 'dataset.input.shape', dataset.inputs.shape
    subset_indexes   = random_subset(num_subset, dataset.num_attributes())
    sub_dataset      = Dataset(dataset.inputs[subset_indexes,:], dataset.outputs)
    attribute, point = min_entropy_split(sub_dataset)
    return subset_indexes[attribute], point

def totally_random_split(dataset):
    #print 'totally random'
    attr     = np.random.randint(0, dataset.num_attributes())
    split_pt = dataset.inputs[attr, np.random.randint(0, dataset.num_examples())]
    return attr, split_pt

class DecisionTree:
    def __init__(self, dataset=None, splitting_func=min_entropy_split):
        self.children   = None
        self.prediction = None
        if dataset is not None:
            self.train(dataset, splitting_func=splitting_func)

    def train(self, dataset, splitting_func=min_entropy_split):
        if not self.make_leaf(dataset):
            #print 'in train.splitting', dataset.num_examples()
            self.split_attribute, self.split_point = splitting_func(dataset)
            #print 'self.split_attribute, self.split_point', self.split_attribute, self.split_point 
            data_sets                              = dataset.split_continuous(self.split_attribute, self.split_point)
            if len(data_sets) < 2:
                self.prediction = dataset.outputs
                return
            
            def tree_split(set):
                #print 'tree', set.num_examples()
                return DecisionTree(set, splitting_func=splitting_func)
            # Create & train child decision nodes
            tests            = create_binary_tests(self.split_attribute, self.split_point)
            self.children    = zip(tests, map(tree_split, data_sets))

    def make_leaf(self, dataset):
        if np.all(dataset.outputs[:,0] == dataset.outputs):
            self.prediction = dataset.outputs[:,0]
            #print 'leaf'
            return True
        elif np.all(dataset.inputs[:,0] == dataset.inputs):
            self.prediction = dataset.outputs
            #print 'leaf'
            return True
        else:
            return False

    def predict(self, input):
        if self.prediction is not None:
            return self.prediction[:, np.random.randint(0, self.prediction.shape[1])]
        else:
            for test, child in self.children:
                if test(input):
                    return child.predict(input)
            raise RuntimeError("DecisionTree: splits not exhaustive, unable to split for input" + str(input.T))

class RFBase:
    def __init__(self, dataset=None, number_of_dimensions=None, number_of_learners=100):
        """ 
            number_of_dimensions   unclear which direction, but should be around 10-20% of original 
                                   data dimension
            number_of_learners        limited by processor performance, higher is better
        """
        self.number_of_learners   = number_of_learners
        self.number_of_dimensions = number_of_dimensions
        if dataset != None:
            self.train(dataset)

    def predict(self, data, vote_combine_function=None):
        def predict_(learner):
            return learner.predict(learner.transform_input(data))
        predictions = map(predict_,self.learners)
        if vote_combine_function is not None:
            return vote_combine_function(predictions)
        else:
            return mode_exhaustive(predictions)

    def train(self, dataset):
        pass

def identity(x):
    return x

class RFBreiman(RFBase):
    def train(self, dataset):
        def train_trees(examples_subset):
            tree = DecisionTree()
            #tree.train(examples_subset, splitting_func=ft.partial(random_subset_split, self.number_of_dimensions))
            tree.train(examples_subset, splitting_func=totally_random_split)
            #use identity function
            tree.transform_input = identity 
            return tree

        if self.number_of_dimensions == None:
            self.number_of_dimensions = min(np.log2(dataset.num_attributes()) + 1, 1)
        points_per_sample = dataset.num_examples() * 1.0 / 3.0
        self.learners     = map(train_trees, dataset.bootstrap_samples(self.number_of_learners, points_per_sample))

class RFRandomInputSubset(RFBase):
    def train(self, dataset):
        def train_trees(examples_subset):
            #select a subset of dimensions
            dims               = random_subset(self.number_of_dimensions, examples_subset.num_attributes())
            subset_input       = examples_subset.inputs[dims, :]
            reduced_sample     = Dataset(subset_input, examples_subset.outputs)
            tree               = DecisionTree(reduced_sample)
            tree.dimensions_subset = dims
            def transform_input(input):
                return input[dims, :]
            tree.transform_input = transform_input
            return tree

        if self.number_of_dimensions == None:
            self.number_of_dimensions = min(np.log2(dataset.num_attributes()) + 1, 1)
        points_per_sample = dataset.num_examples() * 1.0 / 3.0
        self.learners     = map(train_trees, dataset.bootstrap_samples(self.number_of_learners, points_per_sample))

def evaluate_classifier(building_func, data, times=10.0, percentage=None, extra_args={}, test_pca=False):
    '''
        Evaluate classifier by dividing dataset into training and test set.
        @param building_func Function that will build classifier given data and args in extra_args.
        @param data Dataset to use for evaluation/training.
        @param times The number of bootstrap samples to take.
        @param percentage The percentage of data to use for training.
        @param extra_args Extra arguments to pass to building_func.
    '''
    print 'evaluate_classifier: extra_args', extra_args
    total_pts            = data.num_examples()
    testing_errors       = []
    training_errors      = []
    build_times          = []
    classification_times = []
    for i in range(times):
        if percentage == None:
            percentage = (i+1)/times
        num_examples = int(round(total_pts*percentage))
        print 'Evaluate classifier built with', percentage*100, '% data, num examples', num_examples
        subset, unselected = split_random_subset(num_examples, total_pts)
        i                  = data.inputs[:,  subset]
        o                  = data.outputs[:, subset]
        print "Building classifier..."
        if test_pca:
            print '            TESTING PCA'
            import dimreduce as dr
            subseted_dataset   = LinearDimReduceDataset(i,o)
            subseted_dataset.set_projection_vectors(dr.pca_vectors(subseted_dataset.inputs, percent_variance=.95))
            subseted_dataset.reduce_input()
            print 'subseted_dataset.num_attributes(), subseted_dataset.num_examples()', subseted_dataset.num_attributes(), subseted_dataset.num_examples()
        else:
            subseted_dataset   = Dataset(i,o)

        start_time         = time.time()
        classifier         = building_func(subseted_dataset, **extra_args)
        build_times.append(time.time() - start_time)
        print "done building..."

        ##########################################
        #Classify training set
        ##########################################
        count_selected     = []
        for i, idx in enumerate(subset):
            start_time    = time.time()
            if test_pca:
                prediction, _ = classifier.predict(data.reduce(data.inputs[:,idx]))
            else:
                prediction, _ = classifier.predict(data.inputs[:,idx])

            classification_times.append(time.time() - start_time)
            true_val      = data.outputs[:,idx]
            if prediction == true_val:
                count_selected.append(1)
            else:
                count_selected.append(0)
            if i%100 == 0:
                print i
        count_selected = np.matrix(count_selected)

        ##########################################
        #Classify testing set
        ##########################################
        confusion_matrix   = dict()
        count_unselected   = []
        print 'Total points', total_pts
        for idx in unselected:
            start_time    = time.time()
            if test_pca:
                prediction, _ = classifier.predict(data.reduce(data.inputs[:,idx]))
            else:
                prediction, _ = classifier.predict(data.inputs[:,idx])
            classification_times.append(time.time() - start_time)
            true_val      = data.outputs[:,idx]
            if prediction == true_val:
                count_unselected.append(1)
            else:
                count_unselected.append(0)
            if confusion_matrix.has_key(true_val[0,0]):
                if confusion_matrix[true_val[0,0]].has_key(prediction[0,0]):
                    confusion_matrix[true_val[0,0]][prediction[0,0]] = confusion_matrix[true_val[0,0]][prediction[0,0]] + 1
                else:
                    confusion_matrix[true_val[0,0]][prediction[0,0]] = 1
            else:
                confusion_matrix[true_val[0,0]] = dict()
                confusion_matrix[true_val[0,0]][prediction[0,0]] = 1

        training_error = 100.0 * np.sum(count_selected) / float(len(subset))
        testing_error  = 100.0 * np.sum(count_unselected) / float(len(unselected))
        testing_errors.append(testing_error)
        training_errors.append(training_error)
        print 'Correct on training set', training_error, '%'
        print '        on testing set',  testing_error, '%'
        print 'Confusion'
        for k in confusion_matrix.keys():
            sum = 0.0
            for k2 in confusion_matrix[k]:
                sum = sum + confusion_matrix[k][k2]
            for k2 in confusion_matrix[k]:
                print 'true class', k, 'classified as', k2, 100.0 * (confusion_matrix[k][k2] / sum), '% of the time'

    def print_stats(name, list_data):
        m = np.matrix(list_data)
        print '%s: average %f std %f' % (name, m.mean(), np.std(m))

    print_stats('training error', training_errors)
    print_stats('testing error', testing_errors)
    print_stats('build time', build_times)
    print_stats('classification time', classification_times)


if __name__ == '__main__':
    test_iris         = False
    test_pickle       = True
    test_number_trees = False
    test_pca          = False
    if test_iris:
        #Setup for repeated testing
        iris_array = np.matrix(np.loadtxt('iris.data', dtype='|S30', delimiter=','))
        inputs     = np.float32(iris_array[:, 0:4]).T
        outputs    = iris_array[:, 4].T
        dataset    = Dataset(inputs, outputs)

        print '================================'
        print "Test DecisionTree"
        evaluate_classifier(DecisionTree, dataset, 5, .9)

        print '================================'
        #print "Test random forest"
        #for i in range(4):
        #    #print "Test RFRandomInputSubset"
        #    #evaluate_classifier(RFRandomInputSubset, dataset, 1, .7)
        #    print "Test RFBreiman"
        #    evaluate_classifier(RFEntropySplitRandomInputSubset, dataset, 1, .7)
    if test_pickle:
        import pickle as pk
        def load_pickle(filename):
            p = open(filename, 'r')
            picklelicious = pk.load(p)
            p.close()
            return picklelicious

        def print_separator(times=2):
            for i in xrange(times):
                print '==============================================================='

        dataset = load_pickle('PatchClassifier.dataset.pickle')
        #if test_pca:
        #    print_separator(1)
        #    print_separator(1)
        #    dataset.reduce_input()

        if test_number_trees:
            tree_types = [RFBreiman, RFRandomInputSubset]
            #tree_types = [RFBreiman]
            for tree_type in tree_types:
                print_separator()
                print 'Testing', tree_type
                for i in range(10):
                    print tree_type, 'using', (i+1)*10, 'trees'
                    evaluate_classifier(tree_type, dataset, 3, .95, 
                            extra_args={'number_of_learners': (i+1)*10}, test_pca=test_pca)
        else:
            tree_types = [RFBreiman, RFRandomInputSubset]
            #tree_types = [RFRandomInputSubset]
            for tree_type in tree_types:
                print_separator()
                print tree_type
                evaluate_classifier(tree_type, dataset, 10, .95, 
                        extra_args={'number_of_learners': 70}, test_pca=test_pca)






