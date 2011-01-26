import numpy as np

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
