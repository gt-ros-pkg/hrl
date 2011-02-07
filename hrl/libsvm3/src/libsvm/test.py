import svm


labels = [0, 1]
samples = [[0, 0], [0, 1]]

labels = [0, 1, 1, 2]
samples = [[0, 0], [0, 1], [1, 0], [1, 1]]

import svm

labels = [0, 0, 1, 1]
samples = [[1, 1], [1, -1], [-1, 1], [-1, -1]]

param = svm.svm_parameter('-c 1')
problem = svm.svm_problem(labels, samples)

model = svm.libsvm.svm_train(problem, param)
pmodel = svm.toPyModel(model)
pmodel.predict_values(samples[0])
for i in range(len(samples)):
    print svm.libsvm.svm_predict(model, svm.gen_svm_nodearray(samples[i])[0])


r = (c_double*6)()
svm.libsvm.svm_predict_values(model, svm.gen_svm_nodearray(samples[0])[0], r)


