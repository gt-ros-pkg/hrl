
import commands
import sys, os
import glob

l1 = glob.glob('data_1tb/*/mechanism_info.pkl')
l2 = []
for d in l1:
    l2.append('/'.join(['aggregated_pkls_Feb11']+d.split('/')[1:]))

for d1,d2 in zip(l1,l2):
    os.system('cp %s %s'%(d1, d2))

