from os import listdir
from os.path import isfile, join, dirname
import subprocess
import time

current_dir = dirname(__file__)
path = join(current_dir, '../params/output/results/')
successful={}
files = [f for f in listdir(path) if isfile(join(path, f))]

for f in files:
    size = f.split('_')[2].split('x')[0]
    n = f.split('_')[3]
    if size not in successful.keys():
        successful[size]=[n]
    else:
        successful[size].append(n)

current_dir = dirname(__file__)
params_dir = join(current_dir, '../params/test_params')
print(current_dir)

scale_err = '10'
tr_err = '10000'
yaw_err = '50'

for k in successful.keys():
    rerun=[]
    for i in range(10):
        if str(i) not in successful[k]:

                f =  'pd30_subregion_'+str(k)+'x'+str(k)+'_'+str(i)+'.yaml'
                print(f)
                start = time.time()

                result = subprocess.run([current_dir+'/./registration_node',
                params_dir+'/' + f, scale_err, tr_err, yaw_err, '2'], stderr=subprocess.PIPE)

                print(time.time()-start)
            







