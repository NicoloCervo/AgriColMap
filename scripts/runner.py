import subprocess
import time
from os import listdir
from os.path import isfile, join, dirname

"""
script to run registration node on all the pointclouds 
that have parameter file in params/test_params 
"""

current_dir = dirname(__file__)
params_dir = join(current_dir, '../params/test_params')
files = listdir(params_dir)
print(current_dir)
#print(files)
scale_err = '10'
tr_err = '20000'
yaw_err = '200'
for yaw_err in ['50']:
    for f in files:

        print(f)
        start = time.time()

        result = subprocess.run([current_dir+'/../bin/./registration_node',
        params_dir+'/' + f, scale_err, tr_err, yaw_err, '2'], stderr=subprocess.PIPE)

        print(time.time()-start)
