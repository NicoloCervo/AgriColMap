from os import listdir
from os.path import isfile, join, dirname
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
import collections

"""
plots the alignment results using the data stored in params/output/results by registration node
"""

def plot_rates(points, threshold):
    by_size = {}
    for measure in points:
        if measure[0] not in by_size.keys():
            by_size[measure[0]]=[measure[1]]
        else:
            by_size[measure[0]].append(measure[1])

    success_rates = {}
    for key in by_size.keys():
        count=0
        for transl in by_size[key]:
            if transl < threshold:
                count+=1
        success_rates[key]= count/10

    ordered = collections.OrderedDict(sorted(success_rates.items()))

    sizes=[]
    t_rates=[]

    for k, v in ordered.items(): 
        sizes.append(k)
        t_rates.append(v)


    return sizes, t_rates


current_dir = dirname(__file__)
path = join(current_dir, '../params/output/results/')

result_dict={}
transl_points=[]
rot_points=[]
files = [f for f in listdir(path) if isfile(join(path, f))]
results=['file_name    tran_err_x  tran_err_y  rot_err  scale_err\n']

#read and process data
for file in files:
    with open(path+file, 'r') as f:
        line = f.readlines()
    data=[]
    for e in line[0].split(' '):
        data.append(float(e))
    r = R.from_matrix([ 
    [data[0], data[1], data[2]],
    [data[4], data[5], data[6]],
    [data[8], data[9], data[10]]
    ])

    #print(r.as_euler('zyx', degrees=True))

    tran_err_x=data[3]+data[16]-data[19]
    tran_err_y=data[7]+data[17]-data[20]
    rot_err=data[18]-r.as_euler('zyx', degrees=True)[0]
    cloud_size=int(file[15:21].split('x')[0])
    if cloud_size not in result_dict:
        result_dict[cloud_size]=[]
    result_dict[cloud_size].append([tran_err_x, tran_err_y, rot_err])
    transl_points.append( (float(cloud_size), (tran_err_x**2+tran_err_y**2)**(1/2)) )
    rot_points.append( (float(cloud_size), rot_err) )

    results.append('\n'+file +' '+str(tran_err_x)+' '+str(tran_err_y)+' '+str(rot_err))

od = collections.OrderedDict(sorted(result_dict.items()))
for k, v in od.items(): 
    avg=[0,0,0]
    count=0
    for run in v:
        count+=1
        for i in range(3):
            avg[i]= avg[i]+run[i]
    avg = [s/count for s in avg]
    od[k]=avg

#for k, v in od.items(): print([k,v])

#write to file
file1 = open('/home/n/Desktop/Test_results', 'w')
file1.writelines(results)
file1.close()


sizes,t_rates = plot_rates(transl_points, 1)  # 1m
sizes,r_rates = plot_rates(rot_points, 1)     # 1°

plt.plot(sizes,t_rates,"b--o")
plt.plot(sizes,r_rates,"r--P")

plt.show()




