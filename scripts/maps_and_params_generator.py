import numpy as np
from os.path import dirname, join
from os import mkdir

class params_generator():

    def __init__(self):
        current_dir = dirname(__file__)
        path = join(current_dir, '../original_pointclouds/')
        with open(path+'pd30.txt', 'r') as f:
            self.lines = f.readlines()
        

    def process_pointcloud(self, min_x, min_y, max_x, max_y, file_path):

        color_noise=True
        space_noise=True
        crop = True
        norm = True
        transform = False

        theta=0
        translation=[0,0]
        color_variance=255*0.01 #1%
        space_variance=0.20 #20cm

        if transform:

            theta=0  #np.pi/32
            scale=0.02

            transl_mat=np.identity(4)
            transl_mat[3,:2] = translation
            rot_z_mat = [
                [ np.cos(theta),    np.sin(theta),  0.,  0.], 
                [-np.sin(theta),    np.cos(theta),  0.,  0.],
                [0.,                0.,             1.,  0.],
                [0.,                0.,             0.,  1.]]
            scal_mat = np.identity(4)*scale
            scal_mat[3,3] = 1

            T_mat = np.matmul(np.matmul(scal_mat, rot_z_mat), transl_mat)

        sums = [0., 0., 0.]

        points = []
        
        print('parsing')
        for i in range (len(self.lines)):
            point = []
            for coord in self.lines[i].split(' '):
                point.append(float(coord))

            if crop and point[0]>min_x and point[0]<max_x and point[1]>min_y and point[1]<max_y:
                points.append(point)
                if norm:
                    sums = [sums[0]+point[0], sums[1]+point[1], sums[2]+point[2]]
            
            if not crop:
                points.append(point)
                if norm:
                    sums = [sums[0]+point[0], sums[1]+point[1], sums[2]+point[2]]

            point[3] = int(point[3]/256) 
            point[4] = int(point[4]/256)
            point[5] = int(point[5]/256)

        avgs = [sums[0]/len(points), sums[1]/len(points), sums[2]/len(points)]

        offset = [avgs[0]-translation[0], avgs[1]-translation[1], avgs[2]]+[0,0,theta*180/np.pi]
        offset = [round(o,2) for o in offset]
        offset_string = str(offset[0])+" "+str(offset[1])+" "+str(offset[2])+" "+str(offset[3])+" "+str(offset[4])+" "+str(offset[5])
        print(offset_string)
        
        new_lines = []
        print('transf/noise')
        for p in points:

            p[:3] = [p[i]- avgs[i] for i in range(3)]

            if transform:
                p_temp = np.append(p[:3], [1])

                p[:3] = np.matmul(p_temp, T_mat)[:3]

            if space_noise:
                p[:3] = p[:3] + (space_variance*np.random.randn(3))

            if color_noise:
                noise_color = p[3:] + (color_variance*np.random.randn(3)).astype(int)
                np.clip(noise_color, 0, 255, noise_color)
                p[3:] = noise_color

            new_lines.append(str(round(p[0],2))+' '+str(round(p[1],2))+' '+str(round(p[2],2))
                                +' '+str(p[3])+' '+str(p[4])+' '+str(p[5])+'\n')

        header = ('ply\nformat ascii 1.0\ncomment author: pd\ncomment object: padova colored pointcloud\n'
            'element vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\n'
            'property uchar green\nproperty uchar blue\nend_header\n').format(len(new_lines))

        new_lines.insert(0,header)

        file1 = open(file_path + '/subregion30.ply', 'w')
        file1.writelines(new_lines)
        file1.close()

        file1 = open(file_path+'/offset.xyz', 'w')
        file1.writelines(str(offset_string))
        file1.close()

        print('DONE')

def main():

    current_dir = dirname(__file__)#/bin
    
    full_width= 725570 - 723969
    full_height= 5032679 - 5031478
    generator = params_generator()
  
    sizes = [15,25,35,45,55,65,10,20,30,40,50,60,70,80,90,100,120,140,160,180]
    croppings_per_size = 10

    for region_size in sizes:
        for i in range(0, croppings_per_size):

            """ x=np.random.randint(0, full_width-region_size)
            y=np.random.randint(0, full_height-region_size)"""

            x= (full_width-region_size)/2
            y= (full_height-region_size)/2

            min_x=x+723969
            min_y=y+5031478
            max_x=x+region_size+723969
            max_y=y+region_size+5031478

            dir_name='pd30_subregion_'+str(region_size)+'x'+str(region_size)+'_'+str(i)
            
            file_path=join(current_dir, '../maps/')+dir_name

            try:
                mkdir(file_path)
            except OSError as error:
                print(error) 

            #generator.process_pointcloud(min_x, min_y, max_x, max_y, file_path)

            yaml = ('## Input Parameters: \n'
            'input_clouds: \n'
            ' \n'
            '  # Path, Name, and Type of the Clouds to Align \n'
            '  cloud_fixed_path:                   padova_r \n'
            '  cloud_fixed_name:                   region30 \n'
            '  cloud_moving_path:                  {} \n'
            '  cloud_moving_name:                  subregion30 \n'
            '  relative_scale:                     [1, 1]  \n'
            ' \n'
            '## Other Parameters \n'
            'aligner_params: \n'
            ' \n'
            '  # Dense Opt. Flow, and Final Ref. Params \n'
            '  max_iter_number:                    3 \n'
            '  verbosity:                          true \n'
            '  store_dense_optical_flow:           false\n'
            '  show_dpf_correspondences:           false \n'
            '  dense_optical_flow_step:            3 \n'
            '  downsampling_rate:                  0.15 \n'
            '  search_radius:                      0.15 \n'
            '  use_visual_features:                true \n'
            '  visual_features_weight:             1 \n'
            '  use_geometric_features:             true \n'
            '  geometric_features_weight:          0.5 \n').format(dir_name)



            file1 = open(join(current_dir, '../params/test_params/')+ dir_name+'.yaml', 'w')
            file1.writelines(str(yaml))
            file1.close()

if __name__ == "__main__":
    main()