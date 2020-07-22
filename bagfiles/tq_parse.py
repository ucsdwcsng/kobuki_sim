import pandas as pd
import re
import sys

def extract_data(s):
    '''
    Extract data from 5-line-string like this
    At time 1561680141.360
    - Translation: [-0.004, -0.000, 0.010]
    - Rotation: in Quaternion [0.000, 0.000, 0.002, 1.000]
            in RPY (radian) [0.000, -0.000, 0.003]
            in RPY (degree) [0.000, -0.000, 0.200]
    param: s 
    type: list
    return: list --[time, 
                   x, y, z
                   w, i, j, k,
                   roll_radian, pitch_radian, yaw_radian,
                   roll_degree, pitch_degree, yaw_degree]
    '''
    assert isinstance(s, list)      # make sure input is a list
    data = []
    # loop thru every element of the list
    for item in s:
        # extract all numbers from that line
        data_list_str = re.findall(r"[-+]?\d*\.\d+|\d+", item) 
        # convert str numbers to float
        data_list_float = [float(i) for i in data_list_str]    
        data += data_list_float
        
    return data

def location_txt_to_csv(file_name):
    '''Convert tf pose txt file to csv files
    Each line of csv file contains [t,x,y,z,
                                   w,i,j,k,
                                   roll_radian,p_radian,y_radian,
                                   roll_degree,p_dregree,y_degree]
    Arguments:
        file_name {str} -- path to the txt file

    '''
    assert isinstance(file_name,str)
    
    # read the txt file line by line
    with open(file_name) as f:
        content = f.readlines()
    
    # find index of "header" in content
    header_indexes = [i for i, s in enumerate(content) if 'header:' in s]
    
    # parse data at each message segment
    trajectory = []
    for i in range(len(header_indexes)):
        start = header_indexes[i]
        end = start + 16
        segment = content[start:end]

        # parse all data in segment
        trajectory.append(extract_data(segment))
    
    # export trajacotry to csv file
    pd.DataFrame(trajectory).to_csv(file_name[:-4]+".csv",index=False, header=False)

if __name__ == '__main__':
    '''Usage 
    python txt2csv.py /PATH/TO/TXT1 /PATH/TO/TXT2 ...
    '''
    paths = sys.argv[1:]
    for path in paths:    
        location_txt_to_csv(path)
        print("successfully convert {} to csv".format(path))
