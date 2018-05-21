#!/usr/bin/env python

import numpy as np
import pandas as pd

def read_pcd(fp):

    with open(fp) as pcd:
        
        length = 0
        
        for i, line in enumerate(pcd.readlines()):
            length += len(line)
            if 'WIDTH' in line: N = int(line.split()[1])
            if 'FIELDS' in line: F = line.split()[1:]
            if 'DATA' in line: 
                fmt = line.split()[1]
                break
                
        if fmt == 'binary':
            pcd.seek(length)
            arr = np.fromfile(pcd, dtype='f')
            
            arr = arr[:N*4].reshape(-1, 4)
            df = pd.DataFrame(arr, columns=F)

    if fmt == 'ascii':
        df = pd.read_csv(fp, sep=' ', names=F, skiprows=11)
        
    return df

def write_pcd(df, path, binary=True):
    
    with open(path, 'wb') as pcd:
        
        pcd.write('# .PCD v0.7 - Point Cloud Data file format\n')
        pcd.write('VERSION 0.7\n')
        pcd.write('FIELDS ' + ' '.join(df.columns) + '\n')
        pcd.write('SIZE ' + '4 ' * len(df.columns) + '\n')
        pcd.write('TYPE ' + 'F ' * len(df.columns) + '\n')
        pcd.write('COUNT ' + '1 ' * len(df.columns) + '\n')
        pcd.write('WIDTH {}\n'.format(len(df)))
        pcd.write('HEIGHT 1\n')
        pcd.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        pcd.write('POINTS {}\n'.format(len(df)))
        if binary:
            pcd.write('DATA binary\n')
            df.as_matrix().tofile(pcd)
        else:
            pcd.write('DATA ascii\n')
            df.to_csv(pcd, sep=' ', index=False, header=False)
