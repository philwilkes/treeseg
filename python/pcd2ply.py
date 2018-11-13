#!/usr/bin/env python

import sys
import argparse
import numpy as np
import pandas as pd

from pcd2df import read_pcd

def apply_rotation(M, df):
    
    if 'a' not in df.columns:
        df.loc[:, 'a'] = 1
    r_ = np.dot(M, df[['x', 'y', 'z', 'a']].T).T
    df.loc[:, ['x', 'y', 'z']] = r_[:, :3]
    return df

def write_ply(output_name, pc):

    pc = pc[['x', 'y', 'z', 'r', 'g', 'b']]

    with open(output_name, 'w') as ply:

        ply.write("ply\n")
        ply.write('format ascii 1.0\n')
        ply.write("comment Author: Phil Wilkes\n")
        ply.write("obj_info Generated using Python\n")
        ply.write("element vertex {}\n".format(len(pc)))
        ply.write("property float x\n")
        ply.write("property float y\n")
        ply.write("property float z\n")
        ply.write("property int red\n")
        ply.write("property int green\n")
        ply.write("property int blue\n")
        ply.write("end_header\n")
        pc.to_csv(ply, sep=' ', index=False, header=False)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--pcd', nargs='*', help='pcd files')
    parser.add_argument('-r', '--rotation', default='store_false', help='pcd files')
    parser.add_argument('-o', '--filename', help='output name')
    args = parser.parse_args()

    pc = pd.DataFrame()
    
    for pcd in args.pcd:

        df = read_pcd(pcd).astype(float)
        df = df.round({'x':3, 'y':3, 'z':3})
        df['r'] = np.random.randint(0, 255)
        df['g'] = np.random.randint(0, 255)
        df['b'] = np.random.randint(0, 255)
        pc = pc.append(df)
        
    if args.rotation:

        M = np.loadtxt(args.rotation)
        pc = apply_rotation(M, pc)
        
    write_ply(args.filename, pc)