import sys
import argparse
import numpy as np
import pandas as pd

from pcd_io import read_pcd
from ply_io import write_ply

def apply_rotation(M, df):

    if 'a' not in df.columns:
        df.loc[:, 'a'] = 1
    r_ = np.dot(M, df[['x', 'y', 'z', 'a']].T).T
    df.loc[:, ['x', 'y', 'z']] = r_[:, :3]
    return df

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--pcd', nargs='*', help='pcd files')
    parser.add_argument('-r', '--rotation', action='store_true', help='pcd files')
    parser.add_argument('--combine', nargs=1, default=False, help='name of .ply with all point clouds merged')
    parser.add_argument('--colour', nargs=1, default=False, help='add colour to point cloud')
    args = parser.parse_args()

    pc = pd.DataFrame()

    for pcd in args.pcd:

        df = read_pcd(pcd).astype(float)
        df = df.round({v:3 for v in ['x', 'y', 'z', 'intensity'] if v in df.columns})

        if 'red' in df.columns:
            pass
        elif 'rgb' in df.columns:
            cmap = pd.DataFrame.from_dict({c:np.random.randint(0, high=255, size=3) for c in df.rgb.unique()},
                              orient='index')
            cmap.reset_index(inplace=True)
            cmap.columns = ['rgb', 'red', 'green', 'blue']
            df = pd.merge(df, cmap, on='rgb')
        elif args.colour:
            df['red'] = np.random.randint(0, 255)
            df['green'] = np.random.randint(0, 255)
            df['blue'] = np.random.randint(0, 255)
        else:
            pass

        if args.rotation:
            M = np.loadtxt(args.rotation)
            pc = apply_rotation(M, pc)

        if not args.combine:
            write_ply(pcd.replace('.pcd', '.ply'), df)
        else:
            pc = pc.append(df)

    if args.combine:
        write_ply(args.combine[0], pc)

