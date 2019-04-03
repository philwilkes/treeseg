import os, sys
import glob
import pandas as pd
import numpy as np
import struct
import glob

import pcd_io

class downsample2hiRes:

    def __init__(self, tree,
                 tile_index='../../extraction/tile_index.dat',
                 tile_path='../../extraction/rxp2pcd_i/',
                 outpath='.',
                 verbose=True):

        self.tile_index = pd.read_csv(tile_index, names=['tile', 'x', 'y'], sep=' ')
        self.tile_path = tile_path
        self.outpath = outpath
        self.verbose = verbose
        self.extract(tree)
        
    def voxel(self, tmp):

        binarize = lambda x: struct.pack('i', int(int(x * 100) / 5))

        tmp.loc[:, 'xb'] = tmp.x.apply(binarize)
        tmp.loc[:, 'yb'] = tmp.y.apply(binarize)
        tmp.loc[:, 'zb'] = tmp.z.apply(binarize)
        tmp.loc[:, 'B'] = tmp.xb + tmp.yb + tmp.zb

        return tmp

    def extract(self, name):

        if self.verbose: print 'processing tree:', name

        # read tree pcd
        tmp = self.voxel(pcd_io.read_pcd(name))

        # select tiles
        T = self.tile_index[(self.tile_index.x.between(tmp.x.min() - 10, tmp.x.max() + 10)) &
                            (self.tile_index.y.between(tmp.y.min() - 10, tmp.y.max() + 10))].tile.values

        tiles = glob.glob(os.path.join(self.tile_path + '*.pcd'))
        suffix = os.path.split(tiles[0])[1].split('_')[0]

        hiRes_pts = 0
        hiRes_plots = []

        for i, t in enumerate(T):

            tile_name = os.path.join(self.tile_path, '{}_{}.pcd'.format(suffix, t))

            if self.verbose: print '    processing tile:', tile_name
            tile = pcd_io.read_pcd(tile_name)
            #print tmp[['x', 'y', 'z']].min(), tile[['x', 'y', 'z']].min(), tmp[['x', 'y', 'z']].max(), tile[['x', 'y', 'z']].max()
            
            tile_ = tile.loc[(tile.x.between(tmp.x.min() - .1, tmp.x.max() + .1)) & 
                             (tile.y.between(tmp.y.min() - .1, tmp.y.max() + .1)) &
                             (tile.z.between(tmp.z.min() - .1, tmp.z.max() + .1))] #&
#                             (tile.intensity.isin(tmp.intensity))].copy() # intensity == scan position

            if len(tile_) == 0: continue
            tile_ = self.voxel(tile_)   
            tile_ = tile_[tile_.B.isin(np.unique(tmp.B))]
            if len(tile_) == 0: continue
            tile_[['x', 'y', 'z', 'intensity']].values.astype('f4').tofile(os.path.join(self.outpath, name.replace('.pcd', '.{}.tmp'.format(t))))
            hiRes_pts += len(tile_)
            hiRes_plots.append(t)
            
        if self.verbose: print 'total number of points:', hiRes_pts
        if self.verbose: print 'from number of tiles:', len(hiRes_plots)
        output_name = os.path.join(self.outpath, name.replace('.pcd', '.hiRes.ply'))
        if self.verbose: print "    writing to file:", output_name
        with open(output_name, 'w') as ply:
            ply.write("ply\n")
            ply.write('format binary_little_endian 1.0\n')
            ply.write("comment Author: Phil Wilkes\n")
            ply.write("obj_info Generated using Python\n")
            ply.write("element vertex {}\n".format(hiRes_pts))
            ply.write("property float x\n")
            ply.write("property float y\n")
            ply.write("property float z\n")
            ply.write("property float intensity\n")
            ply.write("end_header\n")

            for p in hiRes_plots:
                tmp = os.path.join(self.outpath, name.replace('.pcd', '.{}.tmp'.format(p)))
                if self.verbose: 'adding points from:', tmp
                with open(tmp) as tmpf:
                      ply.write(tmpf.read())
                os.unlink(tmp)
        
        if self.verbose: print '    finished:', name
            
if __name__ == "__main__":
    
    for tree in sys.argv[1:]:
        
        downsample2hiRes(tree)
    
