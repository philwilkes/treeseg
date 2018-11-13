import sys
import numpy as np

from cyl2ply import pandas2ply
from mat2qsm import QSM

for mat in sys.argv[1:]:
    
    qsm = QSM(mat)
    
    pandas2ply(qsm.cyl2pd()[['length', 'radius', 'sx', 'sy', 'sz', 'ax', 'ay', 'az', 'branch']], 
               'branch', 
               mat.replace('.mat', '.ply')) 
    
    if qsm.Tria == 1:
        
        threes = np.ones((len(qsm.tri_facet), 1)) + 2
        facets = np.hstack([threes, qsm.tri_facet])
        
        with open(mat.replace('.mat', '_tri.ply'), 'w') as ply:
            
            ply.write("ply\n")
            ply.write("format ascii 1.0\n")
            ply.write("comment Author: Phil Wilkes\n")
            ply.write("obj_info Generated using Python\n")
            ply.write("element vertex {}\n".format(len(qsm.tri_vert)))
            ply.write("property float x\n")
            ply.write("property float y\n")
            ply.write("property float z\n")
            ply.write("element face {}\n".format(len(facets)))
            ply.write("property list uchar int vertex_indices\n")
            ply.write("end_header\n")
            
            np.savetxt(ply, qsm.tri_vert, fmt='%.3f')
            np.savetxt(ply, facets, fmt='%.i')