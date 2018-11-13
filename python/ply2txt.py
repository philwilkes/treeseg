import numpy as np
import sys

for pc in sys.argv[1:]:

    with open(pc) as ply:

        length = 0

        for i, line in enumerate(ply.readlines()):
            length += len(line)
            if 'element vertex' in line: N = int(line.split()[2])
            if 'end_header' in line: break

        ply.seek(length)
        arr = np.fromfile(ply, dtype='f')

        arr = arr[:N*3].reshape(-1, 3)

        try:
            assert len(arr) == N
            np.savetxt(pc.replace('.ply', '.txt'), arr, fmt='%.3f')
        
        except AssertionError:
            print 'length of point cloud does not equal that specified in the header'