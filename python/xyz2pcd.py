#!/usr/bin/env python

import sys
import os
import pandas as pd

import pcd2df

for txt in sys.argv[1:]:

    if ',' in open(txt).readline():
        sep = ','
    else: sep = ' '
        
    df = pd.read_csv(txt, sep=sep, header=0)
    pcd2df.write_pcd(df[df.columns[:3]], txt.replace('.txt', '.pcd'), binary=True)
