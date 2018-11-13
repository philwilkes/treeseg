import sys
import scipy.io
import pandas as pd
import numpy as np

from cyl2ply import pandas2ply

class QSM:

    def __init__(self, path2mat):
        
        self.mat = scipy.io.loadmat(path2mat)
        if 'qsm' not in self.mat.keys() and 'QSM' not in self.mat.keys():
            self.version = 2
            self.qsm_2()
        else:
            self.version = 2.3
            self.qsm_2_3()
            
    def qsm_2(self):
        
        # cylinder data
        self.cyl_radius = self.mat['Rad']
        self.cyl_length = self.mat['Len']
        self.cyl_start = self.mat['Sta']
        self.cyl_axis = self.mat['Axe']
        self.cyl_branch = self.mat['BoC'][:, 0].reshape(-1, 1)
        self.cyl_BranchOrder = self.mat['BoC'][:, 1].reshape(-1, 1)
        self.cyl_PositionInBranch = self.mat['BoC'][:, 2].reshape(-1, 1)
        self.cyl_parent = self.mat['CPar']
        self.cyl_added = self.mat['Added']
        self.cyl_extension = self.mat['CExt']
        self.cyl_fields = ('radius', 'length', 'start', 'axis', 'parent', 'extension', 'added', 'BranchOrder')
        
        # branch data
        self.branch_order = self.mat['BOrd']
        self.branch_parent = self.mat['BPar']
        self.branch_volume = self.mat['BVol']
        self.branch_length = self.mat['BLen']
        self.branch_angle = self.mat['BAng']
        self.branch_fields = ('order', 'parent', 'volume', 'length', 'angle')
        
        # tree data
        self.TotalVolume = self.mat['TreeData'][0]    # Total volume of the tree
        self.TrunkVolume = self.mat['TreeData'][1]    # Volume of the trunk
        self.BranchVolume = self.mat['TreeData'][2]   # Total volume of all the branches
        self.TreeHeight = self.mat['TreeData'][3]     # Total height of the tree
        self.TrunkLength = self.mat['TreeData'][4]    # Length of the trunk
        self.BranchLength = self.mat['TreeData'][5]   # Total length of all the branches
        self.NumberBranches = self.mat['TreeData'][6] # Total number of branches
        self.MaxBranchOrder = self.mat['TreeData'][7] # Maximum branch order
        self.TotalArea = self.mat['TreeData'][8]      # Total area of cylinders
        self.DBHcyl = self.mat['TreeData'][9]         # Diameter at breast height (cylinder)
        self.DBHqsm = self.mat['TreeData'][10]        # DBH from triangulation
        self.treedata_fields = ('TotalVolume', 'TrunkVolume', 'BranchVolume', 'TreeHeight', 
                                'TrunkLength', 'BranchLength', 'NumberBranches', 'MaxBranchOrder', 
                                'TotalArea', 'DBHqsm', 'DBHcyl')
        
    def qsm_2_3(self):
        
        if 'qsm' not in self.mat.keys(): qsm = 'QSM' # Andy's version capitalises the key
        else: qsm = 'qsm'
        
        # rundata
        self.rundata_fields = self.mat[qsm]['rundata'][0][0][0][0][0].dtype.names
        for var in self.rundata_fields:
            setattr(self, var, self.mat[qsm]['rundata'][0][0][0][0][0][var][0][0][0][0])  
        
        # cyl
        self.cyl_fields = self.mat[qsm]['cylinder'][0][0][0].dtype.names
        for var in self.cyl_fields:
            setattr(self, 'cyl_' + var, self.mat[qsm]['cylinder'][0][0][0][var][0])              

        # branch
        self.branch_fields = self.mat[qsm]['branch'][0][0][0].dtype.names
        for var in self.branch_fields:
            setattr(self, 'branch_' + var, self.mat[qsm]['branch'][0][0][0][var][0])            

        # treedata
        self.treedata_fields = self.mat[qsm]['treedata'][0][0][0].dtype.names
        for var in self.treedata_fields:
            setattr(self, var, self.mat[qsm]['treedata'][0][0][0][var][0][0][0])
            
        # pmdistance
        if self.Dist == 1:
            self.pmdistance_fields = self.mat[qsm]['pmdistance'][0][0].dtype.names
            for var in self.pmdistance_fields:
                setattr(self, 'pmd_' + var, self.mat[qsm]['pmdistance'][0][0][var][0][0])
            
        # triangulation
        if self.Tria == 1:
            self.triagualtion_fields = self.mat[qsm]['triangulation'][0][0].dtype.names
            for var in self.triagualtion_fields:
                setattr(self, 'tri_' + var, self.mat[qsm]['triangulation'][0][0][var][0][0])       
                    
        # optimal models
        if 'models' in self.mat.keys():
            """
            optimise model and metadata
            """
            # optimal model numbers
            self.optimal_models = [m[0] for m in self.mat['models'][0][0]]
            
            # optimal treedata
            for var in self.treedata_fields:
                setattr(self, 'opt_' + var, self.mat['treedata'][0][0][var][0])
                
            # optimal rundata
            for var in self.rundata_fields:
                setattr(self, 'opt_' + var, self.mat['inputs'][0][0][var][0][0])

    def cyl2pd(self):
        
        if self.version == 2.3:

            columns = ['radius', 'length', 'sx', 'sy', 'sz', 'ax', 'ay', 'az', 'parent', 'extension', 
                       'added', 'UnmodRadius', 'branch', 'BranchOrder', 'PositionInBranch']

            arr = np.hstack([self.cyl_radius, self.cyl_length, self.cyl_start, self.cyl_axis, 
                             self.cyl_parent, self.cyl_extension, self.cyl_added, self.cyl_UnmodRadius, 
                             self.cyl_branch, self.cyl_BranchOrder, self.cyl_PositionInBranch])
            
        else:
            
            columns = ['radius', 'length', 'sx', 'sy', 'sz', 'ax', 'ay', 'az', 'parent', 'extension', 
                       'added', 'branch', 'BranchOrder', 'PositionInBranch']
            
            arr = np.hstack([self.cyl_radius, self.cyl_length, self.cyl_start, self.cyl_axis, 
                             self.cyl_parent, self.cyl_extension, self.cyl_added, 
                             self.cyl_branch, self.cyl_BranchOrder, self.cyl_PositionInBranch])

        return pd.DataFrame(data=arr, columns=columns)
        
    def branch2pd(self):
        
        if self.version == 2.3:

            columns = ['BOrd', 'BPar', 'BVol', 'BLen', 'BAng', 'BHei', 'BAzi', 'BDia']

            arr = np.hstack([self.branch_order, self.branch_parent, self.branch_volume,  
                             self.branch_length, self.branch_angle, self.branch_height, 
                             self.branch_azimuth, self.branch_diameter])
        
        else:
            
            columns = ['BOrd', 'BPar', 'BVol', 'BLen', 'BAng']

            arr = np.hstack([self.branch_order, self.branch_parent, self.branch_volume,  
                             self.branch_length, self.branch_angle])
            
            
        return pd.DataFrame(data=arr, columns=columns, index=np.arange(1, len(arr) + 1)) 
            
            

if __name__ == "__main__":
   
    for path2mat in sys.argv[1:]:
       
        qsm = QSM(path2mat)
        print '{}: {}'.format(path2mat, qsm.TotalVolume)
