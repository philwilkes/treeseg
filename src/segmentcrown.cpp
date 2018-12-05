//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <armadillo>

#include "treeseg.h"
#include "leafsep.h"

int main (int argc, char* argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	for(int i=2;i<argc;i++)
	{
		std::vector<std::string> id = getFileID(argv[i]);
        std::string ll; 
        ll += id[0];
        ll += ".tracker";
        if ( !boost::filesystem::exists(ll) )
        {   
            std::ofstream o(ll.c_str()); //creates empty file; just here so loop can check if this tree is being processed already
     
		    std::cout << "----------: " << argv[i] << std::endl;
		    std::cout << "Reading volume cloud: " << std::flush;
		    pcl::PointCloud<pcl::PointXYZI>::Ptr volume(new pcl::PointCloud<pcl::PointXYZI>);
		    reader.read(argv[i],*volume);
		    std::cout << "complete" << std::endl;
		    //
		    std::cout << "Euclidean clustering: " << std::flush;
		    std::vector<std::vector<float>> nndata = dNNz(volume,50,2);
		    float nnmax = 0;
		    for(int i=0;i<nndata.size();i++) if(nndata[i][1] > nnmax) nnmax = nndata[i][1];
		    std::cout << nnmax << ", " << std::flush;
		    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
		    euclideanClustering(volume,nnmax,3,clusters);
		    ss.str("");
		    ss << "ec_" << id[0] << ".pcd";
		    writeClouds(clusters,ss.str(),false);
		    std::cout << ss.str() << std::endl;
		    //	
		    std::cout << "Region-based segmentation: " << std::flush;
		    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> regions;
		    int idx = findPrincipalCloudIdx(clusters);
		    int nnearest = 50;
		    int nmin = 3;
		    float smoothness = atof(argv[1]);
		    regionSegmentation(clusters[idx],nnearest,nmin,smoothness,regions);
		    ss.str("");
		    ss << "ec_rg_" << id[0] << ".pcd";
		    writeClouds(regions,ss.str(),false);
		    std::cout << ss.str() << std::endl;
		    //
		    std::cout << "Leaf stripping: " << std::endl;
		    //
		    std::cout << " Region-wise, " << std::flush;
	            arma::mat rfmat;
	            arma::gmm_full rmodel;
	            gmmByCluster(regions,5,1,5,50,100,rfmat,rmodel);
	            std::vector<int> rclassifications;
	            rclassifications = classifyGmmClusterModel(regions,5,rfmat,rmodel);
	            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> csepclouds;
	            separateCloudsClassifiedByCluster(regions,rclassifications,csepclouds);
		    ss.str("");
		    ss << "ec_rg_rlw_" << id[0] << ".pcd";
		    writeCloudClassifiedByCluster(regions,rclassifications,ss.str());
		    std::cout << ss.str() << std::endl;
		    //
		    std::cout << " Point-wise, " << std::flush;
		    arma::mat pfmat;
		    arma::gmm_diag pmodel;
		    gmmByPoint(csepclouds[1],50,5,1,5,50,100,pfmat,pmodel);
		    std::vector<int> pclassifications;
		    pclassifications = classifyGmmPointModel(csepclouds[1],5,pfmat,pmodel);
		    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> psepclouds;
		    separateCloudsClassifiedByPoint(csepclouds[1],pclassifications,psepclouds);
		    ss.str("");
		    ss << "ec_rg_rlw_plw_" << id[0] << ".pcd";
		    writeCloudClassifiedByPoint(csepclouds[1],pclassifications,ss.str());
		    std::cout << ss.str() << std::endl;
		    //
		    ss.str("");
		    ss << "ec_rg_rlw_plw_w_" << id[0] << ".pcd";
		    pcl::PointCloud<pcl::PointXYZI>::Ptr wood(new pcl::PointCloud<pcl::PointXYZI>);
		    *wood += *csepclouds[0] + *psepclouds[0];
		    writer.write(ss.str(),*wood,true);
		    std::cout << ss.str() << std::endl;
		    //
		    std::cout << "Re-segmenting regions: " << std::flush;
		    regions.clear();
		    regionSegmentation(wood,nnearest,nmin,smoothness+2.5,regions);
		    ss.str("");
		    ss << "ec_rg_rlw_plw_w_rg" << id[0] << ".pcd";
		    writeClouds(regions,ss.str(),false);
		    std::cout << ss.str() << std::endl;
		    //
		    std::cout << "Optimising regions: " << std::flush;
		    removeFarRegions(regions);
		    ss.str("");
		    ss << "ec_rg_rlw_plw_w_rg_o" << id[0] << ".pcd";
		    writeClouds(regions,ss.str(),false);
		    std::cout << ss.str() << std::endl;
		    //
		    std::cout << "Building tree: " << std::flush;
		    pcl::PointCloud<pcl::PointXYZI>::Ptr tree(new pcl::PointCloud<pcl::PointXYZI>);
		    buildTree(regions,tree);
		    ss.str("");
		    ss << "tree_" << id[0] << ".pcd";
		    writer.write(ss.str(),*tree,true);
		    std::cout << ss.str() << std::endl;
        }
	}
	return 0;
}
