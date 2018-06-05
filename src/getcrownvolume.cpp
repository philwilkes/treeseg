//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "treeseg.h"

float maxheight(float dbh)
{
	//m    -> 41.22 * dbh ^ 0.3406
	//ci_u -> 42.30 * dbh ^ 0.3697
	float height = 42.30 * pow(dbh,0.3697) + 5;
	return height;
}

float maxcrown(float dbh)
{
	//m    -> 29.40 * dbh ^ 0.6524
	//ci_u -> 30.36 * dbh ^ 0.6931
	float extent = 30.36 * pow(dbh,0.6931) + 5; 
	return extent;
}

int main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

    for(int i=6;i<argc;i++)
    {   
        std::cout << "---------------" << std::endl;
        std::cout << "processing:" << argv[i] << std::endl;
        // check if file is process already; I will check if the slice (first intermediate file) or final tree (intermed files might be deleted already) are in the folder
        //  
        std::string fname;
        std::vector<std::string> name1;
        std::vector<std::string> name2;
        std::vector<std::string> name3;
        boost::split(name1,argv[i],boost::is_any_of("."));
        boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
        boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
        fname = name3[name3.size()-1];
        //std::string pname = argv[2];
        std::stringstream ss; 
        //ss << pname << "_" << fname << ".pcd";
        std::string ll; 
        ll += fname;
        ll += ".tracker";
        //std::cout << argv[i] << " " << ss.str() << std::endl;
        if ( !boost::filesystem::exists(ss.str()) )
        {   
            if ( !boost::filesystem::exists(ll) )
            {   
                std::ofstream o(ll.c_str()); //creates empty file; just here so loop can check if this tree is being processed already

                // read in stem
        		std::string fname;
                std::vector<std::string> name1,name2,name3;
                boost::split(name1,argv[i],boost::is_any_of("/"));
                boost::split(name2,name1[name1.size()-1],boost::is_any_of("."));
                boost::split(name3,name2[0],boost::is_any_of("_"));
                fname = name3[1];
        
        		pcl::PointCloud<pcl::PointXYZI>::Ptr stem(new pcl::PointCloud<pcl::PointXYZI>);
        		reader.read(argv[i],*stem);
        		//
        		std::cout << "Estimating DBH: " << std::flush;
        		int nnearest = 90;
        		float zstep = 0.75;
        		float diffmax = 0.1;
        		float dbh = getDBH(stem,nnearest,zstep,diffmax);
        		std::cout << dbh << std::endl;
        		//
        		std::cout << "Crown dimensions: " << std::flush;
        		float h = maxheight(dbh);
        		float c = maxcrown(dbh);
        		std::cout << h << "m x " << c << "m (HxW)" << std::endl;

                // read in tiles
                Eigen::Vector4f min,max,centroid;
                pcl::compute3DCentroid(*stem,centroid);
                std::ifstream infile(argv[3]);
                int tile;
                float x,y;
                std::list<int> tiles_list;
                while (infile >> tile >> x >> y)
                {
                    float expansion = c;
                    float xmin = centroid[0]-expansion;
                    float xmax = centroid[0]+expansion;
                    float ymin = centroid[1]-expansion;
                    float ymax= centroid[1]+expansion;
                    if(x >= xmin && x <= xmax && y >= ymin && y <= ymax)
                    {
                        tiles_list.push_back(tile);
                    }
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr plot(new pcl::PointCloud<pcl::PointXYZI>);       
                std::list<int>::iterator it;
                for (it = tiles_list.begin(); it != tiles_list.end(); ++it)
                {
                    std::stringstream ss;
                    ss.str("");
                    ss << argv[1] << argv[2] << "_" << *it << ".downsample.pcd";
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
                    reader.read(ss.str(),*cloud_tmp);
                    *plot += *cloud_tmp;
                }

        		std::cout << "Segmenting volume: " << std::flush;
        		pcl::PointCloud<pcl::PointXYZI>::Ptr xslice(new pcl::PointCloud<pcl::PointXYZI>);
        		pcl::PointCloud<pcl::PointXYZI>::Ptr yslice(new pcl::PointCloud<pcl::PointXYZI>);
        		pcl::PointCloud<pcl::PointXYZI>::Ptr zslice(new pcl::PointCloud<pcl::PointXYZI>);
        		pcl::PointCloud<pcl::PointXYZI>::Ptr volume(new pcl::PointCloud<pcl::PointXYZI>);
        		pcl::getMinMax3D(*stem,min,max);
        		pcl::compute3DCentroid(*stem,centroid);
        		spatial1DFilter(plot,"x",centroid[0]-c/2,centroid[0]+c/2,xslice);
        		spatial1DFilter(xslice,"y",centroid[1]-c/2,centroid[1]+c/2,yslice);
        		spatial1DFilter(yslice,"z",max[2],min[2]+h,zslice);
        		*volume += *stem;
        		*volume += *zslice;
        		std::stringstream ss;
        		ss << "volume_" << fname << ".pcd";
        		writer.write(ss.str(),*volume,true);	
        		std::cout << ss.str() << std::endl;
            }
        }
	}
	return 0;
}
