//Andrew Burt - a.burt@ucl.ac.uk

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <treeseg.h>

int main (int argc, char *argv[])
{
	float resolution = atof(argv[1]);
	float zmin = atof(argv[2]);
	float zmax = atof(argv[3]);
	
    pcl::PointCloud<pcl::PointXYZI>::Ptr plotcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	std::vector<std::string> id = getFileID(argv[4]);
	std::stringstream ss;
    ss << id[1] << ".dem";
    std::ofstream outfile(ss.str().c_str());

    for(int i=4;i<argc;i++)
    {
        std::cout << "adding " << argv[i] << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr slice(new pcl::PointCloud<pcl::PointXYZI>);
        reader.read(argv[i],*cloud_tmp);
        std::vector<std::vector<float>> dem;
        dem = getDemAndSlice(cloud_tmp, resolution, zmin, zmax, slice);
        *plotcloud += *slice;
        for(int j=0;j<dem.size();j++) outfile << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
//        outfile << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
    }

    outfile.close();

	ss.str("");
	ss << id[1] << ".slice.downsample.pcd";
	writer.write(ss.str(), *plotcloud, true);

	return 0;
}
