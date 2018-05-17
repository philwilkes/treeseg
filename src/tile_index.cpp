//Kim Calders: kim.calders@npl.co.uk
//modified by Phil Wilkes

#include <boost/algorithm/string.hpp>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <treeseg.h>

int main (int argc, char** argv)
{
    float area = atof(argv[1]);
    float half_length = sqrt(area)/2;
    for(int i=2;i<argc;i++)
    {
        boost::filesystem::path oname(argv[i]);
        std::vector<std::string> fname1, fname2;
        boost::split(fname1, oname.stem().string(), boost::is_any_of("."));
        boost::split(fname2, fname1.at(0), boost::is_any_of("_"));
        std::string name = fname2.at(1);

        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plot(new pcl::PointCloud<pcl::PointXYZ>);
        reader.read(argv[i],*plot);

        Eigen::Vector4f min,max,centroid;
        pcl::getMinMax3D(*plot,min,max);
        std::cout << name << " " << min[0]+half_length <<" " << min[1]+half_length << std::endl;
    }
    return 0;
}
