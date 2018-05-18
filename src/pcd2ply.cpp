#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::PLYWriter writer;
  writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary, use_camera);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{

    for(int i=1;i<argc;i++)
    {
        // Command line parsing
        bool format = true;
        bool use_camera = false;
        parse_argument (argc, argv, "-format", format);
        parse_argument (argc, argv, "-use_camera", use_camera);
        print_info ("PLY output format: "); print_value ("%s, ", (format ? "binary" : "ascii"));
        print_value ("%s\n", (use_camera ? "using camera" : "no camera"));
        
        // Load the first file
        pcl::PCLPointCloud2 cloud;
        loadCloud (argv[i], cloud); 
        
        // Convert to PLY and save
        boost::filesystem::path oname(argv[i]);
        std::stringstream ss;
        ss << oname.stem().string() << ".ply"; 

        saveCloud (ss.str(), cloud, format, use_camera);
    }
    return (0);
}
