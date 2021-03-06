//Andrew Burt - a.burt.12@ucl.ac.uk

#include <string>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <cmath>

#include <dirent.h>

#include <riegl/scanlib.hpp>

struct pcloud 
{
	std::vector<float> x,y,z;
	std::vector<float> range;
	std::vector<float> amplitude;
	std::vector<float> reflectance;
	std::vector<float> deviation;
	std::vector<float> return_number;
	float scan_number;
	std::vector<double> time;
	float matrix[16];
};

class importer : public scanlib::pointcloud
{	
	pcloud &pc;
	public:
		importer(pcloud &pc) : scanlib::pointcloud(false), pc(pc){}
	protected:
		void on_shot_end()
		{
			for(int i=0;i<targets.size();i++)
			{
				pc.x.push_back(targets[i].vertex[0]);
				pc.y.push_back(targets[i].vertex[1]);
				pc.z.push_back(targets[i].vertex[2]);
				pc.range.push_back(targets[i].echo_range);
				pc.amplitude.push_back(targets[i].amplitude);
				pc.reflectance.push_back(targets[i].reflectance);
				pc.deviation.push_back(targets[i].deviation);
				pc.return_number.push_back(i+1);
				pc.time.push_back(targets[i].time);
			}	
		}
};

int main(int argc,char** argv)
{
	std::string top_dir = argv[1];
	if(top_dir[top_dir.length()-1] != '/') top_dir = top_dir + "/";

    // create output file     

	std::string fname = argv[3];
	std::stringstream ss;
	std::ofstream xyzfile;
	std::string xyzname;
	std::string pcdname;

	ss.str("");
	ss << fname <<  "_mon.xyz";
	xyzfile.open(ss.str(),std::ios::binary);
	xyzname = ss.str();
	ss.str("");
	ss << fname << "_mon.pcd";
	pcdname = ss.str();

    int count = 0;
    float min_refl = atof(argv[2]);

	std::vector<std::string> positions;
	DIR *tdir = NULL;
	tdir = opendir (top_dir.c_str());
	struct dirent *tent = NULL;
	while(tent = readdir(tdir)) positions.push_back(tent->d_name);
	closedir(tdir);
	for(int k=0;k<positions.size();k++)
	{
		if(positions[k][0] == 'S' && positions[k][4] == 'P')
		{
			ss.str("");
			ss << top_dir << positions[k];
			std::string position;
			const char* c_position;
			position = ss.str();
			c_position = position.c_str();
			std::vector<std::string> position_contents;
			DIR *pdir = NULL;
			pdir = opendir(c_position);
			struct dirent *pent = NULL;
			while(pent = readdir(pdir)) position_contents.push_back(pent->d_name);
			closedir(pdir);
			std::string rxpname;
			for(int l=0;l<position_contents.size();l++)
			{
				if(position_contents[l][14] == 'm' && position_contents[l][15] == 'o' && position_contents[l][16] == 'n' && position_contents[l].length() == 21 )
				{
					ss.str("");
					ss << top_dir << positions[k] << "/" << position_contents[l];
					rxpname = ss.str();
				}
			}
			ss.str("");
			ss << top_dir << "matrix/" << positions[k][7] << positions[k][8] << positions[k][9] <<".dat";
			std::string matrixname = ss.str();
			std::cout << rxpname << " " << " " << matrixname << std::endl;
///////
			pcloud pc;
			try
			{

				std::shared_ptr<scanlib::basic_rconnection> rc;
				rc = scanlib::basic_rconnection::create(rxpname);
				rc->open();
				scanlib::decoder_rxpmarker dec(rc);
				importer imp(pc);
				scanlib::buffer buf;
				for(dec.get(buf);!dec.eoi();dec.get(buf))
				{
					imp.dispatch(buf.begin(), buf.end());
				}
				rc->close();
			}
			catch (...)
			{
				continue;
			}
///////
			ss.str("");
			if(positions[k][7] == '0' && positions[k][8] == '0') ss << positions[k][9];
			else if(positions[k][7] == '0') ss << positions[k][8] << positions[k][9];
			else ss << positions[k][7] << positions[k][8] << positions[k][9];
			std::string scan_number = ss.str();
			pc.scan_number = atof(scan_number.c_str());
			std::ifstream mfile;
			mfile.open(matrixname);
			int no_count = 0;
			if(mfile.is_open())
			{
				while(!mfile.eof())
				{
					mfile >> pc.matrix[no_count];
					no_count++;
				}
			}
			for(int m=0;m<pc.x.size();m++)
			{
				float X = ((pc.x[m]*pc.matrix[0])+(pc.y[m]*pc.matrix[1])+(pc.z[m]*pc.matrix[2]))+pc.matrix[3];
				float Y = ((pc.x[m]*pc.matrix[4])+(pc.y[m]*pc.matrix[5])+(pc.z[m]*pc.matrix[6]))+pc.matrix[7];
				float Z = ((pc.x[m]*pc.matrix[8])+(pc.y[m]*pc.matrix[9])+(pc.z[m]*pc.matrix[10]))+pc.matrix[11];
                float D = pc.reflectance[m];

 				if(pc.reflectance[m] > min_refl)
                {
                    xyzfile.write(reinterpret_cast<const char*>(&X),sizeof(X));
 				    xyzfile.write(reinterpret_cast<const char*>(&Y),sizeof(Y));
 				    xyzfile.write(reinterpret_cast<const char*>(&Z),sizeof(Z));
                    xyzfile.write(reinterpret_cast<const char*>(&D),sizeof(D));
 				    count += 1;
 				
                }
			}
		}
	}

	xyzfile.close();

   	ss.str("");
	std::ofstream headerstream("header.tmp");
	headerstream << "VERSION 0.7" << std::endl << "FIELDS x y z intensity" << std::endl << "SIZE 4 4 4 4" << std::endl << "TYPE F F F F" << std::endl << "COUNT 1 1 1 1" << std::endl << "WIDTH " << count << std::endl << "HEIGHT 1" << std::endl << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl << "POINTS " << count << std::endl << "DATA binary" << std::endl;
   	headerstream.close();
   	ss << "cat header.tmp " << xyzname << " > " << pcdname << "; rm header.tmp " << xyzname;
  	const char* cc;
    std::string string;
  	string = ss.str();
  	cc = string.c_str();
  	system(cc);
	return 0;
}
