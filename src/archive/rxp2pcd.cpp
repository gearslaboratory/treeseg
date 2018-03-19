//Andrew Burt - a.burt.12@ucl.ac.uk

#include <string>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>

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
	std::ifstream cfile;
	cfile.open(argv[2]);
	float coordfile[4];
	int no_count = 0;
	if(cfile.is_open())
	{
		while(!cfile.eof())
		{
			cfile >> coordfile[no_count];
			no_count++;
		}
	}
	cfile.close();
	float x_min = coordfile[0]-25;
	float x_max = coordfile[1]+25;
	float y_min = coordfile[2]-25;
	float y_max = coordfile[3]+25;
	std::cout << x_min << " " << x_max << " " << y_min << " " << y_max << std::endl;
	int steps = atoi(argv[3])+1; //+1 is sample pcd for nearestneighbour etc
	float deviation_max = atof(argv[4]);
	std::string fname = argv[5];
	float coords[steps][4];
	int count[steps];
	float delta_x = (x_max - x_min) / float(steps);
	std::cout << delta_x << std::endl;
	for(int i=0;i<steps;i++)
	{
		if(i == steps-1)
		{
			float x_mid = (x_max+x_min)/2;
			float y_mid = (y_max+y_min)/2;
			coords[i][0] = x_mid - 15;
			coords[i][1] = x_mid + 15;
			coords[i][2] = y_mid - 15;
			coords[i][3] = y_mid + 15;
			std::cout << "sample " << coords[i][0] << " " << coords[i][1] << " " << coords[i][2] << " " << coords[i][3] << std::endl;
		}
		else
		{
			coords[i][0] = x_min + (i * delta_x);
			coords[i][1] = x_min + ((i + 1) * delta_x);
			coords[i][2] = y_min;
			coords[i][3] = y_max;
			std::cout << i << " " << coords[i][0] << " " << coords[i][1] << " " << coords[i][2] << " " << coords[i][3] << std::endl;
		}
		//initialise count array
		count[i] = 0;
	}
	std::stringstream ss;
	std::ofstream xyzfiles[steps];
	std::string xyznames[steps];
	std::string pcdnames[steps];
	for(int j=0;j<steps;j++)
	{
		if(j == steps-1)
		{
			ss.str("");
			ss << fname << ".sample.xyz";
			xyzfiles[j].open(ss.str(),std::ios::binary);
			xyznames[j] = ss.str();
			ss.str("");
			ss << fname << ".sample.pcd";
			pcdnames[j] = ss.str();

		}
		else
		{
			ss.str("");
			ss << fname << "_" << j << ".xyz";
			xyzfiles[j].open(ss.str(),std::ios::binary);
			xyznames[j] = ss.str();
			ss.str("");
			ss << fname << "_" << j << ".pcd";
			pcdnames[j] = ss.str();
		}
	}
	std::vector<std::string> positions;
	DIR *tdir = NULL;
	tdir = opendir (top_dir.c_str());
	struct dirent *tent = NULL;
	while(tent = readdir(tdir)) positions.push_back(tent->d_name);
	closedir(tdir);
	for(int k=0;k<positions.size();k++)
	{
		if(positions[k][0] == 'T') // This means: look for folders that start with T: i.e. the ones that have the rxp data
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
				if(position_contents[l][14] == 'r' && position_contents[l][15] == 'x' && position_contents[l][16] == 'p' && position_contents[l].length() == 17 ) //this should work for any standard rxp filename
				{
					ss.str("");
					ss << top_dir << positions[k] << "/" << position_contents[l];
					rxpname = ss.str();
				}
			}
			ss.str("");
			ss << top_dir << "matrix/" << "T" << positions[k][1] << positions[k][2] << positions[k][3] <<".DAT"; //the 3 digits going with the SOP that should start with T. So e.g. T001.DAT, T099.DAT, T125.DAT
			std::string matrixname = ss.str();
			std::cout << rxpname << " " << " " << matrixname << std::endl;
			std::shared_ptr<scanlib::basic_rconnection> rc;
			rc = scanlib::basic_rconnection::create(rxpname);
			rc->open();
			scanlib::decoder_rxpmarker dec(rc);
			pcloud pc;
			importer imp(pc);
			scanlib::buffer buf;
			for(dec.get(buf);!dec.eoi();dec.get(buf))
			{
				imp.dispatch(buf.begin(), buf.end());
			}
			rc->close();
			ss.str("");
			if(positions[k][1] == '0' && positions[k][2] == '0') ss << positions[k][3]; // line 178 (this line) - 182: this just attributes a variable scan number, so any point in the pcd point clouds can be tracked to a specific scan location
			else if(positions[k][1] == '0') ss << positions[k][2] << positions[k][3];
			else ss << positions[k][1] << positions[k][2] << positions[k][3];
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
				for(int n=0;n<steps;n++)
				{
					if(X > coords[n][0] && X < coords[n][1])
					{
						if(Y > coords[n][2] && Y < coords[n][3])
						{
							if(pc.deviation[m] <= deviation_max)
							{
								xyzfiles[n].write(reinterpret_cast<const char*>(&X),sizeof(X));
								xyzfiles[n].write(reinterpret_cast<const char*>(&Y),sizeof(Y));
								xyzfiles[n].write(reinterpret_cast<const char*>(&Z),sizeof(Z));
								count[n] += 1;
							}
						}
					}
				}
			}
		}
	}
	for(int p=0;p<steps;p++)
	{
		xyzfiles[p].close();
	}
	for(int q=0;q<steps;q++)
	{
		std::ofstream headerstream("header.tmp");
		headerstream << "VERSION 0.7" << std::endl << "FIELDS x y z" << std::endl << "SIZE 4 4 4" << std::endl << "TYPE F F F" << std::endl << "COUNT 1 1 1" << std::endl << "WIDTH " << count[q] << std::endl << "HEIGHT 1" << std::endl << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl << "POINTS " << count[q] << std::endl << "DATA binary" << std::endl;
		headerstream.close();
		ss.str("");
		ss << "cat header.tmp " << xyznames[q] << " > " << pcdnames[q] << "; rm header.tmp " << xyznames[q];
		std::string string;
		const char* cc;
		string = ss.str();
		cc = string.c_str();
		system(cc);
	}
	return 0;
}
