//Kim Calders: kim.calders@npl.co.uk

#include <treeseg.hpp>
#include <fstream>
#include <list>
#include <vector>
#include <iostream>

int main (int argc, char** argv)
{
	std::ifstream infile(argv[1]);
	int tile;
	float x,y;
	std::list<int> tiles_list;
	while (infile >> tile >> x >> y)
	{
//		std::cout << tile << " " << x <<" " << y << std::endl;
		float expansion = atof(argv[2]);
		if(expansion<30)
		{
		expansion = 30;
		}
		float xmin = 50-expansion; //change 50 to x from tree
		float xmax = 50+expansion;//change 50 to x from tree
		float ymin = 50-expansion;//change 50 to y from tree
		float ymax= 50+expansion;//change 50 to y from tree
		if(x >= xmin && x <= xmax && y >= ymin && y <= ymax)
		{
//			std::cout << tile << " " << x <<" " << y << std::endl;
			tiles_list.push_back(tile);
		}
	}
	std::list<int>::iterator it;
	for (it = tiles_list.begin(); it != tiles_list.end(); ++it){
//    		std::cout << *it << std::endl;
		std::stringstream ss;
		ss.str("");
		ss << argv[3] << "_" << *it << "_downsample.pcd";
    		std::cout << ss.str() << std::endl;		
	}
//	std::cout << tiles_list.size() << std::endl;
}
