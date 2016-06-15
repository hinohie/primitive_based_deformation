#ifndef __POLY_RECT__
#define __POLY_RECT__

#include <vector>
#include "vector3f.h"

class Polyrect{
public:
	vector<vector3f> v;

	double width, height;

	Polyrect()
	{
		width = 0;
		height =0;

		v.clear();
	}
	Polyrect(double _width, double _height)
	{
		width = _width;
		height = _height;

		v.clear();
	}
	~Polyrect(){
		v.clear();
	}
	void add(vector3f s)
	{
		if(v.size() > 0)
		{
			if(v.back().dist(s) < 0.5)
				return;
		}
		v.push_back(s); 
	}
	
	double max_x;
	double max_y;
	double min_x;
	double min_y;
	
	void sortAO();
	bool PtInRegion(int x, int y);
	bool cross(double mx,double my,vector3f p, vector3f q);


};

#endif