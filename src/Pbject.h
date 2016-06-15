#ifndef __PBJECT_H__
#define __PBJECT_H__
#include "Object.h"
#include "Image.h"
#include<vector>
using namespace std;

class Pbject {
public:
	vector<Object *> objs;
	vector<vector3f> center;
	vector<vector3f> pre_center;
	vector<vector3f> dir;
	vector<int> objs_type;
	vector<vector<int> > base;
	vector<vector<float> > base_color;
	Image *img;
	int n, m;

	bool use_sum_to_max;
	bool use_sum_to_sum;

	Pbject(){img = NULL;}

	void diffuse();

	void inil();
	void draw();
	void simul();

	void draw_base();
	void draw_diffuse();
	void draw_position();

	void capture_setting();
	void capture();
	void capture_example();
};

#endif