#ifndef __OBJECT_H__
#define __OBJECT_H__

#include<vector>
#include<algorithm>
#include<set>
using namespace std;

#include "vector3f.h"
#include "Shader.h"
#include "ObjIO.h"
#include "TetIO.h"
#include <Eigen.h>
#include "matrix33f.h"

#include "tinyxml.h"


#include <string>

class tetObject;
class Point{
public:
	vector3f x;
	vector3f x0;
	vector3f v;
	vector3f a;
	vector3f x_tmp;
	double inv_m;
	double m;
	bool drawable;
	void Integrate(tetObject *me,double dt);
	void collision(tetObject *me);
	void collision();
	matrix33f mxx0t;
};
class Object;
class Cluster{
public:
	int n;
	vector<int> vv;
	vector3f cmp;
	vector3f cmp0;
	matrix33f Eigen;
	matrix33f Ustatic;
	matrix33f R;
	double U[6];
	double Umode[4][6];
	MatrixXd AtA_inv;
	double sum_of_mass;
	bool work;
	bool non_fs;

	Cluster(){
		Eigen.setIden();
	}

	void getCenter(Object *me);
	matrix33f getMomentMatrix(Object *me);
	void precalUstaticMatrix(Object *me);
};
class Constraint{
public:
	double volume_init;
	vector<int> volume_index;
	vector<vector3f> volume_del;
	
	double volume_constraint(Object *me);
	void volume_del_constraint(Object *me);

	void setting(tetTash c, Object *me);
};
class Object{
public:
	
	objModel *model;
	objModel *target[3];
	int target_num;

	int n;
	int en;
	vector<Point> p;
	//
	vector<Point> cp;
	vector<int> cp_index;
	vector<vector3f> cp_param;

	tetObject *cmodel;
	int cn;
//	vector<CObject *> cd;
	//
	vector<Cluster> c;
	vector<vector3f> x0;	// inil_position
	vector<vector3f> x1[3];	// tinil_position
	vector<vector<vector3f> > x2[3];	// tinil_animation_pos
	int fn;
	vector<vector3f> g;		// goal position
	vector3f cmp;			// center of mass
	vector3f cmp0;			// center of mass inil
	bool inil_done;
	double beta;
	vector3f cmp1[4];			// center of mass target

	static GLuint show_type_1;
	static GLuint show_type_2;

	Object(){dt = 0.003;model = NULL;target_num = 0; beta = 0.995; tmodel = NULL;cmodel = NULL;example_rate =NULL;example_rate0=NULL;
	frame_num = 0; rrxx = zzxx = xxzz = NULL;
	inil_done = false;
	example_rate_exist = false;
	}
	~Object(){
		if(model)delete model;
		if(tmodel)delete tmodel;
		if(cmodel) delete cmodel;
		if(example_rate) delete[] example_rate;
		if(example_rate0) delete[] example_rate0;
	}


	double dt;
	bool attack;
	double target_cheat;

	void inil();
	void simul();
	void draw();
	void draw_cmodel(vector3f offset = vector3f(0,0,0));
	void draw_tmodel();
	void draw_example();

	void draw_cmodel_origin();
	void draw_tmodel_origin();

	void ShapeMatching();
	void ShapeMatching_target();
	void Integrate();
	
	void precalExampleUmode();
	void precalAtA();
	void dampingGlobal();
	MatrixXd AtA_inv;

	tetObject *tmodel;
	tetObject *vmodel;
	void inil_by_xml(tetObject *_cmodel = NULL, tetObject *_tmodel = NULL);

	void set_example_rate(double *_rate);

	void capture();
	void capture_setting();

	static void resize(tetObject *tmodel);
	void connect(tetObject *model);

	double wahaha;
	int frame_num;

	bool example_rate_exist;
	void change_example_rate_test(int a1, int a2, int a3, double arg);
	double *example_rate;
	double *example_rate0;

	set<int> ***rrxx;
	set<int> ***zzxx;
	set<int> ***xxzz;


};


#endif