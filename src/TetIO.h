#ifndef __OBJECT_IO_TET_H__
#define __OBJECT_IO_TET_H__

#include"vector3f.h"
#include "ObjIO.h"
#include "tinyxml.h"
#include<vector>
#include<algorithm>
using namespace std;
#define TETRA_WORK
class tetTash{
public:
	int v0,v1,v2,v3;
	tetTash(){v0=v1=v2=v3=0;}
};
#include<set>
class tetObject{
public:
	vector<tetTash> t;
	vector<vector3f> exam_pos[4];
	vector<vector<vector3f> > exam_pos_animated[4];
	vector<int> *neighbor;
	objModel *model;
	int tn;
	int pn;
	int en;
	bool is_mesh;
	vector<bool> corner;
	set<pair<int, pair<int, int> > > corner_set;
	
	double trans_before_x,trans_before_y,trans_before_z;
	double trans_x,trans_y,trans_z;
	double scale_xx;
	tetObject()
	{
		t.clear();
		tn = 0;
		en = 0;
		neighbor = NULL;
		model = NULL;
		is_mesh = false;
		for(int i=0; i<16; i++)scale_matrix[i] = 0;
		scale_matrix[0] = scale_matrix[5] = scale_matrix[10] = 1;
	}
	tetObject(const tetObject *me)
	{
		if(me == NULL){tetObject(); return;}
		this->tn = me->tn;
		this->pn = me->pn;
		this->en = me->en;
		this->is_mesh = me->is_mesh;
		
		this->trans_before_x = me->trans_before_x;
		this->trans_before_y = me->trans_before_y;
		this->trans_before_z = me->trans_before_z;
		this->trans_x = me->trans_x;
		this->trans_y = me->trans_y;
		this->trans_z = me->trans_z;
		this->scale_xx = me->scale_xx;
		this->t = vector<tetTash>(me->t.begin(), me->t.end());
		this->exam_pos[0] = vector<vector3f>(me->exam_pos[0].begin(), me->exam_pos[0].end());
		this->exam_pos[1] = vector<vector3f>(me->exam_pos[1].begin(), me->exam_pos[1].end());
		this->exam_pos[2] = vector<vector3f>(me->exam_pos[2].begin(), me->exam_pos[2].end());
		this->exam_pos[3] = vector<vector3f>(me->exam_pos[3].begin(), me->exam_pos[3].end());
		
		this->model = new objModel(me->model);

		this->neighbor = new vector<int>[me->pn];
		for(int i=0;i<me->pn ;i++)
			this->neighbor[i] = vector<int>(me->neighbor[i].begin(), me->neighbor[i].end());

		this->corner = vector<bool>(me->corner.begin(), me->corner.end());
		this->corner_set = set<pair<int, pair<int, int> > >(me->corner_set.begin(), me->corner_set.end());
		
		this->tet_volume = vector<double>(me->tet_volume.begin(), me->tet_volume.end());
		this->obj_tetmap = vector<int>(me->obj_tetmap.begin(), me->obj_tetmap.end());
		this->obj_param = vector<vector3f>(me->obj_param.begin(), me->obj_param.end());

		for(int i=0;i<16;i++)
			this->scale_matrix[i] = me->scale_matrix[i];
	}
	~tetObject()
	{
		delete[] neighbor;
		delete model;
	}
	void resize();
	void tash_reorder();
	void cornering();
	void normalize();
	bool getModel(const char *filename);
	bool getModel(const char *filename, const char *objname);
	bool loadTET(const char *fnamefname);
	bool loadTET(const char *fnamefname, const char *objname);
	bool loadXML(const string &fname);
	bool loadXML(const TiXmlElement* txe_object);
	void make_neighbor();
	vector<double> tet_volume;
	vector<int> obj_tetmap;
	vector<vector3f> obj_param;
	double scale_matrix[16];
};
#endif
