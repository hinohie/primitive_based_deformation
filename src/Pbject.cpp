#include "Pbject.h"
#include "Image.h"

void Pbject::inil()
{
	if(objs.size()){
		for(auto xx : objs)
			delete xx;
		objs.clear();
		center.clear();
		objs_type.clear();
		base.clear();
		base_color.clear();
		dir.clear();
	}
	if(img == NULL)
	{
		img = new Image("resource/region.png");
		img->set_base();
	}

	n = 5; m = 5;
	tetObject *_cmodel = new tetObject();
//		_cmodel->getModel("resource/model/Simple_Happy_Buddha.tet");
		_cmodel->getModel("resource/model/Eight.tet");
		_cmodel->cornering();
		_cmodel->tash_reorder();
		_cmodel->normalize();
		_cmodel->model->resize();
	Object::resize(_cmodel);
	tetObject *_cmodel2 = new tetObject();
		_cmodel2->getModel("resource/model/Simple_Happy_Buddha.tet");
//		_cmodel->getModel("resource/model/Eight.tet");
		_cmodel2->cornering();
		_cmodel2->tash_reorder();
		_cmodel2->normalize();
		_cmodel2->model->resize();
	Object::resize(_cmodel2);
	tetObject *_cmodel3 = new tetObject();
		_cmodel3->getModel("resource/model/Rotate_Bar.tet");
//		_cmodel->getModel("resource/model/Eight.tet");
		_cmodel3->cornering();
		_cmodel3->tash_reorder();
		_cmodel3->normalize();
		_cmodel3->model->resize();
	Object::resize(_cmodel3);
	
	tetObject *_tmodel = new tetObject();
		_tmodel->getModel("resource/cube4.xml");	//twist
		_tmodel->cornering();
		_tmodel->tash_reorder();
		_tmodel->normalize();
	Object::resize(_tmodel);

	double *rate = new double[3];
	rate[2] = 0;
	double center_max_z, center_min_z;
	double center_max_x, center_min_x;
	double cdx,cdz;
	center_max_x = center_max_z = -1234567;
	center_min_x = center_min_z = 1234567;
	for(int i=0; i<n*m; i++)
	{
		Object *obj = new Object();
		int type = 0;
		printf("%4d(%d) : ",i,type);
		if(type == 0)obj->inil_by_xml(_cmodel, _tmodel);
		if(type == 1)obj->inil_by_xml(_cmodel2, _tmodel);
		if(type == 2)obj->inil_by_xml(_cmodel3, _tmodel);
//		obj->set_example_rate(rate);
		objs.push_back(obj);
		objs_type.push_back(-1);
		dir.push_back(vector3f());
//		center.push_back(vector3f((i%m) * -0.6, 0, (i/m)*-0.6));
//		center.push_back(vector3f((rand() / 32768.0 - 1) * m / 2.0, 0, (rand() / 32768.0 - 1)  * n / 2.0));
		center.push_back(vector3f(-(i%m) + (1 - 1) * 0.7, 0, -(i/m) + (1 - 1) * 0.7));
		center.back() *= 0.8;
		center_max_x = max(center_max_x, center.back().x);
		center_min_x = min(center_min_x, center.back().x);
		center_max_z = max(center_max_z, center.back().z);
		center_min_z = min(center_min_z, center.back().z);
	}
	pre_center = center;
	cdx = center_max_x - center_min_x + 1;
	cdz = center_max_z - center_min_z + 1;
	vector<int> *v;
	vector<float> rate_v;
	v = new vector<int>[img->base_n];
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			int id = img->get_base_id((center[i*m+j].x - center_min_x + cdx/2/m)/cdx, (center[i*m+j].z - center_min_z + cdz/2/n)/cdz);
			if(id >= 0)
				v[id].push_back(i*m + j);
		}
	}
	for(int i=0;i <img->base_n; i++)
	{
		base.push_back(v[i]);
	}
	//
	rate[0] = rate[1] = rate[2] = 0;
	rate[0] = 1;
	for(int i=0;i<v[0].size();i++){
		objs[v[0][i]]->set_example_rate(rate);
		objs_type[v[0][i]]=0;
		dir[v[0][i]] = vector3f(-1,0,0);
	}
	rate_v.clear();
	rate_v.push_back(rate[0]);
	rate_v.push_back(rate[1]);
	rate_v.push_back(rate[2]);
	base_color.push_back(rate_v);
	
	rate[0] = 0; rate[1] = 1;
	for(int i=0;i<v[1].size();i++){
		objs[v[1][i]]->set_example_rate(rate);
		objs_type[v[1][i]]=1;
		dir[v[1][i]] = vector3f(0,0,-1.5);
	}
	rate_v.clear();
	rate_v.push_back(rate[0]);
	rate_v.push_back(rate[1]);
	rate_v.push_back(rate[2]);
	base_color.push_back(rate_v);

	/*
	rate[0] = 0; rate[1] = 1;
	for(int i=0;i<v[2].size();i++){
		objs[v[2][i]]->set_example_rate(rate);
		objs_type[v[2][i]]=1;
		dir[v[2][i]] = vector3f(-0.5,0,-0.5);
	}
	rate_v.clear();
	rate_v.push_back(rate[0]);
	rate_v.push_back(rate[1]);
	rate_v.push_back(rate[2]);
	base_color.push_back(rate_v);
	*/

	delete[] rate;
	delete[] v;
	
	delete _cmodel;
	delete _cmodel2;
	delete _cmodel3;
	delete _tmodel;


	use_sum_to_max = false;
	use_sum_to_sum = true;

	diffuse();

	printf("pbj inil done!\n");
}

void Pbject::draw()
{
	
	for(int l=0; l<objs.size();l++)
	{
		int i, j;
		j = l/m;
		i = l%m;
		objs[l]->draw_cmodel(center[l]);
	}
}

static int frame_num = 0;
void Pbject::simul()
{
	for(int l=0; l<objs.size(); l++)
	{
		objs[l]->simul();
//		center[l] += (dir[l] / 60.0);
		vector3f res;
		center[l] += vector3f(m/2.0, 0, n/2.0) * 0.8;
		res = center[l];
		double dd = vector3f(res.x,0,res.z).dist();
		//res.y = (1 + cos(frame_num * 0.1 + dd))/2;
		center[l] = res;
		center[l] -= vector3f(m/2.0, 0, n/2.0) * 0.8;
		center[l] += dir[l] / 120.0;
	}
}

void Pbject::draw_base()
{
	int i, j, k;
	double dx,dy;
	double sx,sy;
	int mm, nn;
	int scale = 5;
	mm = m * scale;
	nn = n * scale;
	dx = 2.0 / mm;
	dy = 2.0 / nn;
	double dd = min(dx,dy);
	dx = dy = dd;

	sx = -dx * mm / 2.0;
	sy = dy * nn / 2.0;

	for(i=0;i<nn;i++)
		for(j=0;j<mm;j++)
		{
			{
				unsigned char r,g,b,a;
				img->get_color((j+0.5)/mm, (i+0.5)/nn, r,g,b,a);
				double rr,gg,bb,aa;
				rr = r / 255.0;
				gg = g / 255.0;
				bb = b / 255.0;
				aa = a / 255.0;
				glColor3f(rr,gg,bb);
			}
			glBegin(GL_QUADS);
			glVertex3f(sx + dx*j, sy - dy*i, 0);
			glVertex3f(sx + dx*(j+1), sy - dy*i, 0);
			glVertex3f(sx + dx*(j+1), sy - dy*(i+1), 0);
			glVertex3f(sx + dx*j, sy - dy*(i+1), 0);
			glEnd();
		}
}
void Pbject::draw_diffuse()
{
	int i, j, k;
	double dx,dy;
	double sx,sy;

	double scale_xx;
	double max_x, max_z, min_x, min_z;
	max_x = min_x = pre_center[0].x;
	max_z = min_z = pre_center[0].z;
	for(i=1; i<n*m; i++)
	{
		if(max_x < pre_center[i].x) max_x = pre_center[i].x;
		if(min_x > pre_center[i].x) min_x = pre_center[i].x;
		if(max_z < pre_center[i].z) max_z = pre_center[i].z;
		if(min_z > pre_center[i].z) min_z = pre_center[i].z;
	}
	scale_xx = max(max_x - min_x, max_z - min_z) / 2;
	
	dx = 18.0 / m;
	dy = 18.0 / n;
	double dd = min(dx,dy);
	dx = dy = dd * 1;

	sx = -(max_x - min_x)/scale_xx / 2;
	sy = (max_z - min_z)/scale_xx / 2;

	for(i=0;i<n;i++)
		for(j=0;j<m;j++)
		{
			int id = i*m + j;
			if(objs[id]->example_rate_exist)
				glColor3f(objs[id]->example_rate0[0],objs[id]->example_rate0[1],objs[id]->example_rate0[2]);
			else
				glColor3f(1.0, 1.0, 1.0);
			glBegin(GL_QUADS);
			glVertex3f(sx + (pre_center[id].x - dx/2 - min_x) / scale_xx, sy - (pre_center[id].z - dy/2 - min_z) / scale_xx, 0);
			glVertex3f(sx + (pre_center[id].x + dx/2 - min_x) / scale_xx, sy - (pre_center[id].z - dy/2 - min_z) / scale_xx, 0);
			glVertex3f(sx + (pre_center[id].x + dx/2 - min_x) / scale_xx, sy - (pre_center[id].z + dy/2 - min_z) / scale_xx, 0);
			glVertex3f(sx + (pre_center[id].x - dx/2 - min_x) / scale_xx, sy - (pre_center[id].z + dy/2 - min_z) / scale_xx, 0);
			glEnd();
		}
}
void Pbject::draw_position()
{
	int i, j, k;
	double dx,dy;
	double sx,sy;

	double scale_xx;
	double max_x, max_z, min_x, min_z;
	max_x = min_x = center[0].x;
	max_z = min_z = center[0].z;
	for(i=1; i<n*m; i++)
	{
		if(max_x < center[i].x) max_x = center[i].x;
		if(min_x > center[i].x) min_x = center[i].x;
		if(max_z < center[i].z) max_z = center[i].z;
		if(min_z > center[i].z) min_z = center[i].z;
	}
	scale_xx = max(max_x - min_x, max_z - min_z) / 2;
	
	dx = 10.0 / m;
	dy = 10.0 / n;
	double dd = min(dx,dy);
	dx = dy = dd * 1;

	sx = -(max_x - min_x)/scale_xx / 2;
	sy = (max_z - min_z)/scale_xx / 2;

	for(i=0;i<n;i++)
		for(j=0;j<m;j++)
		{
			int id = i*m + j;
			if(objs[id]->example_rate_exist)
				glColor3f(objs[id]->example_rate[0],objs[id]->example_rate[1],objs[id]->example_rate[2]);
			else
				glColor3f(1.0, 1.0, 1.0);
			glBegin(GL_QUADS);
			glVertex3f(sx + (center[id].x - dx/2 - min_x) / scale_xx, sy - (center[id].z - dy/2 - min_z) / scale_xx, 0);
			glVertex3f(sx + (center[id].x + dx/2 - min_x) / scale_xx, sy - (center[id].z - dy/2 - min_z) / scale_xx, 0);
			glVertex3f(sx + (center[id].x + dx/2 - min_x) / scale_xx, sy - (center[id].z + dy/2 - min_z) / scale_xx, 0);
			glVertex3f(sx + (center[id].x - dx/2 - min_x) / scale_xx, sy - (center[id].z + dy/2 - min_z) / scale_xx, 0);
			glEnd();
		}
}

void Pbject::diffuse()
{
	int i, j, k;
	double *rate = new double[3];
	double *weight = new double[base.size()];
	for(i=0; i<n*m; i++)
	{
		if(objs[i]->example_rate_exist) continue; // in base
		rate[0]=rate[1]=rate[2] = 0;
		vector3f dir_sum;
		double sum=-2;
		if(use_sum_to_max) sum = -2;
		else if(use_sum_to_sum) sum = 0;
		for(j=0; j<base.size(); j++)
		{
			weight[j] = 0;
			double s1=-1;
			for(k=0; k<base[j].size(); k++)
			{
			double dd = (center[i] - center[base[j][k]]).dist();
			if(s1 < 0 || s1 > dd) s1 = dd;
			}
			weight[j] = 1/s1;
			//sum += weight[j];
			if(use_sum_to_max)
				sum = max(sum, weight[j]);
			else if(use_sum_to_sum)
				sum = sum + weight[j];
		}
		for(j=0; j<base.size(); j++)
			weight[j] /= sum;
		for(j=0; j<base.size(); j++)
		{
			rate[0] += base_color[j][0] * weight[j];
			rate[1] += base_color[j][1] * weight[j];
			rate[2] += base_color[j][2] * weight[j];

			if(base[j].size())
			dir_sum += dir[base[j][0]] * weight[j];
		}
		objs[i]->set_example_rate(rate);
		dir[i] = dir_sum;
	}
	delete[] weight;
	delete[] rate;
}
static bool capture_inil = false;
static objModel *model_out;
static vector<int> ni;
void Pbject::capture_setting()
{
	model_out = new objModel();
	vector<int> x;
	int xl;
	int i, j, k;
	vector<int> vv_cnt;
	int cnt=0;
	int fcnt=0;

	ni.clear();
	for(i=0; i<objs.size(); i++)
	{
		vv_cnt.push_back(cnt);
		cnt += objs[i]->cp.size();
		for(j=0; j<objs[i]->cp.size();j++)
		ni.push_back(-1);
	}
	vv_cnt.push_back(cnt);
	
	objObject *obj = new objObject();
	for(int l=0; l<objs.size(); l++)
	{
		for(i=0; i<objs[l]->cmodel->model->obj.size(); i++)
		{
			for(j=0; j<objs[l]->cmodel->model->obj[i]->f.size(); j++)
			{
			objFace *face = new objFace();
				for(k=0; k<objs[l]->cmodel->model->obj[i]->f[j]->vv.size(); k++)
				{
					x.push_back(objs[l]->cmodel->model->obj[i]->f[j]->vv[k] + vv_cnt[l]);
					face->vv.push_back(objs[l]->cmodel->model->obj[i]->f[j]->vv[k] + vv_cnt[l]);
					fcnt++;
				}
			obj->f.push_back(face);
			}
		}
		for(i=0;i <objs[l]->cp.size(); i++)
			model_out->vv.push_back(vector3f());
	}


	model_out->obj.push_back(obj);
	
	sort(x.begin(), x.end());
	x.erase(unique(x.begin(),x.end()),x.end());
	for(i=0; i<x.size();i++)
		ni[x[i]] = i;
	
		for(i=0; i<model_out->obj.size(); i++)
		{
			for(j=0; j<model_out->obj[i]->f.size(); j++)
			{
				objFace *face = model_out->obj[i]->f[j];
				for(k=0; k<face->vv.size(); k++)
				{
					face->vv[k] = ni[face->vv[k]];
				}
			}
		}
	
	int n_cnt=0;
	for(i=0; i<objs.size(); i++)
	{
		for(j=vv_cnt[i]; j<vv_cnt[i+1]; j++)
		{
			if(ni[j] >= 0)
			{
				n_cnt++;
			}
		}
	}
	model_out->vv.resize(n_cnt);
	printf("setting done! v: %d, %d f: %d\n",cnt, n_cnt, fcnt);
}
void Pbject::capture()
{
	char filename[40];
	int i, j, k;
	if(!capture_inil)
	{
		capture_inil=true;
		capture_setting();
	}
	int cnt=0;
	for(int l=0; l<objs.size(); l++)
	{
		for(i=0; i<objs[l]->cp.size(); i++)
		{
			vector3f xx = objs[l]->cp[i].x;
			xx += vector3f(objs[l]->cmodel->trans_before_x,objs[l]->cmodel->trans_before_y,objs[l]->cmodel->trans_before_z);
			xx ^= vector3f(objs[l]->cmodel->scale_xx,objs[l]->cmodel->scale_xx,objs[l]->cmodel->scale_xx);
			xx += vector3f(objs[l]->cmodel->trans_x,objs[l]->cmodel->trans_y,objs[l]->cmodel->trans_z);
			model_out->vv[ni[i + cnt]] = xx + center[l];
		}
		cnt += objs[l]->cp.size();
	}
	sprintf(filename,"output/%d.obj",frame_num);
	model_out->writeModel(filename);
	printf("capture %s\n",filename);

	sprintf(filename,"output/%d.shader",frame_num);
	FILE *fp = fopen(filename, "wt");
	cnt=0;
	int n_cnt = 0;
	for(int l=0; l<objs.size(); l++)
	{
		int rr,gg,bb;
		if(objs[l]->example_rate_exist)
		{
			rr = (int)(objs[l]->example_rate[0] * 255);
			gg = (int)(objs[l]->example_rate[1] * 255);
			bb = (int)(objs[l]->example_rate[2] * 255);
		}
		else
		{
			rr = gg = bb = 170;
		}
		if(rr<0)rr=0; if(rr>255) rr=255;
		if(gg<0)gg=0; if(gg>255) gg=255;
		if(bb<0)bb=0; if(bb>255) bb=255;
		for(i=0; i<objs[l]->cp.size(); i++)
		{
			if(ni[cnt + i] >= 0)
			{
				n_cnt++;
				fprintf(fp,"%d %d %d %d\n",n_cnt,rr,gg,bb);
			}
		}
		cnt += objs[l]->cp.size();
	}
	fclose(fp);

	frame_num ++;
}

void Pbject::capture_example()
{
	char filename[40];
	int i, j, k;
	objModel *model_example;
	model_example = new objModel();
	int l = 0;
	// malloc
	for(i=0; i<objs[l]->tmodel->exam_pos[3].size(); i++)
	{
		model_example->vv.push_back(vector3f());
	}
	for(i=0; i<objs[l]->tmodel->model->obj.size(); i++)
	{
		objObject *obj = new objObject();
		for(j=0; j<objs[l]->tmodel->model->obj[i]->f.size(); j++)
		{
			objFace *face = new objFace();
			for(k=0; k<objs[l]->tmodel->model->obj[i]->f[j]->vv.size(); k++)
			{
				face->vv.push_back(objs[l]->tmodel->model->obj[i]->f[j]->vv[k]);
			}
			obj->f.push_back(face);
		}
		model_example->obj.push_back(obj);
	}
	for(i=0; i<objs[l]->tmodel->exam_pos[2].size(); i++)
	{
		vector3f xx = objs[l]->tmodel->exam_pos[3][i];
		xx += vector3f(objs[l]->tmodel->trans_before_x,objs[l]->tmodel->trans_before_y,objs[l]->tmodel->trans_before_z);
		xx ^= vector3f(objs[l]->tmodel->scale_xx,objs[l]->tmodel->scale_xx,objs[l]->tmodel->scale_xx);
		xx += vector3f(objs[l]->tmodel->trans_x,objs[l]->tmodel->trans_y,objs[l]->tmodel->trans_z);
		model_example->vv[i] = xx;
	}
	sprintf(filename,"output/%d.obj",frame_num);
	model_example->writeModel(filename);
	printf("capture %s\n",filename);

	sprintf(filename,"output/%d.shader",frame_num);
	FILE *fp = fopen(filename, "wt");

		int rr,gg,bb;
		rr=gg=bb=170;
		if(rr<0)rr=0; if(rr>255) rr=255;
		if(gg<0)gg=0; if(gg>255) gg=255;
		if(bb<0)bb=0; if(bb>255) bb=255;
		for(i=0; i<objs[l]->tmodel->exam_pos[1].size(); i++)
		{
			fprintf(fp,"%d %d %d %d\n",i,rr,gg,bb);
		}
	fclose(fp);

	frame_num ++;
}