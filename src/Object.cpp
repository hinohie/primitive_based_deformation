#include<stdio.h>
#include"Object.h"


static double trans_before_x = 0,trans_before_y = 0,trans_before_z = 0;
static double trans_x = 0,trans_y = 0,trans_z = 0;
static double scale_xx = 1;
static double xe_min=-99, xe_max=99;
static double ye_min=0, ye_max=99;
static double ze_min=-99, ze_max=99;
void Point::collision(tetObject *me)
{
	double reflect_rate = 0.0;
	double repulse_rate = 0.0;
	double min_x,min_y,min_z;
	double max_x,max_y,max_z;
	double margin = 0.01;
	min_x = ((xe_min - margin) - me->trans_x) / me->scale_xx - me->trans_before_x;
	min_y = ((ye_min - margin) - me->trans_y) / me->scale_xx - me->trans_before_y;
	min_z = ((ze_min - margin) - me->trans_z) / me->scale_xx - me->trans_before_z;
	max_x = ((xe_max + margin) - me->trans_x) / me->scale_xx - me->trans_before_x;
	max_y = ((ye_max + margin) - me->trans_y) / me->scale_xx - me->trans_before_y;
	max_z = ((ze_max + margin) - me->trans_z) / me->scale_xx - me->trans_before_z;
	if(x.x < min_x){ x.x = 2*min_x - x.x; v.x = -v.x * reflect_rate;a.x += repulse_rate * 1.0 * inv_m;}
	if(x.y < min_y){ x.y = 2*min_y - x.y; v.y = -v.y * reflect_rate;a.y += repulse_rate * 1.0 * inv_m;}
	if(x.z < min_z){ x.z = 2*min_z - x.z; v.z = -v.z * reflect_rate;a.z += repulse_rate * 1.0 * inv_m;}
	if(x.x > max_x){ x.x = 2*max_x - x.x; v.x = -v.x * reflect_rate;a.x += repulse_rate * -1.0 * inv_m;}
	if(x.y > max_y){ x.y = 2*max_y - x.y; v.y = -v.y * reflect_rate;a.y += repulse_rate * -1.0 * inv_m;}
	if(x.z > max_z){ x.z = 2*max_z - x.z; v.z = -v.z * reflect_rate;a.z += repulse_rate * -1.0 * inv_m;}
}
void Point::Integrate(tetObject *me,double dt)
{
	v = v + a * dt;
	v *= 0.99;
	x = x + v*dt;
	a = 0;
	collision(me);
}

static void make_twist_gidoong(objModel *model, int n = 20, double dangle = 0)
{
	int i, j;
	double angle = 0;
	objObject *obj = new objObject();
	objFace *face;
	for(i=0; i<n; i++)
	{
		for(j=0;j<4;j++)
		{
			face = new objFace();
			face->vv.push_back(4*i + j);
			face->vv.push_back(4*i + (j+1)%4);
			face->vv.push_back(4*i + 4 + j);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + 4 + j);
			face->vv.push_back(4*i + (j+1)%4);
			face->vv.push_back(4*i + (j+1)%4+4);
			obj->f.push_back(face);
		}
	}

	double dd = 0.25;
	double dh = dd*4 / n;
	for(i=0;i<=n;i++)
	{
		angle = dangle / n * i;
		vector3f tmp;
		tmp = vector3f(dd*cos(angle) - dd*sin(angle), i*dh, dd*sin(angle) + dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle) - dd*sin(angle), i*dh, -dd*sin(angle) + dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle) + dd*sin(angle), i*dh, -dd*sin(angle) - dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(dd*cos(angle) + dd*sin(angle), i*dh, dd*sin(angle) - dd*cos(angle));
		model->vv.push_back(tmp);
	}

	face = new objFace();
	face->vv.push_back(0);
	face->vv.push_back(3);
	face->vv.push_back(1);
	obj->f.push_back(face);
	face = new objFace();
	face->vv.push_back(1);
	face->vv.push_back(3);
	face->vv.push_back(2);
	obj->f.push_back(face);
	face = new objFace();
	face->vv.push_back(4*n + 0);
	face->vv.push_back(4*n + 3);
	face->vv.push_back(4*n + 1);
	obj->f.push_back(face);
	face = new objFace();
	face->vv.push_back(4*n + 1);
	face->vv.push_back(4*n + 3);
	face->vv.push_back(4*n + 2);
	obj->f.push_back(face);

	model->obj.push_back(obj);
	model->normalize();
}

static void make_twist_gidoong_tetra(objModel *model, int n = 20, double dangle = 0)
{
	int i, j, k;
	double angle = 0;
	objObject *obj;
	objFace *face;
//	int ddi[5][4] = {{0,1,3,4},{1,2,3,6},{4,5,6,1},{4,6,7,3},{1,3,4,6}};
	int ddi[6][4] = {{0,1,3,4},{1,2,3,6},{4,7,5,3},{5,7,6,3},{5,1,4,3},{5,6,1,3}};
	for(i=0; i<n; i++)
	{
		// tet 0, 1, 3, 4
			obj = new objObject();
		for(j=0;j<6;j++)
		{
			obj->vv.push_back(4*i + ddi[j][0]);
			obj->vv.push_back(4*i + ddi[j][1]);
			obj->vv.push_back(4*i + ddi[j][2]);
			obj->vv.push_back(4*i + ddi[j][3]);
			obj->n = obj->vv.size();
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][0]);
			face->vv.push_back(4*i + ddi[j][1]);
			face->vv.push_back(4*i + ddi[j][3]);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][0]);
			face->vv.push_back(4*i + ddi[j][2]);
			face->vv.push_back(4*i + ddi[j][1]);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][0]);
			face->vv.push_back(4*i + ddi[j][3]);
			face->vv.push_back(4*i + ddi[j][2]);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][1]);
			face->vv.push_back(4*i + ddi[j][2]);
			face->vv.push_back(4*i + ddi[j][3]);
			obj->f.push_back(face);
		}
			model->obj.push_back(obj);
	}

	double dd = 0.25;
	double dh = dd*4 / n;

	for(i=0;i<=n;i++)
	{
		angle = dangle / n * i - dangle / n * i / 2;
		vector3f tmp;
		
		tmp = vector3f(dd*cos(angle) - dd*sin(angle), i*dh, dd*sin(angle) + dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle) - dd*sin(angle), i*dh, -dd*sin(angle) + dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle) + dd*sin(angle), i*dh, -dd*sin(angle) - dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(dd*cos(angle) + dd*sin(angle), i*dh, dd*sin(angle) - dd*cos(angle));
		model->vv.push_back(tmp);
		
		/*
		tmp = vector3f(dd + 3*dd*cos(angle), i*dh, dd);
		model->vv.push_back(tmp);
		tmp = vector3f(-dd + 3*dd*cos(angle), i*dh, dd);
		model->vv.push_back(tmp);
		tmp = vector3f(-dd + 3*dd*cos(angle), i*dh, -dd);
		model->vv.push_back(tmp);
		tmp = vector3f(dd + 3*dd*cos(angle), i*dh, -dd);
		model->vv.push_back(tmp);
		*/
	}

	model->normalize();
}

static void make_shear_gidoong_tetra(objModel *model, int n = 20, double dangle = 0)
{
	int i, j, k;
	double angle = 0;
	objObject *obj;
	objFace *face;
//	int ddi[5][4] = {{0,1,3,4},{1,2,3,6},{4,5,6,1},{4,6,7,3},{1,3,4,6}};
	int ddi[6][4] = {{0,1,3,4},{1,2,3,6},{4,7,5,3},{5,7,6,3},{5,1,4,3},{5,6,1,3}};
	for(i=0; i<n; i++)
	{
		// tet 0, 1, 3, 4
		for(j=0;j<6;j++)
		{
			obj = new objObject();
			obj->vv.push_back(4*i + ddi[j][0]);
			obj->vv.push_back(4*i + ddi[j][1]);
			obj->vv.push_back(4*i + ddi[j][2]);
			obj->vv.push_back(4*i + ddi[j][3]);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][0]);
			face->vv.push_back(4*i + ddi[j][1]);
			face->vv.push_back(4*i + ddi[j][3]);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][0]);
			face->vv.push_back(4*i + ddi[j][2]);
			face->vv.push_back(4*i + ddi[j][1]);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][0]);
			face->vv.push_back(4*i + ddi[j][3]);
			face->vv.push_back(4*i + ddi[j][2]);
			obj->f.push_back(face);
			face = new objFace();
			face->vv.push_back(4*i + ddi[j][1]);
			face->vv.push_back(4*i + ddi[j][2]);
			face->vv.push_back(4*i + ddi[j][3]);
			obj->f.push_back(face);
			model->obj.push_back(obj);
		}
	}

	double dd = 0.25;
	double dh = dd*4 / n;

	for(i=0;i<=n;i++)
	{
		angle = dangle / n * i;
		vector3f tmp;
		/*
		tmp = vector3f(dd*cos(angle) - dd*sin(angle), i*dh, dd*sin(angle) + dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle) - dd*sin(angle), i*dh, -dd*sin(angle) + dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle) + dd*sin(angle), i*dh, -dd*sin(angle) - dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(dd*cos(angle) + dd*sin(angle), i*dh, dd*sin(angle) - dd*cos(angle));
		model->vv.push_back(tmp);
		*/
		tmp = vector3f(dd + dd*cos(angle), i*dh, dd + dd*cos(angle*1.3));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd + dd*cos(angle), i*dh, dd + dd*cos(angle*1.3));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd + dd*cos(angle), i*dh, -dd + dd*cos(angle*1.3));
		model->vv.push_back(tmp);
		tmp = vector3f(dd + dd*cos(angle), i*dh, -dd + dd*cos(angle*1.3));
		model->vv.push_back(tmp);
		/*
		
		tmp = vector3f(dd*cos(angle), i*dh, dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle), i*dh, dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(-dd*cos(angle), i*dh, -dd*cos(angle));
		model->vv.push_back(tmp);
		tmp = vector3f(dd*cos(angle), i*dh, -dd*cos(angle));
		model->vv.push_back(tmp);
		*/
	}

	model->normalize();
}
static vector3f attack_dir;
static int attack_index = -1;
void Object::inil()
{
#ifdef TETRA_WORK
	inil_by_xml();
#else
	int i, j, k;
	if(model == NULL)
	{
		model = new objModel();
	//	model->getModel("resource/dragon.obj");
	//	model->getModel("resource/cube.obj");
	//	model->getModel("resource/tet.obj");
	//	model->getModel("resource/Butterfly.obj");
	//	model->getModel("resource/Rubik'sCube.obj");
	//	model->getModel("resource/fsaverage.lh.pial.obj");model->normalize();
	//	model->getModel_3ds("G08_1013.3ds"); model->normalize();model->resize();
	//	model->getModel("resource/Tiger.obj");
		make_twist_gidoong_tetra(model,19,0);

		target[0] = new objModel();
		target_cheat=0;
		make_twist_gidoong_tetra(target[0], 19, acos(0.0)*2);
		target[1] = new objModel();
		make_shear_gidoong_tetra(target[1], 19, acos(0.0)*3);
	}
	printf("vertex : %d\n",model->vv.size());
	for(i=j=0;i<model->obj.size();i++)
	{
		j+=model->obj[i]->f.size();
	}
	printf("face   : %d\n",j);
	attack = false;
	attack_index = -1;

	p.clear();
	x0.clear();
	g.clear();
	cmp = vector3f(0,0,0);
	
	n = model->vv.size();
	for(i=0;i<n;i++)
	{
		Point pp;
//		pp.x = model->vv[i]/80 + vector3f(0.23, 0.35, -0.5);	// Dragon
//		pp.x = model->vv[i]/4 + vector3f(0.5, 0.7, 0.5) ;	// Cube
//		pp.x = model->vv[i]/-2 + vector3f(0.5+0.01*i, 0.5, 0.5);	// tet
//		pp.x = model->vv[i] / 80 + vector3f(0,0.2,1.5);
//		pp.x = model->vv[i] / 200 + vector3f(0.5, 0.7, 0.5);
//		pp.x = model->vv[i] / 80 + vector3f(0.25, 1.55, 0.2);		//Brain
//		pp.x = model->vv[i] + vector3f(0, 0.5, 0);;
//		pp.x = model->vv[i] / 1500 + vector3f(1.1, 1, -0.4);	// Tiger
		pp.x = model->vv[i];
		pp.x0 = pp.x;
		pp.v = pp.a = vector3f(0,0,0);
		pp.m = 1;
		pp.inv_m = 1;

		p.push_back(pp);
		x0.push_back(pp.x);
		g.push_back(pp.x);
		cmp += pp.x;
	}
	for(j=0; j<2;j++)
	{
		cmp1[j] = vector3f(0,0,0);
		x1[j].clear();
		for(i=0;i<n;i++)
		{
			cmp1[j] += target[j]->vv[i];
			x1[j].push_back(target[j]->vv[i]);
		}
		cmp1[j]/=n;
	}
	cmp /= n;
	cmp0 = cmp;
	for(i=0;i<n;i++)
	{
		vector3f xx = p[i].x - cmp;
		vector3f axis = vector3f(0, 0.2, 1);
		xx *= 0;
		axis.normalize();
		p[i].v += xx%axis;
		p[i].v += vector3f(0, 0, 0);
	}
	/*
	volume_del.clear();
	volume_index.clear();
	for(i=0; i<model->obj.size();i++)
	{
		objObject *obj = model->obj[i];
		for(j=0; j<obj->f.size();j++)
		{
			objFace *face = obj->f[j];
			for(k=1; k+1<face->vv.size();k++)
			{
				volume_index.push_back(face->vv[0]);
				volume_index.push_back(face->vv[k]);
				volume_index.push_back(face->vv[k+1]);
				volume_del.push_back(0);
				volume_del.push_back(0);
				volume_del.push_back(0);
			}
		}
	}
	
	double res=0;
	for(i=0;i<volume_index.size();i+=3)
	{
		res += (p[volume_index[i+0]].x%p[volume_index[i+1]].x)*p[volume_index[i+2]].x;
	}
	volume_init = res;
	*/



	// cluster wahaha
	int obn = model->obj.size();
	c.clear();
	for(i=0;i<obn;i++)
	{
		Cluster cc;
		cc.n = model->obj[i]->vv.size();
		cc.vv = model->obj[i]->vv;
		cc.precalUstaticMatrix(this);
		c.push_back(cc);
	}
	precalExampleUmode();
	precalAtA();
#endif
}
double cal_triangle_area(vector3f p, vector3f q, vector3f r)
{
	vector3f n = (q-p)%(r-p);
	return n.dist() / 2.0;
}
double cal_tetra_height(vector3f p, vector3f q, vector3f r, vector3f s)
{
	vector3f n = (q-p)%(r-p);
	n.normalize();
	double xx,yy;
	xx = p*n;
	yy = s*n;
	double res = fabs(xx-yy);
	return res;
}

void pung(tetObject *model,double scale)
{
	int i, j, k;
	vector<vector3f> v;
	vector<int> vn;
	v.resize(model->pn);
	vn.resize(model->pn);
	for(i=0;i<model->pn;i++)
	{
		v[i] = 0;
		vn[i] = 0;
	}
	for(i=0;i<model->tn;i++)
	{
		tetTash tash = model->t[i];
		vector3f p0, p1, p2, p3;
		p0 = model->exam_pos[0][tash.v0];
		p1 = model->exam_pos[0][tash.v1];
		p2 = model->exam_pos[0][tash.v2];
		p3 = model->exam_pos[0][tash.v3];
		vector3f cmp = (p0+p1+p2+p3)/4;
		p0 = (p0 - cmp) * scale + cmp;
		p1 = (p1 - cmp) * scale + cmp;
		p2 = (p2 - cmp) * scale + cmp;
		p3 = (p3 - cmp) * scale + cmp;
		v[tash.v0] += p0;
		v[tash.v1] += p1;
		v[tash.v2] += p2;
		v[tash.v3] += p3;
		vn[tash.v0] ++;
		vn[tash.v1] ++;
		vn[tash.v2] ++;
		vn[tash.v3] ++;
	}
	for(i=0;i<model->pn;i++)
	{
		v[i] /= vn[i];
		model->exam_pos[0][i] = v[i];
	}
}
void Object::connect(tetObject *model)
{
//	pung(model,2.0);
	int i, j, k;
	int ten = 20;
	int df = 1;
	if(rrxx==NULL)
	{
		rrxx = new set<int>**[ten];
		for(i=0;i<ten;i++)
		{
			rrxx[i] = new set<int>*[ten];
			for(j=0;j<ten;j++)
			{
				rrxx[i][j] = new set<int>[ten];
			}
		}
	}
	for(i=0;i<ten;i++)
		for(j=0;j<ten;j++)
			for(k=0;k<ten;k++)
				rrxx[i][j][k].clear();
	if(zzxx==NULL)
	{
		zzxx = new set<int>**[ten];
		for(i=0;i<ten;i++)
		{
			zzxx[i] = new set<int>*[ten];
			for(j=0;j<ten;j++)
			{
				zzxx[i][j] = new set<int>[ten];
			}
		}
	}
	for(i=0;i<ten;i++)
		for(j=0;j<ten;j++)
			for(k=0;k<ten;k++)
				zzxx[i][j][k].clear();
	/// inil_zzxx
	for(j=0;j<model->tn;j++)
	{
		int qx,qy,qz;
		int rx,ry,rz;
		int tj=j;
			double sum = 0;
			tetTash tash = model->t[j];
			vector3f p0, p1, p2, p3;
			p0 = model->exam_pos[0][tash.v0];
			p1 = model->exam_pos[0][tash.v1];
			p2 = model->exam_pos[0][tash.v2];
			p3 = model->exam_pos[0][tash.v3];
			p0.x = (p0.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p0.y = (p0.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p0.z = (p0.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p1.x = (p1.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p1.y = (p1.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p1.z = (p1.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p2.x = (p2.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p2.y = (p2.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p2.z = (p2.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p3.x = (p3.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p3.y = (p3.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p3.z = (p3.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			rx = qx = p0.x * ten;
			ry = qy = p0.y * ten;
			rz = qz = p0.z * ten;
			if(qx > p1.x * ten)qx = p1.x * ten; if(rx < p1.x * ten) rx = p1.x * ten;
			if(qy > p1.y * ten)qy = p1.y * ten; if(ry < p1.y * ten) ry = p1.y * ten;
			if(qz > p1.z * ten)qz = p1.z * ten; if(rz < p1.z * ten) rz = p1.z * ten;
			if(qx > p2.x * ten)qx = p2.x * ten; if(rx < p2.x * ten) rx = p2.x * ten;
			if(qy > p2.y * ten)qy = p2.y * ten; if(ry < p2.y * ten) ry = p2.y * ten;
			if(qz > p2.z * ten)qz = p2.z * ten; if(rz < p2.z * ten) rz = p2.z * ten;
			if(qx > p3.x * ten)qx = p3.x * ten; if(rx < p3.x * ten) rx = p3.x * ten;
			if(qy > p3.y * ten)qy = p3.y * ten; if(ry < p3.y * ten) ry = p3.y * ten;
			if(qz > p3.z * ten)qz = p3.z * ten; if(rz < p3.z * ten) rz = p3.z * ten;
			for(int i= qx-df; i<=rx+df;i++)
			for(int j= qy-df; j<=ry+df;j++)
			for(int k= qz-df; k<=rz+df;k++)
				if(i>=0&&i<ten)
				if(j>=0&&j<ten)
				if(k>=0&&k<ten)
				zzxx[i][j][k].insert(tj);
	}
	model->tet_volume.clear();
	model->obj_tetmap.clear();
	model->obj_param.clear();
	// get volume
	for(i=0;i<model->tn; i++)
	{
		tetTash tash = model->t[i];
		vector3f p0, p1, p2, p3;
		p0 = model->exam_pos[0][tash.v0];
		p1 = model->exam_pos[0][tash.v1];
		p2 = model->exam_pos[0][tash.v2];
		p3 = model->exam_pos[0][tash.v3];
		double doc = cal_triangle_area(p0,p1,p2);
		double hei = cal_tetra_height(p0,p1,p2,p3);
		model->tet_volume.push_back(doc * hei / 3.0);
	}

	// mapping point to tet
	int cache_j = -1;
	for(i=0; i<model->model->vv.size(); i++)
	{
		if(i%1000 == 0)printf("%d\n",i);
		vector3f pp = model->model->vv[i];
//		pp.x = (pp.x - model->trans_x) / model->scale_xx * 0.95 - model->trans_before_x;
//		pp.y = (pp.y - model->trans_y) / model->scale_xx * 0.95 - model->trans_before_y;
//		pp.z = (pp.z - model->trans_z) / model->scale_xx * 0.95 - model->trans_before_z;
//		printf("   - %d pp : %lf %lf %lf\n",i,VECTOR3F_PRINTF(pp));
		int px, py, pz;
		int qx, qy, qz;
		int rx, ry, rz;
		px = pp.x * ten;
		py = pp.y * ten;
		pz = pp.z * ten;
		if(px < 0)px=0;if(px>=ten)px=ten-1;
		if(py < 0)py=0;if(py>=ten)py=ten-1;
		if(pz < 0)pz=0;if(pz>=ten)pz=ten-1;
		for(set<int>::iterator k=rrxx[px][py][pz].begin(); k!=rrxx[px][py][pz].end(); k++)
		{
			cache_j = *k;
		if(cache_j>=0 && cache_j < model->tn)
		{
			j=cache_j;
			double sum = 0;
			tetTash tash = model->t[cache_j];
			vector3f p0, p1, p2, p3;
			p0 = model->exam_pos[0][tash.v0];
			p1 = model->exam_pos[0][tash.v1];
			p2 = model->exam_pos[0][tash.v2];
			p3 = model->exam_pos[0][tash.v3];
			p0.x = (p0.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p0.y = (p0.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p0.z = (p0.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p1.x = (p1.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p1.y = (p1.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p1.z = (p1.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p2.x = (p2.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p2.y = (p2.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p2.z = (p2.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p3.x = (p3.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p3.y = (p3.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p3.z = (p3.z + model->trans_before_z) * model->scale_xx + model->trans_z;

			matrix33f A;
			vector3f vres;
			p0 -= p3;p1 -= p3; p2 -= p3;
			A.set(p0.x, p0.y, p0.z,
				p1.x, p1.y, p1.z,
				p2.x, p2.y, p2.z);
			A = A.transpose();
			bool inv_res = A.inverse();
			if(!inv_res)
			{
//				printf("inverse fail at Tash!! %d\n",i);
			}
			else
			{
			vres = A * (pp-p3);
//			printf("%d : %lf %lf %lf\n",j,VECTOR3F_PRINTF(vres));
//			printf("%.15lf %.15lf  %.15lf   %.15lf\n",res,model->tet_volume[j],res / model->tet_volume[j], model->tet_volume[j] / res);
			if(vres.x >=0 && vres.x < 1 && vres.y >= 0 && vres.y < 1 && vres.z >= 0 && vres.z < 1 && vres.x + vres.y + vres.z < 1)
			{
				j = cache_j;
				goto CACHE_SUCCESS;
			}
			else
			{
//			printf("cache fail : %d %d\n",i,j);
//			printf("%lf %lf %lf\n",VECTOR3F_PRINTF(vres));
			}
			}
		}
		}
//		printf("cache fail : %d  %d %d %d  %d %d\n",i,px,py,pz,rrxx[px][py][pz].size(), zzxx[px][py][pz].size());
		int tj=0;
		double tt=-1;
		for(set<int>::iterator k = zzxx[px][py][pz].begin(); k != zzxx[px][py][pz].end(); k++)
		{
			j = *k;
		{
			double sum = 0;
			tetTash tash = model->t[j];
			vector3f p0, p1, p2, p3;
			p0 = model->exam_pos[0][tash.v0];
			p1 = model->exam_pos[0][tash.v1];
			p2 = model->exam_pos[0][tash.v2];
			p3 = model->exam_pos[0][tash.v3];
			p0.x = (p0.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p0.y = (p0.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p0.z = (p0.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p1.x = (p1.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p1.y = (p1.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p1.z = (p1.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p2.x = (p2.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p2.y = (p2.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p2.z = (p2.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p3.x = (p3.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p3.y = (p3.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p3.z = (p3.z + model->trans_before_z) * model->scale_xx + model->trans_z;;

			matrix33f A;
			vector3f vres;
			p0 -= p3;p1 -= p3; p2 -= p3;
			A.set(p0.x, p0.y, p0.z,
				p1.x, p1.y, p1.z,
				p2.x, p2.y, p2.z);
		A = A.transpose();
			bool inv_res = A.inverse();
			if(!inv_res) {
			//	printf("inverse fail at Tash!! %d\n",i);
			}
			else
			{
			vres = A * (pp-p3);
//			printf(" - %d %d %lf %lf %lf %lf\n",j,tj,tt,VECTOR3F_PRINTF(vres));
//			printf("%d  :  %lf %lf %lf %lf\n",j,VECTOR3F_PRINTF(vres),1-vres.x-vres.y-vres.z);
//			printf("%.15lf %.15lf  %.15lf   %.15lf\n",res,model->tet_volume[j],res / model->tet_volume[j], model->tet_volume[j] / res);
			if(vres.x >=0 && vres.x < 1 && vres.y >= 0 && vres.y < 1 && vres.z >= 0 && vres.z < 1 && vres.x + vres.y + vres.z < 1)
			{
				tj = j;
				tt = pp.dist(p3);
				break;
			}
			else if(tt < 0 || tt > pp.dist(p3))
			{
				tt = pp.dist(p3);
				tj = j;
			}
			}
//			if(res / model->tet_volume[j] < 3 && model->tet_volume[j] / res < 3)
//				break;
		}
		}
		j = tj;
		if(tt < 0)
		{
			printf("Something wrong! %d\n",i);
		}
		CACHE_SUCCESS:
		if(j == model->tn)
		{
			printf("Something wrong! %d\n",i);
			j--;
		}
//		else printf("Something well? %d\n",j);
		tetTash tash = model->t[j];
		vector3f p0, p1, p2, p3;
		p0 = model->exam_pos[0][tash.v0];
		p1 = model->exam_pos[0][tash.v1];
		p2 = model->exam_pos[0][tash.v2];
		p3 = model->exam_pos[0][tash.v3];
			p0.x = (p0.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p0.y = (p0.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p0.z = (p0.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p1.x = (p1.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p1.y = (p1.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p1.z = (p1.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p2.x = (p2.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p2.y = (p2.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p2.z = (p2.z + model->trans_before_z) * model->scale_xx + model->trans_z;
			p3.x = (p3.x + model->trans_before_x) * model->scale_xx + model->trans_x;
			p3.y = (p3.y + model->trans_before_y) * model->scale_xx + model->trans_y;
			p3.z = (p3.z + model->trans_before_z) * model->scale_xx + model->trans_z;
		matrix33f A;
		vector3f res;
		p0 -= p3;p1 -= p3; p2 -= p3;
		A.set(p0.x, p0.y, p0.z,
			p1.x, p1.y, p1.z,
			p2.x, p2.y, p2.z);
		A = A.transpose();
		bool inv_res = A.inverse();
		if(!inv_res) printf("inverse fail at connect Tash!! %d\n",i);
		else
		{
		res = A * (pp-p3);
		model->obj_tetmap.push_back(j);
		model->obj_param.push_back(res);
		rrxx[px][py][pz].insert(j);
		}
		//printf("%d -> %d\n",i,j);
		//printf("%lf %lf %lf %lf\n",VECTOR3F_PRINTF(res),1-res.x-res.y-res.z);
	}
}
void Object::resize(tetObject *tmodel)
{
	int i, j, k;
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	min_x = max_x = tmodel->exam_pos[0][0].x;
	min_y = max_y = tmodel->exam_pos[0][0].y;
	min_z = max_z = tmodel->exam_pos[0][0].z;
	for(i=1;i<tmodel->exam_pos[0].size();i++)
	{
//		if(tmodel->exam_pos[0][i].z < -38)continue;
		min_x = min(min_x,tmodel->exam_pos[0][i].x);
		min_y = min(min_y,tmodel->exam_pos[0][i].y);
		min_z = min(min_z,tmodel->exam_pos[0][i].z);
		max_x = max(max_x,tmodel->exam_pos[0][i].x);
		max_y = max(max_y,tmodel->exam_pos[0][i].y);
		max_z = max(max_z,tmodel->exam_pos[0][i].z);
	}
	double maxx = max(max(max_x-min_x,max_y-min_y),max_z-min_z);
	tmodel->scale_xx = 1 / maxx;
	tmodel->trans_before_x = -min_x;
	tmodel->trans_before_y = -min_y;
	tmodel->trans_before_z = -min_z;
	tmodel->trans_x = (maxx - (max_x-min_x))/maxx/2;
	tmodel->trans_y = (maxx - (max_y-min_y))/maxx/2;
	tmodel->trans_z = (maxx - (max_z-min_z))/maxx/2;
}
void Object::set_example_rate(double *_rate)
{
	if(example_rate == NULL)
	{
		example_rate = new double[en];
		example_rate0 = new double[en];
	}
	int i, j;
	double sum = 0;
	for(i=0;i<en; i++)
	{
		example_rate[i] = _rate[i];
		if(example_rate[i] < 0) example_rate[i] = 0;
		sum += example_rate[i];
	}
	if(sum <= 0.0000000001) return;
	for(i=0;i<en; i++)
	{
		example_rate[i] /= sum;
	}
	for(i=0;i<en; i++)
	{
		example_rate0[i] = example_rate[i];
	}
	
	example_rate_exist = true;
}
void Object::change_example_rate_test(int a1, int a2, int a3, double arg)
{
	int i, j;
	if(arg < 0) arg = 0;
	if(arg > 1) arg = 1;
	if(a1 < 0)
	{	example_rate[0] += arg*(0 - example_rate[0])/2;
		example_rate[1] += arg*(0 - example_rate[1])/2;
		example_rate[2] += arg*(1 - example_rate[2])/2;
	}
	else
	{
		example_rate[0] += arg*(example_rate0[a1] - example_rate[0])/2;
		example_rate[1] += arg*(example_rate0[a2] - example_rate[1])/2;
		example_rate[2] += arg*(example_rate0[a3] - example_rate[2])/2;
	}
}
static double tztz;
void Object::inil_by_xml(tetObject *_cmodel, tetObject *_tmodel)
{
#ifdef TETRA_WORK
	inil_done = false;
	attack = false;
	attack_index = -1;
	if(cmodel == NULL)
	{
		if(_cmodel == NULL)
		{
		cmodel = new tetObject();
		cmodel->getModel("resource/model/Simple_Happy_Buddha.tet");
		cmodel->cornering();
		cmodel->tash_reorder();
		cmodel->normalize();
		cmodel->model->resize();
		resize(cmodel);
		if(cmodel->is_mesh) connect(cmodel);
		}
		else
		{
			cmodel = new tetObject(_cmodel);
		if(cmodel->is_mesh) connect(cmodel);
		}
		cn = cmodel->pn;
	}
	target_cheat = 0.00;
	wahaha = 0;
	if(tmodel == NULL)
	{
		if(_tmodel == NULL)
		{
		tmodel = new tetObject();
//		tmodel->getModel("resource/cube5.xml");	//??
		tmodel->getModel("resource/cube4.xml");	//twist
//		tmodel->getModel("resource/cube1.xml");
//		tmodel->getModel("resource/new_cube6.xml");
//		tmodel->getModel("resource/model/Round_Cube.tet");
		tmodel->cornering();
		tmodel->tash_reorder();
		tmodel->normalize();
//		tmodel->resize();
		resize(tmodel);
		}
		else
		{
			tmodel = new tetObject(_tmodel);
		}
		tmodel->en = 4;
		for(int i=1; i<tmodel->en; i++)
			tmodel->exam_pos[i].resize(tmodel->pn);
		fn = 50;
		/* 
		// twist
		for(int i=0; i<tmodel->pn;i++)
		{
			vector3f pp = tmodel->exam_pos[0][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			double dx = pp.x * cos(pp.y * M_PI) - pp.z * sin(pp.y * M_PI);
			double dy = pp.x * sin(pp.y * M_PI) + pp.z * cos(pp.y * M_PI);
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = dx + 0.5;
			pp.z = dy + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			tmodel->exam_pos[1][i] = pp;
		}
		*/
		// head hunter
		for(int i=0; i<tmodel->pn;i++)
		{
			vector3f pp = tmodel->exam_pos[0][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			pp.y = (1 - cos(pp.y * M_PI)) / 2;
			double dx = pp.x * (1 - cos(pp.y * M_PI)*0.9);
			double dy = pp.z * (1 - cos(pp.y * M_PI)*0.9);
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = dx + 0.5;
			pp.z = dy + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			tmodel->exam_pos[1][i] = pp;
		}
		//head hunter animation
		for(int f=0; f<fn; f++)
		{
			vector<vector3f> frame_pos;
			frame_pos.resize(tmodel->pn);
			for(int i=0; i<tmodel->pn; i++)
			{
			vector3f pp = tmodel->exam_pos[1][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			double dx = pp.x + cos(M_PI * 2 * f / fn) * 0.5;
			double dy = pp.z;
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = dx + 0.5;
			pp.z = dy + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			frame_pos[i] = pp;
			}
			tmodel->exam_pos_animated[1].push_back(frame_pos);
		}
		// sleep
		/*
		for(int i=0; i<tmodel->pn;i++)
		{
			vector3f pp = tmodel->exam_pos[0][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			double dx = pp.z - 0.5;
			double dy = pp.y - 0.5;
			double w = sqrt(dx*dx+dy*dy);
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.z = pp.z + 0.2*cos(pp.y * M_PI);
			pp.y *= 1.0;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			tmodel->exam_pos[2][i] = pp;
		}
		*/
		// body hunter
		for(int i=0; i<tmodel->pn;i++)
		{
			vector3f pp = tmodel->exam_pos[0][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			pp.y = sqrt(pp.y);
			double dx = pp.x * (1 + cos(pp.y * M_PI)*0.9);
			double dy = pp.z * (1 + cos(pp.y * M_PI)*0.9);
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = dx + 0.5;
			pp.z = dy + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			tmodel->exam_pos[2][i] = pp;
		}
		//body hunter animation
		for(int f=0; f<fn; f++)
		{
			vector<vector3f> frame_pos;
			frame_pos.resize(tmodel->pn);
			for(int i=0; i<tmodel->pn; i++)
			{
			vector3f pp = tmodel->exam_pos[2][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			double dx = pp.x + cos(M_PI * 2 * f / fn) * 0.5;
			double dy = pp.z;
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = dx + 0.5;
			pp.z = dy + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			frame_pos[i] = pp;
			}
			tmodel->exam_pos_animated[2].push_back(frame_pos);
		}
		// fat hunter
		for(int i=0; i<tmodel->pn;i++)
		{
			vector3f pp = tmodel->exam_pos[0][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			double dd = sqrt(pp.x * pp.x + pp.z * pp.z);
			pp.y = pp.y * (1 + dd * 40)/11;
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = pp.x + 0.5;
			pp.z = pp.z + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			tmodel->exam_pos[3][i] = pp;
		}
		//fat hunter animation
		for(int f=0; f<fn; f++)
		{
			vector<vector3f> frame_pos;
			frame_pos.resize(tmodel->pn);
			for(int i=0; i<tmodel->pn; i++)
			{
			vector3f pp = tmodel->exam_pos[3][i];
			pp.x = (pp.x + tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
			pp.x -= 0.5;
			pp.z -= 0.5;
			double dx = pp.x + cos(M_PI * 2 * f / fn) * 0.5;
			double dy = pp.z;
//			pp.x = (2-2*(w*w*1.8 + 0.4))*pp.x;
			pp.x = dx + 0.5;
			pp.z = dy + 0.5;
			pp.x = (pp.x - tmodel->trans_x) / tmodel->scale_xx - tmodel->trans_before_x;
			pp.y = (pp.y - tmodel->trans_y) / tmodel->scale_xx - tmodel->trans_before_y;
			pp.z = (pp.z - tmodel->trans_z) / tmodel->scale_xx - tmodel->trans_before_z;
			frame_pos[i] = pp;
			}
			tmodel->exam_pos_animated[3].push_back(frame_pos);
		}
		target_cheat = 0.0;
		/*
		trans_before_x = tmodel->trans_before_x;
		trans_before_y = tmodel->trans_before_y;
		trans_before_z = tmodel->trans_before_z;
		
		trans_x = tmodel->trans_x;
		trans_y = tmodel->trans_y;
		trans_z = tmodel->trans_z;
		
		scale_xx = tmodel->scale_xx;
		*/
		tmodel->scale_matrix[0] = 1.47;
		tmodel->scale_matrix[5] = 1.07;
		tmodel->scale_matrix[10] = 1.07;

		tmodel->scale_matrix[3] = - tmodel->scale_matrix[0] / 2 + 0.5;
		tmodel->scale_matrix[7] = - tmodel->scale_matrix[5] / 2 + 0.5;
		tmodel->scale_matrix[11] = - tmodel->scale_matrix[10] / 2 + 0.5;
	}
	/*
	// vmodel = child of tmodel
	if(vmodel == NULL)
	{
		vmodel = new tetObject();
		vmodel->getModel("resource/cube5.xml");	//??
		vmodel->cornering();
		vmodel->tash_reorder();
		vmodel->normalize();
		resize(vmodel);

		
		vmodel->scale_matrix[0] = 1.07;
		vmodel->scale_matrix[5] = 1.07;
		vmodel->scale_matrix[10] = 1.07;
	}*/
	int i, j, k, l;
	p.clear();
	x0.clear();
	g.clear();
	n = tmodel->pn;
	en = tmodel->en-1;
	int ti = 0;
	double tz =tmodel->exam_pos[0][0].y;
	for(i=0;i<n;i++)
	{
		Point pp;
		pp.x = tmodel->exam_pos[0][i];
//		pp.x += vector3f(0, -0.2/scale_xx, 0);
		pp.x0 = pp.x;
		pp.v = vector3f(0,0,0);
		pp.m = 10000.0 / (tmodel->neighbor[i].size() + 1);
		pp.inv_m = 1.0 / pp.m;

		p.push_back(pp);
		x0.push_back(pp.x);
		g.push_back(pp.x);
		pp.a = vector3f();
		if(tz > pp.x.y)
		{
			tz = pp.x.y;
			ti = i;
		}

	}
	tztz = tz;
	
	/*
	for(i=0;i<n;i++)
	{
		p[i].x0.y -= tz;
		p[i].x.y -= tz;
		tmodel->exam_pos[0][i].y -= tz;
	}
	trans_before_y = 0;
	tz=0;
	tztz=tz;
	*/

	for(j=0;j<en;j++)
	{
		x1[j].clear();
		for(i=0;i<n;i++)
		{
			x1[j].push_back(tmodel->exam_pos[j+1][i]);
		}
		x2[j].clear();
		for(int f=0; f<fn; f++)
		{
			vector<vector3f> x2_t;
			for(i=0; i<n; i++)
			{
				x2_t.push_back(tmodel->exam_pos_animated[j+1][f][i]);
			}
			x2[j].push_back(x2_t);
		}
	}

	c.clear();
	c.resize(tmodel->pn);
	for(i=0;i<tmodel->pn;i++)
	{
		Cluster cc;
		cc.vv = tmodel->neighbor[i];
		cc.vv.push_back(i);
		cc.n = cc.vv.size();
		cc.getCenter(this);
		cc.precalUstaticMatrix(this);
		c[i]=cc;
	}
	precalExampleUmode();
	precalAtA();
	vector<int> gcnt;
	gcnt.resize(n);
	for(i=0;i<c.size();i++)
		for(j=0;j<c[i].vv.size();j++)
			gcnt[c[i].vv[j]]++;
	for(i=0;i<n;i++)
	{
		if(gcnt[i]) p[i].drawable =  true;
		else p[i].drawable = false;
	}
	
	// volume constraint
	
	// model constraint
	double *p_m = new double[n];
	int obn = c.size();
	for(i=0;i<n;i++)
		p_m[i] = 0;
	/*
	for(i=0;i<n;i++)
	{
		vector3f pp,qq;
		qq = p[i].x;
		double tmd;
		int ti;
		
		qq.x = (qq.x+tmodel->trans_before_x)*tmodel->scale_xx + tmodel->trans_x;
		qq.y = (qq.y+tmodel->trans_before_y)*tmodel->scale_xx + tmodel->trans_y;
		qq.z = (qq.z+tmodel->trans_before_z)*tmodel->scale_xx + tmodel->trans_z;
		ti = -1;
		for(j=0;j<cn;j++)
		{
			if(cp_index[j] >=0)continue;
			pp = cmodel->exam_pos[0][j];
			pp.x = (pp.x+cmodel->trans_before_x)*cmodel->scale_xx + cmodel->trans_x;
			pp.y = (pp.y+cmodel->trans_before_y)*cmodel->scale_xx + cmodel->trans_y;
			pp.z = (pp.z+cmodel->trans_before_z)*cmodel->scale_xx + cmodel->trans_z;
			if(ti < 0 || tmd > pp.dist(qq))
			{
				ti = j;
				tmd = pp.dist(qq);
			}
		}
		if(ti >= 0)
		{
		cp_index[ti] = i;
		}
	}
	*/
	
	int ten = 1;
	int df = 1;
	if(xxzz==NULL)
	{
		xxzz = new set<int>**[ten];
		for(i=0;i<ten;i++)
		{
			xxzz[i] = new set<int>*[ten];
			for(j=0;j<ten;j++)
			{
				xxzz[i][j] = new set<int>[ten];
			}
		}
		for(i=0;i<ten;i++)
			for(j=0;j<ten;j++)
				for(k=0;k<ten;k++)
					xxzz[i][j][k].clear();
	/// inil_xxzz
	for(j=0;j<tmodel->tn;j++)
	{
		int qx,qy,qz;
		int rx,ry,rz;
		int tj=j;
			double sum = 0;
			tetTash tash = tmodel->t[j];
			vector3f p0, p1, p2, p3;
			p0 = tmodel->exam_pos[0][tash.v0];
			p1 = tmodel->exam_pos[0][tash.v1];
			p2 = tmodel->exam_pos[0][tash.v2];
			p3 = tmodel->exam_pos[0][tash.v3];
			p0.x = (p0.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
			p0.y = (p0.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
			p0.z = (p0.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
			p1.x = (p1.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
			p1.y = (p1.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
			p1.z = (p1.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
			p2.x = (p2.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
			p2.y = (p2.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
			p2.z = (p2.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
			p3.x = (p3.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
			p3.y = (p3.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
			p3.z = (p3.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
			rx = qx = p0.x * ten;
			ry = qy = p0.y * ten;
			rz = qz = p0.z * ten;
			if(qx > p1.x * ten)qx = p1.x * ten; if(rx < p1.x * ten) rx = p1.x * ten;
			if(qy > p1.y * ten)qy = p1.y * ten; if(ry < p1.y * ten) ry = p1.y * ten;
			if(qz > p1.z * ten)qz = p1.z * ten; if(rz < p1.z * ten) rz = p1.z * ten;
			if(qx > p2.x * ten)qx = p2.x * ten; if(rx < p2.x * ten) rx = p2.x * ten;
			if(qy > p2.y * ten)qy = p2.y * ten; if(ry < p2.y * ten) ry = p2.y * ten;
			if(qz > p2.z * ten)qz = p2.z * ten; if(rz < p2.z * ten) rz = p2.z * ten;
			if(qx > p3.x * ten)qx = p3.x * ten; if(rx < p3.x * ten) rx = p3.x * ten;
			if(qy > p3.y * ten)qy = p3.y * ten; if(ry < p3.y * ten) ry = p3.y * ten;
			if(qz > p3.z * ten)qz = p3.z * ten; if(rz < p3.z * ten) rz = p3.z * ten;
			for(int i= qx-df; i<=rx+df;i++)
			for(int j= qy-df; j<=ry+df;j++)
			for(int k= qz-df; k<=rz+df;k++)
				if(i>=0&&i<ten)
				if(j>=0&&j<ten)
				if(k>=0&&k<ten)
				xxzz[i][j][k].insert(tj);
	}
	}
	
	cp.clear();
	cp_index.clear();
	cp_param.clear();
	for(i=0;i<cn;i++)
	{
		cp_index.push_back(-1);
		cp_param.push_back(vector3f());
	}
	for(i=0;i<cn;i++)
	{
		Point pp,qq;
		pp.x = cmodel->exam_pos[0][i];
		pp.x0 = pp.x;
		pp.v = vector3f(0);
		pp.a = 0;
		pp.m = 10000.0 / (cmodel->neighbor[i].size() + 1);
		pp.inv_m = 1.0 / pp.m;
		pp.drawable = true;

		cp.push_back(pp);
		int ti, tj;
		double tmd;
		pp.x.x = (pp.x.x+cmodel->trans_before_x)*cmodel->scale_xx + cmodel->trans_x;
		pp.x.y = (pp.x.y+cmodel->trans_before_y)*cmodel->scale_xx + cmodel->trans_y;
		pp.x.z = (pp.x.z+cmodel->trans_before_z)*cmodel->scale_xx + cmodel->trans_z;
		ti = cp_index[i];
		if(ti < 0){
			ti = 0;
			tmd = -1;
			int px,py,pz;
			px = pp.x.x * ten;
			py = pp.x.y * ten;
			pz = pp.x.z * ten;
			if(px < 0)px=0;if(px>=ten)px=ten-1;
			if(py < 0)py=0;if(py>=ten)py=ten-1;
			if(pz < 0)pz=0;if(pz>=ten)pz=ten-1;
			
				vector3f vres;
			int vr_cnt=0;
			for(set<int>::iterator k = xxzz[px][py][pz].begin(); k != xxzz[px][py][pz].end(); k++)
			{
				j = *k;
				vr_cnt++;
			{
				tetTash tash = tmodel->t[j];
				vector3f p0, p1, p2, p3;
				vector3f tmpv;
				p0 = tmodel->exam_pos[0][tash.v0];
				p1 = tmodel->exam_pos[0][tash.v1];
				p2 = tmodel->exam_pos[0][tash.v2];
				p3 = tmodel->exam_pos[0][tash.v3];
					p0.x = (p0.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p0.y = (p0.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p0.z = (p0.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
					p1.x = (p1.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p1.y = (p1.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p1.z = (p1.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
					p2.x = (p2.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p2.y = (p2.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p2.z = (p2.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
					p3.x = (p3.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p3.y = (p3.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p3.z = (p3.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p0.x + tmodel->scale_matrix[0*4+1] * p0.y + tmodel->scale_matrix[0*4+2] * p0.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p0.x + tmodel->scale_matrix[1*4+1] * p0.y + tmodel->scale_matrix[1*4+2] * p0.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p0.x + tmodel->scale_matrix[2*4+1] * p0.y + tmodel->scale_matrix[2*4+2] * p0.z + tmodel->scale_matrix[2*4+3];
				p0 = tmpv;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p1.x + tmodel->scale_matrix[0*4+1] * p1.y + tmodel->scale_matrix[0*4+2] * p1.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p1.x + tmodel->scale_matrix[1*4+1] * p1.y + tmodel->scale_matrix[1*4+2] * p1.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p1.x + tmodel->scale_matrix[2*4+1] * p1.y + tmodel->scale_matrix[2*4+2] * p1.z + tmodel->scale_matrix[2*4+3];
				p1 = tmpv;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p2.x + tmodel->scale_matrix[0*4+1] * p2.y + tmodel->scale_matrix[0*4+2] * p2.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p2.x + tmodel->scale_matrix[1*4+1] * p2.y + tmodel->scale_matrix[1*4+2] * p2.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p2.x + tmodel->scale_matrix[2*4+1] * p2.y + tmodel->scale_matrix[2*4+2] * p2.z + tmodel->scale_matrix[2*4+3];
				p2 = tmpv;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p3.x + tmodel->scale_matrix[0*4+1] * p3.y + tmodel->scale_matrix[0*4+2] * p3.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p3.x + tmodel->scale_matrix[1*4+1] * p3.y + tmodel->scale_matrix[1*4+2] * p3.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p3.x + tmodel->scale_matrix[2*4+1] * p3.y + tmodel->scale_matrix[2*4+2] * p3.z + tmodel->scale_matrix[2*4+3];
				p3 = tmpv;
				matrix33f A;
				p0 -= p3;p1 -= p3; p2 -= p3;
				A.set(p0.x, p0.y, p0.z,
					p1.x, p1.y, p1.z,
					p2.x, p2.y, p2.z);
				A = A.transpose();
				bool inv_res = A.inverse();
				if(!inv_res) printf("inverse fail at tmodel Tash!! %d\n",i);
				vres = A * (pp.x-p3);
				if(vres.x >=-E && vres.x < 1+E && vres.y >= -E && vres.y < 1+E && vres.z >= -E && vres.z < 1+E && vres.x + vres.y + vres.z < 1+E)
				{
					ti = j;
					cp_param[i] = vres;
					tmd = pp.x.dist(p3);
					break;
				}
				else if(tmd < 0 || tmd > pp.x.dist(p3))
				{
					tmd = pp.x.dist(p3);
					cp_param[i] = vres;
					ti = j;
				}
			}
			}
			if(tmd < 0)
			{
				printf("hull : %d\n",i);
				printf("%d %d   %lf %lf %lf\n",ti,vr_cnt,VECTOR3F_PRINTF(vres));
				ti = j;
				cp_param[i] = vres;
			}
		}
		cp_index[i] = ti;
	}

	delete[] p_m;
	inil_done = true;
	printf("inil_done\n");
#else
	inil();
#endif
}
double Constraint::volume_constraint(Object *me)
{
	int i;
	double res=0;
	res += (me->p[volume_index[0]].x%me->p[volume_index[1]].x)*me->p[volume_index[2]].x;
	res += (me->p[volume_index[0]].x%me->p[volume_index[2]].x)*me->p[volume_index[3]].x;
	res += (me->p[volume_index[0]].x%me->p[volume_index[3]].x)*me->p[volume_index[1]].x;
	res += (me->p[volume_index[1]].x%me->p[volume_index[3]].x)*me->p[volume_index[2]].x;
//	printf("%lf %lf\n",res,volume_init);
	return res - volume_init;
}
void Constraint::volume_del_constraint(Object *me)
{
	int i, j;
	volume_del[0] = me->p[volume_index[1]].x % me->p[volume_index[2]].x + me->p[volume_index[2]].x % me->p[volume_index[3]].x + me->p[volume_index[3]].x % me->p[volume_index[1]].x;
	volume_del[1] = me->p[volume_index[0]].x % me->p[volume_index[3]].x + me->p[volume_index[2]].x % me->p[volume_index[1]].x + me->p[volume_index[3]].x % me->p[volume_index[2]].x;
	volume_del[2] = me->p[volume_index[1]].x % me->p[volume_index[3]].x + me->p[volume_index[3]].x % me->p[volume_index[0]].x + me->p[volume_index[0]].x % me->p[volume_index[1]].x;
	volume_del[3] = me->p[volume_index[0]].x % me->p[volume_index[2]].x + me->p[volume_index[1]].x % me->p[volume_index[0]].x + me->p[volume_index[2]].x % me->p[volume_index[1]].x;
}
void Constraint::setting(tetTash c, Object *me)
{
	int i, j;
	volume_index.clear();
	volume_del.clear();
	volume_index.push_back(c.v1);
	volume_index.push_back(c.v2);
	volume_index.push_back(c.v3);
	volume_index.push_back(c.v0);
	volume_del.push_back(vector3f());
	volume_del.push_back(vector3f());
	volume_del.push_back(vector3f());
	volume_del.push_back(vector3f());
	double res=0;
	res += (me->p[volume_index[0]].x%me->p[volume_index[1]].x)*me->p[volume_index[2]].x;
	res += (me->p[volume_index[0]].x%me->p[volume_index[2]].x)*me->p[volume_index[3]].x;
	res += (me->p[volume_index[0]].x%me->p[volume_index[3]].x)*me->p[volume_index[1]].x;
	res += (me->p[volume_index[1]].x%me->p[volume_index[3]].x)*me->p[volume_index[2]].x;
	volume_init = res;
}

void Object::simul()
{
	if(!inil_done) return;
	//Collision
	int i, j, k;

	for(i=0;i<n;i++)
	{
		p[i].a += vector3f(0, -98/ tmodel->scale_xx, 0);
		if(attack)
		{
		if(wahaha<1)
		{
			p[i].a += vector3f(0, (1000) / tmodel->scale_xx , 0 / tmodel->scale_xx);
			target_cheat = wahaha/2;
		}
		}
		else
		{
			wahaha = 0;
			target_cheat = 0;
		}
		p[i].a -= 0.01*p[i].v / dt;
		p[i].Integrate(tmodel, dt);
	}

	for(i=0;i<n;i++)
	{
		g[i]=0;
	}
	for(i=0;i<n;i++)
		p[i].x_tmp = p[i].x;
	for(int iter = 0; iter < 3; iter++)
	{
		ShapeMatching_target();
		for(i=0;i<n;i++)
			p[i].x = g[i];
	}
	for(i=0;i<n;i++)
		p[i].x = p[i].x_tmp;

	for(i=0;i<n;i++)
	{
		vector3f pv = p[i].v;
		vector3f pa = p[i].a;
//		p[i].v += p[i].a * dt / p[i].m;
		p[i].v = 1.0 * (g[i] - p[i].x) / dt;
		p[i].a = -0.01 * p[i].v / dt;
		p[i].Integrate(tmodel,dt);
		p[i].v += pv;
		p[i].a += pa;
//		p[i].Integrate(dt);
	}
	dampingGlobal();

	
	for(i=0;i<n;i++)
		if(p[i].x0.y < tztz+0.2/tmodel->scale_xx)
		{
			p[i].x = p[i].x0 + vector3f(0.0*sin(wahaha)/tmodel->scale_xx,0,0);
			p[i].v = vector3f();
		}
		
	wahaha += 0.1;
	for(i=0;i<cn; i++)
	{
		cp[i].x=0;
	}
//	if(wahaha < 15)
//	target_cheat += 1.0/600;
//	if(frame_num > 200)
//		target_cheat -= 0.05;
	int obn = c.size();
	for(i=0;i<cn;i++)
	{
		j = cp_index[i];
		tetTash tash = tmodel->t[j];
		vector3f p0, p1, p2, p3;
		vector3f tmpv;
		p0 = p[tash.v0].x;
		p1 = p[tash.v1].x;
		p2 = p[tash.v2].x;
		p3 = p[tash.v3].x;
					p0.x = (p0.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p0.y = (p0.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p0.z = (p0.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
					p1.x = (p1.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p1.y = (p1.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p1.z = (p1.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
					p2.x = (p2.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p2.y = (p2.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p2.z = (p2.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
					p3.x = (p3.x + tmodel->trans_before_x) * tmodel->scale_xx + tmodel->trans_x;
					p3.y = (p3.y + tmodel->trans_before_y) * tmodel->scale_xx + tmodel->trans_y;
					p3.z = (p3.z + tmodel->trans_before_z) * tmodel->scale_xx + tmodel->trans_z;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p0.x + tmodel->scale_matrix[0*4+1] * p0.y + tmodel->scale_matrix[0*4+2] * p0.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p0.x + tmodel->scale_matrix[1*4+1] * p0.y + tmodel->scale_matrix[1*4+2] * p0.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p0.x + tmodel->scale_matrix[2*4+1] * p0.y + tmodel->scale_matrix[2*4+2] * p0.z + tmodel->scale_matrix[2*4+3];
				p0 = tmpv;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p1.x + tmodel->scale_matrix[0*4+1] * p1.y + tmodel->scale_matrix[0*4+2] * p1.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p1.x + tmodel->scale_matrix[1*4+1] * p1.y + tmodel->scale_matrix[1*4+2] * p1.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p1.x + tmodel->scale_matrix[2*4+1] * p1.y + tmodel->scale_matrix[2*4+2] * p1.z + tmodel->scale_matrix[2*4+3];
				p1 = tmpv;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p2.x + tmodel->scale_matrix[0*4+1] * p2.y + tmodel->scale_matrix[0*4+2] * p2.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p2.x + tmodel->scale_matrix[1*4+1] * p2.y + tmodel->scale_matrix[1*4+2] * p2.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p2.x + tmodel->scale_matrix[2*4+1] * p2.y + tmodel->scale_matrix[2*4+2] * p2.z + tmodel->scale_matrix[2*4+3];
				p2 = tmpv;
				tmpv.x = tmodel->scale_matrix[0*4+0] * p3.x + tmodel->scale_matrix[0*4+1] * p3.y + tmodel->scale_matrix[0*4+2] * p3.z + tmodel->scale_matrix[0*4+3];
				tmpv.y = tmodel->scale_matrix[1*4+0] * p3.x + tmodel->scale_matrix[1*4+1] * p3.y + tmodel->scale_matrix[1*4+2] * p3.z + tmodel->scale_matrix[1*4+3];
				tmpv.z = tmodel->scale_matrix[2*4+0] * p3.x + tmodel->scale_matrix[2*4+1] * p3.y + tmodel->scale_matrix[2*4+2] * p3.z + tmodel->scale_matrix[2*4+3];
				p3 = tmpv;
		vector3f res;
		res += cp_param[i].x * p0;
		res += cp_param[i].y * p1;
		res += cp_param[i].z * p2;
		res += (1-cp_param[i].x-cp_param[i].y-cp_param[i].z) * p3;
		res.x = (res.x - cmodel->trans_x) / cmodel->scale_xx - cmodel->trans_before_x;
		res.y = (res.y - cmodel->trans_y) / cmodel->scale_xx - cmodel->trans_before_y;
		res.z = (res.z - cmodel->trans_z) / cmodel->scale_xx - cmodel->trans_before_z;
		cp[i].x = res;
	}

}

// reference: Position Based Dynamics [Muller07]
void Object::dampingGlobal() {
    // get xcm, vcm, and sum of mass.
    vector3f xcm = vector3f(0.0f, 0.0f, 0.0f);
    vector3f vcm = vector3f(0.0f, 0.0f, 0.0f);
    double m = 0.0f;
	int i, j, k;
    for (i=0;i<n;i++) {
        xcm += p[i].x * p[i].m;
        vcm += p[i].v * p[i].m;
        m += p[i].m;
    }
    xcm *= 1.0f / m;
    vcm *= 1.0f / m;
    
    // get L, I, Ri
    vector3f L = vector3f(0.0f, 0.0f, 0.0f);
    matrix33f I = matrix33f(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	for(i=0;i<n;i++)
	{
        vector3f Ri = p[i].x - xcm;
        vector3f tmp = Ri % (p[i].m * p[i].v);
        //        Y::Vector3f tmp = Ri.CrossProduct(p->m * p->v);
        L += tmp;
        matrix33f RTilde = matrix33f(0.0f, - Ri.z, Ri.y, 
                                             Ri.z, 0.0f, - Ri.x,
                                             - Ri.y, Ri.x, 0.0f);
        matrix33f tmpM = RTilde * RTilde.transpose();
        I += tmpM * p[i].m;
    }
    
    // get w
	I.inverse();
    vector3f w = L * I;
    
    // apply the damping
	for(i=0;i<n;i++)
	{
        vector3f Ri = p[i].x - xcm;
        vector3f DVi = vcm + (w % Ri) - p[i].v;
        p[i].v += 0.03 * DVi;
    }
}
void force_normalize(tetObject *p, vector<Point> &q)
{
	int i, j, k, l;
	if(p->is_mesh) return;
	j = p->model->vv.size();
	l = p->model->obj.size();
	vector3f *normal_mean = new vector3f[j];
	int *normal_mean_num = new int[j];

	for(i=0;i<j;i++)
	{
		normal_mean[i] = vector3f();
		normal_mean_num[i] = 0;
	}
	for(int ii = 0; ii<l; ii++)
	{
		k = p->model->obj[ii]->f.size();
		for(i=0;i<k;i++)
		{
			objFace * face = p->model->obj[ii]->f[i];
			face->vn.clear();
			face->vn.resize(face->vv.size());
			for(int jj=1; jj+1<face->vv.size();jj++)
			{
				{
				vector3f n1;
				vector3f p01 = q[face->vv[jj]].x - q[face->vv[0]].x;
				vector3f p12 = q[face->vv[jj+1]].x - q[face->vv[jj]].x;
				n1 = p01 % p12;
				n1.normalize();

				int id;
				id = face->vv[0];
				normal_mean[id] += n1;
				normal_mean_num[id] ++;
				face->vn[0] = id;
				id = face->vv[jj];
				normal_mean[id] += n1;
				normal_mean_num[id] ++;
				face->vn[jj] = id;
				id = face->vv[jj+1];
				normal_mean[id] += n1;
				normal_mean_num[id] ++;
				face->vn[jj+1] = id;
				}
			}
		}
	}
	for(i=0;i<j;i++)
	{
		if(normal_mean_num[i]>0)
			normal_mean[i] /= normal_mean_num[i];
		p->model->vn[i] = normal_mean[i];
	}
	
	delete [] normal_mean;
	delete [] normal_mean_num;
}
void Object::draw_cmodel(vector3f offset)
{
	int i, j, k;
	if(!inil_done) return;
	if(cmodel == NULL) return;
	
	glPushMatrix();
	glTranslated(offset.x,offset.y,offset.z);
	glTranslated(cmodel->trans_x, cmodel->trans_y, cmodel->trans_z);
	glScaled(cmodel->scale_xx,cmodel->scale_xx,cmodel->scale_xx);
	glTranslated(cmodel->trans_before_x, cmodel->trans_before_y, cmodel->trans_before_z);
#ifdef TETRA_WORK
	glPolygonMode(GL_FRONT_AND_BACK,show_type_1);
	if(example_rate_exist)
	{
		glColor3d(example_rate[0], example_rate[1], example_rate[2]);
	}
	else glColor3d(0.8, 0.2, 0.1);
	if(cmodel->is_mesh)
	{
		//refresh vertex point
		for(i=0;i<cmodel->model->vv.size();i++)
		{
			int ti = cmodel->obj_tetmap[i];
			vector3f param = cmodel->obj_param[i];
			vector3f p0,p1,p2,p3;
			tetTash tash = cmodel->t[ti];
			p0 = cp[tash.v0].x;
			p1 = cp[tash.v1].x;
			p2 = cp[tash.v2].x;
			p3 = cp[tash.v3].x;
			vector3f res;
			res += param.x * p0;
			res += param.y * p1;
			res += param.z * p2;
			res += (1-param.x-param.y-param.z) * p3;
			cmodel->model->vv[i] = res;
		}
		for(i=0;i<cmodel->model->obj.size();i++)
		{
			objObject *obj = cmodel->model->obj[i];
			for(j=0;j<obj->f.size();j++)
			{
				objFace *face = obj->f[j];
				for(k=1;k+1<face->vv.size();k++)
				{
					glBegin(GL_TRIANGLES);
						if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[0]].x,cmodel->model->vn[face->vn[0]].y,cmodel->model->vn[face->vn[0]].z);
						glVertex3d(cmodel->model->vv[face->vv[0]].x,cmodel->model->vv[face->vv[0]].y,cmodel->model->vv[face->vv[0]].z);
						if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[k]].x,cmodel->model->vn[face->vn[k]].y,cmodel->model->vn[face->vn[k]].z);
						glVertex3d(cmodel->model->vv[face->vv[k]].x,cmodel->model->vv[face->vv[k]].y,cmodel->model->vv[face->vv[k]].z);
						if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[k+1]].x,cmodel->model->vn[face->vn[k+1]].y,cmodel->model->vn[face->vn[k+1]].z);
						glVertex3d(cmodel->model->vv[face->vv[k+1]].x,cmodel->model->vv[face->vv[k+1]].y,cmodel->model->vv[face->vv[k+1]].z);
					glEnd();
				}
			}
		}
	}
	else
	{
		force_normalize(cmodel, cp);
		for(i=0;i<cmodel->model->obj.size();i++)
		{
			objObject *obj = cmodel->model->obj[i];
			for(j=0;j<obj->f.size();j++)
			{
				objFace *face = obj->f[j];
				for(k=1;k+1<face->vv.size();k++)
				{
					glBegin(GL_TRIANGLES);
						if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[0]].x,cmodel->model->vn[face->vn[0]].y,cmodel->model->vn[face->vn[0]].z);
						glVertex3d(cp[face->vv[0]].x.x,cp[face->vv[0]].x.y,cp[face->vv[0]].x.z);
						if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[k]].x,cmodel->model->vn[face->vn[k]].y,cmodel->model->vn[face->vn[k]].z);
						glVertex3d(cp[face->vv[k]].x.x,cp[face->vv[k]].x.y,cp[face->vv[k]].x.z);
						if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[k+1]].x,cmodel->model->vn[face->vn[k+1]].y,cmodel->model->vn[face->vn[k+1]].z);
						glVertex3d(cp[face->vv[k+1]].x.x,cp[face->vv[k+1]].x.y,cp[face->vv[k+1]].x.z);
					glEnd();
				}
			}
		}
		/*
		for(i=0; i<cmodel->tn;i++)
		{
			tetTash tash = cmodel->t[i];
		
				int p1,p2,p3;
				p1 = tash.v0;p2 = tash.v1;p3 = tash.v2;
				if(cmodel->corner[p1] && cmodel->corner[p2] && cmodel->corner[p3])
				{
				glBegin(GL_POLYGON);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p1].x,cmodel->model->vn[p1].y,cmodel->model->vn[p1].z);
					if(cp[p1].drawable)glVertex3d(cp[p1].x.x,cp[p1].x.y,cp[p1].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p2].x,cmodel->model->vn[p2].y,cmodel->model->vn[p2].z);
					if(cp[p2].drawable)glVertex3d(cp[p2].x.x,cp[p2].x.y,cp[p2].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p3].x,cmodel->model->vn[p3].y,cmodel->model->vn[p3].z);
					if(cp[p3].drawable)glVertex3d(cp[p3].x.x,cp[p3].x.y,cp[p3].x.z);
				glEnd();
				}
				p1 = tash.v0;p2 = tash.v2;p3 = tash.v3;
				if(cmodel->corner[p1] && cmodel->corner[p2] && cmodel->corner[p3])
				{
				glBegin(GL_POLYGON);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p1].x,cmodel->model->vn[p1].y,cmodel->model->vn[p1].z);
					if(cp[p1].drawable)glVertex3d(cp[p1].x.x,cp[p1].x.y,cp[p1].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p2].x,cmodel->model->vn[p2].y,cmodel->model->vn[p2].z);
					if(cp[p2].drawable)glVertex3d(cp[p2].x.x,cp[p2].x.y,cp[p2].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p3].x,cmodel->model->vn[p3].y,cmodel->model->vn[p3].z);
					if(cp[p3].drawable)glVertex3d(cp[p3].x.x,cp[p3].x.y,cp[p3].x.z);
				glEnd();
				}
				p1 = tash.v0;p2 = tash.v3;p3 = tash.v1;
				if(cmodel->corner[p1] && cmodel->corner[p2] && cmodel->corner[p3])
				{
				glBegin(GL_POLYGON);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p1].x,cmodel->model->vn[p1].y,cmodel->model->vn[p1].z);
					if(cp[p1].drawable)glVertex3d(cp[p1].x.x,cp[p1].x.y,cp[p1].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p2].x,cmodel->model->vn[p2].y,cmodel->model->vn[p2].z);
					if(cp[p2].drawable)glVertex3d(cp[p2].x.x,cp[p2].x.y,cp[p2].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p3].x,cmodel->model->vn[p3].y,cmodel->model->vn[p3].z);
					if(cp[p3].drawable)glVertex3d(cp[p3].x.x,cp[p3].x.y,cp[p3].x.z);
				glEnd(); 
				}
				p1 = tash.v1;p2 = tash.v3;p3 = tash.v2;
				if(cmodel->corner[p1] && cmodel->corner[p2] && cmodel->corner[p3])
				{
				glBegin(GL_POLYGON);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p1].x,cmodel->model->vn[p1].y,cmodel->model->vn[p1].z);
					if(cp[p1].drawable)glVertex3d(cp[p1].x.x,cp[p1].x.y,cp[p1].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p2].x,cmodel->model->vn[p2].y,cmodel->model->vn[p2].z);
					if(cp[p2].drawable)glVertex3d(cp[p2].x.x,cp[p2].x.y,cp[p2].x.z);
					if(cmodel->model->vn.size()>0 )glNormal3d(cmodel->model->vn[p3].x,cmodel->model->vn[p3].y,cmodel->model->vn[p3].z);
					if(cp[p3].drawable)glVertex3d(cp[p3].x.x,cp[p3].x.y,cp[p3].x.z);
				glEnd();
				}
		}
		*/
	}
	glPopMatrix();
#else
	glPolygonMode(GL_FRONT_AND_BACK,show_type_1);
	glColor3d(0.8, 0.1, 0.2);
	for(i=0; i<model->obj.size();i++)
	{
		objObject *obj = model->obj[i];
		for(j=0; j<obj->f.size();j++)
		{
			objFace *face = obj->f[j];
			glBegin(GL_POLYGON);
			for(k=0; k<face->vv.size();k++)
			{
				if(model->vn.size()>0)
				glNormal3d(model->vn[face->vn[k]].x,model->vn[face->vn[k]].y,model->vn[face->vn[k]].z);
//				glVertex3d(model->vv[face->vv[k]].x,model->vv[face->vv[k]].y,model->vv[face->vv[k]].z);
				glVertex3d(p[face->vv[k]].x.x,p[face->vv[k]].x.y,p[face->vv[k]].x.z);
			}
			glEnd();
		}
	}
#endif

}
void Object::draw_tmodel()
{
	int i, j, k;
	if(!inil_done) return;
	if(tmodel == NULL) return;
	
	glPushMatrix();
	
	glTranslated(0,0,0);
	glTranslated(tmodel->trans_x, tmodel->trans_y, tmodel->trans_z);
	glScaled(tmodel->scale_xx,tmodel->scale_xx,tmodel->scale_xx);
	glTranslated(tmodel->trans_before_x, tmodel->trans_before_y, tmodel->trans_before_z);
	
#ifdef TETRA_WORK
	glPolygonMode(GL_FRONT_AND_BACK,show_type_1);
	glColor3d(0.1, 0.8, 0.2);
	
	force_normalize(tmodel, p);
	for(i=0; i<tmodel->tn;i++)
	{
		tetTash tash = tmodel->t[i];
		
			int p1,p2,p3;
			p1 = tash.v0;p2 = tash.v1;p3 = tash.v2;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(p[p1].x.x,p[p1].x.y,p[p1].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(p[p2].x.x,p[p2].x.y,p[p2].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(p[p3].x.x,p[p3].x.y,p[p3].x.z);
			glEnd();
			}
			p1 = tash.v0;p2 = tash.v2;p3 = tash.v3;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(p[p1].x.x,p[p1].x.y,p[p1].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(p[p2].x.x,p[p2].x.y,p[p2].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(p[p3].x.x,p[p3].x.y,p[p3].x.z);
			glEnd();
			}
			p1 = tash.v0;p2 = tash.v3;p3 = tash.v1;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(p[p1].x.x,p[p1].x.y,p[p1].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(p[p2].x.x,p[p2].x.y,p[p2].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(p[p3].x.x,p[p3].x.y,p[p3].x.z);
			glEnd(); 
			}
			p1 = tash.v1;p2 = tash.v3;p3 = tash.v2;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(p[p1].x.x,p[p1].x.y,p[p1].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(p[p2].x.x,p[p2].x.y,p[p2].x.z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(p[p3].x.x,p[p3].x.y,p[p3].x.z);
			glEnd();
			}
	}
	glPopMatrix();

#else
	glColor3d(0.1, 0.2, 0.8);
	for(i=0; i<target[target_num]->obj.size();i++)
	{
		objObject *obj = target[target_num]->obj[i];
		for(j=0; j<obj->f.size();j++)
		{
			objFace *face = obj->f[j];
			glBegin(GL_POLYGON);
			for(k=0; k<face->vv.size();k++)
			{
				if(model->vn.size()>0)
				glNormal3d(target[target_num]->vn[face->vn[k]].x,target[target_num]->vn[face->vn[k]].y,target[target_num]->vn[face->vn[k]].z);
				glVertex3d(target[target_num]->vv[face->vv[k]].x,target[target_num]->vv[face->vv[k]].y,target[target_num]->vv[face->vv[k]].z + 1);
//				glVertex3d(p[face->vv[k]].x.x,p[face->vv[k]].x.y,p[face->vv[k]].x.z);
			}
			glEnd();
		}
	}
#endif
}
void Object::draw_example()
{
	int i, j, k;
	glPushMatrix();
	
	glTranslated(0,0,0);
	glTranslated(tmodel->trans_x, tmodel->trans_y, tmodel->trans_z);
	glScaled(tmodel->scale_xx,tmodel->scale_xx,tmodel->scale_xx);
	glTranslated(tmodel->trans_before_x, tmodel->trans_before_y, tmodel->trans_before_z);
	
#ifdef TETRA_WORK
	glPolygonMode(GL_FRONT_AND_BACK,show_type_1);
	
//	for(k=0;k<en;k++)
	k = 0;
	{
		for(i=0; i<tmodel->model->vv.size(); i++)
			tmodel->model->vv[i] = tmodel->exam_pos[k+1][i];
		tmodel->normalize();
	glColor3d(0.1, 0.2, 0.8);
	for(i=0; i<tmodel->tn;i++)
	{
		tetTash tash = tmodel->t[i];
		
			int p1,p2,p3;
			p1 = tash.v0;p2 = tash.v1;p3 = tash.v2;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(x1[k][p1].x,x1[k][p1].y,x1[k][p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(x1[k][p2].x,x1[k][p2].y,x1[k][p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(x1[k][p3].x,x1[k][p3].y,x1[k][p3].z);
			glEnd();
			}
			p1 = tash.v0;p2 = tash.v1;p3 = tash.v3;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(x1[k][p1].x,x1[k][p1].y,x1[k][p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(x1[k][p2].x,x1[k][p2].y,x1[k][p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(x1[k][p3].x,x1[k][p3].y,x1[k][p3].z);
			glEnd();
			}
			p1 = tash.v0;p2 = tash.v2;p3 = tash.v3;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(x1[k][p1].x,x1[k][p1].y,x1[k][p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(x1[k][p2].x,x1[k][p2].y,x1[k][p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(x1[k][p3].x,x1[k][p3].y,x1[k][p3].z);
			glEnd(); 
			}
			p1 = tash.v1;p2 = tash.v2;p3 = tash.v3;
			if(tmodel->corner[p1] && tmodel->corner[p2] && tmodel->corner[p3])
			{
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(x1[k][p1].x,x1[k][p1].y,x1[k][p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(x1[k][p2].x,x1[k][p2].y,x1[k][p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(x1[k][p3].x,x1[k][p3].y,x1[k][p3].z);
			glEnd();
			}
	}
	}
	glPopMatrix();
#else
	
	if(show_type_2 != GL_POINT)
	{
	glPolygonMode(GL_FRONT_AND_BACK,show_type_2);
	glColor4d(0.1, 0.8, 0.2, 0.5);
	for(i=0; i<model->obj.size();i++)
	{
		objObject *obj = model->obj[i];
		for(j=0; j<obj->f.size();j++)
		{
			objFace *face = obj->f[j];
			glBegin(GL_POLYGON);
			for(k=0; k<face->vv.size();k++)
			{
				if(model->vn.size()>0)
				glNormal3d(model->vn[face->vn[k]].x,model->vn[face->vn[k]].y,model->vn[face->vn[k]].z);
				glVertex3d(g[face->vv[k]].x,g[face->vv[k]].y,g[face->vv[k]].z);
			}
			glEnd();
		}
	}
	}
#endif
}


void Object::draw_cmodel_origin()
{	
	int i, j, k;
	if(!inil_done) return;
	if(cmodel == NULL) return;
	
	glPushMatrix();
//	glTranslated(offset.x,offset.y,offset.z);
	glTranslated(cmodel->trans_x, cmodel->trans_y, cmodel->trans_z);
	glScaled(cmodel->scale_xx,cmodel->scale_xx,cmodel->scale_xx);
	glTranslated(cmodel->trans_before_x, cmodel->trans_before_y, cmodel->trans_before_z);
	glPolygonMode(GL_FRONT_AND_BACK,show_type_1);
	glColor3d(0.8, 0.1, 0.2);
	for(i=0; i<cn; i++)
		cmodel->model->vv[i] = cmodel->exam_pos[0][i];
	cmodel->normalize();
	for(i=0;i<cmodel->model->obj.size();i++)
	{
		objObject *obj = cmodel->model->obj[i];
		for(j=0;j<obj->f.size();j++)
		{
			objFace *face = obj->f[j];
			for(k=1;k+1<face->vv.size();k++)
			{
				glBegin(GL_TRIANGLES);
					if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[0]].x,cmodel->model->vn[face->vn[0]].y,cmodel->model->vn[face->vn[0]].z);
					glVertex3d(cmodel->model->vv[face->vv[0]].x,cmodel->model->vv[face->vv[0]].y,cmodel->model->vv[face->vv[0]].z);
					if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[k]].x,cmodel->model->vn[face->vn[k]].y,cmodel->model->vn[face->vn[k]].z);
					glVertex3d(cmodel->model->vv[face->vv[k]].x,cmodel->model->vv[face->vv[k]].y,cmodel->model->vv[face->vv[k]].z);
					if(cmodel->model->vn.size() > 0) glNormal3d(cmodel->model->vn[face->vn[k+1]].x,cmodel->model->vn[face->vn[k+1]].y,cmodel->model->vn[face->vn[k+1]].z);
					glVertex3d(cmodel->model->vv[face->vv[k+1]].x,cmodel->model->vv[face->vv[k+1]].y,cmodel->model->vv[face->vv[k+1]].z);
				glEnd();
			}
		}
	}
	glPopMatrix();
}
void Object::draw_tmodel_origin()
{	
	int i, j, k;
	if(!inil_done) return;
	if(tmodel == NULL) return;
	
	glPushMatrix();
//	glTranslated(offset.x,offset.y,offset.z);
	glTranslated(tmodel->trans_x, tmodel->trans_y, tmodel->trans_z);
	glScaled(tmodel->scale_xx,tmodel->scale_xx,tmodel->scale_xx);
	glTranslated(tmodel->trans_before_x, tmodel->trans_before_y, tmodel->trans_before_z);
	glPolygonMode(GL_FRONT_AND_BACK,show_type_1);
	glColor3d(0.1, 0.8, 0.2);
	for(i=0; i<tmodel->model->vv.size(); i++)
		tmodel->model->vv[i] = tmodel->exam_pos[0][i];
	tmodel->normalize();
	for(i=0;i<tmodel->model->obj.size();i++)
	{
		objObject *obj = tmodel->model->obj[i];
		for(j=0;j<obj->f.size();j++)
		{
			objFace *face = obj->f[j];
			for(k=1;k+1<face->vv.size();k++)
			{
				glBegin(GL_TRIANGLES);
					if(tmodel->model->vn.size() > 0) glNormal3d(tmodel->model->vn[face->vn[0]].x,tmodel->model->vn[face->vn[0]].y,tmodel->model->vn[face->vn[0]].z);
					glVertex3d(tmodel->model->vv[face->vv[0]].x,tmodel->model->vv[face->vv[0]].y,tmodel->model->vv[face->vv[0]].z);
					if(tmodel->model->vn.size() > 0) glNormal3d(tmodel->model->vn[face->vn[k]].x,tmodel->model->vn[face->vn[k]].y,tmodel->model->vn[face->vn[k]].z);
					glVertex3d(tmodel->model->vv[face->vv[k]].x,tmodel->model->vv[face->vv[k]].y,tmodel->model->vv[face->vv[k]].z);
					if(tmodel->model->vn.size() > 0) glNormal3d(tmodel->model->vn[face->vn[k+1]].x,tmodel->model->vn[face->vn[k+1]].y,tmodel->model->vn[face->vn[k+1]].z);
					glVertex3d(tmodel->model->vv[face->vv[k+1]].x,tmodel->model->vv[face->vv[k+1]].y,tmodel->model->vv[face->vv[k+1]].z);
				glEnd();
			}
		}
	}
	glPopMatrix();
}
void Object::draw()
{
	int i, j, k;
	if(!inil_done) return;
///	printf(" %lf %lf %lf\n",VECTOR3F_PRINTF(cp[0].x));
//	printf("%lf %lf %lf\n",cmodel->trans_x, cmodel->trans_y, cmodel->trans_z);
//	printf("%lf %lf %lf\n",cmodel->trans_before_x, cmodel->trans_before_y, cmodel->trans_before_z);
//	printf("%lf\n",cmodel->scale_xx);
	
	/*
	if(show_type_2 != GL_POINT)
	{
	glPolygonMode(GL_FRONT_AND_BACK,show_type_2);
	glColor4d(0.1, 0.8, 0.2, 0.5);
	for(i=0; i<tmodel->tn;i++)
	{
		tetTash tash = tmodel->t[i];
		
			int p1,p2,p3;
			p1 = tash.v0;p2 = tash.v1;p3 = tash.v2;
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(g[p1].x,g[p1].y,g[p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(g[p2].x,g[p2].y,g[p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(g[p3].x,g[p3].y,g[p3].z);
			glEnd();
			p1 = tash.v0;p2 = tash.v2;p3 = tash.v3;
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(g[p1].x,g[p1].y,g[p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(g[p2].x,g[p2].y,g[p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(g[p3].x,g[p3].y,g[p3].z);
			glEnd();
			p1 = tash.v0;p2 = tash.v3;p3 = tash.v1;
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(g[p1].x,g[p1].y,g[p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(g[p2].x,g[p2].y,g[p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(g[p3].x,g[p3].y,g[p3].z);
			glEnd(); 
			p1 = tash.v1;p2 = tash.v3;p3 = tash.v2;
			glBegin(GL_POLYGON);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p1].x,tmodel->model->vn[p1].y,tmodel->model->vn[p1].z);
				if(p[p1].drawable)glVertex3d(g[p1].x,g[p1].y,g[p1].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p2].x,tmodel->model->vn[p2].y,tmodel->model->vn[p2].z);
				if(p[p2].drawable)glVertex3d(g[p2].x,g[p2].y,g[p2].z);
				if(tmodel->model->vn.size()>0 )glNormal3d(tmodel->model->vn[p3].x,tmodel->model->vn[p3].y,tmodel->model->vn[p3].z);
				if(p[p3].drawable)glVertex3d(g[p3].x,g[p3].y,g[p3].z);
			glEnd();
	}
	}
	*/
//#else

//#endif
}
void Object::ShapeMatching()
{

	int i, j, k;
	// get center of mass
	cmp = vector3f();
	for(i=0;i<n;i++)
	{
		cmp += p[i].x;
	}
	cmp /= n;

	// make A
	MatrixXd A(3,3);
	MatrixXd MPQ(3,3), MQQ(3,3);
	MatrixXd S(3,3);
	MPQ.setZero();
	MQQ.setZero();
	S.setZero();
	for(i=0;i<n;i++)
	{
		vector3f vp,vq;
		vp = p[i].x - cmp;
		vq = x0[i] - cmp0;
		MPQ(0,0) += p[i].m * vp.x * vq.x;
		MPQ(0,1) += p[i].m * vp.x * vq.y;
		MPQ(0,2) += p[i].m * vp.x * vq.z;
		MPQ(1,0) += p[i].m * vp.y * vq.x;
		MPQ(1,1) += p[i].m * vp.y * vq.y;
		MPQ(1,2) += p[i].m * vp.y * vq.z;
		MPQ(2,0) += p[i].m * vp.z * vq.x;
		MPQ(2,1) += p[i].m * vp.z * vq.y;
		MPQ(2,2) += p[i].m * vp.z * vq.z;
		
		MQQ(0,0) += p[i].m * vq.x * vq.x;
		MQQ(0,1) += p[i].m * vq.x * vq.y;
		MQQ(0,2) += p[i].m * vq.x * vq.z;
		MQQ(1,0) += p[i].m * vq.y * vq.x;
		MQQ(1,1) += p[i].m * vq.y * vq.y;
		MQQ(1,2) += p[i].m * vq.y * vq.z;
		MQQ(2,0) += p[i].m * vq.z * vq.x;
		MQQ(2,1) += p[i].m * vq.z * vq.y;
		MQQ(2,2) += p[i].m * vq.z * vq.z;
	}

	S = MPQ.transpose() * MPQ;

	SelfAdjointEigenSolver<MatrixXd> eigensolver(S);
	if(eigensolver.info() != Success)
		printf("hull\n");
	else
	{
//		cout << "eigen value  : \n" << eigensolver.eigenvalues() << endl;
//		cout << "eigen vector : \n" << eigensolver.eigenvectors() << endl;

		MatrixXd Y(3,3);
		Y.setZero();
		Y(0,0) = sqrt(eigensolver.eigenvalues()(0));
		Y(1,1) = sqrt(eigensolver.eigenvalues()(1));
		Y(2,2) = sqrt(eigensolver.eigenvalues()(2));
		MatrixXd Q = eigensolver.eigenvectors();
//		cout << S << endl;
//		cout << Q * Y * Q.transpose() << endl;
		S = Q * Y * Q.transpose();

		A = MPQ * S.inverse();
//		cout << "Rotation Matrix : \n" << A <<endl;
		for(i=0;i<n;i++)
		{
			vector3f vq = x0[i] - cmp0;
			g[i].x = A(0,0) * vq.x + A(0,1) * vq.y + A(0,2) * vq.z + cmp.x;
			g[i].y = A(1,0) * vq.x + A(1,1) * vq.y + A(1,2) * vq.z + cmp.y;
			g[i].z = A(2,0) * vq.x + A(2,1) * vq.y + A(2,2) * vq.z + cmp.z;
		}
	}

}
void Cluster::getCenter(Object *me)
{
	int i, j, k;
	double sum_of_mass = 0;
	cmp = vector3f();
	cmp0 = vector3f();
	for(i=0; i<n;i++)
	{
		cmp0 += me->p[vv[i]].x0 * me->p[vv[i]].m;
		cmp += me->p[vv[i]].x * me->p[vv[i]].m;
		sum_of_mass += me->p[vv[i]].m;
	}
	cmp  /= sum_of_mass;
	cmp0 /= sum_of_mass;
}
matrix33f Cluster::getMomentMatrix(Object *me)
{
	matrix33f res;
	matrix33f mcc0t;

	int i, j, k;
	double sum_of_mass = 0;

	res.setZero();
	for(i=0; i<n;i++)
	{
		sum_of_mass += me->p[vv[i]].m;

		res += me->p[vv[i]].mxx0t;
	}


	mcc0t.set(
		sum_of_mass * cmp.x * cmp0.x, sum_of_mass * cmp.x * cmp0.y,sum_of_mass * cmp.x * cmp0.z,
		sum_of_mass * cmp.y * cmp0.x, sum_of_mass * cmp.y * cmp0.y,sum_of_mass * cmp.y * cmp0.z,
		sum_of_mass * cmp.z * cmp0.x, sum_of_mass * cmp.z * cmp0.y,sum_of_mass * cmp.z * cmp0.z
		);
	/*
	printf("Get Momenturm\n");
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			printf("%lf ",res.value[i][j]);
	printf("\n");
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			printf("%lf ",mcc0t.value[i][j]);
	printf("\n");
	*/
	res = res - mcc0t;

	return res;
}
void Cluster::precalUstaticMatrix(Object *me)
{
	matrix33f Fs;
	int i, j, k;
	double sum_of_mass = 0;

	getCenter(me);

//	printf("%d\n",n);
	for(i=0; i<n; i++)
	{
		vector3f qi = me->p[vv[i]].x0 - cmp0;
		matrix33f tmp(
			qi.x * qi.x, qi.x * qi.y, qi.x * qi.z,
			qi.y * qi.x, qi.y * qi.y, qi.y * qi.z,
			qi.z * qi.x, qi.z * qi.y, qi.z * qi.z
			);
//		printf("%lf\n%lf %lf %lf\n",me->p[vv[i]].m, VECTOR3F_PRINTF(qi));
		Fs += tmp * me->p[vv[i]].m;
	}
//	for(i=0;i<3;i++)for(j=0;j<3;j++)printf("%lf ",Fs.value[i][j]);
//	printf("\n");
	non_fs = false;
	if(!Fs.inverse())
	{
		printf("Fs inverse fail!\n");
		non_fs = true;
	}
	else
		Ustatic = Fs;
}
// TODO :: here is hard_coding
void Object::precalExampleUmode()
{
	vector<vector3f> pre_x;
	int i, j, k;
	int obn = c.size();

	for(i=0;i<n;i++)
		pre_x.push_back(p[i].x);
	// FOR REST
	for(i=0;i<n;i++)
		p[i].x = x0[i];
	// update particle
	for(i=0; i<n;i++)
	{
		vector3f t = p[i].x * p[i].m;
		p[i].mxx0t.set(
			t.x * p[i].x0.x, t.x * p[i].x0.y,t.x * p[i].x0.z,
			t.y * p[i].x0.x, t.y * p[i].x0.y,t.y * p[i].x0.z,
			t.z * p[i].x0.x, t.z * p[i].x0.y,t.z * p[i].x0.z
			);
	}
	
	for(i=0;i<obn;i++)
	{
		c[i].getCenter(this);
	//	printf("%d : %lf %lf %lf  %lf %lf %lf\n",i,c[i].cmp.x,c[i].cmp.y,c[i].cmp.z,c[i].cmp0.x,c[i].cmp0.y,c[i].cmp0.z);
	}
	// cluster ShapeMatching
	
	for(i=0;i<obn; i++)
	{
		if(c[i].non_fs) continue;
		matrix33f Fd = c[i].getMomentMatrix(this);
		matrix33f F = Fd * c[i].Ustatic;
		matrix33f Umode = F.transpose() * F;

		// sqrt(Umode)
		MatrixXd S(3,3);
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			{
				S(i,j) = Umode.value[i][j];
			}
		SelfAdjointEigenSolver<MatrixXd> eigensolver(S);
		if(eigensolver.info() != Success)
			printf("hull - precalExampleUmode\n");
		else
		{
			MatrixXd Y(3,3);
			Y.setZero();
			Y(0,0) = eigensolver.eigenvalues()(0);
			Y(1,1) = eigensolver.eigenvalues()(1);
			Y(2,2) = eigensolver.eigenvalues()(2);
			if(Y(0,0) < 0) Y(0,0) = E; else	Y(0,0) = sqrt(Y(0,0));
			if(Y(1,1) < 0) Y(1,1) = E; else	Y(1,1) = sqrt(Y(1,1));
			if(Y(2,2) < 0) Y(2,2) = E; else	Y(2,2) = sqrt(Y(2,2));
			MatrixXd Q = eigensolver.eigenvectors();
			S = Q * Y * Q.transpose();
			
			for(int ii=0;ii<3;ii++)
				for(int j=0;j<3;j++)
				{
					Umode.value[ii][j] = S(ii,j);
				}
			c[i].Umode[0][0] = Umode.value[0][0];
			c[i].Umode[0][1] = Umode.value[1][1];
			c[i].Umode[0][2] = Umode.value[2][2];
			c[i].Umode[0][3] = Umode.value[0][1];
			c[i].Umode[0][4] = Umode.value[0][2];
			c[i].Umode[0][5] = Umode.value[1][2];
		}
	}
	
#ifdef TETRA_WORK
	for(k=0; k<en;k++)
	{
		// BECOM EXAMPLE
		for(i=0;i<n;i++)
			p[i].x = x1[k][i];
		// update particle
		for(i=0; i<n;i++)
		{
			vector3f t = p[i].x * p[i].m;
//			printf("%lf %lf %lf  %lf %lf %lf  %lf\n",t.x, t.y, t.z, p[i].x0.x, p[i].x0.y, p[i].x0.z, p[i].m);
			p[i].mxx0t.set(
				t.x * p[i].x0.x, t.x * p[i].x0.y,t.x * p[i].x0.z,
				t.y * p[i].x0.x, t.y * p[i].x0.y,t.y * p[i].x0.z,
				t.z * p[i].x0.x, t.z * p[i].x0.y,t.z * p[i].x0.z
				);
		}
		// get center_of_mass;
		for(i=0;i<obn;i++)
		{
			c[i].getCenter(this);
		}
		// cluster ShapeMatching
		for(i=0;i<obn; i++)
		{
			matrix33f Fd = c[i].getMomentMatrix(this);
			matrix33f F = Fd * c[i].Ustatic;
			matrix33f Umode = F.transpose() * F;

			// sqrt(Umode)
			MatrixXd S(3,3);
			for(int i=0;i<3;i++)
				for(int j=0;j<3;j++)
					S(i,j) = Umode.value[i][j];
			SelfAdjointEigenSolver<MatrixXd> eigensolver(S);
			if(eigensolver.info() != Success)
				printf("hull - precalExampleUmode %d\n",k);
			else
			{
				MatrixXd Y(3,3);
				Y.setZero();
				Y(0,0) = eigensolver.eigenvalues()(0);
				Y(1,1) = eigensolver.eigenvalues()(1);
				Y(2,2) = eigensolver.eigenvalues()(2);
				if(Y(0,0) < 0) Y(0,0) = E; else	Y(0,0) = sqrt(Y(0,0));
				if(Y(1,1) < 0) Y(1,1) = E; else	Y(1,1) = sqrt(Y(1,1));
				if(Y(2,2) < 0) Y(2,2) = E; else	Y(2,2) = sqrt(Y(2,2));
				MatrixXd Q = eigensolver.eigenvectors();
				S = Q * Y * Q.transpose();
			
				for(int ii=0;ii<3;ii++)
					for(int j=0;j<3;j++)
					{
						Umode.value[ii][j] = S(ii,j);
					}
				double ud = Umode.determinant();
				if(ud < 0) ud = -ud;
				Umode /= pow(ud, 1.0/3.0);
				c[i].Umode[k+1][0] = Umode.value[0][0];
				c[i].Umode[k+1][1] = Umode.value[1][1];
				c[i].Umode[k+1][2] = Umode.value[2][2];
				c[i].Umode[k+1][3] = Umode.value[0][1];
				c[i].Umode[k+1][4] = Umode.value[0][2];
				c[i].Umode[k+1][5] = Umode.value[1][2];
			}
		}
		/*
		for(k=0;k<=en;k++,printf("%d :\n",k))
			for(i=0;i<n;i++, printf("\n"))
				for(j=0;j<6;j++)
					printf("%lf ",c[i].Umode[k][j]);
		printf("\n");
		*/
	}
#else
	// FOR EXAMPLE
	for(i=0;i<n;i++)
		p[i].x = x1[target_num][i];
	// update particle
	for(i=0; i<n;i++)
	{
		vector3f t = p[i].x * p[i].m;
		p[i].mxx0t.set(
			t.x * p[i].x0.x, t.x * p[i].x0.y,t.x * p[i].x0.z,
			t.y * p[i].x0.x, t.y * p[i].x0.y,t.y * p[i].x0.z,
			t.z * p[i].x0.x, t.z * p[i].x0.y,t.z * p[i].x0.z
			);
	}
		for(i=0;i<obn;i++)
		{
			c[i].cmp = vector3f();
			c[i].cmp0 = vector3f();
			double sum_of_mass = 0;
			for(j=0; j<c[i].n;j++)
			{
				c[i].cmp0 += x0[c[i].vv[j]] * p[c[i].vv[j]].m;
				c[i].cmp += p[c[i].vv[j]].x * p[c[i].vv[j]].m;
				sum_of_mass += p[c[i].vv[j]].m;
			}
			c[i].cmp  /= sum_of_mass;
			c[i].cmp0 /= sum_of_mass;
		}

	// cluster ShapeMatching
	
	for(i=0;i<obn; i++)
	{
		matrix33f Fd = c[i].getMomentMatrix(this);
		matrix33f F = Fd * c[i].Ustatic;
		matrix33f Umode = F.transpose() * F;

		// sqrt(Umode)
		MatrixXd S(3,3);
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				S(i,j) = Umode.value[i][j];
		SelfAdjointEigenSolver<MatrixXd> eigensolver(S);
		if(eigensolver.info() != Success)
			printf("hull\n");
		else
		{
			MatrixXd Y(3,3);
			Y.setZero();
			Y(0,0) = eigensolver.eigenvalues()(0);
			Y(1,1) = eigensolver.eigenvalues()(1);
			Y(2,2) = eigensolver.eigenvalues()(2);
			if(Y(0,0) < E) Y(0,0) = E; else	Y(0,0) = 1 / sqrt(Y(0,0));
			if(Y(1,1) < E) Y(1,1) = E; else	Y(1,1) = 1 / sqrt(Y(1,1));
			if(Y(2,2) < E) Y(2,2) = E; else	Y(2,2) = 1 / sqrt(Y(2,2));
			MatrixXd Q = eigensolver.eigenvectors();
			S = Q * Y * Q.transpose();
			
			for(int ii=0;ii<3;ii++)
				for(int j=0;j<3;j++)
				{
					Umode.value[ii][j] = S(ii,j);
				}
			matrix33f R = F * Umode;
			Umode = Umode.inverse();
			if(R.determinant() < 0)
			{
				R *= -1;
				Umode *= -1;
			}
			c[i].Umode[1][0] = Umode.value[0][0];
			c[i].Umode[1][1] = Umode.value[0][1];
			c[i].Umode[1][2] = Umode.value[0][2];
			c[i].Umode[1][3] = Umode.value[1][1];
			c[i].Umode[1][4] = Umode.value[1][2];
			c[i].Umode[1][5] = Umode.value[2][2];
		}
	}
#endif
	for(i=0;i<n;i++)
		p[i].x = pre_x[i];
}
void Object::precalAtA()
{
	int en = 1;
#ifdef TETRA_WORK
	en = this->en;
#endif
	if(en==0)return;
	AtA_inv = MatrixXd(en,en);
	MatrixXd AtA_global(en,en);
	AtA_inv.setZero();
	AtA_global.setZero();
	int i, j, k;
	int obn = c.size();
	for(i=0;i<obn; i++)
	{
		if(c[i].non_fs) continue;
		c[i].AtA_inv = MatrixXd(en,en);
		MatrixXd AtA_local(en,en);
		AtA_local.setZero();
		int ii, jj;
		for(ii=0;ii<en;ii++)
		{
			double Ui[6];
			for(j=0;j<6;j++)Ui[j] = c[i].Umode[ii+1][j] - c[i].Umode[0][j];
			for(jj=ii;jj<en;jj++)
			{
				double Uj[6];
				for(j=0;j<6;j++)Uj[j] = c[i].Umode[jj+1][j] - c[i].Umode[0][j];
				double dd=0;
				for(j=0;j<6;j++) dd += Ui[j] * Uj[j];
				AtA_local(ii,jj) = dd;
				if(ii!=jj)
				AtA_local(jj,ii) = dd;
			}
		}
		AtA_global += AtA_local;
		for(j=0;j<en;j++)
			AtA_local(j,j) += E;
		c[i].AtA_inv = AtA_local.inverse();
	}
	AtA_inv = AtA_global.inverse();
	printf("AtA : %lf\n",AtA_inv(0,0));
}
void Object::ShapeMatching_target()
{
	int i, j, k;
	// update particle
	for(i=0; i<n;i++)
	{
		vector3f t = p[i].x * p[i].m;
		p[i].mxx0t.set(
			t.x * p[i].x0.x, t.x * p[i].x0.y,t.x * p[i].x0.z,
			t.y * p[i].x0.x, t.y * p[i].x0.y,t.y * p[i].x0.z,
			t.z * p[i].x0.x, t.z * p[i].x0.y,t.z * p[i].x0.z
			);
	}
	// cluster cmp cal
	
	int obn = c.size();
	for(i=0;i<obn;i++)
	{
		c[i].getCenter(this);
	}

	// cluster ShapeMatching
	for(i=0;i<obn; i++)
	{
		if(c[i].non_fs)continue;
		matrix33f Fd = c[i].getMomentMatrix(this);
		matrix33f F = Fd * c[i].Ustatic;
		matrix33f Umode = F.transpose() * F;

		c[i].work = false;
		// sqrt(Umode)
//		Umode = c[i].Eigen.transpose() * Umode * c[i].Eigen; // "warm start" [Rivers07]...?
		MatrixXd S(3,3);
		for(int ii=0;ii<3;ii++)
			for(int jj=0;jj<3;jj++)
				S(ii,jj) = Umode.value[ii][jj];
		SelfAdjointEigenSolver<MatrixXd> eigensolver(S);
		if(eigensolver.info() != Success)
			printf("hull - ShapeMatching_target - cluster\n");
		else
		{
			MatrixXd Y(3,3);
			Y.setZero();
			
			Y(0,0) = eigensolver.eigenvalues()(0);
			Y(1,1) = eigensolver.eigenvalues()(1);
			Y(2,2) = eigensolver.eigenvalues()(2);
			if(Y(0,0) < 0) Y(0,0) = 1/E; else	Y(0,0) = 1 / sqrt(Y(0,0));
			if(Y(1,1) < 0) Y(1,1) = 1/E; else	Y(1,1) = 1 / sqrt(Y(1,1));
			if(Y(2,2) < 0) Y(2,2) = 1/E; else	Y(2,2) = 1 / sqrt(Y(2,2));
			MatrixXd Q = eigensolver.eigenvectors();
			S = Q * Y * Q.transpose();
			
			for(int ii=0;ii<3;ii++)
				for(int jj=0;jj<3;jj++)
				{
					c[i].Eigen.value[ii][jj] = Q(ii,jj);
					Umode.value[ii][jj] = S(ii,jj);
				}
			matrix33f R = F * Umode;
			if(Umode.inverse())
			{
				double r_d = R.determinant();
				if(r_d < 0)
				{
					R *= -1;
					Umode *= -1;
					r_d *= -1;
				}
				c[i].R = R;
				c[i].U[0] = Umode.value[0][0];
				c[i].U[1] = Umode.value[1][1];
				c[i].U[2] = Umode.value[2][2];
				c[i].U[3] = Umode.value[0][1];
				c[i].U[4] = Umode.value[0][2];
				c[i].U[5] = Umode.value[1][2];
				c[i].work = true;
			}
		}
	}

	


	// goal position update!
	// dammm it
	double *w;
	int en = 2;
#ifdef TETRA_WORK
	en = this->en+1;
#endif
	w = new double[en];
	for(i=0;i<en;i++)
		w[i] = 0;
	if(en > 1)
	{
		MatrixXd wt(en-1,1);
		wt.setZero();
		for(k=1;k<en;k++)
		for(i=0;i<c.size();i++)
		{
			if(c[i].non_fs)continue;
			double cur[6];
			double exa[6];
			for(j=0;j<6;j++)
				cur[j] = c[i].U[j] - c[i].Umode[0][j];
			for(j=0;j<6;j++)
				exa[j] = c[i].Umode[k][j] - c[i].Umode[0][j];
			for(j=0;j<6;j++)
				wt(k-1,0) += cur[j] * exa[j];
		}
		// weight_global_tmp = AtA_inv * weight_global_tmp; what is this?
		wt = AtA_inv * wt;
		for(i=0;i<en-1;i++)
		{
			w[i+1] = wt(i,0) + target_cheat;
			if(example_rate_exist)
			{
				w[i+1] *= example_rate[i];
			}
		}
	}
	{
		double ssum=0;
		for(i=1;i<en;i++)
			ssum+=w[i];
		w[0] = 1 - ssum;
	}
//	for(i=0;i<en;i++)printf("%lf ",w[i]);printf("\n");
	
	for(i=0;i<en;i++)
		if(w[i]<0)
			break;
	if(i<en)
	{
		double ssum;
		double mmin;
		mmin = w[0];
		ssum = 0;
		for(i=0;i<en;i++)
		if(mmin > w[i]) mmin = w[i];
		for(i=0;i<en;i++)
		{
			w[i] -= mmin;
			ssum += w[i];
		}
		for(i=0;i<en;i++)
			w[i] /= ssum;
	}
	{
		double ssum = 0;
		for(i=1;i<en;i++)
		{
			ssum += w[i];
			w[i] *= beta;
		}
		w[0] += (1.0 - beta) * ssum;
	}
	
//	w[0] = 0; w[1] = 1;
	vector<int> gcnt;
	for(i=0;i<n;i++)
	{
		gcnt.push_back(0);
		g[i] = vector3f();
	}

	for(i=0;i<c.size();i++)
	{/*
		for(j=0;j<c[i].vv.size();j++)
			if(c[i].vv[j]==n-1)break;
		if(j<c[i].vv.size())
		{
			printf("%d %lf\n",i,c[i].R.determinant());
			for(j=0;j<6;j++)
				printf("%lf ",c[i].Umode[0][j]);
			printf("\n");
		}
		if(abs(c[i].R.determinant()) < E) continue;
		*/
		if(!c[i].work) continue;
		double Udash[6];
		for(j=0;j<6;j++)
			Udash[j] = 0;
		for(k=0;k<en;k++)
		for(j=0;j<6;j++)
		{
			Udash[j] += c[i].Umode[k][j] * w[k];
		}
		matrix33f Udash2;
		Udash2.set(Udash[0], Udash[3], Udash[4],
			Udash[3], Udash[1], Udash[5],
			Udash[4], Udash[5], Udash[2]);
		matrix33f Fdash = c[i].R * Udash2;
		for(j=0;j<c[i].vv.size();j++)
		{
			vector3f pi = x0[c[i].vv[j]] - c[i].cmp0;

			g[c[i].vv[j]].x += Fdash.value[0][0] * pi.x + Fdash.value[0][1] * pi.y + Fdash.value[0][2] * pi.z  + c[i].cmp.x;
			g[c[i].vv[j]].y += Fdash.value[1][0] * pi.x + Fdash.value[1][1] * pi.y + Fdash.value[1][2] * pi.z  + c[i].cmp.y;
			g[c[i].vv[j]].z += Fdash.value[2][0] * pi.x + Fdash.value[2][1] * pi.y + Fdash.value[2][2] * pi.z  + c[i].cmp.z;

			/*
			g[c[i].vv[j]].x += Fdash.value[0][0] * pi.x + Fdash.value[1][0] * pi.y + Fdash.value[2][0] * pi.z  + c[i].cmp.x;
			g[c[i].vv[j]].y += Fdash.value[0][1] * pi.x + Fdash.value[1][1] * pi.y + Fdash.value[2][1] * pi.z  + c[i].cmp.y;
			g[c[i].vv[j]].z += Fdash.value[0][2] * pi.x + Fdash.value[1][2] * pi.y + Fdash.value[2][2] * pi.z  + c[i].cmp.z;
			*/
			gcnt[c[i].vv[j]]++;
		}
	}
	for(i=0;i<n;i++)
	{
		if(gcnt[i]>0)
			 g[i] /= gcnt[i];
		else
			g[i] = p[i].x;
//		printf("%lf %lf %lf  %lf %lf %lf\n",p[i].x.x, p[i].x.y, p[i].x.z, g[i].x,g[i].y,g[i].z);
	}
	delete[] w;
}
void Object::Integrate()
{
	int i, j;
	for(i=0; i<n; i++)
	{
		p[i].a += vector3f(0, -1.098, 0);
		p[i].v += 1 * (g[i] - p[i].x) / dt;
//		if(x0[i].y > 1.6) p[i].a = p[i].v = 0;
//		p[i].Integrate(dt);
	}
}
/*
static bool capture_inil = false;
objModel *model_out_target;
objModel *model_out_simple;
void Object::capture_setting()
{
	capture_inil = true;
	model_out_target =  new objModel();
	model_out_simple = new objModel();
	int i, j, k;
	int xl=0;
	vector<int> x;
	int *inv_x = 0;
	if(cmodel->is_mesh)
	{
		//refresh vertex point
		for(i=0;i<cmodel->model->vv.size();i++)
		{
			int ti = cmodel->obj_tetmap[i];
			vector3f param = cmodel->obj_param[i];
			vector3f p0,p1,p2,p3;
			tetTash tash = cmodel->t[ti];
			p0 = cp[tash.v0].x;
			p1 = cp[tash.v1].x;
			p2 = cp[tash.v2].x;
			p3 = cp[tash.v3].x;
			vector3f res;
			res += param.x * p0;
			res += param.y * p1;
			res += param.z * p2;
			res += (1-param.x-param.y-param.z) * p3;
			cmodel->model->vv[i] = res;
			vector3f pp = res;
			pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
			pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
			pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
			model_out_target->vv.push_back(pp);
			model_out_target->vn.push_back(cmodel->model->vn[i]);
		}
	}
	else
	{
		inv_x = new int[cn];
		for(i=0;i<cn; i++)
		{
			if(cmodel->corner[i])
			{
				vector3f pp = cp[i].x;
				pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
				pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
				pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
				pp.z -= 1;
				inv_x[i] = x.size();
				x.push_back(i);
				model_out_target->vv.push_back(pp);
				model_out_target->vn.push_back(cmodel->model->vn[i]);
			}
		}
	}
	objObject *obj = new objObject();
	obj->name = new char[7];
	strcpy(obj->name,"target");
	obj->name[6]=0;
	for(i=0;i<cmodel->model->obj[0]->f.size();i++)
	{
		objFace *face = cmodel->model->obj[0]->f[i];
		bool writeable = true;
		if(!cmodel->is_mesh)
		for(j=0;j<face->vv.size();j++)
			if(!cmodel->corner[face->vv[j]])
				writeable = false;
		if(writeable)
		{
		objFace *nface = new objFace();
		for(j=0;j<face->vv.size();j++)
		{
			k = face->vv[j];
			if(!cmodel->is_mesh)
			{
				k = inv_x[k];
			}
			nface->vv.push_back(k);
			nface->vn.push_back(k);

		}
		obj->f.push_back(nface);
		}
	}
	model_out_target->obj.push_back(obj);

	x.clear();
	if(inv_x)delete[] inv_x;
	if(tmodel->is_mesh)
	{
		//refresh vertex point
		for(i=0;i<tmodel->model->vv.size();i++)
		{
			int ti = tmodel->obj_tetmap[i];
			vector3f param = tmodel->obj_param[i];
			vector3f p0,p1,p2,p3;
			tetTash tash = tmodel->t[ti];
			p0 = p[tash.v0].x;
			p1 = p[tash.v1].x;
			p2 = p[tash.v2].x;
			p3 = p[tash.v3].x;
			vector3f res;
			res += param.x * p0;
			res += param.y * p1;
			res += param.z * p2;
			res += (1-param.x-param.y-param.z) * p3;
			cmodel->model->vv[i] = res;
			vector3f pp = res;
			pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
			pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
			pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
			model_out_simple->vv.push_back(pp);
			model_out_simple->vn.push_back(cmodel->model->vn[i]);
		}
	}
	else
	{
		inv_x = new int[n];
		for(i=0;i<n; i++)
		{
			if(tmodel->corner[i])
			{
				vector3f pp = p[i].x;
				pp.x = (pp.x + tmodel->trans_before_x) *tmodel->scale_xx + tmodel->trans_x;
				pp.y = (pp.y + tmodel->trans_before_y) *tmodel->scale_xx + tmodel->trans_y;
				pp.z = (pp.z + tmodel->trans_before_z) *tmodel->scale_xx + tmodel->trans_z;
				pp.z -= 1;
				inv_x[i] = x.size();
				x.push_back(i);
				model_out_simple->vv.push_back(pp);
				model_out_simple->vn.push_back(tmodel->model->vn[i]);
			}
		}
	}
	obj = new objObject();
	obj->name = new char[5];
	strcpy(obj->name,"rest");
	obj->name[4]=0;
	for(i=0;i<tmodel->model->obj[0]->f.size();i++)
	{
		objFace *face = tmodel->model->obj[0]->f[i];
		bool writeable = true;
		if(!tmodel->is_mesh)
		for(j=0;j<face->vv.size();j++)
			if(!tmodel->corner[face->vv[j]])
				writeable = false;
		if(writeable)
		{
		objFace *nface = new objFace();
		for(j=0;j<face->vv.size();j++)
		{
			k = face->vv[j];
			if(!tmodel->is_mesh)
			{
				k = inv_x[k];
			}
			nface->vv.push_back(k);
			nface->vn.push_back(k);

		}
		obj->f.push_back(nface);
		}
	}
	model_out_simple->obj.push_back(obj);

	if(inv_x) delete[] inv_x;
}
*/
void Object::capture()
{
	char filename[40];
	int i, j, k;
	/*
	if(cmodel->is_mesh)
	{
		//refresh vertex point
		for(i=0;i<cmodel->model->vv.size();i++)
		{
			int ti = cmodel->obj_tetmap[i];
			vector3f param = cmodel->obj_param[i];
			vector3f p0,p1,p2,p3;
			tetTash tash = cmodel->t[ti];
			p0 = cp[tash.v0].x;
			p1 = cp[tash.v1].x;
			p2 = cp[tash.v2].x;
			p3 = cp[tash.v3].x;
			vector3f res;
			res += param.x * p0;
			res += param.y * p1;
			res += param.z * p2;
			res += (1-param.x-param.y-param.z) * p3;
			cmodel->model->vv[i] = res;
			vector3f pp = res;
			pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
			pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
			pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
			model_out_target->vv[i] = (pp);
			model_out_target->vn[i] = (cmodel->model->vn[i]);
		}
	}
	else
	{
		j=0;
		for(i=0;i<cn;i++)
		{
			if(cmodel->corner[i])
			{
				vector3f pp = cp[i].x;
				pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
				pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
				pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
				pp.z -= 1;
				model_out_target->vv[j] = (pp);
				model_out_target->vn[j] = (cmodel->model->vn[i]);
				j++;
			}
		}
	}
	sprintf(filename,"output_target/%d.obj",frame_num);
	model_out_target->writeModel(filename);
	printf("capture %s\n",filename);

	
	if(tmodel->is_mesh)
	{
		//refresh vertex point
		for(i=0;i<tmodel->model->vv.size();i++)
		{
			int ti = tmodel->obj_tetmap[i];
			vector3f param = tmodel->obj_param[i];
			vector3f p0,p1,p2,p3;
			tetTash tash = tmodel->t[ti];
			p0 = cp[tash.v0].x;
			p1 = cp[tash.v1].x;
			p2 = cp[tash.v2].x;
			p3 = cp[tash.v3].x;
			vector3f res;
			res += param.x * p0;
			res += param.y * p1;
			res += param.z * p2;
			res += (1-param.x-param.y-param.z) * p3;
			tmodel->model->vv[i] = res;
			vector3f pp = res;
			pp.x = (pp.x + tmodel->trans_before_x) *tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y) *tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z) *tmodel->scale_xx + tmodel->trans_z;
			model_out_simple->vv[i] = (pp);
			model_out_simple->vn[i] = (tmodel->model->vn[i]);
		}
	}
	else
	{
		j=0;
		for(i=0;i<n;i++)
		{
			if(tmodel->corner[i])
			{
				vector3f pp = p[i].x;
				pp.x = (pp.x + tmodel->trans_before_x) *tmodel->scale_xx + tmodel->trans_x;
				pp.y = (pp.y + tmodel->trans_before_y) *tmodel->scale_xx + tmodel->trans_y;
				pp.z = (pp.z + tmodel->trans_before_z) *tmodel->scale_xx + tmodel->trans_z;
				pp.z -= 1;
				model_out_simple->vv[j] = (pp);
				model_out_simple->vn[j] = (tmodel->model->vn[i]);
				j++;
			}
		}
	}

	sprintf(filename,"output_simple/%d.obj",frame_num);
	model_out_simple->writeModel(filename);
	printf("capture %s\n",filename);
	
	*/
		/*
	if(!cmodel->is_mesh)
	{
		for(i=0;i<cn;i++)
		{
			vector3f pp = cp[i].x;
			pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
			pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
			pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
			pp.z -= 1;
			cmodel->model->vv[i] = (pp);
		}
	}
	else
	{
		for(i=0;i<cmodel->model->vv.size();i++)
		{
			vector3f pp = cmodel->model->vv[i];
			pp.x = (pp.x + cmodel->trans_before_x) *cmodel->scale_xx + cmodel->trans_x;
			pp.y = (pp.y + cmodel->trans_before_y) *cmodel->scale_xx + cmodel->trans_y;
			pp.z = (pp.z + cmodel->trans_before_z) *cmodel->scale_xx + cmodel->trans_z;
			pp.z -= 1;
			cmodel->model->vv[i] = (pp);
		}
	}
	sprintf(filename,"output_target/%d.obj",frame_num);
	cmodel->model->writeModel(filename);
	printf("capture %s\n",filename);
	if(!tmodel->is_mesh)
	{
		for(i=0;i<n;i++)
		{
			vector3f pp = p[i].x;
			pp.x = (pp.x + tmodel->trans_before_x) *tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y) *tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z) *tmodel->scale_xx + tmodel->trans_z;
			pp.z -= 1;
			tmodel->model->vv[i] = (pp);
		}
	}
	else
	{
		for(i=0;i<tmodel->model->vv.size();i++)
		{
			vector3f pp = tmodel->model->vv[i];
			pp.x = (pp.x + tmodel->trans_before_x) *tmodel->scale_xx + tmodel->trans_x;
			pp.y = (pp.y + tmodel->trans_before_y) *tmodel->scale_xx + tmodel->trans_y;
			pp.z = (pp.z + tmodel->trans_before_z) *tmodel->scale_xx + tmodel->trans_z;
			pp.z -= 1;
			tmodel->model->vv[i] = (pp);
		}
	}
	sprintf(filename,"output_simple/%d.obj",frame_num);
	tmodel->model->writeModel(filename);
	printf("capture %s\n",filename);
	frame_num++;
	printf("let it go");
	*/
}