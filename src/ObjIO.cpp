#include"ObjIO.h"
#include <stdio.h>
#include <string.h>
#define MAX_BUF 256
void objModel::parsing_mtl(const char *src)
{
	char buf[MAX_BUF];
	char command[100];
	char elc[MAX_BUF];
	char *name = "";
	double elf;
	double r,g,b;
	int eli;
	int len;
	FILE *fp = fopen(src,"rt");
	if(fp == NULL)
	{
		printf("- no mtl file.\n");
		return;
	}
	objMeterial *cur_mtl = NULL;
	int dd=0;
	while(fgets(buf, MAX_BUF, fp) > 0)
	{
		len = strlen(buf);

		if(sscanf(buf, "%s",command) <= 0)continue;
		if(strcmp(command, "newmtl") == 0) {
			if( cur_mtl != NULL)
				mtl.push_back(cur_mtl);
			int i, j;
			elc[0] = 0;
			for(i=7; i<len; i++)
				elc[i-7] = buf[i];
			elc[len-8] = 0;
			cur_mtl = new objMeterial();
			cur_mtl->name = new char[len-6];
			strcpy(cur_mtl->name ,elc);
		}
		else if(strcmp(command, "Ns") == 0){
			sscanf(buf, "\tNs %lf", &elf);
			cur_mtl->Ns = elf;
		}
		else if(strcmp(command, "d") == 0){
			sscanf(buf, "\td %lf", &elf);
			cur_mtl->d = elf;
		}
		else if(strcmp(command, "Tr") == 0){
			sscanf(buf, "\tTr %lf", &elf);
			cur_mtl->Tr = elf;
		}
		else if(strcmp(command, "illum") == 0){
			sscanf(buf, "\tillum %d", &eli);
			cur_mtl->illum = eli;
		}
		else if(strcmp(command, "Ka") == 0){
			sscanf(buf,"\tKa %lf %lf %lf",&r,&g,&b);
			cur_mtl->Ambient.setf(r,g,b);
		}
		else if(strcmp(command, "Kd") == 0){
			sscanf(buf,"\tKd %lf %lf %lf",&r,&g,&b);
			cur_mtl->Diffuse.setf(r,g,b);
		}
		else if(strcmp(command, "Ks") == 0){
			sscanf(buf,"\tKs %lf %lf %lf",&r,&g,&b);
			cur_mtl->Specular.setf(r,g,b);
		}
	}
	if( cur_mtl != NULL)
		mtl.push_back(cur_mtl);
	fclose(fp);
}
void objModel::getModel(const char* src)
{
	FILE *fp1 = fopen(src,"rt");
	if(fp1 == NULL){
		printf("no file error. %s\n",src);
		return;
	}
	reset();

	char buf[MAX_BUF];
	char command[100];
	char elc[MAX_BUF];
	char directory[MAX_BUF];
	double elf;
	double x,y,z,w;
	int len;
	int jusuck_count = 0;

	int cur_obj_i = -1;
	objObject *cur_obj = NULL;
	
	int dd=0;
	strcpy(directory,src);
	for(int i=0;;i++)
	{
		if(directory[i] == 0)
		{
			while(i>=0 && directory[i] != '/'){
				i--;
			}
			directory[i+1] = 0;
			break;
		}
	}
	int line_num = 0;
	while(fgets(buf, MAX_BUF, fp1) > 0)
	{
		line_num++;
//		printf("#%d\n",line_num);
		len = strlen(buf);
		if(len>0 && buf[0] == '#')
		{
			jusuck_count ++;
			continue;
		}
		else{ jusuck_count = 0;}

		if(sscanf(buf,"%s",command) <= 0) continue;

		if(strcmp(command, "mtllib") == 0)	{
			int i, j;
			int si, ei;
			for(si=0;directory[si];si++)
				elc[si] = directory[si];
			ei = 7;
			if(buf[ei] == '.' && buf[ei+1] =='/')
				ei += 2;
			for(i=ei; i<len; i++)
				elc[i-ei+si] = buf[i];
			elc[len-ei-1+si] = 0;
			parsing_mtl(elc);
		}
		else if(strcmp(command, "v") == 0) {
			int i, j, k;
			int offset=0;
			double v[5];
			vector3f cur_vv;
			while(buf[offset]!='v')offset++;
			offset++;
			while(buf[offset]=='\t' || buf[offset] == 32) offset++;
			for(i=0; offset<len; i++,offset++)
			{
				for(j=0;buf[offset]!=32 && buf[offset] != 10 && buf[offset] != '\t' && buf[offset]>0; j++,offset++)
					elc[j] = buf[offset];
				if(j==0)break;
				elc[j] = 0;
				sscanf(elc,"%lf",&v[i]);
			}
			switch(i)
			{
			case 0:
				cur_vv = vector3f();
				break;
			case 1:
				cur_vv = vector3f(v[0]);
				break;
			case 2:
				cur_vv = vector3f(v[0],v[1],0);
				break;
			case 3:
				cur_vv = vector3f(v[0],v[1],v[2]);
				break;
			case 4:
				cur_vv = vector3f(v[0],v[1],v[2]);	// TODO
				break;
			}
			vv.push_back(cur_vv);
		}
		else if(strcmp(command, "vt") == 0) {
			int i, j, k;
			int offset = 0;
			double v[5];
			vector2f cur_vt;
			while(buf[offset]!='t')offset++;
			offset++;
			while(buf[offset]=='\t' || buf[offset] == 32) offset++;
			for(i=0; offset<len; i++,offset++)
			{
				for(j=0;buf[offset]!=32 && buf[offset] != 10 && buf[offset] != '\t' && buf[offset]>0; j++,offset++)
					elc[j] = buf[offset];
				if(j==0)break;
				elc[j] = 0;
				sscanf(elc,"%lf",&v[i]);
			}
			switch(i)
			{
			case 0:
				cur_vt = vector2f();
				break;
			case 1:
				cur_vt = vector2f(v[0]);
				break;
			case 2:
				cur_vt = vector2f(v[0],v[1]);
				break;
			case 3:
				cur_vt = vector2f(v[0],v[1]);
				break;
			}
			vt.push_back(cur_vt);
		}
		else if(strcmp(command, "vn") == 0) {
			int i, j, k;
			int offset = 0;
			double v[5];
			vector3f cur_vn;
			while(buf[offset]!='n')offset++;
			offset++;
			while(buf[offset]=='\t' || buf[offset] == 32) offset++;
			for(i=0; offset<len; i++,offset++)
			{
				for(j=0;buf[offset]!=32 && buf[offset] != 10 && buf[offset] != '\t' && buf[offset]>0; j++,offset++)
					elc[j] = buf[offset];
				if(j==0)break;
				elc[j] = 0;
				sscanf(elc,"%lf",&v[i]);
			}
			switch(i)
			{
			case 0:
				cur_vn = vector3f();
				break;
			case 1:
				cur_vn = vector3f(v[0]);
				break;
			case 2:
				cur_vn = vector3f(v[0],v[1],0);
				break;
			case 3:
				cur_vn = vector3f(v[0],v[1],v[2]);
				break;
			}
			vn.push_back(cur_vn);
		}
		else if(strcmp(command, "vp") == 0) {
			int i, j, k;
			int offset = 0;
			double v[5];
			vector3f cur_vp;
			while(buf[offset]!='p')offset++;
			offset++;
			while(buf[offset]=='\t' || buf[offset] == 32) offset++;
			for(i=0; offset<len; i++,offset++)
			{
				for(j=0;buf[offset]!=32 && buf[offset] != 10 && buf[offset] != '\t' && buf[offset]>0; j++,offset++)
					elc[j] = buf[offset];
				if(j==0)break;
				elc[j] = 0;
				sscanf(elc,"%lf",&v[i]);
			}
			switch(i)
			{
			case 0:
				cur_vp = vector3f();
				break;
			case 1:
				cur_vp = vector3f(v[0]);
				break;
			case 2:
				cur_vp = vector3f(v[0],v[1],0);
				break;
			case 3:
				cur_vp = vector3f(v[0],v[1],v[2]);
				break;
			}
			vp.push_back(cur_vp);
		}
		
		else if(strcmp(command, "g") == 0) {
			cur_obj_i++;
			cur_obj = new objObject();
			obj.push_back(cur_obj);
			if(len-2 > 0)
			{
				obj[cur_obj_i]->name = new char[len-2];
				sscanf(buf, "g %s",obj[cur_obj_i]->name);
			}
		}
		else if(strcmp(command, "usemtl") == 0) {
			sscanf(buf, "usemtl %s",elc);
			std::vector<objMeterial*>::iterator it;
			for(it = mtl.begin(); it != mtl.end(); it++)
				if(strcmp(elc,(*it)->name) == 0)
					break;
			if(it != mtl.end())
				obj[cur_obj_i]->mtl = *it;
		}
		else if(strcmp(command, "s") == 0) {
			sscanf(buf, "s %s",elc);
			if(strlen(elc)>1)
				cur_obj->smooth = false;
			else
				cur_obj->smooth = true;
		}
		else if(strcmp(command, "f") == 0) {
			objFace *flag = new objFace();
			int v[3] = {0,0,0};
			int offset = 2;
			while(1)
			{
				int i, j, k, l=0;
				for(j=0; offset<len; j++,offset++)
				{
					if(buf[offset] == 32 || buf[offset] == 10 || buf[offset] == '\t')
						break;
					elc[j] = buf[offset];
				}
				if(offset == len || j == 0)break;
				offset++;;
				elc[j] = 0;
				k=0;
				for(i=0; i<=j; i++)
				{
					if(i==j || elc[i] == '/')
					{
						v[l++] = k;
						k=0;
					}
					else
					{
						k*=10;
						k+=(elc[i] - '0');
					}
				}
				if(v[0]>0)
					flag->vv.push_back(v[0]-1);
				if(v[1]>0)
					flag->vt.push_back(v[1]-1);
				if(v[2]>0)
					flag->vn.push_back(v[2]-1);
			}
			if(cur_obj_i < 0)
			{
				cur_obj_i++;
				cur_obj = new objObject();
				obj.push_back(cur_obj);
			}
			obj[cur_obj_i]->f.push_back(flag);
		}
	}
	fclose(fp1);
}

void objModel::rebuild(double error)
{
	int i, j, k;
	int nx, ny, nz;
	nx = ny = nz = 50;
	double dx,dy,dz;
	double min_x,max_x;
	double min_y,max_y;
	double min_z,max_z;
	int *pp;
	min_x = max_x = vv[0].x;
	min_y = max_y = vv[0].y;
	min_z = max_z = vv[0].z;

	hash_map<int, vector<int> > h;

	for(i=1;i<vv.size();i++)
	{
		min_x = min(min_x,vv[i].x);
		min_y = min(min_y,vv[i].y);
		min_z = min(min_z,vv[i].z);
		max_x = max(max_x,vv[i].x);
		max_y = max(max_y,vv[i].y);
		max_z = max(max_z,vv[i].z);
	}
	dx = (max_x - min_x + 10*error)/nx;
	dy = (max_x - min_x + 10*error)/ny;
	dz = (max_x - min_x + 10*error)/nz;
	pp = new int[vv.size()];
	for(i=0;i<vv.size();i++)
	{
		int tx,ty,tz;
		tx = (vv[i].x - min_x) / dx;
		ty = (vv[i].y - min_y) / dy;
		tz = (vv[i].z - min_z) / dy;
		if(tx<0)tx=0; if(tx>=nx)tx=nx-1;
		if(ty<0)ty=0; if(ty>=ny)ty=ny-1;
		if(tz<0)tz=0; if(tz>=nz)tz=nz-1;
		h[tx*ny*nz+ty*nz+tz].push_back(i);
		pp[i] = i;
	}
	for(i=0;i<vv.size();i++)
	{
		int tx,ty,tz;
		tx = (vv[i].x - min_x) / dx;
		ty = (vv[i].y - min_y) / dy;
		tz = (vv[i].z - min_z) / dy;
		if(tx<0)tx=0; if(tx>=nx)tx=nx-1;
		if(ty<0)ty=0; if(ty>=ny)ty=ny-1;
		if(tz<0)tz=0; if(tz>=nz)tz=nz-1;
		int qx,qy,qz;
		for(qx=tx-1; qx<=tx+1;qx++)
		for(qy=ty-1; qy<=ty+1;qy++)
		for(qz=tz-1; qz<=tz+1;qz++)
		{
			if(qx<0||qx>=nx)continue;
			if(qy<0||qy>=ny)continue;
			if(qz<0||qz>=nz)continue;
		vector<int> hv = h[qx*ny*nz+qy*nz+qz];
		for(j=0;j<hv.size();j++)
		{
			if(hv[j] > i && vv[i].dist(vv[hv[j]]) < error)
			{
				k = i;
				int l,ll;
				while(pp[k]!=k)k=pp[k];
				l=i;
				while(pp[l]!=l)
				{
					ll=pp[l];
					pp[l]=k;
					l=ll;
				}
				pp[hv[j]] = k;
			}
		}
		}
	}
	for(i=0;i<obj.size();i++)
	{
		objObject *od = obj[i];
		for(j=0;j<od->f.size(); j++)
		{
			objFace *face = od->f[j];
			for(k=0; k<face->vv.size(); k++)
			{
				face->vv[k] = pp[face->vv[k]];
			}
		}
	}
}
void objModel::resize()
{
	int i, j, k;
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	min_x = max_x = vv[0].x;
	min_y = max_y = vv[0].y;
	min_z = max_z = vv[0].z;
	for(i=1;i<vv.size();i++)
	{
		min_x = min(min_x,vv[i].x);
		min_y = min(min_y,vv[i].y);
		min_z = min(min_z,vv[i].z);
		max_x = max(max_x,vv[i].x);
		max_y = max(max_y,vv[i].y);
		max_z = max(max_z,vv[i].z);
	}
	double maxx = max(max(max_x-min_x,max_y-min_y),max_z-min_z);
	for(i=0;i<vv.size();i++)
	{
		vv[i].x = (vv[i].x - min_x) / maxx + (maxx - (max_x-min_x))/maxx/2;
		vv[i].y = (vv[i].y - min_y) / maxx + (maxx - (max_y-min_y))/maxx/2;
		vv[i].z = (vv[i].z - min_z) / maxx + (maxx - (max_z-min_z))/maxx/2;
	}
}
void objModel::normalize()
{
	vn.clear();

	int i, j, k, l;
	j = vv.size();
	l = obj.size();
	vector3f *normal_mean = new vector3f[j];
	int *normal_mean_num = new int[j];

	for(i=0;i<j;i++)
	{
		normal_mean[i] = vector3f();
		normal_mean_num[i] = 0;
	}
	for(int ii = 0; ii<l; ii++)
	{
		k = obj[ii]->f.size();
		for(i=0;i<k;i++)
		{
			objFace * face = obj[ii]->f[i];
			face->vn.clear();
			face->vn.resize(face->vv.size());
			for(int jj=1; jj+1<face->vv.size();jj++)
			{
				vector3f n1;
				vector3f p01 = vv[face->vv[jj]] - vv[face->vv[0]];
				vector3f p12 = vv[face->vv[jj+1]] - vv[face->vv[jj]];
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
	for(i=0;i<j;i++)
	{
		if(normal_mean_num[i]>0)
			normal_mean[i] /= normal_mean_num[i];
		vn.push_back(normal_mean[i]);
	}

	delete [] normal_mean;
	delete [] normal_mean_num;
}
void objModel::getModel_3ds(const char *filename)
{
	int index=0;
	FILE *l_file; 
	if(fopen_s(&l_file,filename,"rb") !=0 ) {
		printf("error : 3DS load\n");
		exit(0);
	} 
	int i;
	unsigned short l_chunk_id;
	unsigned int l_chunk_lenght; 
	unsigned char l_char; 
	unsigned short l_qty; 
	unsigned short l_face_flags; 

	objObject *cur_obj = new objObject();
	obj.push_back(cur_obj);

	fseek(l_file, 0l, SEEK_END);  
	long l_filelength = ftell(l_file);  
	fseek(l_file, 0l, SEEK_SET);
	while (ftell (l_file) < l_filelength) {
		fread (&l_chunk_id, 2, 1, l_file); 
		fread (&l_chunk_lenght, 4, 1, l_file);	
		switch (l_chunk_id)
		{
		case 0x4d4d: 
			break;    
		case 0x3d3d:
			break;
		case 0x4000: 
			i=0;
			do {
				fread (&l_char, 1, 1, l_file);
				i++;
			} while(l_char != '\0' && i<20);
			break;
		case 0x4100:

			break;
		case 0x4110: 			
			fread (&l_qty, sizeof (unsigned short), 1, l_file);
			for (i=0; i<l_qty; i++) {
				double dx, dy, dz;
				fread (&dx, sizeof(double), 1, l_file);
				fread (&dy, sizeof(double), 1, l_file);
				fread (&dz, sizeof(double), 1, l_file);

				vv.push_back(vector3f(dx,dy,dz));
			}
			break;
		case 0x4120:
			fread (&l_qty, sizeof (unsigned short), 1, l_file);
			for(i=0; i<l_qty; i++) {
				unsigned short face_id[3];
				fread (&face_id[0], sizeof (unsigned short), 1, l_file);
				fread (&face_id[1], sizeof (unsigned short), 1, l_file);
				fread (&face_id[2], sizeof (unsigned short), 1, l_file);
				fread (&l_face_flags, sizeof (unsigned short), 1, l_file);

				objFace *cur_face = new objFace();
				cur_face->vv.push_back(face_id[0]);
				cur_face->vv.push_back(face_id[1]);
				cur_face->vv.push_back(face_id[2]);
				cur_obj->f.push_back(cur_face);
			}
			break;
			break;
		case 0x4140:
		default:
			fseek(l_file, l_chunk_lenght-6, SEEK_CUR);
		} 
	}
	fclose (l_file); 
}

#define WRITE_LINE(ppp) fprintf(fp, "%s\n",(ppp));
static void write_vectors_v(FILE *fp,std::vector<vector3f> &v)
{
	char vv[1024];
	int i, j, l;

	l = v.size();
	for(i=0; i<l; i++)
	{
		sprintf(vv,"v %lf %lf %lf", VECTOR3F_PRINTF(v[i]));
		WRITE_LINE(vv);
	}
}
static void write_vectors_vn(FILE *fp,std::vector<vector3f> &v)
{
	char vn[1024];
	int i, j, l;

	l = v.size();
	for(i=0; i<l; i++)
	{
		sprintf(vn,"vn %lf %lf %lf", VECTOR3F_PRINTF(v[i]));
		WRITE_LINE(vn);
	}
}
static void write_flag(FILE *fp, std::vector<objFace *> &v)
{
	char ff[1024];
	char tf[1024];

	char vv[1024];
	for(std::vector<objFace *>::iterator i = v.begin(); i != v.end(); i++)
	{
		sprintf(ff, "f");
		for(int j=0; j<(*i)->vv.size(); j++)
		{
			sprintf(vv,"%d",(*i)->vv[j]+1);
			sprintf(tf, " %s",vv);
			strcat(ff,tf);
		}
		WRITE_LINE(ff);
	}
}
static void write_obj(FILE *fp, std::vector<objObject *> &v)
{
	int i, j, l;
	l = v.size();
	char *blank = "";

	char on[1024];
	char pn[1024];
	char qn[1024];
	for(i=0; i<l; i++)
	{
		sprintf(qn, "#\n#\tObject %s\n#\n",v[i]->name);
		sprintf(on, "g %s",v[i]->name);
		sprintf(pn, "s %s",v[i]->smooth?"1":"off");
		WRITE_LINE(qn);
		WRITE_LINE(on);
		WRITE_LINE(pn);

		write_flag(fp, v[i]->f);
		sprintf(qn,"# %d flags",v[i]->f.size());
		WRITE_LINE(qn);
		WRITE_LINE(blank);
	}
}
static void write_model(FILE *fp, objModel *model)
{
	char *blank = "";

	char *temp1 = "# This obj file is made by hinohie`s program";
	char *temp2 = "# So here may feel with drug state";

	char vv_line[1024];

	WRITE_LINE(temp1);
	WRITE_LINE(temp2);
	WRITE_LINE(blank);
	
	write_vectors_v(fp, model->vv);
	sprintf(vv_line,"# %d vertex", model->vv.size());
	WRITE_LINE(vv_line);
	WRITE_LINE(blank);

	write_obj(fp, model->obj);
	sprintf(vv_line,"# %d objects", model->obj.size());
	WRITE_LINE(vv_line);
}

void objModel::writeModel(const char *filename)
{
	FILE *fp = fopen(filename,"wt");

	write_model(fp, this);

	fclose(fp);
}