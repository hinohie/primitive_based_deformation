#include "TetIO.h"

void tetObject::resize()
{
	int i, j, k;
	double min_x, max_x;
	double min_y, max_y;
	double min_z, max_z;
	min_x = max_x = exam_pos[0][0].x;
	min_y = max_y = exam_pos[0][0].y;
	min_z = max_z = exam_pos[0][0].z;
	for(i=1;i<exam_pos[0].size();i++)
	{
		min_x = min(min_x,exam_pos[0][i].x);
		min_y = min(min_y,exam_pos[0][i].y);
		min_z = min(min_z,exam_pos[0][i].z);
		max_x = max(max_x,exam_pos[0][i].x);
		max_y = max(max_y,exam_pos[0][i].y);
		max_z = max(max_z,exam_pos[0][i].z);
	}
	double maxx = max(max(max_x-min_x,max_y-min_y),max_z-min_z);
	for(k=0;k<en;k++)
	for(i=0;i<pn;i++)
	{
		exam_pos[k][i].x = (exam_pos[k][i].x - min_x) / maxx + (maxx - (max_x-min_x))/maxx/2;;
		exam_pos[k][i].y = (exam_pos[k][i].y - min_y) / maxx + (maxx - (max_y-min_y))/maxx/2;;
		exam_pos[k][i].z = (exam_pos[k][i].z - min_z) / maxx + (maxx - (max_z-min_z))/maxx/2;;
	}
}
void tetObject::tash_reorder()
{
	int i, j;
	if(!is_mesh)
	model->obj[0]->f.clear();
	for(i=0;i<tn;i++)
	{
		tetTash tash = t[i];
		vector3f p[4];
		vector3f cmp;
		p[0] = exam_pos[0][tash.v0];
		p[1] = exam_pos[0][tash.v1];
		p[2] = exam_pos[0][tash.v2];
		p[3] = exam_pos[0][tash.v3];
		cmp = (p[0] + p[1] + p[2] + p[3])/4;
		int c[4] = {0,1,2,3};
		bool success = false;
		do{
			vector3f p1, p2, p3;
			vector3f p12, p23;
			vector3f pn;
			double pd,qd;
			bool res = true;

			p1 = p[c[0]]; p2 = p[c[1]]; p3 = p[c[2]];
			p12 = p2-p1; p23 = p3-p2;
			pn = p12%p23;
			pd = pn*p1; qd = pn*cmp;
			if(pd < qd) res = false;
			p1 = p[c[0]]; p2 = p[c[2]]; p3 = p[c[3]];
			p12 = p2-p1; p23 = p3-p2;
			pn = p12%p23;
			pd = pn*p1; qd = pn*cmp;
			if(pd < qd) res = false;
			p1 = p[c[0]]; p2 = p[c[3]]; p3 = p[c[1]];
			p12 = p2-p1; p23 = p3-p2;
			pn = p12%p23;
			pd = pn*p1; qd = pn*cmp;
			if(pd < qd) res = false;
			p1 = p[c[1]]; p2 = p[c[3]]; p3 = p[c[2]];
			p12 = p2-p1; p23 = p3-p2;
			pn = p12%p23;
			pd = pn*p1; qd = pn*cmp;
			if(pd < qd) res = false;

			if(res)
			{
				success = true;
				break;
			}
		}while(next_permutation(c,c+4));
		if(success)
		{
//			printf("%d %d %d %d\n",c[0],c[1],c[2],c[3]);
			int o[4] = {tash.v0, tash.v1, tash.v2, tash.v3};
			tash.v0 = o[c[0]];
			tash.v1 = o[c[1]];
			tash.v2 = o[c[2]];
			tash.v3 = o[c[3]];
			t[i] = tash;
		}
		else
		{
			printf("hull %d\n",i);
		}
		if(!is_mesh)
		{
		objFace *face;
		int p1, p2, p3, tmp;
		pair<int, pair<int, int> > p;

		p1 = tash.v0; p2 = tash.v1; p3 = tash.v2;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(corner_set.find(p) != corner_set.end())
		{
		face = new objFace();
		face->vv.push_back(tash.v0);face->vv.push_back(tash.v1);face->vv.push_back(tash.v2);
		model->obj[0]->f.push_back(face);
		}

		p1 = tash.v0; p2 = tash.v2; p3 = tash.v3;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(corner_set.find(p) != corner_set.end())
		{
		face = new objFace();
		face->vv.push_back(tash.v0);face->vv.push_back(tash.v2);face->vv.push_back(tash.v3);
		model->obj[0]->f.push_back(face);
		}

		p1 = tash.v0; p2 = tash.v3; p3 = tash.v1;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(corner_set.find(p) != corner_set.end())
		{
		face = new objFace();
		face->vv.push_back(tash.v0);face->vv.push_back(tash.v3);face->vv.push_back(tash.v1);
		model->obj[0]->f.push_back(face);
		}
		p1 = tash.v1; p2 = tash.v3; p3 = tash.v2;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(corner_set.find(p) != corner_set.end())
		{
		face = new objFace();
		face->vv.push_back(tash.v1);face->vv.push_back(tash.v3);face->vv.push_back(tash.v2);
		model->obj[0]->f.push_back(face);
		}
		}
	}
}
void tetObject::cornering()
{
	if(is_mesh)	return;
	corner_set.clear();
	map<pair<int, pair<int, int> >, int> v;
	int i,j,k;
	corner.clear();
	corner.resize(pn);
	for(i=0;i<pn;i++)
		corner[i] = false;
	for(i=0;i<tn;i++)
	{
		tetTash tash = t[i];

		int p1, p2, p3, tmp;
		pair<int, pair<int, int> > p;

		p1 = tash.v0; p2 = tash.v1; p3 = tash.v2;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(v.count(p) == 0) v[p] = 1;
		else v[p]++;
		p1 = tash.v0; p2 = tash.v2; p3 = tash.v3;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(v.count(p) == 0) v[p] = 1;
		else v[p]++;
		p1 = tash.v0; p2 = tash.v3; p3 = tash.v1;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(v.count(p) == 0) v[p] = 1;
		else v[p]++;
		p1 = tash.v1; p2 = tash.v3; p3 = tash.v2;
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		if(p2>p3){tmp=p3;p3=p2;p2=tmp;}
		if(p1>p2){tmp=p1;p1=p2;p2=tmp;}
		p = pair<int, pair<int, int> >(p1, pair<int, int>(p2,p3));
		if(v.count(p) == 0) v[p] = 1;
		else v[p]++;
	}
	for(map<pair<int, pair<int, int> >, int>::iterator it = v.begin(); it != v.end(); it++)
	{
		pair<int, pair<int, int> > p = it->first;
//		printf("%d %d %d  -> %d\n",p.first, p.second.first, p.second.second, it->second);
		if(it->second == 1)
		{
			corner[p.first] = true;
			corner[p.second.first] = true;
			corner[p.second.second] = true;
			corner_set.insert(p);
		}
	}
}
void tetObject::normalize()
{
	model->vn.clear();

	int i, j, k, l;
	j = model->vv.size();
	l = model->obj.size();
	vector3f *normal_mean = new vector3f[j];
	int *normal_mean_num = new int[j];

	for(i=0;i<j;i++)
	{
		normal_mean[i] = vector3f();
		normal_mean_num[i] = 0;
	}
	for(int ii = 0; ii<l; ii++)
	{
		k = model->obj[ii]->f.size();
		for(i=0;i<k;i++)
		{
			objFace * face = model->obj[ii]->f[i];
			face->vn.clear();
			face->vn.resize(face->vv.size());
			for(int jj=1; jj+1<face->vv.size();jj++)
			{
				if(is_mesh || (corner[face->vv[0]] && corner[face->vv[jj+1]] && corner[face->vv[jj]]))
				{
				vector3f n1;
				vector3f p01 = model->vv[face->vv[jj]] - model->vv[face->vv[0]];
				vector3f p12 = model->vv[face->vv[jj+1]] - model->vv[face->vv[jj]];
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
		model->vn.push_back(normal_mean[i]);
	}

	delete [] normal_mean;
	delete [] normal_mean_num;
}
bool tetObject::getModel(const char *filename)
{
	int i, j, k;
	for(i=0;filename[i];i++);
	for(;i>=0 && filename[i] != '.'; i--);
	i++;
	if(i==0)
	{
		printf("filename_error : no comma\n");
		return false;
	}

	if(strcmp(filename+i,"tet")==0)
		return loadTET(filename);
	else
		return loadXML(filename);

}

bool tetObject::getModel(const char *filename, const char *objname)
{
	int i, j, k;
	for(i=0;filename[i];i++);
	for(;i>=0 && filename[i] != '.'; i--);
	i++;
	if(i==0)
	{
		printf("filename_error : no comma\n");
		return false;
	}

	if(strcmp(filename+i,"tet")==0)
		return loadTET(filename, objname);
	else
		return loadXML(filename);
}
bool strequal(char *p, char *q)
{
	int i;
	for(i=0;p[i]&&q[i];i++){
		char x = p[i];
		char y = q[i];
		if(x >= 'A' && x <= 'Z') x += 'a'-'A';
		if(y >= 'A' && y <= 'Z') y += 'a'-'A';
		if(x-y) return false;
	}
	return !q[i];
}
bool tetObject::loadTET(const char *fname)
{
	FILE *fp = fopen(fname,"rt");
	if(fp == NULL)
	{
		printf("file_open_fail\n");
		return false;
	}
	printf("file %s open\n",fname);
	model = new objModel();
	char buf[1009];
	int i, j, k;
	int line=0;
	int cnt=0;
			int fn;
	while(fgets(buf,1009,fp))
	{
		cnt++;
		if(buf[0] == '#') continue;
		for(i=0;buf[i];i++);
		while(i>0&&(buf[i-1]==' ' || buf[i-1] == 10 || buf[i] == 9 || buf[i] == 13)){i--;buf[i]=0;}
		if(strequal(buf,"end"))break;
		if(strequal(buf,"vertices"))
		{
			fgets(buf,1009,fp);
		cnt++;
			sscanf(buf,"%d",&pn);
			printf("pn : %d\n",pn);
			corner.clear();
			corner.resize(pn);
			model->vv.resize(pn);
			en = 1;
			vector3f c;
			for(i=0;i<pn;i++)
			{
				vector3f pp;
				fgets(buf,1009,fp);
		cnt++;
				int cnt = sscanf(buf,"%lf %lf %lf",&pp.x,&pp.y,&pp.z);
				if(cnt < 3)
				{
					i--;
					pn--;
					continue;
				}
				exam_pos[0].push_back(pp);
				corner[i] = false;
				model->vv[i] = (pp);
				c += pp;
			}
			c /= pn;
			for(i=0;i<pn;i++)
			{
				vector3f pp = exam_pos[0][i];
				vector3f qq;
				pp -= c;
				qq.x = pp.x * cos(pp.y/30) - pp.z * sin(pp.y/30);
				qq.z = pp.x * sin(pp.y/30) + pp.z * cos(pp.y/30);
				qq.y = pp.y;
				qq += c;
				exam_pos[1].push_back(qq);
			}
			
		}
		else if(strequal(buf,"triangles"))
		{
			fgets(buf,1009,fp);
		cnt++;
			sscanf(buf,"%d",&fn);
			printf("face number : %d\n",fn);
			objObject *obj = new objObject();
			for(i=0;i<fn;i++)
			{
				fgets(buf,1009,fp);
		cnt++;
				int p1,p2,p3;
				int cnt = sscanf(buf,"%d %d %d",&p1,&p2,&p3);
				if(cnt < 3)
				{
					printf("buf : %s\n",buf);
					i--;
					fn--;
					continue;
				}
				objFace *face = new objFace();
				p1 -= 1;
				p2 -= 1;
				p3 -= 1;
				face->vv.push_back(p1);
				face->vv.push_back(p2);
				face->vv.push_back(p3);
				obj->f.push_back(face);
			}
			model->obj.push_back(obj);
		}
		else if(strequal(buf,"tetrahedra"))
		{
			fgets(buf,1009,fp);
		cnt++;
			sscanf(buf,"%d",&tn);
			printf("tn : %d\n",tn);
			
			for(i=0;i<tn;i++)
			{
				tetTash tash;
				fgets(buf,1009,fp);
		cnt++;
				int cnt = sscanf(buf,"%d %d %d %d",&tash.v0,&tash.v1,&tash.v2,&tash.v3);
				if(cnt < 4)
				{
					i--;
					tn--;
					continue;
				}
				tash.v0 -= 1;
				tash.v1 -= 1;
				tash.v2 -= 1;
				tash.v3 -= 1;
				t.push_back(tash);
			}
		}
		else if(strequal(buf,"corners"))
		{
			fgets(buf,1009,fp);
		cnt++;
			int cn;
			sscanf(buf,"%d",&cn);
			printf("corner number : %d\n",cn);
			for(i=0;i<cn;i++)
			{
				fgets(buf,1009,fp);
			cnt++;
				int cnt = sscanf(buf,"%d",&j);
				if(cnt < 1)
				{
					i--;cn--;
					continue;
				}
				corner[j-1] = true;
			}
		}
	}
	make_neighbor();
	fclose(fp);
	printf("cnt : %d\n",cnt);
	printf("%d %d  %d %d %d  %d %d\n",tn,t.size(),pn,exam_pos[0].size(),model->vv.size(),fn,model->obj[0]->f.size());
	return true;
}
bool tetObject::loadTET(const char *fname, const char *objname)
{
	is_mesh = true;
	FILE *fp = fopen(fname,"rt");
	if(fp == NULL)
	{
		printf("file_open_fail\n");
		return false;
	}
	model = new objModel();
	model->getModel(objname);
	char buf[1009];
	int i, j, k;
	int line=0;
	int cnt=0;
			int fn;
	while(fgets(buf,1009,fp))
	{
		cnt++;
		if(buf[0] == '#') continue;
		for(i=0;buf[i];i++);
		while(i>0&&(buf[i-1]==' ' || buf[i-1] == 10 || buf[i] == 9 || buf[i] == 13)){i--;buf[i]=0;}
		if(strequal(buf,"end"))break;
		if(strequal(buf,"vertices"))
		{
			fgets(buf,1009,fp);
		cnt++;
			sscanf(buf,"%d",&pn);
			printf("pn : %d\n",pn);
			corner.clear();
			corner.resize(pn);

			en = 1;
			vector3f c;
			for(i=0;i<pn;i++)
			{
				vector3f pp;
				fgets(buf,1009,fp);
		cnt++;
				int cnt = sscanf(buf,"%lf %lf %lf",&pp.x,&pp.y,&pp.z);
				if(cnt < 3)
				{
					i--;
					pn--;
					continue;
				}
				exam_pos[0].push_back(pp);
				corner[i] = false;
				c += pp;
			}
			c /= pn;
			for(i=0;i<pn;i++)
			{
				vector3f pp = exam_pos[0][i];
				vector3f qq;
				pp -= c;
				qq.x = pp.x * cos(pp.y/30) - pp.z * sin(pp.y/30);
				qq.z = pp.x * sin(pp.y/30) + pp.z * cos(pp.y/30);
				qq.y = pp.y;
				qq += c;
				exam_pos[1].push_back(qq);
			}
		}
		else if(strequal(buf,"triangles"))
		{
			fgets(buf,1009,fp);
		cnt++;
			sscanf(buf,"%d",&fn);
			printf("face number : %d\n",fn);
			for(i=0;i<fn;i++)
			{
				fgets(buf,1009,fp);
		cnt++;
			}
		}
		else if(strequal(buf,"tetrahedra"))
		{
			fgets(buf,1009,fp);
		cnt++;
			sscanf(buf,"%d",&tn);
			printf("tn : %d\n",tn);
			
			for(i=0;i<tn;i++)
			{
				tetTash tash;
				fgets(buf,1009,fp);
		cnt++;
				int cnt = sscanf(buf,"%d %d %d %d",&tash.v0,&tash.v1,&tash.v2,&tash.v3);
				if(cnt < 4)
				{
					i--;
					tn--;
					continue;
				}
				tash.v0 -= 1;
				tash.v1 -= 1;
				tash.v2 -= 1;
				tash.v3 -= 1;
				t.push_back(tash);
			}
		}
		else if(strequal(buf,"corners"))
		{
			fgets(buf,1009,fp);
		cnt++;
			int cn;
			sscanf(buf,"%d",&cn);
			printf("cn : %d\n",cn);
			for(i=0;i<cn;i++)
			{
				fgets(buf,1009,fp);
			cnt++;
				int cnt = sscanf(buf,"%d",&j);
				if(cnt < 1)
				{
					i--;cn--;
					continue;
				}
				corner[j-1] = true;
			}
		}
	}
	make_neighbor();
	fclose(fp);
	printf("cnt : %d\n",cnt);
	printf("%d %d  %d %d %d  %d %d\n",tn,t.size(),pn,exam_pos[0].size(),model->vv.size(),fn,model->obj[0]->f.size());
	return true;
}
bool tetObject::loadXML(const string& fname)
{
	TiXmlDocument doc;
	if(!doc.LoadFile(fname.c_str()))
	{
		printf("no such file %s\n",fname.c_str());
		return false;
	}

	TiXmlElement *txe_object = doc.FirstChildElement("object");
	bool res;
	res = loadXML(txe_object);
	printf("%d\n",res);
	return res;
}
void tetObject::make_neighbor()
{
	int i, j, k;
	printf("tet neighbor_come!\n");
	neighbor = new vector<int>[pn];
	for(i=0;i<tn;i++)
	{
		neighbor[t[i].v0].push_back(t[i].v1);neighbor[t[i].v0].push_back(t[i].v2);neighbor[t[i].v0].push_back(t[i].v3);
		neighbor[t[i].v1].push_back(t[i].v0);neighbor[t[i].v1].push_back(t[i].v2);neighbor[t[i].v1].push_back(t[i].v3);
		neighbor[t[i].v2].push_back(t[i].v1);neighbor[t[i].v2].push_back(t[i].v0);neighbor[t[i].v2].push_back(t[i].v3);
		neighbor[t[i].v3].push_back(t[i].v1);neighbor[t[i].v3].push_back(t[i].v2);neighbor[t[i].v3].push_back(t[i].v0);
	}
	for(i=0;i<pn;i++)
	{
		sort(neighbor[i].begin(),neighbor[i].end());
		neighbor[i].erase(unique(neighbor[i].begin(),neighbor[i].end()),neighbor[i].end());
	}
}
bool tetObject::loadXML(const TiXmlElement* txe_object) {
	if(!txe_object){printf("object parsing fail!\n"); return false;}

	 const TiXmlElement* txe_tetmesh = txe_object->FirstChildElement("tetmesh");
    if (!txe_tetmesh)
	{
		printf("tetmesh parsing fail!\n");
        return false;
	}
    
    int n_tetrahedra = 0, n_vertices = 0, n_examples = 0;
    txe_tetmesh->QueryIntAttribute("n_tetrahedra", &n_tetrahedra);
    txe_tetmesh->QueryIntAttribute("n_vertices", &n_vertices);
    txe_tetmesh->QueryIntAttribute("n_examples", &n_examples);
    
    if (n_tetrahedra == 0 || n_vertices == 0 || n_examples == 0)
        return false;
    
    // tetrahedronlist
	tn = n_tetrahedra;
	pn = n_vertices;
	en = n_examples;
    const TiXmlElement* txe_tetlist = txe_tetmesh->FirstChildElement("tetlist");
    const TiXmlElement* txe_tet = txe_tetlist->FirstChildElement("tet");
    for (int i = 0; i < n_tetrahedra; ++i, txe_tet = txe_tet->NextSiblingElement()) {
		tetTash tet;
        txe_tet->QueryIntAttribute("v0", &tet.v0);
        txe_tet->QueryIntAttribute("v1", &tet.v1);
        txe_tet->QueryIntAttribute("v2", &tet.v2);
        txe_tet->QueryIntAttribute("v3", &tet.v3);
        t.push_back(tet);
    }
	printf("%d %d\n",tn,t.size());
    
    // pointlist
    const TiXmlElement* txe_example = txe_tetmesh->FirstChildElement("example");
    for (int i = 0; i < n_examples; ++i, txe_example = txe_example->NextSiblingElement()) {
        const TiXmlElement* txe_pos = txe_example->FirstChildElement("pos");
		exam_pos[i].resize(n_vertices);
        for (int j = 0; j < n_vertices; ++j, txe_pos = txe_pos->NextSiblingElement()) {
            vector3f& pos = exam_pos[i][j];
            txe_pos->QueryDoubleAttribute("x", &pos.x);
            txe_pos->QueryDoubleAttribute("y", &pos.y);
            txe_pos->QueryDoubleAttribute("z", &pos.z);
        }
		/*
		if(i==1)
		{
			for(int j=0;j<n_vertices;j++)
			{
				double w = exam_pos[0][j].y;
				w = cos(w*4);
				w = 2*w*w + 1;
				exam_pos[1][j].x = exam_pos[0][j].x * w;
				exam_pos[1][j].z = exam_pos[0][j].z * w;
				exam_pos[1][j].y = exam_pos[0][j].y * 0.9;
			}
		}
		*/
    }
	printf("%d %d\n",pn,exam_pos[0].size());
    // make neighbor here
	make_neighbor();
    model = new objModel();
    const TiXmlElement* txe_trimesh = txe_object->FirstChildElement("trimesh");
    if (txe_trimesh) {
        int n_faces = 0, n_trivertices = 0;
        txe_trimesh->QueryIntAttribute("n_faces", &n_faces);
        txe_trimesh->QueryIntAttribute("n_vertices", &n_trivertices);
		objObject *obj = new objObject();
        
        // vertices
        const TiXmlElement* txe_pointlist = txe_trimesh->FirstChildElement("pointlist");
        const TiXmlElement* txe_pos = txe_pointlist->FirstChildElement("pos");
        for (int i = 0; i < n_trivertices; ++i, txe_pos = txe_pos->NextSiblingElement()) {
            vector3f p;
            txe_pos->QueryDoubleAttribute("x", &p.x);
            txe_pos->QueryDoubleAttribute("y", &p.y);
            txe_pos->QueryDoubleAttribute("z", &p.z);
			model->vv.push_back(p);
        }
        // faces
        const TiXmlElement* txe_facelist = txe_trimesh->FirstChildElement("facelist");
        const TiXmlElement* txe_face = txe_facelist->FirstChildElement("face");
        for (int i = 0; i < n_faces; ++i, txe_face = txe_face->NextSiblingElement()) {
			objFace *face = new objFace();;
			face->vv.resize(3);
            txe_face->QueryIntAttribute("v0", &face->vv[0]);
            txe_face->QueryIntAttribute("v1", &face->vv[2]);
            txe_face->QueryIntAttribute("v2", &face->vv[1]);
			obj->f.push_back(face);
        }
		model->obj.push_back(obj);
    }
    /*
	later
    const TiXmlElement* txe_config = txe_object->FirstChildElement("config");
    if (txe_config) {
        double damping, stiffness_alpha, beta;
        int useLocalExample;
        txe_config->QueryDoubleAttribute("damping", &damping);
        txe_config->QueryDoubleAttribute("stiffness_alpha", &stiffness_alpha);
        txe_config->QueryIntAttribute("stiffness_iter", &m_config.m_stiffness_iter);
        txe_config->QueryIntAttribute("useLocalExample", &useLocalExample);
        txe_config->QueryDoubleAttribute("beta", &beta);
        m_config.m_damping = damping;
        m_config.m_stiffness_alpha = stiffness_alpha;
        m_config.m_useLocalExample = useLocalExample != 0;
        m_config.m_beta = beta;
    }

    const TiXmlElement* txe_initial = txe_object->FirstChildElement("initial");
    if (txe_initial) {
        const TiXmlElement* txe_offset_min = txe_initial->FirstChildElement("offset_min");
        if (txe_offset_min) {
            double x, y, z;
            txe_offset_min->QueryDoubleAttribute("x", &x);
            txe_offset_min->QueryDoubleAttribute("y", &y);
            txe_offset_min->QueryDoubleAttribute("z", &z);
//            m_initial.m_offset_min = Y::Vector3f(x, y, z);
            m_initial.m_offset = Y::Vector3f(x, y, z);
        }
        const TiXmlElement* txe_offset_max = txe_initial->FirstChildElement("offset_max");
        if (txe_offset_max) {
            double x, y, z;
            txe_offset_max->QueryDoubleAttribute("x", &x);
            txe_offset_max->QueryDoubleAttribute("y", &y);
            txe_offset_max->QueryDoubleAttribute("z", &z);
//            m_initial.m_offset_max = Y::Vector3f(x, y, z);
            m_initial.m_offset = Y::Vector3f(x, y, z);
        }
        const TiXmlElement* txe_velocity = txe_initial->FirstChildElement("velocity");
        if (txe_velocity) {
            double x, y, z;
            txe_velocity->QueryDoubleAttribute("x", &x);
            txe_velocity->QueryDoubleAttribute("y", &y);
            txe_velocity->QueryDoubleAttribute("z", &z);
            m_initial.m_velocity = Y::Vector3f(x, y, z);
        }
    }
    reset();
    */
    return true;
}