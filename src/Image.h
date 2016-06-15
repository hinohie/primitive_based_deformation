#ifndef __MESHLESS_IMAGE__
#define __MESHLESS_IMAGE__

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC

#include "stb_image.h"

#include <memory>
#include <algorithm>
#include <map>
using namespace std;

class Image{
public:
	int w, h, c;
	unsigned char *data;
	map<int, int> base_id;
	int base_n;

	Image(const char *filename)
	{
		FILE *fp = fopen(filename,"rb");
		
		long long int size;

//		GetFileSizeEx(hFile, &size);
		fseek(fp, 0, SEEK_END);
		size = ftell(fp);
		fseek(fp, 0, SEEK_SET);

		unsigned char *f_data = new unsigned char[size];
		fread(f_data,sizeof(unsigned char), size, fp);

		fclose(fp);

		data = stbi_load_from_memory(f_data, size, &w, &h, &c, 4);

		delete[] f_data;
	}
	void get_color(double x, double y, unsigned char &r, unsigned char &g, unsigned char &b, unsigned char &a)
	{
		x = 1-x;
		y = 1-y;
		int i = (int)(y * h);
		int j = (int)(x * w);
		if(i<0)i=0;if(i>=h)i=h-1;
		if(j<0)j=0;if(j>=w)j=w-1;

		int id = i*w + j;
		r = data[id*4 + 0];
		g = data[id*4 + 1];
		b = data[id*4 + 2];
		a = data[id*4 + 3];
	}
	int color_id(unsigned char r,unsigned char g,unsigned char b,unsigned char a)
	{
		int color = 0;
		color += ((int)(r)) << 0;
		color += ((int)(g)) << 8;
		color += ((int)(b)) << 16;
		color += ((int)(a)) << 24;
		return color;
	}
	int get_base_id(double x, double y)
	{
		unsigned char r,g,b,a;
		get_color(x,y,r,g,b,a);
		int color = color_id(r,g,b,a);
		if(color & 0x00FFFFFF)
			return base_id[color];
		return -1;
	}
	void set_base()
	{
		int i, j;
		base_n =0 ;
		for(i=0; i<h; i++)
			for(j=0;j<w; j++)
			{
				int id = i*w + j;
				int color = 0;
				color = color_id(data[id*4+0],data[id*4+1],data[id*4+2],data[id*4+3]);
				if((color & 0x00FFFFFF))
					if(base_id.count(color) == 0)
					{
						base_id[color] = base_n++;
					}
			}
	}
};

#endif