#include "Polyrect.h"

void Polyrect::sortAO()
{

	int i, j;
	int n = v.size();
	max_x = -1;
	max_y = -1;
	min_x = width;
	min_y = height;

	long t1, t2, t3;
	int x, y;
	for(i=0; i<n; i++)
	{
		v[i].x;
		v[i].y;
		if(x < 0)
		{
			x = 0;
		}
		else if(x > width)
		{
			x = width;
		}
		if(y < 0)
		{
			y = 0;
		}
		else if(y > height)
		{
			y = height;
		}
		v[i].x = x;
		v[i].y = y;
		if(max_x < x)
			max_x = x;
		if(max_y < y)
			max_y = y;
		if(min_x > x)
			min_x = x;
		if(min_y > y)
			min_y = y;
	}
}
bool Polyrect::cross(double mx,double my,vector3f p, vector3f q)
{
	double t;
	
	//ax + by = c;
	double a = q.y - p.y;
	double b = p.x - q.x;
	double c = p.x * q.y - q.x * p.y;
	
	//a(mx + t*p) + b(my + t) = c
	//dt = e
	double d = a*(max_x + 1) + b;
	double e = c - a*mx - b*my;
	t = e/d;
	if(t < 0 || t > 1)
		return false;
		double tx = mx + (max_x + 1)*t;
	double ty = my + t;
		// cross point tx,ty
	if(p.y == q.y)
	{
		t = (tx - p.x)/(q.x - p.x);
	}
	else
	{
		t = (ty - p.y)/(q.y - p.y);
	}
	
	return (t >= 0 && t <= 1);
}
bool Polyrect::PtInRegion(int x, int y)
{
	int i, j, k=0;
	j = v.size();
	if(j<3)
		return true;
	if(x > max_x)
		return false;
	if(y > max_y)
		return false;
	for(i=1; i<j; i++)
	{
		if(cross(x, y, v[i-1],v[i]))
			k++;
	}
	if(cross(x, y, v[j-1],v[0]))
		k++;
	
	return (k%2==1);
}