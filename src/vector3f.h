#ifndef __VECTOR_3F_H__
#define __VECTOR_3F_H__

#include <algorithm>
#include <vector>
#include <math.h>
#define __host__
#define __device__
using namespace std;
#define E 0.000001
#define VECTOR3F_PRINTF(v3) ((v3).x), ((v3).y), ((v3).z)

__host__ __device__ struct vector3f{
	double x,y,z;
	__host__ __device__ vector3f(){x=y=z=0;}
	__host__ __device__ vector3f(double p){x=y=z=p;}
	__host__ __device__ vector3f(double nx,double ny,double nz){x=nx;y=ny;z=nz;}
	__host__ __device__ friend vector3f operator +(vector3f p,vector3f q)
	{
		vector3f r = vector3f(p.x+q.x, p.y+q.y, p.z+q.z);
		return r;
	}
	__host__ __device__ friend vector3f operator *(double p,vector3f q)
	{
		vector3f r = vector3f(p*q.x, p*q.y, p*q.z);
		return r;
	}
	__host__ __device__ friend vector3f operator *(vector3f q,double p)
	{
		vector3f r = vector3f(p*q.x, p*q.y, p*q.z);
		return r;
	}
	__host__ __device__ friend vector3f operator /(vector3f q,double p)
	{
		vector3f r = vector3f(q.x/p, q.y/p, q.z/p);
		return r;
	}
	__host__ __device__ friend double operator *(vector3f p,vector3f q)
	{
		double r = (p.x*q.x + p.y*q.y+ p.z*q.z);
		return r;
	}
	__host__ __device__ friend vector3f operator -(vector3f p,vector3f q)
	{
		vector3f r = vector3f(p.x-q.x, p.y-q.y, p.z-q.z);
		return r;
	}
	__host__ __device__ friend vector3f operator %(vector3f p,vector3f q)
	{
		vector3f r = vector3f(p.y*q.z - p.z*q.y, p.z*q.x - p.x*q.z, p.x*q.y - p.y*q.x);
		return r;
	}
	__host__ __device__ friend vector3f operator ^(vector3f p,vector3f q)
	{
		vector3f r = vector3f(p.x*q.x, p.y*q.y, p.z*q.z);
		return r;
	}
	__host__ __device__ void operator += (const vector3f &p)			{ x += p.x;  y += p.y; z += p.z;}
	__host__ __device__ void operator -= (const vector3f &p)			{ x -= p.x;  y -= p.y; z -= p.z;}
	__host__ __device__ void operator *= (double c)						{ x *= c;  y *= c; z *= c;}
	__host__ __device__ void operator /= (double c)						{ x /= c;  y /= c; z /= c;}
	__host__ __device__ void operator ^= (const vector3f &p)			{ x *= p.x;  y *= p.y; z *= p.z;}

	__host__ __device__ double dist()
	{
		return sqrt(x*x+y*y+z*z);
	}
	__host__ __device__ double dist(vector3f p)
	{
		vector3f r = *this - p;
		return r.dist();
	}
	__host__ __device__ void normalize()
	{
		double w = dist();
		if(w < E) return;
		x /= w;
		y /= w;
		z /= w;
	}
};

struct vector2f{
	double x,y;
	vector2f(){x=y=0;}
	vector2f(double p){x=y=p;}
	vector2f(double nx,double ny){x=nx;y=ny;}
	vector2f(int nx,int ny){x=nx;y=ny;}
	friend vector2f operator +(vector2f p,vector2f q)
	{
		vector2f r = vector2f(p.x+q.x, p.y+q.y);
		return r;
	}
	friend vector2f operator *(double p,vector2f q)
	{
		vector2f r = vector2f(p*q.x, p*q.y);
		return r;
	}
	friend vector2f operator *(vector2f q,double p)
	{
		vector2f r = vector2f(p*q.x, p*q.y);
		return r;
	}
	friend vector2f operator /(vector2f q,double p)
	{
		vector2f r = vector2f(q.x/p, q.y/p);
		return r;
	}
	friend double operator *(vector2f p,vector2f q)
	{
		double r = (p.x*q.x + p.y*q.y);
		return r;
	}
	friend vector2f operator -(vector2f p,vector2f q)
	{
		vector2f r = vector2f(p.x-q.x, p.y-q.y);
		return r;
	}
	void operator += (const vector2f &p)			{ x += p.x;  y += p.y;}
	void operator -= (const vector2f &p)			{ x -= p.x;  y -= p.y;}
	void operator *= (double c)						{ x *= c;  y *= c;}
	void operator /= (double c)						{ x /= c;  y /= c;}

	double dist()
	{
		return sqrt(x*x+y*y);
	}
	double dist(vector2f p)
	{
		vector2f r = *this - p;
		return r.dist();
	}
	void normalize()
	{
		double w = dist();
		if(w < E) return;
		x /= w;
		y /= w;
	}
};
#endif