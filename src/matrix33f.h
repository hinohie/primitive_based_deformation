#ifndef __MATRIX_3X3F_H__
#define __MATRIX_3X3F_H__

#include "vector3f.h"
struct matrix33f{
	double value[3][3];
	matrix33f(){setZero();}
	matrix33f(double xx, double xy, double xz,double yx, double yy, double yz,double zx, double zy, double zz)
	{
		set(xx,xy,xz,yx,yy,yz,zx,zy,zz);
	}
	
	void set(double xx, double xy, double xz,double yx, double yy, double yz,double zx, double zy, double zz)
	{
		value[0][0] = xx;value[0][1] = xy;value[0][2] = xz;
		value[1][0] = yx;value[1][1] = yy;value[1][2] = yz;
		value[2][0] = zx;value[2][1] = zy;value[2][2] = zz;
	}
	void setZero()
	{
		set(0,0,0, 0,0,0, 0,0,0);
	}
	void setIden()
	{
		set(1,0,0, 0,1,0, 0,0,1);
	}
	matrix33f &operator = (matrix33f &p)
	{
		for(int i=0;i<3;i++)for(int j=0;j<3;j++)value[i][j] = p.value[i][j];
		return *this;
	}

	friend matrix33f operator +(matrix33f &p, matrix33f &q)
	{
		matrix33f r;
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				r.value[i][j] = p.value[i][j] + q.value[i][j];
		return r;
	}
	friend matrix33f operator -(matrix33f &p, matrix33f &q)
	{
		matrix33f r;
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				r.value[i][j] = p.value[i][j] - q.value[i][j];
		return r;
	}
	friend matrix33f operator *(matrix33f &p, matrix33f &q)
	{
		matrix33f r;
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			{
				r.value[i][j] = 0;
				for(int k=0;k<3;k++)
					r.value[i][j] += p.value[i][k] * q.value[k][j];
			}
		return r;
	}
	friend matrix33f operator *(matrix33f &p, double &q)
	{
		matrix33f r;
		for(int i=0;i<3;i++)
//			for(int j=0;j<3;j++)
			{
				r.value[i][i] = p.value[i][i] * q;
			}
		return r;
	}
	friend matrix33f operator *(double &q, matrix33f &p)
	{
		matrix33f r;
		for(int i=0;i<3;i++)
//			for(int j=0;j<3;j++)
			{
				r.value[i][i] = p.value[i][i] * q;
			}
		return r;
	}
	friend vector3f operator *(matrix33f &p, vector3f &q)
	{
		vector3f r;
		r.x += p.value[0][0] * q.x;
		r.x += p.value[0][1] * q.y;
		r.x += p.value[0][2] * q.z;
		r.y += p.value[1][0] * q.x;
		r.y += p.value[1][1] * q.y;
		r.y += p.value[1][2] * q.z;
		r.z += p.value[2][0] * q.x;
		r.z += p.value[2][1] * q.y;
		r.z += p.value[2][2] * q.z;
		
		return r;
	}
	friend vector3f operator *(vector3f &q,matrix33f &p)
	{
		vector3f r;
		r.x += p.value[0][0] * q.x;
		r.x += p.value[1][0] * q.y;
		r.x += p.value[2][0] * q.z;
		r.y += p.value[0][1] * q.x;
		r.y += p.value[1][1] * q.y;
		r.y += p.value[2][1] * q.z;
		r.z += p.value[0][2] * q.x;
		r.z += p.value[1][2] * q.y;
		r.z += p.value[2][2] * q.z;
		
		return r;
	}
	double &operator()(int i, int j)
	{
		return value[i][i];
	}

	double *operator[](int i)
	{
		return value[i];
	}
	void operator *=(double p){for(int i=0;i<3;i++)for(int j=0;j<3;j++)value[i][j] *= p;}
	void operator /=(double p){for(int i=0;i<3;i++)for(int j=0;j<3;j++)value[i][j] /= p;}
	void operator +=(matrix33f p){for(int i=0;i<3;i++)for(int j=0;j<3;j++)value[i][j] += p.value[i][j];}
	void operator *=(matrix33f p){
		matrix33f r;
		for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)
			r.value[i][j] += value[i][k] * p.value[k][j];
		for(int i=0;i<3;i++)for(int j=0;j<3;j++)value[i][j] = r.value[i][j];
	}
	void operator -=(matrix33f p){for(int i=0;i<3;i++)for(int j=0;j<3;j++)value[i][j] -= p.value[i][j];}
	matrix33f transpose() const
	{
		matrix33f r;
		
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
			{
				r.value[i][j] = value[j][i];
			}
		return r;
	}
	double determinant() const
	{
		double res=0;
		for(int i=0;i<3;i++)
		{
			int j, k;
			double tmp = 0;
			j=(i+1)%3;
			k=(i+2)%3;
			tmp = value[1][j] * value[2][k] - value[1][k] * value[2][j];
			tmp *= value[0][i];
			res += tmp;
		}
//		printf("%lf\n",res);
		return res;
	}
	bool inverse()
	{
		matrix33f r;
		double s = determinant();
		if(s < E*E && s > -E*E){
			printf("inverse fail!\n");
			return false;
		}
		s = 1/s;
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				int i1,i2;
				int j1,j2;
				i1 = (i+1)%3;
				i2 = (i+2)%3;
				j1 = (j+1)%3;
				j2 = (j+2)%3;
				r.value[j][i] = value[i1][j1] * value[i2][j2] - value[i1][j2] * value[i2][j1];
			}
		}
		r *= s;
		for(int i=0; i<3; i++)
			for(int j=0;j<3;j++)
				value[i][j] = r.value[i][j];
		return true;
	}
};

#endif