#include<stdlib.h>
#include<stdio.h>
#include<algorithm>
#include<vector>

#include<GL/glew.h>
#include<glut.h>
/*
#define GLSLAPI
#include<FreeImage.h>
*/
using namespace std;

#include "Pbject.h"

Pbject *pbj;

bool run_simul = true;

static int width, height;

GLenum show_type = GL_FILL;
bool show_axis = true;
bool double_pushed = false;
bool z_rotate = false;

struct Border{
public:
	int sx,sy;
	int w,h;
	Border(){sx = 0; sy = 0; w = 1; h = 1;}
	Border(int _sx, int _sy, int _w, int _h){sx = _sx; sy = _sy; w = _w; h = _h;}
	bool inside(int x,int y)
	{
		y = height - y - 1;
		if(x >= sx && x < sx + w)
			if(y >= sy && y < sy + h)
				return true;
		return false;
	}
};
int bn=7;
int border_thickness = 8;
Border border[7] = {Border(border_thickness,border_thickness,800,800 + 2*border_thickness), 
	Border(800 + 2*border_thickness,border_thickness,200,200),Border(800 + 2*border_thickness,200 + 2*border_thickness,200,200),
	Border(1000 + 3*border_thickness,border_thickness,200,200),Border(1000 + 3*border_thickness,200 + 2*border_thickness,200,200),
	Border(800 + 2*border_thickness,400 + 3*border_thickness,400 + border_thickness,400),
	Border(1200 + 4*border_thickness,border_thickness,400,800 + 2*border_thickness),
};

double x_angle[7];
double y_angle[7];
double z_angle[7];
double x_move[7];
double y_move[7];
double z_move[7];
double scale[7];

int bi5_show_id=0;

double initial_matrix[7][16];
double matrix_stack[7][16];

void reshape(int w, int h)
{
	/*
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-0.70f, 0.70f, -0.70f, 0.70f);
	*/
	width = w; height = h;
	printf("reshape %d %d\n",w,h);
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double aspect = (double)w / h;
	glFrustum(-0.01 * aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
}
/*
static int frame=0;
static int doo = 500;
void capture()
{
	if(doo< 3) {doo++; return;}
	else {doo = 0;}
	printf(" - #%d frame captured\n",frame);
//	pbj->capture();
	printf("out success");
	// Make the BYTE array, factor of 3 because it's RBG.
	BYTE* pixels = new BYTE[ 3 * width * height];

	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	// Convert to FreeImage format & save to file
	FIBITMAP* image = FreeImage_ConvertFromRawBits(pixels, width, height, 3 * width, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
	char filename[50];
	sprintf(filename,"F:/Result_Seen/Euro/Image/%05d.png",frame);
	frame++;
	FreeImage_Save(FIF_PNG, image, filename, 0);

	// Free resources
	FreeImage_Unload(image);
	delete [] pixels;
	
	if(frame > 199) exit(0);
	if(frame >= 49 & frame <= 69){
		for(auto obj : pbj->objs)
		{
			obj->change_example_rate_test(1, 0, 2, (frame - 49) / 20.0);
		}
	}
	if(frame >= 89 & frame <= 109){
		for(auto obj : pbj->objs)
		{
			obj->change_example_rate_test(2, 0, 1, (frame - 89) / 20.0);
		}
	}
	if(frame >= 129 & frame <= 139){
		for(auto obj : pbj->objs)
		{
			obj->change_example_rate_test(-1, -1, -1, (frame - 129) / 10.0);
		}
	}
	if(frame >= 154 & frame <= 184){
		for(auto obj : pbj->objs)
		{
			obj->change_example_rate_test(1, 2, 0, (frame - 154) / 30.0);
		}
	}
}
*/
static bool show_depth = true;
static bool show_normal = true;
void grid_draw()
{
	int i, j, k;
	glPushMatrix();
	glTranslatef(0,-0.1,0);
	glColor3d(0.5,0.5,0.5);
	glNormal3d(0, 1, 0);
	glBegin(GL_LINES);
	for(i=-20;i<=20;i++)
	{
		glVertex3d(-2.1,0,i/10.0);
		glVertex3d(2.1,0,i/10.0);
	}
	glEnd();
	glBegin(GL_LINES);
	for(i=-20;i<=20;i++)
	{
		glVertex3d(i/10.0,0,-2.1);
		glVertex3d(i/10.0,0,2.1);
	}
	glEnd();
	glTranslatef(0,0.1,0);
	glPopMatrix();
}
#include<time.h>
void display()
{
	double aspect;
	int cur_bi;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Scene 01 main view */
	cur_bi=0;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if(show_axis)grid_draw();
	pbj->draw();
	/*
	obj->draw_cmodel();
	obj->draw_cmodel2(vector3f(0,0,-1.2));
	obj->draw_cmodel3(vector3f(-1.2,0,-1.2));
	*/
	glPopMatrix();

	/* Scene 06 Select */
	cur_bi = 5;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.07*aspect, 0.07*aspect, -0.07, 0.07, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	pbj->draw_base();
	glPopMatrix();
	
	/* Scene 03 diffuse */
	cur_bi = 2;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.07*aspect, 0.07*aspect, -0.07, 0.07, 0.03, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	pbj->draw_diffuse();
	glPopMatrix();
	
	/* Scene 05 diffuse */
	cur_bi = 4;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.07*aspect, 0.07*aspect, -0.07, 0.07, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	pbj->draw_position();
	glPopMatrix();
	/* Scene 02 Target */
	/*
	cur_bi = 1;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
//	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	if(show_axis)grid_draw();
	obj->draw_tmodel_origin();
	glPopMatrix();
	*/

	/* Scene 03 cmodel 1 */
	/*
	cur_bi = 2;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
//	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	if(show_axis)grid_draw();
	{
		int st1 = obj->show_type_1;
		obj->draw_cmodel_origin();
		obj->show_type_1 = GL_LINE;
		obj->draw_tmodel_origin();
		obj->show_type_1 = st1;
	}
	glPopMatrix();
	*/
	
	/* Scene 04 cmodel 2 */
	/*
	cur_bi = 3;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
//	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	if(show_axis)grid_draw();
	{
		int st1 = obj->show_type_1;
		obj->draw_cmodel2_origin();
		obj->show_type_1 = GL_LINE;
		obj->draw_tmodel_origin();
		obj->show_type_1 = st1;
	}
	glPopMatrix();
	*/
	
	/* Scene 05 cmodel 3 */
	/*
	cur_bi = 4;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
//	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	if(show_axis)grid_draw();
	{
		int st1 = obj->show_type_1;
		obj->draw_cmodel3_origin();
		obj->show_type_1 = GL_LINE;
		obj->draw_tmodel_origin();
		obj->show_type_1 = st1;
	}
	glPopMatrix();
	*/
	
	/* Scene 06 cmodel 1 */
	/*
	cur_bi = 5;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
//	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	if(show_axis)grid_draw();
	{
		int st1 = obj->show_type_1;
		if(bi5_show_id == 0)obj->draw_cmodel();
		if(bi5_show_id == 1)obj->draw_cmodel2();
		if(bi5_show_id == 2)obj->draw_cmodel3();
		obj->show_type_1 = GL_LINE;
		obj->draw_tmodel();
		obj->show_type_1 = st1;
	}
	glPopMatrix();
	*/
	/* Scene 07 Example */
	/*
	cur_bi = 6;
	glPushMatrix();
	aspect = (double)border[cur_bi].w / border[cur_bi].h;
	glViewport(border[cur_bi].sx, border[cur_bi].sy, border[cur_bi].w, border[cur_bi].h);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-0.01*aspect, 0.01*aspect, -0.01, 0.01, 0.05, 180.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glScalef(scale[cur_bi],scale[cur_bi],scale[cur_bi]);
//	glTranslatef(x_move[cur_bi], y_move[cur_bi],z_move[cur_bi]);
	glRotatef(z_angle[cur_bi],0,0,1);
	glRotatef(x_angle[cur_bi],0,1,0);
	glRotatef(y_angle[cur_bi],1,0,0);
	glMultMatrixd(matrix_stack[cur_bi]);
	
	glPolygonMode(GL_FRONT_AND_BACK,show_type);
	glEnable(GL_POINT_SPRITE_ARB);
	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//	if(show_axis)grid_draw();
	obj->draw_example();
	glPopMatrix();
	*/
	
	glViewport(0, 0, width, height);
	aspect = (double) width/height;
	glPushMatrix();


	if(pbj && pbj->objs.size() && pbj->objs[0]->attack)
	{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1, 1, -1, 1, 0, 2);
	
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	glColor3f(0, 1, 0);
	glNormal3d(0, 0, 1);
	glBegin(GL_QUADS);
	glVertex3d(0.7, 0.7, 0);
	glVertex3d(0.75, 0.7, 0);
	glVertex3d(0.75, 0.75, 0);
	glVertex3d(0.7, 0.75, 0);
	glEnd();
	glBegin(GL_TRIANGLES);
	glVertex3d(0.65, 0.75, 0);
	glVertex3d(0.8, 0.75, 0);
	glVertex3d(0.725, 0.8, 0);
	glEnd();
	}
	glPopMatrix();
	
	glPushMatrix();
	
	aspect = (double)width / height;
	glViewport(0, 0, width, height);
	glPolygonMode(GL_BACK, GL_LINE);
	glCullFace(GL_FRONT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, 0, height, 0, 2);
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 1,
			 0, 0, 0,
			0, 1 ,0);
	// draw border()
	int xl,yl;
	int x[20];
	int y[20];
	xl=2; x[0] = 0; x[1] = width;
	yl=2; y[0] = 0; y[1] = height;
	for(int i=0; i<bn; i++)
	{
		x[xl++] = border[i].sx;
		x[xl++] = border[i].sx + border[i].w;
		y[yl++] = border[i].sy;
		y[yl++] = border[i].sy + border[i].h;
	}
	sort(x,x+xl);sort(y,y+yl);
	xl = unique(x,x+xl)-x;
	yl = unique(y,y+yl)-y;
	glColor3d(1,1,1);
	glNormal3d(0,0,-1);
	for(int i=0; i+1<xl; i++)
		for(int j=0; j+1<yl; j++)
		{
			int k;
			for(k=0;k<bn;k++)
				if(border[k].inside((x[i]+x[i+1])/2,height - (y[j]+y[j+1])/2))
					break;
			glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
			if(k==bn)
			{
				glBegin(GL_QUADS);
					glVertex3d(x[i], y[j],0);
					glVertex3d(x[i], y[j+1],0);
					glVertex3d(x[i+1], y[j+1],0);
					glVertex3d(x[i+1], y[j],0);
				glEnd();
			}

		}
	glPopMatrix();
	//capture();
	glutSwapBuffers();
}
static int cntt=0;
void update()
{
	if(cntt == 180)
	{
//		obj->attack ^= true;
		cntt = 0;
	}
	else
	{
		cntt++;
	}
	if(run_simul)
	{
		pbj->simul();
	}
	glutPostRedisplay();
}

void MultMatrix4fv(double *p,double *q)
{
	int i, j, k;
	double r[16];
	for(i=0; i<4; i++)
		for(j=0; j<4; j++)
		{
			r[i*4+j] = 0;
			for(k=0;k<4; k++)
			{
				r[i*4+j] += p[i*4+k] * q[k*4+j];
			}
		}
	for(i=0;i<16;i++)
		p[i]=r[i];
}

GLuint Object::show_type_1 = GL_FILL;
GLuint Object::show_type_2 = GL_POINT;
void data_inil();
static unsigned char pre_pushed_key = 'w';
static int ccnt = 0;
void Ckeyboard(unsigned char key, int x, int y)
{
	int cnt=10;
	int pnt = 1;
	int i, j, k;
	switch(key){
	case 'P':
		key += ('a'-'A');
	case 'p':
		show_type = GL_POINT;
		break;
	case 'W':
		key += ('a'-'A');
	case 'w':
		show_type = GL_LINE;
		break;
	case 'S':
		key += ('a'-'A');
	case 's':
		show_type = GL_FILL;
		break;
	case 27:
		for(int bi=0; bi < bn; bi++)
		{
		x_angle[bi] = y_angle[bi] = z_angle[bi] = 0;
		x_move[bi] = y_move[bi] = z_move[bi] = 0;
		for(int i=0; i<16; i++)
			matrix_stack[bi][i] = initial_matrix[bi][i];
		}
		break;
	case 'X':
		key += ('a'-'A');
	case 'x':
		show_axis ^= true;
		break;
	case 'Z':
		key += ('a'-'A');
	case 'z':
		z_rotate ^= true;
		break;
	case ' ':
		run_simul ^= true;
		break;
	case 'r':
	case 'R':
//		for(auto obj : pbj->objs)
//			obj->inil();
		pbj->inil();
		break;
	case 'q':
	case 'Q':
		exit(0);
		break;
	case 'g':
	case 'G':
		show_depth ^= true;
		for(auto obj : pbj->objs)
		switch(obj->show_type_1)
		{
		case GL_FILL:
			obj->show_type_1 = GL_POINT;
			break;
		case GL_POINT:
			obj->show_type_1 = GL_LINE;
			break;
		case GL_LINE:
			obj->show_type_1 = GL_FILL;
			break;
		}
		break;
	case 'h':
	case 'H':
		show_normal ^= true;
		for(auto obj : pbj->objs)
		switch(obj->show_type_2)
		{
		case GL_FILL:
			obj->show_type_2 = GL_POINT;
			break;
		case GL_POINT:
			obj->show_type_2 = GL_LINE;
			break;
		case GL_LINE:
			obj->show_type_2 = GL_FILL;
			break;
		}
		break;
	case 't':
	case 'T':
		for(auto obj : pbj->objs)
		obj->attack ^= true;
		break;
	case 'c':
	case 'C':
		for(auto obj : pbj->objs)
		{
		obj->target_num = 0;
		obj->target_cheat += 0.01;
		}
		break;
	case 'v':
	case 'V':
		for(auto obj : pbj->objs)
		{
		obj->target_num = 1;
		obj->target_cheat -= 0.01;
//		if(obj->target_cheat < 0) obj->target_cheat = 0;
		}
		break;
	case 'o':
	case 'O':
		for(auto obj : pbj->objs)
		{
		obj->target_cheat += 0.01;
		}
		break;
	case 'i':
	case 'I':
		for(auto obj : pbj->objs)
		{
		obj->target_cheat -= 0.01;
		}
//		if(obj->target_cheat < 0) obj->target_cheat = 0;
		break;
	case '1':
		
		for(auto obj : pbj->objs)
		obj->simul();
		break;
	}
	if(pre_pushed_key == key)
	{
		if(key == 'w' || key == 's' || key == 'p')
		double_pushed ^= true;
	}
	else
		double_pushed = false;
	pre_pushed_key = key;
}
void Skeyboard(int key, int x,int y)
{
}
void Pmouse(int x,int y)
{

}
static int sx, sy;
static int down_mode=-1;
static int bi = -1;
void Cmouse(int button, int state, int x,int y)
{
	if(state == GLUT_DOWN)
	{
		if(down_mode<0 && button == GLUT_LEFT_BUTTON)
			down_mode = GLUT_LEFT_BUTTON;
		if(down_mode<0 && button == GLUT_RIGHT_BUTTON)
			down_mode = GLUT_RIGHT_BUTTON;
		if(down_mode<0 && button == GLUT_MIDDLE_BUTTON)
			down_mode = GLUT_MIDDLE_BUTTON;
		for(bi=0;bi<bn;bi++) if(border[bi].inside(x, y)) break;
		if(bi == bn) return;
	}
	else if(down_mode >= 0){
		double temp_stack[16];
		glPushMatrix();
		glLoadIdentity();
		glScalef(scale[bi],scale[bi],scale[bi]);
		if(bi==5){if(down_mode == GLUT_LEFT_BUTTON)bi5_show_id = (bi5_show_id + 1)%3;}
//		else if(bi==6){if(down_mode == GLUT_LEFT_BUTTON)obj->example_id_test = (obj->example_id_test)%(obj->tmodel->en-1) + 1;}
		else if(bi == 0)
		glTranslatef(x_move[bi], y_move[bi], z_move[bi]);
		glRotatef(z_angle[bi],0,0,1);
		glRotatef(x_angle[bi],0,1,0);
		glRotatef(y_angle[bi],1,0,0);
		glGetDoublev(GL_MODELVIEW_MATRIX, temp_stack);
		glPopMatrix();
		MultMatrix4fv(matrix_stack[bi], temp_stack);

		down_mode = -1;
		x_angle[bi] = y_angle[bi] = z_angle[bi] = 0;
		x_move[bi] = y_move[bi] = z_move[bi] = 0;
		scale[bi] = 1.0;
		bi = -1;
	}
	sx = sy = -1;
}
void Mmouse(int x,int y)
{
	int tx, ty;
	if(bi >= bn || bi < 0) return;
	if(sx<0 || sy<0){tx=0, ty=0;}
	else{tx = x-sx; ty = y-sy;}
	if(down_mode == GLUT_RIGHT_BUTTON)
	{
		x_angle[bi] += tx / 10.0;
		if(z_rotate) z_angle[bi] += ty/10.0;
		else y_angle[bi] += ty / 10.0;
	}
	else if(down_mode == GLUT_LEFT_BUTTON)
	{
		x_move[bi] += tx / 1000.0;
		if(z_rotate) z_move[bi] -= ty / 1000.0;
		else y_move[bi] -= ty / 1000.0;
	}
	else if(down_mode == GLUT_MIDDLE_BUTTON)
	{
		scale[bi] *= exp((tx - ty)/100.0);
	}
	sx = x;
	sy = y;
}

void data_inil()
{
	glPushMatrix();
	glLoadIdentity();
	glScalef(0.03,0.03,0.03);
	glTranslated(1.2,-1,0);
	glRotatef(0,0,0,1);
	glRotatef(50,1,0,0);
	glRotatef(-5,0,1,0);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[0]);
	glPopMatrix();
	glPushMatrix();
	glLoadIdentity();
	glScalef(0.35,0.35,0.35);
	glTranslated(-0.5,-0.5,-0.5);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[1]);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[2]);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[3]);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[4]);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[5]);
	glGetDoublev(GL_MODELVIEW_MATRIX, initial_matrix[6]);
	glPopMatrix();
	for(int bi=0; bi<bn; bi++)
	{
		scale[bi] = 1;
	for(int i=0; i<16; i++)
		matrix_stack[bi][i] = initial_matrix[bi][i];
	}

	pbj = new Pbject();
	pbj->inil();

//	for(auto obj : pbj->objs)
//		obj->attack ^= true;

}
void gl_inil()
{
	int i, j;
	glEnable(GL_DEPTH_TEST);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);
	
	GLfloat LightPosition[4] = {-8.8, 1.2, -0.2, 1.0};
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	GLfloat LightAmbient[4]  = {0.2, 0.2, 0.2, 0.2};
	GLfloat LightDiffuse[4]  = {1.0, 1.0, 1.0, 1.0};
	GLfloat LightSpecular[4] = {1.0, 1.0, 1.0, 1.0};

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);


}
int main(int argc, char **argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(1600 + 5*border_thickness, 800 + 4*border_thickness);

	glutCreateWindow("MDBtest");

	data_inil();
	gl_inil();
	glClearColor(1.0, 1.0, 1.0, 1.0);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(update);
	
	glutKeyboardFunc( Ckeyboard );
	glutSpecialFunc( Skeyboard );
	glutPassiveMotionFunc( Pmouse );
	glutMouseFunc( Cmouse );
	glutMotionFunc( Mmouse );

	glutMainLoop();

	return 0;
}