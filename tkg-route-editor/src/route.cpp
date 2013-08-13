#include <iostream>
#include <ctime> 
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <complex>
#include <unistd.h>
#include <GL/freeglut.h>
#include "tkgutil.hpp"
#include "gnd-opsm.hpp"
using namespace std;

typedef complex<double> point;

point pointmin(const point &p, const point &q)
{
	return point(min(p.real(),q.real()), min(p.imag(),q.imag()));
}

point pointmax(const point &p, const point &q)
{
	return point(max(p.real(),q.real()), max(p.imag(),q.imag()));
}

struct WayPoint
{
	char f; int a; point p; bool cp;
	WayPoint(char f, point p) : f(f), a(0), p(p), cp(false) {}
};

int wid  = 800;
int hei1 =  20;
int hei2 = 580;

int mouse_btn = -1;
int mouse_px =  0;
int mouse_py =  0;
int mouse_pt =  0;

const int    dclick_range = 10;
const double dclick_time = 0.3;

int edit_mode  =  0;
int edit_file  =  0;

const double select_radius = 0.1;
int   select_idx = -1;

bool  nl_flag  = false;
point nl_point = point(0,0);

double scale = 0.1;
double pos_x = 0.0;
double pos_y = 0.0;

double menu_x = -1.0;
double menu_y = -1.0;

int disp_line = 1;
int disp_edge = 1;
int disp_way  = 3;
int disp_grid = 1;

string cmd;
vector<vector<point> > linemap,edgemap;
vector<WayPoint> route;
vector<point>    spur;
vector<point>    gridmap;
vector<short>    gridval;

string map_path = "";
string map_name = "";

const char *menu_str_way [] = {"*non-display","*number","*flag","*flag+number"};
const char *menu_str_map [] = {"*non-display","*mono",  "*color"};
const char *menu_str_grid[] = {"*non-display","*dark-red",  "*red"};
const char   *adjust_mode[] = {"Odometry","LineOnly","EdgeOnly","Both"};

const int type_route_push = 1;
const int type_route_move = 2;
const int type_editmap_push  = 3;
const int type_editmap_move  = 4;
const int type_editmap_erase = 5;
struct Undo
{
	int type;
	vector<int>    data_i;
	vector<double> data_f;
};
vector<Undo> history;

void route_push(const WayPoint &w)
{
	Undo u;
	u.type = type_route_push;
	history.push_back(u);
	route.push_back(w);
}

void undo_route_push(const Undo &u)
{
	if(!route.empty()) route.pop_back();
}

void route_move(int idx)
{
	Undo u;
	u.type = type_route_move;
	u.data_i.push_back(idx);
	u.data_i.push_back(route[idx].f);
	u.data_i.push_back(route[idx].a);
	u.data_i.push_back(route[idx].cp);
	u.data_f.push_back(route[idx].p.real());
	u.data_f.push_back(route[idx].p.imag());
	history.push_back(u);
}

void undo_route_move(const Undo &u)
{
	int idx = u.data_i[0];
	if(0<=idx && idx<(signed)route.size())
	{
		WayPoint w(0,0);
		w.f  = u.data_i[1];
		w.a  = u.data_i[2];
		w.cp = u.data_i[3];
		w.p  = point(u.data_f[0], u.data_f[1]);
		route[idx] = w;
	}
}

void editmap_move(int idx)
{
	Undo u;
	vector<point> &editmap = (edit_mode==1) ? linemap[edit_file] : edgemap[edit_file];

	u.type = type_editmap_move;
	u.data_i.push_back(idx);
	u.data_i.push_back(edit_mode);
	u.data_i.push_back(edit_file);
	u.data_f.push_back(editmap[idx].real());
	u.data_f.push_back(editmap[idx].imag());
	history.push_back(u);
}

void undo_editmap_move(const Undo &u)
{
	int idx        = u.data_i[0];
	int tedit_mode = u.data_i[1];
	int tedit_file = u.data_i[2];
	vector<point> &editmap = (tedit_mode==1) ? linemap[tedit_file] : edgemap[tedit_file];
	if(0<=idx && idx<(signed)editmap.size())
	{
		editmap[idx] = point(u.data_f[0], u.data_f[1]);
	}
}

void editmap_push(const point &pos)
{
	if(!nl_flag)
	{
		nl_flag  = true;
		nl_point = pos;
		return;
	}

	Undo u;
	vector<point> &editmap = (edit_mode==1) ? linemap[edit_file] : edgemap[edit_file];

	nl_flag = false;
	u.type = type_editmap_push;
	u.data_i.push_back(edit_mode);
	u.data_i.push_back(edit_file);
	history.push_back(u);
	editmap.push_back(nl_point);
	editmap.push_back(pos);		
}

void undo_editmap_push(const Undo &u)
{
	int tedit_mode = u.data_i[0];
	int tedit_file = u.data_i[1];
	vector<point> &editmap = (tedit_mode==1) ? linemap[tedit_file] : edgemap[tedit_file];
	if(!editmap.empty()) editmap.pop_back();
	if(!editmap.empty()) editmap.pop_back();
}

void editmap_erase(int idx)
{
	Undo u;
	vector<point> &editmap = (edit_mode==1) ? linemap[edit_file] : edgemap[edit_file];

	idx = (idx/2)*2;
	u.type = type_editmap_erase;
	u.data_i.push_back(idx);
	u.data_i.push_back(edit_mode);
	u.data_i.push_back(edit_file);
	u.data_f.push_back(editmap[idx+0].real());
	u.data_f.push_back(editmap[idx+0].imag());
	u.data_f.push_back(editmap[idx+1].real());
	u.data_f.push_back(editmap[idx+1].imag());
	history.push_back(u);
	editmap.erase(editmap.begin() + idx, editmap.begin() + idx + 2);
}

void undo_editmap_erase(const Undo &u)
{
	int idx        = u.data_i[0];
	int tedit_mode = u.data_i[1];
	int tedit_file = u.data_i[2];
	vector<point> &editmap = (tedit_mode==1) ? linemap[tedit_file] : edgemap[tedit_file];
	if(0<=idx && idx<=(signed)editmap.size()) // イテレータはendがあるため<=size
	{
		editmap.insert(editmap.begin() + idx, point(u.data_f[2], u.data_f[3]));
		editmap.insert(editmap.begin() + idx, point(u.data_f[0], u.data_f[1]));
	}
}


void undo()
{
	if(history.empty()) return;

	Undo u = history.back();
	history.pop_back();

	switch(u.type)
	{
		case type_route_push:    undo_route_push(u);    break;
		case type_route_move:    undo_route_move(u);    break;
		case type_editmap_push:  undo_editmap_push(u);  break;
		case type_editmap_move:  undo_editmap_move(u);  break;
		case type_editmap_erase: undo_editmap_erase(u); break;
	}

	glutPostRedisplay();
}

vector<double> convert_color(const string &str)
{
	vector<double> ret(3);
	for(int i=0; i<(signed)min(2*ret.size(),str.size()); i++)
	{
		ret[i/2] *= 16;
		ret[i/2] += str[i]<'A' ? str[i]-'0' : str[i]-'A'+10;
	}
	for(int i=0; i<(signed)ret.size(); i++) { ret[i] /= 255; }
	return ret;
}

void myString(const point &p, const char *str)
{
	glRasterPos2d(p.real(), p.imag());
	for(int i=0; str[i]; i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[i]);
	};
}

void myVertex(const point &p)
{
	glVertex2d(p.real(), p.imag());
}

void myCircle(point p, double r)
{
	const int CDIV = 32;

	glBegin(GL_LINE_STRIP);
	for(int i=0; i<CDIV; i++)
	{
		myVertex(p + polar(r, 2*i*M_PI/CDIV));
	}
	myVertex(p + polar(r));
	glEnd();
}

void display_command()
{
  glColor3d(0.3, 0.3, 0.3);
  glBegin(GL_POLYGON);
  glVertex2d(-1, -1);
  glVertex2d( 1, -1);
  glVertex2d( 1,  1);
  glVertex2d(-1,  1);
  glEnd();

	string edit_mode_str;
	if(edit_mode==0) edit_mode_str = tkg::strprintf("route > ");
	if(edit_mode==1) edit_mode_str = tkg::strprintf("line[%02d] > ", edit_file);
	if(edit_mode==2) edit_mode_str = tkg::strprintf("edge[%02d] > ", edit_file);
  	glColor3d(0.0, 1.0, 0.0);
	myString(point(-0.995,-0.6), (edit_mode_str+cmd).c_str());
}

void display_route()
{
	cout << "display_route" << endl;

	// spur
	glBegin(GL_LINE_STRIP);
	if(edit_mode) glColor3d(0.0, 0.0, 0.3);
	else          glColor3d(0.0, 0.0, 1.0);
	for(int i=0; i<(signed)spur.size(); i++)
	{
		myVertex(spur[i]);
	}
	glEnd();

	// route
	glBegin(GL_LINES);
	for(int i=1; i<(signed)route.size(); i++)
	{

		glColor3d(0.3, 0.3, 0.3);
		if(edit_mode==0)
		{
			if(route[i-1].a==0) glColor3d(1.0, 1.0, 1.0);
			if(route[i-1].a==1) glColor3d(1.0, 0.0, 0.0);
			if(route[i-1].a==2) glColor3d(0.0, 1.0, 0.0);
			if(route[i-1].a==3) glColor3d(1.0, 1.0, 0.0);
		}
		myVertex(route[i-1].p);
		myVertex(route[i  ].p);
	}
	glEnd();
}

void display_route_vertex()
{
	glBegin(GL_POINTS);
	for(int i=0; i<(signed)route.size(); i++)
	{
		glColor3d(1.0, 1.0, 0.0);
		if(route[i].cp    ) glColor3d(1.0, 0.0, 0.0);
		if(route[i].f!='A') glColor3d(0.0, 1.0, 1.0);
		myVertex(route[i].p);
	}
	glEnd();

	for(int i=0; i<(signed)route.size(); i++)
	{
		glColor3d(1.0, 1.0, 0.0);
		if(route[i].cp    ) glColor3d(1.0, 0.0, 0.0);
		if(route[i].f!='A') glColor3d(0.0, 1.0, 1.0);

		char str[64] = "";
		if(disp_way==1) sprintf(str, "%d", i);
		if(disp_way==2) sprintf(str, "%c", route[i].f);
		if(disp_way==3) sprintf(str, "%c%d", route[i].f, i);
		myString(route[i].p, str);
	}
}

void display_route_line()
{
	glBegin(GL_LINES);
	for(int i=0; i<(signed)linemap.size(); i++)
	{
		if(disp_line==1) glColor3d(0.7, 0.7, 0.7);
		for(int j=0; j<(signed)linemap[i].size(); j++)
		{
			myVertex(linemap[i][j]);
		}
	}
	glEnd();
}

void display_route_edge()
{
	glBegin(GL_LINES);
	for(int i=0; i<(signed)edgemap.size(); i++)
	{
		if(disp_edge==1) glColor3d(0.7, 0.7, 0.7);
		for(int j=0; j<(signed)edgemap[i].size(); j++)
		{
			myVertex(edgemap[i][j]);
		}
	}
	glEnd();
}

void display_edit_line()
{
	glColor3d(0.0, 0.5, 0.5);
	glBegin(GL_LINES);
	for(int i=0; i<(signed)linemap.size(); i++)
	{
		bool target = (edit_mode==1 && edit_file==i);
		if(disp_line==0 && !target) continue;

		if(target) glColor3d(1.0, 0.0, 1.0);
		else if(disp_line==1) glColor3d(1.0, 1.0, 1.0);
		else if(disp_line==2) glColor3d(0.0, 1.0, 1.0);
		for(int j=0; j<(signed)linemap[i].size(); j++)
		{
			myVertex(linemap[i][j]);
		}
	}
	glEnd();
}

void display_edit_edge()
{
	glBegin(GL_LINES);
	for(int i=0; i<(signed)edgemap.size(); i++)
	{
		bool target = (edit_mode==2 && edit_file==i);
		if(disp_edge==0 && !target) continue;

		if(target) glColor3d(1.0, 0.0, 1.0);
		else if(disp_edge==1) glColor3d(1.0, 1.0, 1.0);
		else if(disp_edge==2) glColor3d(0.0, 1.0, 0.0);

		for(int j=0; j<(signed)edgemap[i].size(); j++)
		{
			myVertex(edgemap[i][j]);
		}
	}
	glEnd();
}

void display_edit_vertex()
{
	vector<point> &editmap = (edit_mode==1) ? linemap[edit_file] : edgemap[edit_file];
	glBegin(GL_POINTS);
	for(int i=0; i<(signed)editmap.size(); i++)
	{
		glColor3d(1.0, 0.0, 1.0);
		myVertex(editmap[i]);
	}
	if(nl_flag) myVertex(nl_point);
	glEnd();
}

void display_gridmap()
{

	glBegin(GL_POINTS);
	for(int i=0; i<(signed)gridmap.size(); i++)
	{
		double val = gridval[i] / 255.0;
		glColor3d(val, val, val);
		myVertex(gridmap[i]);
	}
	glEnd();
}

void display()
{
	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0,0,wid,hei1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
	display_command();

	double asp = (double)wid/hei2;
	glViewport(0,hei1,wid,hei2);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(pos_x - asp/scale, pos_x + asp/scale,
	        pos_y - 1.0/scale, pos_y + 1.0/scale, -1.0, 1.0);

	// origin
	glBegin(GL_LINES);
	glVertex2d(0, 1.0); glVertex2d(0, -1.0);
	glVertex2d(1.0, 0); glVertex2d(-1.0, 0);
	glEnd();

	glPointSize(3);
	if(disp_grid)  display_gridmap();
	glPointSize(5);
	display_route();
	if(edit_mode==0)
	{
		if(disp_line) display_route_line();
		if(disp_edge) display_route_edge();
		display_route_vertex();
	}
	else
	{
		display_edit_line();
		display_edit_edge();
		display_edit_vertex();
	}

	glutSwapBuffers();
}


void cmdAnalyze()
{
	string str;
	istringstream sin(cmd);

	sin >> str;

	if(edit_mode == 0)
	{
		if(str=="erase")
		{
			if(sin >> str)
			{
				int t = atoi(str.c_str());
				if(0<=t && t<(signed)route.size())
				{
					route.erase(route.begin()+t);
				}
			}
		}
		else if(str=="insert")
		{
			if(sin >> str)
			{
				int t = atoi(str.c_str());
				if(1<=t && t<(signed)route.size())
				{
					point pos = (route[t-1].p + route[t].p)/2.0;
					route.insert(route.begin()+t, WayPoint('A',pos));
				}
			}
		}
		else if(str=="clear")
		{
			route.clear();
		}
		else if(str=="way")
		{
			if(sin >> str)
			{
				int t = atoi(str.c_str());
				if(0<=t && t<(signed)route.size()) route[t].f = 'A';
			}
		}
		else if(str=="stop")
		{
			if(sin >> str)
			{
				int t = atoi(str.c_str());
				if(0<=t && t<(signed)route.size()) route[t].f = 'C';
			}
		}
		else if(str=="goal")
		{
			if(sin >> str)
			{
				int t = atoi(str.c_str());
				if(0<=t && t<(signed)route.size()) route[t].f = 'E';
			}
		}
		else if(str=="odm" || str=="line" || str=="edge" || str=="both")
		{
			string str1,str2;
			if(sin >> str1 >> str2)
			{
				int s = atoi(str1.c_str());
				int t = atoi(str2.c_str());
				if(0<=s && s<(signed)route.size() && 0<=t && t<(signed)route.size())
				{
					int val = 0;
					if(str=="odm" ) val=0;
					if(str=="line") val=1;
					if(str=="edge") val=2;
					if(str=="both") val=3;
					for(int i=s; i<t; i++) { route[i].a=val; }
				}
			}
		}
		else if(str=="change")
		{
			if(sin >> str)
			{
				int t = atoi(str.c_str());
				if(0<=t && t<(signed)route.size()) route[t].cp = !route[t].cp;
			}
		}
	}

	if(str=="editroute")
	{
		edit_mode = edit_file = 0; nl_flag = false;
	}

	else if(str=="save")
	{
		ofstream fout;

		fout.open( (map_path+map_name+"keiro.dat").c_str() );
		if(fout)
		{
			for(int i=0; i<(signed)route.size(); i++)
			{
				fout << route[i].f << " " << route[i].p.real() << " " << route[i].p.imag() << endl;
			}
			fout.close();
		}	
	}
}

void keyboard(unsigned char key, int x, int y)
{
	if(key==0x1b) { glutLeaveMainLoop(); }

	if(key==0x1a) { undo(); }

	if(0x20<=key && key<=0x7e)
	{
		cmd += (char)key;
		glutPostRedisplay();
	}
	else if(key==0x08)
	{
		if(!cmd.empty())
		{
			cmd.erase(cmd.size()-1);
			glutPostRedisplay();
		}
	}
	else if(key==0x0d)
	{
		cmdAnalyze();
		cmd.erase();
		glutPostRedisplay();
	}
}

void mouse_route(int button, int state, point pos)
{
	double mindist=1e10;
	for(int i=0; i<(signed)route.size(); i++)
	{
		double dist = abs(pos-route[i].p);
		if(dist < select_radius/scale)
		{
			if(dist<mindist) { select_idx=i; mindist=dist; }
		}
	}

	if(state==0)
	{
		if(select_idx!=-1) route_move(select_idx);
	}
	if(state==2)
	{
		if(select_idx==-1) route_push( WayPoint('A', pos) );
		select_idx=-1;
	}
	if(state==1) { select_idx=-1; }
}


void mouse_edit(int button, int state, point pos)
{
	double mindist=1e10;
	vector<point> &editmap = (edit_mode==1) ? linemap[edit_file] : edgemap[edit_file];
	for(int i=0; i<(signed)editmap.size(); i++)
	{
		double dist = abs(pos-editmap[i]);
		if(dist < select_radius/scale)
		{
			if(dist<mindist) { select_idx=i; mindist=dist; }
		}
	}

	if(state==0)
	{
		if(select_idx!=-1) editmap_move(select_idx);
	}
	if(state==2)
	{
		if(select_idx==-1) editmap_push(pos);
		else               editmap_erase(select_idx);
		select_idx=-1;
	}
	if(state==1) { select_idx=-1; }
}


void mouse(int button, int state, int x, int y)
{
	if(state==0)
	{
		int mt = clock();
		if((double)(mt-mouse_pt)/CLOCKS_PER_SEC < dclick_time)
		{
			if(abs(x-mouse_px)<dclick_range && abs(y-mouse_py)<dclick_range) state=2;
		}
		mouse_pt = (state==2 ? 0 : mt);
	}

	if(button==3) scale/=1.2;
	if(button==4) scale*=1.2;

	double asp = (double)wid/hei2;
	double px =  (2.0*x/wid -1.0)*asp/scale + pos_x;
	double py = -(2.0*y/hei2-1.0)*1.0/scale + pos_y;

	if(button == 2)
	{
		if(edit_mode) mouse_edit (button, state, point(px,py));
		else          mouse_route(button, state, point(px,py));
	}

	if(button==0 || button==2)
	{
		mouse_btn = (state==1 ? -1 : button);
		mouse_px = x; mouse_py = y;
	}

	glutPostRedisplay();
}

void motion_route(point pos)
{
	if(0<=select_idx && select_idx<(signed)route.size())
	{
		route[select_idx].p = pos;
	}

	glutPostRedisplay();
}

void motion_edit(point pos)
{
	vector<point> &editmap = (edit_mode==1) ? linemap[edit_file] : edgemap[edit_file];
	if(0<=select_idx && select_idx<(signed)editmap.size())
	{
		editmap[select_idx] = pos;
	}

	glutPostRedisplay();
}

void motion(int x, int y)
{
	if(mouse_btn==0)
	{
		double asp = (double)wid/hei2;
		pos_x += 2.0*asp/scale*(mouse_px-x)/wid;
		pos_y -= 2.0*1.0/scale*(mouse_py-y)/hei2;	
		glutPostRedisplay();
	}

	double asp = (double)wid/hei2;
	double px =  (2.0*x/wid -1.0)*asp/scale + pos_x;
	double py = -(2.0*y/hei2-1.0)*1.0/scale + pos_y;

	if(mouse_btn == 2)
	{
		if(edit_mode) motion_edit (point(px,py));
		else          motion_route(point(px,py));
	}

	mouse_px = x; mouse_py = y;
}

void menu(int val)
{

}

void menu_way(int val)
{
	glutChangeToMenuEntry(disp_way+1, menu_str_way[disp_way]+1, disp_way+1);
	disp_way = val-1;
	glutChangeToMenuEntry(disp_way+1, menu_str_way[disp_way], disp_way+1);
}

void menu_line(int val)
{
	glutChangeToMenuEntry(disp_line+1, menu_str_map[disp_line]+1, disp_line+1);
	disp_line = val-1;
	glutChangeToMenuEntry(disp_line+1, menu_str_map[disp_line], disp_line+1);
}

void menu_edge(int val)
{
	glutChangeToMenuEntry(disp_edge+1, menu_str_map[disp_edge]+1, disp_edge+1);
	disp_edge = val-1;
	glutChangeToMenuEntry(disp_edge+1, menu_str_map[disp_edge], disp_edge+1);
}

void menu_grid(int val)
{
	glutChangeToMenuEntry(disp_grid+1, menu_str_grid[disp_grid]+1, disp_grid+1);
	disp_grid = val-1;
	glutChangeToMenuEntry(disp_grid+1, menu_str_grid[disp_grid], disp_grid+1);
}

void reshape(int w, int h)
{
	wid=w; hei2=h-hei1;
	glutPostRedisplay();
}

void idle()
{

}

bool optAnalyze(int argc, char **argv)
{
	int opt;

	string map_path;
	ifstream fin("path.conf");
	fin >> map_path;
	if(map_path.size() > 0){
		map_path = map_path +  "/";
	}
	fin.close();

	while((opt = getopt(argc, argv, "m:h")) != -1)
	{
		switch(opt)
		{
			case 'm':
				map_name = optarg;
				if(map_name[map_name.size()-1]!='/') map_name+='/';
				break;

			case 'h':	
				cerr << endl
				<< "\t[options]"  << endl
				<< "\tm <FileName> : data.map." << endl;
				return false;

			default: // (opt=='?')
				cerr << "help : " << argv[0] << " -h" << endl;
				return false;
		}
	}
	return true;
}

void loadconfig()
{
	ifstream fin("path.conf");
	if(fin) {
		fin >> map_path;
		if(map_path.size() > 0){
			map_path = map_path +  "/";
		}
		fin.close();
	}
}

int main(int argc, char **argv)
{
	if(!optAnalyze(argc, argv)) return 1;
	loadconfig();
	

	if(map_name != "")
	{
		cout << "Map: " << map_path << map_name << endl;

//		char fn[256];
		ifstream fin;
		//point pmax(-1e10),pmin(+1e10);
		point pmax(1e1),pmin(-1e1);

		point center = (pmin+pmax)/2.0;
		pos_x = center.real();
		pos_y = center.imag();
		scale = 2 / max((pmax-pmin).real(), (pmax-pmin).imag());

		WayPoint w('A', 0);
		fin.open((map_path+map_name+"keiro.dat").c_str());
		if(fin)
		{
			while(fin >> w.f >> w.p.real() >> w.p.imag())
			{
				route.push_back(w);
			}
			fin.close();
		}
		cout << "KeiroSize: " << route.size() << endl;

		fin.open((map_path+map_name+"spurgl.dat").c_str());
		if(fin)
		{
			point p; char ch;
			while(fin >> ch >> p.real() >> p.imag())
			{
				spur.push_back(p);
			}
			fin.close();
		}

		{ // ---> read opsm map
			gnd::opsm::cmap_t			cnt_smmap;			// probabilistic scan matching counting map
			gnd::opsm::map_t			smmap;				// probabilistic scan matching map
			gnd::bmp8_t bmp8;

			::fprintf(stderr, "\n");
			::fprintf(stderr, " => load scan matching map from \"\x1b[4m%s\x1b[0m\"\n", (map_path+map_name+"opsm-map").c_str());
			if( gnd::opsm::read_counting_map(&cnt_smmap, (map_path+map_name+"opsm-map").c_str()) < 0){
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to load scan matching map \"\x1b[4m%s\x1b[0m\"\n", (map_path+map_name+"opsm-map").c_str());
			}
			else if( gnd::opsm::build_map(&smmap, &cnt_smmap, gnd_mm2dist(1)) < 0) {
				::fprintf(stderr, "  ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build scan matching map \"\x1b[4m%s\x1b[0m\"\n", (map_path+map_name+"opsm-map").c_str());
			}
			else if (gnd::opsm::build_bmp8(&bmp8, &smmap, gnd_m2dist( 1.0 / 32)) < 0) {
				::fprintf(stderr, "  ... \x1b[1mOK\x1b[0m: load scan matching map \"\x1b[4m%s\x1b[0m\"\n", (map_path+map_name+"opsm-map").c_str());
			}

			for(unsigned int y=0; y<bmp8.row(); y++)
			{
				for(unsigned int x=0; x<bmp8.column(); x++)
				{
					unsigned char v;
					int lv;
					bmp8.get(y,x,&v);
					lv = v;
					if(lv) {
						double px;
						double py;
						bmp8.pget_pos_core(y,x, &px, &py);
						point pt(px, py);
						gridmap.push_back( pt );
						gridval.push_back( lv );
					}
				}
			}

		} // <--- read opsm map


	}

	glutInit(&argc, argv);
	glutInitWindowSize(wid,hei1+hei2);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE /*| GLUT_DEPTH*/);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

	glutCreateWindow("Visualizer");
	glClearColor(0.0, 0.0, 0.0, 0.0);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

  int MENU_ID[4];
  MENU_ID[0] = glutCreateMenu(menu_way);
	for(int i=0; i<4; i++) glutAddMenuEntry(menu_str_way[i]+(i==disp_way?0:1), i+1);
  MENU_ID[1] = glutCreateMenu(menu_line);
	for(int i=0; i<3; i++) glutAddMenuEntry(menu_str_map[i]+(i==disp_line?0:1), i+1);
  MENU_ID[2] = glutCreateMenu(menu_edge);
	for(int i=0; i<3; i++) glutAddMenuEntry(menu_str_map[i]+(i==disp_edge?0:1), i+1);
  MENU_ID[3] = glutCreateMenu(menu_grid);
	for(int i=0; i<3; i++) glutAddMenuEntry(menu_str_grid[i]+(i==disp_edge?0:1), i+1);
  glutCreateMenu(menu);
  glutAddSubMenu("waypoint", MENU_ID[0]);
  //glutAddSubMenu("line",     MENU_ID[1]);
  //glutAddSubMenu("edge",     MENU_ID[2]);
  //glutAddSubMenu("gridmap",  MENU_ID[3]);
  glutAttachMenu(GLUT_MIDDLE_BUTTON);

	glutMainLoop();

	return 0;
}
