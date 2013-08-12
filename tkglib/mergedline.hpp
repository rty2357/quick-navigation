#ifndef MERGEDLINE_H
#define MERGEDLINE_H

#include <iostream>

#include <complex>
#include <vector>
#include <algorithm>
using namespace std;
typedef complex<double> point;

// ToDo
//0deg-360deg境界


// geometry
struct line : public vector<point>
{
	line(const point& a, const point& b) { push_back(a); push_back(b); }
};

const double inf = 1e10;
const double eps = 1e-10;
point  vec  (const line&  l)                 { return l[1]-l[0];          }
double abs	(const line&  l)                 { return abs(vec(l));        }
double dot  (const point& a, const point& b) { return (a*conj(b)).real(); }
double cross(const point& a, const point& b) { return (conj(a)*b).imag(); }

int ccw(const point& a, const point& b, const point& c) {
	point u=b-a, v=c-a;
	if(cross(u,v) > 0 ) return +1; // ccw
	if(cross(u,v) < 0 ) return -1; // cw
	if(  dot(u,v) < 0 ) return +2; // cab
	if(abs(u) < abs(v)) return -2; // abc
	return 0;                      // acb
}

int ccw(const line& s, const point& p) {
	return ccw(s[0], s[1], p);
}

point projection(const line& l, const point& p) {
	double t = dot(p-l[0], vec(l)) / norm(vec(l));
	return l[0] + t*vec(l);
}

bool intersectSP(const line& s, const point& p) {
	return abs(s[0]-p)+abs(s[1]-p) < abs(s[1]-s[0])+eps;
}

bool intersectSS(const line& s, const line& t) {
	return ccw(s,t[0])*ccw(s,t[1]) <= 0 
	    && ccw(t,s[0])*ccw(t,s[1]) <= 0;
}

double distanceSP(const line& s, const point& p) {
  point r = projection(s,p);
  if(intersectSP(s,r)) return abs(r-p);
  return min(abs(s[0]-p), abs(s[1]-p));
}

double distanceSS(const line& s, const line& t) {
  if(intersectSS(s,t)) return 0;
  return min(min(distanceSP(s,t[0]), distanceSP(s,t[1])),
             min(distanceSP(t,s[0]), distanceSP(t,s[1])));
}


// union-find
struct UnionFind
{
  vector<int> data;
  UnionFind(int size) : data(size, -1) { }
  bool unionSet(int x, int y)
	{
		x=root(x); y=root(y);
		if(x != y)
		{
			if(data[y]<data[x]) { swap(x, y); }
			data[x] += data[y]; data[y] = x;
		}
		return x != y;
	}
	bool findSet(int x, int y) { return root(x) == root(y); }
	int  root(int x) { return data[x]<0 ? x : data[x]=root(data[x]); }
	int  size(int x) { return -data[root(x)]; }
};

// info
struct info : public line
{
	double dist,rad,weight; int group;
	info(int g)    : line(0,0), group(g), weight(0) {}
	info(line seg) : line(seg), group(0), weight(0) {}
	void setinfo()
	{
			const info& seg = (*this);
			dist   = cross(seg[0], vec(seg)) / abs(seg);
			rad    = arg(vec(seg)) + 1.5*M_PI;
			if(weight==0) { weight = abs(seg[1]-seg[0]); }

			if(dist<0) { dist=-dist; rad+=M_PI; }
			while(2*M_PI<rad) { rad-=2*M_PI; }
	}
	bool operator<(const info& f) const
	{
		return group > f.group;
	}
};

// MergedLine
class MergedLine
{
	private:

		static const double gain     = 5.0; // 1[rad]=gain[m] 
		static const int sentinel = (1<<24);
		// static const double gain     = 1.5; // 1[rad]=gain[m]
		static const double ts_line = 0.5; // m
		static const double ts_seg  = 1.0; // m

		typedef double (*comparator)(const info&, const info&);

		static double dist_line(const info& x, const info& y)
		{
			double rad  = (x.rad  - y.rad );
			double dist = (x.dist - y.dist);

			//rad = gain * min(rad, 2*M_PI-abs(rad));
			return sqrt(rad*rad + dist*dist);
		}

		static double dist_seg(const info& x, const info& y)
		{
			distanceSS(x, y);
		}

		void clustering(int s, int t, int group, comparator cmp, double threshold)
		{
			UnionFind uf(t-s);
			for(int i=  0; i<t-s; i++)
			for(int j=i+1; j<t-s; j++)
			{
				if(cmp(data[s+i],data[s+j]) < threshold)
				{
					uf.unionSet(i, j);
				}
			}

			for(int i=0; i<t-s; i++) { if(uf.data[i]<0) uf.data[i]=--group; }
			for(int i=0; i<t-s; i++) { data[s+i].group = uf.size(i)-1; }
		}

	public:

		vector<info> data;

		void push(const line& seg)
		{
			data.push_back(seg);
		}

		void shift(const point& pos)
		{
			for(int i=0; i<data.size(); i++) { data[i][0]+=pos; data[i][1]+=pos; }
		}

		void prepare()
		{
			for(int i=0; i<data.size(); i++)
			{
				data[i].setinfo();
			}
		}

		void clustering_line()
		{
			clustering(0, data.size(), 0, &dist_line, ts_line);
			sort(data.begin(), data.end());
		}

		void clustering_segment()
		{
			int s=0,t=0,group=0;

			data.push_back( info(sentinel) );	
			while(data[t].group != sentinel)
			{
				for(s=t; data[s].group==data[t].group; t++);

				clustering(s, t, group, &dist_seg, ts_seg);
				group -= 1000;
			}
			data.pop_back();
			sort(data.begin(), data.end());
		}

		void merge()
		{
			int s=0,t=0;
			vector<info> newdata;

			data.push_back( info(sentinel) );		
			while(data[t].group != sentinel)
			{
				for(s=t; data[s].group==data[t].group; t++);

				double dist=0,rad=0,weight=0;
				for(int i=s; i<t; i++)
				{
					dist   += data[i].weight * data[i].dist;
					rad    += data[i].weight * data[i].rad;
					weight += data[i].weight;
				}
				rad  /= weight;
				dist /= weight;

				double minY=inf, maxY=-inf;
				for(int i=s; i<t; i++)
				for(int j=0; j<2; j++)
				{
					point p = data[i][j];
					p -= polar(dist,  rad);
					p *= polar(1.0,  -rad);
					minY = min(minY, p.imag() + p.real());
					maxY = max(maxY, p.imag() - p.real());
				}

				line newline(point(0,minY), point(0,maxY));

				for(int i=0; i<2; i++)
				{
					newline[i] *= polar(1.0,  rad);
					newline[i] += polar(dist, rad);
				}

				newdata.push_back( info(newline) );
				newdata.back().dist   = dist;
				newdata.back().rad    = rad;
				newdata.back().weight = weight;
			}
		
			data = newdata;
		}
};

vector<line> mergeSegment(const vector<line> &seg, point pos=0)
{
	MergedLine mergedline;
	for(int i=0; i<seg.size(); i++) { mergedline.push( seg[i] ); }
	
	mergedline.shift(-pos);
	mergedline.prepare();
	mergedline.clustering_line();
	mergedline.clustering_segment();
	mergedline.merge();
	mergedline.shift(+pos);

	vector<line> result;
	for(int i=0; i<mergedline.data.size(); i++) { result.push_back( mergedline.data[i] ); }
	return result;
}

#endif

