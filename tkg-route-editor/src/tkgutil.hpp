#ifndef TKG_UTIL_HPP
#define TKG_UTIL_HPP

#include <cstdarg>
#include <cmath>
#include <string>
#include <vector>

#include <iostream>
using namespace std;

namespace tkg {

std::string strprintf(const char *format, ... )
{	
    va_list list;
    char str[256];

    va_start(list, format);
//    int size = vsprintf(str, format, list);
    va_end(list);

		return std::string(str);
}

class Exception
{
	public:
		std::string str;
		Exception(std::string str) : str(str) {}
};


class triple
{
	public:
		union
		{
			struct { double x,y,z; };
			double vec[3];
		};

		triple(double t = 0)                 : x(t), y(t), z(t) {}
		triple(double x, double y, double z) : x(x), y(y), z(z) {}

		triple(double *v)
		{
			for(int i=0; i<3; i++) { vec[i]=v[i]; }
		}

		double abs() const
		{
			return sqrt(x*x + y*y + z*z);
		}

		void rotX(double rad)
		{
			double ty=y, tz=z;
			y = ty*cos(rad) - tz*sin(rad);
			z = ty*sin(rad) + tz*cos(rad);
		}
		void rotY(double rad)
		{
			double tz=z, tx=x;
			z = tz*cos(rad) - tx*sin(rad);
			x = tz*sin(rad) + tx*cos(rad);
		}
		void rotZ(double rad)
		{
			double tx=x, ty=y;
			x = tx*cos(rad) - ty*sin(rad);
			y = tx*sin(rad) + ty*cos(rad);
		}

		triple operator+(const triple &t) const { return triple(x+t.x, y+t.y, z+t.z); }
		triple operator-(const triple &t) const { return triple(x-t.x, y-t.y, z-t.z); }

		friend ostream& operator<<(ostream &out, const triple &t)
		{
			return (out << "(" << t.x << "," << t.y << "," << t.z << ")");
		}
};

triple polar(double r, double v, double h)
{
	return triple(r*sin(v)*cos(h), r*sin(v)*sin(h), r*cos(v));
}

vector<double> convert_color(const string &str)
{
	vector<double> ret(4);
	for(int i=0; i<(signed)min(2*ret.size(),str.size()); i++)
	{
		ret[i/2] *= 16;
		ret[i/2] += str[i]<'A' ? str[i]-'0' : str[i]-'A'+10;
	}
	for(int i=0; i<(signed)ret.size(); i++) { ret[i] /= 255; }
	if(str.size() <= 6) { ret[3]=1.0; }
	return ret;
}


}

#endif
