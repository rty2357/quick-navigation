#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>

namespace tkg
{

const double pi = 3.14159265358979323846;

class point3
{
	public:

        union
		{
			double vec[3];
			struct { double x,y,z; };
		};

		point3(double t = 0)                 : x(t), y(t), z(t) {}
		point3(double x, double y, double z) : x(x), y(y), z(z) {}

		point3(double *v)
		{
            for(int i=0; i<3; i++) { vec[i] = v[i]; }
		}

		static point3 polar(double r, double t)
		{
			return point3(r*cos(t), r*sin(t), 0);
		}

		static point3 polar(double r, double v, double h)
		{
			return point3(r*sin(v)*cos(h), r*sin(v)*sin(h), r*cos(v));
		}


        point3 operator+(const point3 &t) const { return point3(x+t.x, y+t.y, z+t.z); }
        point3 operator-(const point3 &t) const { return point3(x-t.x, y-t.y, z-t.z); }


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

/*
        friend ostream& operator<<(ostream &out, const point3 &t)
		{
			return (out << "(" << t.x << "," << t.y << "," << t.z << ")");
        }
*/

};

}

#endif
