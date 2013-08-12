#ifndef GL_GEOMETRY_HPP
#define GL_GEOMETRY_HPP

#include <GL/glu.h>
#include "tkg-geometry.hpp"

namespace tkg
{

inline void ggVertex(const point3 &p)
{
    glVertex3dv( p.vec );
}


inline void ggDrawCross(const point3 &p, double theta, double r)
{
	glBegin(GL_LINES);
    glVertex3dv( (p + point3::polar(r, theta + 0.0*pi)).vec );
    glVertex3dv( (p + point3::polar(r, theta + 1.0*pi)).vec );
    glVertex3dv( (p + point3::polar(r, theta + 0.5*pi)).vec );
    glVertex3dv( (p + point3::polar(r, theta + 1.5*pi)).vec );
	glEnd();
}

}

#endif
