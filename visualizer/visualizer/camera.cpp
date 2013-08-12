#include <GL/glu.h>
#include "camera.hpp"
using namespace tkg;

Camera::Camera()
{
    mode = 2;
    roth = 0.0;
    rotv = 0.0;
    dist = 15.0;
}

void Camera::update()
{
    point3 vec = point3::polar(dist, rotv-pi/2, roth);
    point3 cam = point3::polar(dist, rotv, roth) + pos;
    gluLookAt(cam.x, cam.y, cam.z,  pos.x, pos.y, pos.z, vec.x, vec.y, vec.z);
}

void Camera::scale(double d)
{
    dist += d;
}

void Camera::rotate(double x, double y)
{
    roth += x;
    rotv += y;
}

void Camera::translate(double x, double y)
{
    point3 tmp(y*dist/2, x*dist/2, 0);
    //tmp.rotY(rotv); // not grounding
    tmp.rotZ(roth);
    pos = pos + tmp;
    //if(grounding==1) { camCent.z=0; }
}

void Camera::setpos(double x, double y, double t)
{
    if(mode>=1) pos = point3(x,y,0);
    if(mode>=2) roth = t+3.141592;
}

void Camera::setmode(int m)
{
    mode = m;
}
