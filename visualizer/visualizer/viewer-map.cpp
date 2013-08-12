#include <GL/glu.h>
#include "viewer-map.hpp"
#include "gnd-bmp.hpp"
#include "gnd-opsm.hpp"
using namespace gnd;
using namespace gnd::opsm;

#include <algorithm>
using namespace std;

ViewerMap::ViewerMap()
{
    width  = -1;
    height = -1;
    data   = NULL;
}

ViewerMap::~ViewerMap()
{
    //glDeleteTextures(1 , &textures);
    delete data;
}

bool ViewerMap::read_cmap(const char *dirname)
{
    if( dirname == NULL) return false;
    if(*dirname == '\0') return false;

    map_t  opsm_map;
    cmap_t cnt_map;
    bmp8_t bmp_map;

    // read  map raw data
    if( read_counting_map(&cnt_map, dirname) < 0)
    {
        return false;
    }

    // convert?
    if( gnd::opsm::build_map(&opsm_map, &cnt_map, gnd_mm2dist(1)) < 0 ) {
        //::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to build map\n");
        return false;
    }

    if( build_bmp(&bmp_map, &opsm_map, gnd_m2dist(1.0/10)) < 0 )
    {
        //::fprintf(stderr, " ... \x1b[1m\x1b[31mERROR\x1b[39m\x1b[0m: fail to convert bmp\n");
        return false;
    }

    width  = bmp_map.column();
    height = bmp_map.row();
    base_x = bmp_map.xorg();
    base_y = bmp_map.yorg();
    unit_x = bmp_map.xrsl();
    unit_y = bmp_map.yrsl();

    data = new unsigned char[width*height*3];

    for(int y=0; y<height; y++)
    for(int x=0; x<width;  x++)
    {
        data[(y*width+x)*3+0] = min(255, bmp_map.value(y, x)*2);
        data[(y*width+x)*3+1] = min(255, bmp_map.value(y, x)*2);
        data[(y*width+x)*3+2] = min(255, bmp_map.value(y, x)*2);
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);


    return true;
}

#include <GL/gl.h>
void ViewerMap::draw()
{
    glColor3d(1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_POLYGON);
    glTexCoord2d(1, 1); glVertex3d(base_x + width*unit_x, base_y + height*unit_y, 0.0);
    glTexCoord2d(0, 1); glVertex3d(base_x,                base_y + height*unit_y, 0.0);
    glTexCoord2d(0, 0); glVertex3d(base_x,                base_y,                 0.0);
    glTexCoord2d(1, 0); glVertex3d(base_x + width*unit_x, base_y,                 0.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}
