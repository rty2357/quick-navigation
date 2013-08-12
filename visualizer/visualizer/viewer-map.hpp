#ifndef VIEWER_MAP_HPP
#define VIEWER_MAP_HPP

#include <GL/gl.h>

class ViewerMap
{
    public:

         ViewerMap();
        ~ViewerMap();

        bool read_cmap(const char *dirname);
        void draw();

    private:

        GLuint texture;
        unsigned char  *data;
        int    width, height;
        double base_x, base_y;
        double unit_x, unit_y;
};


#endif // VIEWER_MAP_HPP
