#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <QObject>
#include "tkg-geometry.hpp"

class Camera : public QObject
{
    Q_OBJECT

    public:

        Camera();
        void update();
        void scale    (double d);
        void rotate   (double x, double y);
        void translate(double x, double y);

        void setpos(double x, double y, double t);

    public slots:

        void setmode(int m);

    private:

        int mode;
        double roth;
        double rotv;
        double dist;
        tkg::point3 pos;
};

#endif
