#ifndef VIEWER_SSM_HPP
#define VIEWER_SSM_HPP
#include <QObject>
#include <vector>
#include "ssm-laser.hpp"
#include "ssmtype/spur-odometry.h"

class Stream : public QObject
{
    Q_OBJECT

    public:

        int  view_state;
        int  ssm_id;
        char ssm_name[64];

        Stream(SSMApiBase *ptr);
        void init(const char* name, int id);
        bool open();
        bool close();

    public slots:

        void set_view_state(int state);

    private:

        SSMApiBase *api;
};

class LaserStream : public Stream
{
    public:

        LaserStream();
        SSMSOKUIKIData3D obj;
};


class ViewerSSM : public QObject
{
    static const int MAX_LASER_STREAM = 2;

    Q_OBJECT

    public:

         ViewerSSM();
        ~ViewerSSM();

         bool init();
         void draw();

         bool test_get_pos(double *x, double *y, double *t);

    signals:

         void send_status (const char* msg);
         void send_message(const char* msg);

    private:

        // SSM
        int ssm;

        std::vector<Stream*> ssmlist;

        LaserStream *laser[MAX_LASER_STREAM];

        SSMApi<Spur_Odometry> glpos;


};


#endif // VIEWER_SSM_HPP
