#include <cstdio>
#include <cstring>
#include "viewer-ssm.hpp"
#include "tkg-geometry-gl.hpp"
using namespace std;
using namespace tkg;

#include "main.hpp"
#include "window.hpp"

Stream::Stream(SSMApiBase *ptr)
{
    api = ptr;
    view_state = 3;
}

void Stream::init(const char* name, int id)
{
    ssm_id = id;
    strncpy(ssm_name, name, 64);
}

void Stream::set_view_state(int state)
{
    view_state = state;
    printf("state=%d <= %d\n",view_state,state);
}

bool Stream::open()
{
    return api->open(ssm_name, ssm_id);
}

bool Stream::close()
{
    return api->close();
}

LaserStream::LaserStream() : Stream(&obj)
{
}


ViewerSSM::ViewerSSM()
{
    ssm = 0;

    laser[0] = new LaserStream();
    laser[1] = new LaserStream();
    laser[0]->init("sokuiki_fs", 3);
    laser[1]->init("sokuiki_fs", 4);

    ssmlist.push_back(laser[0]);
    ssmlist.push_back(laser[1]);
}

bool ViewerSSM::init()
{
    if(ssm = initSSM())
    {
        emit send_message("SSMへの接続に成功しました。\n");
        glpos.open("spur_odometry", 0);

        for(int i=0; i<ssmlist.size(); i++)
        {
            char msg[256];
            if(ssmlist[i]->open())
            {
                snprintf(msg, 256, "%s [%d]", ssmlist[i]->ssm_name, ssmlist[i]->ssm_id);
                window->test_add_menu_SSM(ssmlist[i], msg);
                snprintf(msg, 256, "%s [%d] へ接続成功。\n", ssmlist[i]->ssm_name, ssmlist[i]->ssm_id);
            }
            else
            {
                snprintf(msg, 256, "%s [%d] へ接続失敗。\n", ssmlist[i]->ssm_name, ssmlist[i]->ssm_id);
            }
            emit send_message(msg);
        }

        return true;
    }
    emit send_message("SSMへの接続に失敗しました。\n");
    return false;
}

ViewerSSM::~ViewerSSM()
{
    if(ssm)
    {
        glpos.close();
        for(int i=0; i<ssmlist.size(); i++)
        {
            ssmlist[i]->close();
        }

        delete laser[0];
        delete laser[1];
        endSSM();
    }
}

void ViewerSSM::draw()
{
    if(!ssm) return;

    // exec
    glpos.readNew();

    point3 robot_pos(glpos.data.x, glpos.data.y, 0);
    // ----exec

    glColor3d(1,0,0);
    glLineWidth(3);
    ggDrawCross(robot_pos, glpos.data.theta, 1);
    glLineWidth(1);

    const int gsize = 30;
    double gr_x = glpos.data.x;
    double gr_y = glpos.data.y;
    /*
    glVertex3d(gr_x+gsize, gr_y+gsize, 0.0);
    glVertex3d(gr_x-gsize, gr_y+gsize, 0.0);
    glVertex3d(gr_x-gsize, gr_y-gsize, 0.0);
    glVertex3d(gr_x+gsize, gr_y-gsize, 0.0);
    */
    glColor3d(0.2,0.2,0.2);
    glBegin(GL_LINES);
    for(int i=-gsize; i<=gsize; i++)
    {
        glVertex3d(gr_x+i*1.0, gr_y-gsize, 0.0);
        glVertex3d(gr_x+i*1.0, gr_y+gsize, 0.0);
        glVertex3d(gr_x-gsize, gr_y+i*1.0, 0.0);
        glVertex3d(gr_x+gsize, gr_y+i*1.0, 0.0);
    }
    glEnd();


    char status[256];
    snprintf(status, 256, "x = %lf\ny = %lf\nt = %lf\n", glpos.data.x, glpos.data.y, glpos.data.theta);
    emit send_status(status);


    for(int s=0; s<2; s++)
    {
        laser[s]->obj.readNew();

        glpos.readTime(laser[s]->obj.time);
        double robot_theta = glpos.data.theta;
        robot_pos = point3(glpos.data.x, glpos.data.y, 0);

        if(laser[s]->view_state & 1)
        {
            if(s) glColor3d(0,1,0); else glColor3d(0,1,1);
            glPointSize(3);
            glBegin(GL_POINTS);
            for(int i=0; i<laser[s]->obj.data.numPoints(); i++)
            {
                if(laser[s]->obj.data[i].isWarning()) continue;

                point3 ref(laser[s]->obj.data[i].reflect.vec);
                ref.rotZ(robot_theta);
                ggVertex( robot_pos + ref );
            }
            glEnd();
            glPointSize(1);
        }

        if(laser[s]->view_state & 2)
        {
            if(s) glColor3d(0,0.5,0); else glColor3d(0,0.5,0.5);
            glLineWidth(1);
            glBegin(GL_LINES);
            for(int i=0; i<laser[s]->obj.data.numPoints(); i++)
            {
                if(laser[s]->obj.data[i].isWarning()) continue;

                point3 ori(laser[s]->obj.data[i].origin.vec);
                point3 ref(laser[s]->obj.data[i].reflect.vec);
                ori.rotZ(robot_theta);
                ref.rotZ(robot_theta);
                ggVertex( robot_pos + ori );
                ggVertex( robot_pos + ref );
            }
            glEnd();
            glPointSize(1);
        }
    }
}


bool ViewerSSM::test_get_pos(double *x, double *y, double *t)
{
    if(!glpos.isOpen()) return false;
    *x = glpos.data.x;
    *y = glpos.data.y;
    *t = glpos.data.theta;
    return true;
}
