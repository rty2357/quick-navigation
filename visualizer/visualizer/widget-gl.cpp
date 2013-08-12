#include <GL/glu.h>
#include "widget-gl.hpp"
#include "viewer-ssm.hpp"
#include "viewer-map.hpp"

#include <fstream>
using namespace std;

WidgetGL::WidgetGL() : QGLWidget()
{
    printf("GL constructor\n");
    setFocusPolicy(Qt::StrongFocus);

    camera = new Camera;

    vmap = new ViewerMap;
    vssm = new ViewerSSM;
    //mapviewer.read_cmap("/home/ena8781/roboken/map/tc2013");

    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(100);
}

WidgetGL::~WidgetGL()
{
    delete camera;

    delete vmap;
    delete vssm;
    printf("GL destructor\n");
}

bool WidgetGL::init()
{
    vssm->init();

    char path[256];
    ifstream fin("visualizer.conf");
    if(fin) fin >> path;
    vmap->read_cmap(path);
}

void WidgetGL::setfps(int fps)
{
    timer->start(1000/fps);
}

void WidgetGL::initializeGL()
{
    // 背景色の指定
    glClearColor(0, 0, 0, 0);
}

void WidgetGL::resizeGL(int w, int h)
{
    width  = w;
    height = h;
    aspect = (double)w/h;
}

void WidgetGL::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);   //  カラーバッファをクリア
    /*
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    */

    glViewport(0, 0, width, height);


    double cx,cy,th;
    if(vssm->test_get_pos(&cx, &cy, &th))
    {
        camera->setpos(cx,cy,th);
    }

    // カメラの設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, aspect, 0.1, 500);
    camera->update();

    // モデルの描画
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 各種データの描画
    vmap->draw();
    vssm->draw();

    glFlush();
}

void WidgetGL::keyPressEvent(QKeyEvent *event)
{
    double dx = 0;
    double dy = 0;
    printf("%d\n", event->key());
    if(event->key() == Qt::Key_Up   ) dy -= 0.1;
    if(event->key() == Qt::Key_Down ) dy += 0.1;
    if(event->key() == Qt::Key_Left ) dx -= 0.1;
    if(event->key() == Qt::Key_Right) dx += 0.1;



    if(event->modifiers() & Qt::ControlModifier)
    {
        camera->scale(dy*10);
    }
    else if(event->modifiers() & Qt::ShiftModifier)
    {
        camera->rotate(dx,dy);
    }
    else // (NoModifier)
    {
        camera->translate(dx,dy);
    }
}

void WidgetGL::keyReleaseEvent(QKeyEvent *event)
{

}

void WidgetGL::wheelEvent(QWheelEvent *event)
{
    double dd = (double) event->delta() / 120;
    camera->scale(-dd);
}

void WidgetGL::mouseMoveEvent(QMouseEvent *event)
{
    double dx = (double) (mouse_prev_x - event->x()) / width;
    double dy = (double) (mouse_prev_y - event->y()) / height;

    if(mouse_prev_b == Qt::LeftButton)
    {
        camera->translate(dx,dy);
    }

    if(mouse_prev_b == Qt::RightButton)
    {
        camera->rotate(dx,dy);
    }


    mouse_prev_x = event->x();
    mouse_prev_y = event->y();
}

void WidgetGL::mousePressEvent(QMouseEvent *event)
{
    mouse_prev_x = event->x();
    mouse_prev_y = event->y();
    mouse_prev_b = event->button();
}

void WidgetGL::mouseReleaseEvent(QMouseEvent *event)
{
    mouse_prev_x = event->x();
    mouse_prev_y = event->y();
    mouse_prev_b = event->button();
}

