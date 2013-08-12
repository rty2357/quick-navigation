#ifndef WIDGET_GL_HPP
#define WIDGET_GL_HPP

#include <QtOpenGL/QGLWidget>
#include <QTimer>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include "camera.hpp"

class ViewerSSM;
class ViewerMap;

class WidgetGL : public QGLWidget
{
    Q_OBJECT

    public:

         WidgetGL();
        ~WidgetGL();

         bool init();
         ViewerMap* get_vmap() { return vmap; }
         ViewerSSM* get_vssm() { return vssm; }
         Camera*  get_camera() { return camera; }

    public slots:

         void setfps(int fps);

    protected:

        void initializeGL();
        void paintGL();
        void resizeGL(int, int);

        void keyPressEvent(QKeyEvent *event);
        void keyReleaseEvent(QKeyEvent *event);
        void wheelEvent(QWheelEvent *event);
        void mouseMoveEvent(QMouseEvent *event);
        void mousePressEvent(QMouseEvent *event);
        void mouseReleaseEvent(QMouseEvent *event);

    private:

        // FPS制御
        QTimer *timer;

        // データの管理クラス
        ViewerMap *vmap;
        ViewerSSM *vssm;

        // 画面サイズ関連
        int    width;
        int    height;
        double aspect;

        // カメラ
        Camera *camera;

        // マウス制御クラスに移動予定
        int mouse_prev_x;
        int mouse_prev_y;
        int mouse_prev_b;

};

#endif // WIDGET_GL_HPP
