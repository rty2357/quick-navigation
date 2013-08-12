#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QLayout>
#include <QSignalMapper>
#include "window.hpp"
#include "viewer-ssm.hpp"

MainWindow::MainWindow() : QMainWindow()
{
    printf("constructor\n");
    resize(800, 600);
    setWindowTitle("visualizer");

    message = new WidgetMSG();
    status  = new WidgetMSG();
    viewer  = new WidgetGL();

    message->setFixedWidth(200);
    status ->setFixedWidth(200);
    status ->setFixedHeight(100);

    QWidget     *w_layout = new QWidget();
    QHBoxLayout *h_layout = new QHBoxLayout();
    QVBoxLayout *v_layout = new QVBoxLayout();

    v_layout->addWidget(status);
    v_layout->addWidget(message);
    h_layout->addWidget(viewer);
    h_layout->addLayout(v_layout);
    w_layout->setLayout(h_layout);
    setCentralWidget(w_layout);

    QAction *fps01 = new QAction(tr("1 fps"), this);
    fps01->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_1) );
    fps01->setCheckable(true);

    QAction *fps05 = new QAction(tr("5 fps"), this);
    fps05->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_2) );
    fps05->setCheckable(true);

    QAction *fps10 = new QAction(tr("10 fps"), this);
    fps10->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_3) );
    fps10->setCheckable(true);
    fps10->setChecked(true);

    QAction *fps20 = new QAction(tr("20 fps"), this);
    fps20->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_4) );
    fps20->setCheckable(true);

    QAction *fps30 = new QAction(tr("30 fps"), this);
    fps30->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_5) );
    fps30->setCheckable(true);


    QMenu *fps_menu;
    fps_menu = menuBar()->addMenu(tr("&FPS"));
    fps_menu->addAction(fps01);
    fps_menu->addAction(fps05);
    fps_menu->addAction(fps10);
    fps_menu->addAction(fps20);
    fps_menu->addAction(fps30);

    QActionGroup *fps_group = new QActionGroup(this);
    fps_group->setExclusive(true);
    fps_group->addAction(fps01);
    fps_group->addAction(fps05);
    fps_group->addAction(fps10);
    fps_group->addAction(fps20);
    fps_group->addAction(fps30);

    QSignalMapper *signal_mapper = new QSignalMapper(this);
    connect(fps01, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(fps05, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(fps10, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(fps20, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(fps30, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    signal_mapper->setMapping(fps01,  1);
    signal_mapper->setMapping(fps05,  5);
    signal_mapper->setMapping(fps10, 10);
    signal_mapper->setMapping(fps20, 20);
    signal_mapper->setMapping(fps30, 30);
    connect(signal_mapper, SIGNAL(mapped(int)), viewer, SLOT(setfps(int)));




    ssm_menu = menuBar()->addMenu(tr("&SSM"));




    QAction *camera_free = new QAction(tr("free"), this);
    camera_free->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_8) );
    camera_free->setCheckable(true);

    QAction *camera_lock_xy = new QAction(tr("lock (x,y)"), this);
    camera_lock_xy->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_9) );
    camera_lock_xy->setCheckable(true);

    QAction *camera_lock_xyt = new QAction(tr("lock (x,y,theta)"), this);
    camera_lock_xyt->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_0) );
    camera_lock_xyt->setCheckable(true);
    camera_lock_xyt->setChecked(true);

    QMenu *camera_menu;
    camera_menu = menuBar()->addMenu(tr("&Camera"));
    camera_menu->addAction(camera_free);
    camera_menu->addAction(camera_lock_xy);
    camera_menu->addAction(camera_lock_xyt);

    QActionGroup *camera_group = new QActionGroup(this);
    camera_group->setExclusive(true);
    camera_group->addAction(camera_free);
    camera_group->addAction(camera_lock_xy);
    camera_group->addAction(camera_lock_xyt);

    QSignalMapper *signal_mapper2 = new QSignalMapper(this);
    connect(camera_free,     SIGNAL(triggered()), signal_mapper2, SLOT(map()));
    connect(camera_lock_xy,  SIGNAL(triggered()), signal_mapper2, SLOT(map()));
    connect(camera_lock_xyt, SIGNAL(triggered()), signal_mapper2, SLOT(map()));
    signal_mapper2->setMapping(camera_free,     0);
    signal_mapper2->setMapping(camera_lock_xy,  1);
    signal_mapper2->setMapping(camera_lock_xyt, 2);
    connect(signal_mapper2, SIGNAL(mapped(int)), viewer->get_camera(), SLOT(setmode(int)));
}

MainWindow::~MainWindow()
{
    delete viewer;
    delete status;
    delete message;
    printf("destructor\n");
}

bool MainWindow::init()
{
    connect(viewer->get_vssm(), SIGNAL(send_status (const char*)), status,  SLOT(set_message(const char*)));
    connect(viewer->get_vssm(), SIGNAL(send_message(const char*)), message, SLOT(add_message(const char*)));

    viewer->init();
}


void MainWindow::test_add_menu_SSM(QObject* obj, const char* str)
{
    QAction *view00 = new QAction(tr("non-display"), this);
    //view00->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_1) );
    view00->setCheckable(true);

    QAction *view01 = new QAction(tr("point only"), this);
    //view01->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_2) );
    view01->setCheckable(true);

    QAction *view10 = new QAction(tr("laser only"), this);
    //view10->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_3) );
    view10->setCheckable(true);


    QAction *view11 = new QAction(tr("point + laser"), this);
    //view11->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_4) );
    view11->setCheckable(true);
    view11->setChecked(true);

    QMenu *stream_menu = new QMenu(tr(str), this);
    stream_menu->addAction(view00);
    stream_menu->addAction(view01);
    stream_menu->addAction(view10);
    stream_menu->addAction(view11);

    QActionGroup *stream_group = new QActionGroup(this);
    stream_group->setExclusive(true);
    stream_group->addAction(view00);
    stream_group->addAction(view01);
    stream_group->addAction(view10);
    stream_group->addAction(view11);

    ssm_menu->addMenu(stream_menu);

    QSignalMapper *signal_mapper = new QSignalMapper(this);
    connect(view00, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(view01, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(view10, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    connect(view11, SIGNAL(triggered()), signal_mapper, SLOT(map()));
    signal_mapper->setMapping(view00, 0);
    signal_mapper->setMapping(view01, 1);
    signal_mapper->setMapping(view10, 2);
    signal_mapper->setMapping(view11, 3);
    connect(signal_mapper, SIGNAL(mapped(int)), obj, SLOT(set_view_state(int)));
}
