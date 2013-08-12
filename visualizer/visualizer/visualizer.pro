QT += \
    widgets \
    opengl

INCLUDEPATH += \
    ../../tkglib \
    ../../gndlib \
    ../../ssmtype

HEADERS += \
    camera.hpp \
    window.hpp \
    config.hpp \
    controller.hpp \
    main.hpp \
    widget-gl.hpp \
    widget-msg.hpp \
    viewer-ssm.hpp \
    viewer-map.hpp

SOURCES += \
    main.cpp \
    window.cpp \
    camera.cpp \
    widget-gl.cpp \
    viewer-map.cpp \
    viewer-ssm.cpp \
    widget-msg.cpp

LIBS += \
    -lGLU \
    -lssm
