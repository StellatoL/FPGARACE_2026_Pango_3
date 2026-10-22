QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += link_pkgconfig

TARGET = img_recv_demo
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS

# ===== 源码文件 =====
SOURCES += \
    src/core/adas_engine.cpp \
    src/core/ground_calibration.cpp \
    src/inference/neural_contract.cpp \
    src/inference/neural_models.cpp \
    src/inference/adas_pipeline.cpp \
    src/ui/adas_controller.cpp \
    src/main.cpp \
    src/ui/mainwindow.cpp \
    src/inference/plate_infer_thread.cpp \
    src/core/plate_candidate_filter.cpp \
    src/core/plate_recognizer.cpp \
    src/common/plate_text_codec.cpp \
    src/core/traffic_scene_detector.cpp \
    src/core/traffic_scene_rules.cpp \
    src/common/vision_common.cpp \
    src/hardware/pango_fun.cpp

HEADERS += \
    src/core/adas_types.h \
    src/core/adas_engine.h \
    src/core/ground_calibration.h \
    src/core/road_geometry.h \
    src/common/frame_packet.h \
    src/inference/adas_pipeline.h \
    src/inference/neural_contract.h \
    src/inference/neural_models.h \
    src/inference/neural_schedule.h \
    src/ui/adas_controller.h \
    src/inference/detection_mode.h \
    src/ui/mainwindow.h \
    src/inference/plate_infer_thread.h \
    src/core/plate_candidate_filter.h \
    src/core/plate_recognizer.h \
    src/common/plate_text_codec.h \
    src/core/traffic_scene_detector.h \
    src/core/traffic_scene_rules.h \
    src/common/vision_common.h \
    src/common/vision_result.h \
    src/inference/yolov5_postprocess.hpp \
    src/hardware/pango_fun.h

# ===== RGA 2D 硬件加速（可选）=====
!no_rga:exists(/usr/include/rga/rga.h) {
    message("RGA headers found — enabling optional RGA hardware preprocessing")
    DEFINES += PANGO_HAVE_RGA
    SOURCES += src/inference/rga_preprocessor.cpp
    HEADERS += src/inference/rga_preprocessor.h
    LIBS    += -ldl
}

# ===== UI 表单 =====
FORMS += \
    src/ui/mainwindow.ui

# ===== 第三方依赖 =====
PKGCONFIG += opencv4

INCLUDEPATH += \
    $$PWD/third_party \
    $$PWD/third_party/rknn/include \
    $$PWD/src \
    $$PWD/src/ui \
    $$PWD/src/core \
    $$PWD/src/inference \
    $$PWD/src/hardware \
    $$PWD/src/common

LIBS += -L$$PWD/third_party/rknn/lib -lrknnrt

# ===== 资源 =====
RC_ICONS = resources/video.ico

# ===== 默认部署路径 =====
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
