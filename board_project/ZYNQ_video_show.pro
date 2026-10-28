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
    src/core/road_geometry.cpp \
    src/inference/neural_contract.cpp \
    src/inference/neural_models.cpp \
    src/inference/adas_pipeline.cpp \
    src/ui/adas_controller.cpp \
    src/main.cpp \
    src/ui/mainwindow.cpp \
    src/hardware/pango_fun.cpp

HEADERS += \
    src/core/adas_types.h \
    src/core/manual_controls.h \
    src/core/local_planner.h \
    src/core/tracking_evidence.h \
    src/core/tracked_overlays.h \
    src/core/view_tracker.h \
    src/core/adas_engine.h \
    src/core/ground_calibration.h \
    src/core/road_geometry.h \
    src/common/frame_packet.h \
    src/inference/adas_pipeline.h \
    src/inference/neural_contract.h \
    src/inference/neural_models.h \
    src/inference/neural_scene_adapter.h \
    src/inference/neural_schedule.h \
    src/ui/adas_controller.h \
    src/inference/detection_mode.h \
    src/ui/mainwindow.h \
    src/common/vision_result.h \
    src/hardware/pcie_frame_status.h \
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
    $$PWD/third_party/rknn/include \
    $$PWD/src

LIBS += -L$$PWD/third_party/rknn/lib -lrknnrt

# ===== 资源 =====
RC_ICONS = resources/video.ico

# ===== 默认部署路径 =====
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
