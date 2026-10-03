#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QUdpSocket>
#include <QDebug>
#include <QPainter>
#include <QImage>
#include <QDateTime>
#include <QThread>
#include <QLabel>
#include <QElapsedTimer>

#include "inference/detection_mode.h"
#include "hardware/pango_fun.h"
#include "hardware/file_video_source.h"
#include "inference/plate_infer_thread.h"
#include "core/adas_engine.h"

// PCIe DMA 传输单帧图像大小：1280×720×2 字节（RGB565 格式）
#define IMG_SIZE    1280*720*2

namespace Ui {
class MainWindow; }

// 主窗口：PCIe 视频传输上位机的核心界面
//
// 主要功能：
// - PCIe 设备管理（打开/关闭、DMA 读写、BAR0 映射）
// - 视频采集与显示（1280×720 RGB565 实时预览）
// - 视频源：PCIe 采集 / 本地文件回放（U 盘场景视频，互斥切换）
// - 车牌识别模式（YOLOv5 + LPRNet 两阶段推理）
// - 交通场景检测模式（行人/斑马线/红绿灯检测与告警）
// - ADAS 辅助驾驶模式（赛题四场景：红绿灯决策/虚实线变道/导航跟随/行人刹车）
// - 基础图像处理演示（灰度/去噪/二值化/边缘/HSV，评分项）
//
// 数据流：
// FPGA/文件 → PCIe DMA/img_buf[4] →
// MainWindow::recv_img_dis() → PlateInferThread（异步推理） →
// result_ready 信号 → AdasEngine 决策 → draw_overlay() → QLabel 显示
class MainWindow : public QMainWindow
{
    Q_OBJECT

signals:
    // 新帧到达信号（预留：可用于通知其他组件）
    void newframe_ready(int index);

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // 启动即进入 ADAS 模式并回放指定文件（--adas 命令行入口）
    void startup_adas_with_file(const QString &path);

    int dma_len=0; // DMA 传输长度（DW 单位）

public slots:
    // 每秒触发的帧率计算槽函数（由 fps_timer_ 驱动）
    void rate_cacul();

private slots:
    // 打开/关闭 PCIe 设备
    void on_Key1_clicked();

    // 切换至交通场景检测模式
    void on_Gray_clicked();

    // 开始/停止视频采集
    void on_connect_clicked();

    // 保存当前帧为图片
    void on_save_clicked();

    // 录制视频（预留）
    void on_save_video_clicked();

    // 视频源切换（I2C 控制）
    void on_radioCamera_toggled(bool checked);
    void on_radioHDMI_toggled(bool checked);

    // FPGA 预处理开关（I2C 控制）
    void on_chkPreprocess_toggled(bool checked);
    void on_chkPlateOverlay_toggled(bool checked);
    void on_chkWhiteBalance_toggled(bool checked);
    void on_chkDehaze_toggled(bool checked);

    // 摄像头显示按钮（预留）
    void on_cam1_clicked();

    // HDMI 显示按钮（预留）
    void on_cam2_clicked();

    // 测试 DMA 写操作
    void on_dma_write_clicked();

    // 测试 DMA 读操作
    void on_dma_read_clicked();

    // FPGA DMA 传输完成槽函数，index 为 4 缓冲区的索引
    void recv_img_dis(int index);

    // 处理推理线程返回的检测结果（异步回调）
    void handle_infer_result_ready(quint64 sequence, VisionFrameResult result);

    // 切换至车牌识别模式
    void on_pushButton_clicked();

    // ===== ADAS 模式 =====
    // 切换至 ADAS 辅助驾驶模式
    void on_btnAdas_clicked();

    // 车道选择（左/直行/右）
    void on_btnLaneL_clicked();
    void on_btnLaneC_clicked();
    void on_btnLaneR_clicked();

    // 行驶/停车切换
    void on_btnDrive_toggled(bool checked);

    // 转向灯（左/右）
    void on_btnSigL_toggled(bool checked);
    void on_btnSigR_toggled(bool checked);

    // 车速滑块 / 限速设置
    void on_sliderSpeed_valueChanged(int value);
    void on_spinLimit_valueChanged(int value);

    // 强制变道指令（演示报警）
    void on_btnForceLaneL_toggled(bool checked);
    void on_btnForceLaneR_toggled(bool checked);

    // 手动导航备份（N 键循环）
    void on_btnNavNext_clicked();

    // 导航箭头 ROI 调整
    void handle_nav_roi_changed();

    // ===== 文件回放源 =====
    // PCIe/文件源切换
    void on_radioPCIe_toggled(bool checked);
    void on_radioFile_toggled(bool checked);
    // 打开文件选择框
    void on_btnOpenFile_clicked();
    // 播放/暂停
    void on_btnPlayPause_clicked();
    // 循环开关
    void on_chkLoop_toggled(bool checked);
    // 文件播放自然结束
    void handle_file_playback_finished();
    // 文件源错误上报
    void handle_file_source_error(const QString &message);

    // ===== 图像处理演示 =====
    void on_cmbProc_currentIndexChanged(int index);

    // ===== 快捷键 =====
    void handle_hotkey_lane_left();
    void handle_hotkey_lane_center();
    void handle_hotkey_lane_right();
    void handle_hotkey_drive_toggle();
    void handle_hotkey_signal_left();
    void handle_hotkey_signal_right();
    void handle_hotkey_force_lane_left();
    void handle_hotkey_force_lane_right();
    void handle_hotkey_nav_next();
    void handle_hotkey_pause();
    void handle_hotkey_reset();

private:
    // 在 QPixmap 上绘制检测框叠加层和告警标记
    void draw_detection_overlay(QPixmap &pixmap);

    // ADAS 叠加渲染：HUD/箭头/报警横幅/LED 指示
    void draw_adas_overlay(QPixmap &pixmap);

    // 绘制行驶行为箭头（左/直行/右）
    void draw_arrow(QPainter &painter, adas::ArrowDir dir,
                    const QPoint &center, int size, const QColor &color);

    // 基础图像处理演示：对当前帧应用所选算法（就地修改 img_buf 槽位副本）
    void apply_processing_demo(int index);

    // 向推理线程提交当前帧（指针传递，零拷贝）
    void submit_infer_frame(int index);

    // 应用检测模式切换（更新 UI 按钮状态和推理线程模式）
    void apply_detection_mode(DetectionMode mode);

    // ADAS：更新本车状态并重步决策引擎（ego 变更入口）
    void update_ego_state();

    // ADAS：刷新右侧状态面板标签
    void refresh_adas_status_labels();

    // 启动文件回放（自动停 PCIe 采集）；启动 PCIe 采集（自动停文件回放）
    void start_file_playback();
    void stop_file_playback();
    void start_pcie_capture();
    void stop_pcie_capture();

    // 将叠加完成的原始 pixmap（始终 1280×720 原生坐标）按当前 Label 尺寸
    // 保持宽高比缩放后显示。叠加层在原生坐标绘制，仅最终显示缩放，检测框坐标不变。
    void show_scaled_pixmap(const QPixmap &native_pixmap);

    pango_fun *pango_device;                  // FPGA PCIe 通信接口（唯一与 FPGA 交互的对象）
    FileVideoSource *file_source_ = nullptr;  // 文件回放源线程
    PlateInferThread *infer_thread = nullptr; // 推理工作线程
    QTimer *fps_timer_ = nullptr;             // 帧率统计定时器（每秒触发一次）
    QTimer *adas_blink_timer_ = nullptr;      // ADAS 报警闪烁定时器（2Hz，暂停时保持叠加刷新）

    int recvImageNum=0;        // 累计接收帧总数
    int recvImageNum_pre=0;    // 上一秒累计帧数（用于 FPS 计算）
    int buffer_index=0;        // 当前处理缓冲区索引
    QPixmap pixmap;            // 当前显示图像
    void paintEvent(QPaintEvent *event);
    Ui::MainWindow *ui;
    QByteArray Picture_Data;   // 存储一帧图像的数据
    int Len;
    QImage My_Image;           // 当前帧 QImage（RGB16 格式）
    QImage proc_image_;        // 图像处理演示输出帧（独立缓冲，不污染 img_buf）
    QByteArray Rec_Data;
    int width;                 // 图像宽度（默认 1280）
    int length;                // 图像高度（默认 720）
    int   display_h_disp;      // 显示的水平分辨率
    int   display_v_disp;      // 显示的垂直分辨率
    uchar System_Mode;         // 0: 关闭显示  1: 打开显示
    uchar System_Gray=1;       // 灰度模式标志（预留）
    uchar System_GrayScale=0;  // 灰度等级（预留）
    QString ip;                // UDP 目标 IP（预留）
    QString port_buf;          // UDP 端口缓冲（预留）
    int port;                  // UDP 端口号（预留）
    int frameCount=0;          // 当前帧率（每秒帧数）
    quint64 current_frame_sequence = 0;       // 当前帧序号（自增）
    quint64 latest_infer_sequence = 0;        // 最近一次推理完成的帧序号
    VisionFrameResult latest_infer_result;     // 最近一次推理结果
    DetectionMode detection_mode_ = DetectionMode::PlateRecognition; // 当前检测模式

    // ===== ADAS 状态 =====
    adas::AdasEngine adas_engine_;            // 决策引擎（滞回状态）
    adas::EgoState ego_;                       // 本车模拟状态
    adas::AdasDecision last_decision_;         // 最近一次决策结果
    adas::AdasPerception last_perception_;     // 最近一次感知快照（ego 变更时重步引擎用）
    QElapsedTimer adas_blink_clock_;           // 报警闪烁相位时钟（2Hz）
    int manual_nav_cycle_ = 3;                 // 手动导航循环位置：0左 1直 2右 3=自动
    bool file_source_active_ = false;          // 当前是否文件回放（互斥 PCIe）
    int processing_demo_index_ = 0;            // 图像处理演示模式索引（0=关闭）

    unsigned int offset_addr=0;          // DMA 偏移地址
    unsigned int send_offset_addr=0;     // 发送 DMA 偏移地址
    unsigned int send_dma_times_cnt=0;   // 发送 DMA 次数计数
    unsigned int rd_dma_times_cnt=0;     // 读取 DMA 次数计数
};

#endif // MAINWINDOW_H
