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

#include "inference/detection_mode.h"
#include "hardware/pango_fun.h"
#include "common/vision_result.h"
class AdasController;
class QCloseEvent;

// PCIe DMA 传输单帧图像大小：1280×720×2 字节（RGB565 格式）
#define IMG_SIZE    1280*720*2

namespace Ui {
class MainWindow; }

// 保留原设备与预处理控制；AdasController 负责自有帧和ADAS应用流程。
class MainWindow : public QMainWindow
{
    Q_OBJECT

signals:
    // 新帧到达信号（预留：可用于通知其他组件）
    void newframe_ready(int index);

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    bool startup_adas_with_file(const QString &path);
    void set_single_view(bool single);
    void write_adas_report(const QString &path) const;
    void save_adas_screenshot(const QString &path) const;
    void set_detection_mode(DetectionMode mode);
    void closeEvent(QCloseEvent *event) override;

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

    // 切换至车牌识别模式
    void on_pushButton_clicked();

private:
    // 应用检测模式切换（更新 UI 按钮状态和推理线程模式）
    void apply_detection_mode(DetectionMode mode);

    pango_fun *pango_device;                  // FPGA PCIe 通信接口（唯一与 FPGA 交互的对象）
    AdasController *adas_ = nullptr;
    QTimer *fps_timer_ = nullptr;             // 帧率统计定时器（每秒触发一次）

    int recvImageNum=0;        // 累计接收帧总数
    int recvImageNum_pre=0;    // 上一秒累计帧数（用于 FPS 计算）
    int buffer_index=0;        // 当前处理缓冲区索引
    QPixmap pixmap;            // 当前显示图像
    void paintEvent(QPaintEvent *event);
    Ui::MainWindow *ui;
    QByteArray Picture_Data;   // 存储一帧图像的数据
    int Len;
    QImage My_Image;           // 当前帧 QImage（RGB16 格式）
    QByteArray Rec_Data;
    int width;                 // 图像宽度（默认 1280）
    int length;                // 图像高度（默认 720）
    int   display_h_disp;      // 显示的水平分辨率
    int   display_v_disp;      // 显示的垂直分辨率
    uchar System_Mode;         // 0: 关闭显示  1: 打开显示
    uchar System_Gray=1;       // 灰度模式标志（预留）
    uchar System_GrayScale=0;  // 灰度等级（预留）
    QString ip;                // UDP 目标 IP（预留）
    QString port_buf;          // UDP 端口缓冲区（预留）
    int port;                  // UDP 端口号（预留）
    int frameCount=0;          // 当前帧率（每秒帧数）
    quint64 current_frame_sequence = 0;       // 当前帧序号（自增）
    quint64 latest_infer_sequence = 0;        // 最近一次推理完成的帧序号
    VisionFrameResult latest_infer_result;     // 最近一次推理结果
    DetectionMode detection_mode_ = DetectionMode::PlateRecognition; // 当前检测模式

    unsigned int offset_addr=0;          // DMA 偏移地址
    unsigned int send_offset_addr=0;     // 发送 DMA 偏移地址
    unsigned int send_dma_times_cnt=0;   // 发送 DMA 次数计数
    unsigned int rd_dma_times_cnt=0;     // 读取 DMA 次数计数
};

#endif // MAINWINDOW_H
