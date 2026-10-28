#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "inference/detection_mode.h"
#include "hardware/pango_fun.h"
class AdasController;
class QCloseEvent;

namespace Ui {
class MainWindow; }

// 保留原设备与预处理控制；AdasController 负责自有帧和ADAS应用流程。
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void set_single_view(bool single);
    void write_adas_report(const QString &path) const;
    void save_adas_screenshot(const QString &path) const;
    void set_detection_mode(DetectionMode mode);
    void closeEvent(QCloseEvent *event) override;

    int dma_len=0; // DMA 传输长度（DW 单位）

private slots:
    // 打开/关闭 PCIe 设备
    void on_Key1_clicked();

    void on_adasMode_clicked();

    // 开始/停止视频采集
    void on_connect_clicked();

    // 保存当前帧为图片
    void on_save_clicked();

    // 视频源切换（I2C 控制）
    void on_radioCamera_toggled(bool checked);
    void on_radioHDMI_toggled(bool checked);

    // FPGA 预处理开关（I2C 控制）
    void on_chkPreprocess_toggled(bool checked);
    void on_chkPlateOverlay_toggled(bool checked);
    void on_chkWhiteBalance_toggled(bool checked);
    void on_chkDehaze_toggled(bool checked);

    // 纯视频显示
    void on_cam1_clicked();

    // 停止并复位
    void on_cam2_clicked();

    // 测试 DMA 写操作
    void on_dma_write_clicked();

    // 测试 DMA 读操作
    void on_dma_read_clicked();

private:
    // 应用检测模式切换（更新 UI 按钮状态和推理线程模式）
    void apply_detection_mode(DetectionMode mode);

    pango_fun *pango_device;                  // FPGA PCIe 通信接口（唯一与 FPGA 交互的对象）
    AdasController *adas_ = nullptr;
    Ui::MainWindow *ui;
    uchar System_Mode;         // 0: 关闭显示  1: 打开显示
};

#endif // MAINWINDOW_H
