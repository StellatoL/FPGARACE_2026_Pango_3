// 原设备控制窗口；ADAS 帧所有权、推理和绘制由 AdasController 管理。
#include "ui/mainwindow.h"
#include "ui/adas_controller.h"
#include <QCloseEvent>
#include "ui_mainwindow.h"
#include <QString>
#include <QApplication>
#include <QCoreApplication>
#include <QScreen>
#include <QPixmap>
#include <QFileDialog>
#include <QThread>
#include <QPainter>
#include <QDir>
#include <QStandardPaths>
#include <uchar.h>
#include <algorithm>
#include <cstring>
#include <iostream>

#include <chrono>
#include <unistd.h>

namespace {

// 将检测模式枚举转换为中文描述
QString detection_mode_text(DetectionMode mode) {
    switch (mode)
    {
        case DetectionMode::Adas:
            return QStringLiteral("ADAS 辅助驾驶");
        case DetectionMode::TrafficScene:
            return QStringLiteral("行人/斑马线识别");
        case DetectionMode::RawVideo:
            return QStringLiteral("纯视频");
        case DetectionMode::PlateRecognition:
        default:
            return QStringLiteral("车牌识别");
    }
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 初始化图像尺寸（显示区域大小由布局管理，图像按比例缩放填充）
    width = 1280;
    length = 720;
    ui->H_disp->setText("1280");
    ui->V_disp->setText("720");
    System_Mode = 0; // 初始状态：关闭显示

    ui->total_frame->setText("0");
    this->setWindowTitle(QStringLiteral("Pango PCIe — 视频传输与智能识别"));

    // 创建 FPGA 通信接口（独立线程处理 PCIe DMA）
    pango_device = new pango_fun();

    // 预分配显示用 QPixmap（1280×720），后续每帧通过 convertFromImage 原地复用
    pixmap = QPixmap(width, length);
    pixmap.fill(Qt::black);

    // 帧率定时器：每秒触发一次 rate_cacul() 计算实时 FPS
    fps_timer_ = new QTimer(this);
    fps_timer_->setInterval(1000);
    connect(fps_timer_, &QTimer::timeout, this, &MainWindow::rate_cacul);

    // No hardware is opened or queried by file playback/startup.
    adas_ = new AdasController(this, pango_device, ui);
    connect(adas_, &AdasController::mode_requested, this, &MainWindow::apply_detection_mode);
    apply_detection_mode(DetectionMode::Adas);
}

// 每秒帧率计算（由 fps_timer_ 驱动）
void MainWindow::rate_cacul()
{
    if (adas_) return; // Controller reports actual ingress and rendered frame rates.
    const int current_fps = recvImageNum - recvImageNum_pre;
    recvImageNum_pre = recvImageNum;
    ui->frame->setText(QStringLiteral("%1 fps").arg(current_fps));

    // 根据帧率高低切换颜色：≥25 绿色，≥15 黄色，<15 红色
    const char *color = current_fps >= 25 ? "#00aa00" :
                        current_fps >= 15 ? "#aaaa00" : "#cc0000";
    ui->frame->setStyleSheet(
        QStringLiteral("QLabel { color: %1; }").arg(color));
}

// 视频由 AdasController 定时更新 QLabel。
void MainWindow::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
}

// 切换至交通场景检测模式
void MainWindow::on_Gray_clicked()
{
    apply_detection_mode(DetectionMode::TrafficScene);
}

// 打开 / 关闭 PCIe 设备
void MainWindow::on_Key1_clicked()
{
    if (System_Mode == 1 && adas_ && !adas_->stop_source()) return;
    QByteArray send;

    if (System_Mode == 0) // 打开显示
    {
        int dev_pg = -1;
        ui->Key1->setText("关闭");
        System_Mode = 1;

        // 打开 PCIe 设备文件 /dev/pango_pci_driver
        dev_pg = pango_device->getDevice();
        qDebug() << "dev_pg is  " << dev_pg;
        if (dev_pg != -1)
        {
            qDebug() << "open success\n ";
            // 显示 PCIe 设备信息
            ui->HostAddress->setText((QString("%1").arg(
                pango_device->command_operation.get_pci_dev_info.vendor_id,
                4, 16, QLatin1Char('0')).toUpper()));
            ui->HostPort->setText(QString("GEN%1").arg(
                pango_device->command_operation.get_pci_dev_info.link_speed, 0, 10));
            ui->RemAddress->setText(QString("%1").arg(
                pango_device->command_operation.get_pci_dev_info.link_width, 0, 10));
            ui->RemPort->setText(QString("%1").arg(
                pango_device->command_operation.get_pci_dev_info.mps, 0, 10));

            // 初始化 DMA 参数：长度 2560 字节（转为 DW），偏移 0
            pango_device->dma_oper->current_len = 2560 >> 2; // conv to DW
            pango_device->dma_oper->offset_addr = 0;
            pango_device->dma_map(1); // 映射 DMA 缓冲
        }
        else
        {
            System_Mode = 0;
            ui->Key1->setText(QStringLiteral("打开"));
            qDebug() << "open failed\n ";
            ui->HostAddress->setText("error");
            ui->HostPort->setText("error");
            ui->RemAddress->setText("error");
            ui->RemPort->setText("error");
        }
    }
    else if (System_Mode == 1) // 关闭显示
    {
        ui->Key1->setText("打开");
        send = "0";
        System_Mode = 0;

        // 清空显示区域（字号由 style.qss 的 QLabel#Label 控制，无需代码设置）
        ui->Label->clear();
        ui->Label->setText(QStringLiteral("图像显示区域"));
        My_Image = QImage(); // 清除缓存帧，避免后续 resize 时残留旧画面

        // 重置帧率显示
        ui->frame->setText("0 fps");
        ui->frame->setStyleSheet("QLabel { color: #00aa00; }");
        fps_timer_->stop();

        // 解除 DMA 映射
        pango_device->dma_map(0);
        recvImageNum = 0;

        // 使过期的推理结果失效（避免显示旧帧叠加层）
        latest_infer_result = VisionFrameResult();
        latest_infer_sequence = current_frame_sequence + 1;
    }
}

MainWindow::~MainWindow()
{
    if (adas_) adas_->shutdown();
    delete ui;
}

bool MainWindow::startup_adas_with_file(const QString &path) { return adas_->start_file(path); }
void MainWindow::set_single_view(bool single) { adas_->set_layout(single?adas::Layout::Single:adas::Layout::Mosaic); }
void MainWindow::write_adas_report(const QString &path) const { adas_->write_report(path); }
void MainWindow::save_adas_screenshot(const QString &path) const {
    if(!path.isEmpty())const_cast<MainWindow*>(this)->grab().save(path);
}
void MainWindow::set_detection_mode(DetectionMode mode) { apply_detection_mode(mode); }
void MainWindow::closeEvent(QCloseEvent *event) {
    if(adas_&&!adas_->shutdown()){event->ignore();return;}
    event->accept();
}

// 开始 / 停止采集视频
void MainWindow::on_connect_clicked()
{
    if (ui->connect->text() == "采集视频")
    {
        if (adas_ && !adas_->prepare_live()) return;
        // 确保 PCIe 设备已打开
        if (pango_device->dev_fd < 0)
        {
            const int dev_pg = pango_device->getDevice();
            if (dev_pg == -1)
            {
                qWarning() << "failed to open /dev/pango_pci_driver before capture";
                if (adas_) adas_->stop_source();
                return;
            }
        }

        // 启动 FPGA 数据采集线程
        pango_device->stopped = false;
        if (!pango_device->isRunning())
        {
            pango_device->start();
        }

        // 读取 DMA 长度参数（用户可在 UI 配置）
        bool conversionOk;
        dma_len = ui->dma_len->text().toInt(&conversionOk, 10);

        // 通过 PIO 写寄存器通知 FPGA 开始发送数据
        pango_device->pio_write(0xffffffe5, 0, 0);

        // 启动帧率定时器
        fps_timer_->start();

        ui->connect->setText("关闭采集");
    }
    else
    {
        // 通知 FPGA 停止发送数据
        pango_device->stopped = true;
        pango_device->pio_write(0xffffff00, 0, 0);

        // 停止帧率定时器并重置显示
        fps_timer_->stop();
        ui->frame->setText("0 fps");
        ui->frame->setStyleSheet("QLabel { color: #00aa00; }");

        // 使过期推理结果失效
        latest_infer_result = VisionFrameResult();
        latest_infer_sequence = current_frame_sequence + 1;

        ui->connect->setText("采集视频");
        if (adas_) adas_->stop_source();
    }
}

// 保存当前帧为 PNG 图片
void MainWindow::on_save_clicked()
{
    // 从显示区域截图
    QPixmap pixmap = ui->Label->grab();

    // 保存目录：优先系统图片目录，回退到可执行文件所在目录。
    // （原先写死的 Windows 路径 "D:/qt/..." 在 ARM/Linux 上不存在，保存会静默失败）
    QString saveDir =
        QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    if (saveDir.isEmpty()) {
        saveDir = QCoreApplication::applicationDirPath();
    }
    QDir().mkpath(saveDir); // 确保目录存在

    static int num = 0;
    const QString filePath =
        saveDir + "/pic_" + QString::number(num) + ".png";
    if (pixmap.save(filePath)) {
        num++;
        qDebug() << "保存成功：" << filePath;
    } else {
        qWarning() << "保存失败：" << filePath;
    }
}

// 录制视频（预留功能，未实现）
void MainWindow::on_save_video_clicked()
{
    QPixmap pixmap = ui->Label->grab();
    // 视频编码功能预留
}

// 切换至纯视频模式（不做推理，仅显示接收的视频流）
void MainWindow::on_cam1_clicked()
{
    apply_detection_mode(DetectionMode::RawVideo);
}

// 复位：停止采集、清空显示、重置所有状态到初始值
//
// 通信链路回顾（双向）：
//   ARM → FPGA:  PIO write (BAR0) — 启动/停止/配置
//   FPGA → ARM:  DMA        — 图像帧数据
//   FPGA → ARM:  PIO read   — 帧完成状态 + 写索引 (经内核驱动)
// 复位需要关闭 ARM→FPGA 控制通道和 FPGA→ARM 数据通道。
void MainWindow::on_cam2_clicked()
{
    if (adas_ && !adas_->stop_source()) return;
    // 1. 先置停止标志，让采集线程 while(!stopped) 在下一次 dma_read
    //    返回后退出（与 on_connect_clicked 停止分支保持一致）
    pango_device->stopped = true;

    // 2. 通知 FPGA 停止发送数据（ARM→FPGA PIO write: 地址 0 写入 0xffffff00）
    //    FPGA 侧 pio_crtl.v 收到后拉低 start_flag → video_crtl 回到 IDLE →
    //    pcie_tx_fun 停止 DMA 请求 → 内核驱动 read() 返回
    if (pango_device->dev_fd >= 0) {
        pango_device->pio_write(0xffffff00, 0, 0);
    }

    // 3. 停止帧率定时器
    fps_timer_->stop();

    // 4. 重置全部应用层状态
    recvImageNum = 0;
    recvImageNum_pre = 0;
    current_frame_sequence = 0;
    latest_infer_sequence = current_frame_sequence + 1;
    latest_infer_result = VisionFrameResult();

    // 5. 清空显示：回到占位提示（比黑图更利于窗口缩放时自适应）
    My_Image = QImage();
    ui->Label->clear();
    ui->Label->setText(QStringLiteral("图像显示区域"));

    // 6. 重置 UI
    ui->total_frame->setText("0");
    ui->frame->setText("0 fps");
    ui->frame->setStyleSheet("QLabel { color: #00aa00; }");
    ui->connect->setText("采集视频");

    qDebug() << "[RESET] FPGA stopped + state cleared"
             << "| thread running:" << pango_device->isRunning();
}

// 测试 DMA 写操作：向 FPGA 发送启动信号
void MainWindow::on_dma_write_clicked()
{
    if (System_Mode != 1 || !pango_device->mem_pio_addr) return;
    pango_device->pio_write(0xffffffe5, 0, 0);
}

// 测试 DMA 读操作：向 FPGA 发送读请求
void MainWindow::on_dma_read_clicked()
{
    if (System_Mode != 1 || !pango_device->mem_pio_addr) return;
    pango_device->pio_write(0x00000001, 0, 4);
}

// 切换至车牌识别模式
void MainWindow::on_pushButton_clicked()
{
    apply_detection_mode(DetectionMode::PlateRecognition);
}

// 向推理线程提交当前帧（指针传递，零拷贝）


// 处理推理线程返回的检测结果
// 使用帧序号过滤过期结果（避免模式切换或停止采集后显示旧结果）


// 在 QPixmap 上绘制检测框叠加层和告警标记
// 此函数在绘制每帧时调用，使用 latest_infer_result 缓存


// 应用检测模式切换
// 1. 更新本地模式和 UI 按钮状态
// 2. 通知推理线程切换模式
// 3. 使过期结果失效，防止显示旧模式叠加层
// 4. 刷新当前画面（移除旧叠加层）
void MainWindow::apply_detection_mode(DetectionMode mode)
{
    detection_mode_ = mode;

    // 使旧模式推理结果失效
    latest_infer_result = VisionFrameResult();
    latest_infer_sequence = current_frame_sequence + 1;

    if (adas_) adas_->set_mode(mode);

    // 模式按钮为排他 checkable（autoExclusive），高亮当前模式对应按钮。
    // 用 blockSignals 避免 setChecked 反过来触发 clicked → 递归调用本函数。
    ui->pushButton->blockSignals(true);
    ui->Gray->blockSignals(true);
    ui->cam1->blockSignals(true);
    ui->pushButton->setChecked(mode == DetectionMode::PlateRecognition);
    ui->Gray->setChecked(mode == DetectionMode::TrafficScene);
    ui->cam1->setChecked(mode == DetectionMode::RawVideo);
    ui->pushButton->blockSignals(false);
    ui->Gray->blockSignals(false);
    ui->cam1->blockSignals(false);

    statusBar()->showMessage(QStringLiteral("当前检测模式：%1")
                                 .arg(detection_mode_text(mode)));
}

// ============================================================
// I2C 视频源切换槽
// ============================================================

void MainWindow::on_radioCamera_toggled(bool checked) {
    if (!checked) return;
    if (!pango_device) return;
    if (pango_device->i2c_switch_to_camera()) {
        if (adas_) adas_->physical_source_changed();
        statusBar()->showMessage(u8"视频源已切换至：摄像头", 3000);
    } else {
        statusBar()->showMessage(u8"⚠ 摄像头切换失败（I2C 通信错误）", 5000);
        QSignalBlocker b(ui->radioHDMI);
        ui->radioHDMI->setChecked(true); // 回退
    }
}

// ============================================================
// FPGA 预处理开关槽
// ============================================================

void MainWindow::on_chkPreprocess_toggled(bool checked) {
    if (pango_device)
        pango_device->i2c_set_preprocess(checked);
}

void MainWindow::on_chkPlateOverlay_toggled(bool checked) {
    if (pango_device)
        pango_device->i2c_set_plate_overlay(checked);
}

void MainWindow::on_chkWhiteBalance_toggled(bool checked) {
    if (pango_device)
        pango_device->i2c_set_white_balance(checked);
}

void MainWindow::on_chkDehaze_toggled(bool checked) {
    if (pango_device)
        pango_device->i2c_set_dehaze(checked);
}

void MainWindow::on_radioHDMI_toggled(bool checked) {
    if (!checked) return;
    if (!pango_device) return;
    if (pango_device->i2c_switch_to_hdmi()) {
        if (adas_) adas_->physical_source_changed();
        statusBar()->showMessage(u8"视频源已切换至：HDMI", 3000);
    } else {
        statusBar()->showMessage(u8"⚠ HDMI 切换失败（I2C 通信错误）", 5000);
        QSignalBlocker b(ui->radioCamera);
        ui->radioCamera->setChecked(true); // 回退
    }
}
