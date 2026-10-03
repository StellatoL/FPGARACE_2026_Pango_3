// 主窗口实现：PCIe 视频传输上位机
//
// 系统架构（数据流）：
// ┌──────────────┐    PCIe DMA     ┌─────────────────┐
// │ FPGA 摄像头   │ ──────────────→ │ pango_fun (线程) │
// │ (1280×720)   │   4 缓冲环      │ img_buf[4]      │
// └──────────────┘                 └──────┬──────────┘
//                                         │ send_wr_done(index)
//                                         ▼
//                              ┌─────────────────────┐
//                              │ MainWindow          │
//                              │ recv_img_dis()      │ ← 主线程 (GUI)
//                              │ ├─ QImage 显示       │
//                              │ ├─ draw_overlay()   │
//                              │ └─ submit_infer()   │
//                              └─────────┬───────────┘
//                                        │ 指针传递（零拷贝）
//                                        ▼
//                              ┌─────────────────────┐
//                              │ PlateInferThread    │ ← 推理线程
//                              │ ├─ PlateRecognizer  │
//                              │ └─ TrafficDetector  │
//                              └─────────┬───────────┘
//                                        │ result_ready 信号
//                                        ▼
//                              ┌─────────────────────┐
//                              │ MainWindow          │
//                              │ handle_result()     │ ← 更新状态栏
//                              └─────────────────────┘

#include "ui/mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QApplication>
#include <QCoreApplication>
#include <QScreen>
#include <QPixmap>
#include <QFileDialog>
#include <QThread>
#include <QPainter>
#include <QShortcut>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <uchar.h>
#include <algorithm>
#include <cstring>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <unistd.h>

namespace {

// 将检测模式枚举转换为中文描述
QString detection_mode_text(DetectionMode mode) {
    switch (mode)
    {
        case DetectionMode::TrafficScene:
            return QStringLiteral("行人/斑马线识别");
        case DetectionMode::RawVideo:
            return QStringLiteral("纯视频");
        case DetectionMode::Adas:
            return QStringLiteral("ADAS 辅助驾驶");
        case DetectionMode::PlateRecognition:
        default:
            return QStringLiteral("车牌识别");
    }
}

// 构建状态栏文本：包含检测模式、错误信息或状态描述
QString vision_status_text(const VisionFrameResult &result) {
    if (result.mode == DetectionMode::RawVideo) {
        return QStringLiteral("当前检测模式：纯视频");
    }
    const QString mode_text = detection_mode_text(result.mode);
    if (!result.error_message.empty())
    {
        return QStringLiteral("%1：%2")
            .arg(mode_text, QString::fromUtf8(result.error_message.c_str()));
    }
    if (!result.status_text.empty())
    {
        return QStringLiteral("%1：%2")
            .arg(mode_text, QString::fromUtf8(result.status_text.c_str()));
    }
    return QStringLiteral("当前检测模式：%1").arg(mode_text);
}

// 根据背景色亮度自动选择叠加文字颜色（黑底白字 / 白底黑字）
// 使用 NTSC 加权亮度公式：Y = 0.299R + 0.587G + 0.114B
QColor overlay_text_color(const QColor &background) {
    const int luminance =
        background.red() * 299 + background.green() * 587 +
        background.blue() * 114;
    return luminance >= 128000 ? Qt::black : Qt::white;
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
    // FPGA DMA 传输完成 → recv_img_dis 处理新帧
    connect(pango_device, SIGNAL(send_wr_done(int)), this, SLOT(recv_img_dis(int)));

    // 创建推理工作线程
    infer_thread = new PlateInferThread(this);
    infer_thread->set_model_root(QCoreApplication::applicationDirPath());
    // 推理完成信号（QueuedConnection 确保 UI 线程安全）
    connect(infer_thread, &PlateInferThread::result_ready, this,
            &MainWindow::handle_infer_result_ready, Qt::QueuedConnection);
    infer_thread->start();
    apply_detection_mode(DetectionMode::PlateRecognition);

    // 预分配显示用 QPixmap（1280×720），后续每帧通过 convertFromImage 原地复用
    pixmap = QPixmap(width, length);
    pixmap.fill(Qt::black);

    // 帧率定时器：每秒触发一次 rate_cacul() 计算实时 FPS
    fps_timer_ = new QTimer(this);
    fps_timer_->setInterval(1000);
    connect(fps_timer_, &QTimer::timeout, this, &MainWindow::rate_cacul);

    // 读取当前 I2C 视频源，同步 UI 状态
    {
        int src = pango_device->i2c_query_source();
        QSignalBlocker b1(ui->radioCamera);
        QSignalBlocker b2(ui->radioHDMI);
        if (src == 1) {
            ui->radioHDMI->setChecked(true);
            ui->radioCamera->setChecked(false);
        } else {
            ui->radioCamera->setChecked(true); // 默认摄像头
        }

        // 读回 FPGA 预处理开关状态, 同步 UI
        QSignalBlocker bp1(ui->chkPreprocess);
        QSignalBlocker bp2(ui->chkPlateOverlay);
        QSignalBlocker bp3(ui->chkWhiteBalance);
        QSignalBlocker bp4(ui->chkDehaze);
        ui->chkPreprocess->setChecked(pango_device->i2c_read_reg(0x03) == 1);
        ui->chkPlateOverlay->setChecked(pango_device->i2c_read_reg(0x04) == 1);
        ui->chkWhiteBalance->setChecked(pango_device->i2c_read_reg(0x05) == 1);
        ui->chkDehaze->setChecked(pango_device->i2c_read_reg(0x06) == 1);
    }

    // ===== 文件回放源（与 PCIe 同构：写 img_buf 四槽 + 同一显示槽）=====
    file_source_ = new FileVideoSource(pango_device->img_buf, IMG_SIZE, this);
    connect(file_source_, &FileVideoSource::frame_ready,
            this, &MainWindow::recv_img_dis, Qt::QueuedConnection);
    connect(file_source_, &FileVideoSource::playback_finished,
            this, &MainWindow::handle_file_playback_finished, Qt::QueuedConnection);
    connect(file_source_, &FileVideoSource::source_error,
            this, &MainWindow::handle_file_source_error, Qt::QueuedConnection);

    // ===== ADAS 报警闪烁时钟（2Hz；暂停/低帧率时仍刷新叠加）=====
    adas_blink_clock_.start();
    adas_blink_timer_ = new QTimer(this);
    adas_blink_timer_->setInterval(500);
    connect(adas_blink_timer_, &QTimer::timeout, this, [this]() {
        // 仅 ADAS 模式且有画面时重绘（叠加层闪烁相位由时钟计算）
        if (detection_mode_ == DetectionMode::Adas && !My_Image.isNull()) {
            pixmap.convertFromImage(My_Image);
            draw_detection_overlay(pixmap);
            draw_adas_overlay(pixmap);
            show_scaled_pixmap(pixmap);
        }
    });
    // ADAS 模式下才启动闪烁定时器（apply_detection_mode 中切换）

    // ===== 导航 ROI 调整（4 个 spinbox）=====
    connect(ui->spinNavX, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::handle_nav_roi_changed);
    connect(ui->spinNavY, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::handle_nav_roi_changed);
    connect(ui->spinNavW, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::handle_nav_roi_changed);
    connect(ui->spinNavH, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::handle_nav_roi_changed);

    // ===== 键盘快捷键（赛题"按键模拟"要求；板上无 FPGA 按键通路时的操作入口）=====
    auto *sc_lane_l = new QShortcut(QKeySequence(Qt::Key_1), this);
    connect(sc_lane_l, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_lane_left);
    auto *sc_lane_c = new QShortcut(QKeySequence(Qt::Key_2), this);
    connect(sc_lane_c, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_lane_center);
    auto *sc_lane_r = new QShortcut(QKeySequence(Qt::Key_3), this);
    connect(sc_lane_r, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_lane_right);
    auto *sc_drive = new QShortcut(QKeySequence(Qt::Key_Space), this);
    connect(sc_drive, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_drive_toggle);
    auto *sc_sig_l = new QShortcut(QKeySequence(Qt::Key_A), this);
    connect(sc_sig_l, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_signal_left);
    auto *sc_sig_r = new QShortcut(QKeySequence(Qt::Key_D), this);
    connect(sc_sig_r, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_signal_right);
    auto *sc_force_l = new QShortcut(QKeySequence(Qt::Key_J), this);
    connect(sc_force_l, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_force_lane_left);
    auto *sc_force_r = new QShortcut(QKeySequence(Qt::Key_L), this);
    connect(sc_force_r, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_force_lane_right);
    auto *sc_nav = new QShortcut(QKeySequence(Qt::Key_N), this);
    connect(sc_nav, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_nav_next);
    auto *sc_pause = new QShortcut(QKeySequence(Qt::Key_P), this);
    connect(sc_pause, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_pause);
    auto *sc_reset = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(sc_reset, &QShortcut::activated,
            this, &MainWindow::handle_hotkey_reset);
}

// FPGA DMA 传输完成回调（index: FPGA 帧序号 0-3，与 img_buf 槽位一一对应）
// 驱动/DMA 根据 FPGA 帧序号将数据固定写入 img_buf + index*IMG_SIZE，
// 因此此处直接用 index 定位帧数据，无需 ARM 侧槽位轮转。
void MainWindow::recv_img_dis(int index)
{
    recvImageNum++;                   // 累计帧计数
    current_frame_sequence++;         // 帧序号自增

    // 更新总帧数显示
    ui->total_frame->setText(QString::number(recvImageNum));

    // 提交给推理线程（仅指针传递，零拷贝）—— 纯视频模式下跳过推理
    if (detection_mode_ != DetectionMode::RawVideo) {
        submit_infer_frame(index);
    }

    // 显示节流：距上次渲染不足 33ms 则跳过（推理模式保护主线程，纯视频不限帧）
    if (detection_mode_ != DetectionMode::RawVideo) {
        static auto last_render = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - last_render < std::chrono::milliseconds(33)) {
            return;
        }
        last_render = now;
    }

    // 构造 QImage：直接从 DMA 缓冲区读取 RGB565 数据（零拷贝）
    // 图像处理演示模式：先对帧副本应用所选算法再显示（评分项演示）
    const uchar *frame_ptr = pango_device->img_buf + IMG_SIZE * index;
    if (processing_demo_index_ != 0) {
        apply_processing_demo(index);
        My_Image = proc_image_;
    } else {
        My_Image = QImage(frame_ptr, width, length, width * 2,
                          QImage::Format_RGB16);
    }

    // stride 诊断：每 30 帧输出一次（index 现为 ARM 侧环形槽位号 0..3）
    {
      static int diag_skip = 0;
      if (++diag_skip >= 30) {
        diag_skip = 0;
        qDebug().nospace()
            << "[DIAG] slot:" << index
            << " BPL:" << My_Image.bytesPerLine()
            << " exp:" << (width * 2)
            << " ok:" << (My_Image.bytesPerLine() == width * 2)
            << " | data:" << QByteArray(reinterpret_cast<const char *>(frame_ptr), 16).toHex();
      }
    }

    // 复用预分配的 QPixmap，避免每帧分配新的 X11 pixmap
    pixmap.convertFromImage(My_Image);
    draw_detection_overlay(pixmap);
    // ADAS 模式：叠加 HUD/箭头/报警横幅/LED 指示
    if (detection_mode_ == DetectionMode::Adas) {
        draw_adas_overlay(pixmap);
    }
    show_scaled_pixmap(pixmap);
}

// 按当前 Label 尺寸保持宽高比缩放显示。原始 pixmap 始终是 1280×720 原生坐标，
// 叠加层已在原生坐标绘制完毕，这里只对最终画面做等比缩放，不影响检测框坐标计算。
// 视频帧率优先，使用 FastTransformation（最近邻），避免每帧平滑缩放拖慢主线程。
void MainWindow::show_scaled_pixmap(const QPixmap &native_pixmap)
{
    if (native_pixmap.isNull()) {
        return;
    }
    const QSize target = ui->Label->size();
    if (target.width() <= 0 || target.height() <= 0) {
        ui->Label->setPixmap(native_pixmap);
        return;
    }
    ui->Label->setPixmap(native_pixmap.scaled(
        target, Qt::KeepAspectRatio, Qt::FastTransformation));
}

// 每秒帧率计算（由 fps_timer_ 驱动）
void MainWindow::rate_cacul()
{
    const int current_fps = recvImageNum - recvImageNum_pre;
    recvImageNum_pre = recvImageNum;
    ui->frame->setText(QStringLiteral("%1 fps").arg(current_fps));

    // 根据帧率高低切换颜色：≥25 绿色，≥15 黄色，<15 红色
    const char *color = current_fps >= 25 ? "#00aa00" :
                        current_fps >= 15 ? "#aaaa00" : "#cc0000";
    ui->frame->setStyleSheet(
        QStringLiteral("QLabel { color: %1; }").arg(color));
}

// 每收到一帧图都会触发一次画图事件（当前为空实现，由 recv_img_dis 处理显示）
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
    // 安全停止推理线程
    if (infer_thread != nullptr)
    {
        infer_thread->stop();
        infer_thread->wait();
    }
    // 安全停止文件回放线程
    if (file_source_ != nullptr)
    {
        file_source_->close();
    }
    delete ui;
}

// 启动即进入 ADAS 模式并回放指定文件（--adas 命令行入口）
void MainWindow::startup_adas_with_file(const QString &path)
{
    if (file_source_ == nullptr || path.isEmpty()) return;

    if (!file_source_->open(path)) {
        statusBar()->showMessage(
            QStringLiteral("⚠ 无法打开文件：%1").arg(path), 5000);
        return;
    }
    ui->lblFileName->setText(QFileInfo(path).fileName());
    ui->btnPlayPause->setEnabled(true);
    ui->btnPlayPause->setText(QStringLiteral("暂停"));
    ui->radioFile->setChecked(true);   // 触发 start_file_playback（停 PCIe）
    apply_detection_mode(DetectionMode::Adas);
    statusBar()->showMessage(
        QStringLiteral("ADAS 模式 + 文件回放：%1").arg(QFileInfo(path).fileName()),
        5000);
}

// 开始 / 停止采集视频（PCIe 源；与文件回放互斥）
void MainWindow::on_connect_clicked()
{
    if (ui->connect->text() == "采集视频")
    {
        start_pcie_capture();
    }
    else
    {
        stop_pcie_capture();
    }
}

// 启动 PCIe 采集（自动停止文件回放）
void MainWindow::start_pcie_capture()
{
    // 确保 PCIe 设备已打开
    if (pango_device->dev_fd < 0)
    {
        const int dev_pg = pango_device->getDevice();
        if (dev_pg == -1)
        {
            qWarning() << "failed to open /dev/pango_pci_driver before capture";
            statusBar()->showMessage(
                QStringLiteral("⚠ PCIe 设备打开失败（/dev/pango_pci_driver 不存在？）"),
                5000);
            return;
        }
    }

    // 互斥：停止文件回放
    stop_file_playback();

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

// 停止 PCIe 采集
void MainWindow::stop_pcie_capture()
{
    // 通知 FPGA 停止发送数据
    pango_device->stopped = true;
    if (pango_device->dev_fd >= 0)
    {
        pango_device->pio_write(0xffffff00, 0, 0);
    }

    // 停止帧率定时器并重置显示
    fps_timer_->stop();
    ui->frame->setText("0 fps");
    ui->frame->setStyleSheet("QLabel { color: #00aa00; }");

    // 使过期推理结果失效
    latest_infer_result = VisionFrameResult();
    latest_infer_sequence = current_frame_sequence + 1;

    ui->connect->setText("采集视频");
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
    // 1. 先置停止标志，让采集线程 while(!stopped) 在下一次 dma_read
    //    返回后退出（与 stop_pcie_capture 保持一致）
    pango_device->stopped = true;

    // 2. 通知 FPGA 停止发送数据（ARM→FPGA PIO write: 地址 0 写入 0xffffff00）
    //    FPGA 侧 pio_crtl.v 收到后拉低 start_flag → video_crtl 回到 IDLE →
    //    pcie_tx_fun 停止 DMA 请求 → 内核驱动 read() 返回
    if (pango_device->dev_fd >= 0) {
        pango_device->pio_write(0xffffff00, 0, 0);
    }

    // 3. 停止文件回放（若在播放）
    stop_file_playback();

    // 4. 停止帧率定时器
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
    pango_device->pio_write(0xffffffe5, 0, 0);
}

// 测试 DMA 读操作：向 FPGA 发送读请求
void MainWindow::on_dma_read_clicked()
{
    pango_device->pio_write(0x00000001, 0, 4);
}

// 切换至车牌识别模式
void MainWindow::on_pushButton_clicked()
{
    apply_detection_mode(DetectionMode::PlateRecognition);
}

// 向推理线程提交当前帧（指针传递，零拷贝）
void MainWindow::submit_infer_frame(int index)
{
    if (infer_thread == nullptr || !infer_thread->isRunning())
    {
        return;
    }

    const uchar *frame_ptr = pango_device->img_buf + IMG_SIZE * index;
    infer_thread->submit_frame(current_frame_sequence, frame_ptr, width, length,
                               width * 2);
}

// 处理推理线程返回的检测结果
// 使用帧序号过滤过期结果（避免模式切换或停止采集后显示旧结果）
void MainWindow::handle_infer_result_ready(quint64 sequence,
                                           VisionFrameResult result)
{
    if (sequence < latest_infer_sequence)
    {
        return; // 丢弃过期结果
    }
    latest_infer_sequence = sequence;
    latest_infer_result = result;

    // ADAS 模式：用最新感知重步决策引擎（含手动导航覆盖；ego 变更时同样重步）
    if (detection_mode_ == DetectionMode::Adas && result.adas.valid)
    {
        last_perception_ = result.adas;
        update_ego_state();
    }

    statusBar()->showMessage(vision_status_text(result));
}

// 在 QPixmap 上绘制检测框叠加层和告警标记
// 此函数在绘制每帧时调用，使用 latest_infer_result 缓存
void MainWindow::draw_detection_overlay(QPixmap &pixmap)
{
    // 无有效结果或模型未就绪时不绘制
    if (!latest_infer_result.model_ready ||
        (latest_infer_result.overlays.empty() &&
         !latest_infer_result.alert_marker.visible))
    {
        return;
    }

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // 默认画笔：绿色，宽度 3px
    QPen pen(QColor(0, 255, 0));
    pen.setWidth(3);
    painter.setPen(pen);

    // 默认字体：24pt 加粗
    QFont font = painter.font();
    font.setPointSize(24);
    font.setBold(true);
    painter.setFont(font);

    // 绘制所有检测框 + 文本标签
    for (const auto &overlay : latest_infer_result.overlays)
    {
        QRect rect(overlay.left, overlay.top,
                   std::max(0, overlay.right - overlay.left),
                   std::max(0, overlay.bottom - overlay.top));
        if (rect.width() <= 0 || rect.height() <= 0)
        {
            continue;
        }

        // 使用该类别配置的颜色绘制检测框
        const QColor color(overlay.color_r, overlay.color_g, overlay.color_b);
        pen.setColor(color);
        painter.setPen(pen);
        painter.drawRect(rect);

        // 绘制文本标签（如 "京A12345" 或 "zebra 85%"）
        const QString label = QString::fromUtf8(overlay.label.c_str());
        if (label.isEmpty())
        {
            continue;
        }

        // 文本背景：位于检测框上方，与框同色
        const QRect text_rect =
            painter.fontMetrics().boundingRect(label).adjusted(-6, -4, 6, 4);
        QRect background(rect.left(), std::max(0, rect.top() - text_rect.height()),
                         text_rect.width(), text_rect.height());
        painter.fillRect(background, color);

        // 文本颜色：根据背景亮度自动选择黑/白
        painter.setPen(overlay_text_color(color));
        painter.drawText(background.adjusted(4, 0, 0, 0),
                         Qt::AlignVCenter | Qt::AlignLeft, label);
        painter.setPen(pen);
    }

    // 绘制告警标记（如行人闯红灯时在斑马线上绘制红色圆形标记）
    if (latest_infer_result.alert_marker.visible)
    {
        const auto &marker = latest_infer_result.alert_marker;
        const QColor fill_color(marker.fill_r, marker.fill_g, marker.fill_b);
        const QColor stroke_color(marker.stroke_r, marker.stroke_g, marker.stroke_b);

        // 填充圆
        painter.setBrush(fill_color);
        painter.setPen(QPen(stroke_color, 5));
        painter.drawEllipse(QPoint(marker.center_x, marker.center_y), marker.radius,
                            marker.radius);

        // 圆内文字 "!"
        painter.setPen(stroke_color);
        QFont alert_font = painter.font();
        alert_font.setPointSize(std::max(24, marker.radius));
        alert_font.setBold(true);
        painter.setFont(alert_font);

        const QString marker_text = marker.text.empty()
                                        ? QStringLiteral("!")
                                        : QString::fromUtf8(marker.text.c_str());
        const QRect marker_rect(marker.center_x - marker.radius,
                                marker.center_y - marker.radius, marker.radius * 2,
                                marker.radius * 2);
        painter.drawText(marker_rect, Qt::AlignCenter, marker_text);
    }
}

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

    if (infer_thread != nullptr)
    {
        infer_thread->set_mode(mode);
    }

    // 模式按钮为排他 checkable（autoExclusive），高亮当前模式对应按钮。
    // 用 blockSignals 避免 setChecked 反过来触发 clicked → 递归调用本函数。
    ui->pushButton->blockSignals(true);
    ui->Gray->blockSignals(true);
    ui->cam1->blockSignals(true);
    ui->btnAdas->blockSignals(true);
    ui->pushButton->setChecked(mode == DetectionMode::PlateRecognition);
    ui->Gray->setChecked(mode == DetectionMode::TrafficScene);
    ui->cam1->setChecked(mode == DetectionMode::RawVideo);
    ui->btnAdas->setChecked(mode == DetectionMode::Adas);
    ui->pushButton->blockSignals(false);
    ui->Gray->blockSignals(false);
    ui->cam1->blockSignals(false);
    ui->btnAdas->blockSignals(false);

    // ADAS 模式：启动闪烁定时器并复位决策引擎滞回状态；
    // 其他模式：停闪烁定时器并清空决策显示
    // （构造期首次调用时定时器尚未创建，需空指针保护）
    if (mode == DetectionMode::Adas)
    {
        adas_engine_.reset();
        last_decision_ = adas::AdasDecision();
        last_perception_ = adas::AdasPerception();
        if (adas_blink_timer_ != nullptr)
        {
            adas_blink_timer_->start();
        }
    }
    else
    {
        if (adas_blink_timer_ != nullptr)
        {
            adas_blink_timer_->stop();
        }
        last_decision_ = adas::AdasDecision();
        refresh_adas_status_labels();
    }

    // 重绘当前帧（移除旧叠加层，显示新模式状态）
    if (!My_Image.isNull())
    {
        pixmap.convertFromImage(My_Image);
        draw_detection_overlay(pixmap);
        if (mode == DetectionMode::Adas)
        {
            draw_adas_overlay(pixmap);
        }
        show_scaled_pixmap(pixmap);
    }

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
        statusBar()->showMessage(u8"视频源已切换至：HDMI", 3000);
    } else {
        statusBar()->showMessage(u8"⚠ HDMI 切换失败（I2C 通信错误）", 5000);
        QSignalBlocker b(ui->radioCamera);
        ui->radioCamera->setChecked(true); // 回退
    }
}

// ============================================================
// ADAS 辅助驾驶模式（赛题选题三：四个现场场景）
// ============================================================

// 切换至 ADAS 辅助驾驶模式
void MainWindow::on_btnAdas_clicked()
{
    apply_detection_mode(DetectionMode::Adas);
}

// 车道选择（赛题场景1：按键切换车道）
void MainWindow::on_btnLaneL_clicked()
{
    ego_.lane = adas::LaneId::Left;
    update_ego_state();
}

void MainWindow::on_btnLaneC_clicked()
{
    ego_.lane = adas::LaneId::Center;
    update_ego_state();
}

void MainWindow::on_btnLaneR_clicked()
{
    ego_.lane = adas::LaneId::Right;
    update_ego_state();
}

// 行驶/停车切换（赛题场景1：按键模拟停止/行驶；红灯时置行驶即"强制行驶"）
void MainWindow::on_btnDrive_toggled(bool checked)
{
    ego_.driving = checked;
    ui->btnDrive->setText(checked ? QStringLiteral("行驶中（空格停车）")
                                  : QStringLiteral("已停车（空格行驶）"));
    update_ego_state();
}

// 转向灯（赛题场景2：按键模拟左/右转向）
void MainWindow::on_btnSigL_toggled(bool checked)
{
    if (checked) {
        ego_.signal = adas::TurnSignal::Left;
        QSignalBlocker b(ui->btnSigR);
        ui->btnSigR->setChecked(false);
    } else if (ego_.signal == adas::TurnSignal::Left) {
        ego_.signal = adas::TurnSignal::None;
    }
    update_ego_state();
}

void MainWindow::on_btnSigR_toggled(bool checked)
{
    if (checked) {
        ego_.signal = adas::TurnSignal::Right;
        QSignalBlocker b(ui->btnSigL);
        ui->btnSigL->setChecked(false);
    } else if (ego_.signal == adas::TurnSignal::Right) {
        ego_.signal = adas::TurnSignal::None;
    }
    update_ego_state();
}

// 车速滑块（超速提醒的模拟车速）
void MainWindow::on_sliderSpeed_valueChanged(int value)
{
    ego_.speed_kmh = static_cast<float>(value);
    ui->lblSpeed->setText(QString::number(value));
    update_ego_state();
}

// 限速设置
void MainWindow::on_spinLimit_valueChanged(int value)
{
    ego_.speed_limit = static_cast<float>(value);
    update_ego_state();
}

// 强制变道指令（赛题场景2：强制实线变道 → 报警）
void MainWindow::on_btnForceLaneL_toggled(bool checked)
{
    if (checked) {
        ego_.force_lane_change = adas::TurnSignal::Left;
        QSignalBlocker b(ui->btnForceLaneR);
        ui->btnForceLaneR->setChecked(false);
    } else if (ego_.force_lane_change == adas::TurnSignal::Left) {
        ego_.force_lane_change = adas::TurnSignal::None;
    }
    update_ego_state();
}

void MainWindow::on_btnForceLaneR_toggled(bool checked)
{
    if (checked) {
        ego_.force_lane_change = adas::TurnSignal::Right;
        QSignalBlocker b(ui->btnForceLaneL);
        ui->btnForceLaneL->setChecked(false);
    } else if (ego_.force_lane_change == adas::TurnSignal::Right) {
        ego_.force_lane_change = adas::TurnSignal::None;
    }
    update_ego_state();
}

// 手动导航备份（N 键循环：自动→左转→直行→右转→自动）
// 赛题场景3 的演示备份：视频内导航箭头识别失效时手动注入方向
void MainWindow::on_btnNavNext_clicked()
{
    manual_nav_cycle_ = (manual_nav_cycle_ + 1) % 4;
    const char *nav_text;
    switch (manual_nav_cycle_) {
        case 0: nav_text = u8"手动：左转"; break;
        case 1: nav_text = u8"手动：直行"; break;
        case 2: nav_text = u8"手动：右转"; break;
        default: nav_text = u8"自动（识别视频箭头）"; break;
    }
    ui->lblNav->setText(QStringLiteral("导航：%1").arg(QString::fromUtf8(nav_text)));
    update_ego_state();
}

// 导航箭头 ROI 调整（适配不同导航 UI 布局）
void MainWindow::handle_nav_roi_changed()
{
    if (infer_thread != nullptr) {
        infer_thread->set_nav_roi(ui->spinNavX->value(), ui->spinNavY->value(),
                                  ui->spinNavW->value(), ui->spinNavH->value());
    }
}

// ADAS：更新本车状态后重步决策引擎（所有 ego 变更槽的汇聚点）
// 手动导航覆盖：manual_nav_cycle_ != 3 时以手动方向替代视觉识别结果
void MainWindow::update_ego_state()
{
    if (detection_mode_ != DetectionMode::Adas) {
        return;  // 非 ADAS 模式不决策（ego 状态保留，切回时生效）
    }

    adas::AdasPerception p = last_perception_;
    if (manual_nav_cycle_ != 3) {
        p.nav.detected = true;
        p.nav.dir = static_cast<adas::NavDir>(manual_nav_cycle_ + 1);
        p.nav.conf = 1.0f;
    }
    last_decision_ = adas_engine_.step(p, ego_);
    refresh_adas_status_labels();

    // 立即重绘当前帧（叠加层马上反映新决策，不等下一帧）
    if (!My_Image.isNull()) {
        pixmap.convertFromImage(My_Image);
        draw_detection_overlay(pixmap);
        draw_adas_overlay(pixmap);
        show_scaled_pixmap(pixmap);
    }
}

// ADAS：刷新右侧状态面板
void MainWindow::refresh_adas_status_labels()
{
    // 信号灯（带颜色）
    {
        QString text;
        QString color = "#888888";
        switch (last_decision_.light) {
            case adas::LightState::Red:
                text = QStringLiteral("红灯"); color = "#ff3333"; break;
            case adas::LightState::Green:
                text = QStringLiteral("绿灯"); color = "#33ff33"; break;
            case adas::LightState::Yellow:
                text = QStringLiteral("黄灯"); color = "#ffcc00"; break;
            case adas::LightState::Black:
                text = QStringLiteral("黑灯"); break;
            default:
                text = QStringLiteral("无信号"); break;
        }
        ui->lblLight->setText(text);
        ui->lblLight->setStyleSheet(
            QStringLiteral("QLabel { color: %1; font-weight: bold; }").arg(color));
    }

    // 车道线（原始感知：左/右类型）
    {
        const auto &lane = last_perception_.lane;
        QString text;
        if (lane.left_valid || lane.right_valid) {
            const QString left = lane.left_valid
                ? QString::fromUtf8(adas::line_type_text(lane.left_type)) : QStringLiteral("--");
            const QString right = lane.right_valid
                ? QString::fromUtf8(adas::line_type_text(lane.right_type)) : QStringLiteral("--");
            text = QStringLiteral("左%1 右%2").arg(left, right);
        } else {
            text = QStringLiteral("未检出");
        }
        ui->lblLane->setText(text);
    }

    // 决策（箭头方向 + 停/行）
    {
        QString text = QString::fromUtf8(adas::arrow_dir_text(last_decision_.arrow));
        if (last_decision_.stop_required) text += QStringLiteral(" | 停");
        else if (ego_.driving) text += QStringLiteral(" | 行");
        ui->lblDecision->setText(text);
    }

    // 报警
    {
        if (last_decision_.any_alarm()) {
            ui->lblAlarm->setText(QString::fromUtf8(last_decision_.status_text.c_str()));
            ui->lblAlarm->setStyleSheet(
                QStringLiteral("QLabel { color: #ff3333; font-weight: bold; }"));
        } else {
            ui->lblAlarm->setText(QStringLiteral("无"));
            ui->lblAlarm->setStyleSheet(QStringLiteral("QLabel { color: #888888; }"));
        }
    }
}

// ADAS 叠加渲染：HUD / 箭头 / 报警横幅 / 报警边框 / LED 指示
// 在 draw_detection_overlay 之后调用（叠加优先级最高）
void MainWindow::draw_adas_overlay(QPixmap &pixmap)
{
    if (pixmap.isNull()) return;

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const int W = pixmap.width();
    const int H = pixmap.height();

    // 2Hz 闪烁相位（报警闪烁；0 相位可见）
    const bool blink_on = (adas_blink_clock_.elapsed() / 500) % 2 == 0;
    const bool alarming = last_decision_.any_alarm();

    // ===== 1. HUD 面板（左上，半透明黑底白字）=====
    {
        QFont font = painter.font();
        font.setPointSize(13);
        font.setBold(false);
        painter.setFont(font);

        const int hud_w = 330;
        const int line_h = 24;
        const int hud_h = line_h * 6 + 16;
        QRect hud_rect(12, 12, hud_w, hud_h);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 0, 0, 140));
        painter.drawRoundedRect(hud_rect, 8, 8);

        int y = hud_rect.top() + 8;
        const int text_x = hud_rect.left() + 12;

        // 信号灯
        {
            QString light_text;
            QColor light_color(0x99, 0x99, 0x99);
            switch (last_decision_.light) {
                case adas::LightState::Red:
                    light_text = QStringLiteral("红灯"); light_color = QColor(255, 51, 51); break;
                case adas::LightState::Green:
                    light_text = QStringLiteral("绿灯"); light_color = QColor(51, 255, 51); break;
                case adas::LightState::Yellow:
                    light_text = QStringLiteral("黄灯"); light_color = QColor(255, 204, 0); break;
                case adas::LightState::Black:
                    light_text = QStringLiteral("黑灯"); break;
                default:
                    light_text = QStringLiteral("无信号"); break;
            }
            painter.setPen(QColor(0xcc, 0xcc, 0xcc));
            painter.drawText(QPoint(text_x, y), QStringLiteral("信号灯:"));
            painter.setPen(light_color);
            painter.drawText(QPoint(text_x + 80, y), light_text);
            y += line_h;
        }

        // 车道线
        {
            const auto &lane = last_perception_.lane;
            QString lane_text;
            if (lane.left_valid || lane.right_valid) {
                lane_text = QStringLiteral("左%1 右%2")
                    .arg(lane.left_valid ? QString::fromUtf8(adas::line_type_text(lane.left_type)) : QStringLiteral("--"))
                    .arg(lane.right_valid ? QString::fromUtf8(adas::line_type_text(lane.right_type)) : QStringLiteral("--"));
            } else {
                lane_text = QStringLiteral("未检出");
            }
            painter.setPen(QColor(0xcc, 0xcc, 0xcc));
            painter.drawText(QPoint(text_x, y), QStringLiteral("车道线:"));
            painter.setPen(QColor(0x66, 0xcc, 0xff));
            painter.drawText(QPoint(text_x + 80, y), lane_text);
            y += line_h;
        }

        // 车速
        {
            painter.setPen(QColor(0xcc, 0xcc, 0xcc));
            painter.drawText(QPoint(text_x, y), QStringLiteral("车速:"));
            const bool over = last_decision_.alarm_overspeed;
            painter.setPen(over ? QColor(255, 51, 51) : QColor(51, 255, 51));
            painter.drawText(QPoint(text_x + 80, y),
                QStringLiteral("%1 km/h (限%2)")
                    .arg(qRound(ego_.speed_kmh))
                    .arg(qRound(ego_.speed_limit)));
            y += line_h;
        }

        // 导航
        {
            painter.setPen(QColor(0xcc, 0xcc, 0xcc));
            painter.drawText(QPoint(text_x, y), QStringLiteral("导航:"));
            QString nav_text;
            if (manual_nav_cycle_ != 3) {
                nav_text = QStringLiteral("手动%1")
                    .arg(QString::fromUtf8(adas::arrow_dir_text(
                        static_cast<adas::ArrowDir>(manual_nav_cycle_ + 1))));
            } else if (last_perception_.nav.detected) {
                nav_text = QStringLiteral("识别%1")
                    .arg(QString::fromUtf8(adas::arrow_dir_text(
                        static_cast<adas::ArrowDir>(last_perception_.nav.dir))));
            } else {
                nav_text = QStringLiteral("自动（未检出）");
            }
            painter.setPen(QColor(0xff, 0xcc, 0x00));
            painter.drawText(QPoint(text_x + 80, y), nav_text);
            y += line_h;
        }

        // 决策状态
        {
            painter.setPen(QColor(0xcc, 0xcc, 0xcc));
            painter.drawText(QPoint(text_x, y), QStringLiteral("决策:"));
            painter.setPen(alarming ? QColor(255, 51, 51) : QColor(51, 255, 51));
            painter.drawText(QPoint(text_x + 80, y),
                QString::fromUtf8(last_decision_.status_text.c_str()));
            y += line_h;
        }

        // 推理耗时
        {
            painter.setPen(QColor(0x99, 0x99, 0x99));
            painter.drawText(QPoint(text_x, y),
                QStringLiteral("推理: %1 ms")
                    .arg(last_perception_.infer_ms, 0, 'f', 1));
        }
    }

    // ===== 2. 行驶行为箭头（底部中央；赛题：叠加左/右/直行箭头）=====
    if (last_decision_.arrow != adas::ArrowDir::None) {
        const QColor arrow_color = alarming ? QColor(255, 40, 40) : QColor(40, 230, 40);
        // 报警时 2Hz 闪烁；正常时常亮
        const bool arrow_visible = !alarming || blink_on;
        if (arrow_visible) {
            draw_arrow(painter, last_decision_.arrow,
                       QPoint(W / 2, H - 110), 150, arrow_color);
        }
    }

    // ===== 3. 报警渲染（赛题：图像叠加红色模拟报警）=====
    if (alarming) {
        // 紧急刹车：全屏红色半透明叠加（不闪烁，强调严重性）
        if (last_decision_.brake_now) {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(255, 0, 0, 70));
            painter.drawRect(pixmap.rect());
        }

        // 报警边框（2Hz 闪烁）
        if (blink_on) {
            QPen border_pen(QColor(255, 0, 0, 200), 14);
            painter.setPen(border_pen);
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(pixmap.rect());
        }

        // 顶部报警横幅（2Hz 闪烁）
        if (blink_on) {
            QFont banner_font = painter.font();
            banner_font.setPointSize(22);
            banner_font.setBold(true);
            painter.setFont(banner_font);

            const QString banner_text =
                QString::fromUtf8(last_decision_.status_text.c_str());
            const QRect banner_rect(0, 0, W, 64);
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(200, 0, 0, 220));
            painter.drawRect(banner_rect);
            painter.setPen(Qt::white);
            painter.drawText(banner_rect, Qt::AlignCenter,
                             QStringLiteral("⚠ ") + banner_text);
        }
    }

    // ===== 4. LED 指示（右下角；赛题：LED0左/LED1直/LED2右）=====
    {
        const int led_r = 20;
        const int gap = 64;
        const int led_y = H - 60;
        const int start_x = W - 3 * gap - 20;

        const char *led_names[3] = {u8"LED0 左", u8"LED1 直", u8"LED2 右"};
        QFont led_font = painter.font();
        led_font.setPointSize(11);
        painter.setFont(led_font);

        for (int i = 0; i < 3; ++i) {
            const int cx = start_x + i * gap + led_r;
            const bool lit = (last_decision_.led_mask & (1u << i)) != 0;

            // 圆形 LED：亮=报警红/正常绿，灭=暗灰
            QColor fill = lit ? (alarming ? QColor(255, 60, 60) : QColor(60, 255, 60))
                              : QColor(50, 50, 50, 160);
            painter.setPen(QPen(QColor(0, 0, 0, 120), 2));
            painter.setBrush(fill);
            painter.drawEllipse(QPoint(cx, led_y), led_r, led_r);

            // 标签
            painter.setPen(lit ? Qt::white : QColor(0x88, 0x88, 0x88));
            painter.drawText(QRect(cx - led_r - 8, led_y + led_r + 4,
                                   led_r * 2 + 16, 18),
                            Qt::AlignCenter, QString::fromUtf8(led_names[i]));
        }
    }
}

// 绘制行驶行为箭头（左/直行/右；实心多边形 + 半透明填充）
void MainWindow::draw_arrow(QPainter &painter, adas::ArrowDir dir,
                            const QPoint &center, int size, const QColor &color)
{
    const int cx = center.x();
    const int cy = center.y();
    const int half = size / 2;

    QPolygonF polygon;
    switch (dir) {
        case adas::ArrowDir::Straight: {
            // ↑ 箭头：三角头（尖朝上）+ 竖杆
            const int head_h = half * 2 / 3;      // 头高
            const int head_w = half * 2 / 3;      // 头半宽
            const int shaft_w = size / 10;        // 杆半宽
            polygon << QPointF(cx, cy - half)                 // 尖
                   << QPointF(cx + head_w, cy - half + head_h) // 头右底
                   << QPointF(cx + shaft_w, cy - half + head_h)
                   << QPointF(cx + shaft_w, cy + half)        // 杆右下
                   << QPointF(cx - shaft_w, cy + half)        // 杆左下
                   << QPointF(cx - shaft_w, cy - half + head_h)
                   << QPointF(cx - head_w, cy - half + head_h); // 头左底
            break;
        }
        case adas::ArrowDir::Left: {
            // ← 箭头：三角头（尖朝左）+ 横杆
            const int head_w = half * 2 / 3;
            const int head_h = half * 2 / 3;
            const int shaft_h = size / 10;
            polygon << QPointF(cx - half, cy)                 // 尖
                   << QPointF(cx - half + head_w, cy - head_h)
                   << QPointF(cx + half, cy - head_h)
                   << QPointF(cx + half, cy + shaft_h)        // 杆右下
                   << QPointF(cx - half + head_w, cy + shaft_h)
                   << QPointF(cx - half + head_w, cy + head_h);
            break;
        }
        case adas::ArrowDir::Right: {
            // → 箭头：← 的镜像
            const int head_w = half * 2 / 3;
            const int head_h = half * 2 / 3;
            const int shaft_h = size / 10;
            polygon << QPointF(cx + half, cy)                 // 尖
                   << QPointF(cx + half - head_w, cy - head_h)
                   << QPointF(cx - half, cy - head_h)
                   << QPointF(cx - half, cy + shaft_h)
                   << QPointF(cx + half - head_w, cy + shaft_h)
                   << QPointF(cx + half - head_w, cy + head_h);
            break;
        }
        default:
            return;
    }

    painter.setPen(QPen(QColor(255, 255, 255, 180), 3));
    painter.setBrush(QColor(color.red(), color.green(), color.blue(), 190));
    painter.drawPolygon(polygon);
}

// ============================================================
// 基础图像处理演示（评分项：灰度化/去噪/二值化/边缘/色彩空间转换）
// ============================================================

void MainWindow::on_cmbProc_currentIndexChanged(int index)
{
    processing_demo_index_ = index;
    if (index != 0) {
        statusBar()->showMessage(
            QStringLiteral("图像处理演示：%1").arg(ui->cmbProc->currentText()),
            3000);
    }
}

// 对当前帧应用所选处理算法，输出到 proc_image_（独立缓冲，不污染 img_buf）
void MainWindow::apply_processing_demo(int index)
{
    const uchar *frame_ptr = pango_device->img_buf + IMG_SIZE * index;

    // RGB565 帧的 OpenCV 视图（CV_8UC2，不拷贝）
    static cv::Mat src565;
    src565.create(length, width, CV_8UC2);
    std::memcpy(src565.data, frame_ptr, static_cast<size_t>(width) * length * 2);

    switch (processing_demo_index_) {
        case 1: {  // 灰度化
            static cv::Mat gray;
            cv::cvtColor(src565, gray, cv::COLOR_BGR5652GRAY);
            proc_image_ = QImage(gray.data, width, length, width,
                                 QImage::Format_Grayscale8);
            break;
        }
        case 2: {  // 中值去噪（3×3）
            static cv::Mat bgr, out565;
            cv::cvtColor(src565, bgr, cv::COLOR_BGR5652BGR);
            cv::medianBlur(bgr, bgr, 3);
            cv::cvtColor(bgr, out565, cv::COLOR_BGR2BGR565);
            proc_image_ = QImage(out565.data, width, length, width * 2,
                                 QImage::Format_RGB16);
            break;
        }
        case 3: {  // 二值化（大津自适应阈值）
            static cv::Mat gray, binary;
            cv::cvtColor(src565, gray, cv::COLOR_BGR5652GRAY);
            cv::threshold(gray, binary, 0, 255,
                           cv::THRESH_BINARY | cv::THRESH_OTSU);
            proc_image_ = QImage(binary.data, width, length, width,
                                 QImage::Format_Grayscale8);
            break;
        }
        case 4: {  // Sobel 边缘
            static cv::Mat gray, sobel;
            cv::cvtColor(src565, gray, cv::COLOR_BGR5652GRAY);
            cv::Sobel(gray, sobel, CV_16S, 1, 0, 3);
            cv::convertScaleAbs(sobel, sobel);
            proc_image_ = QImage(sobel.data, width, length, width,
                                 QImage::Format_Grayscale8);
            break;
        }
        case 5: {  // HSV 色彩空间转换 + V 通道直方图均衡（色彩增强）
            static cv::Mat bgr, hsv, out565;
            std::vector<cv::Mat> channels;
            cv::cvtColor(src565, bgr, cv::COLOR_BGR5652BGR);
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
            cv::split(hsv, channels);
            cv::equalizeHist(channels[2], channels[2]);  // V 通道
            cv::merge(channels, hsv);
            cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
            cv::cvtColor(bgr, out565, cv::COLOR_BGR2BGR565);
            proc_image_ = QImage(out565.data, width, length, width * 2,
                                 QImage::Format_RGB16);
            break;
        }
        default: {  // 关闭：直接透传 RGB565
            proc_image_ = QImage(frame_ptr, width, length, width * 2,
                                 QImage::Format_RGB16);
            break;
        }
    }
}

// ============================================================
// 文件回放源（U 盘场景视频演示入口；与 PCIe 采集互斥）
// ============================================================

// PCIe 源选中：停止文件回放（不自动启动 PCIe，由"采集视频"按钮控制）
void MainWindow::on_radioPCIe_toggled(bool checked)
{
    if (!checked) return;
    stop_file_playback();
    statusBar()->showMessage(QStringLiteral("视频源：PCIe 采集（点击\"采集视频\"开始）"),
                             3000);
}

// 文件源选中：无文件时弹选择框；有文件则开始回放（自动停 PCIe）
void MainWindow::on_radioFile_toggled(bool checked)
{
    if (!checked) return;
    if (file_source_ == nullptr) return;

    if (!file_source_->is_valid()) {
        // 尚未选择文件：弹文件框
        on_btnOpenFile_clicked();
        // 用户取消选择 → 回退到 PCIe
        if (!file_source_->is_valid()) {
            QSignalBlocker b(ui->radioPCIe);
            ui->radioPCIe->setChecked(true);
        }
        return;
    }
    start_file_playback();
}

// 打开文件选择框（视频/图片）
void MainWindow::on_btnOpenFile_clicked()
{
    if (file_source_ == nullptr) return;

    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("选择场景视频/图片"),
        QString(),
        QStringLiteral(
            "视频/图片 (*.mp4 *.avi *.mkv *.mov *.flv *.ts *.h264 *.png *.jpg *.jpeg *.bmp);;所有文件 (*)"));
    if (path.isEmpty()) return;

    if (file_source_->open(path)) {
        ui->lblFileName->setText(QFileInfo(path).fileName());
        ui->btnPlayPause->setEnabled(true);
        ui->btnPlayPause->setText(QStringLiteral("暂停"));
        // 切换到文件源（触发 start_file_playback → 停 PCIe）
        ui->radioFile->setChecked(true);
        statusBar()->showMessage(
            QStringLiteral("文件回放：%1").arg(QFileInfo(path).fileName()), 3000);
    }
}

// 播放/暂停切换（P 键）
void MainWindow::on_btnPlayPause_clicked()
{
    if (file_source_ == nullptr || !file_source_->is_valid()) return;

    if (!file_source_->isRunning()) {
        // 非循环模式播放结束：重新打开（从头播放）
        file_source_->open(file_source_->path());
        ui->btnPlayPause->setText(QStringLiteral("暂停"));
        return;
    }

    const bool paused = !file_source_->is_paused();
    file_source_->set_paused(paused);
    ui->btnPlayPause->setText(paused ? QStringLiteral("继续")
                                     : QStringLiteral("暂停"));
}

// 循环播放开关
void MainWindow::on_chkLoop_toggled(bool checked)
{
    if (file_source_ != nullptr) {
        file_source_->set_loop(checked);
    }
}

// 文件播放自然结束（非循环模式）
void MainWindow::handle_file_playback_finished()
{
    ui->btnPlayPause->setText(QStringLiteral("重播"));
    statusBar()->showMessage(QStringLiteral("文件播放结束"), 3000);
}

// 文件源错误上报
void MainWindow::handle_file_source_error(const QString &message)
{
    statusBar()->showMessage(QStringLiteral("⚠ 文件回放：%1").arg(message), 5000);
    ui->btnPlayPause->setEnabled(false);
}

// 启动文件回放（自动停 PCIe 采集）
void MainWindow::start_file_playback()
{
    if (file_source_ == nullptr || !file_source_->is_valid()) return;

    // 互斥：停止 PCIe 采集
    stop_pcie_capture();

    // 非循环模式播放已结束（线程退出）：重新打开从头播放
    if (!file_source_->isRunning()) {
        file_source_->open(file_source_->path());
    } else {
        file_source_->set_paused(false);
    }

    file_source_active_ = true;
    ui->btnPlayPause->setEnabled(true);
    ui->btnPlayPause->setText(QStringLiteral("暂停"));

    // 启动帧率定时器（FPS 统计对文件源同样有效）
    fps_timer_->start();
}

// 停止文件回放（暂停线程，保留文件句柄，切回时可继续）
void MainWindow::stop_file_playback()
{
    if (file_source_ != nullptr && file_source_->isRunning()) {
        file_source_->set_paused(true);
    }
    file_source_active_ = false;
}

// ============================================================
// 键盘快捷键（赛题"按键模拟"要求；板上 USB 键盘操作入口）
// ============================================================

void MainWindow::handle_hotkey_lane_left()
{
    ui->btnLaneL->setChecked(true);
    on_btnLaneL_clicked();
}

void MainWindow::handle_hotkey_lane_center()
{
    ui->btnLaneC->setChecked(true);
    on_btnLaneC_clicked();
}

void MainWindow::handle_hotkey_lane_right()
{
    ui->btnLaneR->setChecked(true);
    on_btnLaneR_clicked();
}

void MainWindow::handle_hotkey_drive_toggle()
{
    ui->btnDrive->setChecked(!ego_.driving);
    // on_btnDrive_toggled 由 setChecked 触发
}

void MainWindow::handle_hotkey_signal_left()
{
    // A 键：再按同方向取消
    ui->btnSigL->setChecked(ego_.signal != adas::TurnSignal::Left);
}

void MainWindow::handle_hotkey_signal_right()
{
    ui->btnSigR->setChecked(ego_.signal != adas::TurnSignal::Right);
}

void MainWindow::handle_hotkey_force_lane_left()
{
    ui->btnForceLaneL->setChecked(ego_.force_lane_change != adas::TurnSignal::Left);
}

void MainWindow::handle_hotkey_force_lane_right()
{
    ui->btnForceLaneR->setChecked(ego_.force_lane_change != adas::TurnSignal::Right);
}

void MainWindow::handle_hotkey_nav_next()
{
    on_btnNavNext_clicked();
}

void MainWindow::handle_hotkey_pause()
{
    if (file_source_active_) {
        on_btnPlayPause_clicked();
    }
}

void MainWindow::handle_hotkey_reset()
{
    on_cam2_clicked();
}
