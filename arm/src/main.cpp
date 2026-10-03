// 主入口：创建 Qt 应用程序并显示主窗口
// PCIe 视频传输上位机 — FPGA→ARM 车牌识别 / 交通场景检测 / ADAS 辅助驾驶 / 纯视频
//
// 命令行：
//   img_recv_demo                      # 正常启动
//   img_recv_demo --adas <视频/图片>    # 启动即进入 ADAS 模式并回放指定文件
//                                      # （U 盘场景视频演示快捷入口）

#include "ui/mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QString>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setApplicationName(QStringLiteral("Pango PCIe Video"));

    // 加载暗色主题样式表
    QFile style_file(QStringLiteral("resources/style.qss"));
    if (style_file.open(QFile::ReadOnly)) {
        a.setStyleSheet(QString::fromUtf8(style_file.readAll()));
        style_file.close();
    }

    MainWindow w;
    w.show();

    // --adas <file>：启动即进入 ADAS 模式并回放指定场景视频
    if (argc >= 3 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--adas")) {
        const QString path = QString::fromLocal8Bit(argv[2]);
        w.startup_adas_with_file(path);
    }

    return a.exec();
}
