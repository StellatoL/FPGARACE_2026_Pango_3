#include "ui/mainwindow.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QTimer>
#include <cmath>
#include <opencv2/core.hpp>

int main(int argc,char *argv[]) {
  QApplication app(argc,argv);
  cv::setNumThreads(1); // Bound optional diagnostic OpenCV work.
  app.setApplicationName(QStringLiteral("Pango ADAS"));
  QCommandLineParser parser;
  parser.setApplicationDescription(QStringLiteral("FPGA + RK3568 ADAS · 摄像头 / HDMI 经 FPGA 输入"));
  parser.addHelpOption();
  parser.addOption({"single-view",QStringLiteral("初始布局为单路前视；FPGA实际源确认后自动同步")});
  parser.addOption({"mode",QStringLiteral("adas / raw"),"mode","adas"});
  parser.addOption({"quit-after",QStringLiteral("测试运行秒数，到时正常关闭"),"seconds"});
  parser.addOption({"report",QStringLiteral("退出时保存性能/能力 JSON"),"path"});
  parser.addOption({"screenshot",QStringLiteral("测试结束时保存界面截图"),"path"});
  parser.addOption({"window-size",QStringLiteral("测试窗口尺寸，例如 1024x600"),"WxH"});
  parser.process(app);
  const QString mode=parser.value("mode");
  if(!QStringList{"adas","raw"}.contains(mode))parser.showHelp(2);
  bool ok=true;double duration=0;
  if(parser.isSet("quit-after")) {
    duration=parser.value("quit-after").toDouble(&ok);
    if(!ok||!std::isfinite(duration)||duration<=0||duration>86400)parser.showHelp(2);
  }
  QDir root(QCoreApplication::applicationDirPath());if(!root.exists("resources"))root.cdUp();
  QFile style(root.filePath("resources/style.qss"));
  if(style.open(QFile::ReadOnly))app.setStyleSheet(QString::fromUtf8(style.readAll()));
  MainWindow window;
  if(parser.isSet("single-view"))window.set_single_view(true);
  window.set_detection_mode(mode=="raw"?DetectionMode::RawVideo:DetectionMode::Adas);
  if(parser.isSet("window-size")) {
    auto dimensions=parser.value("window-size").split('x');
    if(dimensions.size()!=2||dimensions[0].toInt()<900||dimensions[1].toInt()<540)parser.showHelp(2);
    window.resize(dimensions[0].toInt(),dimensions[1].toInt());
  }
  window.show();
  QObject::connect(&app,&QCoreApplication::aboutToQuit,&window,[&](){window.write_adas_report(parser.value("report"));});
  if(duration>0)QTimer::singleShot(int(duration*1000),&window,[&](){window.save_adas_screenshot(parser.value("screenshot"));window.close();});
  return app.exec();
}
