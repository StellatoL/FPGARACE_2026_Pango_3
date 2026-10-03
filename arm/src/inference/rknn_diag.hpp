#ifndef RKNN_DIAG_HPP
#define RKNN_DIAG_HPP

// 一次性输出 RKNN 模型的输入/输出张量属性（维度、形状、格式）。
//
// 目的：车牌检测路径对检测头做 HWC→CHW 转置，而交通检测路径直接把原始输出
// 喂给同一个后处理器——两者能同时正确，仅当两个模型的输出内存布局本就不同：
//   - 5D 输出 dims=[1, na, H, W, prop]（permuted，[anchor][row][col][prop]）→ 需要转置
//   - 4D 输出 dims=[1, na*prop, H, W]（native NCHW，[anchor][prop][row][col]）→ 无需转置
// 判据是 n_dims（5D vs 4D），而非 fmt。此日志把“到底是哪种布局”这一唯一未知项
// 在真机上一次性变为已知，便于判断转置逻辑是否与模型匹配。
//
// 只读查询，不改变任何推理行为，也不触碰 FPGA/PCIe 接口。

#include <QDebug>
#include <QString>

#include "ztu/rk/ez_rknn.hpp"

namespace vision {

// 把单个张量属性格式化为 "dims=[1,3,80,80,6] n_dims=5 fmt=..." 形式
inline QString format_rknn_attr(const rknn_tensor_attr &attr) {
  QString dims;
  for (uint32_t i = 0; i < attr.n_dims; ++i) {
    if (i != 0) {
      dims += QLatin1Char(',');
    }
    dims += QString::number(attr.dims[i]);
  }
  return QStringLiteral("[%1] n_dims=%2 dims=[%3] fmt=%4")
      .arg(attr.index)
      .arg(attr.n_dims)
      .arg(dims)
      .arg(static_cast<int>(attr.fmt));
}

// 输出某个模型的全部输入/输出张量属性。model_label 用于区分是哪个模型。
inline void log_rknn_output_layout(const char *model_label,
                                   const ztu::rk::EzRknn &engine) {
  qDebug().nospace() << "[RKNN-IO] " << model_label << " inputs="
                     << engine.input_attrs.size()
                     << " outputs=" << engine.output_attrs.size();
  for (const auto &attr : engine.input_attrs) {
    qDebug().nospace() << "[RKNN-IO] " << model_label
                       << " IN  " << format_rknn_attr(attr);
  }
  for (const auto &attr : engine.output_attrs) {
    qDebug().nospace() << "[RKNN-IO] " << model_label
                       << " OUT " << format_rknn_attr(attr);
  }
}

} // namespace vision

#endif // RKNN_DIAG_HPP
