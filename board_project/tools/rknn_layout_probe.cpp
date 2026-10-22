// 独立诊断工具：直接加载 .rknn 模型并打印其输入/输出张量属性。
//
// 目的：确认三个模型检测头的输出内存布局，从而验证 plate/traffic 两条路径
// 的转置逻辑是否与模型匹配：
//   - 5D 输出 dims=[1, na, H, W, prop]（permuted）→ plate 路径的 HWC→CHW 转置是必需的
//   - 4D 输出 dims=[1, na*prop, H, W]（native NCHW）→ traffic 路径直接喂后处理器才正确
//
// 该工具不依赖 Qt / OpenCV / FPGA / PCIe，只依赖 RKNN 运行时（librknnrt.so）。
// 因此可以在没有采集板卡、没有图形界面的环境下单独运行。
//
// 编译（在仓库根目录）：
//   g++ -std=c++17 -I third_party -I third_party/rknn/include \
//       tools/rknn_layout_probe.cpp -o /tmp/rknn_layout_probe \
//       -L third_party/rknn/lib -lrknnrt -ldl
//
// 运行：
//   LD_LIBRARY_PATH=third_party/rknn/lib /tmp/rknn_layout_probe \
//       models/plate_yolov5.rknn models/plate_lprnet.rknn models/traffic_yolov5.rknn
//
// 若不带参数，默认探测 models/ 下的三个标准模型。

#include <cstdio>
#include <string>
#include <vector>

#include "ztu/rk/ez_rknn.hpp"

namespace {

// 打印单个模型的全部输入/输出张量属性。
void probe_model(const std::string &label, const std::string &path) {
  std::printf("\n==================== %s ====================\n", label.c_str());
  std::printf("path: %s\n", path.c_str());

  try {
    // 使用与主程序一致的 CHW2HWC 布局，确保观察到的 output_attrs 与运行期一致。
    ztu::rk::EzRknn engine(path, ztu::rk::EzRknn::Layout::CHW2HWC);

    std::printf("inputs = %zu, outputs = %zu\n", engine.input_attrs.size(),
                engine.output_attrs.size());

    auto print_attr = [](const char *tag, const rknn_tensor_attr &attr) {
      std::printf("  %s [%u] n_dims=%u dims=[", tag, attr.index, attr.n_dims);
      for (uint32_t i = 0; i < attr.n_dims; ++i) {
        std::printf("%s%u", (i ? "," : ""), attr.dims[i]);
      }
      std::printf("] n_elems=%u fmt=%d type=%d\n", attr.n_elems,
                  static_cast<int>(attr.fmt), static_cast<int>(attr.type));
    };

    for (const auto &attr : engine.input_attrs) {
      print_attr("IN ", attr);
    }
    for (const auto &attr : engine.output_attrs) {
      print_attr("OUT", attr);
    }

    // 针对检测模型给出直接结论。
    bool any_5d = false;
    bool any_4d = false;
    for (const auto &attr : engine.output_attrs) {
      if (attr.n_dims == 5) any_5d = true;
      if (attr.n_dims == 4) any_4d = true;
    }
    if (any_5d) {
      std::printf(
          "  => 检测头为 5D permuted 布局：需要 HWC->CHW 转置（plate 路径正确）\n");
    } else if (any_4d) {
      std::printf(
          "  => 检测头为 4D native NCHW 布局：无需转置（traffic 路径正确）\n");
    }
  } catch (const std::exception &ex) {
    std::printf("  ERROR: 加载失败: %s\n", ex.what());
  }
}

} // namespace

int main(int argc, char **argv) {
  std::vector<std::pair<std::string, std::string>> targets;

  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
      targets.emplace_back(std::string("model#") + std::to_string(i), argv[i]);
    }
  } else {
    targets = {
        {"plate_yolov5 (检测)", "models/plate_yolov5.rknn"},
        {"plate_lprnet (识别)", "models/plate_lprnet.rknn"},
        {"traffic_yolov5 (检测)", "models/traffic_yolov5.rknn"},
    };
  }

  std::printf("RKNN layout probe — 读取 output_attrs，不做推理\n");
  for (const auto &t : targets) {
    probe_model(t.first, t.second);
  }
  std::printf("\n完成。\n");
  return 0;
}
