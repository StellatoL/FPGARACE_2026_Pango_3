#include <QObject>
#include "common/frame_packet.h"
#include <cassert>
#include <iostream>

int main() {
  adas::FramePool pool(2, 1280, 720);
  std::vector<unsigned char> dma(1280 * 720 * 2, 0x23);
  adas::Stamp stamp{1, 1, 1, 1, false};
  auto first = pool.copy(dma.data(), 1280, 720, 2560, stamp, adas::Layout::Mosaic);
  assert(first && first.pixels->bytes[0] == 0x23);
  dma[0] = 0x99;
  auto second = pool.copy(dma.data(), 1280, 720, 2560, stamp, adas::Layout::Mosaic);
  assert(second && first.pixels->bytes[0] == 0x23);
  assert(!pool.copy(dma.data(), 1280, 720, 2560, stamp, adas::Layout::Mosaic));
  second = {};
  assert(pool.copy(dma.data(), 1280, 720, 2560, stamp, adas::Layout::Mosaic));
  auto right = adas::view_rect(adas::Layout::Mosaic, adas::ViewId::Right, 1280, 720);
  assert(right.x == 640 && right.y == 0 && right.w == 640 && right.h == 360);
  auto left = adas::view_rect(adas::Layout::Mosaic, adas::ViewId::Left, 1280, 720);
  assert(left.x == 640 && left.y == 360);
  adas::ViewScheduler schedule;
  int counts[4] = {};
  for (int i = 0; i < 70; ++i) ++counts[static_cast<int>(schedule.next(adas::Layout::Mosaic))];
  assert(counts[0] == 40 && counts[1] == 10 && counts[2] == 10 && counts[3] == 10);
  adas::LatestFrameMailbox mailbox;
  mailbox.publish(first); mailbox.publish(first);
  assert(mailbox.overwritten() == 1 && mailbox.take());
  assert(!mailbox.take());
  assert(!pool.copy(nullptr, 1280, 720, 2560, stamp, adas::Layout::Mosaic));
  assert(!pool.copy(dma.data(), 1280, 720, 1, stamp, adas::Layout::Mosaic));
  std::cout << "frame ownership, bounded pool, crop and scheduling: PASS\n";
}
