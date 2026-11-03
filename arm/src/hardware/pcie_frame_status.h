#ifndef PCIE_FRAME_STATUS_H
#define PCIE_FRAME_STATUS_H

inline bool pango_frame_status_ready(int status) {
  return status > 0 && ((status & 0x01) == 0x01);
}

inline int pango_frame_status_index(int status) {
  return (status >> 1) & 0x03;
}

#endif // PCIE_FRAME_STATUS_H
