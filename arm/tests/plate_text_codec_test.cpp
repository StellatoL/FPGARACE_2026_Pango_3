#include "common/plate_text_codec.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect_eq(const std::string &actual, const std::string &expected,
               const char *name) {
  if (actual == expected) return true;
  std::cerr << name << " failed\nexpected: " << expected
            << "\nactual:   " << actual << '\n';
  return false;
}

} // namespace

int main() {
  bool ok = true;

  // 78-class charset: 0=#(blank), 1=京, ..., 42=0, 43=1, ..., 52=A, 53=B, ...
  ok &= expect_eq(
      lpr::decode_plate_indices(
          {1, 1, 0, 43, 43, 0, 52, 52, 53}),
      u8"京1AB", "deduplicates repeats and blanks");

  // L=62, M=63 (24 letters, no I/O)
  ok &= expect_eq(
      lpr::decode_plate_indices({0, 0, 62, 62, 63}),
      "LM", "ignores leading blanks");

  // Q=66, P=65 — repeated Q separated by blank should be kept
  ok &= expect_eq(
      lpr::decode_plate_indices({66, 0, 66, 65, 0}),
      "QQP", "keeps repeated chars when separated by blanks");

  ok &= expect_eq(lpr::normalize_plate_text(u8"皖AD10005"), u8"皖AD10005",
                  "does not trim green plate length");

  ok &= expect_eq(lpr::normalize_plate_text(u8"皖AFL605"), u8"皖AFL605",
                  "keeps valid 7-char plate text");

  return ok ? 0 : 1;
}
