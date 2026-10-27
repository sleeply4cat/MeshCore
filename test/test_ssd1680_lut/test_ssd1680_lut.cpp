#include <gtest/gtest.h>

#include <helpers/ui/SSD1680Lut.h>

#include <random>
#include <vector>

using namespace SSD1680Lut;

// GxEPD2_213_B74_Fast's defaults
static const int KICK = 0, FRAMES = 8, CLEAR_FRAMES = 16, CLEAR_CYCLES = 3;

static int transition(bool old_black, bool new_black) { return (old_black ? 0 : 2) + (new_black ? 0 : 1); }

TEST(SSD1680Lut, FastCycleBalancesAndLeavesSteadyPixelsAlone) {
  uint8_t lut[LEN];
  for (int kick = 0; kick <= 2; kick++) {
    fastLut(lut, kick, FRAMES);
    EXPECT_EQ(FRAMES - kick, charge(lut, W_B));
    EXPECT_EQ(0, charge(lut, W_B) + charge(lut, B_W));
    EXPECT_EQ(0, charge(lut, B_B));
    EXPECT_EQ(0, charge(lut, W_W));
  }
}

TEST(SSD1680Lut, ClearUndoesTheFastChargeOfBlackPixelsOnly) {
  uint8_t fast[LEN], clear[LEN];
  fastLut(fast, KICK, FRAMES);
  clearLut(clear, CLEAR_FRAMES, CLEAR_CYCLES, FRAMES - KICK);
  EXPECT_EQ(-charge(fast, W_B), charge(clear, B_W));
  EXPECT_EQ(0, charge(clear, W_W));
  clearLut(clear, CLEAR_FRAMES, CLEAR_CYCLES, 0);
  for (int t = 0; t < 4; t++) EXPECT_EQ(0, charge(clear, t));
}

// UI pages (the panel's OTP waveform, taken as balanced on its own: +X to black, -X to white) -> spectrum entry
// -> fast frames and span changes -> exit, as GxEPD2_213_B74_Fast::clearFast() does it: no charge left at the end
TEST(SSD1680Lut, SessionsLeaveNoCharge) {
  const int N = 2000, OTP_X = 40;
  std::vector<int> q(N, 0);
  std::vector<bool> black(N, false), next(N), white(N, false);
  std::mt19937 rng(1);
  uint8_t lut[LEN];
  auto lutUpdate = [&](const std::vector<bool>& to) {
    for (int i = 0; i < N; i++) { q[i] += charge(lut, transition(black[i], to[i])); black[i] = to[i]; }
  };
  auto otpUpdate = [&](const std::vector<bool>& to) {
    for (int i = 0; i < N; i++) {
      if (!black[i] && to[i]) q[i] += OTP_X;
      if (black[i] && !to[i]) q[i] -= OTP_X;
      black[i] = to[i];
    }
  };
  auto randomFrame = [&](int pct) { for (int i = 0; i < N; i++) next[i] = (int)(rng() % 100) < pct; };
  auto clearFast = [&](bool screen_fast) {
    if (!screen_fast) otpUpdate(white);
    clearLut(lut, CLEAR_FRAMES, CLEAR_CYCLES, FRAMES - KICK);
    lutUpdate(white);
  };
  for (int session = 0; session < 10; session++) {
    for (int k = 0; k < 5; k++) { randomFrame(15); otpUpdate(next); }
    clearFast(false);   // entry
    for (int span = 0; span < 3; span++) {
      fastLut(lut, KICK, FRAMES);
      for (int k = 0; k < 100; k++) { randomFrame(30); lutUpdate(next); }
      clearFast(true);   // span change, or exit
    }
  }
  otpUpdate(white);
  for (int i = 0; i < N; i++) ASSERT_EQ(0, q[i]) << "pixel " << i;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
