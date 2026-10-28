#include <gtest/gtest.h>

#include <helpers/ui/SSD1680Lut.h>

#include <random>
#include <vector>

using namespace SSD1680Lut;

// GxEPD2_213_B74_Fast's defaults
static const int KICK = 0, FRAMES = 8;

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

// UI pages (the panel's own waveform) -> spectrum entry -> fast frames and span changes -> exit, as
// GxEPD2_213_B74_Fast::wipeWhite() and SpectrumView do it: the spectrum's waveforms leave no charge of their own
TEST(SSD1680Lut, SessionsLeaveNoCharge) {
  const int N = 2000;
  std::vector<int> q(N, 0);
  std::vector<bool> black(N, false), next(N), white(N, false);
  std::mt19937 rng(1);
  uint8_t fast[LEN];
  fastLut(fast, KICK, FRAMES);
  auto update = [&](const uint8_t* lut, const std::vector<bool>& to) {
    for (int i = 0; i < N; i++) { q[i] += charge(lut, transition(black[i], to[i])); black[i] = to[i]; }
  };
  auto otpUpdate = [&](const std::vector<bool>& to) { black = to; };   // the panel's own: not ours to count
  auto randomFrame = [&](int pct) { for (int i = 0; i < N; i++) next[i] = (int)(rng() % 100) < pct; };
  for (int session = 0; session < 10; session++) {
    for (int k = 0; k < 5; k++) { randomFrame(15); otpUpdate(next); }
    otpUpdate(white);   // entry: the slow path's picture to white with the panel's own full refresh
    for (int span = 0; span < 3; span++) {
      for (int k = 0; k < 100; k++) { randomFrame(30); update(fast, next); }
      update(fast, white);   // span change, or exit: the fast picture to white with the fast waveform
    }
    otpUpdate(white);   // exit: the panel's own full refresh, white to white
  }
  for (int i = 0; i < N; i++) ASSERT_EQ(0, q[i]) << "pixel " << i;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
