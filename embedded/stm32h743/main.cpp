// Bring-up / timing image for the whole-body controller on the STM32H743.
//
// Initialises the controller from the generated default parameters, then runs
// it on the home pose (with a small, changing disturbance so the QP does real
// work) and records the cycle count of every step with the DWT counter.
// Results are in g_bench; read them with a debugger (e.g. `p g_bench` in gdb
// through the Nucleo's ST-LINK). No HAL: the core runs on the reset clock
// (64 MHz HSI) until PLL setup is added; scale cycles accordingly.
#include <cmath>
#include <cstdint>

#include "robot_model_select.h"
#include "wbc_core.h"
#include "wbc_default_params.h"

namespace {

#define REG(addr) (*reinterpret_cast<volatile uint32_t*>(addr))

void enable_caches() {
  constexpr uint32_t SCB_CCR = 0xE000ED14, SCB_ICIALLU = 0xE000EF50, SCB_CSSELR = 0xE000ED84,
                     SCB_CCSIDR = 0xE000ED80, SCB_DCISW = 0xE000EF60;
  // I-cache: invalidate, enable
  asm volatile("dsb\n isb");
  REG(SCB_ICIALLU) = 0;
  asm volatile("dsb\n isb");
  REG(SCB_CCR) |= (1u << 17);
  asm volatile("dsb\n isb");
  // D-cache: invalidate by set/way, enable
  REG(SCB_CSSELR) = 0;
  asm volatile("dsb");
  const uint32_t ccsidr = REG(SCB_CCSIDR);
  uint32_t sets = (ccsidr >> 13) & 0x7FFF;
  do {
    uint32_t ways = (ccsidr >> 3) & 0x3FF;
    do {
      REG(SCB_DCISW) = ((sets << 5) & (0x1FFu << 5)) | ((ways << 30) & (3u << 30));
    } while (ways-- != 0);
  } while (sets-- != 0);
  asm volatile("dsb");
  REG(SCB_CCR) |= (1u << 16);
  asm volatile("dsb\n isb");
}

void enable_cycle_counter() {
  REG(0xE000EDFC) |= (1u << 24);      // DEMCR.TRCENA
  REG(0xE0001FB0) = 0xC5ACCE55;       // DWT lock access (Cortex-M7)
  REG(0xE0001004) = 0;                // CYCCNT
  REG(0xE0001000) |= 1u;              // CYCCNTENA
}

inline uint32_t cycles() { return REG(0xE0001004); }

wbc::WbcCore g_core;   // ~230 kB: AXI SRAM (.bss)

}  // namespace

// Debugger-visible results
struct Bench {
  int init_rc;
  uint32_t arena_used;
  uint32_t steps;
  uint32_t cyc_min, cyc_max;
  uint64_t cyc_sum;
  uint32_t last_iters;
  uint32_t qp_failures;
  double last_tau[wbc::model::NU];
};
extern "C" {
volatile Bench g_bench;
}

int main() {
  enable_caches();
  enable_cycle_counter();

  g_bench.init_rc = g_core.init(WBC_DEFAULT_PARAMS);
  g_bench.arena_used = uint32_t(g_core.arena_used());
  if (g_bench.init_rc != wbc::WBC_OK) {
    for (;;) {
    }
  }
  g_bench.cyc_min = 0xFFFFFFFFu;

  double qpos[wbc::model::NQ], qvel[wbc::model::NV] = {0};
  double tau[wbc::model::NU], info[wbc::WbcCore::INFO_SIZE];
  for (int i = 0; i < wbc::model::NQ; ++i) qpos[i] = wbc::model::HOME_QPOS[i];
  qpos[2] -= 0.08;   // home keyframe is on the 8 cm platform; put the wheels on z = 0

  const double dt = 0.002;
  for (uint32_t k = 0;; ++k) {
    // Slowly varying disturbance: base pitch rate and wheel speeds
    const double t = k * dt;
    qvel[3] = 0.3 * std::sin(2.0 * t);
    qvel[9] = qvel[13] = 2.0 * std::sin(0.7 * t);
    const uint32_t c0 = cycles();
    const int rc = g_core.step(qpos, qvel, 0.3, 0.0, 0.0, 0.0, dt, tau, info);
    const uint32_t c = cycles() - c0;
    g_bench.steps = k + 1;
    if (c < g_bench.cyc_min) g_bench.cyc_min = c;
    if (c > g_bench.cyc_max) g_bench.cyc_max = c;
    g_bench.cyc_sum += c;
    g_bench.last_iters = uint32_t(info[WBC_INFO_QP_ITER]);
    if (rc != 0) g_bench.qp_failures++;
    for (int i = 0; i < wbc::model::NU; ++i) g_bench.last_tau[i] = tau[i];
  }
}
