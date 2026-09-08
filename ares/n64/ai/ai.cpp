#include <n64/n64.hpp>

namespace ares::Nintendo64 {
auto lumiverseSpTraceOn() -> bool;  //rdp/io.cpp (round 16): LUMIVERSE_ARES_N64_SP_TRACE
auto lumiverseSpTrace(const char* who, const char* op, const char* reg, u32 value, u64 extra) -> void;

AI ai;

//Lumiverse round 19 diagnostics (the Master Quest seam-click investigation):
//LUMIVERSE_ARES_N64_AI_TRACE=<path> logs every AI DMA event (enqueue, a
//buffer starting to play, the queue running dry) and — from the RSP side —
//task dispatch / completion / yield events on ONE timebase: the DAC output
//sample index (lumiverseAiSampleIndex, what the app-side WAV is made of),
//the RSP's monotonic cycle counter and the CPU Count. A click found in the
//DAC stream can then be placed against the buffer that was playing and the
//tasks in flight. LUMIVERSE_ARES_N64_AI_DAC_DUMP=<path> writes the raw DAC
//output (s16le stereo at the DAC rate, before the app's resampler).
u64 lumiverseAiSampleIndex = 0;
auto lumiverseAiTrace(const char* what, u32 a, u32 b, u32 c) -> void {
  static FILE* file = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_AI_TRACE");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  if(!file) return;
  fprintf(file, "%s idx=%llu rspc=%llu cc=%llu a=%06x b=%u c=%u\n", what,
    (unsigned long long)lumiverseAiSampleIndex, (unsigned long long)rsp.profile.cycles,
    (unsigned long long)(u64)cpu.scc.count, a, b, c);
}
auto lumiverseAiTraceOn() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AI_TRACE"); return v && v[0]; }();
  return value;
}
#include "io.cpp"
#include "debugger.cpp"
#include "serialization.cpp"

auto AI::load(Node::Object parent) -> void {
  node = parent->append<Node::Object>("AI");

  stream = node->append<Node::Audio::Stream>("AI");
  stream->setChannels(2);
  stream->setFrequency(44100.0);

  debugger.load(node);
}

auto AI::unload() -> void {
  debugger = {};
  node->remove(stream);
  stream.reset();
  node.reset();
}

auto AI::main() -> void {
  //Lumiverse diagnostic (LUMIVERSE_ARES_N64_TIMING_DIAG=1): DAC sample count
  //against the CPU Count register (pairs with the [vi] line in vi/vi.cpp)
  static const bool timingDiag = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_TIMING_DIAG");
    return value && *value == '1';
  }();
  static FILE* dacDump = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_AI_DAC_DUMP");
    return path && path[0] ? fopen(path, "wb") : nullptr;
  }();
  while(Thread::clock < 0) {
    sample();
    stream->frame(dac.left, dac.right);
    lumiverseAiSampleIndex++;
    if(dacDump) {
      const s16 pair[2] = { (s16)(dac.left * 32767.0), (s16)(dac.right * 32767.0) };
      fwrite(pair, 2, 2, dacDump);
    }
    step(dac.period);
    if(timingDiag) {
      static u64 samples = 0;
      if((++samples % 160000) == 0) {
        fprintf(stderr, "[ai] samples=%llu count=%llu dacFrequency=%u period=%u\n",
          (unsigned long long)samples, (unsigned long long)(u64)cpu.scc.count, dac.frequency, dac.period);
      }
    }
  }
}

auto AI::sample() -> void {
  bool active = false;

  if(io.dmaCount && io.dmaLength[0] && io.dmaEnable) {
    io.dmaAddress[0].bit(13,23) += io.dmaAddressCarry;
    auto data = rdram.ram.read<Word>(io.dmaAddress[0], RBusDevice::AI_DMA);
    dac.left  = (s16)(data >> 16) / 32768.0;
    dac.right = (s16)(data >>  0) / 32768.0;

    io.dmaAddress[0].bit(0,12) += 4;
    io.dmaAddressCarry = io.dmaAddress[0].bit(0,12) == 0;
    io.dmaLength[0] -= 4;
    active = true;
  }

  if(io.dmaCount && io.dmaLength[0] == 0) {
    if(--io.dmaCount) {
      io.dmaAddress[0]  = io.dmaAddress[1];
      io.dmaLength[0]   = io.dmaLength[1];
      io.dmaOriginPc[0] = io.dmaOriginPc[1];
      mi.raise(MI::IRQ::AI);
      if(unlikely(lumiverseAiTraceOn())) lumiverseAiTrace("play", io.dmaAddress[0], io.dmaLength[0], io.dmaCount);
    } else {
      if(unlikely(lumiverseAiTraceOn())) lumiverseAiTrace("dry", 0, 0, 0);
    }
  }

  if(!active) {
    dac.left  *= dac.decayFactor;
    dac.right *= dac.decayFactor;
    if(fabs(dac.left)  < 1e-7) dac.left  = 0.0;
    if(fabs(dac.right) < 1e-7) dac.right = 0.0;
  }
}

auto AI::updateDecay() -> void {
  dac.decayFactor = exp(-1.0 / (dac.frequency * 0.003));
}

auto AI::power(bool reset) -> void {
  Thread::reset();
  io = {};
  dac.left  = 0.0;
  dac.right = 0.0;
  dac.frequency = 44100;
  dac.precision = 16;
  dac.period = system.frequency() / dac.frequency;
  updateDecay();
}

}
