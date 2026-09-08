#include <n64/n64.hpp>

//Lumiverse addition: threading primitives for lumiverse-async-audio.cpp
//(included below inside the namespace, where #includes are not possible)
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#if defined(__APPLE__)
  #include <pthread/qos.h>
#endif

namespace ares::Nintendo64 {

RSP rsp;
#include "decoder.cpp"
//Lumiverse diagnostic hook (defined in lumiverse-hle-audio.cpp): records the
//LLE microcode's DMEM->RDRAM DMA writes while an audio-HLE shadow compare is
//outstanding, so writes the HLE never issues become visible
namespace {
  auto lumiverseAudioNoteLLEWrite(u32 dramAddress, u32 length, u32 dmemAddress) -> void;
  //reverse-shadow mode (LUMIVERSE_ARES_N64_AUDIO_HLE=3): the HLE output drives
  //the game and the LLE microcode's buffer writes are diverted into a side
  //image for comparison; returns the side-image pointer for a diverted DMA
  //(nullptr = write RDRAM normally)
  auto lumiverseAudioDivertLLEWrite(u32 dramAddress, u32 length, u32 dmemAddress) -> u8*;
  auto lumiverseAudioFlushDeferredWrites() -> void;
  auto lumiverseAudioNoteTaskEnd(u64 cycles) -> void;
  auto lumiverseAudioProgressDeferredWrites(s64 elapsed, s64 total) -> void;
}
//Lumiverse diagnostic hooks defined in rdp/io.cpp (LUMIVERSE_ARES_N64_IO_POLL_LOG)
auto lumiverseIOPollLog() -> bool;
auto lumiverseIOPollNote(u32 slot, u32 value) -> void;
auto lumiverseSpTraceOn() -> bool;  //rdp/io.cpp (round 16): LUMIVERSE_ARES_N64_SP_TRACE
auto lumiverseSpTrace(const char* who, const char* op, const char* reg, u32 value, u64 extra) -> void;
#include "dma.cpp"
extern u64 lumiverseRdpTaskTag;  //rdp/io.cpp: graphics-task ordinal for dump alignment
#include "lumiverse-async-audio.cpp"
#include "lumiverse-hle.cpp"
#include "lumiverse-hle-gfx.cpp"
#include "lumiverse-hle-audio.cpp"
#include "io.cpp"
#include "interpreter.cpp"
#include "interpreter-ipu.cpp"
#include "interpreter-scc.cpp"
#include "interpreter-vpu.cpp"
#include "recompiler.cpp"
#include "debugger.cpp"
#include "serialization.cpp"
#include "disassembler.cpp"
#include "emux.cpp"

auto RSP::load(Node::Object parent) -> void {
  node = parent->append<Node::Object>("RSP");
  dmem.allocate(4_KiB);
  imem.allocate(4_KiB);
  debugger.load(node);
}

auto RSP::unload() -> void {
  debugger.unload();
  dmem.reset();
  imem.reset();
  node.reset();
}

auto RSP::main() -> void {
  while(Thread::clock < 0) {
    auto clock = Thread::clock;

    if(lumiverseHLEPendingCycles > 0) {
      //Lumiverse addition: a natively-executed task "runs" for its
      //requested duration, then completes exactly like BREAK would
      step(128);
      profile.cycles += 128;
      lumiverseHLEPendingCycles -= 128;
      //round 14: the audio HLE's deferred RDRAM output lands progressively over the modelled duration
      lumiverseAudioProgressDeferredWrites(lumiverseHLERequestedCompletionCycles - lumiverseHLEPendingCycles, lumiverseHLERequestedCompletionCycles);
      if(lumiverseHLEPendingCycles <= 0) {
        lumiverseHLEPendingCycles = 0;
        lumiverseAudioFlushDeferredWrites();  //round 14: the task's RDRAM output lands at completion
        status.halted = 1;
        status.broken = 1;
        status.signal[2] = 1;
        if(status.interruptOnBreak) mi.raise(MI::IRQ::SP);
      }
    } else if(status.halted) {
      //Lumiverse addition: settle a truncated-task audio-HLE shadow compare
      //(debug tool; the game typically crashes after the experiment, so the
      //normal settle-on-next-dispatch never runs)
      if(lumiverseAudioTruncateArmed && lumiverseAudioShadowState.pending) {
        lumiverseAudioTruncateArmed = false;
        lumiverseAudioShadowSettle();
      }
      step(128);
      profile.cycles += 128;
      profile.haltedCycles += 128;
    } else {
      instruction();
    }

    dmaStep(Thread::clock - clock);
  }
}

auto RSP::instruction() -> void {
  if(Accuracy::RSP::Recompiler && recompiler.enabled) {
    auto block = recompiler.block(ipu.pc);
    block->execute(*this);
  } else {
    pipeline.dblIssueCount = 0;
    u32 instruction = imem.read<Word>(ipu.pc);
    //Lumiverse diagnostic (LUMIVERSE_ARES_N64_RSP_DIAG=1): executed-PC
    //histogram of the interpreted RSP, printed every 2^24 instructions, to
    //see whether a resident microcode (Conker's engine never halts the RSP)
    //spends its time working or spinning in a wait loop
    static const int rspDiag = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_DIAG"); return v ? ::atoi(v) : 0; }();
    if(rspDiag) {
      static u32 histogram[1024];
      static u64 total = 0;
      histogram[ipu.pc >> 2 & 1023]++;
      if((++total & 0xffffff) == 0) {
        u32 top[12] = {};
        for(u32 n = 0; n < 12; n++) {
          u32 best = 0;
          for(u32 i = 0; i < 1024; i++) if(histogram[i] > histogram[best]) best = i;
          top[n] = best;
          fprintf(stderr, "%s%03x=%.1f%%", n ? " " : "[rsp-diag] top PCs: ", best << 2, 100.0 * histogram[best] / (f64)0x1000000);
          histogram[best] = 0;
        }
        fprintf(stderr, " (halted cycles so far %llu, exec cycles %llu)\n",
          (unsigned long long)profile.haltedCycles, (unsigned long long)profile.cycles);
        for(auto& h : histogram) h = 0;
      }
    }
    instructionPrologue(instruction);
    branch.begin();
    pipeline.begin();
    //Lumiverse addition (round 12): RSP_FAST_DECODE — OpInfo from the
    //per-IMEM-word cache instead of re-decoding every execution (the decode
    //was ~23% of the interpreted RSP's time in the round-10/12 profiles)
    const bool decodeCache = lumiverseFast.decodeCache;
    OpInfo op0 = decodeCache ? lumiverseDecode(ipu.pc, instruction) : decoderEXECUTE(instruction);
    pipeline.issue(op0);
    interpreterEXECUTE();

    if(!pipeline.singleIssue && !op0.branch()) {
      u32 instruction = imem.read<Word>(ipu.pc + 4);
      OpInfo op1 = decodeCache ? lumiverseDecode(ipu.pc + 4, instruction) : decoderEXECUTE(instruction);

      if(canDualIssue(op0, op1)) {
        pipeline.dblIssueCount = 1;
        instructionEpilogue<0>(0);
        instructionPrologue(instruction);
        branch.begin();
        pipeline.issue(op1);
        interpreterEXECUTE();
      }
    }

    pipeline.end();
    instructionEpilogue<0>(0);
  }

  //this handles all stepping for the interpreter
  //with the recompiler, it only steps for taken branch stalls
  step(pipeline.clocks);
  profile.cycles += pipeline.clocks;
  pipeline.clocksTotal += pipeline.clocks;
}

auto RSP::instructionPrologue(u32 instruction) -> void {
  pipeline.address = ipu.pc;
  pipeline.instruction = instruction;
  //Lumiverse addition (round 12): RSP_FAST_NOTRACE — the tracer call only
  //when armed (traceArmed is kept by the tracer toggle hook and XTRACESTART)
  if(!lumiverseFast.noTrace || unlikely(lumiverseFast.traceArmed)) debugger.instruction();
}

//Lumiverse addition (round 12): read the RSP_FAST configuration (see rsp.hpp).
//Called from power(); the app sets its env knobs before the core is created.
auto RSP::lumiverseLoadFastConfig() -> void {
  auto& f = lumiverseFast;
  const bool traceArmed = f.traceArmed;
  f = {};
  f.traceArmed = traceArmed;
  const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_FAST");
  f.enabled = v && *v == '1';
  if(f.enabled) {
    const char* t = ::getenv("LUMIVERSE_ARES_N64_RSP_FAST_NOTRACE");
    f.noTrace = !t || *t != '0';
    const char* d = ::getenv("LUMIVERSE_ARES_N64_RSP_FAST_DECODE");
    f.decodeCache = !d || *d != '0';
  }
  //keep the invariant info == decoderEXECUTE(word) for every entry
  for(auto& e : f.decode) { e.word = 0; e.info = decoderEXECUTE(0); }
}

auto RSP::instructionBranchEpilogue() -> s32 {
  bool endBlock = branch.state & Branch::EndBlock;
  if(branch.inDelaySlot()) {
    pipeline.stall();
    if(branch.pc & 4) pipeline.singleIssue = 1;
  }

  branch.end();
  ipu.pc = branch.pc;
  return status.halted || endBlock;
}

template<bool Recompiled>
auto RSP::instructionEpilogue(u32 clocks) -> s32 {
  if constexpr(Recompiled) {
    step(clocks);
    profile.cycles += clocks;
    pipeline.clocksTotal += clocks;

    assert(ipu.r[0].u32 == 0);
  } else {
    ipu.r[0].u32 = 0;
  }

  return instructionBranchEpilogue();
}

auto RSP::power(bool reset) -> void {
  //Lumiverse addition: never reset the core out from under an in-flight
  //async audio task
  lumiverseAsyncDrain();
  Thread::reset();
  dmem.fill();
  imem.fill();

  pipeline = {};
  profile = {};
  dma = {};
  lumiverseLoadFastConfig();
  status.semaphore = 0;
  status.halted = 1;
  status.broken = 0;
  status.full = 0;
  status.singleStep = 0;
  status.interruptOnBreak = 0;
  for(auto& signal : status.signal) signal = 0;
  for(auto& r : ipu.r) r.u32 = 0;
  ipu.pc = 0;
  branch.setPc(ipu.pc);
  for(auto& r : vpu.r) r = zero;
  vpu.acch = zero;
  vpu.accm = zero;
  vpu.accl = zero;
  vpu.vcoh = zero;
  vpu.vcol = zero;
  vpu.vcch = zero;
  vpu.vccl = zero;
  vpu.vce = zero;
  vpu.divin = 0;
  vpu.divout = 0;
  vpu.divdp = 0;

  reciprocals[0] = u16(~0);
  for(u16 index : range(1, 512)) {
    u64 a = index + 512;
    u64 b = (u64(1) << 34) / a;
    reciprocals[index] = u16(b + 1 >> 8);
  }

  for(u16 index : range(0, 512)) {
    u64 a = index + 512 >> (index % 2 == 1);
    u64 b = 1 << 17;
    //find the largest b where b < 1.0 / sqrt(a)
    while(a * (b + 1) * (b + 1) < (u64(1) << 44)) b++;
    inverseSquareRoots[index] = u16(b >> 1);
  }

  if constexpr(Accuracy::RSP::Recompiler) {
    auto buffer = ares::Memory::FixedAllocator::get().tryAcquire(1_MiB);
    recompiler.allocator.resize(1_MiB, bump_allocator::executable, buffer);
    recompiler.reset();
  }

  if constexpr(Accuracy::RSP::SISD) {
    platform->status("RSP vectorization disabled (no SSE 4.1 support)");
  }
}

}
