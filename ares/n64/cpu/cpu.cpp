#include <n64/n64.hpp>
#include <nall/gdb/server.hpp>

namespace ares::Nintendo64 {

CPU cpu;
#include "context.cpp"
#include "dcache.cpp"
#include "tlb.cpp"
#include "memory.cpp"
#include "exceptions.cpp"
#include "algorithms.cpp"
#include "interpreter.cpp"
#include "decoder.cpp"
#include "interpreter-ipu.cpp"
#include "interpreter-scc.cpp"
#include "interpreter-fpu.cpp"
#include "interpreter-cop2.cpp"
#include "recompiler.cpp"
#include "recompiler-fpu.cpp"
#include "recompiler-ipu.cpp"
#include "debugger.cpp"
#include "serialization.cpp"
#include "disassembler.cpp"
#include "emux.cpp"

auto CPU::load(Node::Object parent) -> void {
  node = parent->append<Node::Object>("CPU");
  debugger.load(node);
}

auto CPU::unload() -> void {
  debugger.unload();
  node.reset();
}

auto CPU::main() -> void {
  while(!vi.refreshed && GDB::server.reportPC(ipu.pc & 0xFFFFFFFF)) {
    if(instruction()) synchronize();
  }

  vi.refreshed = false;
  queue.remove(Queue::GDB_Poll);
  if(GDB::server.hasClient()) {
    queueInsert(Queue::GDB_Poll, (93750000*2)/60/240);
  }
}

auto CPU::gdbPoll() -> void {
  if(GDB::server.hasClient()) {
    GDB::server.updateLoop();
    queueInsert(Queue::GDB_Poll, (93750000*2)/60/240);
  }
}

auto CPU::queueInsert(u32 event, u32 clocks) -> void {
  if(!queue.insert(event, clocks)) return;
  s64 queueDelta = queue.timeToNextEvent();
  if(queueDelta < 0) queueDelta = 0;
  s64 queueTarget = Thread::clock + queueDelta;
  if(queueTarget < jitClockTarget) jitClockTarget = queueTarget;
}

auto CPU::forceSynchronize() -> void {
  jitClockTarget = 0;
}

auto CPU::synchronize() -> void {
  auto clocks = Thread::clock;
  Thread::clock = 0;
  jitClockTarget = 0;

   vi.clock -= clocks;
   ai.clock -= clocks;
  //Lumiverse addition: while an async audio task is in flight the worker
  //thread owns the RSP (including Thread::clock); the emulation thread only
  //polls for completion here — the safe scheduler point where the deferred
  //BREAK/SIG2 SP interrupt is delivered. See rsp/lumiverse-async-audio.cpp.
  if(!rsp.lumiverseAsyncInFlight) rsp.clock -= clocks;
  rdp.clock -= clocks;
  pif.clock -= clocks;
  vi.main();
  ai.main();
  if(rsp.lumiverseAsyncInFlight) rsp.lumiverseAsyncPoll(clocks);
  else rsp.main();
  rdp.main();
  pif.main();

  queue.step(clocks, [](u32 event) {
    switch(event) {
    case Queue::PI_DMA_Read:   return pi.dmaFinished();
    case Queue::PI_DMA_Write:  return pi.dmaFinished();
    case Queue::PI_BUS_Write:  return pi.writeFinished();
    case Queue::SI_DMA_Read:   return si.dmaRead();
    case Queue::SI_DMA_Write:  return si.dmaWrite();
    case Queue::SI_BUS_Write:  return si.writeFinished();
    case Queue::RTC_Tick:      return cartridge.rtc.tick();
    case Queue::EEPROM_Write:  return cartridge.eepromFinish();
    case Queue::DD_Clock_Tick:  return dd.rtc.tickClock();
    case Queue::DD_MECHA_Response:  return dd.mechaResponse();
    case Queue::DD_BM_Request:  return dd.bmRequest();
    case Queue::DD_Motor_Mode:  return dd.motorChange();
    case Queue::GDB_Poll:      return cpu.gdbPoll();
    }
  });

  clocks >>= 1;
  if(scc.count < scc.compare && scc.count + clocks >= scc.compare) {
    setInterruptPending(Interrupt::Timer, 1);
  }
  scc.count += clocks;
  profile.cpuCycles += clocks;
  if (scc.status.exceptionLevel) profile.cpuCyclesExc += clocks;
}

auto CPU::setInterruptPending(u32 bit, bool value) -> void {
  scc.cause.interruptPending.bit(bit) = value;
  interruptPoll();
}

auto CPU::interruptPoll() -> void {
  if(auto interrupts = scc.cause.interruptPending & scc.status.interruptMask) {
    if(scc.status.interruptEnable && !scc.status.exceptionLevel && !scc.status.errorLevel) {
      forceSynchronize();
    }
  }
}

auto CPU::instruction() -> bool {
  if(auto interrupts = scc.cause.interruptPending & scc.status.interruptMask) {
    if(scc.status.interruptEnable && !scc.status.exceptionLevel && !scc.status.errorLevel) {
      debugger.interrupt(scc.cause.interruptPending);
      step(1 * 2);
      exception.interrupt();
      return true;
    }
  }

  if (scc.nmiPending) {
    debugger.nmi();
    step(1 * 2);
    exception.nmi();
    return true;
  }
  if (scc.sysadFrozen) {
    step(1 * 2);
    return true;
  }

  auto access = devirtualize<Read, Word>(ipu.pc);
  if(!access) return true;

  if(Accuracy::CPU::Recompiler && recompiler.enabled && access.cache) {
    if(vaddrAlignedError<Word>(access.vaddr, false)) return true;
    auto block = recompiler.block(ipu.pc, access.paddr);
    if(block) {
      if(Thread::clock >= jitClockTarget) {
        //Lumiverse addition: LUMIVERSE_ARES_N64_JIT_INTERLEAVE overrides the
        //compile-time interleave cap (cycles the CPU may run unsynchronized).
        //Unset/0 keeps the stock Accuracy::CPU::JitInterleaving value.
        static const s64 jitInterleaving = [] {
          const char* value = ::getenv("LUMIVERSE_ARES_N64_JIT_INTERLEAVE");
          const s64 override = value ? ::atoll(value) : 0;
          return override > 0 ? override : Accuracy::CPU::JitInterleaving;
        }();
        s64 timerDelta = (s64)scc.compare - (s64)scc.count;
        if(timerDelta < 0) timerDelta = 0;
        s64 queueDelta = queue.timeToNextEvent();
        if(queueDelta < 0) queueDelta = 0;
        s64 capBudget = min<s64>(jitInterleaving, min(timerDelta, queueDelta));
        jitClockTarget = Thread::clock + capBudget;
      }
      block->execute(*this);
      return Thread::clock >= jitClockTarget;
    }
  }

  //Lumiverse addition: LUMIVERSE_ARES_N64_CPU_FAST=1 (default off) enables
  //interpreter-path speedups. Device builds cannot obtain JIT memory (the
  //recompiler is force-disabled outside debugger launches — see
  //RetroGamingService.isJITAvailable), so the interpreter is what actually
  //runs on hardware. Sub-knobs, only honored while CPU_FAST=1:
  //  LUMIVERSE_ARES_N64_CPU_FAST_BATCH  clock-tick budget between scheduler
  //    synchronizations (default 32768, 0 = stock per-instruction sync).
  //    Mirrors the recompiler's jitClockTarget budget: min(batch, timer
  //    delta, queue delta); queueInsert() lowers the live target and
  //    forceSynchronize() zeroes it, so interrupts/events still end a batch
  //    immediately.
  //  LUMIVERSE_ARES_N64_CPU_FAST_NOTRACE  (default 1, 0 = off) skip the
  //    per-instruction tracer-enabled virtual call unless the instruction
  //    tracer is armed; recompiler.callInstructionPrologue already tracks
  //    the tracer toggle, and recompiled code compiles the same short-
  //    circuit in, so tracer-off behavior is identical.
  static const bool lumiverseCpuFast = [] {
    const char* v = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST");
    return v && *v == '1';
  }();
  static const s64 lumiverseCpuFastBatch = [] {
    if(!lumiverseCpuFast) return (s64)0;
    const char* v = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_BATCH");
    const s64 value = v ? ::atoll(v) : 32768;
    return value > 0 ? value : (s64)0;
  }();
  static const bool lumiverseCpuFastNoTrace = [] {
    if(!lumiverseCpuFast) return false;
    const char* v = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_NOTRACE");
    return !v || *v != '0';
  }();

  if(lumiverseCpuFastBatch && Thread::clock >= jitClockTarget) {
    //Timer distance is modular: count/compare are n33 and the timer next
    //fires when count crosses compare from below, so once compare has been
    //passed (or is unused, e.g. compare=0) the true distance is the wrap
    //distance, not zero. The recompiler path's non-modular clamp degrades
    //its budget to zero in that state; computing it modularly here keeps
    //batches alive for games that never program the Compare timer.
    s64 timerDelta = (s64)(u64)n33(scc.compare - scc.count);
    s64 queueDelta = queue.timeToNextEvent();
    if(queueDelta < 0) queueDelta = 0;
    jitClockTarget = Thread::clock + min<s64>(lumiverseCpuFastBatch, min(timerDelta, queueDelta));
  }
  auto data = fetch(access);
  if (!data) return true;
  pipeline.begin();
  if(!lumiverseCpuFastNoTrace || unlikely(recompiler.callInstructionPrologue)) {
    instructionPrologue(ipu.pc, *data);
  }
  decoderEXECUTE(*data);
  instructionEpilogue<0>();
  pipeline.end();
  if(lumiverseCpuFastBatch) {
    //Lumiverse diagnostic (temporary): batch-length telemetry under
    //LUMIVERSE_ARES_N64_CPU_DIAG=1.
    static const bool diag = [] {
      const char* v = ::getenv("LUMIVERSE_ARES_N64_CPU_DIAG");
      return v && *v == '1';
    }();
    const bool batchEnd = Thread::clock >= jitClockTarget;
    if(unlikely(diag)) {
      static u64 instrs = 0, syncs = 0, timerCap = 0, queueCap = 0, forceCap = 0;
      instrs++;
      if(batchEnd) {
        syncs++;
        if(jitClockTarget == 0) {
          forceCap++;
          static struct { u64 pc; u32 op; u64 count; } fhist[32] = {};
          u64 pc = ipu.pc; u32 slot = (u32)((pc >> 2) ^ (pc >> 11)) & 31;
          if(fhist[slot].pc == pc) fhist[slot].count++;
          else if(fhist[slot].count < 8) { fhist[slot].pc = pc; fhist[slot].op = *data; fhist[slot].count = 1; }
          if((forceCap & 0xffffff) == 0) for(auto& h : fhist) if(h.count > 65536)
            fprintf(stderr, "[cpu-diag]   forcePC=%016llx op=%08x count=%llu\n",
              (unsigned long long)h.pc, h.op, (unsigned long long)h.count);
        }
        else {
          s64 timerDelta = (s64)scc.compare - (s64)scc.count;
          s64 queueDelta = queue.timeToNextEvent();
          if(timerDelta <= queueDelta && timerDelta < lumiverseCpuFastBatch) timerCap++;
          else if(queueDelta < lumiverseCpuFastBatch) queueCap++;
        }
        if((syncs & 0x3ffff) == 0)
          fprintf(stderr, "[cpu-diag] instrs=%llu syncs=%llu avgBatch=%.1f force=%llu timerCap=%llu queueCap=%llu\n",
            (unsigned long long)instrs, (unsigned long long)syncs,
            (double)instrs / (double)syncs, (unsigned long long)forceCap,
            (unsigned long long)timerCap, (unsigned long long)queueCap);
      }
    }
    return batchEnd;
  }
  return true;
}

auto CPU::instructionPrologue(u64 address, u32 instruction) -> void {
  debugger.instruction(address, instruction);
}

template<bool Recompiled>
auto CPU::instructionEpilogue() -> void {
  if constexpr(!Recompiled) {
    ipu.r[0].u64 = 0;
  }
}

auto CPU::raiseCoprocessor1Exception() -> void {
  exception.coprocessor1();
}

auto CPU::power(bool reset) -> void {
  Thread::reset();

  context.endian = Context::Endian::Big;
  context.mode = Context::Mode::Kernel;
  context.bits = 64;
  for(auto& segment : context.segment) segment = Context::Segment::Unused;
  icache.power(reset);
  dcache.power(reset);
  for(auto& entry : tlb.entry) entry = {}, entry.synchronize();
  tlb.physicalAddress = 0;
  for(auto& r : ipu.r) r.u64 = 0;
  ipu.lo.u64 = 0;
  ipu.hi.u64 = 0;
  ipu.r[29].u64 = 0xffff'ffff'a400'1ff0ull;  //stack pointer
  pipeline.setPc(0xffff'ffff'bfc0'0000ull);
  scc = {};
  for(auto& r : fpu.r) r.u64 = 0;
  fpu.csr = {};
  cop2 = {};
  emuxState = {};
  fenv.setRound(float_env::toNearest);
  context.setMode();

  if constexpr(Accuracy::CPU::Recompiler) {
    auto buffer = ares::Memory::FixedAllocator::get().tryAcquire(63_MiB);
    recompiler.allocator.resize(63_MiB, bump_allocator::executable, buffer);
    recompiler.reset();
  }
}

}
