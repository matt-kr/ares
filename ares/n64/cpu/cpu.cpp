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

  //Lumiverse addition: the inline load/store fast path (cpu.hpp) bypasses
  //the GDB watchpoint report, so it is suspended whenever a debugger client
  //has breakpoints/watchpoints; re-evaluated once per batch here.
  lumiverseFast.memoryLive = lumiverseFast.memory && !GDB::server.hasBreakpoints();

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

  //Lumiverse addition: LUMIVERSE_ARES_N64_CPU_FAST=1 (default off) enables
  //interpreter-path speedups. Device builds cannot obtain JIT memory (the
  //recompiler is force-disabled outside debugger launches — see
  //RetroGamingService.isJITAvailable), so the interpreter is what actually
  //runs on hardware. Configuration is read once in power() into
  //lumiverseFast (cpu.hpp). Sub-knobs, only honored while CPU_FAST=1:
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
  //  LUMIVERSE_ARES_N64_CPU_FAST_FETCH  (default 1, 0 = off) inline the
  //    instruction fetch for an aligned PC in KSEG0 RDRAM (the range the
  //    general devirtualize() already special-cases): skips the non-inlined
  //    devirtualize/vaddrAlignedError/fetch(PhysAccess) call chain. Any
  //    other PC (unaligned, TLB-mapped, KSEG1, ROM) takes the stock path,
  //    so exceptions and cache behavior are unchanged.
  //  LUMIVERSE_ARES_N64_CPU_FAST_MEM  (default 1, 0 = off) same idea for
  //    aligned KSEG0-RDRAM loads/stores (cpu.hpp read<Size>/write<Size>);
  //    suspended automatically while GDB breakpoints/watchpoints exist.
  const auto& fast = lumiverseFast;
  auto beginBatch = [&] {
    //Timer distance is modular: count/compare are n33 and the timer next
    //fires when count crosses compare from below, so once compare has been
    //passed (or is unused, e.g. compare=0) the true distance is the wrap
    //distance, not zero. The recompiler path's non-modular clamp degrades
    //its budget to zero in that state; computing it modularly here keeps
    //batches alive for games that never program the Compare timer.
    s64 timerDelta = (s64)(u64)n33(scc.compare - scc.count);
    s64 queueDelta = queue.timeToNextEvent();
    if(queueDelta < 0) queueDelta = 0;
    jitClockTarget = Thread::clock + min<s64>(fast.batch, min(timerDelta, queueDelta));
  };
  const u64 pcExec = ipu.pc;
  u32 opcodeWord;

  u32 fastFetchPaddr = 0;
  if(fast.fetch && !(Accuracy::CPU::Recompiler && recompiler.enabled) && !(pcExec & 3)
  && ((pcExec - 0xffff'ffff'8000'0000ull) <= 0x03ef'ffffull
      ? (fastFetchPaddr = (u32)pcExec & 0x3eff'ffff, true)
      : lumiverseTlbMemoHit<false>(pcExec, fastFetchPaddr))) {
    //fast fetch: aligned KSEG0 RDRAM (cached), or a TLB-mapped page whose
    //translation to cached RDRAM was memoized by the general path (round
    //8, LumiverseTlbMemo). Equivalent to fetch(devirtualize<Read, Word>(pc))
    //for these address classes.
    if(fast.batch && Thread::clock >= jitClockTarget) beginBatch();
    u32 paddr = fastFetchPaddr;
    if(context.littleEndian()) paddr ^= 4;
    opcodeWord = icache.fetch(pcExec, paddr, cpu);
    //idle-loop skip (CPU_FAST_IDLE): `j self` with a nop delay slot, seen
    //at the loop head (not in the delay slot). Step the whole machine one
    //quantum (bounded by the Compare timer and the next queued event) and
    //return to the scheduler; a pending interrupt is taken at the next
    //instruction() exactly as it would be after another loop iteration.
    if(fast.idle && (opcodeWord >> 26) == 2 && !pipeline.inDelaySlot()
    && (((pcExec + 4) & ~0x0fff'ffffull) | (u64)((opcodeWord & 0x03ff'ffff) << 2)) == pcExec
    && icache.fetch(pcExec + 4, paddr + 4, cpu) == 0) {
      s64 wait = fast.idle;
      const s64 timerDelta = (s64)(u64)n33(scc.compare - scc.count) * 2;
      const s64 queueDelta = queue.timeToNextEvent();
      if(timerDelta > 0 && timerDelta < wait) wait = timerDelta;
      if(queueDelta > 0 && queueDelta < wait) wait = queueDelta;
      wait &= ~1;
      if(wait >= 4) {
        step((u32)wait);
        return true;
      }
    }
    step(1 * 2);
  } else {
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

  if(fast.batch && Thread::clock >= jitClockTarget) beginBatch();
  auto data = fetch(access);
  if (!data) return true;
  opcodeWord = *data;
  }  //general fetch path

  pipeline.begin();
  if(!fast.noTrace || unlikely(recompiler.callInstructionPrologue)) {
    instructionPrologue(ipu.pc, opcodeWord);
  }
  decoderEXECUTE(opcodeWord);
  instructionEpilogue<0>();
  pipeline.end();
  if(fast.batch) {
    //Lumiverse diagnostic (temporary): batch-length telemetry under
    //LUMIVERSE_ARES_N64_CPU_DIAG=1; =2 adds an executed-PC histogram (top
    //entries every 2^28 instructions) to expose idle/poll loops.
    const bool batchEnd = Thread::clock >= jitClockTarget;
    if(unlikely(fast.diag)) {
      static u64 instrs = 0, syncs = 0, timerCap = 0, queueCap = 0, forceCap = 0;
      instrs++;
      if(fast.diag >= 2) {
        static struct { u64 pc; u32 op; u64 count; } hist[1 << 16] = {};
        auto& h = hist[(u32)(pcExec >> 2) & 0xffff];
        if(h.pc == pcExec) h.count++;
        else if(h.count < 256) { h.pc = pcExec; h.op = opcodeWord; h.count = 1; }
        else h.count -= 256;
        if((instrs & 0xfffffff) == 0) {
          //print the 12 hottest slots
          u32 top[12] = {}; u32 topCount = 0;
          for(u32 index = 0; index < (1 << 16); index++) {
            const u64 count = hist[index].count;
            if(!count) continue;
            u32 pos = topCount < 12 ? topCount++ : 12;
            //insertion into a descending list
            while(pos > 0 && hist[top[pos - 1]].count < count) { if(pos < 12) top[pos] = top[pos - 1]; pos--; }
            if(pos < 12) top[pos] = index;
          }
          fprintf(stderr, "[cpu-diag] pc histogram after %llu instrs:\n", (unsigned long long)instrs);
          for(u32 index = 0; index < topCount; index++) {
            auto& e = hist[top[index]];
            fprintf(stderr, "[cpu-diag]   pc=%016llx op=%08x count=%llu (%.1f%%)\n",
              (unsigned long long)e.pc, e.op, (unsigned long long)e.count, 100.0 * (double)e.count / (double)instrs);
          }
        }
      }
      if(batchEnd) {
        syncs++;
        if(jitClockTarget == 0) {
          forceCap++;
          static struct { u64 pc; u32 op; u64 count; } fhist[32] = {};
          u64 pc = ipu.pc; u32 slot = (u32)((pc >> 2) ^ (pc >> 11)) & 31;
          if(fhist[slot].pc == pc) fhist[slot].count++;
          else if(fhist[slot].count < 8) { fhist[slot].pc = pc; fhist[slot].op = opcodeWord; fhist[slot].count = 1; }
          if((forceCap & 0xffffff) == 0) for(auto& h : fhist) if(h.count > 65536)
            fprintf(stderr, "[cpu-diag]   forcePC=%016llx op=%08x count=%llu\n",
              (unsigned long long)h.pc, h.op, (unsigned long long)h.count);
        }
        else {
          s64 timerDelta = (s64)(u64)n33(scc.compare - scc.count);
          s64 queueDelta = queue.timeToNextEvent();
          if(timerDelta <= queueDelta && timerDelta < fast.batch) timerCap++;
          else if(queueDelta < fast.batch) queueCap++;
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

//Lumiverse addition: read the CPU_FAST configuration (see instruction()).
//Called from power(); the app sets its env knobs before the core is created.
auto CPU::lumiverseLoadFastConfig() -> void {
  auto& f = lumiverseFast;
  f = {};
  const char* v = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST");
  f.enabled = v && *v == '1';
  if(f.enabled) {
    const char* b = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_BATCH");
    const s64 batch = b ? ::atoll(b) : 32768;
    f.batch = batch > 0 ? batch : 0;
    const char* t = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_NOTRACE");
    f.noTrace = !t || *t != '0';
    const char* fe = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_FETCH");
    f.fetch = !fe || *fe != '0';
    const char* me = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_MEM");
    f.memory = !me || *me != '0';
    //LUMIVERSE_ARES_N64_CPU_FAST_TLB (default 1, 0 = off): one-page memo of
    //the last TLB translation per direction (see LumiverseTlbMemo)
    const char* te = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_TLB");
    f.tlb = !te || *te != '0';
    //LUMIVERSE_ARES_N64_CPU_FAST_IDLE (default 1024, 0 = off): clock quantum
    //(2 per CPU cycle) stepped at once while the CPU sits in a `j self; nop`
    //idle loop — round 10, from the CPU_DIAG histogram of Star Wars: Rogue
    //Squadron (60% of all executed instructions were that loop at
    //0x80001804). Only an interrupt leaves such a loop, so stepping the
    //machine in quanta instead of interpreting two instructions per two
    //cycles changes nothing but the interrupt latency (<= the quantum,
    //1024 clocks = 5.5 us, against a hardware latency of a few cycles).
    const char* id = ::getenv("LUMIVERSE_ARES_N64_CPU_FAST_IDLE");
    const s64 idle = id ? ::atoll(id) : 1024;
    f.idle = idle > 0 ? (idle & ~1) : 0;
    const char* d = ::getenv("LUMIVERSE_ARES_N64_CPU_DIAG");
    f.diag = d ? ::atoi(d) : 0;
  }
  f.memoryLive = f.memory && !GDB::server.hasBreakpoints();
  lumiverseTlbMemoInvalidate();
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
  lumiverseLoadFastConfig();

  if constexpr(Accuracy::CPU::Recompiler) {
    auto buffer = ares::Memory::FixedAllocator::get().tryAcquire(63_MiB);
    recompiler.allocator.resize(63_MiB, bump_allocator::executable, buffer);
    recompiler.reset();
  }
}

}
