//Lumiverse addition: opt-in asynchronous execution of audio (type-2) RSP
//tasks on a worker thread (LUMIVERSE_ARES_N64_ASYNC_AUDIO=1, default off).
//
//Motivation: with graphics tasks HLE'd (lumiverse-hle-gfx.cpp), the LLE
//audio microcode is the largest remaining RSP cost (~5-6 ms/frame on device,
//about half of all task dispatches in OoT). Audio tasks are pure
//run-to-completion batch jobs: DMA the command list + sample data in, mix,
//DMA the output buffers out, BREAK. Nothing on the emulated CPU consumes the
//results until the SP "task done" interrupt fires, and real hardware tasks
//took time anyway, so a DELAYED completion is authentic.
//
//Model: the SP_STATUS start write (osSpTaskStartGo) is intercepted after its
//set/clear bits are applied; instead of interleaving the LLE RSP with the
//CPU inside CPU::synchronize(), the whole ares RSP core runs to completion
//(BREAK/halt) on a single persistent worker thread while the emulated CPU
//keeps running. Completion (BREAK + SIG2 already applied by the microcode
//itself through the normal MTC0 paths) is observed by the emulation thread
//at its next CPU::synchronize() and the SP interrupt is raised THERE — MI is
//never touched from the worker.
//
//Ownership / race analysis (see M1-REPORT.md "Final-3ms round" for the full
//writeup):
//  - The RSP core (ipu/vpu/dmem/imem/pipeline/recompiler/dma/status,
//    Thread::clock) is a singleton owned by exactly one thread at a time:
//    the emulation thread normally, the worker between Begin and completion.
//    While in flight the emulation thread must not touch any of it:
//      * CPU::synchronize() skips rsp.clock adjustment and rsp.main() and
//        calls rsp.lumiverseAsyncPoll() instead (cpu.cpp).
//      * CPU reads of SP registers are served from a snapshot taken at
//        dispatch (authentic "task running" view: halted=0 broken=0, DMA
//        idle, signals as the start write left them) — rsp/io.cpp.
//      * Any CPU WRITE to the SP range (registers, DMEM/IMEM, SP_PC) blocks
//        in lumiverseAsyncDrain() until the task completes, then applies
//        normally. This also serializes back-to-back task dispatches (the
//        next osSpTaskStartGo is itself an SP_STATUS write).
//  - RDRAM: the worker reads task inputs (command list, sample banks) and
//    writes output buffers via SP DMA. The OS contract is that the CPU does
//    not touch a dispatched task's buffers until the done-interrupt (audio
//    heaps are double-buffered); paraLLEl-RDP already reads RDRAM from its
//    own threads on the same assumption. Plain byte arrays — races would
//    require a misbehaving game and were not observed (AI-dump validation).
//  - MI/CPU side effects from the worker are suppressed and deferred:
//    status.interruptOnBreak is cleared for the duration so BREAK cannot
//    call mi.raise(); SP_STATUS writes from the microcode that set/clear the
//    SP interrupt are recorded (lumiverseAsyncNoteSP*) and replayed on the
//    emulation thread at completion; cpu.forceSynchronize() is skipped in
//    worker context (rsp/io.cpp).
//  - Audio microcodes never touch DPC/RDP (type-2 tasks do not own the RDP;
//    verified across SF64/OoT/MM/Banjo censuses), and gfx-as-type-2 ucodes
//    (Banjo-Kazooie) are excluded by the banner check in lumiverse-hle.cpp.
//
//This file is included from rsp.cpp inside namespace ares::Nintendo64 before
//lumiverse-hle.cpp and io.cpp; it may not #include anything (rsp.cpp pulls
//in <atomic>/<thread>/<mutex>/<condition_variable> ahead of the namespace).

namespace {

auto lumiverseAsyncAudioEnabled() -> bool {
  static bool enabled = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_ASYNC_AUDIO");
    return value && ::atoi(value) >= 1;
  }();
  return enabled;
}

//LUMIVERSE_ARES_N64_ASYNC_AUDIO_SLIP_US=<µs>: bounded-slip delivery.
//Strict mode (0, default) delivers the completion IRQ at the exact emulated
//time serial LLE would, blocking the emulation thread if the worker has not
//finished when that window arrives. With slip N, the emulation thread keeps
//running instead and the IRQ may land up to ~N emulated microseconds later
//than serial (never earlier) — authentic in spirit: real hardware task
//durations varied and the OS tolerates IRQ latency. This converts the
//blocking wait into genuine overlap for OoT-class tasks whose delivery
//window arrives before the worker can finish. The bound is enforced in
//EMULATED cycles: once the budget passes (earliest-serial-point + slip) the
//emulation thread still blocks, so a slow worker cannot slip unboundedly.
//Thread::clock ticks at 187.5 per µs (93.75 MHz CPU, 2 half-cycles each).
auto lumiverseAsyncSlipCycles() -> s64 {
  static s64 slip = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_ASYNC_AUDIO_SLIP_US");
    const double us = value ? ::atof(value) : 0.0;
    return us > 0.0 ? (s64)(us * 187.5) : 0;
  }();
  return slip;
}

//LUMIVERSE_ARES_N64_ASYNC_AUDIO_LOG:
//  1 = per-1024-completions summary (late/drain counts, wall-wait stats,
//      worker execution wall time, slip stats)
//  2 = additionally one line per LATE completion (the slip budget ran out
//      before the worker finished) with the blocked wall time in µs
auto lumiverseAsyncAudioLogLevel() -> int {
  static int level = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_ASYNC_AUDIO_LOG");
    return value ? ::atoi(value) : 0;
  }();
  return level;
}

auto lumiverseAsyncNowUs() -> u64 {
  return (u64)std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

//true only on the worker thread while it is executing the RSP core; io.cpp
//uses it to defer MI side effects and skip cpu.forceSynchronize().
thread_local bool lumiverseRSPWorkerContextTLS = false;

auto lumiverseOnRSPWorker() -> bool {
  return lumiverseRSPWorkerContextTLS;
}

struct LumiverseAsyncAudio {
  //---- emulation-thread-owned state ----
  bool savedInterruptOnBreak = false;
  u32 statusSnapshot = 0;     //SP_STATUS view served to CPU reads mid-task
  bool startRequested = false;  //set by the dispatch hook, consumed by io.cpp
  u64 tasksRun = 0;
  u64 drains = 0;
  //serial-equivalent RSP clock, maintained while the worker owns the real
  //Thread::clock: completion is delivered only once the CPU has "paid" the
  //task's true cycle count, so the SP interrupt lands at the same emulated
  //time as serial LLE execution (validated byte-exact via the AI dump).
  s64 shadowClock = 0;
  //smallest task duration observed so far (0 until the first completion):
  //once the budget reaches this bound the emulation thread must learn the
  //true duration to stay serial-exact, so it waits for the worker if needed.
  s64 minDuration = 0;

  //---- worker plumbing ----
  std::thread worker;
  std::mutex mutex;
  std::condition_variable cv;
  bool workerStarted = false;
  bool taskPending = false;     //guarded by mutex
  bool shutdown = false;        //guarded by mutex
  std::atomic<bool> taskDone{false};
  //cycles the task consumed on the RSP core (worker-published):
  //durationCycles = full task length (final clock placement);
  //durationToHalt = up to the START of the halting instruction (delivery rule)
  std::atomic<s64> durationCycles{0};
  std::atomic<s64> durationToHalt{0};
  //deferred MI ops recorded by the microcode's own SP_STATUS writes
  std::atomic<bool> workerRaisedSP{false};
  std::atomic<bool> workerLoweredSP{false};

  //---- lateness instrumentation (emulation-thread-owned unless noted) ----
  u64 completions = 0;      //tasks fully delivered
  u64 lateCount = 0;        //blocked: slip budget (0 in strict mode) ran out
  u64 lateWaitTotalUs = 0;  //wall time the emulation thread spent blocked
  u64 lateWaitMaxUs = 0;
  u64 slipCount = 0;        //deliveries later than the serial point (slip mode)
  u64 slipTotalCycles = 0;  //emulated half-cycles of IRQ slip vs serial
  u64 slipMaxCycles = 0;
  u64 dispatchTimeUs = 0;   //wall clock at Begin (for window-arrival math)
  std::atomic<u64> workerExecTotalUs{0};  //worker-side execution wall time
  std::atomic<u64> workerExecMaxUs{0};

  ~LumiverseAsyncAudio() {
    if(!workerStarted) return;
    {
      std::lock_guard<std::mutex> lock(mutex);
      shutdown = true;
    }
    cv.notify_all();
    if(worker.joinable()) worker.join();
  }
};

LumiverseAsyncAudio lumiverseAsync;

//run the LLE RSP core to completion; called on the worker thread only.
//Returns the clock value at the START of the halting instruction/block —
//serial rsp.main() runs `while(Thread::clock < 0) instruction();`, so the
//task completes in the first scheduler window whose budget makes that
//pre-halt clock negative; Poll reproduces the exact same rule.
auto lumiverseAsyncExecuteTask() -> s64 {
  //audio tasks are tens of thousands of instructions; this guard only trips
  //on a wedged/foreign microcode. There is no safe fallback once state has
  //been mutated: finish loudly (equivalent to the hang LLE would produce).
  constexpr u64 stepGuard = 50'000'000;
  u64 steps = 0;
  s64 preHaltClock = rsp.clock;
  while(!rsp.status.halted) {
    preHaltClock = rsp.clock;
    rsp.instruction();
    rsp.dmaStep(rsp.clock - preHaltClock);
    if(++steps >= stepGuard) {
      fprintf(stderr, "[rsp-async-audio] step guard exceeded (microcode never halted) — forcing halt\n");
      rsp.status.halted = 1;
      rsp.status.broken = 1;
      break;
    }
  }
  //let any in-flight SP DMA settle (BREAK can retire before the final
  //writeback completes; the microcode's last DMA writes the output buffers)
  u32 dmaSpins = 0;
  while(rsp.dma.busy.any() && dmaSpins++ < 4096) {
    rsp.dmaStep(4096);
  }
  return preHaltClock;
}

auto lumiverseAsyncWorkerMain() -> void {
  lumiverseRSPWorkerContextTLS = true;
#if defined(__APPLE__)
  //audio tasks feed real-time playback and the emulation thread blocks on
  //this worker at the serial delivery window — a descheduled worker stalls
  //the whole emulator, so it must compete at interactive priority
  ::pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
  for(;;) {
    {
      std::unique_lock<std::mutex> lock(lumiverseAsync.mutex);
      lumiverseAsync.cv.wait(lock, [] {
        return lumiverseAsync.taskPending || lumiverseAsync.shutdown;
      });
      if(lumiverseAsync.shutdown) return;
      lumiverseAsync.taskPending = false;
    }
    const u64 execStartUs = lumiverseAsyncNowUs();
    const s64 startClock = rsp.clock;
    const s64 preHaltClock = lumiverseAsyncExecuteTask();
    lumiverseAsync.durationCycles.store(rsp.clock - startClock, std::memory_order_relaxed);
    lumiverseAsync.durationToHalt.store(preHaltClock - startClock, std::memory_order_relaxed);
    const u64 execUs = lumiverseAsyncNowUs() - execStartUs;
    lumiverseAsync.workerExecTotalUs.fetch_add(execUs, std::memory_order_relaxed);
    if(execUs > lumiverseAsync.workerExecMaxUs.load(std::memory_order_relaxed)) {
      lumiverseAsync.workerExecMaxUs.store(execUs, std::memory_order_relaxed);
    }
    {
      //publish all worker writes (RSP state, RDRAM output buffers) before
      //taskDone becomes visible; pair with the acquire loads below
      std::lock_guard<std::mutex> lock(lumiverseAsync.mutex);
      lumiverseAsync.taskDone.store(true, std::memory_order_release);
    }
    lumiverseAsync.cv.notify_all();
  }
}

//called by the dispatch hook (lumiverse-hle.cpp) when a type-2 non-gfx task
//is being dispatched and async execution is enabled
auto lumiverseAsyncRequestStart() -> void {
  lumiverseAsync.startRequested = true;
}

//io.cpp consumes the request after the SP_STATUS start write's own set/clear
//bits have been fully processed (the write typically clears SIG1/SIG2 —
//the microcode must see that, not the pre-write state)
auto lumiverseAsyncConsumeStartRequest() -> bool {
  const bool requested = lumiverseAsync.startRequested;
  lumiverseAsync.startRequested = false;
  return requested;
}

//deferred MI ops (recorded from the worker via rsp/io.cpp SP_STATUS writes)
auto lumiverseAsyncNoteSPRaise() -> void {
  lumiverseAsync.workerRaisedSP.store(true, std::memory_order_relaxed);
}

auto lumiverseAsyncNoteSPLower() -> void {
  lumiverseAsync.workerLoweredSP.store(true, std::memory_order_relaxed);
}

//SP_STATUS view served to CPU reads while the worker owns the RSP:
//running (halted=0 broken=0), DMA idle, signals as the start write left them
auto lumiverseAsyncIOReadShadow(u32 address, u32& data) -> bool {
  switch(address) {
  case 0: case 1: case 2: case 3:  //DMA address/length registers
    data = 0;
    return true;
  case 4:  //SP_STATUS
    data = lumiverseAsync.statusSnapshot;
    return true;
  case 5: case 6:  //SP_DMA_FULL / SP_DMA_BUSY
    data = 0;
    return true;
  default:  //SP_SEMAPHORE has a read side effect: caller drains first
    return false;
  }
}

}  //namespace

//dispatch: called from rsp/io.cpp after the SP_STATUS start write has been
//fully processed (status.halted already cleared). Emulation thread only.
auto RSP::lumiverseAsyncBegin() -> void {
  auto& async = lumiverseAsync;

  //snapshot the "task running" SP_STATUS view for CPU reads
  n32 snapshot;
  snapshot.bit( 0) = 0;  //halted
  snapshot.bit( 1) = 0;  //broken
  snapshot.bit( 2) = 0;  //dma busy
  snapshot.bit( 3) = 0;  //dma full
  snapshot.bit( 4) = 0;  //io full
  snapshot.bit( 5) = status.singleStep;
  snapshot.bit( 6) = status.interruptOnBreak;
  for(u32 index = 0; index < 8; index++) snapshot.bit(7 + index) = status.signal[index];
  async.statusSnapshot = snapshot;

  //suppress worker-side mi.raise() from BREAK; restored at completion
  async.savedInterruptOnBreak = status.interruptOnBreak;
  status.interruptOnBreak = 0;

  async.workerRaisedSP.store(false, std::memory_order_relaxed);
  async.workerLoweredSP.store(false, std::memory_order_relaxed);
  async.taskDone.store(false, std::memory_order_relaxed);
  async.durationCycles.store(0, std::memory_order_relaxed);
  async.durationToHalt.store(0, std::memory_order_relaxed);
  async.shadowClock = clock;  //serial-equivalent RSP clock at dispatch
  async.dispatchTimeUs = lumiverseAsyncNowUs();
  async.tasksRun++;

  if(!async.workerStarted) {
    async.workerStarted = true;
    async.worker = std::thread(lumiverseAsyncWorkerMain);
  }

  lumiverseAsyncInFlight = true;  //emulation-thread-owned; gates cpu.cpp/io.cpp
  {
    std::lock_guard<std::mutex> lock(async.mutex);
    async.taskPending = true;
  }
  async.cv.notify_all();
}

//applies the completed task's side effects; emulation thread only, and only
//after taskDone has been observed with acquire semantics
auto RSP::lumiverseAsyncComplete() -> void {
  auto& async = lumiverseAsync;
  lumiverseAsyncInFlight = false;

  status.interruptOnBreak = async.savedInterruptOnBreak;

  //the microcode finished with BREAK (+SIG2 via its own MTC0 write); deliver
  //the SP interrupt on THIS thread exactly as BREAK would have
  const bool raise = async.workerRaisedSP.load(std::memory_order_relaxed)
    || (status.broken && status.interruptOnBreak);
  if(async.workerLoweredSP.load(std::memory_order_relaxed) && !raise) mi.lower(MI::IRQ::SP);
  if(raise) mi.raise(MI::IRQ::SP);

  //restore the serial-equivalent timeline: the RSP clock lands exactly where
  //serial LLE execution would have left it (shadow budget + task duration);
  //any remaining negative balance is burned by normal halted stepping
  Thread::clock = async.shadowClock + async.durationCycles.load(std::memory_order_relaxed);
  dma.clock = 0;

  async.completions++;

  static int spIrqLog = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_SP_IRQ_LOG");
    return value ? ::atoi(value) : 0;
  }();
  if(spIrqLog) {
    fprintf(stderr, "[async-complete] task=%llu duration=%lld shadow=%lld drains=%llu\n",
      (unsigned long long)async.tasksRun,
      (long long)async.durationCycles.load(std::memory_order_relaxed),
      (long long)async.shadowClock,
      (unsigned long long)async.drains);
  }

  //per-1024 lateness summary (LUMIVERSE_ARES_N64_ASYNC_AUDIO_LOG=1)
  if(lumiverseAsyncAudioLogLevel() >= 1 && (async.completions & 1023) == 0) {
    const u64 execTotal = async.workerExecTotalUs.load(std::memory_order_relaxed);
    fprintf(stderr,
      "[async-audio] tasks=%llu late=%llu (%.1f%%) waitTotal=%llums waitMax=%lluus "
      "workerExecAvg=%lluus workerExecMax=%lluus slips=%llu slipAvg=%lluus slipMax=%lluus drains=%llu\n",
      (unsigned long long)async.completions,
      (unsigned long long)async.lateCount,
      100.0 * (double)async.lateCount / (double)async.completions,
      (unsigned long long)(async.lateWaitTotalUs / 1000),
      (unsigned long long)async.lateWaitMaxUs,
      (unsigned long long)(execTotal / async.completions),
      (unsigned long long)async.workerExecMaxUs.load(std::memory_order_relaxed),
      (unsigned long long)async.slipCount,
      (unsigned long long)(async.slipCount ? async.slipTotalCycles / async.slipCount / 187 : 0),
      (unsigned long long)(async.slipMaxCycles / 187),
      (unsigned long long)async.drains);
  }
}

//polled from CPU::synchronize() while a task is in flight (cpu.cpp).
//clocks = the budget the serial scheduler would have granted the RSP this
//synchronize. Completion is delivered at the EXACT emulated time serial LLE
//would deliver it: once the budget covers the task's true duration. Until
//the budget reaches the smallest duration ever observed the emulation
//thread runs fully concurrent with the worker; past that bound it must know
//the true duration to stay serial-exact, so it blocks for the worker if the
//worker has not finished yet (on device the emulation thread is the slow
//side, so this wait is rare; on an uncapped host it bounds the overlap).
auto RSP::lumiverseAsyncPoll(s64 clocks) -> void {
  auto& async = lumiverseAsync;
  async.shadowClock -= clocks;
  if(!async.taskDone.load(std::memory_order_acquire)) {
    //keep running while the serial delivery window has not arrived; in
    //bounded-slip mode, additionally keep running (overlapping the worker)
    //until the slip budget past the earliest possible serial point is spent
    if(async.shadowClock + async.minDuration + lumiverseAsyncSlipCycles() > 0) return;
    //LATE: the slip budget (0 in strict mode) ran out before the worker
    //finished — the emulation thread must block. Every blocked µs stretches
    //the frame, so this is what the slip knob exists to eliminate.
    const u64 waitStartUs = lumiverseAsyncNowUs();
    {
      std::unique_lock<std::mutex> lock(async.mutex);
      async.cv.wait(lock, [&] {
        return async.taskDone.load(std::memory_order_acquire);
      });
    }
    const u64 waitedUs = lumiverseAsyncNowUs() - waitStartUs;
    async.lateCount++;
    async.lateWaitTotalUs += waitedUs;
    if(waitedUs > async.lateWaitMaxUs) async.lateWaitMaxUs = waitedUs;
    if(lumiverseAsyncAudioLogLevel() >= 2) {
      fprintf(stderr, "[async-audio] late completion: waited=%lluus (task=%llu, windowAt=%llums after dispatch)\n",
        (unsigned long long)waitedUs, (unsigned long long)async.tasksRun,
        (unsigned long long)((waitStartUs - async.dispatchTimeUs) / 1000));
    }
  }
  const s64 durationToHalt = async.durationToHalt.load(std::memory_order_relaxed);
  if(async.minDuration == 0 || durationToHalt < async.minDuration) async.minDuration = durationToHalt;
  if(async.shadowClock + durationToHalt < 0) {
    //IRQ slip vs the serial delivery point (0 modulo window granularity in
    //strict mode; bounded by the slip budget otherwise)
    const s64 slipped = -(async.shadowClock + durationToHalt);
    if(slipped > 0 && lumiverseAsyncSlipCycles() > 0) {
      async.slipCount++;
      async.slipTotalCycles += (u64)slipped;
      if((u64)slipped > async.slipMaxCycles) async.slipMaxCycles = (u64)slipped;
    }
    lumiverseAsyncComplete();
  }
}

//blocks the emulation thread until the in-flight task completes; used to
//serialize any CPU access that would race the worker (SP writes, DMEM/IMEM
//access, SP_PC, semaphore reads, save states, power)
auto RSP::lumiverseAsyncDrain() -> void {
  if(!lumiverseAsyncInFlight) return;
  auto& async = lumiverseAsync;
  async.drains++;
  {
    std::unique_lock<std::mutex> lock(async.mutex);
    async.cv.wait(lock, [&] {
      return async.taskDone.load(std::memory_order_acquire);
    });
  }
  lumiverseAsyncComplete();
}
