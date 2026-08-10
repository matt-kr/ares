//Lumiverse addition: RSP task-dispatch hook.
//
//Called from SP_STATUS writes on the halted -> running transition, i.e. the
//moment the OS dispatches an OSTask (osSpTaskStartGo). The OSTask struct sits
//at DMEM 0xFC0 (16 big-endian words).
//
//LUMIVERSE_ARES_N64_RSP_HLE:
//  unset/0 = off (stock LLE behavior, zero overhead beyond one branch)
//  1       = census: log each unique task signature once + periodic counts
//  2       = census + execute recognized graphics tasks natively (HLE),
//            falling back to LLE for anything unrecognized
//
//This file is included from rsp.cpp inside namespace ares::Nintendo64, so it
//may not #include anything; it sticks to fixed-size tables instead of STL.

namespace {

struct LumiverseTaskSignature {
  u64 ucodeHash = 0;
  u32 type = 0;
  u64 dispatchCount = 0;
  //banner contains "Gfx": excludes gfx microcodes dispatched as type-2 tasks
  //(Banjo-Kazooie) from async audio execution — they touch the RDP
  bool bannerIsGfx = false;
  //hash matches LUMIVERSE_ARES_N64_ASYNC_AUDIO_EXCLUDE (see below)
  bool asyncExcluded = false;
};

//LUMIVERSE_ARES_N64_ASYNC_AUDIO_EXCLUDE=<prefix>[,<prefix>...] — lowercase
//hex prefixes matched against the 16-digit ucode hash; matching audio
//ucodes fall back to serial LLE without disabling async audio globally
//(e.g. "ca93,c1a9" excludes OoT's and SF64's audio ucodes).
auto lumiverseAsyncAudioHashExcluded(u64 ucodeHash) -> bool {
  static const char* list = ::getenv("LUMIVERSE_ARES_N64_ASYNC_AUDIO_EXCLUDE");
  if(!list || !list[0]) return false;
  char hash[17];
  snprintf(hash, sizeof(hash), "%016llx", (unsigned long long)ucodeHash);
  auto lower = [](char c) -> char { return c >= 'A' && c <= 'F' ? c - 'A' + 'a' : c; };
  const char* scan = list;
  while(*scan) {
    u32 length = 0;
    while(scan[length] && scan[length] != ',') length++;
    if(length >= 1 && length <= 16) {
      bool match = true;
      for(u32 index = 0; index < length && match; index++) {
        if(lower(scan[index]) != hash[index]) match = false;
      }
      if(match) return true;
    }
    scan += length;
    if(*scan == ',') scan++;
  }
  return false;
}

constexpr u32 LumiverseMaxTaskSignatures = 32;

auto lumiverseHLELevel() -> int {
  static int level = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE");
    return value ? ::atoi(value) : 0;
  }();
  return level;
}

//per-task debug prints (task fields, RDRAM probes). Default off: these hit
//stderr on every graphics task and are measurable on device.
auto lumiverseHLEDebug() -> int {
  static int debug = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_DEBUG");
    return value ? ::atoi(value) : 0;
  }();
  return debug;
}

//LUMIVERSE_ARES_N64_RELAX_IO_SYNC=1 skips the cpu.forceSynchronize() calls
//triggered by SP status/semaphore READS (rsp/io.cpp) and DPC READS
//(rdp/io.cpp). Those syncs bound poll staleness under LLE; with graphics
//HLE'd the registers are stable between tasks and the forced JIT-window
//resets only cost time. Reads still see current state — the CPU simply keeps
//its JIT batch instead of re-synchronizing immediately. Default off.
auto lumiverseRelaxIOSync() -> int {
  static int relax = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_RELAX_IO_SYNC");
    return value ? ::atoi(value) : 0;
  }();
  return relax;
}

//side-effect-free RDRAM byte read (bypasses RBus accounting)
auto lumiverseRDRAMByte(u32 address) -> u8 {
  return rdram.ram.Memory::Writable::read<Byte>(address & 0x00ffffff);
}

auto lumiverseHashRDRAM(u32 address, u32 length) -> u64 {
  u64 hash = 14695981039346656037ull;  //FNV-1a
  for(u32 index = 0; index < length; index++) {
    hash = (hash ^ lumiverseRDRAMByte(address + index)) * 1099511628211ull;
  }
  return hash;
}

//scans the ucode data segment for the embedded ASCII banner, e.g.
//"RSP Gfx ucode F3DEX.NoN fifo 1.22" / "RSP SW Version ..."
auto lumiverseFindBanner(u32 dataAddress, u32 scanLength, char* out, u32 outSize) -> bool {
  for(u32 index = 0; index + 4 < scanLength; index++) {
    if(lumiverseRDRAMByte(dataAddress + index + 0) != 'R') continue;
    if(lumiverseRDRAMByte(dataAddress + index + 1) != 'S') continue;
    if(lumiverseRDRAMByte(dataAddress + index + 2) != 'P') continue;
    u32 length = 0;
    while(length < outSize - 1) {
      u8 byte = lumiverseRDRAMByte(dataAddress + index + length);
      if(byte < 0x20 || byte > 0x7e) break;
      out[length++] = (char)byte;
    }
    out[length] = 0;
    if(length >= 8) return true;
  }
  out[0] = 0;
  return false;
}

//implemented in lumiverse-hle-gfx.cpp; returns true when the task was
//executed natively (RDP commands already queued via vulkan.queueHLECommands)
auto lumiverseExecuteGraphicsTask(const u32 task[16], u64 ucodeHash) -> bool;

}  //namespace

auto RSP::lumiverseTaskDispatchHook() -> bool {
  const int level = lumiverseHLELevel();
  //async audio needs the census machinery (ucode identity) even at level 0
  if(level < 1 && !lumiverseAsyncAudioEnabled()) return false;

  u32 task[16];
  for(u32 index = 0; index < 16; index++) {
    task[index] = dmem.read<Word>(0xfc0 + index * 4);
  }
  const u32 taskType      = task[ 0];
  const u32 ucode         = task[ 4] & 0x00ffffff;
  const u32 ucodeSize     = task[ 5];
  const u32 ucodeData     = task[ 6] & 0x00ffffff;
  const u32 ucodeDataSize = task[ 7];
  const u32 dataPtr       = task[12] & 0x00ffffff;
  const u32 dataSize      = task[13];

  //sanity guard: some games start the RSP before a valid OSTask is present
  //in DMEM (boot/IPL noise: garbage type field, absurd sizes). Never let
  //those reach the census or HLE dispatch logic; LLE runs them unchanged.
  if(taskType != 1 && taskType != 2) return false;
  if(!ucode || ucodeSize == 0 || ucodeSize > 0x1000) return false;
  if(ucodeDataSize > 0x1000) return false;

  //hash the microcode text: identity of the task's program
  const u32 hashLength = ucodeSize >= 256 && ucodeSize <= 0x1000 ? ucodeSize : 0x1000;
  const u64 ucodeHash = ucode ? lumiverseHashRDRAM(ucode, hashLength) : 0;

  //diagnostic (shares LUMIVERSE_ARES_N64_SP_IRQ_LOG with mi.cpp): dispatch
  //timestamps to pair with [sp-irq] lines for per-task latency comparison
  static int spIrqLog = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_SP_IRQ_LOG");
    return value ? ::atoi(value) : 0;
  }();
  if(spIrqLog) {
    fprintf(stderr, "[sp-dispatch] type=%u count=%llu\n",
      taskType, (unsigned long long)(u64)cpu.scc.count);
  }

  static LumiverseTaskSignature signatures[LumiverseMaxTaskSignatures];
  static u32 signatureCount = 0;
  static u64 totalDispatches = 0;
  totalDispatches++;

  LumiverseTaskSignature* signature = nullptr;
  for(u32 index = 0; index < signatureCount; index++) {
    if(signatures[index].ucodeHash == ucodeHash && signatures[index].type == taskType) {
      signature = &signatures[index];
      break;
    }
  }
  if(!signature && signatureCount < LumiverseMaxTaskSignatures) {
    signature = &signatures[signatureCount++];
    signature->ucodeHash = ucodeHash;
    signature->type = taskType;

    char banner[96];
    lumiverseFindBanner(ucodeData, ucodeDataSize ? (ucodeDataSize < 0x1000 ? ucodeDataSize : 0x1000) : 0x800, banner, sizeof(banner));
    for(const char* scan = banner; scan[0]; scan++) {
      if(scan[0] == 'G' && scan[1] == 'f' && scan[2] == 'x') { signature->bannerIsGfx = true; break; }
    }
    signature->asyncExcluded = lumiverseAsyncAudioHashExcluded(ucodeHash);
    fprintf(stderr,
      "[rsp-hle] new task: type=%u hash=%016llx ucode=%06x/%u data=%06x/%u dl=%06x/%u banner=\"%s\"%s\n",
      taskType, (unsigned long long)ucodeHash, ucode, ucodeSize, ucodeData, ucodeDataSize,
      dataPtr, dataSize, banner, signature->asyncExcluded ? " [async-audio excluded]" : "");
  }
  if(signature) signature->dispatchCount++;

  //first few graphics tasks: log the fifo/output fields to understand what
  //the game expects the microcode to write back
  //(LUMIVERSE_ARES_N64_RSP_HLE_DEBUG=1 only; keep the hot path quiet)
  static u32 gfxTaskFieldLogs = 0;
  if(lumiverseHLEDebug() >= 1 && taskType == 1 && gfxTaskFieldLogs < 8) {
    gfxTaskFieldLogs++;
    const u32 outBuff = task[10] & 0x00ffffff;
    const u32 outBuffSize = task[11] & 0x00ffffff;
    auto word = [&](u32 address) -> u32 {
      return rdram.ram.Memory::Writable::read<Word>(address & 0x00ffffff);
    };
    fprintf(stderr,
      "[rsp-hle] gfx task fields: flags=%08x dramStack=%08x/%u outBuff=%08x outBuffSize=%08x yield=%08x/%u\n",
      task[1], task[8], task[9], task[10], task[11], task[14], task[15]);
    fprintf(stderr,
      "[rsp-hle]   probe *[outBuffSize]=%08x fifo[0..3]=%08x %08x %08x %08x\n",
      word(outBuffSize), word(outBuff), word(outBuff + 4), word(outBuff + 8), word(outBuff + 12));
  }

  if((totalDispatches & 1023) == 0) {
    fprintf(stderr, "[rsp-hle] dispatches=%llu:", (unsigned long long)totalDispatches);
    for(u32 index = 0; index < signatureCount; index++) {
      fprintf(stderr, " type%u/%04llx=%llu",
        signatures[index].type,
        (unsigned long long)(signatures[index].ucodeHash & 0xffff),
        (unsigned long long)signatures[index].dispatchCount);
    }
    fprintf(stderr, "\n");
  }

  //audio command-list (alist) census: audio ABI commands are 8-byte pairs
  //with the opcode in the top byte of the first word (0x00..0x1f range for
  //the Nintendo ABI families). Enable with LUMIVERSE_ARES_N64_RSP_HLE_ALIST_LOG=1.
  static int alistLog = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_ALIST_LOG");
    return value ? ::atoi(value) : 0;
  }();
  if(alistLog >= 1 && taskType == 2 && dataPtr && dataSize >= 8 && dataSize <= 0x10000) {
    static u64 alistCounts[64];
    static u64 alistTasks = 0;
    static bool alistSeen[64];
    bool newOpcode = false;
    for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
      const u8 opcode = lumiverseRDRAMByte(dataPtr + offset) & 0x3f;
      alistCounts[opcode]++;
      if(!alistSeen[opcode]) { alistSeen[opcode] = true; newOpcode = true; }
    }
    alistTasks++;
    if(newOpcode || (alistTasks & 1023) == 0) {
      fprintf(stderr, "[rsp-hle-alist] hash=%04llx tasks=%llu census:",
        (unsigned long long)(ucodeHash & 0xffff), (unsigned long long)alistTasks);
      for(u32 opcode = 0; opcode < 64; opcode++) {
        if(alistSeen[opcode]) fprintf(stderr, " %02x=%llu", opcode, (unsigned long long)alistCounts[opcode]);
      }
      fprintf(stderr, "\n");
    }
  }

  //level 2: execute recognized graphics tasks natively; anything else (and
  //any failure) falls back to LLE by returning false.
  if(level >= 2 && taskType == 1) {
    if(lumiverseExecuteGraphicsTask(task, ucodeHash)) {
      //task ran natively. The caller (io.cpp SP_STATUS write) applies the
      //completion status — BREAK plus the microcode's task-done signal
      //(SIG2, per n64js devices/sp.js TASKDONE|BROKE|HALT) — after the
      //start write's own set/clear bits have been processed.
      return true;
    }
  }

  //async audio (LUMIVERSE_ARES_N64_ASYNC_AUDIO=1): request worker-thread
  //execution of type-2 (audio) tasks. The task still dispatches through the
  //normal LLE path (halted -> 0, status bits processed); io.cpp hands the
  //core to the worker only after this SP_STATUS write fully applies.
  //Excluded: gfx-banner ucodes running as type 2 (Banjo-Kazooie — they kick
  //the RDP) and unrecognized signatures (table overflow, e.g. MK64's
  //self-patching audio ucode would thrash the table; run those LLE).
  if(lumiverseAsyncAudioEnabled() && taskType == 2 && signature
  && !signature->bannerIsGfx && !signature->asyncExcluded) {
    lumiverseAsyncRequestStart();
  }
  return false;
}
