//Lumiverse addition: LUMIVERSE_ARES_N64_RELAX_IO_SYNC=1 skips the
//cpu.forceSynchronize() on DPC register READS (mirrors the SP status-read
//relaxation in rsp/io.cpp; see rsp/lumiverse-hle.cpp for rationale).
//Default off = stock behavior.
namespace {
auto lumiverseRelaxDPCReadSync() -> int {
  static int relax = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_RELAX_IO_SYNC");
    return value ? ::atoi(value) : 0;
  }();
  return relax;
}
}

//Lumiverse diagnostic (LUMIVERSE_ARES_N64_IO_POLL_LOG=1): count CPU reads of
//the DPC registers (and SP_STATUS in rsp/io.cpp) and print the histogram with
//the last value seen every 2^16 reads — shows what a stalled game is polling
//LUMIVERSE_ARES_N64_RSP_HLE_DPC_LOG_LIMIT: how many [rdp-dpc] lines each of
//the two DPC logs prints (default 64; raise to trace a hang)
auto lumiverseDpcLogLimit() -> u32 {
  static const u32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_DPC_LOG_LIMIT"); return v ? (u32)::strtoul(v, nullptr, 10) : 64u; }();
  return value;
}

auto lumiverseIOPollLog() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_IO_POLL_LOG"); return v && v[0] == '1'; }();
  return value;
}
u64 lumiverseIOPollCounts[16];
u32 lumiverseIOPollLast[16];
u64 lumiverseIOPollTotal = 0;
auto lumiverseIOPollNote(u32 slot, u32 value) -> void {
  lumiverseIOPollCounts[slot]++;
  lumiverseIOPollLast[slot] = value;
  if((++lumiverseIOPollTotal & 0xffff) == 0) {
    static const char* names[16] = {"DPC_START","DPC_END","DPC_CURRENT","DPC_STATUS","DPC_CLOCK","DPC_BUSY","DPC_PIPE","DPC_TMEM",
      "SP_STATUS","SP_DMA_FULL","SP_DMA_BUSY","SP_SEMAPHORE","cpu:DMEM","cpu:IMEM","rsp:SP_STATUS","rsp:SP_other/DPC"};
    fprintf(stderr, "[io-poll] cpu count=%llu:", (unsigned long long)cpu.scc.count);
    for(u32 i = 0; i < 16; i++) if(lumiverseIOPollCounts[i]) fprintf(stderr, " %s=%llu(last %08x)", names[i], (unsigned long long)lumiverseIOPollCounts[i], lumiverseIOPollLast[i]);
    fprintf(stderr, "\n");
    for(auto& c : lumiverseIOPollCounts) c = 0;
  }
}

auto RDP::readWord(u32 address, Thread& thread) -> u32 {
  address = (address & 0x1f) >> 2;
  n32 data;

  const bool syncOnRead = &thread == &cpu && !lumiverseRelaxDPCReadSync();

  if(address == 0) {
    //DPC_START
    data.bit(0,23) = command.start;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(address == 1) {
    //DPC_END
    data.bit(0,23) = command.end;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(address == 2) {
    //DPC_CURRENT
    data.bit(0,23) = command.current;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(address == 3) {
    //DPC_STATUS
    data.bit( 0) = command.source;
    data.bit( 1) = command.freeze || command.crashed;
    data.bit( 2) = command.flush;
    data.bit( 3) = command.startGclk;
    data.bit( 4) = command.tmemBusy > 0;
    data.bit( 5) = command.pipeBusy > 0;
    data.bit( 6) = command.bufferBusy > 0;
    data.bit( 7) = command.ready;
    data.bit( 8) = 0;  //DMA busy
    data.bit( 9) = command.endValid;
    data.bit(10) = command.startValid;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(address == 4) {
    //DPC_CLOCK
    data.bit(0,23) = command.clock - (Thread::clock - thread.clock) / 3;
    if(!lumiverseRelaxDPCReadSync()) cpu.forceSynchronize();
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(address == 5) {
    //DPC_BUSY
    data.bit(0,23) = command.bufferBusy;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(address == 6) {
    //DPC_PIPE_BUSY
    data.bit(0,23) = command.pipeBusy;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(data == 7) {
    //DPC_TMEM_BUSY
    data.bit(0,23) = command.tmemBusy;
    if(syncOnRead) cpu.forceSynchronize();
  }

  if(lumiverseIOPollLog()) lumiverseIOPollNote(&thread == &cpu ? (address & 7) : 15, data);
  debugger.ioDPC(Read, address, data);
  return data;
}

auto RDP::writeWord(u32 address, u32 data_, Thread& thread) -> void {
  address = (address & 0x1f) >> 2;
  n32 data = data_;

  if(address == 0) {
    //DPC_START
    if(!command.startValid) command.start = data.bit(0,23) & ~7;
    command.startValid = 1;
  }

  if(address == 1) {
    //DPC_END
    //Lumiverse diagnostic: log the first DP kicks and their origin (CPU vs
    //RSP) to understand per-game RDP submission (enable with
    //LUMIVERSE_ARES_N64_RSP_HLE_DPC_LOG=1).
    static int dpcLog = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_DPC_LOG");
      return value ? ::atoi(value) : 0;
    }();
    if(dpcLog >= 1) {
      static u32 logged = 0;
      if(logged++ < lumiverseDpcLogLimit()) {
        fprintf(stderr, "[rdp-dpc] end=%06x start=%06x startValid=%u origin=%s freeze=%u cur=%06x\n",
          (u32)data.bit(0,23), command.start, (u32)command.startValid, &thread == &cpu ? "cpu" : "rsp", (u32)command.freeze, command.current);
      }
    }
    command.end = data.bit(0,23) & ~7;
    if(command.startValid) {
      command.current = command.start;
      command.startValid = 0;
    }
    flushCommands();
    if(&thread == &cpu) cpu.forceSynchronize();
  }

  if(address == 2) {
    //DPC_CURRENT (read-only)
  }

  if(address == 3) {
    //DPC_STATUS
    //Lumiverse diagnostic (LUMIVERSE_ARES_N64_RSP_HLE_DPC_LOG=1): who writes
    //which status bits (the fifo microcode's freeze/xbus handshake)
    static int dpcStatusLog = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_DPC_LOG"); return v ? ::atoi(v) : 0; }();
    if(dpcStatusLog >= 1) {
      static u32 logged = 0;
      if(logged++ < lumiverseDpcLogLimit()) fprintf(stderr, "[rdp-dpc] STATUS write %08x origin=%s (freeze=%u source=%u start=%06x end=%06x current=%06x)\n",
        (u32)data, &thread == &cpu ? "cpu" : "rsp", (u32)command.freeze, (u32)command.source, command.start, command.end, command.current);
    }
    if(data.bit(0)) command.source = 0;
    if(data.bit(1)) command.source = 1;
    if(data.bit(2)) command.freeze = 0, flushCommands();
    if(data.bit(3)) command.freeze = 1;
    if(data.bit(4)) command.flush = 0;
    if(data.bit(5)) command.flush = 1;
    if(data.bit(6) && !command.crashed) command.tmemBusy = 0;
    if(data.bit(7) && !command.crashed) command.pipeBusy = 0;
    if(data.bit(8) && !command.crashed) command.bufferBusy = 0;
    if(data.bit(9)) command.clock = (Thread::clock - thread.clock) / 3;
  }

  if(address == 4) {
    //DPC_CLOCK (read-only)
  }

  if(address == 5) {
    //DPC_BUSY (read-only)
  }

  if(address == 6) {
    //DPC_PIPE_BUSY (read-only)
  }

  if(address == 7) {
    //DPC_TMEM_BUSY (read-only)
  }

  debugger.ioDPC(Write, address, data);
}

auto RDP::IO::readWord(u32 address, Thread& thread) -> u32 {
  address = (address & 0xfffff) >> 2;
  n32 data;

  if(address == 0) {
    //DPS_TBIST
    data.bit(0)    = bist.check;
    data.bit(1)    = bist.go;
    data.bit(2)    = bist.done;
    data.bit(3,10) = bist.fail;
  }

  if(address == 1) {
    //DPS_TEST_MODE
    data.bit(0) = test.enable;
  }

  if(address == 2) {
    //DPS_BUFTEST_ADDR
    data.bit(0,6) = test.address;
  }

  if(address == 3) {
    //DPS_BUFTEST_DATA
    data.bit(0,31) = test.data[test.address];
  }

  self.debugger.ioDPS(Read, address, data);
  return data;
}

auto RDP::IO::writeWord(u32 address, u32 data_, Thread& thread) -> void {
  address = (address & 0xfffff) >> 2;
  n32 data = data_;

  if(address == 0) {
    //DPS_TBIST
    bist.check = data.bit(0);
    bist.go    = data.bit(1);
    if(data.bit(2)) bist.done = 0;
  }

  if(address == 1) {
    //DPS_TEST_MODE
    test.enable = data.bit(0);
  }

  if(address == 2) {
    //DPS_BUFTEST_ADDR
    test.address = data.bit(0,6);
  }

  if(address == 3) {
    //DPS_BUFTEST_DATA
    u32 value = data.bit(0,31);
    u32 column = test.address % 4;
    
    if(column == 2) {
      value &= 0xFF;
    } else if(column == 3) {
      value = 0;
    }

    test.data[test.address] = value;
  }

  self.debugger.ioDPS(Write, address, data);
}

auto RDP::flushCommands() -> void {
  if(command.freeze || command.crashed) return;
  command.bufferBusy = 1;
  command.pipeBusy = 1;
  command.startGclk = 1;
  //Lumiverse diagnostic: LUMIVERSE_ARES_N64_RDP_STREAM_DUMP=<path> appends
  //every consumed RDP command range as text words (source-aware) before the
  //renderer eats it. Empirical analysis only; default off.
  static FILE* lumiverseStreamDump = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_RDP_STREAM_DUMP");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  if(lumiverseStreamDump && command.end > command.current) {
    auto& memory = !command.source ? (Memory::Writable&)rdram.ram : (Memory::Writable&)rsp.dmem;
    fprintf(lumiverseStreamDump, "kick src=%s cur=%06x end=%06x\n",
      command.source ? "xbus" : "rdram", (u32)command.current, (u32)command.end);
    static const u32 dumpBytes = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RDP_STREAM_DUMP_BYTES"); return v ? (u32)::strtoul(v, nullptr, 0) : 0x4000u; }();
    for(u32 address = command.current; address + 8 <= command.end && address < command.current + dumpBytes; address += 8) {
      fprintf(lumiverseStreamDump, "  %08x %08x\n",
        memory.readUnaligned<Word>(address), memory.readUnaligned<Word>(address + 4));
    }
    fflush(lumiverseStreamDump);
  }
  if(command.end > command.current) render();
  command.ready = 1;
}
