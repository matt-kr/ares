auto AI::readWord(u32 address, Thread& thread) -> u32 {
  address = (address & 0x1f) >> 2;
  n32 data;

  if(address != 3) {
    //AI_LENGTH (mirrored)
    data.bit(0,17) = io.dmaLength[0];
  }

  if(address == 3) {
    //AI_STATUS
    data.bit( 0) = io.dmaCount > 1;
    data.bit(20) = 1;
    data.bit(24) = 1;
    data.bit(25) = io.dmaEnable;
    data.bit(30) = io.dmaCount > 0;
    data.bit(31) = io.dmaCount > 1;
    cpu.forceSynchronize();
  }

  debugger.io(Read, address, data);
  return data;
}

auto AI::writeWord(u32 address, u32 data_, Thread& thread) -> void {
  address = (address & 0x1f) >> 2;
  n32 data = data_;

  if(address == 0) {
    //AI_DRAM_ADDRESS
    if(io.dmaCount < 2) {
      io.dmaAddress[io.dmaCount] = data.bit(0,23) & ~7;
    }
  }

  if(address == 1) {
    //AI_LENGTH
    n18 length = data.bit(0,17) & ~7;
    if(io.dmaCount < 2) {
      if(io.dmaCount == 0) mi.raise(MI::IRQ::AI);
      io.dmaLength[io.dmaCount] = length;
      io.dmaOriginPc[io.dmaCount] = cpu.ipu.pc;
      //Lumiverse addition: LUMIVERSE_ARES_N64_AI_DUMP=<path> appends every
      //submitted AI DMA buffer ([addr u32][len u32][payload], big-endian)
      //at enqueue time. Used to validate serial-vs-async audio execution
      //byte-exactly, independent of playback timing. Default off.
      static FILE* lumiverseAIDump = []() -> FILE* {
        const char* path = ::getenv("LUMIVERSE_ARES_N64_AI_DUMP");
        return path && path[0] ? ::fopen(path, "wb") : nullptr;
      }();
      if(lumiverseAIDump) {
        const u32 dmaAddress = io.dmaAddress[io.dmaCount];
        const u32 dmaLength = length;
        auto put32 = [&](u32 value) {
          u8 bytes[4] = {u8(value >> 24), u8(value >> 16), u8(value >> 8), u8(value)};
          ::fwrite(bytes, 1, 4, lumiverseAIDump);
        };
        put32(dmaAddress);
        put32(dmaLength);
        for(u32 offset = 0; offset < dmaLength; offset += 8) {
          const u64 value = rdram.ram.read<Dual>(dmaAddress + offset, RBusDevice::AI_DMA);
          u8 bytes[8] = {
            u8(value >> 56), u8(value >> 48), u8(value >> 40), u8(value >> 32),
            u8(value >> 24), u8(value >> 16), u8(value >>  8), u8(value >>  0),
          };
          ::fwrite(bytes, 1, 8, lumiverseAIDump);
        }
        ::fflush(lumiverseAIDump);
      }
      io.dmaCount++;
    }
  }

  if(address == 2) {
    //AI_CONTROL
    io.dmaEnable = data.bit(0);
  }

  if(address == 3) {
    //AI_STATUS
    mi.lower(MI::IRQ::AI);
  }

  if(address == 4) {
    //AI_DACRATE
    auto frequency = dac.frequency;
    io.dacRate = data.bit(0,13);
    dac.frequency = max(1, system.videoFrequency() / (io.dacRate + 1));
    dac.period = system.frequency() / dac.frequency;
    if(frequency != dac.frequency) {
      stream->setFrequency(dac.frequency);
      updateDecay();
      //Lumiverse diagnostic: the game derives dacRate from the video clock
      //it believes it runs on (osTvType), so this line exposes a PAL ROM
      //booting in NTSC mode (dacRate ~1520 for 32 kHz) vs PAL (~1550), and
      //VI_V_SYNC shows the field length it programmed (525 vs 625 half-lines).
      static u32 logged = 0;
      if(logged++ < 4) {
        fprintf(stderr, "[ai] dacRate=%u -> %u Hz (video clock %u, VI halfLinesPerField=%u quarterLine=%u)\n",
          (u32)io.dacRate, dac.frequency, system.videoFrequency(),
          (u32)vi.io.halfLinesPerField, (u32)vi.io.quarterLineDuration);
      }
    }
  }

  if(address == 5) {
    //AI_BITRATE
    io.bitRate = data.bit(0,3);
    dac.precision = io.bitRate + 1;
  }

  debugger.io(Write, address, data);
}
