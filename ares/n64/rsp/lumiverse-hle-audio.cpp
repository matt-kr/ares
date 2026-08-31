//Lumiverse addition: native (HLE) execution of RSP audio tasks.
//
//Parses the audio command list (alist) of recognized ABI2-family audio
//microcodes and synthesizes the output samples in C++, writing them to the
//RDRAM buffers the commands specify. The task then completes instantly
//(BREAK + SIG2 + SP IRQ applied by io.cpp), removing all LLE RSP audio
//execution from the emulation thread. Unrecognized microcodes, and any task
//containing a command this interpreter does not fully understand, return
//false from a side-effect-free validation pass and run on the LLE core
//unchanged.
//
//Quality bar (per project direction): "sounds correct", NOT bit-exact.
//ADPCM decode is integer-exact by construction; resampling is linear
//interpolation (the microcode uses a small windowed FIR); envelope ramps and
//mixing use the empirically-derived integer semantics below.
//
//Clean-room provenance (see References/rsp-hle/PROVENANCE.md):
//  - EMPIRICAL observation of our own emulator only: every command encoding
//    below was derived from raw alist dumps captured via
//    LUMIVERSE_ARES_N64_RSP_HLE_ALIST_DUMP (lumiverse-hle.cpp) from
//    Star Fox 64 (U) and Zelda OoT MQ (E), cross-checked against the
//    RDRAM outputs of our LLE core via the shadow-compare mode in this file
//    (LUMIVERSE_ARES_N64_AUDIO_HLE=2). Buffer/loop/envelope semantics were
//    inferred from those dumps (e.g. envelope volume timelines across tasks
//    proved the rate is applied once per 8-sample vector).
//  - VADPCM frame/predictor structure (9-byte frames, 4-bit residuals,
//    order-2 predictor books at 2x8 s16 per predictor, Q11 coefficients):
//    public N64 homebrew documentation (n64brew wiki / libdragon's audio
//    tooling docs, Unlicense). The matrix-form expansion used here follows
//    from the order-2 recursion by first principles.
//  - Ultra64 SDK man-page knowledge (spec only): general alist architecture
//    (aSetBuffer supplying in/out/count to the following command, A_INIT/
//    A_LOOP flag conventions, segment-free physical addressing).
//  No GPL or Nintendo-derived source was consulted. The ucode binaries were
//  never disassembled; identification is by FNV-1a hash of the microcode
//  text, captured by our census.
//
//Command set (opcode -> semantics, empirically verified):
//  0x00 NOOP
//  0x01 ADPCM     flags=cmd0.b2 (1=init, 2=loop, 4=2-bit residual frames),
//                 state addr=cmd1; in/out/count from SETBUFF. The 16 history
//                 samples are written as a prefix at `out`, decoded data at
//                 out+0x20; whole frames are always decoded (count rounds
//                 the write up to a frame) but the saved state ends at the
//                 exact count offset. Frames: 9 bytes/16 samples (4-bit) or
//                 5 bytes/16 samples (2-bit, Majora's Mask ambience)
//  0x02 CLEARBUFF dmem=cmd0.lo16, count=cmd1
//  0x04 MIXER     (SF64) count=cmd0.b2 x16 bytes, gain=s16 cmd0.lo16,
//                 in=cmd1.hi16, out=cmd1.lo16; out += in*gain>>15 (saturated)
//  0x05 RESAMPLE  flags=cmd0.b2 (1=init), pitch=u16 cmd0.lo16 (Q15),
//                 state addr=cmd1; in/out/count from SETBUFF (count=out
//                 bytes). Interpolation window starts two input samples
//                 before `in` (phase verified against isolated LLE runs)
//  0x07 FILTER    (Zeldas) flags=cmd0.b2: 2=set (length=cmd0.lo16, coef
//                 table addr=cmd1: 8 s16 taps); 0/1=run in-place on dmem
//                 buffer cmd0.lo16, state addr=cmd1 (1=init). Applied taps =
//                 (table + state kernel)/2, causal (identity table = 3-sample
//                 delay at half gain on the first run); the state kernel
//                 converges to the table at ~15/16 per run (crossfade)
//  0x08 SETBUFF   in=cmd0.lo16, out=cmd1.hi16, count=cmd1.lo16
//  0x0a DMEMMOVE  in=cmd0.lo16, out=cmd1.hi16, count=cmd1.lo16 rounded UP
//                 to a 16-byte vector multiple
//  0x0b LOADADPCM count bytes=cmd0.lo16, book addr=cmd1
//                 (predictor book: order 2, per predictor 2 rows x 8 s16,
//                 row1 doubling as the recursion's impulse response)
//  0x0c MIXER     same encoding/semantics as 0x04
//  0x0d INTERLEAVE l=cmd1.hi16, r=cmd1.lo16; out/count(bytes per channel)
//                 embedded in cmd0 (count=bits12-23, out=bits0-11) when
//                 nonzero (Zeldas), else from SETBUFF (SF64)
//  0x0f SETLOOP   loop state addr=cmd1 (16 s16 decoded samples at loop point)
//  0x11 COPY      (SF64) count samples=cmd0.lo16, src=cmd1.hi16, dst=cmd1.lo16
//  0x12 ENVSETUP1 wet send=cmd0.b2 (Q8), ramp rates=cmd1 (s16 L,R; applied
//                 to the volume once per 8-sample vector)
//  0x13 ENVMIX    in=cmd0.b2<<4, count samples=cmd0.b1, cmd1 = four output
//                 buses (dryL,dryR,wetL,wetR as addr>>4 bytes);
//                 dry += in*vol>>16 (vol Q16), wet += dry_send*wet>>8,
//                 saturated. cmd0 low-byte bits 0x10/0x01 appear rarely and
//                 are ignored (semantics unknown; see report)
//  0x14 LOADBUFF  count=cmd0 bits12-23, dmem=cmd0 bits0-11, RDRAM addr=cmd1
//  0x15 SAVEBUFF  same encoding as LOADBUFF, direction reversed
//  0x16 ENVSETUP2 current volumes=cmd1 (u16 L,R, Q16)
//  0x1a DUPLICATE (SF64) tile count(cmd1.lo16) bytes at src(cmd0.lo16) into
//                 flags(cmd0.b2) consecutive copies at dst(cmd1.hi16) —
//                 extends short looped waveforms for the resampler
//
//DIALECTS: the ABI2 command set above covers the SF64/Zelda family. The
//older ABI1 family (SM64 / Wave Race 64 / Pokemon Snap, SETBUFF-driven with
//a segment table) is implemented as a second dialect: LOADBUFF(04)/
//SAVEBUFF(06) move via SETBUFF in/out/count, SEGMENT(07) fills the address
//table, SETVOL(09) uses the A_VOL(4)|A_LEFT(2)/A_AUX(8) flag map, ENVMIXER
//(03) mixes into SETBUFF.out + the three A_AUX buffers with Q16-fractional
//per-sample exponential volume ramps (rate = Q16 multiplier per 8-sample
//group, applied per sample at rate^(1/8), clamped at target) whose
//parameters persist in the per-voice state (CONTINUE chunks carry no
//SETVOLs), and POLEF(0e) is a one-pole IIR whose pole comes from the
//LOADADPCM table (row1[0], Q14) with a Q14 input gain. All fitted against
//isolated LLE runs via the truncated-task oracle.
//
//A third family ("naudio": Super Smash Bros. / Kirby) is only partially
//decoded (fixed 0x170-byte chunks, state addresses in cmd0.lo24, an
//80+ byte per-voice RDRAM parameter block holding per-lane vol int/frac
//vectors, targets, 16.16 rates and wet/dry gains, envmix output at fixed
//dmem 0x9d0/0xb40/0xcb0/0xe20). It stays OFF the whitelist; the
//LUMIVERSE_ARES_N64_AUDIO_HLE_NAUDIO=1 gate enables the experimental
//skeleton for further oracle work only.
//
//This file is included from rsp.cpp inside namespace ares::Nintendo64 (same
//translation unit as lumiverse-hle.cpp); no #includes allowed here.

namespace {

//LUMIVERSE_ARES_N64_AUDIO_HLE:
//  unset/0 = off (default; stock behavior)
//  1       = execute whitelisted audio tasks natively
//  2       = shadow mode (validation): run the interpreter but buffer its
//            RDRAM writes, let LLE execute the task, and byte-compare the
//            buffered writes against LLE's actual output at the next audio
//            dispatch. Reports per-write differences. Audio still comes from
//            LLE in this mode.
auto lumiverseAudioHLELevel() -> int {
  static int level = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE");
    return value ? ::atoi(value) : 0;
  }();
  return level;
}

auto lumiverseAudioHLEDebug() -> int {
  static int debug = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_DEBUG");
    return value ? ::atoi(value) : 0;
  }();
  return debug;
}

//command-set dialects (analogous to the gfx interpreter's GBI0/1/2)
enum : u32 {
  LumiverseAudioDialectABI2 = 0,   //SF64 / Zelda family (validated first)
  LumiverseAudioDialectABI1 = 1,   //SM64 / Wave Race / Pokemon Snap family
  LumiverseAudioDialectNaudio = 2, //Smash Bros / Kirby family (fixed layout)
};

//whitelisted audio microcode hashes (FNV-1a over the ucode text, captured by
//the census in lumiverse-hle.cpp)
constexpr u64 LumiverseAudioUcodeStarFox64 = 0xc1a98f5d15c322c7ull;  //Star Fox 64 (U)
constexpr u64 LumiverseAudioUcodeZeldaMQ   = 0x9d3b431ff4357876ull;  //Zelda OoT Master Quest (E)
constexpr u64 LumiverseAudioUcodeZeldaOoTU = 0xca93aeebeae5d62full;  //Zelda OoT (U)
constexpr u64 LumiverseAudioUcodeMajoraU   = 0xa8df9aeb4a7cb685ull;  //Majora's Mask (U)

constexpr u64 LumiverseAudioUcodeSM64WR    = 0x394bf43d72dfa31dull;  //Super Mario 64 (U) + Wave Race 64 (U), shared
constexpr u64 LumiverseAudioUcodeSnapU     = 0x7123e4d5f82ae6a5ull;  //Pokemon Snap (U)
constexpr u64 LumiverseAudioUcodeSmashU    = 0xbbca23e37b0bc136ull;  //Super Smash Bros. (U) (+ Kirby 64 family)

//LUMIVERSE_ARES_N64_AUDIO_HLE_NAUDIO=1 enables the still-experimental naudio
//dialect (default off until validated)
auto lumiverseAudioNaudioEnabled() -> bool {
  static bool enabled = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NAUDIO");
    return value && value[0] == '1';
  }();
  return enabled;
}

//returns the command-set dialect for a whitelisted audio ucode, -1 otherwise
auto lumiverseAudioDialectForHash(u64 hash) -> s32 {
  if(hash == LumiverseAudioUcodeStarFox64) return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeZeldaMQ)   return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeZeldaOoTU) return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeMajoraU)   return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeSM64WR)    return LumiverseAudioDialectABI1;
  if(hash == LumiverseAudioUcodeSnapU)     return LumiverseAudioDialectABI1;
  if(hash == LumiverseAudioUcodeSmashU && lumiverseAudioNaudioEnabled()) return LumiverseAudioDialectNaudio;
  return -1;
}

//----------------------------------------------------------------------------
//RDRAM write redirection (shadow mode buffers writes instead of applying)
//----------------------------------------------------------------------------

struct LumiverseAudioShadow {
  static constexpr u32 MaxWrites = 512;
  static constexpr u32 MaxBytes  = 262144;
  struct Write { u32 addr; u32 length; u32 offset; u32 command; };
  Write writes[MaxWrites];
  u8 data[MaxBytes];
  u32 writeCount = 0;
  u32 byteCount = 0;
  bool overflow = false;
  bool pending = false;   //a compare against LLE output is outstanding
  u64 task = 0;
  bool active = false;    //currently buffering (shadow mode execution)
  u32 currentCommand = 0; //alist command index for diagnostics
};

auto lumiverseAudioRDRAMReadByte(u32 address) -> u8 {
  return rdram.ram.Memory::Writable::read<Byte>(address & 0x00ffffff);
}

auto lumiverseAudioReadRDRAM(LumiverseAudioShadow& shadow, u32 address, u8* out, u32 length) -> void {
  address &= 0x00ffffff;
  for(u32 index = 0; index < length; index++) out[index] = lumiverseAudioRDRAMReadByte(address + index);
  if(!shadow.active) return;
  //shadow mode: overlay this task's own pending writes (last-write-wins) so
  //save-then-reload sequences within a task see their own data
  for(u32 w = 0; w < shadow.writeCount; w++) {
    const auto& write = shadow.writes[w];
    const u32 begin = write.addr > address ? write.addr : address;
    const u32 end = (write.addr + write.length) < (address + length) ? (write.addr + write.length) : (address + length);
    for(u32 a = begin; a < end; a++) out[a - address] = shadow.data[write.offset + (a - write.addr)];
  }
}

auto lumiverseAudioWriteRDRAM(LumiverseAudioShadow& shadow, u32 address, const u8* src, u32 length) -> void {
  address &= 0x00ffffff;
  if(!shadow.active) {
    for(u32 index = 0; index < length; index++) {
      rdram.ram.Memory::Writable::write<Byte>((address + index) & 0x00ffffff, src[index]);
    }
    return;
  }
  if(shadow.writeCount >= LumiverseAudioShadow::MaxWrites
  || shadow.byteCount + length > LumiverseAudioShadow::MaxBytes) {
    shadow.overflow = true;
    return;
  }
  auto& write = shadow.writes[shadow.writeCount++];
  write.addr = address;
  write.length = length;
  write.offset = shadow.byteCount;
  write.command = shadow.currentCommand;
  for(u32 index = 0; index < length; index++) shadow.data[shadow.byteCount + index] = src[index];
  shadow.byteCount += length;
}

//----------------------------------------------------------------------------
//private per-voice state table (ADPCM history, resampler position, filter
//taps). Keyed by the RDRAM state address each command supplies. The LLE
//microcode keeps this state in RDRAM in its own layout; since HLE executes
//every task for a whitelisted ucode, the layout can be ours. (If a task ever
//falls back to LLE mid-stream the voice states diverge briefly; envelopes
//and A_INIT flags re-seed them within a few frames.)
//----------------------------------------------------------------------------

struct LumiverseAudioVoiceState {
  u32 addr = 0;      //0 = free slot
  s16 samples[16] = {};
  u32 frac = 0;      //resampler fractional position (Q16 fraction bits)
};

constexpr u32 LumiverseAudioStateSlots = 1024;  //power of two

auto lumiverseAudioState(u32 addr) -> LumiverseAudioVoiceState& {
  static LumiverseAudioVoiceState slots[LumiverseAudioStateSlots];
  addr &= 0x00ffffff;
  u32 index = (addr * 2654435761u) >> 22 & (LumiverseAudioStateSlots - 1);
  for(u32 probe = 0; probe < 8; probe++) {
    auto& slot = slots[index];
    if(slot.addr == addr) return slot;
    if(slot.addr == 0) { slot.addr = addr; return slot; }
    index = index + 1 & (LumiverseAudioStateSlots - 1);
  }
  //table pressure: recycle the probed slot (worst case a one-chunk glitch on
  //one voice; never observed with 1024 slots)
  auto& slot = slots[index];
  slot.addr = addr;
  for(auto& sample : slot.samples) sample = 0;
  slot.frac = 0;
  return slot;
}

//----------------------------------------------------------------------------
//interpreter state
//----------------------------------------------------------------------------

struct LumiverseAudioMachine {
  u8 dmem[4096] = {};      //virtual DMEM workspace (persists across tasks)
  s16 book[256] = {};      //ADPCM predictor book (max 8 predictors)
  u32 bookEntries = 0;
  u32 loopAddr = 0;
  //SETBUFF
  u32 inBuf = 0, outBuf = 0, bufCount = 0;
  //envelope (ABI2 style)
  u32 volL = 0, volR = 0;
  s32 rateL = 0, rateR = 0;
  u32 wetGain = 0;
  //filter (ABI2 Zeldas)
  u32 filterLength = 0;
  u32 filterCoefAddr = 0;
  //ABI1 state
  u32 segments[16] = {};
  u32 aux1 = 0, aux2 = 0, aux3 = 0;         //A_AUX SETBUFF: dryR, wetL, wetR
  s32 setVolL = 0, setVolR = 0;             //aSetVolume A_VOL left/right
  s32 setTargetL = 0, setTargetR = 0;       //aSetVolume A_RATE targets
  u32 setRateL = 0x10000, setRateR = 0x10000;  //Q16 per-8-sample multipliers
  s32 setDryVol = 0, setWetVol = 0;         //aSetVolume A_AUX
  //statistics
  u64 tasksExecuted = 0;
  u64 tasksFallback = 0;
  u64 unknownEnvmixFlags = 0;
};

//ABI1 RDRAM addresses go through a segment table (aSegment); ABI2 uses
//physical addresses directly
auto lumiverseAudioResolve(const LumiverseAudioMachine& m, u32 dialect, u32 raw) -> u32 {
  if(dialect == LumiverseAudioDialectABI1) {
    return (m.segments[raw >> 24 & 0xf] + (raw & 0x00ffffff)) & 0x00ffffff;
  }
  return raw & 0x00ffffff;
}

auto lumiverseAudioClamp16(s32 value) -> s16 {
  if(value > 32767) return 32767;
  if(value < -32768) return -32768;
  return (s16)value;
}

auto lumiverseAudioDmemReadS16(const u8* dmem, u32 offset) -> s16 {
  return (s16)((u16)dmem[offset & 0xfff] << 8 | dmem[(offset + 1) & 0xfff]);
}

auto lumiverseAudioDmemWriteS16(u8* dmem, u32 offset, s16 value) -> void {
  dmem[offset & 0xfff] = (u16)value >> 8;
  dmem[(offset + 1) & 0xfff] = (u8)value;
}

//----------------------------------------------------------------------------
//validation pass: walk the alist with a copy of the scalar state and verify
//every command is fully understood and in bounds. No side effects, so an
//unsupported task falls back to LLE with zero corruption.
//----------------------------------------------------------------------------

struct LumiverseAudioValidator {
  u32 bookEntries;
  u32 loopAddr;
  u32 inBuf, outBuf, bufCount;
  u32 filterLength;
  u32 filterCoefAddr;
  u32 aux1, aux2, aux3;
};

auto lumiverseAudioValidate(const LumiverseAudioMachine& machine, u32 dataPtr, u32 dataSize, u32 dialect) -> bool {
  LumiverseAudioValidator v;
  v.bookEntries = machine.bookEntries;
  v.loopAddr = machine.loopAddr;
  v.inBuf = machine.inBuf;
  v.outBuf = machine.outBuf;
  v.bufCount = machine.bufCount;
  v.filterLength = machine.filterLength;
  v.filterCoefAddr = machine.filterCoefAddr;
  v.aux1 = machine.aux1;
  v.aux2 = machine.aux2;
  v.aux3 = machine.aux3;
  const bool abi1 = dialect == LumiverseAudioDialectABI1;
  if(dialect == LumiverseAudioDialectNaudio) return true;  //EXPERIMENT: permissive while decoding

  static u64 rejectLogs = 0;
  auto reject = [&](u32 offset, u32 w0, u32 w1, const char* reason) -> bool {
    if(rejectLogs < 32) {
      rejectLogs++;
      fprintf(stderr, "[rsp-hle-audio] fallback: cmd %08x %08x at +%u: %s\n", w0, w1, offset, reason);
    }
    return false;
  };

  for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
    u32 w0 = 0, w1 = 0;
    for(u32 b = 0; b < 4; b++) w0 = w0 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + b);
    for(u32 b = 0; b < 4; b++) w1 = w1 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + 4 + b);
    const u8 op = w0 >> 24;
    const u32 flags = w0 >> 16 & 0xff;

    switch(op) {
    case 0x00:  //NOOP
      break;
    case 0x01: {  //ADPCM
      if(flags & ~7u) return reject(offset, w0, w1, "adpcm flags");
      if((flags & 2) && !v.loopAddr) return reject(offset, w0, w1, "adpcm loop without setloop");
      if(!v.bookEntries && v.bufCount) return reject(offset, w0, w1, "adpcm without book");
      if(v.bufCount & 1) return reject(offset, w0, w1, "adpcm count");
      const u32 frameBytes = (flags & 4) ? 5 : 9;  //bit 2 = 2-bit residuals
      if(v.outBuf + 32 + 32 * ((v.bufCount + 31) / 32) > 0x1000) return reject(offset, w0, w1, "adpcm out range");
      if(v.inBuf + frameBytes * ((v.bufCount + 31) / 32) > 0x1000) return reject(offset, w0, w1, "adpcm in range");
      if(!(w1 & 0x00ffffff)) return reject(offset, w0, w1, "adpcm state addr");
      break;
    }
    case 0x02: {  //CLEARBUFF
      const u32 dmemAddr = w0 & 0xffff, count = w1 & 0xffff;
      if(dmemAddr + count > 0x1000) return reject(offset, w0, w1, "clearbuff range");
      break;
    }
    case 0x04: {  //ABI2: MIXER; ABI1: LOADBUFF (uses SETBUFF in/count)
      if(abi1) {
        if(v.inBuf + v.bufCount > 0x1000) return reject(offset, w0, w1, "loadbuff range");
        break;
      }
      const u32 count = (w0 >> 16 & 0xff) << 4;
      const u32 in = w1 >> 16, out = w1 & 0xffff;
      if(!count) return reject(offset, w0, w1, "mixer count");
      if(in + count > 0x1000 || out + count > 0x1000) return reject(offset, w0, w1, "mixer range");
      break;
    }
    case 0x0c: {  //MIXER (ABI1: count from SETBUFF; ABI2: count embedded)
      const u32 count = abi1 ? v.bufCount : (w0 >> 16 & 0xff) << 4;
      const u32 in = w1 >> 16, out = w1 & 0xffff;
      if(!count) return reject(offset, w0, w1, "mixer count");
      if(in + count > 0x1000 || out + count > 0x1000) return reject(offset, w0, w1, "mixer range");
      break;
    }
    case 0x03: {  //ENVMIXER (ABI1)
      if(!abi1) return reject(offset, w0, w1, "envmixer in abi2");
      if(flags & ~9u) return reject(offset, w0, w1, "envmixer flags");
      const u32 count = v.bufCount;
      if(!count || (count & 1)) return reject(offset, w0, w1, "envmixer count");
      if(v.inBuf + count > 0x1000 || v.outBuf + count > 0x1000) return reject(offset, w0, w1, "envmixer dry range");
      if(v.aux1 + count > 0x1000) return reject(offset, w0, w1, "envmixer aux1 range");
      if((flags & 8) && (v.aux2 + count > 0x1000 || v.aux3 + count > 0x1000)) return reject(offset, w0, w1, "envmixer wet range");
      break;
    }
    case 0x06: {  //SAVEBUFF (ABI1; uses SETBUFF out/count)
      if(!abi1) return reject(offset, w0, w1, "savebuff in abi2");
      if(v.outBuf + v.bufCount > 0x1000) return reject(offset, w0, w1, "savebuff range");
      break;
    }
    case 0x09: {  //SETVOL (ABI1)
      if(!abi1) return reject(offset, w0, w1, "setvol in abi2");
      if(flags != 0 && flags != 2 && flags != 4 && flags != 6 && flags != 8) return reject(offset, w0, w1, "setvol flags");
      break;
    }
    case 0x0e: {  //POLEF (ABI1, Pokemon Snap)
      if(!abi1) return reject(offset, w0, w1, "polef in abi2");
      if(flags & ~1u) return reject(offset, w0, w1, "polef flags");
      if(!v.bookEntries) return reject(offset, w0, w1, "polef without table");
      if(v.inBuf + v.bufCount > 0x1000 || v.outBuf + v.bufCount > 0x1000) return reject(offset, w0, w1, "polef range");
      break;
    }
    case 0x05: {  //RESAMPLE
      if(flags & ~1u) return reject(offset, w0, w1, "resample flags");
      if(!v.bufCount || v.bufCount & 1) return reject(offset, w0, w1, "resample count");
      if(v.outBuf + v.bufCount > 0x1000) return reject(offset, w0, w1, "resample out range");
      if(v.inBuf & 1) return reject(offset, w0, w1, "resample in align");
      const u32 pitch = w0 & 0xffff;
      const u32 consumed = ((u64)(v.bufCount / 2) * (pitch << 1) + 0x1ffff) >> 16;
      if(v.inBuf + 2 * consumed + 4 > 0x1000) return reject(offset, w0, w1, "resample in range");
      if(!(w1 & 0x00ffffff)) return reject(offset, w0, w1, "resample state addr");
      break;
    }
    case 0x07: {  //ABI1: SEGMENT; ABI2 Zeldas: FILTER
      if(abi1) break;
      if(flags == 2) {
        const u32 count = w0 & 0xffff;
        if(!count || (count & 1) || count > 0x1000) return reject(offset, w0, w1, "filter set count");
        if(!(w1 & 0x00ffffff)) return reject(offset, w0, w1, "filter coef addr");
        v.filterLength = count;
        v.filterCoefAddr = w1 & 0x00ffffff;
      } else if(flags <= 1) {
        if(!v.filterLength || !v.filterCoefAddr) return reject(offset, w0, w1, "filter run before set");
        const u32 buffer = w0 & 0xffff;
        if(buffer + v.filterLength > 0x1000) return reject(offset, w0, w1, "filter range");
        if(!(w1 & 0x00ffffff)) return reject(offset, w0, w1, "filter state addr");
      } else {
        return reject(offset, w0, w1, "filter flags");
      }
      break;
    }
    case 0x08: {  //SETBUFF
      if(abi1 && flags == 8) {
        //A_AUX: three auxiliary buffers (dry R, wet L, wet R)
        v.aux1 = w0 & 0xffff;
        v.aux2 = w1 >> 16;
        v.aux3 = w1 & 0xffff;
        if(v.aux1 >= 0x1000 || v.aux2 >= 0x1000 || v.aux3 >= 0x1000) return reject(offset, w0, w1, "setbuff aux range");
        break;
      }
      if(flags) return reject(offset, w0, w1, "setbuff flags");
      v.inBuf = w0 & 0xffff;
      v.outBuf = w1 >> 16;
      v.bufCount = w1 & 0xffff;
      if(v.inBuf >= 0x1000 || v.outBuf >= 0x1000 || v.bufCount > 0x1000) return reject(offset, w0, w1, "setbuff range");
      break;
    }
    case 0x0a: {  //DMEMMOVE (count rounds up to 16)
      const u32 in = w0 & 0xffff, out = w1 >> 16;
      const u32 count = ((w1 & 0xffff) + 15) & ~15u;
      if(in + count > 0x1000 || out + count > 0x1000) return reject(offset, w0, w1, "dmemmove range");
      break;
    }
    case 0x0b: {  //LOADADPCM (count = bytes)
      const u32 count = w0 & 0xffff;
      if(!count || count % 32 || count > sizeof(LumiverseAudioMachine::book)) return reject(offset, w0, w1, "loadadpcm count");
      if(!(w1 & 0x00ffffff)) return reject(offset, w0, w1, "loadadpcm addr");
      v.bookEntries = count / 2;
      break;
    }
    case 0x0d: {  //INTERLEAVE
      u32 out, count;
      if(w0 & 0x00ffffff) { out = w0 & 0xfff; count = w0 >> 12 & 0xfff; }
      else { out = v.outBuf; count = v.bufCount; }
      const u32 l = w1 >> 16, r = w1 & 0xffff;
      if(!count || (count & 1)) return reject(offset, w0, w1, "interleave count");
      if(l + count > 0x1000 || r + count > 0x1000 || out + 2 * count > 0x1000) return reject(offset, w0, w1, "interleave range");
      break;
    }
    case 0x0f:  //SETLOOP
      if(!(w1 & 0x00ffffff)) return reject(offset, w0, w1, "setloop addr");
      v.loopAddr = w1 & 0x00ffffff;
      break;
    case 0x11: {  //COPY (SF64)
      const u32 count = (w0 & 0xffff) * 2;
      const u32 src = w1 >> 16, dst = w1 & 0xffff;
      if(!count || src + count > 0x1000 || dst + count > 0x1000) return reject(offset, w0, w1, "copy range");
      break;
    }
    case 0x12:  //ENVSETUP1
    case 0x16:  //ENVSETUP2
      if(abi1) return reject(offset, w0, w1, "envsetup in abi1");
      break;
    case 0x1a: {  //DUPLICATE
      const u32 copies = flags ? flags : 1;
      const u32 src = w0 & 0xffff;
      const u32 dst = w1 >> 16;
      const u32 count = w1 & 0xffff;
      if(!count || copies > 4) return reject(offset, w0, w1, "duplicate params");
      if(src + count > 0x1000 || dst + copies * count > 0x1000) return reject(offset, w0, w1, "duplicate range");
      break;
    }
    case 0x13: {  //ENVMIX
      const u32 in = (w0 >> 16 & 0xff) << 4;
      const u32 count = w0 >> 8 & 0xff;
      if(!count || count % 8) return reject(offset, w0, w1, "envmix count");
      if(in + 2 * count > 0x1000) return reject(offset, w0, w1, "envmix in range");
      const u32 buses[4] = { (w1 >> 24) << 4, (w1 >> 16 & 0xff) << 4, (w1 >> 8 & 0xff) << 4, (w1 & 0xff) << 4 };
      for(u32 bus : buses) if(bus + 2 * count > 0x1000) return reject(offset, w0, w1, "envmix bus range");
      break;
    }
    case 0x14:    //LOADBUFF
    case 0x15: {  //SAVEBUFF
      const u32 count = w0 >> 12 & 0xfff, dmemAddr = w0 & 0xfff;
      if(dmemAddr + count > 0x1000) return reject(offset, w0, w1, "load/savebuff range");
      if(count && !(w1 & 0x00ffffff)) return reject(offset, w0, w1, "load/savebuff addr");
      break;
    }
    default:
      return reject(offset, w0, w1, "unknown opcode");
    }
  }
  return true;
}

//----------------------------------------------------------------------------
//execution pass
//----------------------------------------------------------------------------

//naudio dialect (Smash Bros family): fixed 0x170-byte chunk layout, commands
//carry state addresses in cmd0.lo24 and dmem/count in cmd1. EXPERIMENTAL.
auto lumiverseAudioExecuteNaudio(LumiverseAudioMachine& m, LumiverseAudioShadow& shadow, u32 w0, u32 w1) -> void {
  u8* dmem = m.dmem;
  const u8 op = w0 >> 24;
  switch(op) {
  case 0x02: {  //CLEARBUFF
    const u32 dmemAddr = w0 & 0xffff, count = w1 & 0xffff;
    for(u32 index = 0; index < count && dmemAddr + index < 0x1000; index++) dmem[dmemAddr + index] = 0;
    break;
  }
  case 0x04: {  //LOADBUFF count=cmd0 bits12-23, dmem=cmd0 lo12, addr=cmd1
    const u32 count = w0 >> 12 & 0xfff, dmemAddr = w0 & 0xfff;
    if(!count) break;
    u8 scratch[0x1000];
    lumiverseAudioReadRDRAM(shadow, w1 & 0x00ffffff, scratch, count);
    for(u32 index = 0; index < count; index++) dmem[(dmemAddr + index) & 0xfff] = scratch[index];
    break;
  }
  case 0x06: {  //SAVEBUFF
    const u32 count = w0 >> 12 & 0xfff, dmemAddr = w0 & 0xfff;
    if(!count) break;
    u8 scratch[0x1000];
    for(u32 index = 0; index < count; index++) scratch[index] = dmem[(dmemAddr + index) & 0xfff];
    lumiverseAudioWriteRDRAM(shadow, w1 & 0x00ffffff, scratch, count);
    break;
  }
  case 0x0b: {  //LOADADPCM count=cmd0.lo16 bytes, addr=cmd1
    const u32 count = w0 & 0xffff;
    if(!count || count > sizeof(LumiverseAudioMachine::book)) break;
    u8 raw[sizeof(LumiverseAudioMachine::book)];
    lumiverseAudioReadRDRAM(shadow, w1 & 0x00ffffff, raw, count);
    for(u32 index = 0; index < count / 2; index++) {
      m.book[index] = (s16)((u16)raw[index * 2] << 8 | raw[index * 2 + 1]);
    }
    m.bookEntries = count / 2;
    break;
  }
  default:
    break;  //remaining ops decoded via oracle
  }
}

auto lumiverseAudioExecute(LumiverseAudioMachine& m, LumiverseAudioShadow& shadow, u32 dataPtr, u32 dataSize, u32 dialect) -> void {
  u8* dmem = m.dmem;

  for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
    shadow.currentCommand = offset / 8;
    u32 w0 = 0, w1 = 0;
    for(u32 b = 0; b < 4; b++) w0 = w0 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + b);
    for(u32 b = 0; b < 4; b++) w1 = w1 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + 4 + b);
    const u8 op = w0 >> 24;
    const u32 flags = w0 >> 16 & 0xff;
    const u32 address = lumiverseAudioResolve(m, dialect, w1);
    const bool abi1 = dialect == LumiverseAudioDialectABI1;
    if(dialect == LumiverseAudioDialectNaudio) {
      lumiverseAudioExecuteNaudio(m, shadow, w0, w1);
      continue;
    }

    switch(op) {
    case 0x00:
      break;

    case 0x01: {  //ADPCM: decode 9-byte/16-sample VADPCM frames
      auto& state = lumiverseAudioState(address);
      s16 last[16];
      if(flags & 1) {
        for(auto& sample : last) sample = 0;
      } else if(flags & 2) {
        u8 raw[32];
        lumiverseAudioReadRDRAM(shadow, m.loopAddr, raw, 32);
        for(u32 index = 0; index < 16; index++) last[index] = (s16)((u16)raw[index * 2] << 8 | raw[index * 2 + 1]);
      } else {
        for(u32 index = 0; index < 16; index++) last[index] = state.samples[index];
      }
      s32 prev2 = last[14], prev1 = last[15];

      u32 in = m.inBuf;
      u32 out = m.outBuf;
      //output layout (verified against an isolated LLE decode via the
      //truncated-task oracle): the 16 history samples are written first at
      //`out` (zeros on INIT, the loop state on LOOP, the previous chunk's
      //tail otherwise), and decoded data starts at out+0x20. Downstream
      //commands rely on the prefix (the resampler reads its history from it).
      for(u32 index = 0; index < 16; index++) lumiverseAudioDmemWriteS16(dmem, out + index * 2, last[index]);
      out += 32;
      //whole 9-byte/16-sample frames: a count that is not a frame multiple
      //still decodes (and writes) the full final frame — later commands read
      //into the rounded-up region (verified against SF64's looping-voice
      //DMEM layout). count==0 only applies the flag's state action.
      const u32 frames = (m.bufCount + 31) / 32;
      for(u32 frame = 0; frame < frames; frame++) {
        const u8 header = dmem[in++ & 0xfff];
        const u32 scale = header >> 4;
        u32 predictor = header & 0xf;
        if(predictor * 16 + 16 > m.bookEntries) predictor = 0;
        const s16* row0 = &m.book[predictor * 16];      //coefficients of x[n-2]
        const s16* row1 = &m.book[predictor * 16 + 8];  //coefficients of x[n-1]

        s32 residual[16];
        if(flags & 4) {
          //2-bit residual frames (5 bytes/16 samples; Majora's Mask ambience)
          for(u32 byteIndex = 0; byteIndex < 4; byteIndex++) {
            const u8 byte = dmem[in++ & 0xfff];
            for(u32 sub = 0; sub < 4; sub++) {
              s32 value = byte >> (6 - sub * 2) & 3;
              if(value >= 2) value -= 4;
              residual[byteIndex * 4 + sub] = value << scale;
            }
          }
        } else {
          for(u32 byteIndex = 0; byteIndex < 8; byteIndex++) {
            const u8 byte = dmem[in++ & 0xfff];
            s32 hi = byte >> 4, lo = byte & 0xf;
            if(hi >= 8) hi -= 16;
            if(lo >= 8) lo -= 16;
            residual[byteIndex * 2 + 0] = hi << scale;
            residual[byteIndex * 2 + 1] = lo << scale;
          }
        }

        for(u32 half = 0; half < 2; half++) {
          const s32* r = &residual[half * 8];
          s16 decoded[8];
          for(u32 j = 0; j < 8; j++) {
            //order-2 IIR in exact matrix form: row1 doubles as the impulse
            //response of the recursion (response to a residual at lag k+1
            //equals the x[n-1] coefficient of sample k)
            s64 acc = (s64)row0[j] * prev2 + (s64)row1[j] * prev1 + ((s64)r[j] << 11);
            for(u32 k = 0; k < j; k++) acc += (s64)row1[j - 1 - k] * r[k];
            decoded[j] = lumiverseAudioClamp16((s32)(acc >> 11));
          }
          for(u32 j = 0; j < 8; j++) lumiverseAudioDmemWriteS16(dmem, out + j * 2, decoded[j]);
          out += 16;
          prev2 = decoded[6];
          prev1 = decoded[7];
        }
      }
      //saved state = the 16 samples ending at the exact (unrounded) count
      //position. The window starts at original out + count: for counts under
      //0x20 it dips into the state prefix, which sits contiguously before
      //the decoded data. (Partial counts round the WRITE up to a full frame
      //but the state still ends at count — verified by shadow-comparing
      //SF64's looping voices, which chain through partial-count decodes.)
      for(u32 index = 0; index < 16; index++) {
        state.samples[index] = lumiverseAudioDmemReadS16(dmem, m.outBuf + m.bufCount + index * 2);
      }
      break;
    }

    case 0x02: {  //CLEARBUFF
      const u32 dmemAddr = w0 & 0xffff;
      const u32 count = w1 & 0xffff;
      for(u32 index = 0; index < count; index++) dmem[(dmemAddr + index) & 0xfff] = 0;
      break;
    }

    case 0x04: {  //ABI1: LOADBUFF via SETBUFF (in, count); ABI2: MIXER
      if(abi1) {
        if(!m.bufCount) break;
        u8 scratch[0x1000];
        lumiverseAudioReadRDRAM(shadow, address, scratch, m.bufCount);
        for(u32 index = 0; index < m.bufCount; index++) dmem[(m.inBuf + index) & 0xfff] = scratch[index];
        break;
      }
      const u32 count = (w0 >> 16 & 0xff) << 4;
      const s32 gain = (s16)(w0 & 0xffff);
      const u32 in = w1 >> 16, out = w1 & 0xffff;
      for(u32 index = 0; index < count; index += 2) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, in + index);
        const s32 y = lumiverseAudioDmemReadS16(dmem, out + index);
        lumiverseAudioDmemWriteS16(dmem, out + index, lumiverseAudioClamp16(y + (x * gain + 0x4000 >> 15)));
      }
      break;
    }

    case 0x06: {  //SAVEBUFF (ABI1): SETBUFF out/count -> RDRAM
      if(!m.bufCount) break;
      u8 scratch[0x1000];
      for(u32 index = 0; index < m.bufCount; index++) scratch[index] = dmem[(m.outBuf + index) & 0xfff];
      lumiverseAudioWriteRDRAM(shadow, address, scratch, m.bufCount);
      break;
    }

    case 0x09:  //SETVOL (ABI1): flags = A_VOL(4)|A_LEFT(2) / A_AUX(8)
      switch(flags) {
      case 0x06: m.setVolL = (s16)(w0 & 0xffff); break;
      case 0x04: m.setVolR = (s16)(w0 & 0xffff); break;
      case 0x02: m.setTargetL = (s16)(w0 & 0xffff); m.setRateL = w1; break;
      case 0x00: m.setTargetR = (s16)(w0 & 0xffff); m.setRateR = w1; break;
      case 0x08: m.setDryVol = (s16)(w0 & 0xffff); m.setWetVol = (s16)(w1 & 0xffff); break;
      }
      break;

    case 0x0e: {  //POLEF (ABI1): one-pole IIR lowpass, in-place per SETBUFF.
      //The table loaded via LOADADPCM holds [8 zeros][a^(j+1) in Q14]; the
      //command's low 16 bits are the input gain in Q14 (observed g = 1 - a
      //for unity DC gain). y[n] = (x[n]*g + y[n-1]*a) >> 14.
      auto& state = lumiverseAudioState(address);
      const s32 pole = m.bookEntries > 8 ? m.book[8] : 0;
      const s32 gain = (s16)(w0 & 0xffff);
      s32 previous = (flags & 1) ? 0 : state.samples[0];
      const u32 samples = m.bufCount / 2;
      for(u32 n = 0; n < samples; n++) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, m.inBuf + n * 2);
        const s32 y = lumiverseAudioClamp16((s32)(((s64)x * gain + (s64)previous * pole) >> 14));
        lumiverseAudioDmemWriteS16(dmem, m.outBuf + n * 2, (s16)y);
        previous = y;
      }
      state.samples[0] = (s16)previous;
      break;
    }

    case 0x03: {  //ENVMIXER (ABI1): in/dryL from SETBUFF, dryR/wetL/wetR aux.
      //Volumes are tracked at Q16 fractional precision: the multiplicative
      //ramp starts voices at vol=1 (exponential attack), which an integer
      //vol register would freeze at 1 forever (verified against LLE's
      //per-sample gain curve on SM64's title voice: rate 0x1717e/group grows
      //1 -> 1.44 -> 2.07 ... only with the fraction kept).
      //the envelope parameters live in the per-voice state: CONTINUE
      //chunks are issued WITHOUT fresh aSetVolume commands (observed in
      //SM64's alists), so target/rate/dry/wet must persist per voice, not
      //in machine-global registers
      auto& state = lumiverseAudioState(address);
      s64 vl32, vr32;
      s32 targetL, targetR, dry, wet;
      u32 rateL, rateR;
      if(flags & 1) {
        vl32 = (s64)m.setVolL << 16;
        vr32 = (s64)m.setVolR << 16;
        targetL = m.setTargetL; targetR = m.setTargetR;
        rateL = m.setRateL; rateR = m.setRateR;
        dry = m.setDryVol; wet = m.setWetVol;
      } else {
        vl32 = (s64)(u16)state.samples[0] << 16 | (u16)state.samples[1];
        vr32 = (s64)(u16)state.samples[2] << 16 | (u16)state.samples[3];
        targetL = state.samples[4]; targetR = state.samples[5];
        rateL = (u32)(u16)state.samples[6] << 16 | (u16)state.samples[7];
        rateR = (u32)(u16)state.samples[8] << 16 | (u16)state.samples[9];
        dry = state.samples[10]; wet = state.samples[11];
      }
      const u32 samples = m.bufCount / 2;
      auto accumulate = [&](u32 bus, u32 n, s32 value) {
        const s32 current = lumiverseAudioDmemReadS16(dmem, bus + n * 2);
        lumiverseAudioDmemWriteS16(dmem, bus + n * 2, lumiverseAudioClamp16(current + value));
      };
      //the microcode ramps per SAMPLE at rate^(1/8) (the documented rate is
      //per 8-sample group; LLE's per-lane gain curve on attacks fits the
      //interpolated form)
      auto eighthRoot = [](u32 rate) -> u32 {
        const double value = (double)rate / 65536.0;
        return (u32)(__builtin_sqrt(__builtin_sqrt(__builtin_sqrt(value))) * 65536.0 + 0.5);
      };
      const u32 r8L = eighthRoot(rateL);
      const u32 r8R = eighthRoot(rateR);
      auto ramp = [](s64 vol32, s32 target, u32 rate, u32 rate8) -> s64 {
        s64 next = vol32 * rate8 >> 16;
        const s64 target32 = (s64)target << 16;
        if(rate >= 0x10000) { if(next > target32) next = target32; }
        else { if(next < target32) next = target32; }
        if(next > 0x7fff0000ll) next = 0x7fff0000ll;
        if(next < 0) next = 0;
        return next;
      };
      for(u32 n = 0; n < samples; n++) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, m.inBuf + n * 2);
        const s32 vl = (s32)(vl32 >> 16), vr = (s32)(vr32 >> 16);
        const s32 l = x * vl + 0x4000 >> 15;
        const s32 r = x * vr + 0x4000 >> 15;
        accumulate(m.outBuf, n, l * dry + 0x4000 >> 15);
        accumulate(m.aux1, n, r * dry + 0x4000 >> 15);
        if(flags & 8) {
          accumulate(m.aux2, n, l * wet + 0x4000 >> 15);
          accumulate(m.aux3, n, r * wet + 0x4000 >> 15);
        }
        vl32 = ramp(vl32, targetL, rateL, r8L);
        vr32 = ramp(vr32, targetR, rateR, r8R);
      }
      state.samples[0] = (s16)(u16)(vl32 >> 16);
      state.samples[1] = (s16)(u16)(vl32 & 0xffff);
      state.samples[2] = (s16)(u16)(vr32 >> 16);
      state.samples[3] = (s16)(u16)(vr32 & 0xffff);
      state.samples[4] = (s16)targetL;
      state.samples[5] = (s16)targetR;
      state.samples[6] = (s16)(u16)(rateL >> 16);
      state.samples[7] = (s16)(u16)(rateL & 0xffff);
      state.samples[8] = (s16)(u16)(rateR >> 16);
      state.samples[9] = (s16)(u16)(rateR & 0xffff);
      state.samples[10] = (s16)dry;
      state.samples[11] = (s16)wet;
      break;
    }

    case 0x0c: {  //MIXER: out += in * gain (Q15, saturated)
      const u32 count = abi1 ? m.bufCount : (w0 >> 16 & 0xff) << 4;
      const s32 gain = (s16)(w0 & 0xffff);
      const u32 in = w1 >> 16, out = w1 & 0xffff;
      for(u32 index = 0; index < count; index += 2) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, in + index);
        const s32 y = lumiverseAudioDmemReadS16(dmem, out + index);
        lumiverseAudioDmemWriteS16(dmem, out + index, lumiverseAudioClamp16(y + (x * gain + 0x4000 >> 15)));
      }
      break;
    }

    case 0x05: {  //RESAMPLE (linear interpolation; ucode uses a short FIR)
      auto& state = lumiverseAudioState(address);
      const u32 pitch = w0 & 0xffff;
      s16 history[8];
      u32 frac;
      if(flags & 1) {
        for(auto& sample : history) sample = 0;
        frac = 0;
      } else {
        for(u32 index = 0; index < 8; index++) history[index] = state.samples[index];
        frac = state.frac & 0xffff;
      }
      const u32 in = m.inBuf, out = m.outBuf;
      const u32 outSamples = m.bufCount / 2;
      auto sampleAt = [&](u32 index) -> s32 {
        if(index < 8) return history[index];
        return lumiverseAudioDmemReadS16(dmem, in + (index - 8) * 2);
      };
      //position base: history occupies virtual indices 0..7, dmem input
      //starts at index 8. The microcode's first output interpolates from two
      //input samples BEFORE the nominal `in` pointer (phase measured against
      //an isolated LLE resample via the truncated-task oracle: base 8 made
      //our output lead LLE's by 2 input samples), hence base 6.
      u64 position = ((u64)6 << 16) + frac;
      const u32 step = pitch << 1;  //Q16 samples per output sample
      for(u32 n = 0; n < outSamples; n++) {
        const u32 index = (u32)(position >> 16);
        const s64 f = (s64)(position & 0xffff);  //Q16 fraction
        //Catmull-Rom cubic between x1 (index-1) and x2 (index) — our own
        //4-tap kernel, close in response to the microcode's short FIR
        const s64 x0 = sampleAt(index - 2);
        const s64 x1 = sampleAt(index - 1);
        const s64 x2 = sampleAt(index);
        const s64 x3 = sampleAt(index + 1);
        const s64 f2 = f * f >> 16, f3 = f2 * f >> 16;
        const s64 catmull = x1
          + ((f  * (x2 - x0)) >> 17)
          + ((f2 * (2 * x0 - 5 * x1 + 4 * x2 - x3)) >> 17)
          + ((f3 * (3 * (x1 - x2) + x3 - x0)) >> 17);
        //cubic B-spline (smoothing, never overshoots)
        const s64 oneMinusF = 0x10000 - f;
        const s64 omf2 = oneMinusF * oneMinusF >> 16, omf3 = omf2 * oneMinusF >> 16;
        const s64 bspline = (omf3 * x0
          + (0x40000 - 6 * f2 + 3 * f3) * x1
          + (0x10000 + 3 * f + 3 * f2 - 3 * f3) * x2
          + f3 * x3) / 6 >> 16;
        //Mitchell blend (2/3 Catmull-Rom + 1/3 B-spline): mild smoothing
        //comparable to the microcode's short windowed FIR; plain Catmull-Rom
        //overshoots on sharp attacks and produced isolated clicks the LLE
        //reference doesn't have. Clamp to the bracketing samples as a final
        //overshoot guard.
        s64 y = (2 * catmull + bspline) / 3;
        const s64 lower = x1 < x2 ? x1 : x2;
        const s64 upper = x1 < x2 ? x2 : x1;
        if(y < lower) y = lower;
        if(y > upper) y = upper;
        lumiverseAudioDmemWriteS16(dmem, out + n * 2, lumiverseAudioClamp16((s32)y));
        position += step;
      }
      //save the history window so the next chunk continues the stream
      //exactly: with base 6, virtual index (consumed + i) must become the
      //next chunk's hist[i] (i.e. window = v[end-6 .. end+1]); a window
      //ending at floor(position) instead skips two samples at every chunk
      //boundary and clicks at each seam (measured: clicks clustered at
      //offsets 0/176/352 inside AI buffers before this fix)
      const u32 endIndex = (u32)(position >> 16);
      for(u32 index = 0; index < 8; index++) {
        const u32 sourceIndex = endIndex - 6 + index;
        state.samples[index] = (s16)(sourceIndex < 8 ? history[sourceIndex] : lumiverseAudioDmemReadS16(dmem, in + (sourceIndex - 8) * 2));
      }
      state.frac = (u32)(position & 0xffff);
      break;
    }

    case 0x07: {  //ABI1: SEGMENT; ABI2 Zeldas: FILTER
      if(abi1) {
        m.segments[w1 >> 24 & 0xf] = w1 & 0x00ffffff;
        break;
      }
      if(flags == 2) {
        m.filterLength = w0 & 0xffff;
        m.filterCoefAddr = address;
        break;
      }
      auto& state = lumiverseAudioState(address);
      u8 rawCoefficients[16];
      lumiverseAudioReadRDRAM(shadow, m.filterCoefAddr, rawCoefficients, 16);
      //crossfaded kernel (measured by poking probe tables into an isolated
      //LLE filter run): the applied taps are the average of the table taps
      //and a per-voice state kernel that converges toward the table at
      //~15/16 per run and starts at zero on INIT. Steady state is therefore
      //a full-gain convolution; table changes (and note starts) fade in.
      s32 taps[8];
      for(u32 index = 0; index < 8; index++) {
        const s32 tap = (s16)((u16)rawCoefficients[index * 2] << 8 | rawCoefficients[index * 2 + 1]);
        const s32 kernel = (flags & 1) ? 0 : state.samples[8 + index];
        taps[index] = tap + kernel >> 1;
        state.samples[8 + index] = (s16)(tap - (tap - kernel >> 4));
      }
      s16 history[8];
      if(flags & 1) for(auto& sample : history) sample = 0;
      else for(u32 index = 0; index < 8; index++) history[index] = state.samples[index];

      const u32 buffer = w0 & 0xffff;
      const u32 samples = m.filterLength / 2;
      auto inputAt = [&](s32 index) -> s32 {
        if(index < 0) return history[8 + index];
        return lumiverseAudioDmemReadS16(dmem, buffer + (u32)index * 2);
      };
      //save the input tail before overwriting in place
      s16 newHistory[8];
      for(u32 index = 0; index < 8; index++) {
        newHistory[index] = (s16)inputAt((s32)samples - 8 + (s32)index);
      }
      s16 scratch[0x800];  //filterLength <= 0x1000 bytes, validated
      for(u32 n = 0; n < samples; n++) {
        s64 acc = 0;
        //plain causal convolution: the common identity table (unity at
        //tap 3) yields a 3-sample delay, matching an isolated LLE filter
        //run (truncated-task oracle showed our zero-centered variant led
        //LLE by exactly 3 samples)
        for(u32 k = 0; k < 8; k++) acc += (s64)taps[k] * inputAt((s32)n - (s32)k);
        scratch[n] = lumiverseAudioClamp16((s32)(acc >> 15));
      }
      for(u32 n = 0; n < samples; n++) lumiverseAudioDmemWriteS16(dmem, buffer + n * 2, scratch[n]);
      for(u32 index = 0; index < 8; index++) state.samples[index] = newHistory[index];
      break;
    }

    case 0x08:  //SETBUFF (flags 8 = A_AUX aux buffers, ABI1)
      if(abi1 && flags == 8) {
        m.aux1 = w0 & 0xffff;
        m.aux2 = w1 >> 16;
        m.aux3 = w1 & 0xffff;
        break;
      }
      m.inBuf = w0 & 0xffff;
      m.outBuf = w1 >> 16;
      m.bufCount = w1 & 0xffff;
      break;

    case 0x0a: {  //DMEMMOVE (count rounds up to a 16-byte vector multiple —
                  //verified via the truncated-task oracle on SF64 task 144:
                  //count 0x1c2 moves 0x1d0 bytes)
      const u32 in = w0 & 0xffff, out = w1 >> 16;
      const u32 count = ((w1 & 0xffff) + 15) & ~15u;
      u8 scratch[0x1000];
      for(u32 index = 0; index < count; index++) scratch[index] = dmem[(in + index) & 0xfff];
      for(u32 index = 0; index < count; index++) dmem[(out + index) & 0xfff] = scratch[index];
      break;
    }

    case 0x0b: {  //LOADADPCM: count is in BYTES (0x40 = 2 predictors)
      const u32 count = w0 & 0xffff;
      u8 raw[sizeof(LumiverseAudioMachine::book)];
      lumiverseAudioReadRDRAM(shadow, address, raw, count);
      for(u32 index = 0; index < count / 2; index++) {
        m.book[index] = (s16)((u16)raw[index * 2] << 8 | raw[index * 2 + 1]);
      }
      m.bookEntries = count / 2;
      break;
    }

    case 0x0d: {  //INTERLEAVE
      u32 out, count;
      if(w0 & 0x00ffffff) { out = w0 & 0xfff; count = w0 >> 12 & 0xfff; }
      else { out = m.outBuf; count = m.bufCount; }
      const u32 l = w1 >> 16, r = w1 & 0xffff;
      const u32 samples = count / 2;
      s16 scratch[0x800];
      for(u32 n = 0; n < samples; n++) {
        scratch[n * 2 + 0] = lumiverseAudioDmemReadS16(dmem, l + n * 2);
        scratch[n * 2 + 1] = lumiverseAudioDmemReadS16(dmem, r + n * 2);
      }
      for(u32 n = 0; n < samples * 2; n++) lumiverseAudioDmemWriteS16(dmem, out + n * 2, scratch[n]);
      break;
    }

    case 0x0f:  //SETLOOP
      m.loopAddr = address;
      break;

    case 0x11: {  //COPY (SF64): count samples from src to dst
      const u32 count = (w0 & 0xffff) * 2;
      const u32 src = w1 >> 16, dst = w1 & 0xffff;
      u8 scratch[0x1000];
      for(u32 index = 0; index < count; index++) scratch[index] = dmem[(src + index) & 0xfff];
      for(u32 index = 0; index < count; index++) dmem[(dst + index) & 0xfff] = scratch[index];
      break;
    }

    case 0x12:  //ENVSETUP1
      m.wetGain = w0 >> 16 & 0xff;
      m.rateL = (s16)(w1 >> 16);
      m.rateR = (s16)(w1 & 0xffff);
      break;

    case 0x1a: {  //DUPLICATE (SF64): tile `count` bytes at src into `flags`
                  //consecutive copies at dst — extends short looped
                  //waveforms (verified against an isolated LLE run: flags=1
                  //copied 0x80 bytes from 0x5f0 to 0x670)
      const u32 copies = flags ? flags : 1;
      const u32 src = w0 & 0xffff;
      const u32 dst = w1 >> 16;
      const u32 count = w1 & 0xffff;
      u8 scratch[0x1000];
      for(u32 index = 0; index < count; index++) scratch[index] = dmem[(src + index) & 0xfff];
      for(u32 copy = 0; copy < copies; copy++) {
        for(u32 index = 0; index < count; index++) dmem[(dst + copy * count + index) & 0xfff] = scratch[index];
      }
      break;
    }

    case 0x16:  //ENVSETUP2
      m.volL = w1 >> 16;
      m.volR = w1 & 0xffff;
      break;

    case 0x13: {  //ENVMIX
      const u32 in = (w0 >> 16 & 0xff) << 4;
      const u32 count = w0 >> 8 & 0xff;
      const u32 extraFlags = w0 & 0xff;
      if(extraFlags) m.unknownEnvmixFlags++;
      const u32 dryL = (w1 >> 24) << 4;
      const u32 dryR = (w1 >> 16 & 0xff) << 4;
      const u32 wetL = (w1 >> 8 & 0xff) << 4;
      const u32 wetR = (w1 & 0xff) << 4;
      s32 vl = (s32)m.volL, vr = (s32)m.volR;
      const s32 wet = (s32)m.wetGain;
      auto accumulate = [&](u32 bus, u32 n, s32 value) {
        const s32 current = lumiverseAudioDmemReadS16(dmem, bus + n * 2);
        lumiverseAudioDmemWriteS16(dmem, bus + n * 2, lumiverseAudioClamp16(current + value));
      };
      for(u32 n = 0; n < count; n++) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, in + n * 2);
        //volumes are Q16 (full scale 0x10000) and the wet send is Q8 —
        //shadow amplitude ratios showed a uniform 2x excess with Q15/Q7,
        //and OoT passes wet-send values above 0x7f
        const s32 sampleL = x * vl + 0x8000 >> 16;
        const s32 sampleR = x * vr + 0x8000 >> 16;
        accumulate(dryL, n, sampleL);
        accumulate(dryR, n, sampleR);
        accumulate(wetL, n, sampleL * wet + 0x80 >> 8);
        accumulate(wetR, n, sampleR * wet + 0x80 >> 8);
        if((n & 7) == 7) {
          //rate applies once per 8-sample vector (verified from volume
          //timelines across consecutive tasks)
          vl += m.rateL; if(vl < 0) vl = 0; if(vl > 0xffff) vl = 0xffff;
          vr += m.rateR; if(vr < 0) vr = 0; if(vr > 0xffff) vr = 0xffff;
        }
      }
      break;
    }

    case 0x14: {  //LOADBUFF
      const u32 count = w0 >> 12 & 0xfff, dmemAddr = w0 & 0xfff;
      if(!count) break;
      u8 scratch[0x1000];
      lumiverseAudioReadRDRAM(shadow, address, scratch, count);
      for(u32 index = 0; index < count; index++) dmem[(dmemAddr + index) & 0xfff] = scratch[index];
      break;
    }

    case 0x15: {  //SAVEBUFF
      const u32 count = w0 >> 12 & 0xfff, dmemAddr = w0 & 0xfff;
      if(!count) break;
      u8 scratch[0x1000];
      for(u32 index = 0; index < count; index++) scratch[index] = dmem[(dmemAddr + index) & 0xfff];
      lumiverseAudioWriteRDRAM(shadow, address, scratch, count);
      break;
    }

    default:
      break;  //validated: unreachable
    }
  }
}

//----------------------------------------------------------------------------
//shadow compare: after LLE has executed the task this interpreter shadowed,
//byte-compare our buffered writes against what LLE actually wrote
//----------------------------------------------------------------------------

auto lumiverseAudioShadowCompare(LumiverseAudioShadow& shadow) -> void {
  if(!shadow.pending) return;
  shadow.pending = false;
  u32 exact = 0, close = 0, bad = 0;
  s32 taskMax = 0;
  static u64 detailLines = 0;
  for(u32 w = 0; w < shadow.writeCount; w++) {
    const auto& write = shadow.writes[w];
    s32 maxDiff = 0;
    u32 firstBad = 0xffffffff;
    u64 sumMine = 0, sumTheirs = 0;
    for(u32 index = 0; index + 1 < write.length; index += 2) {
      const s16 mine = (s16)((u16)shadow.data[write.offset + index] << 8 | shadow.data[write.offset + index + 1]);
      const s16 theirs = (s16)((u16)lumiverseAudioRDRAMReadByte(write.addr + index) << 8 | lumiverseAudioRDRAMReadByte(write.addr + index + 1));
      sumMine += mine < 0 ? -mine : mine;
      sumTheirs += theirs < 0 ? -theirs : theirs;
      s32 diff = (s32)mine - theirs;
      if(diff < 0) diff = -diff;
      if(diff > maxDiff) { maxDiff = diff; if(firstBad == 0xffffffff && diff > 64) firstBad = index; }
    }
    if(maxDiff > taskMax) taskMax = maxDiff;
    if(maxDiff == 0) exact++;
    else if(maxDiff <= 64) close++;
    else {
      bad++;
      if(maxDiff > 200 && detailLines < 400) {
        detailLines++;
        fprintf(stderr, "[rsp-hle-audio-shadow]   write cmd=%u addr=%06x len=%u maxdiff=%d firstbad=+%u ratio=%.3f\n",
          write.command, write.addr, write.length, maxDiff, firstBad,
          sumTheirs ? (double)sumMine / (double)sumTheirs : -1.0);
      }
    }
  }
  static u64 compared = 0;
  compared++;
  if(bad || (compared & 255) == 1) {
    fprintf(stderr, "[rsp-hle-audio-shadow] task %llu: writes=%u exact=%u close=%u bad=%u maxdiff=%d%s\n",
      (unsigned long long)shadow.task, shadow.writeCount, exact, close, bad, taskMax,
      shadow.overflow ? " (overflow)" : "");
  }
}

//debug>=4: on the first few mismatching tasks, dump the task's alist plus
//both DMEM images (ours and the LLE core's) for offline analysis
struct LumiverseAudioDebugCapture {
  u32 alist[0x2000];
  u32 alistWords = 0;
  u8 dmem[4096];
};

auto lumiverseAudioDebugDump(const LumiverseAudioDebugCapture& capture, u64 task) -> void {
  static u32 dumps = 0;
  if(dumps >= 3) return;
  dumps++;
  char path[128];
  snprintf(path, sizeof(path), "/tmp/lumiverse-audio-debug-%llu.txt", (unsigned long long)task);
  FILE* fp = fopen(path, "w");
  if(!fp) return;
  fprintf(fp, "task %llu alist:\n", (unsigned long long)task);
  for(u32 index = 0; index + 1 < capture.alistWords; index += 2) {
    fprintf(fp, "  %08x %08x\n", capture.alist[index], capture.alist[index + 1]);
  }
  fprintf(fp, "dmem (mine | theirs):\n");
  for(u32 offset = 0x000; offset < 0xfc0; offset += 16) {
    fprintf(fp, "%03x:", offset);
    for(u32 b = 0; b < 16; b += 2) fprintf(fp, " %04x", (u16)((u16)capture.dmem[offset + b] << 8 | capture.dmem[offset + b + 1]));
    fprintf(fp, " |");
    for(u32 b = 0; b < 16; b += 2) fprintf(fp, " %04x", (u16)rsp.dmem.read<Half>(offset + b));
    fprintf(fp, "\n");
  }
  fclose(fp);
  fprintf(stderr, "[rsp-hle-audio-dmem] dumped %s\n", path);
}

//optional (LUMIVERSE_ARES_N64_AUDIO_HLE_DEBUG>=2): diff our virtual DMEM
//workspace against the real RSP DMEM the LLE core just used. The alist's
//dmem offsets address real DMEM directly, so matching regions prove each
//command's semantics; mismatching regions pinpoint the failing command.
auto lumiverseAudioShadowCompareDMEM(const LumiverseAudioMachine& machine, u64 task) -> s32 {
  static u64 lines = 0;
  s32 worst = 0;
  u32 rangeStart = 0;
  s32 rangeMax = 0;
  bool inRange = false;
  //0xf80.. holds ucode-private state (LLE resampler state observed at 0xf90)
  //and the OSTask struct at 0xfc0: excluded
  for(u32 offset = 0x400; offset <= 0xf80; offset += 2) {
    s32 diff = 0;
    if(offset < 0xf80) {
      const s16 mine = (s16)((u16)machine.dmem[offset] << 8 | machine.dmem[offset + 1]);
      const s16 theirs = (s16)rsp.dmem.read<Half>(offset);
      diff = (s32)mine - theirs;
      if(diff < 0) diff = -diff;
    }
    if(diff > worst) worst = diff;
    if(diff > 8) {
      if(!inRange) { inRange = true; rangeStart = offset; rangeMax = 0; }
      if(diff > rangeMax) rangeMax = diff;
    } else if(inRange) {
      inRange = false;
      if(rangeMax > 256 && lines < 600) {
        lines++;
        fprintf(stderr, "[rsp-hle-audio-dmem] task %llu: dmem %03x..%03x maxdiff=%d\n",
          (unsigned long long)task, rangeStart, offset, rangeMax);
        if(lumiverseAudioHLEDebug() >= 3) {
          fprintf(stderr, "[rsp-hle-audio-dmem]   mine  :");
          for(u32 b = 0; b < 16 && rangeStart + b * 2 < 0xfc0; b++) {
            fprintf(stderr, " %04x", (u16)((u16)machine.dmem[rangeStart + b * 2] << 8 | machine.dmem[rangeStart + b * 2 + 1]));
          }
          fprintf(stderr, "\n[rsp-hle-audio-dmem]   theirs:");
          for(u32 b = 0; b < 16 && rangeStart + b * 2 < 0xfc0; b++) {
            fprintf(stderr, " %04x", (u16)rsp.dmem.read<Half>(rangeStart + b * 2));
          }
          fprintf(stderr, "\n");
        }
      }
    }
  }
  return worst;
}

//----------------------------------------------------------------------------
//entry point (declared in lumiverse-hle.cpp)
//----------------------------------------------------------------------------

LumiverseAudioMachine lumiverseAudioMachine;
LumiverseAudioShadow lumiverseAudioShadowState;
LumiverseAudioDebugCapture* lumiverseAudioCapture = nullptr;
//set when a truncated-task experiment shadowed a task: the game usually
//crashes on the corrupted audio, so the comparison is settled from
//RSP::main()'s halted path instead of the next dispatch
bool lumiverseAudioTruncateArmed = false;

//settle any outstanding shadow comparison (called from every RSP task
//dispatch so a comparison still happens when the game stops issuing audio
//tasks, e.g. after a truncated-task experiment)
auto lumiverseAudioShadowSettle() -> void {
  if(lumiverseAudioHLELevel() < 2) return;
  auto& shadow = lumiverseAudioShadowState;
  const bool hadPending = shadow.pending;
  lumiverseAudioShadowCompare(shadow);
  if(hadPending && lumiverseAudioHLEDebug() >= 2) {
    const s32 worst = lumiverseAudioShadowCompareDMEM(lumiverseAudioMachine, shadow.task);
    static s64 truncateTask = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_TASK");
      return value ? ::atoll(value) : -1;
    }();
    //with a truncate target set, dump only that task (don't burn the dump
    //quota on earlier tasks)
    const bool dumpWanted = truncateTask >= 0
      ? shadow.task == (u64)truncateTask
      : worst > 1000;
    if(dumpWanted && lumiverseAudioCapture) lumiverseAudioDebugDump(*lumiverseAudioCapture, shadow.task);
  }
}

auto lumiverseExecuteAudioTask(const u32 task[16], u64 ucodeHash) -> bool {
  const int level = lumiverseAudioHLELevel();
  if(level < 1) return false;
  const s32 dialect = lumiverseAudioDialectForHash(ucodeHash);
  if(dialect < 0) return false;

  const u32 dataPtr = task[12] & 0x00ffffff;
  u32 dataSize = task[13];
  if(!dataPtr || dataSize < 8 || dataSize > 0x10000) return false;

  auto& machine = lumiverseAudioMachine;
  auto& shadow = lumiverseAudioShadowState;
  static u64 taskIndex = 0;

  auto*& capture = lumiverseAudioCapture;
  if(!capture && lumiverseAudioHLEDebug() >= 4) capture = new LumiverseAudioDebugCapture;

  //debug tool (shadow mode only): truncate one task's alist IN RDRAM after
  //command index LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_CMD on task
  //..._TRUNCATE_TASK, so both LLE and this interpreter execute exactly that
  //prefix — a per-command oracle for isolating one command's semantics.
  //Deliberately corrupts the game's audio; harvesting data only.
  if(level >= 2) {
    static s64 truncateTask = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_TASK");
      return value ? ::atoll(value) : -1;
    }();
    static s64 truncateCommand = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_CMD");
      return value ? ::atoll(value) : -1;
    }();
    if(truncateTask >= 0 && (u64)truncateTask == taskIndex && truncateCommand >= 0) {
      //shrink the OSTask's dataSize (DMEM 0xfc0 + 13*4) so the microcode
      //stops after the chosen command; also truncate our own walk
      const u32 truncatedSize = (u32)(truncateCommand + 1) * 8;
      if(truncatedSize < dataSize) {
        dataSize = truncatedSize;
        rsp.dmem.write<Word>(0xfc0 + 13 * 4, truncatedSize);
      }
      lumiverseAudioTruncateArmed = true;
      //optional: overwrite a 32-byte RDRAM block (e.g. a filter coefficient
      //table) with a probe pattern of distinct power-of-two taps so the
      //LLE-vs-HLE comparison reveals the exact per-tap transformation
      static s64 pokeAddr = [] {
        const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_POKE");
        return value ? ::strtoll(value, nullptr, 16) : -1;
      }();
      if(pokeAddr >= 0) {
        static const u16 probe[16] = {
          0x4000, 0x1000, 0x0400, 0x0100, 0xc000, 0xf000, 0x0040, 0x0010,
          0x2000, 0x0800, 0x0200, 0x0080, 0x1000, 0x0008, 0x0002, 0xe000 };
        for(u32 index = 0; index < 16; index++) {
          rdram.ram.Memory::Writable::write<Byte>(((u32)pokeAddr + index * 2 + 0) & 0x00ffffff, probe[index] >> 8);
          rdram.ram.Memory::Writable::write<Byte>(((u32)pokeAddr + index * 2 + 1) & 0x00ffffff, probe[index] & 0xff);
        }
        fprintf(stderr, "[rsp-hle-audio] poked probe table at %06x\n", (u32)pokeAddr & 0x00ffffff);
      }
      fprintf(stderr, "[rsp-hle-audio] truncated task %llu after command %lld (size=%u)\n",
        (unsigned long long)taskIndex, (long long)truncateCommand, dataSize);
    }
  }

  //shadow mode: settle the previous task's comparison first (LLE has
  //completed it by now — audio tasks are serialized)
  if(level >= 2) lumiverseAudioShadowSettle();

  if(!lumiverseAudioValidate(machine, dataPtr, dataSize, (u32)dialect)) {
    machine.tasksFallback++;
    return false;
  }

  shadow.active = level >= 2;
  shadow.writeCount = 0;
  shadow.byteCount = 0;
  shadow.overflow = false;
  shadow.task = taskIndex;

  lumiverseAudioExecute(machine, shadow, dataPtr, dataSize, (u32)dialect);

  if(capture && level >= 2) {
    //stash the alist and our workspace now: the game reuses the alist buffer
    //for the next task, so it must be captured before the compare happens
    capture->alistWords = 0;
    for(u32 offset = 0; offset + 4 <= dataSize && capture->alistWords < 0x2000; offset += 4) {
      u32 word = 0;
      for(u32 b = 0; b < 4; b++) word = word << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + b);
      capture->alist[capture->alistWords++] = word;
    }
    for(u32 index = 0; index < 4096; index++) capture->dmem[index] = machine.dmem[index];
  }

  machine.tasksExecuted++;
  taskIndex++;
  if(lumiverseAudioHLEDebug() >= 1 && (machine.tasksExecuted & 1023) == 1) {
    fprintf(stderr, "[rsp-hle-audio] executed=%llu fallback=%llu envmix-unknown-flags=%llu\n",
      (unsigned long long)machine.tasksExecuted, (unsigned long long)machine.tasksFallback,
      (unsigned long long)machine.unknownEnvmixFlags);
  }

  if(shadow.active) {
    shadow.active = false;
    shadow.pending = true;
    return false;  //LLE still executes the task; we only compare
  }
  return true;
}

}  //namespace
