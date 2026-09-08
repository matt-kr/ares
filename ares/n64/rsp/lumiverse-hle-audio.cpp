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
//  0x11 DECIMATE  count=cmd0.lo16 OUTPUT samples, src=cmd1.hi16, dst=cmd1.lo16:
//                 dst[i] = src[2i] (2:1 pre-halving for high pitch ratios;
//                 oracle-exact on OoT (U) and SF64 — was mis-implemented as
//                 a plain copy through round 6)
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
//A third family ("naudio": Super Smash Bros. / Kirby 64) uses fixed
//0x170-byte chunks with state addresses in cmd0.lo24 and a per-voice RDRAM
//parameter block; see lumiverseAudioExecuteNaudio for the decoded layout.
//Validated via native-stream click counts, envelope correlation and level
//gates on both games (round 4).
//
//This file is included from rsp.cpp inside namespace ares::Nintendo64 (same
//translation unit as lumiverse-hle.cpp); no #includes allowed here.

namespace {

//LUMIVERSE_ARES_N64_AUDIO_HLE:
//  unset/0 = off (default; stock behavior)
//  1       = execute whitelisted audio tasks natively
//  3       = reverse shadow (validation): the interpreter's output is applied
//            to RDRAM and drives the game; the LLE microcode still runs each
//            task but its buffer writes (>32 bytes) are diverted into a side
//            image and compared against ours — i.e. the per-task compare
//            with the HLE's own inputs (state blobs stay LLE-owned)
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

//DEBUG>=3 env-line quota (round 14: LUMIVERSE_ARES_N64_AUDIO_HLE_ENV_LINES, default 4000)
static auto lumiverseAudioEnvLineCap() -> u32 {
  static const u32 cap = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_ENV_LINES"); return v ? (u32)::atoi(v) : 4000u; }();
  return cap;
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
//round 9: found by the library census + AUDIO_HLE_TRY executor trials; both
//pass the WAV gate on the ABI1 executor with zero fallbacks (Doom 64:
//envCorr 0.98/0.94, level 1.005/1.012, 0 clicks; Cruis'n USA: envCorr
//1.000/1.000, level 1.007, 0 clicks; 4000-step runs vs LLE audio)
constexpr u64 LumiverseAudioUcodeDoom64    = 0xd74cbe704463fdd3ull;  //Doom 64 (U)
constexpr u64 LumiverseAudioUcodeCruisnUSA = 0xecd40fd97f420e51ull;  //Cruis'n USA (U)
constexpr u64 LumiverseAudioUcodeSmashU    = 0xbbca23e37b0bc136ull;  //Super Smash Bros. (U) + Kirby 64
//round 11: Yoshi's Story's audio ucode is an ABI2 revision (its alist uses
//exactly the Zelda command set minus FILTER/DUPLICATE: 01 02 05 08 0a 0b 0c
//0d 0f 11 12 13 14 15 16). Truncated-task DMEM oracles on our own LLE show
//ADPCM, DECIMATE, INTERLEAVE, ENVMIX and the buffer moves sample-exact;
//RESAMPLE differs by the known interpolator approximation (~300 LSB). Shadow
//over 2049 title/menu tasks: 0 fallbacks, level 1.005, dratio 1.026, every
//output write corr >= 0.999; WAV vs LLE through world map + level 1-1:
//envCorr 0.993, level 1.005, dropouts equal. Click counts sit 25% above
//LLE's, but all of the excess is one noise burst (the level-entry whoosh):
//the |delta| distribution there is LLE's scaled by 1.2-1.3 at every
//threshold with the same onset and decay — the resampler's HF excess on a
//downsampled noise voice (round 8's class), not discontinuities.
constexpr u64 LumiverseAudioUcodeYoshi     = 0x2b5c40620a4cec16ull;  //Yoshi's Story (U)
//Rare's engines (round 8 census): Banjo-Kazooie's and Banjo-Tooie's audio
//ucodes issue the same fixed-0x170-chunk command set as naudio (04/06
//LOADBUFF/SAVEBUFF with count in cmd0 bits 12-23, 0c MIXER, 03 ENVMIX with
//an RDRAM parameter block, 09 init params, 05 RESAMPLE with the Q13 pitch +
//1/8-sample pointer) — gated on shadow validation, see the round-8 notes
//Ogre Battle 64 (round 12): naudio command set; it was the title whose
//ENVMIX parameter blocks exposed the real envelope arithmetic (additive
//vector-lane ramps, see the ENVMIX handler). Shadow 2049 tasks / 0
//fallbacks: level 1.000, dratio 0.999 (was 0.78 / 0.52 with the round-4
//multiplicative model); WAV vs LLE audio over 4000 steps: clicks 0/0,
//dropouts 16/16 vs 15/15 (the round-9 right-channel dropouts are gone),
//envCorr 0.996/0.997, level 0.998.
constexpr u64 LumiverseAudioUcodeOgre      = 0x6951fd2f2620850bull;  //Ogre Battle 64 (U)
constexpr u64 LumiverseAudioUcodeBanjoK    = 0xb5cc72d845279b67ull;  //Banjo-Kazooie (U)
constexpr u64 LumiverseAudioUcodeBanjoT    = 0x7497287654db04f8ull;  //Banjo-Tooie (U)
constexpr u64 LumiverseAudioUcodeConker    = 0x22df24b6bcc461d4ull;  //Conker's Bad Fur Day (U)
constexpr u64 LumiverseAudioUcodePD        = 0xbba48a2eb4ca6feaull;  //Perfect Dark (U)
constexpr u64 LumiverseAudioUcodeDK64      = 0xf93c418265e63af1ull;  //Donkey Kong 64 (U)

auto lumiverseAudioRareOptIn() -> bool {
  static const bool value = [] {
    const char* env = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_RARE");
    return env && env[0] == '1';
  }();
  return value;
}

//explicit LUMIVERSE_ARES_N64_AUDIO_HLE_RARE=0 disables the whitelisted Rare
//titles too (round 13: Perfect Dark)
auto lumiverseAudioRareOptOut() -> bool {
  static const bool value = [] {
    const char* env = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_RARE");
    return env && env[0] == '0';
  }();
  return value;
}

//LUMIVERSE_ARES_N64_AUDIO_HLE_TRY=<16-hex ucode hash>:<dialect 0|1|2>: run
//one un-whitelisted ucode on a chosen executor (census/shadow work only)
auto lumiverseAudioTryDialect(u64 hash) -> s32 {
  static const u64 tryHash = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRY"); return v ? (u64)::strtoull(v, nullptr, 16) : 0ull; }();
  static const s32 tryDialect = [] {
    const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRY");
    const char* colon = v ? ::strchr(v, ':') : nullptr;
    return colon ? (s32)::atoi(colon + 1) : -1;
  }();
  if(tryHash && hash == tryHash && tryDialect >= 0 && tryDialect <= 2) return tryDialect;
  return -1;
}

//returns the command-set dialect for a whitelisted audio ucode, -1 otherwise
auto lumiverseAudioDialectForHash(u64 hash) -> s32 {
  if(const s32 tried = lumiverseAudioTryDialect(hash); tried >= 0) return tried;
  if(hash == LumiverseAudioUcodeStarFox64) return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeZeldaMQ)   return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeZeldaOoTU) return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeMajoraU)   return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeYoshi)     return LumiverseAudioDialectABI2;
  if(hash == LumiverseAudioUcodeSM64WR)    return LumiverseAudioDialectABI1;
  if(hash == LumiverseAudioUcodeSnapU)     return LumiverseAudioDialectABI1;
  if(hash == LumiverseAudioUcodeDoom64)    return LumiverseAudioDialectABI1;
  if(hash == LumiverseAudioUcodeCruisnUSA) return LumiverseAudioDialectABI1;
  if(hash == LumiverseAudioUcodeSmashU)    return LumiverseAudioDialectNaudio;
  if(hash == LumiverseAudioUcodeOgre)      return LumiverseAudioDialectNaudio;
  //round 13: Banjo-Kazooie on the reverb-tap model + measured kernel —
  //Spiral Mountain (20000 steps): shared-timeline gate envCorr 0.999,
  //level 1.002, clicks 13/16 vs LLE's 15/16, frames identical; against a
  //plain LLE-audio run the envelopes correlate 0.893 only because LLE's
  //RSP time shifts the intro's timeline (level 1.002, clicks 15/17 vs
  //15/16). LUMIVERSE_ARES_N64_AUDIO_HLE_RARE=0 keeps it on LLE.
  if(hash == LumiverseAudioUcodeBanjoK && !lumiverseAudioRareOptOut()) return LumiverseAudioDialectNaudio;
  if(hash == LumiverseAudioUcodeBanjoT && lumiverseAudioRareOptIn()) return LumiverseAudioDialectNaudio;
  if(hash == LumiverseAudioUcodeConker && lumiverseAudioRareOptIn()) return LumiverseAudioDialectNaudio;
  //round 13: Perfect Dark passes the WAV gate under the global gate
  //(Carrington Institute, 7000 steps, HLE vs LLE audio: envCorr 0.993,
  //level 1.004, clicks 0/0 vs 1/1) — whitelisted; LUMIVERSE_ARES_N64_AUDIO_HLE_RARE=0
  //keeps the Rare family (incl. PD) on LLE
  if(hash == LumiverseAudioUcodePD     && !lumiverseAudioRareOptOut()) return LumiverseAudioDialectNaudio;
  //round 14: DK64 under the global gate (fair gate 0.999 with frames and
  //alists identical, real gate 0.999 with the cost-modelled completion);
  //Banjo-Tooie's fair gate is identical too but its real gate stays at
  //0.59 (its voice manager is sensitive to the completion time beyond the
  //1.2% cost model), so it and Conker (07/08 echo ring) stay opt-in
  if(hash == LumiverseAudioUcodeDK64   && !lumiverseAudioRareOptOut()) return LumiverseAudioDialectNaudio;
  return -1;
}

//self-modifying audio ucodes (round 11): the full-image hash changes every
//dispatch (27-31 images per run), so these families are identified by the
//FNV-1a hash of the first LumiverseAudioUcodePrefixLength bytes instead —
//see the census note in lumiverse-hle.cpp. Consulted only when the full
//hash is not whitelisted. LUMIVERSE_ARES_N64_AUDIO_HLE_TRY matches either.
//round-11 census (31 images each -> one prefix hash each):
//  Mario Kart 64      9cf990efdedb3b1e  ABI2 shape (Zelda's command set with
//                     the ABI1-style SEGMENT(0) opener); shadow 2049 tasks /
//                     0 fallbacks, level 1.003, dratio 1.002; WAV vs LLE
//                     (title -> menus -> race): clicks 0/0, dropouts 3/3,
//                     envCorr 0.92/0.93, level 1.003 -> SHIPPED
//  Pilotwings 64 / Shadows of the Empire / Cruis'n World share
//                     1d095e498bf7f786  ABI1 (Cruis'n USA's executor); shadow
//                     PW 2049/0 level 1.003 dratio 1.020, CW 2049/0 level
//                     1.010 dratio 1.108, SOTE 1 task (its title has no
//                     audio tasks); WAV: CW clicks 0/0 envCorr 0.98 level
//                     0.99; PW clicks 0/0 level 1.003 but envCorr 0.86 —
//                     its intro flight diverges between the runs (frames 12%
//                     px apart from step 500), aligned windows correlate
//                     0.93-0.97 -> SHIPPED as a family
//  GoldenEye          ea212b6bce94100c  ABI1; shadow 1025/0 level 0.997
//                     dratio 1.03, but WAV clicks 217/254 vs 70/98 with a
//                     1.5-1.8x >8 kHz excess in the theme's loud bars ->
//                     NOT shipped (opt-in LUMIVERSE_ARES_N64_AUDIO_HLE_GE=1)
//  Paper Mario        4ea075cb3bd248f6  a different ABI1 variant: alist at
//                     DMEM 0x2c0, buffers addressed from a base, MIXER
//                     count implicit -> not attempted
constexpr u64 LumiverseAudioPrefixMK64   = 0x9cf990efdedb3b1eull;
constexpr u64 LumiverseAudioPrefixPWSOTE = 0x1d095e498bf7f786ull;
constexpr u64 LumiverseAudioPrefixGE     = 0xea212b6bce94100cull;
constexpr u64 LumiverseAudioPrefixPM     = 0x4ea075cb3bd248f6ull;  //Paper Mario (naudio family, round 12)
//round 15: San Francisco Rush's stable image (fabfd6fff1f6c2f1) shares the
//Pilotwings/SOTE/Cruis'n World prefix and had been running on the ABI1
//executor since round 11 without a gate (round 9 had NOT shipped it: WAV
//envCorr 0.84). Regression pass with the app env: shadow per-task exact
//(level 0.999, dratio 1.000 over 10500 writes) but fair gate 0.908 / 0.910
//and real gate 0.82 / 0.80 with the race diverging — the game path is
//sensitive to the task timing in a way the other ABI1 titles are not.
//Correct beats fast: excluded from the prefix family, runs LLE.
//round 16: the "timing sensitivity" was the 80-byte ENVMIXER state block
//the ABI1 microcode writes after every ENVMIXER and Rush's driver reads
//back (with the block kept LLE-owned in reverse-shadow mode the fair gate
//went 0.976 -> 1.000 with identical frames). The ABI1 executor now writes
//that block (lumiverseAudioAbiEnvState), Rush's fair gate is 1.000 / 1.000
//with frames identical on the LLE timeline, and it is back on the prefix
//family; the real gate stays at the chaos floor of its race (0.85; LLE
//audio on the Turbo timeline scores 0.82 against stock, round 15).
//LUMIVERSE_ARES_N64_AUDIO_HLE_RUSH=0 opts it out again.
constexpr u64 LumiverseAudioUcodeRush    = 0xfabfd6fff1f6c2f1ull;
//round 15: Tony Hawk's Pro Skater 2's audio image (stable, 971eca78873b6c68)
//shares Paper Mario's 3 KiB prefix and is dispatched through the PM prefix
//entry above. Shadow: 2049 tasks / 0 fallbacks, level 1.000, 0 bad writes
//over 4715 buffer writes; real gate vs LLE audio on the app timeline 0.976,
//level 0.998 (the round-14 "level 0.129" was an intermediate binary's run,
//not reproducible on the shipped code). Kept on the executor.

auto lumiverseAudioDialectForPrefixHash(u64 prefixHash) -> s32 {
  if(const s32 tried = lumiverseAudioTryDialect(prefixHash); tried >= 0) return tried;
  if(prefixHash == LumiverseAudioPrefixMK64)   return LumiverseAudioDialectABI2;
  if(prefixHash == LumiverseAudioPrefixPWSOTE) return LumiverseAudioDialectABI1;
  //round 14: GoldenEye whitelisted — with the measured resampler kernel on
  //the ABI1 path (ABI_KERNEL) its Dam clicks went 221/254 -> 58/90 against
  //LLE's 69/91, fair gate 0.994/0.991; LUMIVERSE_ARES_N64_AUDIO_HLE_GE=0
  //opts out
  static const bool geOptOut = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_GE"); return v && v[0] == '0'; }();
  if(prefixHash == LumiverseAudioPrefixGE && !geOptOut) return LumiverseAudioDialectABI1;
  //round 13: Paper Mario on the naudio executor with the Rare-engine reverb
  //model (state-blob tap + one-pole 0e): title + intro + Peach's castle
  //(14000 steps) HLE vs LLE audio envCorr 0.948 (0.979 on the shared
  //timeline), level 0.945, clicks 0/0, dropouts 20/14 vs 18/13 — the
  //round-12 10-22x >8 kHz excess and 347 clicks are gone.
  //LUMIVERSE_ARES_N64_AUDIO_HLE_PM=0 keeps it on LLE.
  static const bool pmOptOut = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_PM"); return v && v[0] == '0'; }();
  if(prefixHash == LumiverseAudioPrefixPM && !pmOptOut) return LumiverseAudioDialectNaudio;
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

auto lumiverseAudioStateBytes() -> u32 {
  static const u32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_STATE_BYTES"); return v ? (u32)::atoi(v) : 32u; }();
  return value;
}

auto lumiverseAudioOwnDmem() -> s32;
auto lumiverseAudioNaStateWrites() -> bool;
extern u16 lumiverseAudioNaLastBlockLanes[3];
//round 14: LUMIVERSE_ARES_N64_AUDIO_HLE_DEFER_WRITES (default 1): the
//executor's RDRAM writes are held back and applied when the task COMPLETES
//(the HLE completion point, or — in reverse-shadow mode — the microcode's
//BREAK) instead of at dispatch. The Rare engines read the ENVMIX parameter
//blocks back on the CPU while the RSP is still working on the task: with
//the blocks written at dispatch the CPU saw the NEW volumes a task early,
//and Banjo-Tooie's / DK64's game path diverged (reverse-shadow frames
//differed from ~step 4000 with bit-exact blocks; identical with the blocks
//LLE-owned). 0 = write at dispatch (rounds 4-13).
auto lumiverseAudioDeferWrites() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_DEFER_WRITES"); return !v || v[0] != '0'; }();
  return value;
}
struct LumiverseAudioDeferredWrite { u32 addr, length, offset, command; bool landed; };
static LumiverseAudioDeferredWrite lumiverseAudioDeferred[4096];
static u32 lumiverseAudioDeferredCount = 0;
static u8 lumiverseAudioDeferredData[1 << 20];
static u32 lumiverseAudioDeferredBytes = 0;
static u32 lumiverseAudioDeferredCommands = 1;   //command count of the task the entries belong to
static auto lumiverseAudioLandDeferred(LumiverseAudioDeferredWrite& d) -> void {
  if(d.landed) return;
  for(u32 index = 0; index < d.length; index++) {
    rdram.ram.Memory::Writable::write<Byte>((d.addr + index) & 0x00ffffff, lumiverseAudioDeferredData[d.offset + index]);
  }
  d.landed = true;
}
auto lumiverseAudioFlushDeferredWrites() -> void {
  for(u32 w = 0; w < lumiverseAudioDeferredCount; w++) lumiverseAudioLandDeferred(lumiverseAudioDeferred[w]);
  lumiverseAudioDeferredCount = 0;
  lumiverseAudioDeferredBytes = 0;
}
//reverse-shadow timing parity: when the microcode DMAs a range out, the
//EARLIEST of our still-pending writes to that range lands at that moment
//(one per microcode DMA — the Rare engines write a voice's ENVMIX block
//after every chunk, four times per task, and the CPU reads it in between;
//landing all four at the first DMA showed the CPU the last chunk's volume
//three chunks early). Whatever is left lands at BREAK.
auto lumiverseAudioReleaseDeferredWrites(u32 address, u32 length) -> void {
  for(u32 w = 0; w < lumiverseAudioDeferredCount; w++) {
    auto& d = lumiverseAudioDeferred[w];
    if(d.landed || d.addr + d.length <= address || d.addr >= address + length) continue;
    lumiverseAudioLandDeferred(d);
    return;
  }
}
//real HLE with a modelled task duration (rsp.cpp, LUMIVERSE_ARES_N64_RSP_HLE_
//AUDIO_CYCLES_PER_CMD): writes land progressively — entry k (from command
//c of N) lands once the elapsed fraction of the modelled duration reaches
//c / N, approximating the microcode's in-order DMA schedule
//round 14: modelled microcode schedule — the cumulative cost (RSP cycles)
//of the alist up to and including each command, from a per-opcode cost
//table fitted on 4303 Banjo-Tooie / DK64 / Banjo-Kazooie LLE tasks (rms
//error 1.2% of the task duration; a flat per-command cost was 14-18%).
//A deferred write from command c lands once the modelled elapsed time
//reaches the cost prefix at c. Empty (no model) = land by command fraction.
static u32 lumiverseAudioCmdCostPrefix[8192];
static u32 lumiverseAudioCmdCostCount = 0;
auto lumiverseAudioProgressDeferredWrites(s64 elapsed, s64 total) -> void {
  if(total <= 0) return;
  for(u32 w = 0; w < lumiverseAudioDeferredCount; w++) {
    auto& d = lumiverseAudioDeferred[w];
    if(d.landed) continue;
    if(lumiverseAudioCmdCostCount) {
      const u32 c = d.command < lumiverseAudioCmdCostCount ? d.command : lumiverseAudioCmdCostCount - 1;
      if((s64)lumiverseAudioCmdCostPrefix[c] <= elapsed) lumiverseAudioLandDeferred(d);
    } else if((s64)d.command * total <= elapsed * (s64)lumiverseAudioDeferredCommands) lumiverseAudioLandDeferred(d);
  }
}
auto lumiverseAudioSetDeferredCommands(u32 commands) -> void { lumiverseAudioDeferredCommands = commands ? commands : 1; }
//LUMIVERSE_ARES_N64_RSP_HLE_AUDIO_COST_MODEL (default 1): the Rare engines
//(Banjo-Kazooie/Tooie, DK64, Conker) complete after the modelled duration
//with their output landing on the modelled schedule; 0 = instant completion
//(rounds 4-13). Perfect Dark's engine fits a different table and stays
//instant (shipped so in round 13). Costs in RSP cycles per command.
auto lumiverseAudioCostModel() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_AUDIO_COST_MODEL"); return !v || v[0] != '0'; }();
  return value;
}
static const u16 lumiverseAudioRareCost[32] = {
  0, 1630, 265, 2999, 97, 3055, 186, 0,  0, 171, 910, 220, 1111, 0, 0, 855,
  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0 };
constexpr u32 lumiverseAudioRareCostConstant = 2460;
//round 14 diagnostic: LLE audio task duration (RSP cycles from dispatch to
//BREAK) in shadow modes, printed with the executed= summary
u64 lumiverseAudioTaskStartCycles = 0; u64 lumiverseAudioTaskCycleSum = 0; u64 lumiverseAudioTaskCycleCount = 0; u64 lumiverseAudioTaskCycleMax = 0;
u32 lumiverseAudioTaskOps[32] = {};
auto lumiverseAudioNoteTaskEnd(u64 cycles) -> void {
  if(!lumiverseAudioTaskStartCycles) return;
  const u64 d = cycles - lumiverseAudioTaskStartCycles;
  lumiverseAudioTaskStartCycles = 0;
  //DEBUG>=2: per-task LLE cost next to the task's opcode histogram (for the
  //per-opcode cost fit behind the modelled task duration)
  if(lumiverseAudioHLEDebug() >= 2) {
    static u32 lines = 0;
    if(lines++ < 20000) {
      fprintf(stderr, "[rsp-hle-audio-cost] cycles=%llu ops", (unsigned long long)d);
      for(u32 op = 0; op < 32; op++) if(lumiverseAudioTaskOps[op]) fprintf(stderr, " %02x:%u", op, lumiverseAudioTaskOps[op]);
      fprintf(stderr, "\n");
    }
  }
  lumiverseAudioTaskCycleSum += d; lumiverseAudioTaskCycleCount++;
  if(d > lumiverseAudioTaskCycleMax) lumiverseAudioTaskCycleMax = d;
}
auto lumiverseAudioReadRDRAM(LumiverseAudioShadow& shadow, u32 address, u8* out, u32 length) -> void {
  address &= 0x00ffffff;
  for(u32 index = 0; index < length; index++) out[index] = lumiverseAudioRDRAMReadByte(address + index);
  //round 14: this task's deferred (not yet landed) writes are what the
  //microcode would have put in RDRAM by now — save-then-reload sequences
  //inside a task must see them (real HLE has no shadow copy to serve them)
  for(u32 w = 0; w < lumiverseAudioDeferredCount; w++) {
    const auto& d = lumiverseAudioDeferred[w];
    if(d.landed) continue;
    const u32 begin = d.addr > address ? d.addr : address;
    const u32 end = (d.addr + d.length) < (address + length) ? (d.addr + d.length) : (address + length);
    for(u32 a = begin; a < end; a++) out[a - address] = lumiverseAudioDeferredData[d.offset + (a - d.addr)];
  }
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

auto lumiverseAudioWriteRDRAM(LumiverseAudioShadow& shadow, u32 address, const u8* src, u32 length, bool forceOwn = false) -> void {
  address &= 0x00ffffff;
  if(!shadow.active || (lumiverseAudioHLELevel() == 3 && (length > lumiverseAudioStateBytes() || forceOwn))) {
    if(lumiverseAudioDeferWrites() && lumiverseAudioDeferredCount < 4096 && lumiverseAudioDeferredBytes + length <= sizeof(lumiverseAudioDeferredData)) {
      auto& d = lumiverseAudioDeferred[lumiverseAudioDeferredCount++];
      d.addr = address; d.length = length; d.offset = lumiverseAudioDeferredBytes; d.command = shadow.currentCommand; d.landed = false;
      for(u32 index = 0; index < length; index++) lumiverseAudioDeferredData[d.offset + index] = src[index];
      lumiverseAudioDeferredBytes += length;
    } else {
      for(u32 index = 0; index < length; index++) {
        rdram.ram.Memory::Writable::write<Byte>((address + index) & 0x00ffffff, src[index]);
      }
    }
    if(!shadow.active) return;
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
  //naudio (Smash/Kirby) state
  u32 naLoadCount = 0;          //bytes staged by the last 04 load
  u32 naDecodeSamples = 0;      //samples produced by the last decode/clear
  s32 naPendVolL = 0, naPendVolR = 0;
  s32 naPendTargetL = 0, naPendTargetR = 0;
  u32 naPendRateL = 0x10000, naPendRateR = 0x10000;
  s32 naPendWet = 0x7fff, naPendDry = 0x7fff;
  //round 12 (Paper Mario): the reverb form of RESAMPLE reads the window the
  //engine LOADBUFFs at 0x2e0 and writes 0x170 (truncated-task oracle);
  //selected by the most recent data load
  u32 naSrcMode = 0;          //0 = voice (decode buffer -> staging), 1 = reverb window 0x2e0 -> 0x170
  bool naPolefFir = false;    //Paper Mario's 0x0e = 3-sample-delay lowpass (LSQ fit)
  //round 13 (Rare engines): 0x0e is a one-pole lowpass on the 0x170 buffer
  //(see the handler); enabled per ucode
  bool naPolefOnePole = false;
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

//index of the task being validated (for fallback diagnostics; set by the
//entry point before validation so oracle experiments can target the task)
u64 lumiverseAudioValidateTaskIndex = 0;

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
      fprintf(stderr, "[rsp-hle-audio] fallback: task %llu cmd %08x %08x at +%u (cmd %u): %s\n",
        (unsigned long long)lumiverseAudioValidateTaskIndex, w0, w1, offset, offset / 8, reason);
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
    case 0x09: {  //SETVOL (ABI1) / ABI2 op 0x09 (Zelda menus; see executor)
      if(!abi1) {
        //ABI2: cmd0 = 09 | flags(1..3) << 16 | dmem A; cmd1 = dmem B << 16 | count bytes
        //(observed only as 09 0[123] 0580 / 0600 0080 in OoT (U)/MQ menus)
        const u32 a = w0 & 0xffff, b = w1 >> 16, count = w1 & 0xffff;
        if(flags < 1 || flags > 3) return reject(offset, w0, w1, "abi2 op09 flags");
        if(!count || (count & 0xf) || a + count > 0x1000 || b + count * flags > 0x1000) return reject(offset, w0, w1, "abi2 op09 range");
        break;
      }
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
      //Mario Kart 64's ABI2 revision (round 11): every task opens with
      //`07000000 00000000` — the ABI1-style SEGMENT(0) with nothing to set;
      //no DMEM or RDRAM effect (truncated-task oracle). Accept as a no-op.
      if((w0 & 0x00ffffff) == 0 && w1 == 0) break;
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
    case 0x11: {  //DECIMATE (2:1 copy): count OUTPUT samples, reads 2*count
      //round 10: the source read wraps at the end of DMEM (12-bit RSP
      //addressing). Pokemon Stadium 2's reverb chain issues the in-place form
      //`11 0000d0 0e10 0e10` (208 output samples from 0xe10: the read runs to
      //0x1150); the truncated-task oracle (TRUNCATE_MATCH) shows LLE writing
      //all 208 outputs at 0xe10..0xfb0 with dst[n] = src[(0xe10 + 4n) & 0xfff]
      //(192/192 verifiable samples). Only the first 0x50 are consumed (the
      //SAVEBUFF that follows); the wrapped tail is dead data. Whole-task LLE
      //fallback for 291/3364 of its tasks before this.
      const u32 count = w0 & 0xffff;
      const u32 dst = w1 & 0xffff;
      if(!count || dst + count * 2 > 0x1000) return reject(offset, w0, w1, "decimate range");
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

//naudio dialect (Super Smash Bros. / Kirby 64 family). Fixed alist-space
//layout derived empirically (per-command DMEM oracles; every alist dmem
//field maps to physical DMEM at +0x4f0, so this interpreter uses the alist
//values literally in its own workspace):
//  0x000 staging (compressed input, then the resampler output = envmix in)
//  0x170 decode buffer (16-sample state prefix, data at 0x190)
//  0x4e0/0x650 wet L/R buses, 0x7c0/0x930 dry L/R buses (0x170 each)
//Commands (fields verified against isolated LLE runs):
//  0x02 CLEARBUFF dmem=cmd0.lo16, count=cmd1
//  0x04 LOADBUFF  count=cmd0 bits12-23, dmem=cmd0.lo12, RDRAM addr=cmd1
//  0x06 SAVEBUFF  same fields, direction reversed
//  0x0b LOADADPCM count bytes=cmd0.lo16, addr=cmd1 (book)
//  0x01 ADPCM     state addr=cmd0.lo24; cmd1 = [flags:4][outBytes:12]
//                 [flags:4][chunk 0x170]; decodes staged bytes to the decode
//                 buffer (state prefix + outBytes, 9-byte/16-sample frames)
//  0x05 RESAMPLE  state addr=cmd0.lo24; pitch=cmd1>>16 (Q13, 0x2000=1.0);
//                 reads the decode stream, writes 184 samples to staging
//  0x03 ENVMIX    cmd0 = [flags byte][init vol u16], cmd1 = RDRAM param
//                 block (per-lane vol int/frac vectors, targets, 16.16
//                 rates, wet/dry Q15); wet buses += (x*vol>>15)*wet>>15,
//                 dry buses += (x*vol>>15)*dry>>15
//  0x09 pending init-params for the next ENVMIX (flags 0x00/0x04/0x06)
//  0x0c MIXER     gain=s16 cmd0.lo16, in=cmd1.hi16, out=cmd1.lo16, 0x170 bytes
//  0x0d INTERLEAVE / 0x0e POLEF: see handlers
//round 13: the naudio-family resampler's interpolation kernel h(t), t = i/64
//for i = 0..128 (Q15), least-squares-fitted from truncated-task oracles of
//the reverb tap on our own LLE (Banjo-Kazooie phase sweep, 22 consecutive
//tasks at pitch 0x1fff: every phase gives the four taps at distances 1+f,
//f, 1-f, 2-f; the points folded on the mirror symmetry and smoothed).
//h(0) = 0.80, h(0.5) = 0.51, h(1) = 0.10, h(1.5) = -0.01, h(2) = 0.
//round 16: LUMIVERSE_ARES_N64_AUDIO_HLE_NA_POLEF_STATE (default 1): the Rare
//one-pole 0e keeps its history in the 8-byte RDRAM block at cmd1.lo24 (see
//the handler); 0 = the round-13 "w1 bit 24 = init" form
//round 16: LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_ENV_STATE (default 1): the ABI1
//ENVMIXER writes the microcode's 80-byte state block (see the handler)
auto lumiverseAudioAbiEnvState() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_ENV_STATE"); return !v || v[0] != '0'; }();
  return value;
}
auto lumiverseAudioNaPolefState() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_POLEF_STATE"); return !v || v[0] != '0'; }();
  return value;
}
auto lumiverseAudioNaStateWrites() -> bool;

auto lumiverseAudioNaKernelAt(u32 t) -> s32 {  //t in Q16 samples
  static const s16 kernel[129] = {
    26177,26152,26065,25986,25944,25954,25865,25724,25543,25386,25204,25003,24773,24487,24174,23843,
    23547,23232,22894,22570,22175,21742,21358,20896,20425,20012,19596,19169,18725,18287,17728,17090,
    16719,16347,15703,15132,14672,14171,13690,13223,12769,12256,11746,11301,10771,10279,9868,9417,
    8991,8581,8113,7659,7229,6858,6485,6127,5800,5393,5036,4746,4489,4218,3934,3630,
    3340,3075,2864,2657,2421,2141,1941,1734,1523,1359,1184,1015,884,768,657,549,
    451,346,232,154,80,12,-38,-64,-88,-132,-170,-216,-272,-291,-307,-321,
    -327,-332,-341,-345,-343,-343,-335,-324,-329,-315,-296,-286,-259,-239,-231,-235,
    -230,-216,-189,-159,-132,-123,-121,-119,-113,-95,-84,-80,-89,-91,-85,-52,0 };
  if(t >= 0x20000) return 0;
  const u32 index = t >> 10, f = t & 0x3ff;
  return (kernel[index] * (s32)(0x400 - f) + kernel[index + 1] * (s32)f) >> 10;
}

auto lumiverseAudioExecuteNaudio(LumiverseAudioMachine& m, LumiverseAudioShadow& shadow, u32 w0, u32 w1) -> void {
  u8* dmem = m.dmem;
  const u8 op = w0 >> 24;
  constexpr u32 NaChunk = 0x170;
  constexpr u32 NaStaging = 0x000;
  constexpr u32 NaDecode = 0x170;

  switch(op) {
  case 0x02: {  //CLEARBUFF
    const u32 dmemAddr = w0 & 0xffff, count = w1 & 0xffff;
    for(u32 index = 0; index < count; index++) dmem[(dmemAddr + index) & 0xfff] = 0;
    if(dmemAddr == NaDecode && count > 32) { m.naDecodeSamples = (count - 32) / 2; m.naSrcMode = 0; }
    break;
  }
  case 0x04: {  //LOADBUFF
    const u32 count = w0 >> 12 & 0xfff, dmemAddr = w0 & 0xfff;
    if(!count) break;
    u8 scratch[0x1000];
    lumiverseAudioReadRDRAM(shadow, w1 & 0x00ffffff, scratch, count);
    for(u32 index = 0; index < count; index++) dmem[(dmemAddr + index) & 0xfff] = scratch[index];
    if(dmemAddr == NaStaging) m.naLoadCount = count;
    if(dmemAddr >= 0x2e0 && dmemAddr < 0x300) m.naSrcMode = 1;
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
  case 0x0b: {  //LOADADPCM
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
  case 0x01: {  //ADPCM decode staged bytes -> decode buffer
    auto& state = lumiverseAudioState(w0 & 0x00ffffff);
    const u32 outBytes = w1 >> 16 & 0xfff;
    const u32 flags = w1 >> 28;         //bit 28 = voice start, bit 29 = loop (round 16)
    const bool init = flags & 1;
    //round 16 (Conker's Bad Fur Day, looping voices; truncated-task oracle
    //on task 494 commands 12-13): cmd1's low 12 bits are the OUTPUT address
    //(0x170 for a chunk's first piece — the constant the round-12 layout
    //note took it for). A voice whose loop point falls inside the chunk is
    //decoded in two pieces: `01 ... 00bc6170` (94 samples to the loop end,
    //data at 0x190..0x24c), `0f` SETLOOP, `01 ... 20cc4260` (flag 2 = the
    //16-sample history comes from the SETLOOP block; prefix at 0x260, data
    //at 0x280), then `0a 027e 024c 00ce` slides the second piece down to
    //butt against the first. LLE's prefix at 0x260 is byte-for-byte the
    //SETLOOP block (0x2410 0x0bc3 0xf004 ...). A count that is not a frame
    //multiple decodes the whole final frame (0xbc -> 6 frames), like the
    //ABI executor; the slide overwrites the excess.
    const u32 outBase = w1 & 0xfff;
    s16 last[16];
    if(init) for(auto& sample : last) sample = 0;
    else if(flags & 2) {
      u8 raw[32];
      lumiverseAudioReadRDRAM(shadow, m.loopAddr, raw, 32);
      for(u32 index = 0; index < 16; index++) last[index] = (s16)((u16)raw[index * 2] << 8 | raw[index * 2 + 1]);
    }
    else for(u32 index = 0; index < 16; index++) last[index] = state.samples[index];
    //state prefix at the output base
    for(u32 index = 0; index < 16; index++) lumiverseAudioDmemWriteS16(dmem, outBase + index * 2, last[index]);
    s32 prev2 = last[14], prev1 = last[15];
    //cmd1 bits 12-15 = byte offset of the first frame inside the staged
    //block (the loader DMAs 8-byte-aligned; verified bit-exact against LLE
    //decodes at offsets 0 and 2)
    u32 in = NaStaging + (w1 >> 12 & 0xf);
    u32 out = outBase + 32;
    const u32 frames = (outBytes + 31) / 32;
    for(u32 frame = 0; frame < frames; frame++) {
      const u8 header = dmem[in++ & 0xfff];
      const u32 scale = header >> 4;
      u32 predictor = header & 0xf;
      if(predictor * 16 + 16 > m.bookEntries) predictor = 0;
      const s16* row0 = &m.book[predictor * 16];
      const s16* row1 = &m.book[predictor * 16 + 8];
      s32 residual[16];
      for(u32 byteIndex = 0; byteIndex < 8; byteIndex++) {
        const u8 byte = dmem[in++ & 0xfff];
        s32 hi = byte >> 4, lo = byte & 0xf;
        if(hi >= 8) hi -= 16;
        if(lo >= 8) lo -= 16;
        residual[byteIndex * 2 + 0] = hi << scale;
        residual[byteIndex * 2 + 1] = lo << scale;
      }
      for(u32 half = 0; half < 2; half++) {
        const s32* r = &residual[half * 8];
        s16 decoded[8];
        for(u32 j = 0; j < 8; j++) {
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
    for(u32 index = 0; index < 16; index++) {
      state.samples[index] = lumiverseAudioDmemReadS16(dmem, out - 32 + index * 2);
    }
    if(lumiverseAudioNaStateWrites()) {
      u8 blob[32];
      for(u32 index = 0; index < 16; index++) { blob[index * 2] = (u16)state.samples[index] >> 8; blob[index * 2 + 1] = (u8)state.samples[index]; }
      lumiverseAudioWriteRDRAM(shadow, w0 & 0x00ffffff, blob, 32);
    }
    //a second piece (the loop continuation above) extends the chunk's data
    if(outBase == NaDecode) m.naDecodeSamples = outBytes / 2;
    else m.naDecodeSamples += outBytes / 2;
    m.naSrcMode = 0;
    break;
  }
  case 0x0f: {  //SETLOOP (round 16): the loop-point history block for the next ADPCM
    m.loopAddr = w1 & 0x00ffffff;
    break;
  }
  case 0x05: {  //RESAMPLE decode stream -> 184 samples at staging.
    //Positioning: cmd1.lo12 is the engine's own stream position in 1/8-sample
    //units (whole-sample quantized). The integer start is resynced from it
    //every chunk relative to a per-voice anchor captured at init (bit 14 of
    //the pitch field) — a free-running position model drifts against the
    //engine's avail-feed and eventually clamps audibly (the round-3 click
    //source). The sub-sample fraction is carried locally for smoothness.
    auto& state = lumiverseAudioState(w0 & 0x00ffffff);
    const u32 pitch = w1 >> 16 & 0x3fff;   //Q13, 0x2000 = 1.0
    const bool init = (w1 >> 16 & 0x4000) != 0;
    const s32 ptrWhole = (s32)((w1 & 0xfff) >> 3);
    s32 anchor;
    s64 position;
    //reverb form (round 12, Paper Mario oracle task 1440): source = the
    //window the engine loaded at 0x2e0 (8-12 history samples + ~176 data),
    //output at 0x170; LLE's out[n] tracks window[n-2], so the position
    //restarts two samples before the window each chunk (the engine slides
    //the RDRAM window itself); only the fraction is carried
    //(validated on Paper Mario only — gated to its ucode; Smash/Kirby/Ogre
    //never load a window at 0x2e0 in the runs gated so far, but stay on the
    //voice form regardless)
    //round 13 (Rare engines: Banjo-Kazooie/Tooie, Conker, Perfect Dark,
    //Donkey Kong 64 — the same naudio-family command set): the reverb tap
    //is this window form run as a true resampler whose state the microcode
    //keeps in RDRAM at cmd0.lo24 as 16 bytes: [4 history samples s16]
    //[fraction u16 Q16][3 unused]. Established with truncated-task oracles
    //on our own LLE (DEBUG=4 DMEM dumps, LLE's 0x170 output least-squares-
    //fitted against the loaded window, residual 0.5 LSB rms in every case;
    //22 consecutive tasks of one Banjo-Kazooie voice plus single tasks of
    //the other four titles):
    //  * read position P(n) = n + (ptr - 371) + frac, ptr = cmd1.lo12 >> 3
    //    (0xb99 = 371 -> n + frac with a 187-sample window, 0xb81 = 368 ->
    //    n - 3 + frac with a 184-sample window); the fraction is constant
    //    through the chunk and is the value READ from the state;
    //  * the interpolation kernel is a smooth 4-tap window h(t) with
    //    h(0) = 0.80, h(0.5) = 0.51, h(1) = 0.10, h(1.5) = -0.01, h(2) = 0
    //    (the per-phase taps below, mirror-symmetric in the phase), i.e.
    //    the tap band-limits even at integer phase — the property the
    //    round-12 Paper Mario loop lacked;
    //  * after the chunk the stored fraction moves by 184 * 2 * (pitch -
    //    0x2000) (Q16, wrapping: 0x1fff -> -0x170 per chunk, the chorus
    //    drift the engine compensates with its 0xb81/0xb99 pointer forms);
    //  * the stored history is the last four samples the read reached,
    //    W[183 + (ptr - 371) + 0..3]; samples before the window (n + base
    //    + frac < 0 for the 0xb81 form) come from the history;
    //  * pitch bit 14 (0x6000 forms, first task) = init: zero state.
    //Bits 14/15 of the pointer halfword (0x4b99 / 0xcb99) do not change the
    //fitted behaviour and are ignored.
    const bool rareWindow = m.naSrcMode == 1 && !m.naPolefFir;
    if(rareWindow) {
      auto kernelAt = lumiverseAudioNaKernelAt;
      if(false) {
      static const s16 kernel[129] = {
        26177,26152,26065,25986,25944,25954,25865,25724,25543,25386,25204,25003,24773,24487,24174,23843,
        23547,23232,22894,22570,22175,21742,21358,20896,20425,20012,19596,19169,18725,18287,17728,17090,
        16719,16347,15703,15132,14672,14171,13690,13223,12769,12256,11746,11301,10771,10279,9868,9417,
        8991,8581,8113,7659,7229,6858,6485,6127,5800,5393,5036,4746,4489,4218,3934,3630,
        3340,3075,2864,2657,2421,2141,1941,1734,1523,1359,1184,1015,884,768,657,549,
        451,346,232,154,80,12,-38,-64,-88,-132,-170,-216,-272,-291,-307,-321,
        -327,-332,-341,-345,-343,-343,-335,-324,-329,-315,-296,-286,-259,-239,-231,-235,
        -230,-216,-189,-159,-132,-123,-121,-119,-113,-95,-84,-80,-89,-91,-85,-52,0 };
      (void)kernel;
      }
      const u32 stateAddr = w0 & 0x00ffffff;
      u8 blob[16];
      s32 hist[4] = {};
      u32 frac = 0;
      if(!init) {
        lumiverseAudioReadRDRAM(shadow, stateAddr, blob, 16);
        for(u32 j = 0; j < 4; j++) hist[j] = (s16)((u16)blob[j * 2] << 8 | blob[j * 2 + 1]);
        frac = (u16)((u16)blob[8] << 8 | blob[9]);
      }
      const s32 base = ptrWhole - 371;
      auto win = [&](s32 index) -> s32 {
        if(index >= 0) return lumiverseAudioDmemReadS16(dmem, (0x2e0 + index * 2) & 0xfff);
        index += 4;
        return hist[index < 0 ? 0 : index];
      };
      //taps for sample offsets -1, 0, +1, +2 from floor(P): distances 1+f, f, 1-f, 2-f
      s32 taps[4] = { kernelAt(0x10000 + frac), kernelAt(frac), kernelAt(0x10000 - frac), kernelAt(0x20000 - frac) };
      const s32 sum = taps[0] + taps[1] + taps[2] + taps[3];
      if(sum > 0) for(auto& tap : taps) tap = (s32)(((s64)tap << 15) / sum);
      for(u32 n = 0; n < NaChunk / 2; n++) {
        const s32 c = (s32)n + base;
        const s64 acc = (s64)taps[0] * win(c - 1) + (s64)taps[1] * win(c) + (s64)taps[2] * win(c + 1) + (s64)taps[3] * win(c + 2);
        lumiverseAudioDmemWriteS16(dmem, 0x170 + n * 2, lumiverseAudioClamp16((s32)((acc + 0x4000) >> 15)));
      }
      //state write-back
      const u32 nextFrac = (u32)((s32)frac + 2 * (s32)(NaChunk / 2) * ((s32)pitch - 0x2000)) & 0xffff;
      for(u32 j = 0; j < 4; j++) {
        const s32 v = win(183 + base + (s32)j);
        blob[j * 2] = (u16)v >> 8; blob[j * 2 + 1] = (u8)v;
      }
      blob[8] = nextFrac >> 8; blob[9] = (u8)nextFrac;
      for(u32 j = 10; j < 16; j++) blob[j] = 0;
      lumiverseAudioWriteRDRAM(shadow, stateAddr, blob, 16);
      break;
    }
    const bool reverb = m.naSrcMode == 1 && m.naPolefFir;
    if(reverb) {
      static const s32 phase = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_REVERB_PHASE"); return v ? ::atoi(v) : -2; }();
      const u32 stepR = pitch << 3;
      s64 pos = ((s64)phase << 16) + (init ? 0 : (s32)(state.frac & 0xffff));
      auto at = [&](s32 index) -> s32 {
        if(index < -8) index = -8;
        return lumiverseAudioDmemReadS16(dmem, 0x2e0 + index * 2);
      };
      for(u32 n = 0; n < NaChunk / 2; n++) {
        const s32 index = (s32)(pos >> 16);
        const s64 f = pos - ((s64)index << 16);
        const s64 x0 = at(index - 2), x1 = at(index - 1), x2 = at(index), x3 = at(index + 1);
        const s64 f2 = f * f >> 16, f3 = f2 * f >> 16;
        const s64 catmull = x1 + ((f * (x2 - x0)) >> 17) + ((f2 * (2 * x0 - 5 * x1 + 4 * x2 - x3)) >> 17) + ((f3 * (3 * (x1 - x2) + x3 - x0)) >> 17);
        s64 y = catmull;
        const s64 lower = x1 < x2 ? x1 : x2, upper = x1 < x2 ? x2 : x1;
        if(y < lower) y = lower;
        if(y > upper) y = upper;
        lumiverseAudioDmemWriteS16(dmem, 0x170 + n * 2, lumiverseAudioClamp16((s32)y));
        pos += stepR;
      }
      state.frac = (u32)(pos & 0xffff);
      break;
    }
    //round 13 (Banjo-Tooie voice oracle, task 64 cmd 140): LLE's first
    //output of a freshly started voice interpolates from three samples
    //BEFORE the data start (the zeroed state prefix): out[n] = D[n*pitch - 3]
    //(linear fit, 33 LSB rms) — the microcode resampler's FIR group delay.
    //LUMIVERSE_ARES_N64_AUDIO_HLE_NA_VOICE_PHASE (samples, default 3).
    //round 13: with the measured kernel (below) the shadow's bad-write sum
    //is lowest at phase 2 (Smash 37.0k/34.8k/22.1k/35.4k and Banjo-Tooie
    //37.9k/35.7k/28.5k/37.3k for phases 0/1/2/3 over 3000 steps).
    static const s32 voicePhase = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_VOICE_PHASE"); return v ? ::atoi(v) : 2; }();
    if(init) {
      anchor = ptrWhole;        //stream position 0 = data start at init
      position = -((s64)voicePhase << 16);
      state.samples[1] = (s16)anchor;
    } else {
      anchor = state.samples[1];
      position = (s64)(s32)state.frac;   //continuous Q16 stream position
      //voice first seen mid-stream (HLE enabled late): adopt the pointer
      if(anchor == 0 && state.samples[2] == 0) { anchor = ptrWhole; position = 0; state.samples[1] = (s16)anchor; }
      //drift servo: the command's 1/8-sample pointer carries the engine's
      //own whole-sample start; resync only past a 2-sample tolerance so
      //ordinary chunks stay sample-continuous (per-chunk snapping produced
      //audible seams on noise content)
      const s32 wholeStart = ptrWhole - anchor - voicePhase;
      const s32 err = wholeStart - (s32)(position >> 16);
      if(err > 2 || err < -2) position = ((s64)wholeStart << 16) | (position & 0xffff);
    }
    state.samples[2] = 1;       //anchor-valid marker
    const u32 dataStart = NaDecode + 32;
    const u32 step = pitch << 3;           //Q13 -> Q16 per output sample
    //band-limit when consuming faster than 1:1 — the microcode's short FIR
    //rolls off high frequencies, and skipping that on downsampled NOISE
    //content leaves aliased full-band energy (measurably more large
    //sample-to-sample jumps than the LLE render)
    const bool prefilter = pitch >= 0x2000;  //at or above 1:1
    auto sampleRaw = [&](s32 index) -> s32 {
      if(index < -16) index = -16;
      return lumiverseAudioDmemReadS16(dmem, dataStart + index * 2);
    };
    auto sampleAt = [&](s32 index) -> s32 {
      if(!prefilter) return sampleRaw(index);
      return (sampleRaw(index - 1) + 2 * sampleRaw(index) + sampleRaw(index + 1)) >> 2;
    };
    //round 13: the microcode's own interpolation kernel, measured on the
    //reverb tap of the same command (see the window form above), applied
    //at the same effective position (index - 1) + f as the cubic below;
    //it band-limits by itself (0.1/0.8/0.1 at integer phase), so the 1-2-1
    //prefilter is not used with it. LUMIVERSE_ARES_N64_AUDIO_HLE_NA_KERNEL
    //(default 1; 0 = the round-4 Mitchell-blend cubic + prefilter).
    static const bool measuredKernel = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_KERNEL"); return !v || v[0] != '0'; }();
    if(measuredKernel) {
      for(u32 n = 0; n < NaChunk / 2; n++) {
        const s32 index = (s32)(position >> 16);
        const u32 f = (u32)(position - ((s64)index << 16));
        s32 taps[4] = { lumiverseAudioNaKernelAt(0x10000 + f), lumiverseAudioNaKernelAt(f), lumiverseAudioNaKernelAt(0x10000 - f), lumiverseAudioNaKernelAt(0x20000 - f) };
        const s32 sum = taps[0] + taps[1] + taps[2] + taps[3];
        if(sum > 0) for(auto& tap : taps) tap = (s32)(((s64)tap << 15) / sum);
        const s64 acc = (s64)taps[0] * sampleRaw(index - 2) + (s64)taps[1] * sampleRaw(index - 1) + (s64)taps[2] * sampleRaw(index) + (s64)taps[3] * sampleRaw(index + 1);
        lumiverseAudioDmemWriteS16(dmem, NaStaging + n * 2, lumiverseAudioClamp16((s32)((acc + 0x4000) >> 15)));
        position += step;
      }
      const s32 avail = (s32)(m.naDecodeSamples ? m.naDecodeSamples : NaChunk / 2);
      state.frac = (u32)(s32)(position - ((s64)avail << 16));
      if(lumiverseAudioNaStateWrites()) {
        const s32 last = (s32)((position - step) >> 16);
        u8 blob[16];
        for(u32 j = 0; j < 4; j++) { const s32 v = sampleRaw(last + (s32)j); blob[j * 2] = (u16)v >> 8; blob[j * 2 + 1] = (u8)v; }
        const u32 f = (u32)(position & 0xffff);
        blob[8] = f >> 8; blob[9] = (u8)f;
        for(u32 j = 0; j < 3; j++) { blob[10 + j * 2] = lumiverseAudioNaLastBlockLanes[j] >> 8; blob[11 + j * 2] = (u8)lumiverseAudioNaLastBlockLanes[j]; }
        lumiverseAudioWriteRDRAM(shadow, w0 & 0x00ffffff, blob, 16);
      }
      break;
    }
    for(u32 n = 0; n < NaChunk / 2; n++) {
      const s32 index = (s32)(position >> 16);
      const s64 f = position - ((s64)index << 16);
      const s64 x0 = sampleAt(index - 2);
      const s64 x1 = sampleAt(index - 1);
      const s64 x2 = sampleAt(index);
      const s64 x3 = sampleAt(index + 1);
      const s64 f2 = f * f >> 16, f3 = f2 * f >> 16;
      const s64 catmull = x1
        + ((f  * (x2 - x0)) >> 17)
        + ((f2 * (2 * x0 - 5 * x1 + 4 * x2 - x3)) >> 17)
        + ((f3 * (3 * (x1 - x2) + x3 - x0)) >> 17);
      const s64 oneMinusF = 0x10000 - f;
      const s64 omf2 = oneMinusF * oneMinusF >> 16, omf3 = omf2 * oneMinusF >> 16;
      const s64 bspline = (omf3 * x0
        + (0x40000 - 6 * f2 + 3 * f3) * x1
        + (0x10000 + 3 * f + 3 * f2 - 3 * f3) * x2
        + f3 * x3) / 6 >> 16;
      s64 y = (2 * catmull + bspline) / 3;
      const s64 lower = x1 < x2 ? x1 : x2;
      const s64 upper = x1 < x2 ? x2 : x1;
      if(y < lower) y = lower;
      if(y > upper) y = upper;
      lumiverseAudioDmemWriteS16(dmem, NaStaging + n * 2, lumiverseAudioClamp16((s32)y));
      position += step;
    }
    //carry the continuous position, rebased past the data this chunk provided
    const s32 avail = (s32)(m.naDecodeSamples ? m.naDecodeSamples : NaChunk / 2);
    state.frac = (u32)(s32)(position - ((s64)avail << 16));
    break;
  }
  case 0x09: {  //pending envmix init-params
    const u32 flags = w0 >> 16 & 0xff;
    const s32 value = (s16)(w0 & 0xffff);
    if(lumiverseAudioHLEDebug() >= 3) {
      static u32 lines = 0;
      if(m.tasksExecuted >= 200 && lines < 200) { lines++; fprintf(stderr, "[rsp-hle-audio-env] task %llu cmd %08x %08x (09)\n", (unsigned long long)m.tasksExecuted, w0, w1); }
    }
    //mapping verified against an observed init (voice block 0x8d130):
    //f=00 carries targetL+rateL, f=04 targetR+rateR, f=06 the starting
    //volL plus wet/dry gains; the ENVMIX command embeds the starting volR
    switch(flags) {
    case 0x00: m.naPendTargetL = value; m.naPendRateL = w1; break;
    case 0x04: m.naPendTargetR = value; m.naPendRateR = w1; break;
    case 0x06: m.naPendVolL = value;
               m.naPendWet = (s16)(w1 >> 16); m.naPendDry = (s16)(w1 & 0xffff); break;
    }
    break;
  }
  case 0x03: {  //ENVMIX using the RDRAM parameter block
    const u32 flags = w0 >> 16 & 0xff;
    const u32 address = w1 & 0x00ffffff;
    u8 raw[0x50];
    lumiverseAudioReadRDRAM(shadow, address, raw, 0x50);
    auto rd16 = [&](u32 offset) -> s32 { return (s16)((u16)raw[offset] << 8 | raw[offset + 1]); };
    auto rdu16 = [&](u32 offset) -> u32 { return (u16)((u16)raw[offset] << 8 | raw[offset + 1]); };
    //round 12 diagnostic (DEBUG>=3): the command, the pending 09 params and
    //the block as the game left it, for the envelope-model fits
    if(lumiverseAudioHLEDebug() >= 3) {
      static u32 lines = 0;
      if(m.tasksExecuted >= 200 && lines < lumiverseAudioEnvLineCap()) {
        lines++;
        fprintf(stderr, "[rsp-hle-audio-env] task %llu cmd %08x %08x addr %06x pend L=%04x/%08x R=%04x/%08x volL=%04x wet=%04x dry=%04x | before",
          (unsigned long long)m.tasksExecuted, w0, w1, address, (u16)m.naPendTargetL, m.naPendRateL, (u16)m.naPendTargetR, m.naPendRateR,
          (u16)m.naPendVolL, (u16)m.naPendWet, (u16)m.naPendDry);
        for(u32 o = 0x00; o < 0x50; o += 2) fprintf(stderr, "%s %04x", o % 16 == 0 ? " |" : "", rdu16(o));
        fprintf(stderr, "\n");
      }
    }
    //round 12 (Ogre Battle / Paper Mario block dumps under DEBUG>=3, then
    //shadow-exact on the 80-byte block writes): the ramp is ADDITIVE and
    //vectorised. The 8 lanes hold the volumes of the 8 samples of the
    //current group, lane i = base + (i+1)*step with step = rate/8 (16.16,
    //arithmetic shift: 0x003d8cfb -> +7.69/sample, 0xfffeb9e0 -> -0.16),
    //advanced by 8*step per group. The int part is clamped toward the
    //target while the fraction keeps accumulating (LLE's stored frac lanes
    //keep moving while its int lanes sit at the target). The stored block
    //holds the LAST group's lanes; a continuation reads them back and
    //advances one group first. Smash's round-4 "multiplicative" fit was the
    //same arithmetic (23 groups * -1.2734 = -29: 0x804 -> 0x7e7).
    //Init: volL from the 09/06 param, volR from the value embedded in this
    //command, targets/rates from 09/00 and 09/04 (targetR used to be taken
    //from the embedded value; Ogre's blocks carry 09/04's value there).
    //LUMIVERSE_ARES_N64_AUDIO_HLE_NA_ENV bits (model A/B): 1 = no
    //pre-increment on init, 2 = stored lanes already advanced.
    static const u32 envModel = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_ENV"); return v ? (u32)::atoi(v) : 0u; }();
    s64 vl[8], vr[8];
    s32 targetL, targetR, dry, wet;
    u32 rateL, rateR;
    auto stepOf = [](u32 rate) -> s64 { return (s64)((s32)rate >> 3); };
    if(flags & 1) {
      const s32 initVolR = (s16)(w0 & 0xffff);
      targetL = m.naPendTargetL; targetR = m.naPendTargetR;
      rateL = m.naPendRateL; rateR = m.naPendRateR;
      wet = m.naPendWet; dry = m.naPendDry;
      //round 13 (Banjo-Tooie / DK64 block oracles): the per-lane offsets
      //of lanes 0..6 are ((i+1) * rate) >> 16 * 0x2000, i.e. multiply
      //first, then shift (the old (i+1) * (rate >> 3) lost the rate's low
      //three bits: LLE's lanes sit +((i+1) * (rate mod 8)) >> 3 above it,
      //verified exactly on every init block of both titles). Lane 7 is NOT
      //(8 * rate) >> 3: LLE's lane 7 carries a residue of about
      //-ceil(rate / 65536) on negative rates (six cases, exact) but not on
      //rate 0x7fffffff (its fraction lane reads 0xffff there), and its
      //integer lane is one higher in about a third of the cases — the
      //16-bit multiplier form behind it is not established yet, so lane 7
      //keeps the round-12 rule. This is the last known difference of the
      //block LLE writes and is what still flips Banjo-Tooie's and DK64's
      //game path under HLE (they read the block back).
      //round 14 (2101 of 2101 fully-captured Banjo-Tooie / DK64 task blocks
      //exact, DEBUG=3 dumps replayed through a Python model of the block):
      //lane 7's multiplier is 0xffff — the (i+1)/8 Q16 table saturates at
      //its top entry — and every lane is formed in a 48-bit accumulator
      //(base << 16) + ((rate * mult) >> 16) whose 16-bit results are
      //RSP-clamped: the int part saturates to +-0x7fff/-0x8000 and the
      //fraction reads 0xffff (0x0000) when the int part overflowed
      //positive (negative). That is what put the "instant" rate
      //0x7fffffff's lane-7 fraction at 0xffff (base + 0x7fff7fff overflows
      //-> 0x7fff:ffff, the target clamp then lowers the int) while the
      //negative ramps carried rate - ceil(rate / 65536), and rate
      //0x80000000 left 0x0000:8000 (no overflow). LUMIVERSE_ARES_N64_
      //AUDIO_HLE_NA_LANE7=0 restores the round-12/13 lane-7 rule.
      static const bool lane7Model = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_LANE7"); return !v || v[0] != '0'; }();
      const s64 pre = (envModel & 1) ? 0 : 1;
      auto laneInit = [&](s32 base, u32 rate, u32 k) -> s64 {
        if(!lane7Model) {
          const s64 mult = (s64)k * 0x2000;
          return ((s64)base << 16) + (k == 8 ? 8 * stepOf(rate) : (((s64)(s32)rate * mult) >> 16));
        }
        const s64 mult = k >= 8 ? 0xffff : (s64)k * 0x2000;
        const s64 acc = ((s64)base << 16) + (((s64)(s32)rate * mult) >> 16);
        const s64 hi = acc >> 16;
        if(hi > 0x7fff) return ((s64)0x7fff << 16) | 0xffff;
        if(hi < -0x8000) return ((s64)-0x8000 << 16);
        return acc;
      };
      for(u32 i = 0; i < 8; i++) {
        vl[i] = laneInit(m.naPendVolL, rateL, i + pre);
        vr[i] = laneInit(initVolR, rateR, i + pre);
      }
    } else {
      targetL = rd16(0x40); rateL = rdu16(0x42) << 16 | rdu16(0x44);
      targetR = rd16(0x46); rateR = rdu16(0x48) << 16 | rdu16(0x4a);
      wet = rd16(0x4c); dry = rd16(0x4e);
      //round 13 (Banjo-Tooie per-chunk truncated-task oracles, tasks
      //200-202 voice 04f748): the per-GROUP advance is the 32-bit rate
      //itself, not 8 * (rate >> 3) — the latter drops the rate's low three
      //fraction bits (a clamped voice at rate 0x7fffffff moves its frac
      //lanes by -1 per group in LLE, -8 in the old model; ramps at
      //0xffd82519 / 0xffdfc0ec by +1 / +4 per group more than the old
      //model). The Rare engine reads these blocks back on the CPU, so the
      //frac lanes decide game-side behaviour (reverse-shadow frames of BT
      //and DK64 diverged until the blocks were LLE-owned).
      const s64 adv = (envModel & 2) ? 0 : 1;
      for(u32 i = 0; i < 8; i++) {
        vl[i] = (s64)(s32)(rdu16(0x00 + i * 2) << 16 | rdu16(0x10 + i * 2)) + adv * (s64)(s32)rateL;
        vr[i] = (s64)(s32)(rdu16(0x20 + i * 2) << 16 | rdu16(0x30 + i * 2)) + adv * (s64)(s32)rateR;
      }
    }
    const s64 stepL = stepOf(rateL), stepR = stepOf(rateR);
    const s64 groupL = (s64)(s32)rateL, groupR = (s64)(s32)rateR;
    //round 14: a zero rate clamps like a positive one (LLE's lanes drop to
    //the target on rate 0 blocks; 8 such blocks in the DK64 dump). Under the
    //NA_LANE7=0 A/B the round-13 rule (no clamp at rate 0) is kept too.
    static const bool zeroRateClamps = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_LANE7"); return !v || v[0] != '0'; }();
    auto clampLane = [](s64& v, s32 target, s64 step) {
      s32 vi = (s32)(v >> 16);
      if((step > 0 || (step == 0 && zeroRateClamps)) && vi > target) vi = target;
      if(step < 0 && vi < target) vi = target;
      if(vi > 0x7fff) vi = 0x7fff;
      if(vi < -0x8000) vi = -0x8000;
      v = ((s64)vi << 16) | (v & 0xffff);
    };
    constexpr u32 groups = NaChunk / 16;
    auto accumulate = [&](u32 bus, u32 n, s32 value) {
      const s32 current = lumiverseAudioDmemReadS16(dmem, bus + n * 2);
      lumiverseAudioDmemWriteS16(dmem, bus + n * 2, lumiverseAudioClamp16(current + value));
    };
    for(u32 g = 0; g < groups; g++) {
      if(g) for(u32 i = 0; i < 8; i++) { vl[i] += groupL; vr[i] += groupR; }
      for(u32 i = 0; i < 8; i++) { clampLane(vl[i], targetL, stepL); clampLane(vr[i], targetR, stepR); }
      for(u32 i = 0; i < 8; i++) {
        const u32 n = g * 8 + i;
        const s32 x = lumiverseAudioDmemReadS16(dmem, NaStaging + n * 2);
        const s32 vlInt = (s32)(vl[i] >> 16), vrInt = (s32)(vr[i] >> 16);
        const s32 l = x * vlInt + 0x4000 >> 15;
        const s32 r = x * vrInt + 0x4000 >> 15;
        accumulate(0x4e0, n, l * wet + 0x4000 >> 15);
        accumulate(0x650, n, r * wet + 0x4000 >> 15);
        accumulate(0x7c0, n, l * dry + 0x4000 >> 15);
        accumulate(0x930, n, r * dry + 0x4000 >> 15);
      }
    }
    //write the block back: the last group's lanes (clamped int, raw frac)
    u8 outBlock[0x50];
    auto wr16 = [&](u32 offset, u32 value) { outBlock[offset] = value >> 8; outBlock[offset + 1] = (u8)value; };
    for(u32 lane = 0; lane < 8; lane++) {
      wr16(0x00 + lane * 2, (u32)(vl[lane] >> 16));
      wr16(0x10 + lane * 2, (u32)(vl[lane] & 0xffff));
      wr16(0x20 + lane * 2, (u32)(vr[lane] >> 16));
      wr16(0x30 + lane * 2, (u32)(vr[lane] & 0xffff));
    }
    wr16(0x40, (u32)targetL); wr16(0x42, rateL >> 16); wr16(0x44, rateL & 0xffff);
    wr16(0x46, (u32)targetR); wr16(0x48, rateR >> 16); wr16(0x4a, rateR & 0xffff);
    wr16(0x4c, (u32)wet); wr16(0x4e, (u32)dry);
    lumiverseAudioWriteRDRAM(shadow, address, outBlock, 0x50, lumiverseAudioOwnDmem() >= 0);
    for(u32 j = 0; j < 3; j++) lumiverseAudioNaLastBlockLanes[j] = (u16)outBlock[(5 + j) * 2] << 8 | outBlock[(5 + j) * 2 + 1];
    if(lumiverseAudioHLEDebug() >= 3) {
      static u32 lines = 0;
      if(m.tasksExecuted >= 200 && lines < lumiverseAudioEnvLineCap()) {
        lines++;
        fprintf(stderr, "[rsp-hle-audio-env] task %llu addr %06x after", (unsigned long long)m.tasksExecuted, address);
        for(u32 o = 0x00; o < 0x50; o += 2) fprintf(stderr, "%s %04x", o % 16 == 0 ? " |" : "", (u32)((u16)outBlock[o] << 8 | outBlock[o + 1]));
        fprintf(stderr, "\n");
      }
    }
    break;
  }
  case 0x0c: {  //MIXER (0x170 bytes)
    const s32 gain = (s16)(w0 & 0xffff);
    const u32 in = w1 >> 16 & 0xffff, out = w1 & 0xffff;
    for(u32 index = 0; index < NaChunk; index += 2) {
      const s32 x = lumiverseAudioDmemReadS16(dmem, in + index);
      const s32 y = lumiverseAudioDmemReadS16(dmem, out + index);
      lumiverseAudioDmemWriteS16(dmem, out + index, lumiverseAudioClamp16(y + (x * gain + 0x4000 >> 15)));
    }
    break;
  }
  case 0x0a: {  //DMEMMOVE (in=cmd0.lo16, out=cmd1.hi16; the reversed
                //direction A/B-tested 2.6x worse against LLE output)
    const u32 in = w0 & 0xffff, out = w1 >> 16 & 0xffff, count = w1 & 0xffff;
    u8 scratch[0x1000];
    for(u32 index = 0; index < count; index++) scratch[index] = dmem[(in + index) & 0xfff];
    for(u32 index = 0; index < count; index++) dmem[(out + index) & 0xfff] = scratch[index];
    break;
  }
  case 0x0d: {  //INTERLEAVE: fixed buffers (fields carry unrelated values in
                //the observed streams): L=0x4e0, R=0x650 -> stereo at 0x000
    s16 scratch[NaChunk];
    for(u32 n = 0; n < NaChunk / 2; n++) {
      scratch[n * 2 + 0] = lumiverseAudioDmemReadS16(dmem, 0x4e0 + n * 2);
      scratch[n * 2 + 1] = lumiverseAudioDmemReadS16(dmem, 0x650 + n * 2);
    }
    for(u32 n = 0; n < NaChunk; n++) lumiverseAudioDmemWriteS16(dmem, NaStaging + n * 2, scratch[n]);
    break;
  }
  case 0x0e: {  //POLEF: an isolated LLE run showed NO dmem buffer writes
                //from this op in the observed stream (internal state only),
                //so it is treated as a no-op pending better evidence — the
                //in-place-filter guess measurably roughened the output.
    //Paper Mario (round 12): its reverb loop runs 0x0e on the 0x170 buffer
    //in place (table [8 zeros][0.625^(j+1) Q14], gain 0x1800). The
    //truncated-task oracle (task 1146) shows LLE's output is NOT the ABI1
    //one-pole: a least-squares fit over the chunk gives y[n] = FIR(x) with
    //taps (x16384) -150 -398 4813 9583 1692 -817 -65 121 on x[n-0..7] plus
    //0.098*y[n-1], residual 0.4 LSB — a unity-DC lowpass with a 3-sample
    //group delay. Applied only for PM's ucode (naPolefFir); w1 bit 24 = init
    //(history zero).
    //round 13 (Rare engines): the same command in Conker's Bad Fur Day is a
    //plain one-pole lowpass on the 0x170 buffer in place — truncated-task
    //oracle before/after the command (task 700): y[n] = (g*x[n] + a*y[n-1])
    //>> 14 with g = cmd0.lo16 (0x2d80) and a = table[8] of the 16-entry
    //table the preceding 0b loaded (0x1280 = 0.289; the table is
    //[8 zeros][a^(j+1) Q14] and g = 1 - a, a unity-DC lowpass); LSQ fit
    //g 11649 / a 4732 (x16384) residual 0.49 LSB, the state = y[n-1].
    //w1 bit 24 = init (history zero). Enabled per ucode (naPolefOnePole).
    //round 16 (Conker's Bad Fur Day, first-area tasks): w1 bit 24 is NOT an
    //init flag — every one of Conker's 9632 gain-carrying 0e commands has it
    //set (the 206 without it have gain 0), and the truncated-task oracle
    //(task 493 command 34) shows LLE's first output y[0] = 1276 for x[0] =
    //1402 with g = 0.711 / a = 0.289, i.e. a carried y[-1] of ~965 where
    //this handler started from zero (996). The history lives in RDRAM: the
    //microcode writes 8 bytes (DMEM 0x7d8) to the command's cmd1.lo24
    //address after every chunk — the blob is the chunk's last four outputs
    //y[180..183] (our own carried y[183] sits within 6 LSB of its last
    //word) — and reads it back at the next chunk, so a CPU-side clear of
    //that block is what resets the filter. NA_POLEF_STATE=0 restores the
    //round-13 flag form. The state read sees this task's deferred writes.
    if(m.naPolefOnePole) {
      const u32 stateAddr = w1 & 0x00ffffff;
      auto& state = lumiverseAudioState(stateAddr);
      const s32 gain = (s16)(w0 & 0xffff);
      const s32 pole = m.bookEntries >= 16 ? m.book[8] : 0;
      const bool init = (w1 >> 24) & 1;
      s32 prevY;
      if(lumiverseAudioNaPolefState()) {
        u8 blob[8];
        lumiverseAudioReadRDRAM(shadow, stateAddr, blob, 8);
        prevY = (s16)((u16)blob[6] << 8 | blob[7]);
      } else {
        prevY = init ? 0 : state.samples[8];
      }
      for(u32 n = 0; n < NaChunk / 2; n++) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, 0x170 + n * 2);
        prevY = lumiverseAudioClamp16((s32)(((s64)gain * x + (s64)pole * prevY) >> 14));
        lumiverseAudioDmemWriteS16(dmem, 0x170 + n * 2, (s16)prevY);
      }
      state.samples[8] = (s16)prevY;
      if(lumiverseAudioNaPolefState() && lumiverseAudioNaStateWrites()) {
        u8 blob[8];
        for(u32 k = 0; k < 4; k++) {
          const s16 y = lumiverseAudioDmemReadS16(dmem, 0x170 + (NaChunk / 2 - 4 + k) * 2);
          blob[k * 2] = (u16)y >> 8; blob[k * 2 + 1] = (u8)y;
        }
        lumiverseAudioWriteRDRAM(shadow, stateAddr, blob, 8);
      }
      break;
    }
    if(m.naPolefFir) {
      auto& state = lumiverseAudioState(w1 & 0x00ffffff);
      static const s32 taps[8] = { -150, -398, 4813, 9583, 1692, -817, -65, 121 };
      constexpr s32 pole = 1604;
      const bool init = (w1 >> 24) & 1;
      s32 hist[8]; s32 prevY;
      if(init) { for(auto& h : hist) h = 0; prevY = 0; }
      else { for(u32 k = 0; k < 8; k++) hist[k] = state.samples[k]; prevY = state.samples[8]; }
      s16 out[NaChunk / 2];
      for(u32 n = 0; n < NaChunk / 2; n++) {
        s64 acc = (s64)prevY * pole;
        for(u32 k = 0; k < 8; k++) {
          const s32 xk = (s32)n >= (s32)k ? lumiverseAudioDmemReadS16(dmem, 0x170 + (n - k) * 2) : hist[7 - (k - n - 1)];
          acc += (s64)taps[k] * xk;
        }
        prevY = lumiverseAudioClamp16((s32)(acc >> 14));
        out[n] = (s16)prevY;
      }
      for(u32 k = 0; k < 8; k++) state.samples[k] = lumiverseAudioDmemReadS16(dmem, 0x170 + (NaChunk / 2 - 8 + k) * 2);
      state.samples[8] = (s16)prevY;
      for(u32 n = 0; n < NaChunk / 2; n++) lumiverseAudioDmemWriteS16(dmem, 0x170 + n * 2, out[n]);
    }
    break;
  }
  default:
    break;
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
      if(!abi1) {
        //ABI2 (Zelda family) op 0x09 = DUPLICATE with SF64's 0x1a field
        //layout: src=cmd0.lo16, copies=flags, dst=cmd1.hi16, count=cmd1.lo16.
        //Empirical (OoT (U) file-select / pause menus): a LOADBUFF of 0x80
        //bytes to 0x580 is followed by 09 0[123] 0580 / 0600 0080 and then a
        //SETBUFF in=0x580 count=0x160 + RESAMPLE — a 64-sample looped SFX
        //waveform tiled to cover the resampler's input window. Shadow-mode
        //RDRAM compare turned the affected tasks' writes from bad to exact
        //with this implementation (round 6).
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
      //round 16: the per-lane volumes of the last 8-sample group, for the
      //microcode's state block (below)
      s64 laneL[8] = {}, laneR[8] = {};
      for(u32 n = 0; n < samples; n++) {
        const s32 x = lumiverseAudioDmemReadS16(dmem, m.inBuf + n * 2);
        const s32 vl = (s32)(vl32 >> 16), vr = (s32)(vr32 >> 16);
        laneL[n & 7] = vl32; laneR[n & 7] = vr32;
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
      //round 16 (San Francisco Rush): the ABI1 microcode writes an 80-byte
      //state block to the command's address after every ENVMIXER (LLE-only
      //write class 80/f90, 59 per task) and Rush's driver READS it — with
      //the block kept LLE-owned in reverse-shadow mode the race's fair gate
      //went from 0.976 (0.61 in the worst 5 s window) to 1.000 with identical
      //frames, so the game path depends on it. Layout from the DEBUG>=3 blob
      //probe next to this executor's state: [volL int x8 lanes][volL frac
      //x8][volR int x8][volR frac x8][targetL rateL.hi rateL.lo targetR
      //rateR.hi rateR.lo dry wet]; the lanes are the last group's per-lane
      //volumes (lane i = lane 0 * rate^(i/8), i.e. this executor's
      //per-sample ramp). At rest (rate 0x7fffffff, target reached) LLE
      //holds frac 0xffff. Written from this executor's model (within 0.1%
      //of LLE's lanes on the ramping blobs); not bit-exact.
      //LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_ENV_STATE=0 disables the write.
      if(lumiverseAudioAbiEnvState()) {
        u8 blob[80];
        auto put = [&](u32 offset, u32 value) { blob[offset] = (u8)(value >> 8); blob[offset + 1] = (u8)value; };
        const bool restL = rateL >= 0x10000 ? (s32)(vl32 >> 16) >= targetL : (s32)(vl32 >> 16) <= targetL;
        const bool restR = rateR >= 0x10000 ? (s32)(vr32 >> 16) >= targetR : (s32)(vr32 >> 16) <= targetR;
        for(u32 i = 0; i < 8; i++) {
          const s64 vl = samples >= 8 ? laneL[i] : laneL[0], vr = samples >= 8 ? laneR[i] : laneR[0];
          put(0x00 + i * 2, (u32)(vl >> 16)); put(0x10 + i * 2, restL ? 0xffff : (u32)(vl & 0xffff));
          put(0x20 + i * 2, (u32)(vr >> 16)); put(0x30 + i * 2, restR ? 0xffff : (u32)(vr & 0xffff));
        }
        put(0x40, (u32)targetL); put(0x42, rateL >> 16); put(0x44, rateL & 0xffff);
        put(0x46, (u32)targetR); put(0x48, rateR >> 16); put(0x4a, rateR & 0xffff);
        put(0x4c, (u32)dry); put(0x4e, (u32)wet);
        lumiverseAudioWriteRDRAM(shadow, address, blob, 80);
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
      auto sampleRaw = [&](u32 index) -> s32 {
        if(index < 8) return history[index];
        return lumiverseAudioDmemReadS16(dmem, in + (index - 8) * 2);
      };
      //band-limit when consuming faster than 1:1 (round 8): the microcode's
      //resampler is a short windowed FIR whose response rolls off before
      //Nyquist; a bare 4-tap interpolator leaves aliased full-band energy on
      //DOWNSAMPLED noise-like voices (Zelda's wind/water/insect ambience) —
      //measured as 1.2-1.7x the >8 kHz energy and 1.5x the large sample-to-
      //sample jumps of the LLE render in Kokiri Forest. Same original 1-2-1
      //prefilter the naudio path has used since round 4 (Smash/Kirby), where
      //it closed the identical gap. LUMIVERSE_ARES_N64_AUDIO_HLE_RESAMPLE_PREFILTER=0
      //disables it for A/B.
      //Kernel strength: (1, D-2, 1)/D. The plain 1-2-1 (D=4) overshoots on
      //Zelda (>8 kHz energy 0.5x LLE where it had been 1.3x); the default D
      //is the value that brought the LLE-vs-HLE >8 kHz ratio inside the
      //LLE-vs-LLE run-to-run band (see the round-8 report). =0 disables.
      static const u32 prefilterDenominator = [] () -> u32 {
        const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_RESAMPLE_PREFILTER");
        if(!v) return 16;
        const int value = ::atoi(v);
        if(value == 1) return 4;      //legacy "on" = 1-2-1
        return value < 3 ? 0 : (u32)value;
      }();
      const bool prefilter = prefilterDenominator && pitch > 0x8000;  //step > 1.0
      const s32 prefilterCenter = (s32)prefilterDenominator - 2;
      auto sampleAt = [&](u32 index) -> s32 {
        if(!prefilter) return sampleRaw(index);
        return (sampleRaw(index - 1) + prefilterCenter * sampleRaw(index) + sampleRaw(index + 1)) / (s32)prefilterDenominator;
      };
      //position base: history occupies virtual indices 0..7, dmem input
      //starts at index 8. The microcode's first output interpolates from two
      //input samples BEFORE the nominal `in` pointer (phase measured against
      //an isolated LLE resample via the truncated-task oracle: base 8 made
      //our output lead LLE's by 2 input samples), hence base 6.
      u64 position = ((u64)6 << 16) + frac;
      const u32 step = pitch << 1;  //Q16 samples per output sample
      //round 14: LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_KERNEL=1 runs this
      //resampler on the kernel measured on the naudio-family RESAMPLE
      //(round 13, lumiverseAudioNaKernelAt) at the same effective position
      //(x1 = index - 1, x2 = index), without the prefilter (the kernel
      //band-limits by itself). Default 1 since round 14: GoldenEye's Dam
      //clicks 221/254 -> 58/90 (LLE 69/91); SF64/OoT fair gates 1.000/0.999
      //with identical frames; ten-title ABI1/ABI2 real-gate regression
      //neutral (SM64/MK64/MM/WR/PW/D64/CU/PS same, Yoshi clicks 202->185).
      //0 = the round-4/8 Mitchell blend + prefilter.
      static const bool abiKernel = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_KERNEL"); return !v || v[0] != '0'; }();
      if(abiKernel) {
        for(u32 n = 0; n < outSamples; n++) {
          const u32 index = (u32)(position >> 16);
          const u32 f = (u32)(position & 0xffff);
          s32 taps[4] = { lumiverseAudioNaKernelAt(0x10000 + f), lumiverseAudioNaKernelAt(f), lumiverseAudioNaKernelAt(0x10000 - f), lumiverseAudioNaKernelAt(0x20000 - f) };
          const s32 sum = taps[0] + taps[1] + taps[2] + taps[3];
          if(sum > 0) for(auto& tap : taps) tap = (s32)(((s64)tap << 15) / sum);
          const s64 acc = (s64)taps[0] * sampleRaw(index - 2) + (s64)taps[1] * sampleRaw(index - 1) + (s64)taps[2] * sampleRaw(index) + (s64)taps[3] * sampleRaw(index + 1);
          lumiverseAudioDmemWriteS16(dmem, out + n * 2, lumiverseAudioClamp16((s32)((acc + 0x4000) >> 15)));
          position += step;
        }
      } else
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
      if((w0 & 0x00ffffff) == 0 && w1 == 0) break;  //MK64: SEGMENT(0) no-op
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

    case 0x11: {  //DECIMATE: dst[i] = src[2i] for count output samples.
      //Previously implemented as a plain sample copy ("COPY"), which is
      //WRONG: the truncated-task oracle (LUMIVERSE_ARES_N64_AUDIO_HLE_
      //TRUNCATE_OP=11) on both Zelda OoT (U) task 534 and Star Fox 64 shows
      //LLE writing exactly every second source sample (src 0x5a0 samples
      //0,2,4.. -> dst 0x3e0; src 0x610 -> dst 0x470). The alist uses it to
      //pre-halve voices whose pitch ratio exceeds the resampler's range
      //(the following RESAMPLE runs at half the nominal ratio), so the
      //plain copy played every such voice an OCTAVE LOW at half speed —
      //the "instrument bent flat" in Zelda's menus; MQ's intro exercises
      //it in 4356 of 9342 tasks. Even-sample phase; no filtering.
      const u32 count = w0 & 0xffff;
      const u32 src = w1 >> 16, dst = w1 & 0xffff;
      s16 scratch[0x800];
      //source reads wrap at 0x1000 (the accessor masks) — see the validator
      for(u32 index = 0; index < count && index < 0x800; index++) scratch[index] = lumiverseAudioDmemReadS16(dmem, src + index * 4);
      for(u32 index = 0; index < count && index < 0x800; index++) lumiverseAudioDmemWriteS16(dmem, dst + index * 2, scratch[index]);
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

//lowest zero-lag correlation among the last compared task's buffer writes
//(2.0 = nothing compared); the debug dump keys on it
f64 lumiverseAudioShadowWorstCorr = 2.0;

//LLE write set: every DMEM->RDRAM DMA the microcode issued for the shadowed
//task (recorded from dma.cpp while the compare is pending). Compared against
//our own write list to expose writes the HLE never makes at all — the
//shadow compare otherwise only judges writes we DO issue.
struct LumiverseAudioLLEWrite { u32 addr; u32 length; u32 dmem; };
LumiverseAudioLLEWrite lumiverseAudioLLEWrites[2048];
u32 lumiverseAudioLLEWriteCount = 0;
bool lumiverseAudioLLEWriteOverflow = false;
extern LumiverseAudioShadow lumiverseAudioShadowState;
//reverse-shadow side image (8 MiB, allocated on first use in level 3)
u8* lumiverseAudioSideImage = nullptr;
//round 14 diagnostic: LUMIVERSE_ARES_N64_AUDIO_HLE_OWN_DMEM=<hex> — in
//reverse-shadow mode, LLE writes of <= STATE_BYTES whose DMEM source is this
//address are NOT LLE-owned (they go to the side image, and our matching
//ENVMIX block writes go to RDRAM), to tell the ENVMIX blocks apart from the
//other writes of the same length
auto lumiverseAudioOwnDmem() -> s32 {
  static const s32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_OWN_DMEM"); return v ? (s32)::strtol(v, nullptr, 16) : -1; }();
  return value;
}
auto lumiverseAudioDivertLLEWrite(u32 dramAddress, u32 length, u32 dmemAddress) -> u8* {
  if(lumiverseAudioHLELevel() != 3 || !lumiverseAudioShadowState.pending) return nullptr;
  //LUMIVERSE_ARES_N64_AUDIO_HLE_STATE_BYTES (default 32): writes up to this
  //length stay LLE-owned in reverse-shadow mode (round 13: 80 tests whether
  //a game reads the naudio ENVMIX parameter blocks)
  const bool ownDmem = lumiverseAudioOwnDmem() >= 0 && length == 80 && (s32)(dmemAddress & 0xfff) == lumiverseAudioOwnDmem();
  if(length <= lumiverseAudioStateBytes() && !ownDmem) return nullptr;  //state blobs stay in RDRAM: the microcode's own next task reads them
  lumiverseAudioReleaseDeferredWrites(dramAddress & 0x00ffffff, length);
  if(!lumiverseAudioSideImage) lumiverseAudioSideImage = new u8[0x800000]();
  return lumiverseAudioSideImage;
}
//byte of the LLE result for a compare: RDRAM in shadow mode, the side image
//in reverse-shadow mode
auto lumiverseAudioTheirsByte(u32 address) -> u8 {
  if(lumiverseAudioHLELevel() == 3 && lumiverseAudioSideImage) return lumiverseAudioSideImage[address & 0x007fffff];
  return lumiverseAudioRDRAMReadByte(address);
}
auto lumiverseAudioNoteLLEWrite(u32 dramAddress, u32 length, u32 dmemAddress) -> void {
  if(!lumiverseAudioShadowState.pending) return;
  if(lumiverseAudioLLEWriteCount >= 2048) { lumiverseAudioLLEWriteOverflow = true; return; }
  auto& w = lumiverseAudioLLEWrites[lumiverseAudioLLEWriteCount++];
  w.addr = dramAddress & 0x00ffffff; w.length = length; w.dmem = dmemAddress & 0xfff;
}

//running aggregate of buffer-write statistics (diagnostic summary)
struct LumiverseAudioShadowAggregate {
  u64 writes = 0;
  f64 energyMine = 0, energyTheirs = 0;
  f64 dEnergyMine = 0, dEnergyTheirs = 0;
  f64 chEnergyMine[2] = {}, chEnergyTheirs[2] = {};
  f64 chDEnergyMine[2] = {}, chDEnergyTheirs[2] = {};
  u64 satMine = 0, satTheirs = 0;
  struct Class { u32 length; u64 writes; f64 energyMine, energyTheirs, dEnergyMine, dEnergyTheirs; } classes[12];
  u32 classCount = 0;
};
LumiverseAudioShadowAggregate lumiverseAudioShadowAgg;

auto lumiverseAudioShadowCompare(LumiverseAudioShadow& shadow) -> void {
  if(!shadow.pending) return;
  shadow.pending = false;
  //Writes are judged in two classes: sample BUFFERS (SAVEBUFF of mixed
  //output, > 32 bytes) and STATE blobs (<= 32 bytes: resampler/ADPCM
  //history saved for the next chunk). State blobs are private to the
  //microcode's own format — this interpreter keeps its own per-voice state
  //table — so they always differ and say nothing about audibility; only
  //buffer writes are reported in the "bad" figure. For a bad buffer write
  //the zero-lag correlation coefficient and RMS ratio are printed: an
  //approximation mismatch (resampler kernel) keeps corr ~0.99 / ratio ~1,
  //a wrong pitch or envelope collapses corr or moves the ratio.
  u32 exact = 0, close = 0, bad = 0, stateDiff = 0;
  s32 taskMax = 0;
  static u64 detailLines = 0;
  lumiverseAudioShadowWorstCorr = 2.0;
  //LLE's RDRAM is inspected only after the whole task ran, so a write that
  //a LATER write of the same task overlaps (delay-line traffic: save, then
  //load-mix-save to the same address — the Rare engines and the Zelda
  //reverb do this every task) can only be judged on the bytes no later
  //write covers. Bytes fully overwritten are skipped; a write with nothing
  //left to compare is counted as "covered".
  u32 covered = 0;
  static u8 lastWriter[LumiverseAudioShadow::MaxBytes];
  for(u32 w = 0; w < shadow.writeCount; w++) {
    const auto& write = shadow.writes[w];
    for(u32 index = 0; index < write.length; index++) lastWriter[write.offset + index] = 1;
    for(u32 later = w + 1; later < shadow.writeCount; later++) {
      const auto& other = shadow.writes[later];
      const u32 begin = other.addr > write.addr ? other.addr : write.addr;
      const u32 end = (other.addr + other.length) < (write.addr + write.length) ? (other.addr + other.length) : (write.addr + write.length);
      for(u32 a = begin; a < end; a++) lastWriter[write.offset + (a - write.addr)] = 0;
    }
    if(lumiverseAudioHLELevel() == 3) {
      //reverse shadow: only bytes the microcode actually wrote this task are
      //in the side image; anything else is stale from an earlier task
      for(u32 index = 0; index < write.length; index++) {
        if(!lastWriter[write.offset + index]) continue;
        const u32 a = write.addr + index;
        bool hit = false;
        for(u32 l = 0; l < lumiverseAudioLLEWriteCount && !hit; l++) {
          const auto& lw = lumiverseAudioLLEWrites[l];
          hit = lw.length > 32 && a >= lw.addr && a < lw.addr + lw.length;
        }
        if(!hit) lastWriter[write.offset + index] = 0;
      }
    }
    u32 comparable = 0;
    for(u32 index = 0; index < write.length; index++) comparable += lastWriter[write.offset + index];
    if(!comparable) { covered++; continue; }
    s32 maxDiff = 0;
    u32 firstBad = 0xffffffff;
    f64 sumMine2 = 0, sumTheirs2 = 0, sumCross = 0;
    //first-difference energy (a cheap high-frequency proxy): a resampler or
    //filter that passes more treble than the microcode's shows up as a
    //dratio > 1 even when the zero-lag correlation stays ~0.99
    f64 sumDMine2 = 0, sumDTheirs2 = 0;
    s32 prevMine = 0, prevTheirs = 0;
    u32 satMine = 0, satTheirs = 0;
    for(u32 index = 0; index + 1 < write.length; index += 2) {
      if(!lastWriter[write.offset + index]) { prevMine = prevTheirs = 0; continue; }
      const s16 mine = (s16)((u16)shadow.data[write.offset + index] << 8 | shadow.data[write.offset + index + 1]);
      const s16 theirs = (s16)((u16)lumiverseAudioTheirsByte(write.addr + index) << 8 | lumiverseAudioTheirsByte(write.addr + index + 1));
      sumMine2 += (f64)mine * mine;
      sumTheirs2 += (f64)theirs * theirs;
      sumCross += (f64)mine * theirs;
      if(index) {
        sumDMine2 += (f64)(mine - prevMine) * (mine - prevMine);
        sumDTheirs2 += (f64)(theirs - prevTheirs) * (theirs - prevTheirs);
      }
      prevMine = mine; prevTheirs = theirs;
      if(mine >= 32700 || mine <= -32700) satMine++;
      if(theirs >= 32700 || theirs <= -32700) satTheirs++;
      s32 diff = (s32)mine - theirs;
      if(diff < 0) diff = -diff;
      if(diff > maxDiff) { maxDiff = diff; if(firstBad == 0xffffffff && diff > 64) firstBad = index; }
    }
    const bool stateBlob = write.length <= 32;
    if(stateBlob) { if(maxDiff) stateDiff++; continue; }
    if(maxDiff > taskMax) taskMax = maxDiff;
    //aggregate spectral/level statistics over every buffer write with
    //signal on both sides (printed with the periodic summary line)
    if(sumTheirs2 > 0 && sumMine2 > 0) {
      //per write-length class (the alist's buffer roles are distinguishable
      //by size: main output chunks, reverb lines, aux buffers)
      {
        auto& a = lumiverseAudioShadowAgg;
        u32 c = 0;
        for(; c < a.classCount; c++) if(a.classes[c].length == write.length) break;
        if(c == a.classCount && a.classCount < 12) a.classes[a.classCount++] = {write.length, 0, 0, 0, 0, 0};
        if(c < a.classCount) {
          a.classes[c].writes++;
          a.classes[c].energyMine += sumMine2; a.classes[c].energyTheirs += sumTheirs2;
          a.classes[c].dEnergyMine += sumDMine2; a.classes[c].dEnergyTheirs += sumDTheirs2;
        }
      }
      lumiverseAudioShadowAgg.writes++;
      lumiverseAudioShadowAgg.energyMine += sumMine2;
      lumiverseAudioShadowAgg.energyTheirs += sumTheirs2;
      lumiverseAudioShadowAgg.dEnergyMine += sumDMine2;
      lumiverseAudioShadowAgg.dEnergyTheirs += sumDTheirs2;
      lumiverseAudioShadowAgg.satMine += satMine;
      lumiverseAudioShadowAgg.satTheirs += satTheirs;
      //per-parity split for interleaved stereo buffers (L = even sample)
      f64 e[2][2] = {}, d[2][2] = {};
      s32 pm[2] = {}, pt[2] = {};
      for(u32 index = 0; index + 1 < write.length; index += 2) {
        if(!lastWriter[write.offset + index]) continue;
        const u32 ch = (index >> 1) & 1;
        const s16 mine = (s16)((u16)shadow.data[write.offset + index] << 8 | shadow.data[write.offset + index + 1]);
        const s16 theirs = (s16)((u16)lumiverseAudioTheirsByte(write.addr + index) << 8 | lumiverseAudioTheirsByte(write.addr + index + 1));
        e[ch][0] += (f64)mine * mine; e[ch][1] += (f64)theirs * theirs;
        if(index >= 4) { d[ch][0] += (f64)(mine - pm[ch]) * (mine - pm[ch]); d[ch][1] += (f64)(theirs - pt[ch]) * (theirs - pt[ch]); }
        pm[ch] = mine; pt[ch] = theirs;
      }
      for(u32 ch = 0; ch < 2; ch++) {
        lumiverseAudioShadowAgg.chEnergyMine[ch] += e[ch][0];
        lumiverseAudioShadowAgg.chEnergyTheirs[ch] += e[ch][1];
        lumiverseAudioShadowAgg.chDEnergyMine[ch] += d[ch][0];
        lumiverseAudioShadowAgg.chDEnergyTheirs[ch] += d[ch][1];
      }
    }
    //round 13 diagnostic (DEBUG>=3): every naudio ENVMIX block write, ours
    //and LLE's, matching or not (sequential pairing with the env lines)
    if(write.length == 80 && lumiverseAudioHLEDebug() >= 3) {
      static u32 allBlockLines = 0;
      if(allBlockLines < 20000) {
        allBlockLines++;
        u32 lleDmem = 0xfff;
        for(u32 l = 0; l < lumiverseAudioLLEWriteCount; l++) if(lumiverseAudioLLEWrites[l].addr == write.addr && lumiverseAudioLLEWrites[l].length == 80) { lleDmem = lumiverseAudioLLEWrites[l].dmem; break; }
        fprintf(stderr, "[rsp-hle-audio-shadow]     blockall addr=%06x dmem=%03x mine  :", write.addr, lleDmem);
        for(u32 b = 0; b < 80; b += 2) fprintf(stderr, "%s %04x", b % 16 == 0 ? " |" : "", (u16)((u16)shadow.data[write.offset + b] << 8 | shadow.data[write.offset + b + 1]));
        fprintf(stderr, "\n[rsp-hle-audio-shadow]     blockall theirs:");
        for(u32 b = 0; b < 80; b += 2) fprintf(stderr, "%s %04x", b % 16 == 0 ? " |" : "", (u16)((u16)lumiverseAudioTheirsByte(write.addr + b) << 8 | lumiverseAudioTheirsByte(write.addr + b + 1)));
        fprintf(stderr, "\n");
      }
    }
    if(maxDiff == 0) exact++;
    else if(maxDiff <= 64) close++;
    else {
      bad++;
      const f64 corr = (sumMine2 > 0 && sumTheirs2 > 0) ? sumCross / (sqrt(sumMine2) * sqrt(sumTheirs2)) : -2.0;
      if(corr > -2.0 && corr < lumiverseAudioShadowWorstCorr) lumiverseAudioShadowWorstCorr = corr;
      if(detailLines < (lumiverseAudioHLEDebug() >= 3 ? 400000u : 2000u)) {
        detailLines++;
        const f64 ratio = sumTheirs2 > 0 ? sqrt(sumMine2 / sumTheirs2) : -1.0;
        const f64 dratio = sumDTheirs2 > 0 ? sqrt(sumDMine2 / sumDTheirs2) : -1.0;
        fprintf(stderr, "[rsp-hle-audio-shadow]   write cmd=%u addr=%06x len=%u maxdiff=%d firstbad=+%u corr=%.3f ratio=%.3f dratio=%.3f sat=%u/%u\n",
          write.command, write.addr, write.length, maxDiff, firstBad, corr, ratio, dratio, satMine, satTheirs);
        //round 12 diagnostic (DEBUG>=3): the naudio ENVMIX parameter block
        //(80 bytes) side by side, ours vs LLE's, for the first few mismatches
        static u32 blockLines = 0;
        if(write.length == 80 && lumiverseAudioHLEDebug() >= 3 && blockLines < 4000) {
          blockLines++;
          fprintf(stderr, "[rsp-hle-audio-shadow]     block addr=%06x mine  :", write.addr);
          for(u32 b = 0; b < 80; b += 2) fprintf(stderr, "%s %04x", b % 16 == 0 ? " |" : "", (u16)((u16)shadow.data[write.offset + b] << 8 | shadow.data[write.offset + b + 1]));
          fprintf(stderr, "\n[rsp-hle-audio-shadow]     block theirs:");
          for(u32 b = 0; b < 80; b += 2) fprintf(stderr, "%s %04x", b % 16 == 0 ? " |" : "", (u16)((u16)lumiverseAudioTheirsByte(write.addr + b) << 8 | lumiverseAudioTheirsByte(write.addr + b + 1)));
          fprintf(stderr, "\n");
        }
      }
    }
  }
  //round 14: ours-only writes — bytes WE write that no LLE DMA of this task
  //touched (in reverse-shadow / real HLE those land in RDRAM the game may
  //own: a clobber). Counted per task; the first 200 printed at DEBUG>=2.
  {
    static u64 oursOnlyWrites = 0, oursOnlyBytes = 0, oursOnlyLines = 0;
    u32 taskOursOnly = 0;
    for(u32 w = 0; w < shadow.writeCount; w++) {
      const auto& write = shadow.writes[w];
      u32 uncovered = 0; u32 firstUncovered = 0xffffffff;
      for(u32 a = write.addr; a < write.addr + write.length; a++) {
        bool hit = false;
        for(u32 l = 0; l < lumiverseAudioLLEWriteCount && !hit; l++) {
          const auto& lw = lumiverseAudioLLEWrites[l];
          hit = a >= lw.addr && a < lw.addr + lw.length;
        }
        if(!hit) { uncovered++; if(firstUncovered == 0xffffffff) firstUncovered = a; }
      }
      if(uncovered && !lumiverseAudioLLEWriteOverflow) {
        oursOnlyWrites++; oursOnlyBytes += uncovered; taskOursOnly++;
        if(lumiverseAudioHLEDebug() >= 2 && oursOnlyLines < 200) {
          oursOnlyLines++;
          fprintf(stderr, "[rsp-hle-audio-shadow]   OURS-only write task=%llu cmd=%u addr=%06x len=%u uncovered=%u first=%06x\n",
            (unsigned long long)shadow.task, write.command, write.addr, write.length, uncovered, firstUncovered);
        }
      }
    }
    if(taskOursOnly && lumiverseAudioHLEDebug() >= 2 && (oursOnlyWrites & 255) == 0)
      fprintf(stderr, "[rsp-hle-audio-shadow] ours-only writes so far: %llu (%llu bytes)\n", (unsigned long long)oursOnlyWrites, (unsigned long long)oursOnlyBytes);
  }
  //LLE-only writes: bytes the microcode DMA'd out that no write of ours covers
  static u64 lleWrites = 0, lleOnlyWrites = 0, lleOnlyBytes = 0, lleOnlyLines = 0;
  for(u32 l = 0; l < lumiverseAudioLLEWriteCount; l++) {
    const auto& lw = lumiverseAudioLLEWrites[l];
    lleWrites++;
    u32 uncovered = 0;
    for(u32 a = lw.addr; a < lw.addr + lw.length; a++) {
      bool hit = false;
      for(u32 w = 0; w < shadow.writeCount && !hit; w++) {
        const auto& mine = shadow.writes[w];
        hit = a >= mine.addr && a < mine.addr + mine.length;
      }
      if(!hit) uncovered++;
    }
    if(uncovered) {
      lleOnlyWrites++; lleOnlyBytes += uncovered;
      //histogram by (length, dmem source) so the summary shows the classes
      static struct { u32 length, dmem; u64 count; } classes[64];
      static u32 classCount = 0;
      u32 c = 0;
      for(; c < classCount; c++) if(classes[c].length == lw.length && classes[c].dmem == lw.dmem) break;
      if(c == classCount && classCount < 64) { classes[classCount++] = {lw.length, lw.dmem, 0}; }
      if(c < classCount) classes[c].count++;
      if((lleOnlyWrites & 4095) == 0) {
        fprintf(stderr, "[rsp-hle-audio-shadow] LLE-only write classes (len/dmem=count):");
        for(u32 k = 0; k < classCount; k++) fprintf(stderr, " %u/%03x=%llu", classes[k].length, classes[k].dmem, (unsigned long long)classes[k].count);
        fprintf(stderr, "\n");
      }
      //distinct small LLE-only write addresses (debug>=3), once each, to
      //cross-reference against the alist's SETLOOP/state operands offline
      if(lumiverseAudioHLEDebug() >= 3 && lw.length <= 32) {
        static u32 seenAddr[4096]; static u32 seenCount = 0;
        bool seen = false;
        for(u32 i = 0; i < seenCount && !seen; i++) seen = seenAddr[i] == lw.addr;
        if(!seen && seenCount < 4096) {
          seenAddr[seenCount++] = lw.addr;
          fprintf(stderr, "[rsp-hle-audio-shadow]   LLE-only small write addr=%06x len=%u dmem=%03x task=%llu\n", lw.addr, lw.length, lw.dmem, (unsigned long long)shadow.task);
        }
      }
      //state-blob probe (debug>=3): print LLE's 32-byte state write next to
      //our private state for the same address, to learn the blob layout
      static u32 blobLines = 0;
      static const u64 blobMinTask = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_DUMP_MIN_TASK"); return v ? (u64)::atoll(v) : 0; }();
      if(lumiverseAudioHLEDebug() >= 3 && (lw.length == 32 || lw.length == 8 || lw.length == 16 || lw.length == 80) && blobLines < 400 && shadow.task >= blobMinTask) {
        blobLines++;
        auto& st = lumiverseAudioState(lw.addr);
        fprintf(stderr, "[rsp-hle-audio-shadow]   state blob addr=%06x len=%u dmem=%03x task=%llu LLE:", lw.addr, lw.length, lw.dmem, (unsigned long long)shadow.task);
        for(u32 b = 0; b < lw.length; b += 2) fprintf(stderr, " %04x", (u16)((u16)lumiverseAudioRDRAMReadByte(lw.addr + b) << 8 | lumiverseAudioRDRAMReadByte(lw.addr + b + 1)));
        fprintf(stderr, "\n[rsp-hle-audio-shadow]     ours (samples[0..15], frac=%04x):", st.frac & 0xffff);
        for(u32 b = 0; b < 16; b++) fprintf(stderr, " %04x", (u16)st.samples[b]);
        if(lw.length == 80) { fprintf(stderr, " | env: vl=%04x.%04x vr=%04x.%04x tgt=%04x/%04x rate=%04x%04x/%04x%04x dry=%04x wet=%04x", (u16)st.samples[0], (u16)st.samples[1], (u16)st.samples[2], (u16)st.samples[3], (u16)st.samples[4], (u16)st.samples[5], (u16)st.samples[6], (u16)st.samples[7], (u16)st.samples[8], (u16)st.samples[9], (u16)st.samples[10], (u16)st.samples[11]); }
        fprintf(stderr, "\n");
      }
      if(lleOnlyLines < 400 && lw.length > 32) {
        lleOnlyLines++;
        //print the first 8 samples LLE wrote there
        fprintf(stderr, "[rsp-hle-audio-shadow]   LLE-only write addr=%06x len=%u dmem=%03x uncovered=%u:", lw.addr, lw.length, lw.dmem, uncovered);
        for(u32 b = 0; b < 16 && b < lw.length; b += 2) fprintf(stderr, " %04x", (u16)((u16)lumiverseAudioRDRAMReadByte(lw.addr + b) << 8 | lumiverseAudioRDRAMReadByte(lw.addr + b + 1)));
        fprintf(stderr, "\n");
      }
    }
  }
  lumiverseAudioLLEWriteCount = 0;
  static u64 compared = 0;
  compared++;
  if((compared & 255) == 0) {
    fprintf(stderr, "[rsp-hle-audio-shadow] LLE write set: %llu writes, %llu with bytes the HLE never wrote (%llu bytes)%s\n",
      (unsigned long long)lleWrites, (unsigned long long)lleOnlyWrites, (unsigned long long)lleOnlyBytes,
      lumiverseAudioLLEWriteOverflow ? " (overflow)" : "");
    const auto& a = lumiverseAudioShadowAgg;
    fprintf(stderr, "[rsp-hle-audio-shadow] aggregate over %llu buffer writes: level=%.3f (L %.3f R %.3f) dratio=%.3f (L %.3f R %.3f) sat=%llu/%llu\n",
      (unsigned long long)a.writes,
      a.energyTheirs > 0 ? sqrt(a.energyMine / a.energyTheirs) : -1.0,
      a.chEnergyTheirs[0] > 0 ? sqrt(a.chEnergyMine[0] / a.chEnergyTheirs[0]) : -1.0,
      a.chEnergyTheirs[1] > 0 ? sqrt(a.chEnergyMine[1] / a.chEnergyTheirs[1]) : -1.0,
      a.dEnergyTheirs > 0 ? sqrt(a.dEnergyMine / a.dEnergyTheirs) : -1.0,
      a.chDEnergyTheirs[0] > 0 ? sqrt(a.chDEnergyMine[0] / a.chDEnergyTheirs[0]) : -1.0,
      a.chDEnergyTheirs[1] > 0 ? sqrt(a.chDEnergyMine[1] / a.chDEnergyTheirs[1]) : -1.0,
      (unsigned long long)a.satMine, (unsigned long long)a.satTheirs);
    fprintf(stderr, "[rsp-hle-audio-shadow] per length class (len:writes level dratio):");
    for(u32 c = 0; c < a.classCount; c++) {
      const auto& k = a.classes[c];
      fprintf(stderr, " %u:%llu %.3f %.3f", k.length, (unsigned long long)k.writes,
        k.energyTheirs > 0 ? sqrt(k.energyMine / k.energyTheirs) : -1.0,
        k.dEnergyTheirs > 0 ? sqrt(k.dEnergyMine / k.dEnergyTheirs) : -1.0);
    }
    fprintf(stderr, "\n");
  }
  if(bad || (compared & 255) == 1) {
    fprintf(stderr, "[rsp-hle-audio-shadow] task %llu: writes=%u exact=%u close=%u bad=%u covered=%u stateDiff=%u maxdiff=%d%s\n",
      (unsigned long long)shadow.task, shadow.writeCount, exact, close, bad, covered, stateDiff, taskMax,
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
  //LUMIVERSE_ARES_N64_AUDIO_HLE_DUMP_MIN_TASK=<n>: ignore tasks before n so
  //the quota is not spent on boot-time mismatches (run timing is not
  //reproducible across runs — the RDP's deferred SyncFull makes DP
  //interrupt timing wall-clock dependent — so a task cannot be targeted by
  //number from a previous run)
  static const u64 minTask = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_DUMP_MIN_TASK");
    return value ? (u64)::atoll(value) : 0;
  }();
  if(task < minTask) return;
  static u32 dumps = 0;
  if(dumps >= 6) return;
  dumps++;
  char path[1024];
  //round 13: honour TMPDIR so parallel oracle runs do not overwrite each other
  const char* tmpdir = ::getenv("TMPDIR");
  snprintf(path, sizeof(path), "%s/lumiverse-audio-debug-%llu.txt", tmpdir && *tmpdir ? tmpdir : "/tmp", (unsigned long long)task);
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
      if(rangeMax > 256 && lines < (lumiverseAudioHLEDebug() >= 3 ? 20000 : 600)) {
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
//task index a truncation experiment targeted (either selector); the settle
//path dumps exactly that task's DMEM images
s64 lumiverseAudioTruncateDumpTask = -1;

//settle any outstanding shadow comparison (called from every RSP task
//dispatch so a comparison still happens when the game stops issuing audio
//tasks, e.g. after a truncated-task experiment)
//round 16 diagnostic (DEBUG>=3, shadow mode): Conker's op 0x07 — the
//streamed-voice decoder (576 twelve-bit samples per call, written to the
//0x07 address as three 384-byte DMAs from DMEM 0xe70; state block at the
//preceding 0x08 address, 1088 bytes, written back from DMEM 0x8a0). The
//probe prints the state block as the task is dispatched (before LLE runs
//it) and the state block + output region after LLE ran, up to
//LUMIVERSE_ARES_N64_AUDIO_HLE_PROBE07 tasks, as raw material for decoding
//the format offline. Not an emulation path.
struct LumiverseAudioProbe07 { bool armed = false; u64 task = 0; u32 stateAddr = 0; u32 outAddr = 0; u32 param = 0; u32 count = 0; };
LumiverseAudioProbe07 lumiverseAudioProbe07;
auto lumiverseAudioProbe07Limit() -> u32 {
  static const u32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_PROBE07"); return v ? (u32)::atoi(v) : 0u; }();
  return value;
}
auto lumiverseAudioProbe07Dump(const char* tag, u32 addr, u32 bytes) -> void {
  fprintf(stderr, "[rsp-hle-audio-07] task %llu %s @%06x:", (unsigned long long)lumiverseAudioProbe07.task, tag, addr);
  for(u32 b = 0; b < bytes; b += 2) fprintf(stderr, " %04x", (u16)((u16)lumiverseAudioRDRAMReadByte(addr + b) << 8 | lumiverseAudioRDRAMReadByte(addr + b + 1)));
  fprintf(stderr, "\n");
}

auto lumiverseAudioShadowSettle() -> void {
  if(lumiverseAudioHLELevel() < 2) return;
  auto& shadow = lumiverseAudioShadowState;
  const bool hadPending = shadow.pending;
  if(hadPending && lumiverseAudioProbe07.armed && lumiverseAudioProbe07.task == shadow.task) {
    lumiverseAudioProbe07.armed = false;
    lumiverseAudioProbe07Dump("post-state", lumiverseAudioProbe07.stateAddr, 1088);
    lumiverseAudioProbe07Dump("post-out", lumiverseAudioProbe07.outAddr, 1152);
  }
  lumiverseAudioShadowCompare(shadow);
  if(hadPending && lumiverseAudioHLEDebug() >= 2) {
    const s32 worst = lumiverseAudioShadowCompareDMEM(lumiverseAudioMachine, shadow.task);
    static s64 truncateTask = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_TASK");
      return value ? ::atoll(value) : -1;
    }();
    //with a truncate target set, dump only that task (don't burn the dump
    //quota on earlier tasks)
    //with a truncation selector configured, dump ONLY the targeted task (the
    //three-dump quota would otherwise be spent on early boot mismatches)
    static const bool selectorConfigured = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_OP") != nullptr || ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_MATCH") != nullptr || truncateTask >= 0;
    //without a selector, dump the tasks whose OUTPUT buffers correlate
    //poorly with LLE's (an audible divergence), not merely DMEM scratch
    //differences (the resampler approximation alone exceeds 1000 LSB)
    const bool dumpWanted = lumiverseAudioTruncateDumpTask >= 0
      ? shadow.task == (u64)lumiverseAudioTruncateDumpTask
      : (selectorConfigured ? false : lumiverseAudioShadowWorstCorr < 0.6);
    (void)worst;
    if(dumpWanted && lumiverseAudioCapture) lumiverseAudioDebugDump(*lumiverseAudioCapture, shadow.task);
  }
}

//round 14: LUMIVERSE_ARES_N64_RSP_HLE_AUDIO_CYCLES_PER_CMD (default 0 =
//instant completion): a natively executed audio task reports completion
//after commands * this many RSP cycles, and its RDRAM output lands
//progressively over that time (lumiverseAudioProgressDeferredWrites). The
//Rare engines read the ENVMIX blocks back while the microcode works; with
//instant completion (and with all output landing at once) their game path
//diverged from LLE's. Measured LLE cost: Banjo-Tooie ~690 cycles/command
//(7.4 ms tasks), DK64 similar (the shadow prints "LLE task duration").
//round 14: LUMIVERSE_ARES_N64_AUDIO_HLE_NA_STATE_WRITES (default 1): the
//naudio voice path also writes the two per-voice state blobs the microcode
//keeps in RDRAM (the 32-byte ADPCM decoder state = the last 16 decoded
//samples, bit-exact; the 16-byte RESAMPLE state = [4 history][fraction]
//[3 words]). The 3 trailing words are what the microcode's state-staging
//area (DMEM 0xfa0) held from the previous ENVMIX block staged there — its L
//int lanes 5..7 (block dumps: "0b51 0b49 0b41" next to a block whose lanes
//step by 8). Real HLE never wrote either blob before this round.
auto lumiverseAudioNaStateWrites() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_STATE_WRITES"); return !v || v[0] != '0'; }();
  return value;
}
u16 lumiverseAudioNaLastBlockLanes[3] = {};
//round 16: LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_COST (default 1): the ABI1/ABI2
//executors report task completion after commands x the title's measured
//LLE cost instead of instantly. With the live COP0 Count (CPU_FAST_COUNT)
//the game can see how long an audio task takes; instant completion moved
//the H-vs-M0 gates of every ABI title that times its audio thread (SF64
//0.998 -> 0.971, SM64 0.996 -> 0.984, Pilotwings 0.946 -> 0.889) and the
//measured mean restores them (0.998 / 0.997 with all frames identical /
//0.951). Means = our own LLE's per-task RSP cycles over the shadow runs of
//this round (the "LLE task duration" line), keyed on the stable ucode hash
//or the self-modifying family's prefix; unknown ABI images get 600.
auto lumiverseAudioAbiCostModel() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_ABI_COST"); return !v || v[0] != '0'; }();
  return value;
}
auto lumiverseAudioAbiCyclesPerCommand(u64 ucodeHash, u64 prefixHash) -> s32 {
  struct Entry { u64 hash; s32 cycles; };
  static const Entry byHash[] = {
    { LumiverseAudioUcodeStarFox64, 879 }, { LumiverseAudioUcodeSM64WR, 670 },   //SM64 684, Wave Race 652
    { LumiverseAudioUcodeZeldaOoTU, 931 }, { LumiverseAudioUcodeZeldaMQ, 860 },
    { LumiverseAudioUcodeMajoraU, 930 },   //Majora 947, Pokemon Stadium 2 (same image) 908
    { LumiverseAudioUcodeSnapU, 390 }, { LumiverseAudioUcodeDoom64, 472 }, { LumiverseAudioUcodeCruisnUSA, 512 },
    { LumiverseAudioUcodeRush, 436 }, { LumiverseAudioUcodeYoshi, 937 },
  };
  static const Entry byPrefix[] = {
    { LumiverseAudioPrefixMK64, 843 }, { LumiverseAudioPrefixGE, 514 },
    { LumiverseAudioPrefixPWSOTE, 370 },   //Pilotwings 365, Shadows of the Empire 385, Cruis'n World 369
  };
  for(auto& e : byHash) if(e.hash == ucodeHash) return e.cycles;
  for(auto& e : byPrefix) if(e.hash == prefixHash) return e.cycles;
  return 600;
}

auto lumiverseAudioCyclesPerCommand() -> s32 {
  static const s32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_AUDIO_CYCLES_PER_CMD"); return v ? ::atoi(v) : 0; }();
  return value;
}
extern s32 lumiverseHLERequestedCompletionCycles;
u64 lumiverseAudioCommandSum = 0;
auto lumiverseExecuteAudioTask(const u32 task[16], u64 ucodeHash) -> bool {
  const int level = lumiverseAudioHLELevel();
  if(level < 1) return false;
  s32 dialect = lumiverseAudioDialectForHash(ucodeHash);
  bool polefFir = false;
  static const bool rushOptOut = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_RUSH"); return v && v[0] == '0'; }();
  if(dialect < 0 && ucodeHash == LumiverseAudioUcodeRush && rushOptOut) return false;
  u64 prefixHash = 0;
  if(dialect < 0) {
    //self-modifying ucode? identify the family by its stable prefix
    const u32 ucode = task[4] & 0x00ffffff;
    const u32 ucodeSize = task[5];
    if(ucode && (ucodeSize == 0 || ucodeSize >= LumiverseAudioUcodePrefixLength)) {
      prefixHash = lumiverseHashRDRAM(ucode, LumiverseAudioUcodePrefixLength);
      dialect = lumiverseAudioDialectForPrefixHash(prefixHash);
      polefFir = prefixHash == LumiverseAudioPrefixPM;
    }
  }
  if(dialect < 0) return false;
  //round 13: LUMIVERSE_ARES_N64_AUDIO_HLE_PM_MODEL=1 runs Paper Mario's
  //reverb on the Rare-engine model (state-blob tap + one-pole 0e) instead
  //of the round-12 fits (validation only)
  static const bool pmRareModel = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_PM_MODEL"); return !v || v[0] != '0'; }();
  lumiverseAudioMachine.naPolefFir = polefFir && !pmRareModel;
  static const bool onePoleKnob = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_NA_POLEF"); return !v || v[0] != '0'; }();
  //Banjo-Kazooie's older ucode issues the same 0e command but leaves the
  //buffer untouched (before/after oracles on tasks 200 and 450: only 8
  //microcode-internal DMEM bytes change), so the one-pole is off there
  lumiverseAudioMachine.naPolefOnePole = onePoleKnob && (ucodeHash == LumiverseAudioUcodeBanjoT
    || ucodeHash == LumiverseAudioUcodeConker || ucodeHash == LumiverseAudioUcodePD || ucodeHash == LumiverseAudioUcodeDK64
    || (polefFir && pmRareModel));

  const u32 dataPtr = task[12] & 0x00ffffff;
  u32 dataSize = task[13];
  if(!dataPtr || dataSize < 8 || dataSize > 0x10000) return false;
  //round 14: any RDRAM writes still held from the previous task land now
  //(normally flushed at that task's completion; this covers a task that
  //never reported completion)
  lumiverseAudioFlushDeferredWrites();
  lumiverseAudioTaskStartCycles = rsp.profile.cycles ? rsp.profile.cycles : 1;

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
    //alternative selector: LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_OP=<hex opcode>
    //truncates the FIRST task whose alist contains that opcode, right after
    //its first occurrence (or right before it with ..._TRUNCATE_BEFORE=1),
    //so an experiment does not depend on task numbering.
    //two hex digits match the opcode; four match the opcode AND flags byte
    //(e.g. 0700 = FILTER apply, not the 0702 coefficient-table setup)
    static s64 truncateOp = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_OP");
      return value ? ::strtoll(value, nullptr, 16) : -1;
    }();
    static const bool truncateOpWithFlags = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_OP");
      return value && ::strlen(value) >= 4;
    }();
    static bool truncateBefore = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_BEFORE");
      return value && *value == '1';
    }();
    static bool truncateOpDone = false;
    //LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_MIN_TASK=<n>: only consider tasks
    //from index n on (skip boot-time tasks whose buffers are silent)
    static const u64 truncateMinTask = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_MIN_TASK");
      return value ? (u64)::atoll(value) : 0;
    }();
    //LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_MATCH=<w0 hex>:<w1 hex>[:<skip>]:
    //truncate the first task (after skipping `skip` matching tasks) whose
    //alist contains exactly that command pair, right after it (or before it
    //with TRUNCATE_BEFORE=1). Unlike TRUNCATE_OP this counts EVERY task,
    //validated or not, so a command form the validator rejects can still be
    //put under the oracle (Pokemon Stadium 2's in-place DECIMATE, round 10).
    static const bool matchConfigured = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_MATCH") != nullptr;
    static u32 matchW0 = 0, matchW1 = 0;
    static u64 matchSkip = 0;
    static bool matchParsed = false;
    static u64 seenTasks = 0;
    if(matchConfigured && !matchParsed) {
      matchParsed = true;
      const char* value = ::getenv("LUMIVERSE_ARES_N64_AUDIO_HLE_TRUNCATE_MATCH");
      char* end = nullptr;
      matchW0 = (u32)::strtoul(value, &end, 16);
      if(end && *end == ':') matchW1 = (u32)::strtoul(end + 1, &end, 16);
      if(end && *end == ':') matchSkip = (u64)::strtoull(end + 1, nullptr, 10);
    }
    seenTasks++;
    if(matchConfigured && !truncateOpDone && truncateTask < 0) {
      for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
        u32 w0 = 0, w1 = 0;
        for(u32 b = 0; b < 4; b++) w0 = w0 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + b);
        for(u32 b = 0; b < 4; b++) w1 = w1 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + 4 + b);
        if(w0 == matchW0 && w1 == matchW1) {
          if(matchSkip > 0) { matchSkip--; break; }
          const s64 index = (s64)(offset / 8) - (truncateBefore ? 1 : 0);
          if(index >= 0) {
            truncateOpDone = true;
            truncateTask = (s64)taskIndex;
            truncateCommand = index;
            fprintf(stderr, "[rsp-hle-audio] truncate-match %08x %08x: seen-task %llu (executed index %llu) command %lld\n",
              matchW0, matchW1, (unsigned long long)seenTasks - 1, (unsigned long long)taskIndex, (long long)index);
          }
          break;
        }
      }
    }
    if(truncateOp >= 0 && !truncateOpDone && truncateTask < 0 && taskIndex >= truncateMinTask) {
      for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
        const u32 head = truncateOpWithFlags
          ? ((u32)lumiverseAudioRDRAMReadByte(dataPtr + offset) << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + 1))
          : lumiverseAudioRDRAMReadByte(dataPtr + offset);
        if(head == (u32)truncateOp) {
          const s64 index = (s64)(offset / 8) - (truncateBefore ? 1 : 0);
          if(index >= 0) {
            truncateOpDone = true;
            truncateTask = (s64)taskIndex;
            truncateCommand = index;
            fprintf(stderr, "[rsp-hle-audio] truncate-op %02llx: task %llu command %lld\n",
              (long long)truncateOp, (unsigned long long)taskIndex, (long long)index);
          }
          break;
        }
      }
    }
    //fire once: taskIndex only advances on validated tasks, so a rejected
    //target would otherwise be re-truncated on every following task
    static bool truncateApplied = false;
    if(truncateTask >= 0 && (u64)truncateTask == taskIndex && truncateCommand >= 0 && !truncateApplied) {
      truncateApplied = true;
      //shrink the OSTask's dataSize (DMEM 0xfc0 + 13*4) so the microcode
      //stops after the chosen command; also truncate our own walk
      const u32 truncatedSize = (u32)(truncateCommand + 1) * 8;
      if(truncatedSize < dataSize) {
        dataSize = truncatedSize;
        rsp.dmem.write<Word>(0xfc0 + 13 * 4, truncatedSize);
      }
      lumiverseAudioTruncateArmed = true;
      lumiverseAudioTruncateDumpTask = (s64)taskIndex;
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
  //round 16: op 0x07 probe (see lumiverseAudioProbe07)
  if(level >= 2 && lumiverseAudioHLEDebug() >= 3 && lumiverseAudioProbe07.count < lumiverseAudioProbe07Limit()) {
    u32 stateAddr = 0;
    for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
      u32 w0 = 0, w1 = 0;
      for(u32 b = 0; b < 4; b++) w0 = w0 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + b);
      for(u32 b = 0; b < 4; b++) w1 = w1 << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + 4 + b);
      if((w0 >> 24) == 0x08) stateAddr = w1 & 0x00ffffff;
      if((w0 >> 24) == 0x07 && stateAddr && !lumiverseAudioProbe07.armed) {
        auto& pr = lumiverseAudioProbe07;
        pr.armed = true; pr.task = taskIndex; pr.stateAddr = stateAddr; pr.outAddr = w1 & 0x00ffffff; pr.param = w0 & 0xffffff; pr.count++;
        fprintf(stderr, "[rsp-hle-audio-07] task %llu cmd %u: 07 param=%06x out=%06x state=%06x\n", (unsigned long long)taskIndex, offset / 8, pr.param, pr.outAddr, pr.stateAddr);
        lumiverseAudioProbe07Dump("pre-state", stateAddr, 1088);
        lumiverseAudioProbe07Dump("pre-out", pr.outAddr, 1152);
      }
    }
  }

  lumiverseAudioValidateTaskIndex = taskIndex;
  if(!lumiverseAudioValidate(machine, dataPtr, dataSize, (u32)dialect)) {
    machine.tasksFallback++;
    //oracle on a task the validator rejects: nothing of ours to compare, but
    //LLE's DMEM after the truncated prefix is exactly the evidence wanted —
    //stash the alist and arm a write-less settle so the dump still happens
    if(level >= 2 && lumiverseAudioTruncateArmed && lumiverseAudioTruncateDumpTask == (s64)taskIndex) {
      if(capture) {
        capture->alistWords = 0;
        for(u32 offset = 0; offset + 4 <= dataSize && capture->alistWords < 0x2000; offset += 4) {
          u32 word = 0;
          for(u32 b = 0; b < 4; b++) word = word << 8 | lumiverseAudioRDRAMReadByte(dataPtr + offset + b);
          capture->alist[capture->alistWords++] = word;
        }
      }
      shadow.active = false;
      shadow.writeCount = 0;
      shadow.byteCount = 0;
      shadow.overflow = false;
      shadow.task = taskIndex;
      shadow.pending = true;
    }
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
    if(lumiverseAudioTaskCycleCount) fprintf(stderr, "[rsp-hle-audio] LLE task duration: mean %.0f cycles (%.0f us) max %llu over %llu tasks; mean %.0f commands/task (%.0f cycles/command)\n",
      (double)lumiverseAudioTaskCycleSum / lumiverseAudioTaskCycleCount, (double)lumiverseAudioTaskCycleSum / lumiverseAudioTaskCycleCount / 62.5, (unsigned long long)lumiverseAudioTaskCycleMax, (unsigned long long)lumiverseAudioTaskCycleCount,
      (double)lumiverseAudioCommandSum / machine.tasksExecuted, (double)lumiverseAudioTaskCycleSum / (lumiverseAudioCommandSum ? lumiverseAudioCommandSum : 1));
    fprintf(stderr, "[rsp-hle-audio] executed=%llu fallback=%llu envmix-unknown-flags=%llu\n",
      (unsigned long long)machine.tasksExecuted, (unsigned long long)machine.tasksFallback,
      (unsigned long long)machine.unknownEnvmixFlags);
  }

  //round 14: modelled task duration (see lumiverseAudioCyclesPerCommand)
  //and the command count the progressive landing of the deferred output
  //writes is scaled by
  {
    const u32 commands = dataSize / 8;
    lumiverseAudioCommandSum += commands;
    for(auto& c : lumiverseAudioTaskOps) c = 0;
    lumiverseAudioCmdCostCount = 0;
    const bool rareEngine = ucodeHash == LumiverseAudioUcodeBanjoK || ucodeHash == LumiverseAudioUcodeBanjoT
      || ucodeHash == LumiverseAudioUcodeDK64 || ucodeHash == LumiverseAudioUcodeConker;
    const bool costModel = lumiverseAudioCostModel() && rareEngine && lumiverseAudioCyclesPerCommand() <= 0;
    u32 cost = lumiverseAudioRareCostConstant;
    for(u32 offset = 0; offset + 8 <= dataSize; offset += 8) {
      const u32 op = lumiverseAudioRDRAMReadByte(dataPtr + offset) & 0x1f;
      lumiverseAudioTaskOps[op]++;
      if(costModel && lumiverseAudioCmdCostCount < 8192) { cost += lumiverseAudioRareCost[op]; lumiverseAudioCmdCostPrefix[lumiverseAudioCmdCostCount++] = cost; }
    }
    lumiverseAudioSetDeferredCommands(commands);
    //round 15 diagnostic (DEBUG>=2): the task's dispatch time on the RSP
    //cycle counter, its command count and (real HLE) the modelled duration,
    //so the CPU read log (rspc=) can be placed on the task timeline
    if(lumiverseAudioHLEDebug() >= 2) fprintf(stderr, "[rsp-hle-audio-task] dispatch rspc=%llu modelled=%u cmds=%u shadow=%u\n",
      (unsigned long long)rsp.profile.cycles, shadow.active ? 0u : (costModel ? cost : 0u), commands, (u32)shadow.active);
    //round 16: ABI1/ABI2 titles complete after their measured mean cost (naudio titles stay instant)
    const s32 abiCost = (!rareEngine && dialect != (s32)LumiverseAudioDialectNaudio && lumiverseAudioAbiCostModel()) ? lumiverseAudioAbiCyclesPerCommand(ucodeHash, prefixHash) : 0;
    if(!shadow.active) {
      if(lumiverseAudioCyclesPerCommand() > 0) lumiverseHLERequestedCompletionCycles = (s32)(commands * (u32)lumiverseAudioCyclesPerCommand());
      else if(costModel) lumiverseHLERequestedCompletionCycles = (s32)cost;
      else if(abiCost > 0) lumiverseHLERequestedCompletionCycles = (s32)(commands * (u32)abiCost);
    }
  }
  if(shadow.active) {
    shadow.active = false;
    shadow.pending = true;
    return false;  //LLE still executes the task; we only compare
  }
  return true;
}

}  //namespace
