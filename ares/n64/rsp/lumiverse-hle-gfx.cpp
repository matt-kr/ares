//Lumiverse addition: native (HLE) execution of RSP graphics tasks.
//
//Translates F3DEX-family display lists straight into low-level RDP commands
//and feeds them to paraLLEl-RDP via vulkan.queueHLECommands(), bypassing the
//LLE RSP entirely for recognized microcodes. Unrecognized microcodes return
//false and run on the LLE core unchanged.
//
//Clean-room provenance (see References/rsp-hle/PROVENANCE.md):
//  - n64js (MIT) — GBI1/F3DEX command semantics, matrix/vertex/light formats,
//    microcode detection, viewport + clip-code conventions
//  - libdragon rdpq_tri.c (Unlicense) — RDP triangle coefficient computation
//    (edge walker slopes, shade/texture/Z gradients); ported faithfully with
//    one fix: rdpq_tri.c line 346 uses `&&` where `&` is intended (DtDy
//    fractional bits); we emit the correct fraction.
//  - n64brew / CloudModding docs — RDP command encodings (read as spec only)
//This file is included from rsp.cpp inside namespace ares::Nintendo64 (same
//translation unit as lumiverse-hle.cpp); no #includes allowed here.

namespace {

//SF64 (U) + Yoshi's Story: "RSP Gfx ucode F3DEX.NoN     1.22 Yoshitaka Yasumoto Nintendo."
constexpr u64 LumiverseUcodeF3DEXNoN122 = 0xe19a52e44209a2e6ull;
//Banjo-Kazooie (U): "RSP Gfx ucode F3DEX 1.21"
constexpr u64 LumiverseUcodeF3DEX121 = 0x868fc897807a6dcbull;
//Doom 64 (U): "RSP Gfx ucode F3DEX.NoN 1.00"
constexpr u64 LumiverseUcodeF3DEXNoN100 = 0xa497382b6f43c20eull;
//Mario Kart 64 (U): "RSP Gfx ucode F3DEX 0.95"
constexpr u64 LumiverseUcodeF3DEX095 = 0xc034eac9da9c9eceull;
//Zelda OoT Master Quest (E): "RSP Gfx ucode F3DZEX.NoN fifo 2.08J Yoshitaka Yasumoto/Kawasedo 1999."
constexpr u64 LumiverseUcodeF3DZEXNoN208J = 0x25b57dd6a30111d1ull;
//Zelda OoT (U): F3DZEX.NoN fifo 2.06H
constexpr u64 LumiverseUcodeF3DZEXNoN206H = 0x53b27814790b1a3bull;
//Majora's Mask (U): F3DZEX.NoN fifo 2.08I
constexpr u64 LumiverseUcodeF3DZEXNoN208I = 0xf7ec201e8049d5cfull;
//Banjo-Tooie (U) + Tony Hawk's Pro Skater 2 (U): "RSP Gfx ucode F3DEX.NoN fifo 2.08  Yoshitaka Yasumoto 1999 Nintendo."
constexpr u64 LumiverseUcodeF3DEX2NoN208 = 0x64df8bf96faf6149ull;
//2026-08-29 census additions — all "fifo 2.xx" = F3DEX2/GBI2 command set.
//Super Smash Bros. (U) + Kirby 64 (U): "RSP Gfx ucode F3DEX fifo 2.04H"
constexpr u64 LumiverseUcodeF3DEX2204H = 0xef47e4ae07558fa7ull;
//Pokemon Stadium (U): "RSP Gfx ucode F3DEX fifo 2.06"
constexpr u64 LumiverseUcodeF3DEX2206 = 0x9a69128077e0353bull;
//Donkey Kong 64 (U): "RSP Gfx ucode F3DEX fifo 2.07" — hash kept for the
//census, but NOT whitelisted: under HLE the game hangs (CPU stuck at the
//exception vector, gfx dispatch stops) right as the DK Rap starts, while
//rendering up to that point is correct and the opcode census is ordinary.
//Rare-engine pattern (BK type-2 gfx, BT slower with HLE): their engines
//appear to depend on RSP task timing/DPC behavior HLE doesn't reproduce.
constexpr u64 LumiverseUcodeF3DEX2207 = 0xfcf475ff1d56eb94ull;
//Pokemon Snap (U): "RSP Gfx ucode F3DEX.NoN fifo 2.08H"
constexpr u64 LumiverseUcodeF3DEX2NoN208H = 0xe444097f7a0d4d6eull;
//Pokemon Stadium 2 (U) + Paper Mario (U): "RSP Gfx ucode F3DEX fifo 2.08 Yoshitaka Yasumoto/Kawasedo 1999."
constexpr u64 LumiverseUcodeF3DEX2208K = 0x622da83a65470bbbull;
//Ogre Battle 64 (U): "RSP Gfx ucode F3DEX fifo 2.08 Yoshitaka Yasumoto 1999 Nintendo."
constexpr u64 LumiverseUcodeF3DEX2208 = 0x2f87f5f429f21a49ull;

//Fast3D / GBI0 (launch generation, banner "RSP SW Version: 2.0D, 04-01-96";
//n64js gbi0.js + microcodes.js). Standard vertex encoding:
constexpr u64 LumiverseUcodeFast3DSM64 = 0xf22d8c44b6bcbad8ull;  //Super Mario 64 (U)
constexpr u64 LumiverseUcodeFast3DPW64 = 0x062fc0aa4c90aa8cull;  //Pilotwings 64 (U)
constexpr u64 LumiverseUcodeFast3DCUSA = 0x9079734c8d1f0adeull;  //Cruis'n USA (U)
//Wave Race 64 (U): GBI0WR vertex variant (index stride 5, n/v0 repacked)
constexpr u64 LumiverseUcodeFast3DWR64 = 0xee91e7386f55e7f0ull;
//Shadows of the Empire (U): GBI0SE vertex variant (stride 5, n=len/33, v0=0)
constexpr u64 LumiverseUcodeFast3DSOTE = 0x362af95700e02370ull;
//GoldenEye (U) "RSP SW Version: 2.0G" hash c8f38644ac25bbab: modified Fast3D
//that bakes raw RDP triangle streams into the DL (0xb4/0xb2/0xb3 pairs;
//n64js GBI0GE only approximates them as screen rects). NOT whitelisted —
//per-task LLE fallback keeps it correct.

#if defined(VULKAN)

//----------------------------------------------------------------------------
//RDRAM access (side-effect-free; addresses are physical, masked to 24 bits)
//----------------------------------------------------------------------------

auto lumiverseGfxWord(u32 address) -> u32 {
  return rdram.ram.Memory::Writable::read<Word>(address & 0x00ffffff);
}

auto lumiverseGfxHalf(u32 address) -> u16 {
  return rdram.ram.Memory::Writable::read<Half>(address & 0x00ffffff);
}

auto lumiverseGfxShort(u32 address) -> s16 {
  return (s16)lumiverseGfxHalf(address);
}

auto lumiverseGfxByte(u32 address) -> u8 {
  return rdram.ram.Memory::Writable::read<Byte>(address & 0x00ffffff);
}

auto lumiverseGfxSByte(u32 address) -> s8 {
  return (s8)lumiverseGfxByte(address);
}

//----------------------------------------------------------------------------
//GBI1 (F3DEX v1) constants — encodings from n64js gbi.js/gbi1.js (MIT)
//----------------------------------------------------------------------------

//geometry mode bits shared by GBI1 and GBI2 (n64js gbi.js): zbuffer, fog,
//lighting and texgen use the same bit positions in both dialects; shading
//smooth and cull bits differ and are parameterized per dialect below.
constexpr u32 LumiverseGeomZBuffer         = 0x00000001;
constexpr u32 LumiverseGeomTextureEnable   = 0x00000002;  //ucode-internal
constexpr u32 LumiverseGeomShade           = 0x00000004;
constexpr u32 LumiverseGeomFog             = 0x00010000;
constexpr u32 LumiverseGeomLighting       = 0x00020000;
constexpr u32 LumiverseGeomTextureGen      = 0x00040000;
constexpr u32 LumiverseGeomTextureGenLinear= 0x00080000;
//GBI1 (GeometryModeGBI1)
constexpr u32 LumiverseGeom1ShadingSmooth  = 0x00000200;
constexpr u32 LumiverseGeom1CullFront      = 0x00001000;
constexpr u32 LumiverseGeom1CullBack       = 0x00002000;
//GBI2 (GeometryModeGBI2). NOTE: n64js gbi.js lists CULL_BACK=0x200 /
//CULL_FRONT=0x400, but with the screen-space winding convention validated
//pixel-exact against LLE on GBI1 content, OoT terrain (which sets 0x400)
//only renders correctly with the opposite assignment: 0x200=front, 0x400=back.
constexpr u32 LumiverseGeom2CullFront      = 0x00000200;
constexpr u32 LumiverseGeom2CullBack      = 0x00000400;
constexpr u32 LumiverseGeom2ShadingSmooth  = 0x00200000;

enum : u32 { LumiverseDialectGBI1 = 0, LumiverseDialectGBI2 = 1, LumiverseDialectGBI0 = 2 };

//GBI0 G_VTX encoding variants (n64js gbi0.js subclasses)
enum : u32 { LumiverseGBI0VertexStandard = 0, LumiverseGBI0VertexWaveRace = 1, LumiverseGBI0VertexSOTE = 2 };

//clip codes (internal convention; only used self-consistently)
constexpr u8 LumiverseClipNX   = 0x01;  //x < -w
constexpr u8 LumiverseClipPX   = 0x02;  //x > +w
constexpr u8 LumiverseClipNY   = 0x04;  //y < -w
constexpr u8 LumiverseClipPY   = 0x08;  //y > +w
constexpr u8 LumiverseClipFar  = 0x10;  //z > +w
constexpr u8 LumiverseClipNear = 0x20;  //z < -w (NoN: never rejected)
constexpr u8 LumiverseClipCullMask = LumiverseClipNX | LumiverseClipPX
  | LumiverseClipNY | LumiverseClipPY | LumiverseClipFar;

constexpr f32 LumiverseWEpsilon = 1e-3f;

//----------------------------------------------------------------------------
//output stream: complete RDP commands, host byte order, flushed in chunks
//----------------------------------------------------------------------------

struct LumiverseGfxOut {
  static constexpr u32 Capacity = 0xd000;  //words; < 0x8000 pairs per flush
  static constexpr u32 FlushAt  = 0xc000;
  u32 words[Capacity];
  u32 count = 0;
  bool failed = false;
  //fifo mode: instead of queueHLECommands, write the stream into the game's
  //fifo buffer in RDRAM and kick the DPC registers exactly like the fifo
  //microcode does. Some games (Banjo-Tooie) poll DPC progress rather than
  //waiting for a SyncFull interrupt, so the authentic path is required.
  bool fifo = false;
  u32 fifoStart = 0;
  u32 fifoEnd = 0;
};

auto lumiverseGfxFlush(LumiverseGfxOut& out) -> void {
  if(out.count == 0) return;

  if(out.fifo) {
    const u32 capacityWords = (out.fifoEnd - out.fifoStart) / 4;
    if(capacityWords >= 2) {
      u32 offset = 0;
      while(offset < out.count) {
        u32 chunk = out.count - offset;
        if(chunk > capacityWords) chunk = capacityWords & ~1u;
        for(u32 index = 0; index < chunk; index++) {
          rdram.ram.Memory::Writable::write<Word>(out.fifoStart + index * 4, out.words[offset + index]);
        }
        //DPC_START then DPC_END; paraLLEl consumes the range synchronously
        rdp.writeWord(0x00, out.fifoStart, rsp);
        rdp.writeWord(0x04, out.fifoStart + chunk * 4, rsp);
        offset += chunk;
      }
      out.count = 0;
      return;
    }
    //degenerate fifo: fall through to the direct queue
  }

  if(!vulkan.queueHLECommands(out.words, out.count)) {
    if(!out.failed) fprintf(stderr, "[rsp-hle-gfx] queueHLECommands failed\n");
    out.failed = true;
  }
  out.count = 0;
}

auto lumiverseGfxEmit(LumiverseGfxOut& out, u32 word) -> void {
  if(out.count >= LumiverseGfxOut::Capacity) { out.failed = true; return; }
  out.words[out.count++] = word;
}

auto lumiverseGfxEmit2(LumiverseGfxOut& out, u32 hi, u32 lo) -> void {
  lumiverseGfxEmit(out, hi);
  lumiverseGfxEmit(out, lo);
  if(out.count >= LumiverseGfxOut::FlushAt) lumiverseGfxFlush(out);
}

//----------------------------------------------------------------------------
//state machine
//----------------------------------------------------------------------------

struct LumiverseGfxVertex {
  f32 x, y, z, w;    //clip space
  f32 u, v;          //texels (texture scale applied)
  f32 r, g, b, a;    //0..255
  u8 clip = 0;
};

struct LumiverseGfxLight {
  f32 r = 0, g = 0, b = 0;   //0..255
  f32 x = 1, y = 0, z = 0;   //normalized direction
};

struct LumiverseGfxTileSize {
  u32 uls = 0, ult = 0, lrs = 0, lrt = 0;  //10.2 fixed
};

struct LumiverseGfxMachine {
  //display list
  u32 pc = 0;
  u32 stack[32];
  u32 stackDepth = 0;
  u32 cmd0 = 0, cmd1 = 0;
  bool running = false;

  //RSP state
  u32 segments[16];
  f32 projStack[8][16];
  u32 projDepth = 0;
  f32 mvStack[32][16];
  u32 mvDepth = 0;
  f32 combined[16];
  bool combinedDirty = true;
  LumiverseGfxVertex verts[32];
  u32 geometryMode = 0;
  u32 otherModeL = 0, otherModeH = 0;
  bool otherModeDirty = true;
  f32 vpScaleX = 160, vpScaleY = 120, vpScaleZ = 511;
  f32 vpTransX = 160, vpTransY = 120, vpTransZ = 511;
  LumiverseGfxLight lights[8];
  u32 numLights = 0;
  f32 fogMul = 0, fogOff = 0;
  f32 texScaleS = 1.0f, texScaleT = 1.0f;
  u32 texTile = 0, texLevel = 0;
  u32 rdpHalf1 = 0;
  LumiverseGfxTileSize tileSizes[8];
  //GoldenEye 2.0G "baked RDP stream": the game pre-computes RDP triangle
  //commands on the CPU and embeds them in the display list as a run of
  //0xb4/0xb2 pairs (one 32-bit RDP word per GBI command, in cmd1) closed by
  //0xb3 carrying the final word. Format established EMPIRICALLY from our own
  //DL dumps (LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP) cross-checked against
  //the RDP stream the LLE microcode kicked for the same task
  //(LUMIVERSE_ARES_N64_RDP_STREAM_DUMP): the cmd1 words concatenate to
  //exactly the RDP command LLE emits (e.g. 40 words = one 0xce
  //shade+texture+z triangle), which the microcode kicks qword by qword.
  //n64js (MIT) documents the same word-stream shape for its GBI0GE class.
  bool bakedRDP = false;
  u32 baked[64];
  u32 bakedCount = 0;

  //dialect (GBI1 = F3DEX v1, GBI2 = F3DEX2/F3DZEX, GBI0 = Fast3D) and its
  //bit positions
  u32 dialect = LumiverseDialectGBI1;
  u32 smoothMask = LumiverseGeom1ShadingSmooth;
  u32 cullFrontMask = LumiverseGeom1CullFront;
  u32 cullBackMask = LumiverseGeom1CullBack;
  u32 shadeMask = LumiverseGeomShade;  //0 = shade always on (GBI2)
  //vertex-index divisor in TRI/quad byte-index fields (n64js vertexStride):
  //GBI1 = 2, GBI0 standard/GE = 10, GBI0 WaveRace/SOTE = 5
  u32 triStride = 2;
  u32 gbi0Vertex = LumiverseGBI0VertexStandard;  //G_VTX encoding variant

  //output
  LumiverseGfxOut out;

  //diagnostics
  u64 trisEmitted = 0;
  u64 trisClipped = 0;
  u64 trisRejected = 0;
};

auto lumiverseGfxLogLevel() -> int {
  static int level = [] {
    const char* value = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_LOG");
    return value ? ::atoi(value) : 0;
  }();
  return level;
}

auto lumiverseGfxSegmentAddress(LumiverseGfxMachine& m, u32 address) -> u32 {
  const u32 segment = (address >> 24) & 0xf;
  return (m.segments[segment] + (address & 0x00ffffff)) & 0x00ffffff;
}

auto lumiverseGfxReset(LumiverseGfxMachine& m, u32 pc, u32 dialect, u32 gbi0Vertex = LumiverseGBI0VertexStandard, bool bakedRDP = false) -> void {
  //Per-task, not per-process. `machine` is a function-local static that lives
  //for the whole process, and nothing else ever clears these two: once a
  //single task overflowed the buffer or lost a queueHLECommands, `failed`
  //stayed set forever. Both interpreters end with `return !m.out.failed`, so
  //every later task — including every task of the NEXT ROM loaded — reported
  //"HLE declined" while still having emitted and flushed its RDP stream. The
  //caller then re-ran the same display list on the LLE RSP, so the RDP got
  //the frame twice. That is the "Star Fox is fine cold but broken after
  //Zelda" signature. Clearing here restores the intended meaning: did THIS
  //task fail. NOTE: deliberately does not touch out.fifo/fifoStart/fifoEnd —
  //the caller sets those immediately before invoking the interpreter.
  m.out.count = 0;
  m.out.failed = false;
  m.pc = pc;
  m.stackDepth = 0;
  m.running = true;
  m.dialect = dialect;
  m.gbi0Vertex = gbi0Vertex;
  if(dialect == LumiverseDialectGBI2) {
    m.smoothMask = LumiverseGeom2ShadingSmooth;
    m.cullFrontMask = LumiverseGeom2CullFront;
    m.cullBackMask = LumiverseGeom2CullBack;
    m.shadeMask = 0;  //GBI2: shade coefficients always generated (n64js)
    m.triStride = 2;
  } else {
    //GBI0 shares GBI1's geometry-mode bit positions (n64js: GBI0 extends
    //GBI1 without overriding them); only the index stride differs
    m.smoothMask = LumiverseGeom1ShadingSmooth;
    m.cullFrontMask = LumiverseGeom1CullFront;
    m.cullBackMask = LumiverseGeom1CullBack;
    m.shadeMask = LumiverseGeomShade;
    if(dialect == LumiverseDialectGBI0) {
      m.triStride = gbi0Vertex == LumiverseGBI0VertexStandard ? 10 : 5;
    } else {
      m.triStride = 2;
    }
  }
  for(u32 index = 0; index < 16; index++) m.segments[index] = 0;
  //identity matrices
  for(u32 index = 0; index < 16; index++) {
    m.projStack[0][index] = (index % 5) == 0 ? 1.0f : 0.0f;
    m.mvStack[0][index] = (index % 5) == 0 ? 1.0f : 0.0f;
  }
  m.projDepth = 0;
  m.mvDepth = 0;
  m.combinedDirty = true;
  for(auto& vertex : m.verts) vertex = {};
  m.geometryMode = 0;
  //initial RDP other-mode shadow (n64js rsp_state.js reset values)
  m.otherModeL = 0x00500001;
  m.otherModeH = 0x00000000;
  m.otherModeDirty = true;
  m.vpScaleX = 160; m.vpScaleY = 120; m.vpScaleZ = 511;
  m.vpTransX = 160; m.vpTransY = 120; m.vpTransZ = 511;
  for(auto& light : m.lights) light = {};
  m.numLights = 0;
  m.fogMul = 0; m.fogOff = 0;
  m.texScaleS = 1.0f; m.texScaleT = 1.0f;
  m.texTile = 0; m.texLevel = 0;
  m.rdpHalf1 = 0;
  for(auto& size : m.tileSizes) size = {};
  m.bakedRDP = bakedRDP;
  m.bakedCount = 0;
}

//RDP command length in 64-bit words by 6-bit command code (the table
//paraLLEl-RDP's dispatch in vulkan.cpp uses; ares' own vendored source)
constexpr u32 lumiverseGfxRDPCommandQwords[64] = {
  1, 1, 1, 1, 1, 1, 1, 1, 4, 6,12,14,12,14,20,22,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

//expected word count of a baked RDP command from its first word (only the
//triangle family and the fixed-size RDP commands are accepted; anything
//else is rejected by the dry-run so the task falls back to LLE)
auto lumiverseGfxBakedExpectedWords(u32 firstWord) -> u32 {
  const u32 code = firstWord >> 24 & 0x3f;
  if(firstWord >> 24 < 0xc8) return 0;  //not an RDP command opcode
  return lumiverseGfxRDPCommandQwords[code] * 2;
}

//advance to the next display-list command; false when the list is finished
auto lumiverseGfxNextCommand(LumiverseGfxMachine& m) -> bool {
  if(!m.running) return false;
  m.cmd0 = lumiverseGfxWord(m.pc + 0);
  m.cmd1 = lumiverseGfxWord(m.pc + 4);
  m.pc += 8;
  return true;
}

auto lumiverseGfxEndDisplayList(LumiverseGfxMachine& m) -> void {
  if(m.stackDepth == 0) {
    m.running = false;
    return;
  }
  m.pc = m.stack[--m.stackDepth];
}

//----------------------------------------------------------------------------
//matrices (format from n64js gbi_microcode.js loadMatrix; row-vector math:
//clip = v * modelview * projection with matrices as stored in RDRAM)
//----------------------------------------------------------------------------

auto lumiverseGfxLoadMatrix(u32 address, f32* matrix) -> void {
  constexpr f32 recip = 1.0f / 65536.0f;
  for(u32 row = 0; row < 4; row++) {
    for(u32 col = 0; col < 4; col++) {
      const s16 integer = lumiverseGfxShort(address + row * 8 + col * 2);
      const u16 fraction = lumiverseGfxHalf(address + row * 8 + col * 2 + 32);
      matrix[row * 4 + col] = (f32)(((s32)integer << 16) | fraction) * recip;
    }
  }
}

//out = a * b (row-major product; both may not alias out)
auto lumiverseGfxMatrixMultiply(const f32* a, const f32* b, f32* out) -> void {
  for(u32 row = 0; row < 4; row++) {
    for(u32 col = 0; col < 4; col++) {
      out[row * 4 + col] =
        a[row * 4 + 0] * b[0 * 4 + col] +
        a[row * 4 + 1] * b[1 * 4 + col] +
        a[row * 4 + 2] * b[2 * 4 + col] +
        a[row * 4 + 3] * b[3 * 4 + col];
    }
  }
}

auto lumiverseGfxUpdateCombined(LumiverseGfxMachine& m) -> void {
  if(!m.combinedDirty) return;
  lumiverseGfxMatrixMultiply(m.mvStack[m.mvDepth], m.projStack[m.projDepth], m.combined);
  m.combinedDirty = false;
}

//----------------------------------------------------------------------------
//triangle emission — port of libdragon rdpq_tri.c rdpq_triangle_cpu()
//(Unlicense/public domain); emits EDGE/SHADE/TEXTURE/ZBUFFER blocks
//----------------------------------------------------------------------------

struct LumiverseEmitVertex {
  f32 x, y;           //screen pixels
  f32 r, g, b, a;     //0..1
  f32 s, t;           //texels
  f32 invW;
  f32 z;              //0..1; RDP z = z * 0x7fff
};

auto lumiverseFloatToS16_16(f32 f) -> s32 {
  if(f >= 32768.f) return 0x7fffffff;
  if(f < -32768.f) return (s32)0x80000000;
  return (s32)::floorf(f * 65536.f);
}

struct LumiverseTriEdgeData {
  f32 hx, hy, mx, my, fy, ish, attrFactor;
};

template<typename T> auto lumiverseClampValue(T v, T lo, T hi) -> T {
  return v < lo ? lo : v > hi ? hi : v;
}

auto lumiverseWriteEdgeCoeffs(LumiverseGfxOut& out, LumiverseTriEdgeData& data,
  u32 opcode, u32 tile, u32 level,
  const LumiverseEmitVertex* v1, const LumiverseEmitVertex* v2, const LumiverseEmitVertex* v3) -> void {
  const f32 x1 = v1->x;
  const f32 x2 = v2->x;
  const f32 x3 = v3->x;
  const f32 y1 = ::floorf(v1->y * 4) / 4;
  const f32 y2 = ::floorf(v2->y * 4) / 4;
  const f32 y3 = ::floorf(v3->y * 4) / 4;

  const f32 toFixed11_2 = 4.0f;
  const s32 y1f = lumiverseClampValue<s32>((s32)::floorf(v1->y * toFixed11_2), -4096 * 4, 4095 * 4);
  const s32 y2f = lumiverseClampValue<s32>((s32)::floorf(v2->y * toFixed11_2), -4096 * 4, 4095 * 4);
  const s32 y3f = lumiverseClampValue<s32>((s32)::floorf(v3->y * toFixed11_2), -4096 * 4, 4095 * 4);

  data.hx = x3 - x1;
  data.hy = y3 - y1;
  data.mx = x2 - x1;
  data.my = y2 - y1;
  const f32 lx = x3 - x2;
  const f32 ly = y3 - y2;

  const f32 nz = (data.hx * data.my) - (data.hy * data.mx);
  data.attrFactor = (::fabsf(nz) > 1.17549435e-38f) ? (-1.0f / nz) : 0;
  const u32 lft = nz < 0;

  data.ish = (::fabsf(data.hy) > 1.17549435e-38f) ? (data.hx / data.hy) : 0;
  const f32 ism = (::fabsf(data.my) > 1.17549435e-38f) ? (data.mx / data.my) : 0;
  const f32 isl = (::fabsf(ly) > 1.17549435e-38f) ? (lx / ly) : 0;
  data.fy = ::floorf(y1) - y1;

  const f32 xh = x1 + data.fy * data.ish;
  const f32 xm = x1 + data.fy * ism;
  const f32 xl = x2;

  lumiverseGfxEmit(out, (opcode << 24) | ((lft & 1) << 23) | ((level & 7) << 19) | ((tile & 7) << 16) | ((u32)y3f & 0x3fff));
  lumiverseGfxEmit(out, (((u32)y2f & 0x3fff) << 16) | ((u32)y1f & 0x3fff));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(xl));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(isl));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(xh));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(data.ish));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(xm));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(ism));
}

auto lumiverseWriteShadeCoeffs(LumiverseGfxOut& out, LumiverseTriEdgeData& data,
  const LumiverseEmitVertex* v1, const LumiverseEmitVertex* v2, const LumiverseEmitVertex* v3) -> void {
  const f32 mr = (v2->r - v1->r) * 255.f;
  const f32 mg = (v2->g - v1->g) * 255.f;
  const f32 mb = (v2->b - v1->b) * 255.f;
  const f32 ma = (v2->a - v1->a) * 255.f;
  const f32 hr = (v3->r - v1->r) * 255.f;
  const f32 hg = (v3->g - v1->g) * 255.f;
  const f32 hb = (v3->b - v1->b) * 255.f;
  const f32 ha = (v3->a - v1->a) * 255.f;

  const f32 nxR = data.hy * mr - data.my * hr;
  const f32 nxG = data.hy * mg - data.my * hg;
  const f32 nxB = data.hy * mb - data.my * hb;
  const f32 nxA = data.hy * ma - data.my * ha;
  const f32 nyR = data.mx * hr - data.hx * mr;
  const f32 nyG = data.mx * hg - data.hx * mg;
  const f32 nyB = data.mx * hb - data.hx * mb;
  const f32 nyA = data.mx * ha - data.hx * ma;

  const f32 DrDx = nxR * data.attrFactor;
  const f32 DgDx = nxG * data.attrFactor;
  const f32 DbDx = nxB * data.attrFactor;
  const f32 DaDx = nxA * data.attrFactor;
  const f32 DrDy = nyR * data.attrFactor;
  const f32 DgDy = nyG * data.attrFactor;
  const f32 DbDy = nyB * data.attrFactor;
  const f32 DaDy = nyA * data.attrFactor;

  const f32 DrDe = DrDy + DrDx * data.ish;
  const f32 DgDe = DgDy + DgDx * data.ish;
  const f32 DbDe = DbDy + DbDx * data.ish;
  const f32 DaDe = DaDy + DaDx * data.ish;

  const s32 finalR = lumiverseFloatToS16_16(v1->r * 255.f + data.fy * DrDe);
  const s32 finalG = lumiverseFloatToS16_16(v1->g * 255.f + data.fy * DgDe);
  const s32 finalB = lumiverseFloatToS16_16(v1->b * 255.f + data.fy * DbDe);
  const s32 finalA = lumiverseFloatToS16_16(v1->a * 255.f + data.fy * DaDe);

  const s32 DrDxF = lumiverseFloatToS16_16(DrDx);
  const s32 DgDxF = lumiverseFloatToS16_16(DgDx);
  const s32 DbDxF = lumiverseFloatToS16_16(DbDx);
  const s32 DaDxF = lumiverseFloatToS16_16(DaDx);

  const s32 DrDeF = lumiverseFloatToS16_16(DrDe);
  const s32 DgDeF = lumiverseFloatToS16_16(DgDe);
  const s32 DbDeF = lumiverseFloatToS16_16(DbDe);
  const s32 DaDeF = lumiverseFloatToS16_16(DaDe);

  const s32 DrDyF = lumiverseFloatToS16_16(DrDy);
  const s32 DgDyF = lumiverseFloatToS16_16(DgDy);
  const s32 DbDyF = lumiverseFloatToS16_16(DbDy);
  const s32 DaDyF = lumiverseFloatToS16_16(DaDy);

  lumiverseGfxEmit(out, ((u32)finalR & 0xffff0000) | (0xffff & ((u32)finalG >> 16)));
  lumiverseGfxEmit(out, ((u32)finalB & 0xffff0000) | (0xffff & ((u32)finalA >> 16)));
  lumiverseGfxEmit(out, ((u32)DrDxF & 0xffff0000) | (0xffff & ((u32)DgDxF >> 16)));
  lumiverseGfxEmit(out, ((u32)DbDxF & 0xffff0000) | (0xffff & ((u32)DaDxF >> 16)));
  lumiverseGfxEmit(out, ((u32)finalR << 16) | ((u32)finalG & 0xffff));
  lumiverseGfxEmit(out, ((u32)finalB << 16) | ((u32)finalA & 0xffff));
  lumiverseGfxEmit(out, ((u32)DrDxF << 16) | ((u32)DgDxF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DbDxF << 16) | ((u32)DaDxF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DrDeF & 0xffff0000) | (0xffff & ((u32)DgDeF >> 16)));
  lumiverseGfxEmit(out, ((u32)DbDeF & 0xffff0000) | (0xffff & ((u32)DaDeF >> 16)));
  lumiverseGfxEmit(out, ((u32)DrDyF & 0xffff0000) | (0xffff & ((u32)DgDyF >> 16)));
  lumiverseGfxEmit(out, ((u32)DbDyF & 0xffff0000) | (0xffff & ((u32)DaDyF >> 16)));
  lumiverseGfxEmit(out, ((u32)DrDeF << 16) | ((u32)DgDeF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DbDeF << 16) | ((u32)DaDeF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DrDyF << 16) | ((u32)DgDyF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DbDyF << 16) | ((u32)DaDyF & 0xffff));
}

auto lumiverseWriteTexCoeffs(LumiverseGfxOut& out, LumiverseTriEdgeData& data,
  const LumiverseEmitVertex* v1, const LumiverseEmitVertex* v2, const LumiverseEmitVertex* v3) -> void {
  f32 s1 = v1->s * 32.f, t1 = v1->t * 32.f, invw1 = v1->invW;
  f32 s2 = v2->s * 32.f, t2 = v2->t * 32.f, invw2 = v2->invW;
  f32 s3 = v3->s * 32.f, t3 = v3->t * 32.f, invw3 = v3->invW;

  const f32 maxInvW = invw1 > invw2 ? (invw1 > invw3 ? invw1 : invw3) : (invw2 > invw3 ? invw2 : invw3);
  const f32 minw = 1.0f / maxInvW;

  invw1 *= minw;
  invw2 *= minw;
  invw3 *= minw;

  s1 *= invw1;
  t1 *= invw1;
  s2 *= invw2;
  t2 *= invw2;
  s3 *= invw3;
  t3 *= invw3;

  invw1 *= 0x7fff;
  invw2 *= 0x7fff;
  invw3 *= 0x7fff;

  const f32 ms = s2 - s1;
  const f32 mt = t2 - t1;
  const f32 mw = invw2 - invw1;
  const f32 hs = s3 - s1;
  const f32 ht = t3 - t1;
  const f32 hw = invw3 - invw1;

  const f32 nxS = data.hy * ms - data.my * hs;
  const f32 nxT = data.hy * mt - data.my * ht;
  const f32 nxW = data.hy * mw - data.my * hw;
  const f32 nyS = data.mx * hs - data.hx * ms;
  const f32 nyT = data.mx * ht - data.hx * mt;
  const f32 nyW = data.mx * hw - data.hx * mw;

  const f32 DsDx = nxS * data.attrFactor;
  const f32 DtDx = nxT * data.attrFactor;
  const f32 DwDx = nxW * data.attrFactor;
  const f32 DsDy = nyS * data.attrFactor;
  const f32 DtDy = nyT * data.attrFactor;
  const f32 DwDy = nyW * data.attrFactor;

  const f32 DsDe = DsDy + DsDx * data.ish;
  const f32 DtDe = DtDy + DtDx * data.ish;
  const f32 DwDe = DwDy + DwDx * data.ish;

  const s32 finalS = lumiverseFloatToS16_16(s1 + data.fy * DsDe);
  const s32 finalT = lumiverseFloatToS16_16(t1 + data.fy * DtDe);
  const s32 finalW = lumiverseFloatToS16_16(invw1 + data.fy * DwDe);

  const s32 DsDxF = lumiverseFloatToS16_16(DsDx);
  const s32 DtDxF = lumiverseFloatToS16_16(DtDx);
  const s32 DwDxF = lumiverseFloatToS16_16(DwDx);

  const s32 DsDeF = lumiverseFloatToS16_16(DsDe);
  const s32 DtDeF = lumiverseFloatToS16_16(DtDe);
  const s32 DwDeF = lumiverseFloatToS16_16(DwDe);

  const s32 DsDyF = lumiverseFloatToS16_16(DsDy);
  const s32 DtDyF = lumiverseFloatToS16_16(DtDy);
  const s32 DwDyF = lumiverseFloatToS16_16(DwDy);

  lumiverseGfxEmit(out, ((u32)finalS & 0xffff0000) | (0xffff & ((u32)finalT >> 16)));
  lumiverseGfxEmit(out, ((u32)finalW & 0xffff0000));
  lumiverseGfxEmit(out, ((u32)DsDxF & 0xffff0000) | (0xffff & ((u32)DtDxF >> 16)));
  lumiverseGfxEmit(out, ((u32)DwDxF & 0xffff0000));
  lumiverseGfxEmit(out, ((u32)finalS << 16) | ((u32)finalT & 0xffff));
  lumiverseGfxEmit(out, ((u32)finalW << 16));
  lumiverseGfxEmit(out, ((u32)DsDxF << 16) | ((u32)DtDxF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DwDxF << 16));
  lumiverseGfxEmit(out, ((u32)DsDeF & 0xffff0000) | (0xffff & ((u32)DtDeF >> 16)));
  lumiverseGfxEmit(out, ((u32)DwDeF & 0xffff0000));
  lumiverseGfxEmit(out, ((u32)DsDyF & 0xffff0000) | (0xffff & ((u32)DtDyF >> 16)));
  lumiverseGfxEmit(out, ((u32)DwDyF & 0xffff0000));
  lumiverseGfxEmit(out, ((u32)DsDeF << 16) | ((u32)DtDeF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DwDeF << 16));
  lumiverseGfxEmit(out, ((u32)DsDyF << 16) | ((u32)DtDyF & 0xffff));
  lumiverseGfxEmit(out, ((u32)DwDyF << 16));
}

auto lumiverseWriteZBufCoeffs(LumiverseGfxOut& out, LumiverseTriEdgeData& data,
  const LumiverseEmitVertex* v1, const LumiverseEmitVertex* v2, const LumiverseEmitVertex* v3) -> void {
  const f32 z1 = v1->z * 0x7fff;
  const f32 z2 = v2->z * 0x7fff;
  const f32 z3 = v3->z * 0x7fff;

  const f32 mz = z2 - z1;
  const f32 hz = z3 - z1;

  const f32 nxz = data.hy * mz - data.my * hz;
  const f32 nyz = data.mx * hz - data.hx * mz;

  const f32 DzDx = nxz * data.attrFactor;
  const f32 DzDy = nyz * data.attrFactor;
  const f32 DzDe = DzDy + DzDx * data.ish;

  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(z1 + data.fy * DzDe));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(DzDx));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(DzDe));
  lumiverseGfxEmit(out, (u32)lumiverseFloatToS16_16(DzDy));
}

//emit one RDP triangle command (edge + optional shade/tex/z blocks)
auto lumiverseEmitTriangle(LumiverseGfxMachine& m, bool shade, bool tex, bool zbuf,
  const LumiverseEmitVertex* v1, const LumiverseEmitVertex* v2, const LumiverseEmitVertex* v3) -> void {
  u32 opcode = 0x08;
  if(shade) opcode |= 0x4;
  if(tex) opcode |= 0x2;
  if(zbuf) opcode |= 0x1;

  //sort by y (rdpq_triangle_cpu)
  if(v1->y > v2->y) { auto t = v1; v1 = v2; v2 = t; }
  if(v2->y > v3->y) { auto t = v2; v2 = v3; v3 = t; }
  if(v1->y > v2->y) { auto t = v1; v1 = v2; v2 = t; }

  LumiverseTriEdgeData data;
  lumiverseWriteEdgeCoeffs(m.out, data, opcode, m.texTile, m.texLevel, v1, v2, v3);
  if(shade) lumiverseWriteShadeCoeffs(m.out, data, v1, v2, v3);
  if(tex) lumiverseWriteTexCoeffs(m.out, data, v1, v2, v3);
  if(zbuf) lumiverseWriteZBufCoeffs(m.out, data, v1, v2, v3);
  if(m.out.count >= LumiverseGfxOut::FlushAt) lumiverseGfxFlush(m.out);
  m.trisEmitted++;
}

//----------------------------------------------------------------------------
//vertex transform pipeline
//----------------------------------------------------------------------------

auto lumiverseGfxCalcClipFlags(f32 x, f32 y, f32 z, f32 w) -> u8 {
  u8 flags = 0;
  if(x < -w) flags |= LumiverseClipNX;
  else if(x > w) flags |= LumiverseClipPX;
  if(y < -w) flags |= LumiverseClipNY;
  else if(y > w) flags |= LumiverseClipPY;
  if(z > w) flags |= LumiverseClipFar;
  else if(z < -w) flags |= LumiverseClipNear;
  return flags;
}

//lighting (port of n64js gbi_microcode.js calculateLighting; colors 0..255)
auto lumiverseGfxCalculateLighting(LumiverseGfxMachine& m, f32 nx, f32 ny, f32 nz, f32* outR, f32* outG, f32* outB) -> void {
  const u32 numLights = m.numLights <= 7 ? m.numLights : 7;
  f32 r = m.lights[numLights].r;
  f32 g = m.lights[numLights].g;
  f32 b = m.lights[numLights].b;
  for(u32 index = 0; index < numLights; index++) {
    const auto& light = m.lights[index];
    const f32 d = nx * light.x + ny * light.y + nz * light.z;
    if(d > 0.0f) {
      r += light.r * d;
      g += light.g * d;
      b += light.b * d;
    }
  }
  *outR = r < 255.f ? r : 255.f;
  *outG = g < 255.f ? g : 255.f;
  *outB = b < 255.f ? b : 255.f;
}

auto lumiverseGfxLoadVertices(LumiverseGfxMachine& m, u32 v0, u32 n, u32 address) -> void {
  if(v0 + n > 32) return;
  lumiverseGfxUpdateCombined(m);

  if(lumiverseGfxLogLevel() >= 3 && (m.geometryMode & LumiverseGeomLighting)) {
    static u32 logged = 0;
    if(logged++ < 24) {
      fprintf(stderr, "[rsp-hle-gfx] lighting: numLights=%u", m.numLights);
      for(u32 index = 0; index <= (m.numLights <= 7 ? m.numLights : 7); index++) {
        const auto& light = m.lights[index];
        fprintf(stderr, " L%u=(%.0f,%.0f,%.0f dir %.2f,%.2f,%.2f)", index,
          light.r, light.g, light.b, light.x, light.y, light.z);
      }
      fprintf(stderr, "\n");
    }
  }
  const f32* mv = m.mvStack[m.mvDepth];
  const f32* wvp = m.combined;

  const bool lighting = m.geometryMode & LumiverseGeomLighting;
  const bool texgen = m.geometryMode & LumiverseGeomTextureGen;
  const bool texgenLinear = m.geometryMode & LumiverseGeomTextureGenLinear;

  //texture coords are 11.5 fixed point; scale to texels (n64js loadVertices)
  const f32 scaleS = m.texScaleS / 32.0f;
  const f32 scaleT = m.texScaleT / 32.0f;

  for(u32 index = 0; index < n; index++) {
    const u32 base = address + index * 16;
    auto& vertex = m.verts[v0 + index];

    const f32 x = lumiverseGfxShort(base + 0);
    const f32 y = lumiverseGfxShort(base + 2);
    const f32 z = lumiverseGfxShort(base + 4);
    vertex.u = (f32)lumiverseGfxShort(base + 8) * scaleS;
    vertex.v = (f32)lumiverseGfxShort(base + 10) * scaleT;

    vertex.x = x * wvp[0] + y * wvp[4] + z * wvp[8]  + wvp[12];
    vertex.y = x * wvp[1] + y * wvp[5] + z * wvp[9]  + wvp[13];
    vertex.z = x * wvp[2] + y * wvp[6] + z * wvp[10] + wvp[14];
    vertex.w = x * wvp[3] + y * wvp[7] + z * wvp[11] + wvp[15];
    vertex.clip = lumiverseGfxCalcClipFlags(vertex.x, vertex.y, vertex.z, vertex.w);

    if(lighting) {
      const f32 nxRaw = lumiverseGfxSByte(base + 12);
      const f32 nyRaw = lumiverseGfxSByte(base + 13);
      const f32 nzRaw = lumiverseGfxSByte(base + 14);
      //transform normal by modelview 3x3 (row-vector), then normalize
      f32 nx = nxRaw * mv[0] + nyRaw * mv[4] + nzRaw * mv[8];
      f32 ny = nxRaw * mv[1] + nyRaw * mv[5] + nzRaw * mv[9];
      f32 nz = nxRaw * mv[2] + nyRaw * mv[6] + nzRaw * mv[10];
      const f32 lengthSquared = nx * nx + ny * ny + nz * nz;
      if(lengthSquared > 0.0f) {
        const f32 invLength = 1.0f / ::sqrtf(lengthSquared);
        nx *= invLength; ny *= invLength; nz *= invLength;
      }
      lumiverseGfxCalculateLighting(m, nx, ny, nz, &vertex.r, &vertex.g, &vertex.b);
      vertex.a = lumiverseGfxByte(base + 15);
      if(texgen) {
        //n64js projected_vertex.js texgen approximations, scaled to the
        //render tile's size (SetTileSize tracked per tile)
        const auto& size = m.tileSizes[m.texTile & 7];
        const f32 width = (f32)((size.lrs - size.uls) >> 2) + 1.0f;
        const f32 height = (f32)((size.lrt - size.ult) >> 2) + 1.0f;
        f32 u01, v01;
        if(texgenLinear) {
          u01 = ::acosf(lumiverseClampValue(nx, -1.0f, 1.0f)) * 0.318309886f;
          v01 = ::acosf(lumiverseClampValue(ny, -1.0f, 1.0f)) * 0.318309886f;
        } else {
          u01 = 0.5f * (1.0f + nx);
          v01 = 0.5f * (1.0f + ny);
        }
        vertex.u = u01 * width;
        vertex.v = v01 * height;
      }
    } else {
      vertex.r = lumiverseGfxByte(base + 12);
      vertex.g = lumiverseGfxByte(base + 13);
      vertex.b = lumiverseGfxByte(base + 14);
      vertex.a = lumiverseGfxByte(base + 15);
    }
  }
}

//----------------------------------------------------------------------------
//clipping + triangle assembly
//----------------------------------------------------------------------------

struct LumiverseClipVertex {
  f32 x, y, z, w;
  f32 u, v;
  f32 r, g, b, a;
};

auto lumiverseGfxLerpVertex(const LumiverseClipVertex& a, const LumiverseClipVertex& b, f32 t, LumiverseClipVertex& out) -> void {
  out.x = a.x + (b.x - a.x) * t;
  out.y = a.y + (b.y - a.y) * t;
  out.z = a.z + (b.z - a.z) * t;
  out.w = a.w + (b.w - a.w) * t;
  out.u = a.u + (b.u - a.u) * t;
  out.v = a.v + (b.v - a.v) * t;
  out.r = a.r + (b.r - a.r) * t;
  out.g = a.g + (b.g - a.g) * t;
  out.b = a.b + (b.b - a.b) * t;
  out.a = a.a + (b.a - a.a) * t;
}

//Sutherland-Hodgman against one plane; dot(v) >= 0 keeps the vertex.
//planeSelect: 0:w>=eps 1:x<=w 2:x>=-w 3:y<=w 4:y>=-w 5:z<=w (far)
auto lumiverseGfxPlaneDot(const LumiverseClipVertex& v, u32 planeSelect) -> f32 {
  switch(planeSelect) {
  case 0: return v.w - LumiverseWEpsilon;
  case 1: return v.w - v.x;
  case 2: return v.w + v.x;
  case 3: return v.w - v.y;
  case 4: return v.w + v.y;
  case 5: return v.w - v.z;
  }
  return 0;
}

auto lumiverseGfxClipPolygon(LumiverseClipVertex* verts, u32 count) -> u32 {
  LumiverseClipVertex scratch[16];
  for(u32 plane = 0; plane < 6; plane++) {
    if(count < 3) return 0;
    u32 outCount = 0;
    for(u32 index = 0; index < count; index++) {
      const auto& current = verts[index];
      const auto& next = verts[(index + 1) % count];
      const f32 dc = lumiverseGfxPlaneDot(current, plane);
      const f32 dn = lumiverseGfxPlaneDot(next, plane);
      if(dc >= 0) {
        if(outCount < 16) scratch[outCount++] = current;
      }
      if((dc >= 0) != (dn >= 0)) {
        const f32 t = dc / (dc - dn);
        if(outCount < 16) lumiverseGfxLerpVertex(current, next, t, scratch[outCount++]);
      }
    }
    count = outCount;
    for(u32 index = 0; index < count; index++) verts[index] = scratch[index];
  }
  return count;
}

//project a clip-space vertex to an emission vertex (viewport transform,
//fog, perspective attributes). n64js viewport convention: screen y flipped.
auto lumiverseGfxProject(LumiverseGfxMachine& m, const LumiverseClipVertex& in, LumiverseEmitVertex& out) -> void {
  const f32 invW = 1.0f / in.w;
  out.x = m.vpTransX + m.vpScaleX * (in.x * invW);
  out.y = m.vpTransY - m.vpScaleY * (in.y * invW);
  f32 sz = m.vpTransZ + m.vpScaleZ * (in.z * invW);
  sz = lumiverseClampValue(sz, 0.0f, 1023.0f);
  out.z = sz * 32.0f / 32767.0f;
  out.invW = invW;
  out.s = in.u;
  out.t = in.v;
  out.r = in.r * (1.0f / 255.0f);
  out.g = in.g * (1.0f / 255.0f);
  out.b = in.b * (1.0f / 255.0f);
  f32 alpha = in.a;
  if(m.geometryMode & LumiverseGeomFog) {
    alpha = lumiverseClampValue((in.z * invW) * m.fogMul + m.fogOff, 0.0f, 255.0f);
    if(lumiverseGfxLogLevel() >= 3) {
      static u32 logged = 0;
      if(logged++ < 32) {
        fprintf(stderr, "[rsp-hle-gfx] fog: z/w=%.3f mul=%.0f off=%.0f alpha=%.0f\n",
          in.z * invW, m.fogMul, m.fogOff, alpha);
      }
    }
  }
  out.a = alpha * (1.0f / 255.0f);
}

auto lumiverseGfxDrawTriangle(LumiverseGfxMachine& m, u32 index0, u32 index1, u32 index2, u32 flatIndex) -> void {
  if(index0 >= 32 || index1 >= 32 || index2 >= 32) return;
  const auto& a = m.verts[index0];
  const auto& b = m.verts[index1];
  const auto& c = m.verts[index2];

  //trivial rejection: all vertices outside the same frustum plane (near
  //excluded: F3DEX.NoN draws through the near plane, RDP scissors)
  if(a.clip & b.clip & c.clip & LumiverseClipCullMask) { m.trisRejected++; return; }

  LumiverseClipVertex poly[16];
  const LumiverseGfxVertex* source[3] = { &a, &b, &c };
  for(u32 index = 0; index < 3; index++) {
    const auto& v = *source[index];
    poly[index] = { v.x, v.y, v.z, v.w, v.u, v.v, v.r, v.g, v.b, v.a };
  }

  //flat shading: all three vertices take the flag vertex's color
  const bool smooth = m.geometryMode & m.smoothMask;
  if(!smooth) {
    const auto& flat = *source[flatIndex < 3 ? flatIndex : 0];
    for(u32 index = 0; index < 3; index++) {
      poly[index].r = flat.r;
      poly[index].g = flat.g;
      poly[index].b = flat.b;
      poly[index].a = flat.a;
    }
  }

  u32 count = 3;
  const u8 combinedFlags = a.clip | b.clip | c.clip;
  if(combinedFlags || a.w < LumiverseWEpsilon || b.w < LumiverseWEpsilon || c.w < LumiverseWEpsilon) {
    count = lumiverseGfxClipPolygon(poly, 3);
    if(count < 3) { m.trisRejected++; return; }
    m.trisClipped++;
  }

  LumiverseEmitVertex emitVerts[16];
  for(u32 index = 0; index < count; index++) {
    lumiverseGfxProject(m, poly[index], emitVerts[index]);
  }

  //backface culling on the projected polygon (shoelace; screen y is down).
  //Front faces are counter-clockwise in y-up convention (n64js/GL default),
  //i.e. negative signed area with y-down screen coordinates.
  const u32 cull = m.geometryMode & (m.cullFrontMask | m.cullBackMask);
  if(cull) {
    f32 area2 = 0;
    for(u32 index = 0; index < count; index++) {
      const auto& p = emitVerts[index];
      const auto& q = emitVerts[(index + 1) % count];
      area2 += p.x * q.y - q.x * p.y;
    }
    const bool front = area2 < 0;
    if((cull & m.cullBackMask) && !front) { m.trisRejected++; return; }
    if((cull & m.cullFrontMask) && front) { m.trisRejected++; return; }
  }

  //lazily emit pending other-mode state before drawing
  if(m.otherModeDirty) {
    lumiverseGfxEmit2(m.out, 0xef000000 | (m.otherModeH & 0x00ffffff), m.otherModeL);
    m.otherModeDirty = false;
  }

  const bool shade = m.shadeMask ? (m.geometryMode & m.shadeMask) != 0 : true;
  const bool tex = m.geometryMode & LumiverseGeomTextureEnable;
  const bool zbuf = m.geometryMode & LumiverseGeomZBuffer;
  if(lumiverseGfxLogLevel() >= 3) {
    static u32 logged = 0;
    if((logged++ & 0x3fff) == 0) {
      const auto& e = emitVerts[0];
      fprintf(stderr,
        "[rsp-hle-gfx] tri: shade=%d tex=%d z=%d gm=%08x om=%06x:%08x vp(z %.0f+%.0f) v0 xy(%.1f,%.1f) z01=%.3f rgba(%.0f,%.0f,%.0f,%.0f)\n",
        shade, tex, zbuf, m.geometryMode, m.otherModeH & 0xffffff, m.otherModeL,
        m.vpTransZ, m.vpScaleZ, e.x, e.y, e.z, e.r * 255, e.g * 255, e.b * 255, e.a * 255);
    }
  }
  for(u32 index = 2; index < count; index++) {
    lumiverseEmitTriangle(m, shade, tex, zbuf, &emitVerts[0], &emitVerts[index - 1], &emitVerts[index]);
  }
}

//----------------------------------------------------------------------------
//display-list interpreter (F3DEX / GBI1 command set, from n64js gbi1.js)
//----------------------------------------------------------------------------

auto lumiverseGfxLogUnimplementedOnce(u8 opcode, u32 cmd0, u32 cmd1) -> void {
  static bool logged[256];
  if(logged[opcode]) return;
  logged[opcode] = true;
  fprintf(stderr, "[rsp-hle-gfx] unimplemented op %02x cmd0=%08x cmd1=%08x\n", opcode, cmd0, cmd1);
}

//shared GBI1/GBI0 interpreter (Fast3D reuses the GBI1 command set with the
//divergences handled per-case above: G_VTX/TRI4/CULLDL/RDPHalf_Cont/0xb0)
auto lumiverseGfxExecuteTask(LumiverseGfxMachine& m, u32 pc,
  u32 dialect = LumiverseDialectGBI1, u32 gbi0Vertex = LumiverseGBI0VertexStandard, bool bakedRDP = false) -> bool {
  lumiverseGfxReset(m, pc, dialect, gbi0Vertex, bakedRDP);
  u32 safety = 0;

  while(lumiverseGfxNextCommand(m)) {
    if(++safety > 1000000) {
      fprintf(stderr, "[rsp-hle-gfx] runaway display list, aborting task\n");
      break;
    }
    const u32 cmd0 = m.cmd0;
    const u32 cmd1 = m.cmd1;
    const u8 opcode = cmd0 >> 24;

    switch(opcode) {

    case 0x00: break;  //G_SPNOOP
    case 0xc0: break;  //G_NOOP

    case 0x01: {  //G_MTX
      const u32 flags = (cmd0 >> 16) & 0xff;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      f32 matrix[16];
      lumiverseGfxLoadMatrix(address, matrix);

      const bool projection = flags & 0x01;  //G_MTX_PROJECTION
      const bool load = flags & 0x02;        //G_MTX_LOAD
      const bool push = flags & 0x04;        //G_MTX_PUSH

      f32* stackBase = projection ? &m.projStack[0][0] : &m.mvStack[0][0];
      u32& depth = projection ? m.projDepth : m.mvDepth;
      const u32 maxDepth = projection ? 7 : 31;
      f32* top = stackBase + depth * 16;

      f32 result[16];
      if(!load) {
        lumiverseGfxMatrixMultiply(matrix, top, result);  //incoming applied first
      } else {
        for(u32 index = 0; index < 16; index++) result[index] = matrix[index];
      }
      if(push && depth < maxDepth) depth++;
      f32* target = stackBase + depth * 16;
      for(u32 index = 0; index < 16; index++) target[index] = result[index];
      m.combinedDirty = true;
      break;
    }

    case 0xbd: {  //G_POPMTX (always modelview, per n64js)
      if(m.mvDepth > 0) m.mvDepth--;
      m.combinedDirty = true;
      break;
    }

    case 0x03: {  //G_MOVEMEM
      const u32 type = (cmd0 >> 16) & 0xff;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      switch(type) {
      case 0x80: {  //G_MV_VIEWPORT
        m.vpScaleX = (f32)lumiverseGfxShort(address + 0) / 4.0f;
        m.vpScaleY = (f32)lumiverseGfxShort(address + 2) / 4.0f;
        m.vpScaleZ = (f32)lumiverseGfxShort(address + 4);
        m.vpTransX = (f32)lumiverseGfxShort(address + 8) / 4.0f;
        m.vpTransY = (f32)lumiverseGfxShort(address + 10) / 4.0f;
        m.vpTransZ = (f32)lumiverseGfxShort(address + 12);
        break;
      }
      case 0x82: case 0x84: break;  //LOOKATY/LOOKATX: ignored (n64js)
      case 0x86: case 0x88: case 0x8a: case 0x8c:
      case 0x8e: case 0x90: case 0x92: case 0x94: {  //G_MV_L0..L7
        const u32 lightIndex = (type - 0x86) / 2;
        auto& light = m.lights[lightIndex];
        light.r = lumiverseGfxByte(address + 0);
        light.g = lumiverseGfxByte(address + 1);
        light.b = lumiverseGfxByte(address + 2);
        f32 x = lumiverseGfxSByte(address + 8);
        f32 y = lumiverseGfxSByte(address + 9);
        f32 z = lumiverseGfxSByte(address + 10);
        const f32 lengthSquared = x * x + y * y + z * z;
        if(lengthSquared > 0.0f) {
          const f32 invLength = 1.0f / ::sqrtf(lengthSquared);
          x *= invLength; y *= invLength; z *= invLength;
        }
        light.x = x; light.y = y; light.z = z;
        break;
      }
      default:
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        break;
      }
      break;
    }

    case 0xbc: {  //G_MOVEWORD
      const u32 type = cmd0 & 0xff;
      const u32 offset = (cmd0 >> 8) & 0xffff;
      switch(type) {
      case 0x02:  //G_MW_NUMLIGHT
        m.numLights = (((cmd1 - 0x80000000u) >> 5) - 1) & 7;
        break;
      case 0x04: break;  //G_MW_CLIP: RDP scissor handles it
      case 0x06:  //G_MW_SEGMENT
        m.segments[(offset >> 2) & 0xf] = cmd1 & 0x00ffffff;
        break;
      case 0x08:  //G_MW_FOG
        m.fogMul = (f32)(s16)(cmd1 >> 16);
        m.fogOff = (f32)(s16)(cmd1 & 0xffff);
        break;
      case 0x0a: {  //G_MW_LIGHTCOL
        const u32 lightIndex = (offset >> 5) & 7;
        if((offset & 0x1f) == 0) {
          auto& light = m.lights[lightIndex];
          light.r = (cmd1 >> 24) & 0xff;
          light.g = (cmd1 >> 16) & 0xff;
          light.b = (cmd1 >> 8) & 0xff;
        }
        break;
      }
      case 0x0e: break;  //G_MW_PERSPNORM: precision hint only
      default:
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        break;
      }
      break;
    }

    case 0x04: {  //G_VTX (encoding differs per dialect/variant; n64js gbi0.js/gbi1.js)
      u32 v0, n;
      if(m.dialect == LumiverseDialectGBI0) {
        if(m.gbi0Vertex == LumiverseGBI0VertexWaveRace) {
          v0 = ((cmd0 >> 16) & 0xff) / 5;
          n = (cmd0 >> 9) & 0x7f;
        } else if(m.gbi0Vertex == LumiverseGBI0VertexSOTE) {
          v0 = 0;
          n = (((cmd0 >> 4) & 0xfff) / 33) + 1;
        } else {
          v0 = (cmd0 >> 16) & 0xf;
          n = ((cmd0 >> 20) & 0xf) + 1;
        }
      } else {
        v0 = ((cmd0 >> 16) & 0xff) / 2;
        n = (cmd0 >> 10) & 0x3f;
      }
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      lumiverseGfxLoadVertices(m, v0, n, address);
      break;
    }

    case 0xb2: {  //GBI1: G_MODIFYVTX; GBI0: RDPHalf_Cont (n64js: warn + ignore);
                  //GoldenEye: continuation word of a baked RDP command
      if(m.bakedRDP) {
        if(m.bakedCount < 64) m.baked[m.bakedCount++] = cmd1;
        break;
      }
      if(m.dialect == LumiverseDialectGBI0) {
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        break;
      }
      const u32 where = (cmd0 >> 16) & 0xff;
      const u32 vertexIndex = (cmd0 & 0xffff) / 2;
      if(vertexIndex < 32) {
        auto& vertex = m.verts[vertexIndex];
        if(where == 0x10) {  //G_MWO_POINT_RGBA
          vertex.r = (cmd1 >> 24) & 0xff;
          vertex.g = (cmd1 >> 16) & 0xff;
          vertex.b = (cmd1 >> 8) & 0xff;
          vertex.a = (cmd1 >> 0) & 0xff;
        } else if(where == 0x14) {  //G_MWO_POINT_ST (10.5 fixed, tex scale applied)
          vertex.u = (f32)(s16)(cmd1 >> 16) * m.texScaleS / 32.0f;
          vertex.v = (f32)(s16)(cmd1 & 0xffff) * m.texScaleT / 32.0f;
        } else {
          lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        }
      }
      break;
    }

    case 0x06: {  //G_DL
      const u32 param = (cmd0 >> 16) & 0xff;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      if(param == 0) {  //G_DL_PUSH
        if(m.stackDepth < 32) m.stack[m.stackDepth++] = m.pc;
      }
      m.pc = address;
      break;
    }

    case 0xb8:  //G_ENDDL
      lumiverseGfxEndDisplayList(m);
      break;

    case 0xb0:  //GBI1: G_BRANCH_Z (n64js: branch always); GBI0: unknown (dry-run rejects)
      if(m.dialect == LumiverseDialectGBI0) {
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        break;
      }
      m.pc = lumiverseGfxSegmentAddress(m, m.rdpHalf1);
      break;

    case 0xbe: {  //G_CULLDL (GBI0 packs vertex-buffer offsets /40; n64js gbi0.js)
      u32 begin, end;
      if(m.dialect == LumiverseDialectGBI0) {
        begin = ((cmd0 & 0x00ffffff) / 40) & 0xf;
        end = (cmd1 / 40) & 0xf;
      } else {
        begin = (cmd0 & 0xffff) >> 1;
        end = (cmd1 & 0xffff) >> 1;
      }
      if(end > begin && end < 32) {
        u8 flags = 0xff;
        for(u32 index = begin; index <= end; index++) flags &= m.verts[index].clip;
        if(flags & LumiverseClipCullMask) lumiverseGfxEndDisplayList(m);
      }
      break;
    }

    case 0xb6:  //G_CLEARGEOMETRYMODE
      m.geometryMode &= ~cmd1;
      break;
    case 0xb7:  //G_SETGEOMETRYMODE
      m.geometryMode |= cmd1;
      break;

    case 0xb9: {  //G_SETOTHERMODE_L
      const u32 shift = (cmd0 >> 8) & 0xff;
      const u32 length = cmd0 & 0xff;
      const u32 mask = length >= 32 ? 0xffffffffu : (((1u << length) - 1) << shift);
      m.otherModeL = (m.otherModeL & ~mask) | cmd1;
      m.otherModeDirty = true;
      break;
    }
    case 0xba: {  //G_SETOTHERMODE_H
      const u32 shift = (cmd0 >> 8) & 0xff;
      const u32 length = cmd0 & 0xff;
      const u32 mask = length >= 32 ? 0xffffffffu : (((1u << length) - 1) << shift);
      m.otherModeH = (m.otherModeH & ~mask) | cmd1;
      m.otherModeDirty = true;
      break;
    }
    case 0xef:  //G_RDPSETOTHERMODE
      m.otherModeH = cmd0 & 0x00ffffff;
      m.otherModeL = cmd1;
      m.otherModeDirty = true;
      break;

    case 0xbb: {  //G_TEXTURE
      const u32 level = (cmd0 >> 11) & 0x7;
      const u32 tile = (cmd0 >> 8) & 0x7;
      const u32 on = cmd0 & 0xff;
      const u32 rawS = (cmd1 >> 16) & 0xffff;
      const u32 rawT = cmd1 & 0xffff;
      m.texScaleS = (rawS == 0 || rawS == 0xffff) ? 1.0f : (f32)rawS / 65536.0f;
      m.texScaleT = (rawT == 0 || rawT == 0xffff) ? 1.0f : (f32)rawT / 65536.0f;
      m.texLevel = level;
      m.texTile = tile;
      if(on) m.geometryMode |= LumiverseGeomTextureEnable;
      else m.geometryMode &= ~LumiverseGeomTextureEnable;
      break;
    }

    case 0xb4:  //G_RDPHALF_1 (GoldenEye: first word of a baked RDP command)
      m.rdpHalf1 = cmd1;
      if(m.bakedRDP && m.bakedCount < 64) m.baked[m.bakedCount++] = cmd1;
      break;
    case 0xb3:  //G_RDPHALF_2 (GoldenEye: final word -> emit the baked command)
      if(m.bakedRDP) {
        if(m.bakedCount < 64) m.baked[m.bakedCount++] = cmd1;
        const u32 expected = m.bakedCount ? lumiverseGfxBakedExpectedWords(m.baked[0]) : 0;
        if(expected && expected == m.bakedCount) {
          //same lazily-flushed other-mode as every other draw
          if(m.otherModeDirty) {
            lumiverseGfxEmit2(m.out, 0xef000000 | (m.otherModeH & 0x00ffffff), m.otherModeL);
            m.otherModeDirty = false;
          }
          for(u32 index = 0; index + 1 < m.bakedCount; index += 2) lumiverseGfxEmit2(m.out, m.baked[index], m.baked[index + 1]);
          m.trisEmitted += (m.baked[0] >> 24) >= 0xc8 && (m.baked[0] >> 24) <= 0xcf;
        } else {
          //the dry-run validated every sequence, so this cannot happen; fail
          //the task rather than emit a partial command
          m.out.failed = true;
        }
        m.bakedCount = 0;
      }
      break;

    case 0xbf: {  //G_TRI1 (byte indices divided by the dialect's stride)
      //flag selects which of the triangle's three corners provides the
      //flat-shade color (gsSP1Triangle flag parameter)
      const u32 flag = (cmd1 >> 24) & 0xff;
      lumiverseGfxDrawTriangle(m,
        ((cmd1 >> 16) & 0xff) / m.triStride,
        ((cmd1 >> 8) & 0xff) / m.triStride,
        ((cmd1 >> 0) & 0xff) / m.triStride,
        flag);
      break;
    }

    case 0xb1: {  //GBI1: G_TRI2; GBI0: G_TRI4 (nibble indices; n64js gbi0.js)
      if(m.dialect == LumiverseDialectGBI0) {
        const u32 index00 = (cmd0 >> 0) & 0xf, index01 = (cmd1 >> 0) & 0xf, index02 = (cmd1 >> 4) & 0xf;
        const u32 index03 = (cmd0 >> 4) & 0xf, index04 = (cmd1 >> 8) & 0xf, index05 = (cmd1 >> 12) & 0xf;
        const u32 index06 = (cmd0 >> 8) & 0xf, index07 = (cmd1 >> 16) & 0xf, index08 = (cmd1 >> 20) & 0xf;
        const u32 index09 = (cmd0 >> 12) & 0xf, index10 = (cmd1 >> 24) & 0xf, index11 = (cmd1 >> 28) & 0xf;
        if(index00 != index01) lumiverseGfxDrawTriangle(m, index00, index01, index02, 0);
        if(index03 != index04) lumiverseGfxDrawTriangle(m, index03, index04, index05, 0);
        if(index06 != index07) lumiverseGfxDrawTriangle(m, index06, index07, index08, 0);
        if(index09 != index10) lumiverseGfxDrawTriangle(m, index09, index10, index11, 0);
        break;
      }
      lumiverseGfxDrawTriangle(m,
        ((cmd0 >> 16) & 0xff) / m.triStride,
        ((cmd0 >> 8) & 0xff) / m.triStride,
        ((cmd0 >> 0) & 0xff) / m.triStride, 0);
      lumiverseGfxDrawTriangle(m,
        ((cmd1 >> 16) & 0xff) / m.triStride,
        ((cmd1 >> 8) & 0xff) / m.triStride,
        ((cmd1 >> 0) & 0xff) / m.triStride, 0);
      break;
    }

    case 0xb5: {  //G_QUAD / "Line3D" (n64js gbi1.js: two triangles)
      const u32 index3 = ((cmd1 >> 24) & 0xff) / m.triStride;
      const u32 index0 = ((cmd1 >> 16) & 0xff) / m.triStride;
      const u32 index1 = ((cmd1 >> 8) & 0xff) / m.triStride;
      const u32 index2 = ((cmd1 >> 0) & 0xff) / m.triStride;
      lumiverseGfxDrawTriangle(m, index0, index1, index2, 0);
      lumiverseGfxDrawTriangle(m, index2, index3, index0, 0);
      break;
    }

    //--- RDP passthrough ---------------------------------------------------

    case 0xe4:    //G_TEXRECT
    case 0xe5: {  //G_TEXRECTFLIP
      //two following GBI words carry the ST / DsDx-DtDy halves (n64js
      //gbi_microcode.js executeTexRect reads the next two commands' cmd1)
      lumiverseGfxNextCommand(m);
      const u32 cmd2 = m.cmd1;
      lumiverseGfxNextCommand(m);
      const u32 cmd3 = m.cmd1;
      if(m.otherModeDirty) {
        lumiverseGfxEmit2(m.out, 0xef000000 | (m.otherModeH & 0x00ffffff), m.otherModeL);
        m.otherModeDirty = false;
      }
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      lumiverseGfxEmit2(m.out, cmd2, cmd3);
      break;
    }

    case 0xf6:  //G_FILLRECT
      if(m.otherModeDirty) {
        lumiverseGfxEmit2(m.out, 0xef000000 | (m.otherModeH & 0x00ffffff), m.otherModeL);
        m.otherModeDirty = false;
      }
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      break;

    case 0xfd:  //G_SETTIMG (segment-translated address)
    case 0xfe:  //G_SETZIMG
    case 0xff:  //G_SETCIMG
      lumiverseGfxEmit2(m.out, cmd0, lumiverseGfxSegmentAddress(m, cmd1));
      break;

    case 0xf2: {  //G_SETTILESIZE (tracked for texgen, then passed through)
      auto& size = m.tileSizes[(cmd1 >> 24) & 7];
      size.uls = (cmd0 >> 12) & 0xfff;
      size.ult = (cmd0 >> 0) & 0xfff;
      size.lrs = (cmd1 >> 12) & 0xfff;
      size.lrt = (cmd1 >> 0) & 0xfff;
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      break;
    }

    case 0xe6:  //G_RDPLOADSYNC
    case 0xe7:  //G_RDPPIPESYNC
    case 0xe8:  //G_RDPTILESYNC
    case 0xe9:  //G_RDPFULLSYNC
    case 0xea:  //G_SETKEYGB
    case 0xeb:  //G_SETKEYR
    case 0xec:  //G_SETCONVERT
    case 0xed:  //G_SETSCISSOR
    case 0xee:  //G_SETPRIMDEPTH
    case 0xf0:  //G_LOADTLUT
    case 0xf3:  //G_LOADBLOCK
    case 0xf4:  //G_LOADTILE
    case 0xf5:  //G_SETTILE
    case 0xf7:  //G_SETFILLCOLOR
    case 0xf8:  //G_SETFOGCOLOR
    case 0xf9:  //G_SETBLENDCOLOR
    case 0xfa:  //G_SETPRIMCOLOR
    case 0xfb:  //G_SETENVCOLOR
    case 0xfc:  //G_SETCOMBINE
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      break;

    default:
      //dry-run should have caught this; skip to stay consistent
      lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
      break;
    }
  }

  lumiverseGfxFlush(m.out);
  return !m.out.failed;
}

//----------------------------------------------------------------------------
//display-list interpreter, GBI2 dialect (F3DEX2 / F3DZEX; n64js gbi2.js)
//----------------------------------------------------------------------------

//screen-space depth of a stored vertex (viewport z units, 0..~1023)
auto lumiverseGfxVertexScreenZ(LumiverseGfxMachine& m, u32 index) -> f32 {
  const auto& vertex = m.verts[index];
  if(vertex.w < LumiverseWEpsilon) return 0.0f;
  const f32 sz = m.vpTransZ + m.vpScaleZ * (vertex.z / vertex.w);
  return lumiverseClampValue(sz, 0.0f, 1023.0f);
}

auto lumiverseGfxEmitOtherMode(LumiverseGfxMachine& m) -> void {
  if(!m.otherModeDirty) return;
  lumiverseGfxEmit2(m.out, 0xef000000 | (m.otherModeH & 0x00ffffff), m.otherModeL);
  m.otherModeDirty = false;
}

auto lumiverseGfxExecuteTaskGBI2(LumiverseGfxMachine& m, u32 pc) -> bool {
  lumiverseGfxReset(m, pc, LumiverseDialectGBI2);
  u32 safety = 0;
  bool combinedForced = false;

  while(lumiverseGfxNextCommand(m)) {
    if(++safety > 1000000) {
      fprintf(stderr, "[rsp-hle-gfx] runaway display list, aborting task\n");
      break;
    }
    const u32 cmd0 = m.cmd0;
    const u32 cmd1 = m.cmd1;
    const u8 opcode = cmd0 >> 24;

    switch(opcode) {

    case 0x00: break;  //G_NOOP
    case 0xe0: break;  //G_SPNOOP

    case 0x01: {  //G_VTX: n in bits 12-19, vend*2 in low byte, v0 = vend - n
      const u32 n = (cmd0 >> 12) & 0xff;
      const u32 vend = (cmd0 & 0xff) >> 1;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      if(vend >= n) lumiverseGfxLoadVertices(m, vend - n, n, address);
      break;
    }

    case 0x02: {  //G_MODIFYVTX
      const u32 where = (cmd0 >> 16) & 0xff;
      const u32 vertexIndex = (cmd0 >> 1) & 0x7fff;
      if(vertexIndex < 32) {
        auto& vertex = m.verts[vertexIndex];
        if(where == 0x10) {  //G_MWO_POINT_RGBA
          vertex.r = (cmd1 >> 24) & 0xff;
          vertex.g = (cmd1 >> 16) & 0xff;
          vertex.b = (cmd1 >> 8) & 0xff;
          vertex.a = (cmd1 >> 0) & 0xff;
        } else if(where == 0x14) {  //G_MWO_POINT_ST
          vertex.u = (f32)(s16)(cmd1 >> 16) * m.texScaleS / 32.0f;
          vertex.v = (f32)(s16)(cmd1 & 0xffff) * m.texScaleT / 32.0f;
        } else {
          lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        }
      }
      break;
    }

    case 0x03: {  //G_CULLDL
      const u32 begin = (cmd0 & 0xffff) >> 1;
      const u32 end = (cmd1 & 0xffff) >> 1;
      if(end > begin && end < 32) {
        u8 flags = 0xff;
        for(u32 index = begin; index <= end; index++) flags &= m.verts[index].clip;
        if(flags & LumiverseClipCullMask) lumiverseGfxEndDisplayList(m);
      }
      break;
    }

    case 0x04: {  //G_BRANCH_Z: branch to RDPHALF_1 target if vtx depth <= zval
      //vertex index is encoded twice (idx*5 in bits 12-23, idx*2 in bits 1-11)
      const u32 vertexIndex = ((cmd0 >> 12) & 0xfff) / 5;
      bool branch = true;
      if(vertexIndex < 32) {
        //compare in RDP z units (screen z * 32, i.e. 0..0x7fe0) against the
        //command's 32-bit zval; scale verified by logging OoT values
        const f32 sz = lumiverseGfxVertexScreenZ(m, vertexIndex);
        const s32 vertexZ = (s32)(sz * 32.0f);
        branch = vertexZ <= (s32)cmd1;
        if(lumiverseGfxLogLevel() >= 2) {
          static u32 logged = 0;
          if(logged++ < 32) {
            fprintf(stderr, "[rsp-hle-gfx] branchZ vtx=%u sz=%.1f z32=%d zval=%d(0x%08x) -> %s\n",
              vertexIndex, sz, vertexZ, (s32)cmd1, cmd1, branch ? "branch" : "continue");
          }
        }
      }
      if(branch) m.pc = lumiverseGfxSegmentAddress(m, m.rdpHalf1);
      break;
    }

    case 0x05: {  //G_TRI1 (provoking vertex is corner 0; macro pre-rotates)
      lumiverseGfxDrawTriangle(m,
        (cmd0 >> 17) & 0x7f,
        (cmd0 >> 9) & 0x7f,
        (cmd0 >> 1) & 0x7f, 0);
      break;
    }

    case 0x06:    //G_TRI2
    case 0x07: {  //G_QUAD (same encoding)
      lumiverseGfxDrawTriangle(m,
        (cmd0 >> 17) & 0x7f,
        (cmd0 >> 9) & 0x7f,
        (cmd0 >> 1) & 0x7f, 0);
      lumiverseGfxDrawTriangle(m,
        (cmd1 >> 17) & 0x7f,
        (cmd1 >> 9) & 0x7f,
        (cmd1 >> 1) & 0x7f, 0);
      break;
    }

    case 0xd5: {  //G_SPECIAL_1 (Super Smash Bros., fifo 2.04H): observed only
                  //as d5000001/00000000 between a G_MTX and a run of
                  //G_MW_MATRIX patches. The only semantics under which those
                  //patches can matter is "materialize the combined MVP now"
                  //(so the lazy MV x P recompute cannot clobber them); frames
                  //validated against LLE checkpoints — see M1-REPORT.
      static u32 seen = 0;
      if(cmd1 != 0 || (cmd0 & 0x00ffffff) != 1) {
        if(seen++ < 4) fprintf(stderr, "[rsp-hle-gfx] G_SPECIAL_1 variant cmd0=%08x cmd1=%08x\n", cmd0, cmd1);
      }
      lumiverseGfxUpdateCombined(m);
      break;
    }
    case 0xd6: break;  //G_DMA_IO: no-op (n64js)

    case 0xd7: {  //G_TEXTURE
      const u32 level = (cmd0 >> 11) & 0x7;
      const u32 tile = (cmd0 >> 8) & 0x7;
      const u32 on = (cmd0 >> 1) & 0x01;
      const u32 rawS = (cmd1 >> 16) & 0xffff;
      const u32 rawT = cmd1 & 0xffff;
      m.texScaleS = (rawS == 0 || rawS == 0xffff) ? 1.0f : (f32)rawS / 65536.0f;
      m.texScaleT = (rawT == 0 || rawT == 0xffff) ? 1.0f : (f32)rawT / 65536.0f;
      m.texLevel = level;
      m.texTile = tile;
      if(on) m.geometryMode |= LumiverseGeomTextureEnable;
      else m.geometryMode &= ~LumiverseGeomTextureEnable;
      break;
    }

    case 0xd8: {  //G_POPMTX: cmd1 = bytes of matrices to pop (64 each)
      u32 count = cmd1 >> 6;
      if(count == 0) count = 1;
      while(count-- && m.mvDepth > 0) m.mvDepth--;
      m.combinedDirty = true;
      combinedForced = false;
      break;
    }

    case 0xd9: {  //G_GEOMETRYMODE: clear ~(cmd0 & 0xffffff), set cmd1
      //texture enable is ucode-internal, controlled only by G_TEXTURE
      m.geometryMode &= ((cmd0 & 0x00ffffff) | LumiverseGeomTextureEnable);
      m.geometryMode |= (cmd1 & ~LumiverseGeomTextureEnable);
      break;
    }

    case 0xda: {  //G_MTX: push = bit0 clear, load = bit1, projection = bit2
      const bool push = (cmd0 & 0x1) == 0;
      const bool load = (cmd0 >> 1) & 0x1;
      const bool projection = (cmd0 >> 2) & 0x1;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      f32 matrix[16];
      lumiverseGfxLoadMatrix(address, matrix);

      f32* stackBase = projection ? &m.projStack[0][0] : &m.mvStack[0][0];
      u32& depth = projection ? m.projDepth : m.mvDepth;
      const u32 maxDepth = projection ? 7 : 31;
      f32* top = stackBase + depth * 16;

      f32 result[16];
      if(!load) {
        lumiverseGfxMatrixMultiply(matrix, top, result);
      } else {
        for(u32 index = 0; index < 16; index++) result[index] = matrix[index];
      }
      if(push && depth < maxDepth) depth++;
      f32* target = stackBase + depth * 16;
      for(u32 index = 0; index < 16; index++) target[index] = result[index];
      m.combinedDirty = true;
      combinedForced = false;
      break;
    }

    case 0xdb: {  //G_MOVEWORD: type in bits 16-23, offset in low 16
      const u32 type = (cmd0 >> 16) & 0xff;
      const u32 offset = cmd0 & 0xffff;
      switch(type) {
      case 0x00: {  //G_MW_MATRIX: patch one 32-bit word of the combined MVP
        //(Super Smash Bros., fifo 2.04H). Empirical (own DL dumps): the game
        //issues G_MTX, then 0xd5 (see below), then a run of these at offsets
        //0x00..0x3c — the 64-byte DMEM matrix image, s16 integer parts at
        //0x00..0x1f, u16 fraction parts at 0x20..0x3f, each word covering
        //two row-major elements. The lazily-recomputed MVP must already be
        //current (0xd5 does that) or the patch would be lost at the next
        //vertex load; materialize defensively here as well.
        lumiverseGfxUpdateCombined(m);
        const u32 element = (offset & 0x1e) >> 1;
        if(element + 1 < 16) {
          const bool fraction = (offset & 0x20) != 0;
          const u32 halves[2] = { cmd1 >> 16, cmd1 & 0xffff };
          for(u32 index = 0; index < 2; index++) {
            f32& value = m.combined[element + index];
            s32 fixed = (s32)::lrintf(value * 65536.0f);
            if(fraction) fixed = (fixed & ~0xffff) | (s32)halves[index];
            else         fixed = ((s32)(s16)halves[index] << 16) | (fixed & 0xffff);
            value = (f32)fixed / 65536.0f;
          }
        }
        break;
      }
      case 0x02:  //G_MW_NUMLIGHT: value = numLights * 24
        m.numLights = (cmd1 / 24) & 7;
        break;
      case 0x04: break;  //G_MW_CLIP
      case 0x06:  //G_MW_SEGMENT
        m.segments[(offset >> 2) & 0xf] = cmd1 & 0x00ffffff;
        break;
      case 0x08:  //G_MW_FOG
        m.fogMul = (f32)(s16)(cmd1 >> 16);
        m.fogOff = (f32)(s16)(cmd1 & 0xffff);
        break;
      case 0x0a: {  //G_MW_LIGHTCOL (GBI2: 24-byte light stride)
        if((offset % 0x18) == 0) {
          const u32 lightIndex = (offset / 0x18) & 7;
          auto& light = m.lights[lightIndex];
          light.r = (cmd1 >> 24) & 0xff;
          light.g = (cmd1 >> 16) & 0xff;
          light.b = (cmd1 >> 8) & 0xff;
        }
        break;
      }
      case 0x0c: break;  //G_MW_FORCEMTX flag: matrix itself arrives via movemem
      case 0x0e: break;  //G_MW_PERSPNORM
      default:
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        break;
      }
      break;
    }

    case 0xdc: {  //G_MOVEMEM: type = cmd0 & 0xfe, offset = ((cmd0>>8)&0xff)*8
      const u32 type = cmd0 & 0xfe;
      const u32 offset = ((cmd0 >> 8) & 0xff) << 3;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      switch(type) {
      case 8:  //G_GBI2_MV_VIEWPORT
        m.vpScaleX = (f32)lumiverseGfxShort(address + 0) / 4.0f;
        m.vpScaleY = (f32)lumiverseGfxShort(address + 2) / 4.0f;
        m.vpScaleZ = (f32)lumiverseGfxShort(address + 4);
        m.vpTransX = (f32)lumiverseGfxShort(address + 8) / 4.0f;
        m.vpTransY = (f32)lumiverseGfxShort(address + 10) / 4.0f;
        m.vpTransZ = (f32)lumiverseGfxShort(address + 12);
        break;
      case 10: {  //G_GBI2_MV_LIGHT: offset 0/24 = lookat (ignored), 48+ = lights
        if(offset >= 48) {
          const u32 lightIndex = (offset - 48) / 24;
          if(lightIndex < 8) {
            auto& light = m.lights[lightIndex];
            light.r = lumiverseGfxByte(address + 0);
            light.g = lumiverseGfxByte(address + 1);
            light.b = lumiverseGfxByte(address + 2);
            f32 x = lumiverseGfxSByte(address + 8);
            f32 y = lumiverseGfxSByte(address + 9);
            f32 z = lumiverseGfxSByte(address + 10);
            const f32 lengthSquared = x * x + y * y + z * z;
            if(lengthSquared > 0.0f) {
              const f32 invLength = 1.0f / ::sqrtf(lengthSquared);
              x *= invLength; y *= invLength; z *= invLength;
            }
            light.x = x; light.y = y; light.z = z;
          }
        }
        break;
      }
      case 14: {  //G_GBI2_MV_MATRIX: gSPForceMatrix — replaces combined MVP
        lumiverseGfxLoadMatrix(address, m.combined);
        m.combinedDirty = false;
        combinedForced = true;
        break;
      }
      default:
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
        break;
      }
      break;
    }

    case 0xde: {  //G_DL
      const u32 param = (cmd0 >> 16) & 0xff;
      const u32 address = lumiverseGfxSegmentAddress(m, cmd1);
      if(param == 0) {
        if(m.stackDepth < 32) m.stack[m.stackDepth++] = m.pc;
      }
      m.pc = address;
      break;
    }

    case 0xdf:  //G_ENDDL
      lumiverseGfxEndDisplayList(m);
      break;

    case 0xe1:  //G_RDPHALF_1
      m.rdpHalf1 = cmd1;
      break;
    case 0xf1:  //G_RDPHALF_2
      break;

    case 0xe2: {  //G_SETOTHERMODE_L (GBI2 mask: top (len+1) bits >> shift)
      const u32 shift = (cmd0 >> 8) & 0xff;
      const u32 length = cmd0 & 0xff;
      const u32 mask = ((u32)((s32)0x80000000 >> length)) >> shift;
      m.otherModeL = (m.otherModeL & ~mask) | cmd1;
      m.otherModeDirty = true;
      break;
    }
    case 0xe3: {  //G_SETOTHERMODE_H
      const u32 shift = (cmd0 >> 8) & 0xff;
      const u32 length = cmd0 & 0xff;
      const u32 mask = ((u32)((s32)0x80000000 >> length)) >> shift;
      m.otherModeH = (m.otherModeH & ~mask) | cmd1;
      m.otherModeDirty = true;
      break;
    }
    case 0xef:  //G_RDPSETOTHERMODE
      m.otherModeH = cmd0 & 0x00ffffff;
      m.otherModeL = cmd1;
      m.otherModeDirty = true;
      break;

    //--- RDP passthrough (same as GBI1) ------------------------------------

    case 0xe4:    //G_TEXRECT
    case 0xe5: {  //G_TEXRECTFLIP
      lumiverseGfxNextCommand(m);
      const u32 cmd2 = m.cmd1;
      lumiverseGfxNextCommand(m);
      const u32 cmd3 = m.cmd1;
      lumiverseGfxEmitOtherMode(m);
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      lumiverseGfxEmit2(m.out, cmd2, cmd3);
      break;
    }

    case 0xf6:  //G_FILLRECT
      lumiverseGfxEmitOtherMode(m);
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      break;

    case 0xfd:  //G_SETTIMG
    case 0xfe:  //G_SETZIMG
    case 0xff:  //G_SETCIMG
      lumiverseGfxEmit2(m.out, cmd0, lumiverseGfxSegmentAddress(m, cmd1));
      break;

    case 0xf2: {  //G_SETTILESIZE
      auto& size = m.tileSizes[(cmd1 >> 24) & 7];
      size.uls = (cmd0 >> 12) & 0xfff;
      size.ult = (cmd0 >> 0) & 0xfff;
      size.lrs = (cmd1 >> 12) & 0xfff;
      size.lrt = (cmd1 >> 0) & 0xfff;
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      break;
    }

    case 0xe6: case 0xe7: case 0xe8: case 0xe9:  //RDP syncs
    case 0xea: case 0xeb: case 0xec:             //setkey/setconvert
    case 0xed: case 0xee:                        //scissor, primdepth
    case 0xf0: case 0xf3: case 0xf4: case 0xf5:  //tlut/loads/settile
    case 0xf7: case 0xf8: case 0xf9: case 0xfa:  //fill/fog/blend/prim colors
    case 0xfb: case 0xfc:                        //env color, combine
      lumiverseGfxEmit2(m.out, cmd0, cmd1);
      break;

    default:
      lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
      break;
    }
  }

  lumiverseGfxFlush(m.out);
  return !m.out.failed;
}

//----------------------------------------------------------------------------
//dry-run: walk the display list without emitting; returns false if any
//unsupported opcode is reachable so the whole task can fall back to LLE
//----------------------------------------------------------------------------

//per-opcode census across all dry-runs (logged for diagnostics)
struct LumiverseGfxCensus {
  u64 counts[256] = {};
  bool seen[256] = {};
};

auto lumiverseGfxOpcodeSupported(u8 opcode) -> bool {
  switch(opcode) {
  case 0x00: case 0x01: case 0x03: case 0x04: case 0x06:
  case 0xb0: case 0xb1: case 0xb2: case 0xb3: case 0xb4:
  case 0xb5: case 0xb6: case 0xb7: case 0xb8: case 0xb9:
  case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe:
  case 0xbf: case 0xc0:
  case 0xe4: case 0xe5: case 0xe6: case 0xe7: case 0xe8:
  case 0xe9: case 0xea: case 0xeb: case 0xec: case 0xed:
  case 0xee: case 0xef: case 0xf0: case 0xf2: case 0xf3:
  case 0xf4: case 0xf5: case 0xf6: case 0xf7: case 0xf8:
  case 0xf9: case 0xfa: case 0xfb: case 0xfc: case 0xfd:
  case 0xfe: case 0xff:
    return true;
  default:
    return false;
  }
}

//one line per distinct silent rejection reason (the dry-run used to fall a
//task back to LLE for these without any trace — GoldenEye's in-game
//fallbacks were invisible in the logs); counts are kept for the summary
u64 lumiverseGfxRejectCounts[8];
auto lumiverseGfxLogRejectOnce(u32 reason, u8 opcode, u32 cmd0, u32 cmd1, const char* what) -> void {
  static bool logged[8];
  if(reason < 8) lumiverseGfxRejectCounts[reason]++;
  if(reason < 8 && !logged[reason]) {
    logged[reason] = true;
    fprintf(stderr, "[rsp-hle-gfx] %s: op %02x cmd0=%08x cmd1=%08x -> LLE fallback\n", what, opcode, cmd0, cmd1);
  }
}

//dry-run walker shared by GBI1 and GBI0 (Fast3D): flow control + segment
//table only; any unsupported opcode fails the whole task over to LLE
auto lumiverseGfxDryRun(u32 pc, LumiverseGfxCensus& census, u32 dialect = LumiverseDialectGBI1, bool bakedRDP = false) -> bool {
  u32 segments[16] = {};
  u32 stack[32];
  u32 stackDepth = 0;
  u32 rdpHalf1 = 0;
  bool newOpcode = false;
  bool supported = true;
  //GoldenEye baked RDP stream validation: every 0xb4/0xb2.../0xb3 run must
  //form exactly one RDP command of the length its first word implies, with
  //no other GBI command interleaved; anything else fails the task to LLE
  u32 bakedFirst = 0;
  u32 bakedCount = 0;

  auto segmentAddress = [&](u32 address) -> u32 {
    return (segments[(address >> 24) & 0xf] + (address & 0x00ffffff)) & 0x00ffffff;
  };

  //LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP=<path>: append every walked DL
  //command (empirical analysis of undecoded dialects, e.g. GoldenEye 2.0G)
  static FILE* dlDump = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  if(dlDump) fprintf(dlDump, "task dl=%06x dialect=%u\n", pc, dialect);

  u32 steps = 0;
  while(pc && steps++ < 1000000) {
    const u32 cmd0 = lumiverseGfxWord(pc + 0);
    const u32 cmd1 = lumiverseGfxWord(pc + 4);
    pc += 8;
    const u8 opcode = cmd0 >> 24;
    if(dlDump && steps < 20000) fprintf(dlDump, "  %06x: %08x %08x\n", pc - 8, cmd0, cmd1);

    census.counts[opcode]++;
    if(!census.seen[opcode]) { census.seen[opcode] = true; newOpcode = true; }

    if(!lumiverseGfxOpcodeSupported(opcode)) {
      static bool logged[256];
      if(!logged[opcode]) {
        logged[opcode] = true;
        fprintf(stderr, "[rsp-hle-gfx] unsupported op %02x cmd0=%08x cmd1=%08x -> LLE fallback\n", opcode, cmd0, cmd1);
      }
      supported = false;
      continue;  //keep walking to complete the census
    }

    if(bakedRDP) {
      if(opcode == 0xb4 || opcode == 0xb2 || opcode == 0xb3) {
        if(bakedCount == 0) bakedFirst = cmd1;
        bakedCount++;
        if(opcode == 0xb3) {
          const u32 expected = lumiverseGfxBakedExpectedWords(bakedFirst);
          if(!expected || expected != bakedCount || bakedCount > 64) {
            lumiverseGfxLogRejectOnce(4, opcode, bakedFirst, cmd1, "GE baked-RDP stream shape");
            supported = false;
          }
          bakedCount = 0;
        }
      } else if(bakedCount) {
        lumiverseGfxLogRejectOnce(5, opcode, cmd0, cmd1, "GE baked-RDP stream interrupted");
        supported = false;
        bakedCount = 0;
      }
    }

    switch(opcode) {
    case 0x06:
      if(((cmd0 >> 16) & 0xff) == 0) {
        if(stackDepth >= 32) { lumiverseGfxLogRejectOnce(0, opcode, cmd0, cmd1, "DL stack overflow"); return false; }
        stack[stackDepth++] = pc;
      }
      pc = segmentAddress(cmd1);
      break;
    case 0xb8:
      if(stackDepth == 0) pc = 0;
      else pc = stack[--stackDepth];
      break;
    case 0xb0:
      //GBI1: G_BRANCH_Z (branch always, n64js); GBI0: unknown opcode
      if(dialect == LumiverseDialectGBI0) { lumiverseGfxLogRejectOnce(1, opcode, cmd0, cmd1, "GBI0 op b0"); supported = false; }
      else pc = segmentAddress(rdpHalf1);
      break;
    case 0xb4:
      rdpHalf1 = cmd1;
      break;
    case 0xbc:
      if((cmd0 & 0xff) == 0x06) segments[((cmd0 >> 8) & 0xffff) >> 2 & 0xf] = cmd1 & 0x00ffffff;
      else if((cmd0 & 0xff) == 0x00 || (cmd0 & 0xff) == 0x0c) {  //MW_MATRIX / MW_POINTS
        lumiverseGfxLogRejectOnce(2, opcode, cmd0, cmd1, "moveword MATRIX/POINTS");
        supported = false;
      }
      break;
    case 0xb2: {
      //GBI1: G_MODIFYVTX (only RGBA/ST cases handled); GBI0: RDPHalf_Cont,
      //ignored like n64js (no state effect)
      if(dialect == LumiverseDialectGBI0) break;
      const u32 where = (cmd0 >> 16) & 0xff;
      if(where != 0x10 && where != 0x14) {  //XY/ZSCREEN unhandled
        lumiverseGfxLogRejectOnce(3, opcode, cmd0, cmd1, "modifyvtx XY/ZSCREEN");
        supported = false;
      }
      break;
    }
    case 0xe4: case 0xe5:
      pc += 16;  //consumes the two following commands
      break;
    default:
      break;
    }
  }

  if(steps >= 1000000) {
    fprintf(stderr, "[rsp-hle-gfx] dry-run exceeded step limit -> LLE fallback\n");
    return false;
  }
  if(bakedRDP && bakedCount) {
    lumiverseGfxLogRejectOnce(5, 0xb8, 0, 0, "GE baked-RDP stream unterminated");
    supported = false;
  }

  if(newOpcode && lumiverseGfxLogLevel() >= 1) {
    fprintf(stderr, "[rsp-hle-gfx] opcode census:");
    for(u32 opcode = 0; opcode < 256; opcode++) {
      if(census.seen[opcode]) fprintf(stderr, " %02x=%llu", opcode, (unsigned long long)census.counts[opcode]);
    }
    fprintf(stderr, "\n");
  }

  return supported;
}

auto lumiverseGfxOpcodeSupportedGBI2(u8 opcode) -> bool {
  switch(opcode) {
  case 0x00: case 0x01: case 0x02: case 0x03: case 0x04:
  case 0x05: case 0x06: case 0x07:
  case 0xd5: case 0xd6: case 0xd7: case 0xd8: case 0xd9: case 0xda:
  case 0xdb: case 0xdc: case 0xde: case 0xdf:
  case 0xe0: case 0xe1: case 0xe2: case 0xe3:
  case 0xe4: case 0xe5: case 0xe6: case 0xe7: case 0xe8:
  case 0xe9: case 0xea: case 0xeb: case 0xec: case 0xed:
  case 0xee: case 0xef: case 0xf0: case 0xf1: case 0xf2:
  case 0xf3: case 0xf4: case 0xf5: case 0xf6: case 0xf7:
  case 0xf8: case 0xf9: case 0xfa: case 0xfb: case 0xfc:
  case 0xfd: case 0xfe: case 0xff:
    return true;
  default:
    return false;
  }
}

//GBI2 dry-run: BranchZ execution is conditional, so both the fall-through
//path and the branch target must validate; targets are queued and scanned.
auto lumiverseGfxDryRunGBI2(u32 rootPC, LumiverseGfxCensus& census) -> bool {
  u32 segments[16] = {};
  u32 stack[32];
  u32 stackDepth = 0;
  u32 rdpHalf1 = 0;
  u32 pending[64];
  u32 pendingCount = 0;
  bool newOpcode = false;
  bool supported = true;

  auto segmentAddress = [&](u32 address) -> u32 {
    return (segments[(address >> 24) & 0xf] + (address & 0x00ffffff)) & 0x00ffffff;
  };

  u32 steps = 0;
  u32 pc = rootPC;
  while(pc && steps++ < 1000000) {
    const u32 cmd0 = lumiverseGfxWord(pc + 0);
    const u32 cmd1 = lumiverseGfxWord(pc + 4);
    pc += 8;
    const u8 opcode = cmd0 >> 24;

    census.counts[opcode]++;
    if(!census.seen[opcode]) { census.seen[opcode] = true; newOpcode = true; }

    if(!lumiverseGfxOpcodeSupportedGBI2(opcode)) {
      static bool logged[256];
      if(!logged[opcode]) {
        logged[opcode] = true;
        fprintf(stderr,
          "[rsp-hle-gfx] unsupported gbi2 op %02x cmd0=%08x cmd1=%08x at pc=%06x (root=%06x depth=%u pending=%u) -> LLE fallback\n",
          opcode, cmd0, cmd1, pc - 8, rootPC, stackDepth, pendingCount);
        //empirical aid: the surrounding DL commands (once per opcode)
        for(s32 offset = -12; offset <= 12; offset++) {
          const u32 at = pc - 8 + offset * 8;
          fprintf(stderr, "[rsp-hle-gfx]   %s %06x: %08x %08x\n",
            offset == 0 ? ">" : " ", at, lumiverseGfxWord(at), lumiverseGfxWord(at + 4));
        }
      }
      supported = false;
      break;  //stop walking: a bad branch would only pollute the census
    } else switch(opcode) {
    case 0xde:
      if(((cmd0 >> 16) & 0xff) == 0) {
        if(stackDepth >= 32) return false;
        stack[stackDepth++] = pc;
      }
      pc = segmentAddress(cmd1);
      break;
    case 0xdf:
      if(stackDepth == 0) pc = 0;
      else pc = stack[--stackDepth];
      break;
    case 0x04:  //G_BRANCH_Z: scan the branch target too, keep walking inline
      if(rdpHalf1 && pendingCount < 64) pending[pendingCount++] = segmentAddress(rdpHalf1);
      break;
    case 0xe1:
      rdpHalf1 = cmd1;
      break;
    case 0xdb: {
      const u32 type = (cmd0 >> 16) & 0xff;
      if(type == 0x06) segments[(cmd0 >> 2) & 0xf] = cmd1 & 0x00ffffff;
      //type 0x00 (G_MW_MATRIX) is implemented for GBI2 (Smash) — see executor
      break;
    }
    case 0x02: {
      const u32 where = (cmd0 >> 16) & 0xff;
      if(where != 0x10 && where != 0x14) supported = false;  //XY/ZSCREEN
      break;
    }
    case 0xdc: {
      const u32 type = cmd0 & 0xfe;
      if(type != 8 && type != 10 && type != 14) supported = false;
      break;
    }
    case 0xe4: case 0xe5:
      pc += 16;
      break;
    default:
      break;
    }

    if(!pc && pendingCount) {
      pc = pending[--pendingCount];
      stackDepth = 0;
    }
  }

  if(steps >= 1000000) {
    fprintf(stderr, "[rsp-hle-gfx] gbi2 dry-run exceeded step limit -> LLE fallback\n");
    return false;
  }

  if(newOpcode && lumiverseGfxLogLevel() >= 1) {
    fprintf(stderr, "[rsp-hle-gfx] gbi2 opcode census:");
    for(u32 opcode = 0; opcode < 256; opcode++) {
      if(census.seen[opcode]) fprintf(stderr, " %02x=%llu", opcode, (unsigned long long)census.counts[opcode]);
    }
    fprintf(stderr, "\n");
  }

  return supported;
}

#endif  //defined(VULKAN)

//----------------------------------------------------------------------------
//entry point (declared in lumiverse-hle.cpp)
//----------------------------------------------------------------------------

auto lumiverseExecuteGraphicsTask(const u32 task[16], u64 ucodeHash) -> bool {
#if defined(VULKAN)
  u32 dialect;
  u32 gbi0Vertex = LumiverseGBI0VertexStandard;
  bool geBaked = false;
  switch(ucodeHash) {
  case LumiverseUcodeF3DEXNoN122:   dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DEX121:      dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DEXNoN100:   dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DEX095:      dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DZEXNoN208J: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DZEXNoN206H: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DZEXNoN208I: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2NoN208:  dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2204H:    dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2206:     dialect = LumiverseDialectGBI2; break;
  //LumiverseUcodeF3DEX2207 (DK64) deliberately absent — see its declaration
  case LumiverseUcodeF3DEX2NoN208H: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2208K:    dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2208:     dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeFast3DSM64:    dialect = LumiverseDialectGBI0; break;
  case LumiverseUcodeFast3DPW64:    dialect = LumiverseDialectGBI0; break;
  case LumiverseUcodeFast3DCUSA:    dialect = LumiverseDialectGBI0; break;
  case LumiverseUcodeFast3DWR64:    dialect = LumiverseDialectGBI0; gbi0Vertex = LumiverseGBI0VertexWaveRace; break;
  case LumiverseUcodeFast3DSOTE:    dialect = LumiverseDialectGBI0; gbi0Vertex = LumiverseGBI0VertexSOTE; break;
  //GoldenEye (U) "RSP SW Version: 2.0G": standard Fast3D command set in the
  //menus; in-game every task also carries baked RDP triangle streams
  //(0xb4/0xb2/0xb3 word runs), which are passed through verbatim after
  //structural validation in the dry-run (see LumiverseGfxMachine::bakedRDP)
  case 0xc8f38644ac25bbabull: {
    //LUMIVERSE 2026-09-01: device report "007 is worse now" right after the
    //baked-stream pass-through shipped (host: 0 fallbacks, ~70% paced util).
    //Until a device log explains it, GoldenEye stays on the LLE path unless
    //explicitly opted in.
    const char* optIn = ::getenv("LUMIVERSE_ARES_N64_GE_HLE");
    if(!optIn || optIn[0] != '1') return false;
    dialect = LumiverseDialectGBI0; geBaked = true; break;
  }
  default: return false;
  }
  if(!vulkan.enable) return false;

  static LumiverseGfxMachine machine;
  static LumiverseGfxCensus census;
  static u64 tasksExecuted = 0;
  static u64 tasksFallback = 0;

  const u32 dataPtr = task[12] & 0x00ffffff;
  if(!dataPtr) return false;

  const bool walkable = dialect == LumiverseDialectGBI2
    ? lumiverseGfxDryRunGBI2(dataPtr, census)
    : lumiverseGfxDryRun(dataPtr, census, dialect, geBaked);
  if(!walkable) {
    tasksFallback++;
    return false;
  }

  //fifo microcodes (every ucode seen so far, including SF64's 1.22, carries
  //a fifo): reproduce the authentic output path — write the RDP stream into
  //the game's fifo (task output_buff .. output_buff_size, both addresses)
  //and kick DPC, so games that poll DPC progress instead of waiting on
  //SyncFull interrupts (Rare engines) stay happy.
  const u32 fifoStart = task[10] & 0x00ffffff;
  const u32 fifoEnd = task[11] & 0x00ffffff;
  machine.out.fifo = fifoEnd > fifoStart + 8;
  machine.out.fifoStart = fifoStart;
  machine.out.fifoEnd = fifoEnd;
  if(machine.out.fifo) {
    rdp.writeWord(0x0c, 0x0001, rsp);  //DPC_STATUS: clear xbus -> RDRAM source
  }

  const bool ok = dialect == LumiverseDialectGBI2
    ? lumiverseGfxExecuteTaskGBI2(machine, dataPtr)
    : lumiverseGfxExecuteTask(machine, dataPtr, dialect, gbi0Vertex, geBaked);
  if(ok) tasksExecuted++;
  else tasksFallback++;

  if(lumiverseGfxLogLevel() >= 1 && ((tasksExecuted + tasksFallback) & 63) == 1) {
    fprintf(stderr,
      "[rsp-hle-gfx] tasks=%llu fallback=%llu tris=%llu clipped=%llu rejected=%llu rejects(stack/b0/mw/mvtx/baked/bakedcut)=%llu/%llu/%llu/%llu/%llu/%llu\n",
      (unsigned long long)tasksExecuted, (unsigned long long)tasksFallback,
      (unsigned long long)machine.trisEmitted, (unsigned long long)machine.trisClipped,
      (unsigned long long)machine.trisRejected,
      (unsigned long long)lumiverseGfxRejectCounts[0], (unsigned long long)lumiverseGfxRejectCounts[1],
      (unsigned long long)lumiverseGfxRejectCounts[2], (unsigned long long)lumiverseGfxRejectCounts[3],
      (unsigned long long)lumiverseGfxRejectCounts[4], (unsigned long long)lumiverseGfxRejectCounts[5]);
  }

  return ok;
#else
  (void)task;
  (void)ucodeHash;
  return false;
#endif
}

}  //namespace
