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
//Donkey Kong 64 (U): "RSP Gfx ucode F3DEX fifo 2.07" — whitelisted again
//in round 9 (see the dispatch switch for the two fifo bugs that used to
//crash/hang it at the DK Rap)
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
auto lumiverseGfxWriteWord(u32 address, u32 value) -> void {
  rdram.ram.Memory::Writable::write<Word>(address & 0x00ffffff, value);
}
//round 20 oracle: LUMIVERSE_ARES_N64_GFX_LIGHT_PATCH="<col hex6>:<dx>,<dy>,<dz>:<param>:<px>,<py>,<pz>:<amb hex6>" —
//the shadow executor (which runs BEFORE the LLE RSP on the same task)
//rewrites every 48-byte light entry Conker's DL loads (movemem index 0x0a,
//offsets >= 0x60): the first entry becomes the probe light, the others go
//black, the ambient entry (the last one, dir = 0) takes <amb>. The LLE
//microcode then DMAs the patched entries, so its per-vertex colours are
//the response to ONE known light — the decoder for the packed normals.
struct LumiverseGfxLightPatch { bool on = false; u32 col = 0; s32 dir[3] = {}; u32 param = 0; s32 pos[3] = {}; u32 amb = 0; };
auto lumiverseGfxLightPatch() -> const LumiverseGfxLightPatch& {
  static const LumiverseGfxLightPatch p = [] {
    LumiverseGfxLightPatch r; const char* v = ::getenv("LUMIVERSE_ARES_N64_GFX_LIGHT_PATCH"); if(!v || !v[0]) return r;
    unsigned col, amb; int dx, dy, dz, param, px, py, pz;
    if(::sscanf(v, "%x:%d,%d,%d:%d:%d,%d,%d:%x", &col, &dx, &dy, &dz, &param, &px, &py, &pz, &amb) == 9) {
      r.on = true; r.col = col; r.dir[0] = dx; r.dir[1] = dy; r.dir[2] = dz; r.param = param; r.pos[0] = px; r.pos[1] = py; r.pos[2] = pz; r.amb = amb;
    }
    return r;
  }();
  return p;
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

enum : u32 { LumiverseDialectGBI1 = 0, LumiverseDialectGBI2 = 1, LumiverseDialectGBI0 = 2,
  //S2DEX (2D sprite/background microcode): S2DEX 1.xx shares GBI1's flow
  //control (Yoshi's Story), S2DEX2 shares GBI2's (loaded via G_LOAD_UCODE
  //by the Zeldas and Kirby 64 for their 2D screens)
  LumiverseDialectS2DEX = 3, LumiverseDialectS2DEX2 = 4 };
//Yoshi's Story (U): "RSP Gfx ucode S2DEX  1.06 Yoshitaka Yasumoto Nintendo." — every
//graphics task of the game (round-9 census: 2557 of 2557)
constexpr u64 LumiverseUcodeS2DEX106 = 0x0de2cc6b5e6b2759ull;

//GBI0 G_VTX encoding variants (n64js gbi0.js subclasses). PD (round 12):
//Perfect Dark's bannerless Rare microcode — the standard G_VTX word, but a
//12-byte vertex (x y z s16, pad, colour index byte, s t 11.5) whose colour
//(or packed normal + alpha under lighting) is fetched from a colour table
//set by the custom op 0x07 (n64js gbi0.js GBI0PD: SetVertexColorIndex)
enum : u32 { LumiverseGBI0VertexStandard = 0, LumiverseGBI0VertexWaveRace = 1, LumiverseGBI0VertexSOTE = 2, LumiverseGBI0VertexPD = 3 };

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
  //last SETSCISSOR that went through (10.2, xl/yl exclusive): RDP state that
  //persists across tasks; S2DEX clips its backgrounds against it (round 11,
  //Kirby 64's BG_COPY HUD: frame x 7..327 drawn as 10..309.75 with S = 3
  //under a (10,10)-(310,230) scissor)
  u32 scissorXH = 0, scissorYH = 0, scissorXL = 0xfff, scissorYL = 0xfff;
  //fifo mode: instead of queueHLECommands, write the stream into the game's
  //fifo buffer in RDRAM and kick the DPC registers exactly like the fifo
  //microcode does. Some games (Banjo-Tooie) poll DPC progress rather than
  //waiting for a SyncFull interrupt, so the authentic path is required.
  bool fifo = false;
  u32 fifoStart = 0;
  u32 fifoEnd = 0;
  //round 9: the fifo is written like the microcode writes it — sequentially
  //from fifoStart, DPC_END extended per flush, wrapping only once the RDP
  //has consumed the buffer. The old per-flush "restart at fifoStart" lost
  //every flush but the last while the game held DPC FREEZE (Rare engines:
  //Donkey Kong 64 / Banjo-Tooie / THPS2 set FREEZE around each task), which
  //dropped whole command runs (missing SetColorImage/texture loads) and
  //crashed DK64 at the DK Rap
  u32 fifoPos = 0;
  bool fifoStarted = false;
  //qwords still to come for the RDP command being emitted (0 = at a
  //command boundary); flushes and fifo chunks are cut on boundaries only
  u32 pendingQwords = 0;
};

u64 lumiverseGfxFifoWrapsFrozen = 0;  //diagnostic: wraps forced under FREEZE

extern const u32 lumiverseGfxRDPCommandQwords[64];

//largest command-aligned prefix of out.words[offset .. offset+limit)
auto lumiverseGfxAlignedChunk(const LumiverseGfxOut& out, u32 offset, u32 limit) -> u32 {
  u32 pos = 0, boundary = 0;
  while(pos < limit) {
    const u32 code = out.words[offset + pos] >> 24 & 0x3f;
    const u32 length = lumiverseGfxRDPCommandQwords[code] * 2;
    if(pos + length > limit) break;
    pos += length;
    boundary = pos;
  }
  return boundary;
}

//round 19: LUMIVERSE_ARES_N64_RSP_HLE_GFX_SHADOW=1 — at census level (LLE
//executes every task) the executor also runs each whitelisted task with its
//RDP output DISCARDED, so the projected-vertex dump describes exactly the
//frame the LLE microcode drew (Conker's timeline is host-speed dependent, so
//an HLE run's frame N is not the LLE run's frame N)
static bool lumiverseGfxShadowDiscard = false;
//LUMIVERSE_ARES_N64_RSP_HLE_GFX_SHADOW_DUMP=<path>: the shadow executor's RDP
//stream as text words, tagged like the LLE RDP_STREAM_DUMP of the same run
auto lumiverseGfxShadowDumpFile() -> FILE* {
  static FILE* file = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_SHADOW_DUMP");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  return file;
}
auto lumiverseGfxFlush(LumiverseGfxOut& out) -> void {
  if(lumiverseGfxShadowDiscard) {
    if(FILE* dump = lumiverseGfxShadowDumpFile()) {
      fprintf(dump, "kick task=%llu src=shadow cur=000000 end=%06x\n", (unsigned long long)lumiverseRdpTaskTag, out.count * 4);
      for(u32 index = 0; index + 1 < out.count; index += 2) fprintf(dump, "  %08x %08x\n", out.words[index], out.words[index + 1]);
    }
    out.count = 0; return;
  }
  if(out.count == 0) return;

  if(out.fifo) {
    const u32 capacityWords = ((out.fifoEnd - out.fifoStart) / 4) & ~1u;
    if(capacityWords >= 2) {
      u32 offset = 0;
      while(offset < out.count) {
        u32 space = ((out.fifoEnd - out.fifoPos) / 4) & ~1u;
        const u32 remaining = out.count - offset;
        if(space < 2 || (remaining > space && out.fifoPos != out.fifoStart)) {
          //wrap to the start of the fifo. paraLLEl consumed everything up to
          //DPC_END synchronously at the last kick, unless the game holds
          //DPC FREEZE (the RDP then never consumes and the buffer cannot be
          //reused): release the freeze for the pending range and restore it,
          //exactly the state the game will see after its own unfreeze
          if(rdp.command.freeze) {
            lumiverseGfxFifoWrapsFrozen++;
            rdp.writeWord(0x0c, 0x0004, rsp);
            rdp.writeWord(0x0c, 0x0008, rsp);
          }
          out.fifoPos = out.fifoStart;
          out.fifoStarted = false;
          space = capacityWords;
        }
        u32 chunk = remaining;
        if(chunk > space) {
          //cut on an RDP command boundary: a command split across the fifo
          //end leaves a partial command in paraLLEl's queue, and the fork's
          //render() used to drop the next kick outright when that backlog
          //plus a full-fifo range exceeded its queue (DK64 hang, round 9)
          chunk = lumiverseGfxAlignedChunk(out, offset, space);
          if(chunk == 0) {  //fifo too small for one command: wrap and retry
            if(out.fifoPos == out.fifoStart) { out.failed = true; out.count = 0; return; }
            if(rdp.command.freeze) {
              lumiverseGfxFifoWrapsFrozen++;
              rdp.writeWord(0x0c, 0x0004, rsp);
              rdp.writeWord(0x0c, 0x0008, rsp);
            }
            out.fifoPos = out.fifoStart;
            out.fifoStarted = false;
            continue;
          }
        }
        for(u32 index = 0; index < chunk; index++) {
          rdram.ram.Memory::Writable::write<Word>(out.fifoPos + index * 4, out.words[offset + index]);
        }
        //DPC_START once per fifo pass, then DPC_END per flush: the RDP runs
        //current..end at every END write (or at the game's unfreeze)
        if(!out.fifoStarted) {
          rdp.writeWord(0x00, out.fifoPos, rsp);
          out.fifoStarted = true;
        }
        out.fifoPos += chunk * 4;
        rdp.writeWord(0x04, out.fifoPos, rsp);
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
  if((hi >> 24 & 0x3f) == 0x2d) {  //SETSCISSOR: track for S2DEX background clipping
    out.scissorXH = hi >> 12 & 0xfff; out.scissorYH = hi & 0xfff;
    out.scissorXL = lo >> 12 & 0xfff; out.scissorYL = lo & 0xfff;
  }
  if(out.pendingQwords == 0) out.pendingQwords = lumiverseGfxRDPCommandQwords[hi >> 24 & 0x3f];
  lumiverseGfxEmit(out, hi);
  lumiverseGfxEmit(out, lo);
  out.pendingQwords--;
  if(out.count >= LumiverseGfxOut::FlushAt && out.pendingQwords == 0) lumiverseGfxFlush(out);
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
  //guard-band clip ratio (gSPClipRatio / G_MW_CLIP): the microcode clips
  //x/y against +-ratio*w, not the viewport, and lets the RDP scissor the
  //rest, so an edge-crossing triangle keeps its original vertices and
  //attribute starts (round 11). Games send FRUSTRATIO_2 (Yoshi's Story
  //every task); 2 is the microcode's own DATA default (SF64/OoT never send
  //it and their edge triangles match LLE at 2)
  f32 clipRatio = 2.0f;
  LumiverseGfxLight lights[8];
  u32 numLights = 0;
  f32 fogMul = 0, fogOff = 0;
  //round 20: Conker's lighting (F3DEXBG): 48-byte light entries with a
  //colour, a direction, an attenuation parameter and a view-space
  //position; the vertex's colour bytes are a colour (the normal is packed
  //in the flag halfword); the view matrix is the one multiplied into the
  //projection (G_MTX proj, load=0). Established with the light-patch
  //oracle on the LLE stream, see the round-20 report.
  struct { f32 r, g, b; f32 dx, dy, dz; u32 param; f32 px, py, pz; } conkerLights[16] = {};
  u32 conkerNumLights = 0;
  f32 conkerView[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
  //round 21: the per-slot packed normals of Conker's movemem index 0x0e
  //block (32 halfwords, one per vertex-buffer slot: x in the high byte, y
  //in the low byte, s8 / 127; z is the low byte of the vertex's flag
  //halfword) — see lumiverseGfxLoadVertices
  u16 conkerNormalXY[32] = {};
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

  //S2DEX object state (round 10): uObjMtx 2x2 (16.16) + translation (10.2)
  //+ base scale (5.10), and the gSPObjRenderMode flags. Field layouts from
  //n64js gbi_s2dex.js (MIT); rendering semantics established against our
  //own LLE RDP streams (Yoshi's Story title sprites)
  f32 objA = 1.0f, objB = 0.0f, objC = 0.0f, objD = 1.0f;
  f32 objX = 0.0f, objY = 0.0f;
  f32 objScaleX = 1.0f, objScaleY = 1.0f;
  u32 objRenderMode = 0;
  u32 objTileToggle = 0;

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
  u32 pdColorAddr = 0;  //PD op 0x07: RDRAM colour table for the 12-byte vertices

  //output
  LumiverseGfxOut out;

  //diagnostics
  u64 trisEmitted = 0;
  u64 trisClipped = 0;
  u64 trisRejected = 0;
};

//Banjo-Tooie emits GBI2 op 0x08 (cmd0=08000800 cmd1=00040200) in every
//post-intro display list; n64js lists 0x08 as G_LINE3D (line microcodes
//only). Experiment knob: treat it as a no-op instead of failing the task
//over to LLE, validated against LLE frame checkpoints (round 8).
//round 19: Conker's Bad Fur Day "F3DEXBG.NoN fifo 2.08" — the current task
//runs Rare's custom F3DEX2 build (set by lumiverseGfxResolveDialect). Its
//display lists carry opcodes 0x10-0x1f (every one of the sixteen, plus the
//standard 0x01 VTX / 0x05 TRI1 / 0x06 TRI2 next to them) — our DL dumps of
//the intro: one 8-byte command each, followed by ordinary GBI2 commands,
//not a packed-data run as round 8 assumed. Decoded empirically against the
//LLE RDP stream (see lumiverseGfxConkerTriangles).
static bool lumiverseGfxConker = false;
//LUMIVERSE_ARES_N64_CONKER_TRI_DECODE: candidate bit layout for the 0x1x
//triangle-list command while it is being established (0 = draw nothing)
//LUMIVERSE_ARES_N64_CONKER_MV14: 1 = treat Conker's movemem index 0x0e as
//gSPForceMatrix (rounds 8-19, wrong), 0 = matrix state untouched (default)
auto lumiverseGfxConkerMv14() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_MV14"); return v && v[0] == '1'; }();
  return value;
}
//LUMIVERSE_ARES_N64_CONKER_NEARFAR: which clip flags drop a list triangle
//whole: 1 = near|far (round 19's rule — fitted while movemem 0x0e was still
//trashing the MVP, so most "near" vertices were garbage; with that fixed
//the LLE stream clips near vertices like any F3DEX2.NoN build: the
//ground-plane quads of the logo frames, 33k px each, only appear with
//0 or 2), 2 = far only, 3 = near only, 0 = none (default; 0 and 2 are
//identical over the intro window, the general clipper handles far)
auto lumiverseGfxConkerNearFarMask() -> u8 {
  static const u8 value = [] () -> u8 {
    const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_NEARFAR"); const int mode = v ? ::atoi(v) : 0;
    return mode == 1 ? (LumiverseClipFar | LumiverseClipNear) : mode == 2 ? LumiverseClipFar : mode == 3 ? LumiverseClipNear : 0;
  }();
  return value;
}
//LUMIVERSE_ARES_N64_CONKER_LIGHTING: 1 (default) = the round-20 Conker
//lighting model, 0 = the F3DEX2 path (colour bytes as normals; wrong)
auto lumiverseGfxConkerLighting() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_LIGHTING"); return !v || v[0] != '0'; }();
  return value;
}
//LUMIVERSE_ARES_N64_CONKER_NORMAL_FACTOR: stand-in for max(0, N.dir) on
//packed-normal vertices, percent (default 60 = the mean of
//(LLE - ambient) / (colour x attenuation) over the logo scene's 39
//matched character vertices; median 0.73, quartiles 0.35 / 0.85)
auto lumiverseGfxConkerNormalFactor() -> f32 {
  static const f32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_NORMAL_FACTOR"); return (v ? ::atoi(v) : 60) / 100.0f; }();
  return value;
}
//LUMIVERSE_ARES_N64_CONKER_NORMALS: 1 (default) = per-vertex packed
//normals (round 21: x/y = the movemem 0x0e block's halfword at the
//vertex's buffer slot, z = the flag byte, s8 / 127, rotated by the bone
//matrix and normalised — the light-patch oracle recovers them to a median
//1.5 degrees over 4,228 logo-scene vertices), 0 = the round-20 flat
//stand-in (CONKER_NORMAL_FACTOR)
auto lumiverseGfxConkerNormals() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_NORMALS"); return !v || v[0] != '0'; }();
  return value;
}
//LUMIVERSE_ARES_N64_CONKER_FAR_LIGHT: 1 = a point light 1,448+ units from a
//packed-normal vertex is unattenuated (16-bit wrap of d^2/64 — fitted to
//ONE entry of the dialogue close-up: (7c5e5e), dir (0,0,-126), param 0x14,
//LLE att 0.99-1.02 where 1/d^2 gives 0.017; but (ffac51) at the SAME
//position with param 0x20 attenuates by 1/d^2 in LLE, so the rule is
//not the microcode's — default 0 = 1/d^2 everywhere)
auto lumiverseGfxConkerFarLight() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_FAR_LIGHT"); return v && v[0] == '1'; }();
  return value;
}
auto lumiverseGfxConkerTriDecode() -> int {
  static const int value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_TRI_DECODE"); return v ? ::atoi(v) : 4; }();
  return value;
}
//LUMIVERSE_ARES_N64_RSP_HLE_GFX_VTX_DUMP=<path>: the executor's projected
//screen position of every loaded vertex + every Conker 0x1x command, tagged
//by the RDP task ordinal — the ground truth the LLE RDP stream's triangles
//are matched against to read the command's index layout
auto lumiverseGfxVtxDumpFile() -> FILE* {
  static FILE* file = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_VTX_DUMP");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  return file;
}

auto lumiverseGfxOp08NoOp() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_GBI2_OP08_NOOP"); return v && v[0] == '1'; }();
  return value;
}

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

//per-dialect bit positions / index stride; used by reset and by G_LOAD_UCODE
//mid-task switches (state other than these is deliberately kept)
auto lumiverseGfxSetDialect(LumiverseGfxMachine& m, u32 dialect) -> void {
  m.dialect = dialect;
  if(dialect == LumiverseDialectGBI2 || dialect == LumiverseDialectS2DEX2) {
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
      m.triStride = (m.gbi0Vertex == LumiverseGBI0VertexStandard || m.gbi0Vertex == LumiverseGBI0VertexPD) ? 10 : 5;
    } else {
      m.triStride = 2;
    }
  }
}

//G_LOAD_UCODE target identification (round 10): the text image at the
//command's address is hashed exactly like the census hashes an OSTask's
//ucode (FNV-1a over 4 KiB), so the same constants identify it. Cached per
//text address (validated by the first and middle words of the image).
//Returns the dialect or -1 for anything not whitelisted here. Every new
//(address, hash) pair is logged once so unknown loads can be censused.
auto lumiverseGfxLoadUcodeDialect(u32 textAddress) -> s32 {
  textAddress &= 0x00ffffff;
  struct Entry { u32 address; u32 word0; u32 word800; s32 dialect; u64 hash; };
  static Entry cache[8];
  static u32 cacheCount = 0;
  const u32 word0 = lumiverseGfxWord(textAddress);
  const u32 word800 = lumiverseGfxWord(textAddress + 0x800);
  for(u32 index = 0; index < cacheCount; index++) {
    auto& entry = cache[index];
    if(entry.address == textAddress && entry.word0 == word0 && entry.word800 == word800) return entry.dialect;
  }
  u64 hash = 14695981039346656037ull;
  for(u32 index = 0; index < 0x1000; index++) {
    hash = (hash ^ lumiverseGfxByte(textAddress + index)) * 1099511628211ull;
  }
  s32 dialect = -1;
  switch(hash) {
  case LumiverseUcodeF3DEXNoN122: case LumiverseUcodeF3DEX121:
  case LumiverseUcodeF3DEXNoN100: case LumiverseUcodeF3DEX095:
  case 0x2d3bb1207b5332deull:
    dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeS2DEX106:
    dialect = LumiverseDialectS2DEX; break;
  //S2DEX2 images loaded mid-task by Majora's Mask (text 1abab0) and Kirby 64
  //(text 03be30; Kirby also dispatches it as a standalone OSTask)
  //round 18: Ocarina of Time (U) loads its own S2DEX2 image (text 0e5300)
  //for the pause subscreens and the file select — the 100k-step soak spent
  //most of its time paused and 76% of its gfx tasks fell back on this load
  case 0x2dbd2c59e8565ef7ull: case 0x9e66b240ccd2d588ull: case 0x1176972d8a97a042ull:
    dialect = LumiverseDialectS2DEX2; break;
  case LumiverseUcodeF3DZEXNoN208J: case LumiverseUcodeF3DZEXNoN206H: case LumiverseUcodeF3DZEXNoN208I:
  case LumiverseUcodeF3DEX2NoN208: case LumiverseUcodeF3DEX2204H: case LumiverseUcodeF3DEX2206:
  case LumiverseUcodeF3DEX2207: case 0xfdeacd35544ff754ull: case LumiverseUcodeF3DEX2NoN208H:
  case LumiverseUcodeF3DEX2208K: case LumiverseUcodeF3DEX2208:
    dialect = LumiverseDialectGBI2; break;
  //round 19: Conker's Bad Fur Day reloads its OWN resident image (text
  //0c1060) with G_LOAD_UCODE in every frame's display list; the walker used
  //to stop there, so only the prefix of each DL was ever seen
  case 0x63a15d2f6bdae1f5ull:
    dialect = LumiverseDialectGBI2; break;
  default: break;
  }
  static u32 logged = 0;
  if(logged < 8) {
    logged++;
    fprintf(stderr, "[rsp-hle-gfx] load-ucode text=%06x hash=%016llx -> dialect %d\n",
      textAddress, (unsigned long long)hash, dialect);
  }
  if(cacheCount < 8) cache[cacheCount++] = { textAddress, word0, word800, dialect, hash };
  else cache[(u32)(hash & 7)] = { textAddress, word0, word800, dialect, hash };
  return dialect;
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
  m.gbi0Vertex = gbi0Vertex;
  lumiverseGfxSetDialect(m, dialect);
  m.objA = 1.0f; m.objB = 0.0f; m.objC = 0.0f; m.objD = 1.0f;
  m.objX = 0.0f; m.objY = 0.0f;
  m.objScaleX = 1.0f; m.objScaleY = 1.0f;
  m.objRenderMode = 0;
  m.objTileToggle = 0;
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
  m.clipRatio = 2.0f;
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
const u32 lumiverseGfxRDPCommandQwords[64] = {
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

  const bool pd = m.gbi0Vertex == LumiverseGBI0VertexPD;
  for(u32 index = 0; index < n; index++) {
    const u32 base = address + index * (pd ? 12 : 16);
    auto& vertex = m.verts[v0 + index];
    //colour/normal bytes: inline at +12 (standard), or in the op-07 colour
    //table at the vertex's index byte (+7) for Perfect Dark
    const u32 colorBase = pd ? m.pdColorAddr + lumiverseGfxByte(base + 7) : base + 12;

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

    if(lighting && lumiverseGfxConker && lumiverseGfxConkerLighting()) {
      //Conker (round 20): colour = vertex colour x (ambient + sum over the
      //lights of colour x min(1, param x 2^19 / d^2)), d = distance from
      //the view-space vertex to the light's position (LLE oracle: the
      //logo-scene floor fits 38 + 2^20/d^2 to 1.3 rms over 45 vertices;
      //the light-patch probes give col x param x 2^19 / d^2 exactly, no
      //direction term for vertices without a packed normal). Vertices of
      //batches with geometry-mode bit 0x400000 carry a packed normal in
      //the flag halfword and take col x att x max(0, N.dir); N's packing
      //is NOT decoded yet — a flat 0.5 stands in for max(0, N.dir),
      //which is the class of residual documented in the report.
      const f32 vx = x * mv[0] + y * mv[4] + z * mv[8]  + mv[12];
      const f32 vy = x * mv[1] + y * mv[5] + z * mv[9]  + mv[13];
      const f32 vz = x * mv[2] + y * mv[6] + z * mv[10] + mv[14];
      const f32* cv = m.conkerView;
      const f32 ex = vx * cv[0] + vy * cv[4] + vz * cv[8]  + cv[12];
      const f32 ey = vx * cv[1] + vy * cv[5] + vz * cv[9]  + cv[13];
      const f32 ez = vx * cv[2] + vy * cv[6] + vz * cv[10] + cv[14];
      const auto& amb = m.conkerLights[m.conkerNumLights];
      f32 ir = amb.r, ig = amb.g, ib = amb.b;
      //round 21: vertices of batches with geometry-mode bit 0x400000 carry
      //a packed normal — x and y in the movemem 0x0e block's halfword at
      //the vertex's buffer slot (high / low byte), z in the low byte of the
      //flag halfword, all s8 / 127 — rotated by the bone's modelview and
      //normalised (the recovered length is independent of the bone scale);
      //the light directions are in the same (modelview output) space.
      //Established with the light-patch oracle: median 1.5 degrees from
      //the LLE-recovered normals over 4,228 vertices, see the report.
      const bool packedNormal = (m.geometryMode & 0x400000) != 0;
      f32 nnx = 0, nny = 0, nnz = 0;
      if(packedNormal && lumiverseGfxConkerNormals()) {
        const u16 xy = m.conkerNormalXY[(v0 + index) & 31];
        const f32 px = (s8)(xy >> 8), py = (s8)(xy & 0xff), pz = (s8)lumiverseGfxByte(base + 7);
        nnx = px * mv[0] + py * mv[4] + pz * mv[8];
        nny = px * mv[1] + py * mv[5] + pz * mv[9];
        nnz = px * mv[2] + py * mv[6] + pz * mv[10];
        //the LLE response is |p|/127 x cos x 127/128 x 254/255 (the
        //light-patch oracle: f = 0.981 x N.dir over 15,688 probe samples,
        //the packed vector is not renormalised, |p| = 0.986..1.0): keep the
        //packed length, the s8 direction is scaled by 1/128 below
        const f32 l2 = nnx * nnx + nny * nny + nnz * nnz;
        const f32 lp = ::sqrtf(px * px + py * py + pz * pz) / 127.0f;
        if(l2 > 0.0f) { const f32 inv = lp / ::sqrtf(l2); nnx *= inv; nny *= inv; nnz *= inv; }
      }
      const f32 directional = packedNormal && !lumiverseGfxConkerNormals() ? lumiverseGfxConkerNormalFactor() : 1.0f;
      const bool decodedNormal = packedNormal && lumiverseGfxConkerNormals();
      for(u32 li = 0; li < m.conkerNumLights; li++) {
        const auto& l = m.conkerLights[li];
        //round 21: a param-0 entry is an UNattenuated (directional) light
        //for packed-normal vertices — the logo scene's (105,105,105) entry
        //lights the characters at colour x max(0, N.dir) — and nothing
        //for vertices without a normal (round 20: the param-0 probe on the
        //floor gives 0)
        if(!l.param && !decodedNormal) continue;
        const f32 ddx = ex - l.px, ddy = ey - l.py, ddz = ez - l.pz;
        const f32 d2 = ddx * ddx + ddy * ddy + ddz * ddz;
        //param x 2^19 / d^2 on the 0..255 scale (the floor's 2^20/d^2 was param 2)
        //round 21: the microcode's d^2 is a 16-bit quantity of d^2 / 64 —
        //a light 1,448+ units away wraps it and comes out UNattenuated (the
        //dialogue close-up's (7c5e5e) light at d = 1,510, param 0x14: LLE
        //att_eff 0.99-1.02 over 52 single-lit vertices where 1/d^2 gives
        //0.017; the two lights at d = 540-600 fit 1/d^2 to 2 decimals)
        //...packed-normal vertices only: the no-normal path keeps 1/d^2 to
        //any distance (the same close-up's room vertices: median 1 LSB
        //without the wrap, 20 with it)
        const s16 q = (s16)((u32)(d2 / 64.0f) & 0xffff);
        f32 att = !l.param ? 1.0f
          : decodedNormal && lumiverseGfxConkerFarLight() ? (q <= 0 ? 1.0f : (f32)l.param * 8192.0f / (f32)q / 255.0f)
          : d2 > 1.0f ? (f32)l.param * 524288.0f / d2 / 255.0f : 1.0f;
        if(att > 1.0f) att = 1.0f;
        att *= directional;
        if(decodedNormal) {
          const f32 dot = (nnx * l.dx + nny * l.dy + nnz * l.dz) / 128.0f * (254.0f / 255.0f);
          att *= dot > 0.0f ? dot : 0.0f;
        }
        ir += l.r * att; ig += l.g * att; ib += l.b * att;
      }
      const f32 cr = lumiverseGfxByte(colorBase + 0), cg = lumiverseGfxByte(colorBase + 1), cb = lumiverseGfxByte(colorBase + 2);
      //round 21: the light sum saturates at 255 per channel BEFORE the
      //vertex colour multiply (LLE: a vertex with vcol (192,154,76) under
      //a sum > 255 comes out exactly (192,154,76))
      if(ir > 255.0f) ir = 255.0f; if(ig > 255.0f) ig = 255.0f; if(ib > 255.0f) ib = 255.0f;
      vertex.r = lumiverseClampValue(cr * ir / 255.0f, 0.0f, 255.0f);
      vertex.g = lumiverseClampValue(cg * ig / 255.0f, 0.0f, 255.0f);
      vertex.b = lumiverseClampValue(cb * ib / 255.0f, 0.0f, 255.0f);
      vertex.a = lumiverseGfxByte(colorBase + 3);
    } else if(lighting) {
      const f32 nxRaw = lumiverseGfxSByte(colorBase + 0);
      const f32 nyRaw = lumiverseGfxSByte(colorBase + 1);
      const f32 nzRaw = lumiverseGfxSByte(colorBase + 2);
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
      vertex.a = lumiverseGfxByte(colorBase + 3);
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
      vertex.r = lumiverseGfxByte(colorBase + 0);
      vertex.g = lumiverseGfxByte(colorBase + 1);
      vertex.b = lumiverseGfxByte(colorBase + 2);
      vertex.a = lumiverseGfxByte(colorBase + 3);
    }
  }
  if(FILE* dump = lumiverseGfxVtxDumpFile()) {
    static u32 lines = 0;
    if(lines++ < 400000) {
      fprintf(dump, "vtx task=%llu v0=%u n=%u at=%06x vp=%.2f,%.2f,%.2f,%.2f ratio=%.2f geom=%08x cmd=%08x%08x seg=", (unsigned long long)lumiverseRdpTaskTag, v0, n, address,
        m.vpTransX, m.vpScaleX, m.vpTransY, m.vpScaleY, m.clipRatio, m.geometryMode, m.cmd0, m.cmd1);
      for(u32 s = 0; s < 16; s++) fprintf(dump, "%06x%s", m.segments[s], s == 15 ? "\n" : ",");
      fprintf(dump, "  mvp:"); for(u32 e = 0; e < 16; e++) fprintf(dump, " %.4f", wvp[e]); fprintf(dump, "\n");
      for(u32 index = v0; index < v0 + n; index++) {
        const auto& v = m.verts[index];
        const f32 invW = v.w != 0.0f ? 1.0f / v.w : 0.0f;
        const u32 raw = address + (index - v0) * (pd ? 12 : 16);
        fprintf(dump, "  %2u: sx=%.2f sy=%.2f z/w=%.4f w=%.2f clip=%02x rgba=%.0f,%.0f,%.0f,%.0f xyz=%.3f,%.3f,%.3f uv=%.2f,%.2f raw=%08x%08x%08x%08x\n", index,
          m.vpTransX + m.vpScaleX * (v.x * invW), m.vpTransY - m.vpScaleY * (v.y * invW), v.z * invW, v.w, v.clip, v.r, v.g, v.b, v.a, v.x, v.y, v.z, v.u, v.v,
          lumiverseGfxWord(raw), lumiverseGfxWord(raw + 4), lumiverseGfxWord(raw + 8), lumiverseGfxWord(raw + 12));
      }
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

//LUMIVERSE_ARES_N64_RSP_HLE_GFX_CLIP_RATIO: 0 = clip x/y at the exact
//viewport (rounds 1-10); unset/1 = clip at the microcode's guard-band ratio
//(G_MW_CLIP, default 2) and let the RDP scissor, which keeps the original
//vertices — and therefore the edge and attribute starts — of every triangle
//that merely crosses a screen edge (round 11; validated against LLE RDP
//streams: Yoshi's Story title flag strips start at y=-1.75 unclipped)
auto lumiverseGfxClipRatioEnabled() -> bool {
  static const bool value = [] {
    const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_CLIP_RATIO");
    return !v || v[0] != '0';
  }();
  return value;
}

//G_MW_CLIP word: the ratio as a signed 16-bit magnitude (+r for the RNX/RNY
//words, -r for RPX/RPY — Yoshi's Story: 00000002 / 0000fffe). The
//microcode's own DATA default is 2; anything outside 1..7 is ignored.
auto lumiverseGfxSetClipRatio(LumiverseGfxMachine& m, u32 cmd1) -> void {
  s32 ratio = (s32)(s16)(cmd1 & 0xffff);
  if(ratio < 0) ratio = -ratio;
  if(ratio >= 1 && ratio <= 7) m.clipRatio = (f32)ratio;
}

//Sutherland-Hodgman against one plane; dot(v) >= 0 keeps the vertex.
//planeSelect: 0:w>=eps 1:x<=r*w 2:x>=-r*w 3:y<=r*w 4:y>=-r*w 5:z<=w (far)
//(r = guard-band ratio, 1 = the exact viewport)
auto lumiverseGfxPlaneDot(const LumiverseClipVertex& v, u32 planeSelect, f32 ratio) -> f32 {
  switch(planeSelect) {
  case 0: return v.w - LumiverseWEpsilon;
  case 1: return ratio * v.w - v.x;
  case 2: return ratio * v.w + v.x;
  case 3: return ratio * v.w - v.y;
  case 4: return ratio * v.w + v.y;
  case 5: return v.w - v.z;
  }
  return 0;
}

auto lumiverseGfxClipPolygon(LumiverseClipVertex* verts, u32 count, f32 ratio) -> u32 {
  LumiverseClipVertex scratch[16];
  for(u32 plane = 0; plane < 6; plane++) {
    if(count < 3) return 0;
    u32 outCount = 0;
    for(u32 index = 0; index < count; index++) {
      const auto& current = verts[index];
      const auto& next = verts[(index + 1) % count];
      const f32 dc = lumiverseGfxPlaneDot(current, plane, ratio);
      const f32 dn = lumiverseGfxPlaneDot(next, plane, ratio);
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
  //round 20: provenance of every triangle (standard and Conker lists) in the
  //vertex dump — the RDP-triangle ordinal it starts at links the shadow
  //stream's triangles back to the DL command that produced them
  if(FILE* dump = lumiverseGfxVtxDumpFile()) {
    auto sx = [&](const LumiverseGfxVertex& v) { return v.w != 0.0f ? m.vpTransX + m.vpScaleX * (v.x / v.w) : 0.0f; };
    auto sy = [&](const LumiverseGfxVertex& v) { return v.w != 0.0f ? m.vpTransY - m.vpScaleY * (v.y / v.w) : 0.0f; };
    fprintf(dump, "dt task=%llu pc=%06x %08x %08x | %u %u %u | (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f) clip=%02x%02x%02x w=%.1f,%.1f,%.1f geom=%08x n=%llu\n", (unsigned long long)lumiverseRdpTaskTag,
      m.pc - 8, m.cmd0, m.cmd1, index0, index1, index2, sx(a), sy(a), sx(b), sy(b), sx(c), sy(c), a.clip, b.clip, c.clip, a.w, b.w, c.w, m.geometryMode, (unsigned long long)m.trisEmitted);
  }

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
  //clip only when a vertex leaves the guard band (|x|,|y| > ratio*w), the far
  //plane or the w epsilon; inside the band the RDP scissor does the work on
  //the ORIGINAL triangle, as the microcode does. The exact-viewport clip
  //flags stay in use for the trivial reject above and G_CULLDL.
  const f32 ratio = lumiverseGfxClipRatioEnabled() ? m.clipRatio : 1.0f;
  bool needsClip = a.w < LumiverseWEpsilon || b.w < LumiverseWEpsilon || c.w < LumiverseWEpsilon;
  if(!needsClip) {
    if(ratio <= 1.0f) needsClip = combinedFlags != 0;
    else if(combinedFlags) {
      for(u32 index = 0; index < 3 && !needsClip; index++) {
        const auto& v = *source[index];
        const f32 limit = ratio * v.w;
        needsClip = v.x < -limit || v.x > limit || v.y < -limit || v.y > limit || v.z > v.w;
      }
    }
  }
  if(needsClip) {
    count = lumiverseGfxClipPolygon(poly, 3, ratio);
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

auto lumiverseGfxEmitOtherMode(LumiverseGfxMachine& m) -> void;  //defined with the GBI2 interpreter

//----------------------------------------------------------------------------
//S2DEX object commands (round 10). Shared by the S2DEX 1.xx (GBI1 base,
//Yoshi's Story) and S2DEX2 (GBI2 base) dialects; only opcode numbers differ.
//Structure layouts: n64js gbi_s2dex.js (MIT). The RDP command sequences
//(texture load, tile setup, the two textured triangles of a sprite, the
//shrink/bilerp trimming) were established from our own LLE RDP stream dumps
//of the same display lists.
//----------------------------------------------------------------------------

//gSPObjLoadTxtr: uObjTxtr at `at` (24 bytes). Always loads (the sid/flag/
//mask texture-cache test is not modelled: a redundant load is harmless)
auto lumiverseGfxS2DEXLoadTexture(LumiverseGfxMachine& m, u32 at) -> bool {
  const u32 type = lumiverseGfxWord(at + 0);
  const u32 image = lumiverseGfxSegmentAddress(m, lumiverseGfxWord(at + 4));
  const u32 f8 = lumiverseGfxHalf(at + 8);
  const u32 f10 = lumiverseGfxHalf(at + 10);
  const u32 f12 = lumiverseGfxHalf(at + 12);
  const u32 command = type & 0xff;
  const u32 fmtSiz = (type >> 8) & 0xff;
  const u32 lineMask = (type >> 16) & 0xff;
  switch(command) {
  case 0x33: {  //G_OBJLT_TXTRBLOCK: tmem, tsize (qwords - 1), tline (dxt)
    lumiverseGfxEmit2(m.out, 0xfd000000 | fmtSiz << 16 | (f10 & 0xfff), image);
    lumiverseGfxEmit2(m.out, 0xf5000000 | fmtSiz << 16 | ((((f10 + 1) & lineMask) >> 2) & 0x1ff) << 9 | (f8 & 0x1ff), 0x07000000);
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, 0xf3000000, 0x07000000 | ((f10 << 2) & 0xfff) << 12 | (f12 & 0xfff));
    return true;
  }
  case 0x30: {  //G_OBJLT_TLUT: phead = TMEM word address (0x100 = the TLUT
                //half), pnum = entries - 1 (LLE: SETTIMG width-1 = pnum,
                //SETTILE tmem = phead, LOADTLUT lrs = pnum << 2)
    lumiverseGfxEmit2(m.out, 0xfd100000 | (f10 & 0xfff), image);
    lumiverseGfxEmit2(m.out, 0xe8000000, 0);
    lumiverseGfxEmit2(m.out, 0xf5000000 | (f8 & 0x1ff), 0x07000000);
    lumiverseGfxEmit2(m.out, 0xf0000000, 0x07000000 | ((f10 << 2) & 0xfff) << 12);
    return true;
  }
  case 0x34: {  //G_OBJLT_TXTRTILE: tmem, twidth (16-bit texels - 1), theight (10.2)
    //Yoshi's Story gameplay sprites (24x21 CI8: twidth 11, theight 83 =
    //20.75): SETTIMG width twidth+1, tile line ((twidth+1) & lineMask) >> 2,
    //LOADTILE lrs = twidth << 2, lrt = theight
    lumiverseGfxEmit2(m.out, 0xfd000000 | fmtSiz << 16 | (f10 & 0xfff), image);
    lumiverseGfxEmit2(m.out, 0xf5000000 | fmtSiz << 16 | ((((f10 + 1) & lineMask) >> 2) & 0x1ff) << 9 | (f8 & 0x1ff), 0x07000000);
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, 0xf4000000, 0x07000000 | ((f10 << 2) & 0xfff) << 12 | (f12 & 0xfff));
    return true;
  }
  default:
    return false;
  }
}

//gSPObjSprite / gSPObjLoadTxSprite: uObjSprite at `at` (24 bytes), drawn
//with the current uObjMtx as two textured triangles
auto lumiverseGfxS2DEXDrawSprite(LumiverseGfxMachine& m, u32 at) -> void {
  const f32 objX = (f32)lumiverseGfxShort(at + 0) / 4.0f;
  const f32 scaleW = (f32)lumiverseGfxHalf(at + 2) / 1024.0f;
  const f32 imageW = (f32)lumiverseGfxHalf(at + 4) / 32.0f;
  const f32 objY = (f32)lumiverseGfxShort(at + 8) / 4.0f;
  const f32 scaleH = (f32)lumiverseGfxHalf(at + 10) / 1024.0f;
  const f32 imageH = (f32)lumiverseGfxHalf(at + 12) / 32.0f;
  const u32 stride = lumiverseGfxHalf(at + 16);
  const u32 adrs = lumiverseGfxHalf(at + 18);
  const u32 fmt = lumiverseGfxByte(at + 20) & 7;
  const u32 siz = lumiverseGfxByte(at + 21) & 3;
  const u32 pal = lumiverseGfxByte(at + 22) & 0xf;
  const u32 flags = lumiverseGfxByte(at + 23);
  if(scaleW <= 0.0f || scaleH <= 0.0f || imageW <= 0.0f || imageH <= 0.0f) return;

  //the microcode programs the TLUT type per sprite format (LLE: an I4
  //sprite after a G_TT_RGBA16 game setting is drawn with TEXTLUT none)
  u32 otherModeH = m.otherModeH;
  if(fmt == 2) { if((otherModeH & 0xc000) == 0) otherModeH |= 0x8000; }
  else otherModeH &= ~0xc000u;
  if(otherModeH != m.otherModeH) { m.otherModeH = otherModeH; m.otherModeDirty = true; }
  lumiverseGfxEmitOtherMode(m);

  //render tile: sprite format, line = stride, tmem = adrs, clamp S/T. LLE
  //alternates consecutive sprites between tiles 0 and 2 (RDP tile-descriptor
  //double buffering); reproduced so the streams compare like for like
  const u32 tile = m.objTileToggle;
  m.objTileToggle ^= 2;
  const bool clamp = !(m.objRenderMode & 0x01);  //G_OBJRM_NOTXCLAMP
  lumiverseGfxEmit2(m.out,
    0xf5000000 | fmt << 21 | siz << 19 | (stride & 0x1ff) << 9 | (adrs & 0x1ff),
    tile << 24 | pal << 20 | (clamp ? (2u << 18 | 2u << 8) : 0));
  const u32 lrs = ((u32)(imageW - 1.0f) << 2) & 0xfff;
  const u32 lrt = ((u32)(imageH - 1.0f) << 2) & 0xfff;
  lumiverseGfxEmit2(m.out, 0xf2000000, tile << 24 | lrs << 12 | lrt);
  lumiverseGfxEmit2(m.out, 0xe7000000, 0);

  //object rectangle in object space; SHRINKSIZE_1/2 (0x10/0x20) and BILERP
  //(0x08) each trim one half texel off the FAR end of the object (LLE with
  //mode 0x18: a 128x50 sprite at scale 1 covers x 96..223 with S 0..127;
  //with a flipping matrix D<0 the trimmed end lands at the screen top —
  //Yoshi's "Select Yoshi" sprites, D=-1.8: LLE y 208.0..235.0 = raw
  //206.4+1.8 .. 235.2, floored to quarter pixels)
  const u32 mode = m.objRenderMode;
  const f32 shrink = ((mode & 0x10) ? 0.5f : (mode & 0x20) ? 1.0f : 0.0f) + ((mode & 0x08) ? 0.5f : 0.0f);
  const f32 objW = (imageW - shrink) / scaleW;
  const f32 objH = (imageH - shrink) / scaleH;
  const f32 sLo = 0.0f, sHi = imageW - shrink;
  const f32 tLo = 0.0f, tHi = imageH - shrink;
  auto quarter = [](f32 value) -> f32 { return ::floorf(value * 4.0f) / 4.0f; };

  LumiverseEmitVertex v[4];
  for(auto& e : v) { e.x = e.y = 0.0f; e.r = e.g = e.b = e.a = 1.0f; e.s = e.t = 0.0f; e.invW = 1.0f; e.z = 0.0f; }
  const bool axisAligned = m.objB == 0.0f && m.objC == 0.0f;
  if(axisAligned) {
    f32 x0 = quarter(m.objX + m.objA * objX), x1 = quarter(m.objX + m.objA * (objX + objW));
    f32 y0 = quarter(m.objY + m.objD * objY), y1 = quarter(m.objY + m.objD * (objY + objH));
    f32 s0 = sLo, s1 = sHi, t0 = tLo, t1 = tHi;
    if(x1 < x0) { f32 t = x0; x0 = x1; x1 = t; t = s0; s0 = s1; s1 = t; }
    if(y1 < y0) { f32 t = y0; y0 = y1; y1 = t; t = t0; t0 = t1; t1 = t; }
    if(flags & 0x01) { f32 t = s0; s0 = s1; s1 = t; }  //G_OBJ_FLAG_FLIPS
    if(flags & 0x10) { f32 t = t0; t0 = t1; t1 = t; }  //G_OBJ_FLAG_FLIPT
    if(x1 <= x0 || y1 <= y0) return;
    v[0].x = x0; v[0].y = y0; v[0].s = s0; v[0].t = t0;
    v[1].x = x1; v[1].y = y0; v[1].s = s1; v[1].t = t0;
    v[2].x = x0; v[2].y = y1; v[2].s = s0; v[2].t = t1;
    v[3].x = x1; v[3].y = y1; v[3].s = s1; v[3].t = t1;
  } else {
    //rotated sprite: full 2x2 transform of the four object corners; no
    //shrink modelling (not yet observed on LLE)
    const f32 ox[4] = { objX, objX + objW, objX, objX + objW };
    const f32 oy[4] = { objY, objY, objY + objH, objY + objH };
    f32 su[4] = { sLo, sHi, sLo, sHi };
    f32 tv[4] = { tLo, tLo, tHi, tHi };
    if(flags & 0x01) { su[0] = su[2] = sHi; su[1] = su[3] = sLo; }
    if(flags & 0x10) { tv[0] = tv[1] = tHi; tv[2] = tv[3] = tLo; }
    for(u32 index = 0; index < 4; index++) {
      v[index].x = m.objX + m.objA * ox[index] + m.objB * oy[index];
      v[index].y = m.objY + m.objC * ox[index] + m.objD * oy[index];
      v[index].s = su[index];
      v[index].t = tv[index];
    }
  }
  const u32 savedTile = m.texTile, savedLevel = m.texLevel;
  m.texTile = tile; m.texLevel = 0;
  lumiverseEmitTriangle(m, false, true, false, &v[0], &v[1], &v[2]);
  lumiverseEmitTriangle(m, false, true, false, &v[1], &v[3], &v[2]);
  m.texTile = savedTile; m.texLevel = savedLevel;
}

//gSPObjRectangle / gSPObjRectangleR: uObjSprite drawn as one TEXRECT; the R
//form applies the sub-matrix (translation + 1/BaseScale) and its scale as
//the rectangle's texel step. Derived from LLE (Yoshi's Story HUD, task 706:
//X 253, Y 189.75, BaseScale 2.201/1.636, 304x16 I4 at (-492,-235) ->
//TEXRECT (29.25,45.75)-(167,55) dsdx 0x08ce dtdy 0x068b). BILERP offsets the
//object start by half a texel; the shrink trims the far end as for sprites.
auto lumiverseGfxS2DEXDrawRect(LumiverseGfxMachine& m, u32 at, bool subMatrix) -> void {
  const f32 objX = (f32)lumiverseGfxShort(at + 0) / 4.0f;
  const u32 scaleWRaw = lumiverseGfxHalf(at + 2);
  const f32 scaleW = (f32)scaleWRaw / 1024.0f;
  const f32 imageW = (f32)lumiverseGfxHalf(at + 4) / 32.0f;
  const f32 objY = (f32)lumiverseGfxShort(at + 8) / 4.0f;
  const u32 scaleHRaw = lumiverseGfxHalf(at + 10);
  const f32 scaleH = (f32)scaleHRaw / 1024.0f;
  const f32 imageH = (f32)lumiverseGfxHalf(at + 12) / 32.0f;
  const u32 stride = lumiverseGfxHalf(at + 16);
  const u32 adrs = lumiverseGfxHalf(at + 18);
  const u32 fmt = lumiverseGfxByte(at + 20) & 7;
  const u32 siz = lumiverseGfxByte(at + 21) & 3;
  const u32 pal = lumiverseGfxByte(at + 22) & 0xf;
  const u32 flags = lumiverseGfxByte(at + 23);
  if(scaleW <= 0.0f || scaleH <= 0.0f || imageW <= 0.0f || imageH <= 0.0f) return;

  u32 otherModeH = m.otherModeH;
  if(fmt == 2) { if((otherModeH & 0xc000) == 0) otherModeH |= 0x8000; }
  else otherModeH &= ~0xc000u;
  if(otherModeH != m.otherModeH) { m.otherModeH = otherModeH; m.otherModeDirty = true; }
  lumiverseGfxEmitOtherMode(m);

  const u32 tile = m.objTileToggle;
  m.objTileToggle ^= 2;
  const bool clamp = !(m.objRenderMode & 0x01);
  lumiverseGfxEmit2(m.out,
    0xf5000000 | fmt << 21 | siz << 19 | (stride & 0x1ff) << 9 | (adrs & 0x1ff),
    tile << 24 | pal << 20 | (clamp ? (2u << 18 | 2u << 8) : 0));
  lumiverseGfxEmit2(m.out, 0xf2000000, tile << 24 | (((u32)(imageW - 1.0f) << 2) & 0xfff) << 12 | (((u32)(imageH - 1.0f) << 2) & 0xfff));
  lumiverseGfxEmit2(m.out, 0xe7000000, 0);

  const u32 mode = m.objRenderMode;
  const f32 shrink = ((mode & 0x10) ? 0.5f : (mode & 0x20) ? 1.0f : 0.0f) + ((mode & 0x08) ? 0.5f : 0.0f);
  const f32 bilerp = (mode & 0x08) ? 0.5f : 0.0f;
  //object-space extents (texels / scale): the whole rectangle sits half a
  //texel early under BILERP and the far end is trimmed by the shrink (LLE
  //task 706: far edges 167.0 / 55.0 = nearest quarter of the -0.5 / -1.5
  //texel bounds; -1.0 alone would give 167.25 / 55.25)
  f32 ox0 = objX - bilerp / scaleW, ox1 = objX + (imageW - shrink - bilerp) / scaleW;
  f32 oy0 = objY - bilerp / scaleH, oy1 = objY + (imageH - shrink - bilerp) / scaleH;
  //texel step per pixel (5.10): the sprite scale, times the base scale for R
  f32 dsdx = scaleW, dtdy = scaleH;
  f32 x0, x1, y0, y1;
  if(subMatrix) {
    if(m.objScaleX <= 0.0f || m.objScaleY <= 0.0f) return;
    x0 = m.objX + ox0 / m.objScaleX; x1 = m.objX + ox1 / m.objScaleX;
    y0 = m.objY + oy0 / m.objScaleY; y1 = m.objY + oy1 / m.objScaleY;
    dsdx *= m.objScaleX; dtdy *= m.objScaleY;
  } else {
    x0 = ox0; x1 = ox1; y0 = oy0; y1 = oy1;
  }
  auto quarterNear = [](f32 value) -> f32 { return ::floorf(value * 4.0f + 0.5f) / 4.0f; };
  x0 = quarterNear(x0); x1 = quarterNear(x1); y0 = quarterNear(y0); y1 = quarterNear(y1);
  if(x1 <= x0 || y1 <= y0) return;
  if(x0 < 0.0f) x0 = 0.0f;
  if(y0 < 0.0f) y0 = 0.0f;
  if(x1 > 1023.0f) x1 = 1023.0f;
  if(y1 > 1023.0f) y1 = 1023.0f;
  const u32 xl = (u32)(x1 * 4.0f) & 0xfff, yl = (u32)(y1 * 4.0f) & 0xfff;
  const u32 xh = (u32)(x0 * 4.0f) & 0xfff, yh = (u32)(y0 * 4.0f) & 0xfff;
  u32 s0 = 0, t0 = 0;
  if(flags & 0x01) s0 = ((u32)((imageW - 1.0f) * 32.0f)) & 0xffff;
  if(flags & 0x10) t0 = ((u32)((imageH - 1.0f) * 32.0f)) & 0xffff;
  s32 ds = (s32)(dsdx * 1024.0f), dt = (s32)(dtdy * 1024.0f);
  if(flags & 0x01) ds = -ds;
  if(flags & 0x10) dt = -dt;
  lumiverseGfxEmit2(m.out, 0xe4000000 | xl << 12 | yl, tile << 24 | xh << 12 | yh);
  lumiverseGfxEmit2(m.out, s0 << 16 | t0, ((u32)ds & 0xffff) << 16 | ((u32)dt & 0xffff));
  (void)scaleWRaw; (void)scaleHRaw;
}

//gSPBgRect1Cyc: uObjBg at `at` (40 bytes). Supported: unscaled, unflipped,
//unscrolled images (imageX = imageY = 0, scale 1.0) of any format — the
//Zeldas' saved-frame backgrounds (pause/transition screens, title) and
//Yoshi's static screens; scrolling backgrounds fall back (dry-run). LLE
//draws the image in strips: one LOADTILE of (rows + 1) lines into tile 7
//then a 1-cycle TEXRECT of `rows` lines from tile 0, rows chosen so a strip
//fits TMEM (Majora, 320x240 RGBA16: 48 strips of 5). Loads use the RGBA16
//view of the row (CI8 loads as half-width RGBA16, like LLE).
auto lumiverseGfxS2DEXBackgroundSupported(u32 at) -> bool {
  const u32 imageW = lumiverseGfxHalf(at + 2) >> 2, imageH = lumiverseGfxHalf(at + 10) >> 2;
  const u32 frameW = lumiverseGfxHalf(at + 6) >> 2, frameH = lumiverseGfxHalf(at + 14) >> 2;
  const u32 imageLoad = lumiverseGfxHalf(at + 20);
  const u32 imageSiz = lumiverseGfxByte(at + 23) & 3;
  const u32 imageFlip = lumiverseGfxHalf(at + 26);
  const u32 scaleW = lumiverseGfxHalf(at + 28), scaleH = lumiverseGfxHalf(at + 30);
  if(imageFlip != 0) return false;
  if(scaleW != 0x400 || scaleH != 0x400) return false;
  if(imageLoad != 0xfff4 && imageLoad != 0x0033) return false;
  if(imageSiz != 1 && imageSiz != 2) return false;  //8/16-bit validated (Yoshi CI8, Majora RGBA16)
  if(!imageW || !imageH || !frameW || !frameH) return false;
  //a horizontal wrap (imageX != 0) needs an 8-texel margin (see draw); an
  //unscrolled row only has to fit
  const u32 imageX = lumiverseGfxHalf(at + 0);
  if(imageX ? frameW + 8 > imageW : frameW > imageW) return false;
  if(frameH > imageH) return false;
  return true;
}

auto lumiverseGfxS2DEXDrawBackground(LumiverseGfxMachine& m, u32 at) -> void {
  const f32 imageXf = (f32)lumiverseGfxHalf(at + 0) / 32.0f;
  const u32 imageW = lumiverseGfxHalf(at + 2) >> 2;
  const f32 frameX = (f32)lumiverseGfxShort(at + 4) / 4.0f;
  const u32 frameW = lumiverseGfxHalf(at + 6) >> 2;
  const u32 imageYi = lumiverseGfxHalf(at + 8) >> 5;
  const u32 imageH = lumiverseGfxHalf(at + 10) >> 2;
  const f32 frameY = (f32)lumiverseGfxShort(at + 12) / 4.0f;
  const u32 frameH = lumiverseGfxHalf(at + 14) >> 2;
  const u32 imagePtr = lumiverseGfxSegmentAddress(m, lumiverseGfxWord(at + 16));
  const u32 imageFmt = lumiverseGfxByte(at + 22) & 7;
  const u32 imageSiz = lumiverseGfxByte(at + 23) & 3;
  const u32 imagePal = lumiverseGfxHalf(at + 24) & 0xf;

  u32 otherModeH = m.otherModeH;
  if(imageFmt == 2) { if((otherModeH & 0xc000) == 0) otherModeH |= 0x8000; }
  else otherModeH &= ~0xc000u;
  if(otherModeH != m.otherModeH) { m.otherModeH = otherModeH; m.otherModeDirty = true; }
  lumiverseGfxEmitOtherMode(m);

  //bytes per row of the image and its RGBA16 view used for loading; TMEM
  //holds 4 KiB, half of it for CI images (the TLUT lives in the upper half:
  //LLE loads Yoshi's 336-wide CI8 rows in strips of 5 = 2048/336 - 1,
  //Majora's 320-wide RGBA16 in strips of 5 = 4096/640 - 1)
  const u32 bytesPerTexel = imageSiz == 1 ? 1 : 2;
  const u32 rowBytes = imageW * bytesPerTexel;
  const u32 line = (rowBytes + 7) / 8;              //qwords per row
  if(!line) return;
  const u32 loadWidth = line * 4;                   //16-bit texels per loaded row
  u32 rows = (imageFmt == 2 ? 2048 : 4096) / (line * 8);
  if(rows < 2) return;
  rows -= 1;                                        //one extra line for bilinear filtering
  //horizontal scroll: the load starts at the 8-byte-aligned column below
  //imageX and runs one full row, which in RDRAM continues into the NEXT
  //row's first columns — exactly how LLE serves the wrap (its strips load
  //from column 328 of a 336-wide image scrolled to 335.375 and draw with
  //S = 7.375); the seam samples the neighbouring row, as on hardware
  const u32 scrollBytes = ((u32)(imageXf * (f32)bytesPerTexel)) & ~7u;
  const f32 sStart = imageXf - (f32)scrollBytes / (f32)bytesPerTexel;
  const u32 fmtSizLoad = 0x10;                      //RGBA16
  const u32 fmtSizDraw = imageFmt << 5 | imageSiz << 3;
  lumiverseGfxEmit2(m.out, 0xf5000000 | fmtSizLoad << 16 | (line & 0x1ff) << 9, 0x07000000);
  lumiverseGfxEmit2(m.out, 0xf5000000 | fmtSizDraw << 16 | (line & 0x1ff) << 9, imagePal << 20 | 0x0007c1f0);
  lumiverseGfxEmit2(m.out, 0xf2000000, 0);
  const f32 x0 = frameX, x1 = ::floorf(frameX + (f32)frameW);
  const u32 sWord = ((u32)(sStart * 32.0f)) & 0xffff;
  //vertical scroll with wrap: strips never cross the image's last row
  u32 y = 0;
  while(y < frameH) {
    const u32 srcRow = (imageYi + y) % imageH;
    u32 count = frameH - y < rows ? frameH - y : rows;
    if(srcRow + count > imageH) count = imageH - srcRow;
    lumiverseGfxEmit2(m.out, 0xfd000000 | fmtSizLoad << 16 | ((loadWidth - 1) & 0xfff), imagePtr + srcRow * rowBytes + scrollBytes);
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, 0xf4000000, 0x07000000 | (((loadWidth - 1) << 2) & 0xfff) << 12 | (((count + 1) << 2) - 1));
    lumiverseGfxEmit2(m.out, 0xe7000000, 0);
    const f32 ya = frameY + (f32)y, yb = frameY + (f32)(y + count);
    const u32 xl = (u32)(x1 * 4.0f) & 0xfff, yl = (u32)(yb * 4.0f) & 0xfff;
    const u32 xh = (u32)(x0 * 4.0f) & 0xfff, yh = (u32)(ya * 4.0f) & 0xfff;
    lumiverseGfxEmit2(m.out, 0xe4000000 | xl << 12 | yl, xh << 12 | yh);
    lumiverseGfxEmit2(m.out, sWord << 16, 0x04000400);
    y += count;
  }
}

//4-bit BG_1CYC (I4 / CI4), derived from Kirby 64's LLE streams (round 11:
//I4 208x32 and 208x16 title/pause plates, CI4 64x77 file-select panel,
//CI4 32x20 / 32x32 HUD icons). The microcode treats the row as RGBA16
//texel pairs and loads ONE EXTRA texel per row (LOADTILE lrs = width<<2, not
//(width-1)<<2) with the tile line one qword wider than the row; strips draw
//`count` rows from `count+1` loaded lines; the last strip draws one row
//fewer than remains (the frame's last row and column are never drawn:
//TEXRECT xl = frameX+frameW-1, yl = frameY+frameH-1) and, before its main
//load, loads the wrapped image row 0 as the guard line at TMEM row count+1
//— as one line when that TMEM address (qwords) is even, otherwise as two
//lines from image row -1 one TMEM row earlier (LLE: CI4 77 rows = strips of
//50 + 26 with the guard pair at tmem 0x82; I4 32 rows = one strip of 31
//with the guard line at 0x1c0). Unscrolled, unflipped, unscaled only.
auto lumiverseGfxS2DEXBackground4BitSupported(u32 at) -> bool {
  const u32 imageX = lumiverseGfxHalf(at + 0), imageY = lumiverseGfxHalf(at + 8);
  const u32 imageW = lumiverseGfxHalf(at + 2) >> 2, imageH = lumiverseGfxHalf(at + 10) >> 2;
  const u32 frameW = lumiverseGfxHalf(at + 6) >> 2, frameH = lumiverseGfxHalf(at + 14) >> 2;
  const u32 imageLoad = lumiverseGfxHalf(at + 20);
  const u32 imageFmt = lumiverseGfxByte(at + 22) & 7;
  const u32 imageSiz = lumiverseGfxByte(at + 23) & 3;
  const u32 imageFlip = lumiverseGfxHalf(at + 26);
  const u32 scaleW = lumiverseGfxHalf(at + 28), scaleH = lumiverseGfxHalf(at + 30);
  //4-bit images always; 8/16-bit ones only when the frame is narrower than
  //the image (Kirby's 300-wide window on a 304-wide CI8 panel: LLE uses the
  //same +1-texel / line+1 / wrapped-guard form there, while a frame that
  //spans the whole image width keeps the round-10 form — validated
  //byte-identical on Kirby's 320x240 boot backgrounds)
  if(imageSiz == 0) { if(imageFmt != 2 && imageFmt != 4) return false; }
  else if(imageSiz == 1 || imageSiz == 2) { if(frameW >= imageW) return false; if(imageFmt != 2 && imageFmt != 0 && imageFmt != 4) return false; }
  else return false;
  if(imageX || imageY || imageFlip) return false;
  if(scaleW != 0x400 || scaleH != 0x400) return false;
  if(imageLoad != 0xfff4) return false;
  if(!imageW || !imageH || !frameW || !frameH || (imageSiz == 0 && (imageW & 1))) return false;
  if(frameW > imageW || frameH > imageH || imageH < 2) return false;
  const u32 rowBytes = imageSiz == 0 ? imageW / 2 : imageSiz == 1 ? imageW : imageW * 2;
  const u32 line = (rowBytes + 7) / 8 + 1;
  if(line * 2 + 1 > (imageFmt == 2 ? 256u : 512u)) return false;
  return true;
}

auto lumiverseGfxS2DEXDrawBackground4Bit(LumiverseGfxMachine& m, u32 at) -> void {
  const u32 imageW = lumiverseGfxHalf(at + 2) >> 2;
  const s32 frameX = (s32)lumiverseGfxShort(at + 4) & ~3;
  const u32 frameW = lumiverseGfxHalf(at + 6) >> 2;
  const u32 imageH = lumiverseGfxHalf(at + 10) >> 2;
  const s32 frameY = (s32)lumiverseGfxShort(at + 12) & ~3;
  const u32 frameH = lumiverseGfxHalf(at + 14) >> 2;
  const u32 imagePtr = lumiverseGfxSegmentAddress(m, lumiverseGfxWord(at + 16));
  const u32 imageFmt = lumiverseGfxByte(at + 22) & 7;
  const u32 imageSiz = lumiverseGfxByte(at + 23) & 3;
  const u32 imagePal = lumiverseGfxHalf(at + 24) & 0xf;

  u32 otherModeH = m.otherModeH;
  if(imageFmt == 2) { if((otherModeH & 0xc000) == 0) otherModeH |= 0x8000; }
  else otherModeH &= ~0xc000u;
  if(otherModeH != m.otherModeH) { m.otherModeH = otherModeH; m.otherModeDirty = true; }
  lumiverseGfxEmitOtherMode(m);

  const u32 rowBytes = imageSiz == 0 ? imageW / 2 : imageSiz == 1 ? imageW : imageW * 2;
  const u32 qwords = (rowBytes + 7) / 8;
  const u32 line = qwords + 1;
  const u32 loadWidth = qwords * 4;                 //RGBA16 texels per row
  const u32 rowsPerStrip = (imageFmt == 2 ? 256 : 512) / line - 1;
  const u32 loadTile = 0xf5000000 | 0x10 << 16 | (line & 0x1ff) << 9;
  const u32 drawTile = 0xf5000000 | (imageFmt << 5 | imageSiz << 3) << 16 | (line & 0x1ff) << 9;
  const u32 timg = 0xfd000000 | 0x10 << 16 | ((loadWidth - 1) & 0xfff);
  lumiverseGfxEmit2(m.out, loadTile, 0x07000000);
  lumiverseGfxEmit2(m.out, drawTile, imagePal << 20 | 0x0007c1f0);
  lumiverseGfxEmit2(m.out, 0xf2000000, 0);
  //the image's own last column and row are never drawn (LLE: 208-wide
  //frame of a 208-wide image ends at x+207; a 20-wide frame of a 32-wide
  //image at x+20), so the drawn extent is the frame clipped to imageW-1 /
  //imageH-1
  const u32 drawW = frameW < imageW - 1 ? frameW : imageW - 1;
  const u32 xh = (u32)frameX & 0xfff, xl = (u32)(frameX + (s32)drawW * 4) & 0xfff;
  const u32 lrs = (loadWidth << 2) & 0xfff;
  u32 remaining = frameH < imageH - 1 ? frameH : imageH - 1, row = 0;
  s32 y = frameY;
  while(remaining > 0) {
    const u32 count = remaining < rowsPerStrip ? remaining : rowsPerStrip;
    //the strip loads count+1 lines; when that reaches the image's end the
    //microcode first loads the wrapped row 0 as the line after them
    if(row + count + 1 >= imageH) {
      //wrapped guard line (image row 0) at TMEM row count+1
      const u32 guardRow = count + 1;
      if(((guardRow * line) & 1) == 0) {
        lumiverseGfxEmit2(m.out, loadTile | ((guardRow * line) & 0x1ff), 0x07000000);
        lumiverseGfxEmit2(m.out, 0xe6000000, 0);
        lumiverseGfxEmit2(m.out, timg, imagePtr);
        lumiverseGfxEmit2(m.out, 0xf4000000, 0x07000000 | lrs << 12 | 3);
      } else {
        lumiverseGfxEmit2(m.out, loadTile | (((guardRow - 1) * line) & 0x1ff), 0x07000000);
        lumiverseGfxEmit2(m.out, 0xe6000000, 0);
        lumiverseGfxEmit2(m.out, timg, imagePtr - rowBytes);
        lumiverseGfxEmit2(m.out, 0xf4000000, 0x07000000 | lrs << 12 | 7);
      }
      lumiverseGfxEmit2(m.out, loadTile, 0x07000000);
    }
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, timg, imagePtr + row * rowBytes);
    lumiverseGfxEmit2(m.out, 0xf4000000, 0x07000000 | lrs << 12 | ((count << 2) | 3));
    lumiverseGfxEmit2(m.out, 0xe7000000, 0);
    const u32 yh = (u32)y & 0xfff, yl = (u32)(y + (s32)count * 4) & 0xfff;
    lumiverseGfxEmit2(m.out, 0xe4000000 | xl << 12 | yl, xh << 12 | yh);
    lumiverseGfxEmit2(m.out, 0, 0x04000400);
    row += count; y += (s32)count * 4; remaining -= count;
  }
}

//gSPBgRectCopy (BG_COPY): the copy-mode background. Derived from Kirby 64's
//gameplay HUD (RGBA16 320x48 at (7,182) under a (10,10)-(310,230)
//scissor, LLE RDP stream, round 11): the frame is clipped to the scissor
//(x 10..309.75, S starts at 3), the load/render tiles use the CPU-computed
//uObjBg tail fields (tmemW = tile line in qwords, tmemH = rows per strip,
//tmemLoadTH = LOADTILE lrt), each strip is one LOADTILE of `rows` lines
//and one inclusive copy-mode TEXRECT (dsdx 4.0), the strips leave exactly
//one row for a tail that LOADBLOCKs that row and the wrapped next row
//(tile 6 / tile 7) and draws it as a one-line TEXRECT. Validated only for
//unscrolled, unflipped, 16-bit images.
auto lumiverseGfxS2DEXBackgroundCopySupported(u32 at) -> bool {
  const u32 imageX = lumiverseGfxHalf(at + 0), imageY = lumiverseGfxHalf(at + 8);
  const u32 imageW = lumiverseGfxHalf(at + 2) >> 2, imageH = lumiverseGfxHalf(at + 10) >> 2;
  const u32 frameW = lumiverseGfxHalf(at + 6) >> 2, frameH = lumiverseGfxHalf(at + 14) >> 2;
  const u32 imageLoad = lumiverseGfxHalf(at + 20);
  const u32 imageSiz = lumiverseGfxByte(at + 23) & 3;
  const u32 imageFlip = lumiverseGfxHalf(at + 26);
  const u32 tmemW = lumiverseGfxHalf(at + 28), tmemH = lumiverseGfxHalf(at + 30) >> 2;
  if(imageX || imageY || imageFlip) return false;
  if(imageLoad != 0xfff4) return false;
  if(imageSiz != 2) return false;
  if(!imageW || !imageH || !frameW || !frameH || !tmemW || !tmemH) return false;
  if(frameW > imageW || frameH > imageH) return false;
  if(tmemW > 0x1ff || (imageW * 2) / 8 > tmemW) return false;
  return true;
}

auto lumiverseGfxS2DEXDrawBackgroundCopy(LumiverseGfxMachine& m, u32 at) -> void {
  const u32 imageW = lumiverseGfxHalf(at + 2) >> 2;
  const s32 frameX = (s32)lumiverseGfxShort(at + 4), frameW = (s32)(lumiverseGfxHalf(at + 6) & ~3u);  //10.2
  const u32 imageH = lumiverseGfxHalf(at + 10) >> 2;
  const s32 frameY = (s32)lumiverseGfxShort(at + 12), frameH = (s32)(lumiverseGfxHalf(at + 14) & ~3u);
  const u32 imagePtr = lumiverseGfxSegmentAddress(m, lumiverseGfxWord(at + 16));
  const u32 imageFmt = lumiverseGfxByte(at + 22) & 7;
  const u32 imageSiz = lumiverseGfxByte(at + 23) & 3;
  const u32 imagePal = lumiverseGfxHalf(at + 24) & 0xf;
  const u32 tmemW = lumiverseGfxHalf(at + 28);
  const u32 tmemH = lumiverseGfxHalf(at + 30) >> 2;
  const u32 rowBytes = imageW * 2;

  //clip the frame to the scissor (all 10.2; scissor xl/yl exclusive)
  const s32 x0 = frameX > (s32)m.out.scissorXH ? frameX : (s32)m.out.scissorXH;
  const s32 x1 = frameX + frameW < (s32)m.out.scissorXL ? frameX + frameW : (s32)m.out.scissorXL;
  const s32 y0 = frameY > (s32)m.out.scissorYH ? frameY : (s32)m.out.scissorYH;
  const s32 y1 = frameY + frameH < (s32)m.out.scissorYL ? frameY + frameH : (s32)m.out.scissorYL;
  if(x1 <= x0 || y1 <= y0) return;
  const u32 sStart = (u32)(x0 - frameX) >> 2;         //texels skipped on the left
  u32 row = (u32)(y0 - frameY) >> 2;                  //first image row
  u32 remaining = (u32)(y1 - y0) >> 2;
  const u32 xl = (u32)(x1 - 1) & 0xfff, xh = (u32)x0 & 0xfff;
  const u32 lrs = (xl - xh + (sStart << 2)) & 0xfff;

  lumiverseGfxEmitOtherMode(m);
  const u32 fmtSiz = imageFmt << 5 | imageSiz << 3;
  const u32 tileWord = 0xf5000000 | fmtSiz << 16 | (tmemW & 0x1ff) << 9;
  lumiverseGfxEmit2(m.out, tileWord, 0x07000000);
  lumiverseGfxEmit2(m.out, 0xf2000000, 0x00000000);
  lumiverseGfxEmit2(m.out, tileWord, imagePal << 20 | 0x0007c1f0);
  const u32 timg = 0xfd000000 | fmtSiz << 16 | ((imageW - 1) & 0xfff);
  u32 y = (u32)y0;
  while(remaining > 1) {
    u32 count = remaining - 1 < tmemH ? remaining - 1 : tmemH;
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, timg, imagePtr + row * rowBytes);
    lumiverseGfxEmit2(m.out, 0xf4000000, 0x07000000 | lrs << 12 | (((count - 1) << 2) | 3));
    lumiverseGfxEmit2(m.out, 0xe7000000, 0);
    lumiverseGfxEmit2(m.out, 0xe4000000 | xl << 12 | ((y + count * 4 - 1) & 0xfff), xh << 12 | (y & 0xfff));
    lumiverseGfxEmit2(m.out, (sStart << 5) << 16, 0x10000400);
    row += count; y += count * 4; remaining -= count;
  }
  if(remaining == 1) {
    //tail: this row via LOADBLOCK into tile 6, the wrapped next row after it
    //(tile 6 tmem = one row, LOADBLOCK on tile 7 with lrs 0xfff — the
    //microcode's own words), then one copy-mode line from tile 0
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, timg, imagePtr + row * rowBytes);
    lumiverseGfxEmit2(m.out, 0xf5000000 | fmtSiz << 16, 0x06000000);
    lumiverseGfxEmit2(m.out, 0xf3000000, 0x06000000 | ((imageW - 1) & 0xfff) << 12);
    lumiverseGfxEmit2(m.out, 0xe6000000, 0);
    lumiverseGfxEmit2(m.out, timg, imagePtr + ((row + 1) % imageH) * rowBytes);
    lumiverseGfxEmit2(m.out, 0xf5000000 | fmtSiz << 16 | ((rowBytes / 8) & 0x1ff), 0x06000000);
    lumiverseGfxEmit2(m.out, 0xf3000000, 0x07fff000);
    lumiverseGfxEmit2(m.out, 0xe7000000, 0);
    lumiverseGfxEmit2(m.out, 0xe4000000 | xl << 12 | (y & 0xfff), xh << 12 | (y & 0xfff));
    lumiverseGfxEmit2(m.out, (sStart << 5) << 16, 0x10000400);
  }
}

auto lumiverseGfxS2DEXLoadMatrix(LumiverseGfxMachine& m, u32 at, bool full) -> void {
  if(full) {
    m.objA = (f32)(s32)lumiverseGfxWord(at + 0) / 65536.0f;
    m.objB = (f32)(s32)lumiverseGfxWord(at + 4) / 65536.0f;
    m.objC = (f32)(s32)lumiverseGfxWord(at + 8) / 65536.0f;
    m.objD = (f32)(s32)lumiverseGfxWord(at + 12) / 65536.0f;
    at += 16;
  }
  m.objX = (f32)lumiverseGfxShort(at + 0) / 4.0f;
  m.objY = (f32)lumiverseGfxShort(at + 2) / 4.0f;
  m.objScaleX = (f32)lumiverseGfxHalf(at + 4) / 1024.0f;
  m.objScaleY = (f32)lumiverseGfxHalf(at + 6) / 1024.0f;
}

//executes one S2DEX-specific command; false = not an S2DEX command (the
//base dialect's interpreter handles it)
auto lumiverseGfxExecuteS2DEX(LumiverseGfxMachine& m, u8 opcode, u32 cmd0, u32 cmd1, bool gbi2) -> bool {
  //inline RDP triangle (G_RDP_TRI*: the CPU computed the coefficients, the
  //microcode forwards the whole command)
  if(opcode >= 0xc8 && opcode <= 0xcf) {
    lumiverseGfxEmitOtherMode(m);
    const u32 qwords = lumiverseGfxRDPCommandQwords[opcode & 0x3f];
    lumiverseGfxEmit2(m.out, cmd0, cmd1);
    for(u32 q = 1; q < qwords; q++) {
      if(!lumiverseGfxNextCommand(m)) { m.out.failed = true; return true; }
      lumiverseGfxEmit2(m.out, m.cmd0, m.cmd1);
    }
    m.trisEmitted++;
    return true;
  }
  const u32 at = lumiverseGfxSegmentAddress(m, cmd1);
  if(!gbi2) {
    switch(opcode) {
    case 0x01:  //G_BG_1CYC
      if(lumiverseGfxS2DEXBackground4BitSupported(at)) lumiverseGfxS2DEXDrawBackground4Bit(m, at);
      else lumiverseGfxS2DEXDrawBackground(m, at);
      return true;
    case 0x02:  //G_BG_COPY
      lumiverseGfxS2DEXDrawBackgroundCopy(m, at);
      return true;
    case 0x03:  //G_OBJ_RECTANGLE
      lumiverseGfxS2DEXDrawRect(m, at, false);
      return true;
    case 0xb2:  //G_OBJ_RECTANGLE_R
      lumiverseGfxS2DEXDrawRect(m, at, true);
      return true;
    case 0xc3:  //G_OBJ_LDTX_RECT
      if(!lumiverseGfxS2DEXLoadTexture(m, at)) { m.out.failed = true; return true; }
      lumiverseGfxS2DEXDrawRect(m, at + 24, false);
      return true;
    case 0xc4:  //G_OBJ_LDTX_RECT_R
      if(!lumiverseGfxS2DEXLoadTexture(m, at)) { m.out.failed = true; return true; }
      lumiverseGfxS2DEXDrawRect(m, at + 24, true);
      return true;
    case 0x04:  //G_OBJ_SPRITE
      lumiverseGfxS2DEXDrawSprite(m, at);
      return true;
    case 0x05: {  //G_OBJ_MOVEMEM: 0 = uObjMtx, 2 = uObjSubMtx
      const u32 index = cmd0 & 0xffff;
      if(index == 0) lumiverseGfxS2DEXLoadMatrix(m, at, true);
      else if(index == 2) lumiverseGfxS2DEXLoadMatrix(m, at, false);
      return true;
    }
    case 0xb1:  //G_OBJ_RENDERMODE
      m.objRenderMode = cmd1 & 0xffff;
      return true;
    case 0xc1:  //G_OBJ_LOADTXTR
      if(!lumiverseGfxS2DEXLoadTexture(m, at)) m.out.failed = true;
      return true;
    case 0xc2:  //G_OBJ_LDTX_SPRITE
      if(!lumiverseGfxS2DEXLoadTexture(m, at)) { m.out.failed = true; return true; }
      lumiverseGfxS2DEXDrawSprite(m, at + 24);
      return true;
    default:
      return false;
    }
  }
  switch(opcode) {
  case 0x01:  //G_OBJ_RECTANGLE
    lumiverseGfxS2DEXDrawRect(m, at, false);
    return true;
  case 0xda:  //G_OBJ_RECTANGLE_R
    lumiverseGfxS2DEXDrawRect(m, at, true);
    return true;
  case 0x07:  //G_OBJ_LDTX_RECT
    if(!lumiverseGfxS2DEXLoadTexture(m, at)) { m.out.failed = true; return true; }
    lumiverseGfxS2DEXDrawRect(m, at + 24, false);
    return true;
  case 0x08:  //G_OBJ_LDTX_RECT_R
    if(!lumiverseGfxS2DEXLoadTexture(m, at)) { m.out.failed = true; return true; }
    lumiverseGfxS2DEXDrawRect(m, at + 24, true);
    return true;
  case 0x09:  //G_BG_1CYC
    if(lumiverseGfxS2DEXBackground4BitSupported(at)) lumiverseGfxS2DEXDrawBackground4Bit(m, at);
    else lumiverseGfxS2DEXDrawBackground(m, at);
    return true;
  case 0x0a:  //G_BG_COPY
    lumiverseGfxS2DEXDrawBackgroundCopy(m, at);
    return true;
  case 0x02:  //G_OBJ_SPRITE
    lumiverseGfxS2DEXDrawSprite(m, at);
    return true;
  case 0x05:  //G_OBJ_LOADTXTR
    if(!lumiverseGfxS2DEXLoadTexture(m, at)) m.out.failed = true;
    return true;
  case 0x06:  //G_OBJ_LDTX_SPRITE
    if(!lumiverseGfxS2DEXLoadTexture(m, at)) { m.out.failed = true; return true; }
    lumiverseGfxS2DEXDrawSprite(m, at + 24);
    return true;
  case 0x0b:  //G_OBJ_RENDERMODE
    m.objRenderMode = cmd1 & 0xffff;
    return true;
  case 0xdc: {  //G_MOVEMEM types 0/2 = uObjMtx / uObjSubMtx
    const u32 type = cmd0 & 0xfe;
    if(type == 0) { lumiverseGfxS2DEXLoadMatrix(m, at, true); return true; }
    if(type == 2) { lumiverseGfxS2DEXLoadMatrix(m, at, false); return true; }
    return false;
  }
  default:
    return false;
  }
}

//G_LOAD_UCODE (GBI1 0xaf / GBI2 0xdd): switch the interpreter's dialect;
//the RSP keeps the display list position, segment table and RDP state,
//which is exactly what our machine keeps too. Unknown targets fail the task
//(the dry-run refuses them first).
auto lumiverseGfxLoadUcode(LumiverseGfxMachine& m, u32 cmd1) -> void {
  const s32 dialect = lumiverseGfxLoadUcodeDialect(cmd1);
  if(dialect < 0) { m.out.failed = true; m.running = false; return; }
  //Interpreter state is KEPT across the switch. Empirically the RSP keeps
  //at least the segment table and the display-list position/stack across
  //G_LOAD_UCODE (Majora's Mask comes back from its S2DEX2 background with
  //no segment movewords and continues through segment-relative sub-DLs; a
  //reset-everything attempt walked garbage — "runaway display list").
  //Yoshi's Story re-sends its segments and other-modes after every switch,
  //so either model works there. Other-mode is re-emitted lazily.
  lumiverseGfxSetDialect(m, (u32)dialect);
  m.otherModeDirty = true;
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

    if(m.dialect == LumiverseDialectS2DEX && lumiverseGfxExecuteS2DEX(m, opcode, cmd0, cmd1, false)) {
      if(m.out.failed) break;
      continue;
    }

    switch(opcode) {

    case 0x00: break;  //G_SPNOOP
    case 0xc0: break;  //G_NOOP

    case 0xaf:  //G_LOAD_UCODE (Yoshi's Story: S2DEX <-> F3DEX twice per task)
      lumiverseGfxLoadUcode(m, cmd1);
      break;

    case 0x07:  //Perfect Dark: colour table for the following G_VTX (n64js
                //GBI0PD SetVertexColorIndex); cmd0's fields are ignored
      if(m.dialect == LumiverseDialectGBI0 && m.gbi0Vertex == LumiverseGBI0VertexPD) {
        m.pdColorAddr = lumiverseGfxSegmentAddress(m, cmd1);
      } else {
        lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
      }
      break;

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
      case 0x04:  //G_MW_CLIP: guard-band ratio (four words, offsets 0x04/0x0c
                  //= +r, 0x14/0x1c = -r as s16; Yoshi's Story sends 2/-2)
        lumiverseGfxSetClipRatio(m, cmd1);
        break;
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
      case 0x0c: {  //G_MW_POINTS: patch one field of a loaded vertex (Cruis'n
                    //USA). offset = vertex * 40 + field, the microcode's
                    //internal vertex stride; the RGBA/ST fields carry the
                    //same payload as G_MODIFYVTX (dry-run admits only those)
        const u32 vertexIndex = offset / 40;
        const u32 field = offset % 40;
        if(vertexIndex < 32) {
          auto& vertex = m.verts[vertexIndex];
          if(field == 0x10) {  //G_MWO_POINT_RGBA
            vertex.r = (cmd1 >> 24) & 0xff;
            vertex.g = (cmd1 >> 16) & 0xff;
            vertex.b = (cmd1 >> 8) & 0xff;
            vertex.a = (cmd1 >> 0) & 0xff;
          } else if(field == 0x14) {  //G_MWO_POINT_ST: the value lands in the
                                      //microcode's internal vertex, i.e. it
                                      //is the already-scaled S10.5 texcoord
                                      //(validated against LLE: applying the
                                      //G_TEXTURE scale again picks the wrong
                                      //half of Cruis'n USA's flipping logo)
            vertex.u = (f32)(s16)(cmd1 >> 16) / 32.0f;
            vertex.v = (f32)(s16)(cmd1 & 0xffff) / 32.0f;
          } else {
            lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
          }
        }
        break;
      }
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
//Conker 0x10-0x1f: candidate decodes of one 8-byte command into triangles
//over the standard vertex buffer (indices < 32). Variant 1: the 60 bits
//below the opcode's top nibble as twelve 5-bit indices, most significant
//first, four triangles; a triangle with a repeated index is padding.
//Variant 2: the 56 bits below the opcode byte as eleven 5-bit indices
//(three triangles + a spare bit). Variant 3: as 1 but least significant
//first. Which one the microcode implements is settled by the RDP stream
//diff (rdpcmp.py) — see the round-19 report.
auto lumiverseGfxConkerTriangles(LumiverseGfxMachine& m, u32 cmd0, u32 cmd1) -> void {
  if(FILE* dump = lumiverseGfxVtxDumpFile()) fprintf(dump, "cmd task=%llu %08x %08x\n", (unsigned long long)lumiverseRdpTaskTag, cmd0, cmd1);
  const int variant = lumiverseGfxConkerTriDecode();
  if(variant == 0) return;
  const u64 q = (u64)cmd0 << 32 | cmd1;
  u32 idx[12]; u32 count = 0;
  if(variant == 1) { for(u32 i = 0; i < 12; i++) idx[i] = (q >> (55 - i * 5)) & 31; count = 12; }
  else if(variant == 2) { for(u32 i = 0; i < 11; i++) idx[i] = (q >> (51 - i * 5)) & 31; count = 9; }
  else if(variant == 3) { for(u32 i = 0; i < 12; i++) idx[i] = (q >> (i * 5)) & 31; count = 12; }
  else if(variant == 4) {
    //the layout the LLE RDP stream confirms (round 19, N-logo scene, every
    //command of the first vertex batches matched 4/4): four 15-bit
    //triangles [v0 bits 0-4][v1 5-9][v2 10-14] — A at bits 0-14, B at
    //15-29, C at 32-46, D = (bits 47-59 << 2) | bits 30-31 (the opcode's
    //low nibble is part of D); the microcode issues them as B, A, C, D; a
    //triangle with a repeated index is padding
    const u32 tri[4] = { (u32)(q >> 15) & 0x7fff, (u32)q & 0x7fff, (u32)(q >> 32) & 0x7fff, (u32)(((q >> 47) & 0x1fff) << 2 | ((q >> 30) & 3)) };
    //winding: the field order (bits 0-4, 5-9, 10-14) is the REVERSE of the
    //draw order — read forward, every triangle of the N-logo scene failed
    //the executor's backface test while the LLE microcode drew it (the
    //standard 05/06 triangles keep the normal sense: inverting the cull for
    //the whole pipeline instead dropped the match rate 57% -> 45%)
    for(u32 t = 0; t < 4; t++) { idx[t * 3] = tri[t] >> 10 & 31; idx[t * 3 + 1] = tri[t] >> 5 & 31; idx[t * 3 + 2] = tri[t] & 31; }
    count = 12;
  }
  else return;
  for(u32 t = 0; t + 3 <= count; t += 3) {
    const u32 a = idx[t], b = idx[t + 1], c = idx[t + 2];
    if(a == b || b == c || a == c) continue;
    //the microcode's z rule for these lists (LLE RDP stream of the N-logo
    //frame, 2,220 decoded candidates vs 770 drawn): a triangle with any
    //vertex beyond the far plane or in front of the near plane is dropped
    //whole — no z clipping — while x/y excursions inside the guard band are
    //drawn (scissored) as usual; backface culling applies with the winding
    //above (accepted set: 0 of 178 fully-inside triangles back-facing)
    if(variant == 4 && a < 32 && b < 32 && c < 32 && ((m.verts[a].clip | m.verts[b].clip | m.verts[c].clip) & lumiverseGfxConkerNearFarMask())) { m.trisRejected++; continue; }
    if(FILE* dump = lumiverseGfxVtxDumpFile()) {
      const auto& va = m.verts[a]; const auto& vb = m.verts[b]; const auto& vc = m.verts[c];
      auto sx = [&](const LumiverseGfxVertex& v) { return v.w != 0.0f ? m.vpTransX + m.vpScaleX * (v.x / v.w) : 0.0f; };
      auto sy = [&](const LumiverseGfxVertex& v) { return v.w != 0.0f ? m.vpTransY - m.vpScaleY * (v.y / v.w) : 0.0f; };
      fprintf(dump, "tri task=%llu %u %u %u | (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f) clip=%02x%02x%02x tris=%llu rej=%llu\n", (unsigned long long)lumiverseRdpTaskTag, a, b, c,
        sx(va), sy(va), sx(vb), sy(vb), sx(vc), sy(vc), va.clip, vb.clip, vc.clip, (unsigned long long)m.trisEmitted, (unsigned long long)m.trisRejected);
    }
    lumiverseGfxDrawTriangle(m, a, b, c, a);
  }
}

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

    if(m.dialect == LumiverseDialectS2DEX2 && lumiverseGfxExecuteS2DEX(m, opcode, cmd0, cmd1, true)) {
      if(m.out.failed) break;
      continue;
    }

    switch(opcode) {

    case 0x00: break;  //G_NOOP
    case 0xe0: break;  //G_SPNOOP

    case 0xdd:  //G_LOAD_UCODE (Zeldas / Kirby: F3DZEX <-> S2DEX2 for 2D screens)
      lumiverseGfxLoadUcode(m, cmd1);
      break;

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

    case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
    case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
      if(lumiverseGfxConker) { lumiverseGfxConkerTriangles(m, cmd0, cmd1); break; }
      lumiverseGfxLogUnimplementedOnce(opcode, cmd0, cmd1);
      break;

    case 0x08: {  //(experiment) G_LINE3D-slot opcode as a no-op (Banjo-Tooie)
      if(!lumiverseGfxOp08NoOp()) {
        static u32 logged = 0;
        if(logged++ < 4) fprintf(stderr, "[rsp-hle-gfx] executor hit gbi2 op 08 at pc=%06x (depth=%u) cmd0=%08x cmd1=%08x -> task abort\n", m.pc - 8, m.stackDepth, cmd0, cmd1);
        return false;
      }
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
      if(FILE* dump = lumiverseGfxVtxDumpFile()) {
        fprintf(dump, "mtx task=%llu pc=%06x %08x %08x at=%06x push=%d load=%d proj=%d:", (unsigned long long)lumiverseRdpTaskTag, m.pc - 8, cmd0, cmd1, address, push, load, projection);
        for(u32 e = 0; e < 16; e++) fprintf(dump, " %.4f", matrix[e]); fprintf(dump, "\n");
      }

      f32* stackBase = projection ? &m.projStack[0][0] : &m.mvStack[0][0];
      u32& depth = projection ? m.projDepth : m.mvDepth;
      const u32 maxDepth = projection ? 7 : 31;
      f32* top = stackBase + depth * 16;

      f32 result[16];
      if(!load) {
        lumiverseGfxMatrixMultiply(matrix, top, result);
        if(lumiverseGfxConker && projection) for(u32 index = 0; index < 16; index++) m.conkerView[index] = matrix[index];
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
      if(FILE* dump = lumiverseGfxVtxDumpFile()) fprintf(dump, "mw task=%llu pc=%06x %08x %08x\n", (unsigned long long)lumiverseRdpTaskTag, m.pc - 8, cmd0, cmd1);
      //round 21 oracle: LUMIVERSE_ARES_N64_GFX_MW_PATCH="<type hex>:<offset hex>:<w1 hex>[,...]"
      //(shadow mode) rewrites the data word of matching G_MOVEWORD commands
      //in RDRAM after the executor read them, so the LLE microcode — which
      //runs the task next — sees the patched value; the DL is rebuilt by
      //the CPU every frame. Used on Conker's undecoded type 0x10 block.
      if(lumiverseGfxShadowDiscard) {
        static const char* patch = ::getenv("LUMIVERSE_ARES_N64_GFX_MW_PATCH");
        if(patch) {
          const char* c = patch;
          while(*c) {
            char* end = nullptr;
            const u32 ptype = (u32)::strtoul(c, &end, 16);
            if(!end || *end != ':') break;
            const u32 poffset = (u32)::strtoul(end + 1, &end, 16);
            if(!end || *end != ':') break;
            const u32 pw1 = (u32)::strtoul(end + 1, &end, 16);
            if(ptype == type && poffset == offset) {
              lumiverseGfxWriteWord(m.pc - 4, pw1);
              static u32 logged = 0;
              if(logged++ < 4) fprintf(stderr, "[rsp-hle-gfx] mw patch task=%llu type=%02x offset=%04x %08x -> %08x\n", (unsigned long long)lumiverseRdpTaskTag, type, offset, cmd1, pw1);
            }
            if(!end || *end != ',') break;
            c = end + 1;
          }
        }
      }
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
        if(lumiverseGfxConker) m.conkerNumLights = (cmd1 / 48) < 15 ? cmd1 / 48 : 15;  //48-byte entries
        break;
      case 0x04:  //G_MW_CLIP: guard-band ratio
        lumiverseGfxSetClipRatio(m, cmd1);
        break;
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
      if(lumiverseGfxConker && lumiverseGfxShadowDiscard && type == 10 && offset >= 0x60 && lumiverseGfxLightPatch().on) {
        //the game's own light structs are CPU state (patching them in place
        //stalled the game at 428 tasks per 2400 steps): build the patched
        //copy in a scratch block at the top of RDRAM and redirect THIS
        //command's pointer (the DL is rebuilt by the CPU every frame)
        const auto& p = lumiverseGfxLightPatch();
        const u32 size = (((cmd0 >> 19) & 0x1f) + 1) * 8;
        const u32 scratch = 0x7fe000 + (offset - 0x60);
        for(u32 e = 0; e < size / 4; e++) lumiverseGfxWriteWord(scratch + e * 4, lumiverseGfxWord(address + e * 4));
        lumiverseGfxWriteWord(m.pc - 4, 0x80000000 | scratch);
        for(u32 entry = 0; entry * 48 < size; entry++) {
          const u32 at = scratch + entry * 48;
          const bool ambient = lumiverseGfxWord(at + 8) == 0 && lumiverseGfxWord(at + 12) == 0 && lumiverseGfxWord(at + 32) == 0;  //dir 0, no param, no position
          const bool probe = offset == 0x60 && entry == 0;
          const u32 col = ambient ? p.amb << 8 : probe ? p.col << 8 : 0;
          lumiverseGfxWriteWord(at + 0, col); lumiverseGfxWriteWord(at + 4, col);
          if(!ambient) {
            lumiverseGfxWriteWord(at + 8, probe ? ((u32)(u8)p.dir[0] << 24 | (u32)(u8)p.dir[1] << 16 | (u32)(u8)p.dir[2] << 8) : 0x00007f00);
            lumiverseGfxWriteWord(at + 12, probe ? p.param << 24 : 0);
            const u32 w0 = probe ? ((u32)(u16)p.pos[0] << 16 | (u16)p.pos[1]) : 0, w1 = probe ? (u32)(u16)p.pos[2] << 16 : 0;
            lumiverseGfxWriteWord(at + 32, w0); lumiverseGfxWriteWord(at + 36, w1); lumiverseGfxWriteWord(at + 40, w0); lumiverseGfxWriteWord(at + 44, w1);
          }
        }
      }
      if(FILE* dump = lumiverseGfxVtxDumpFile()) {
        const u32 size = (((cmd0 >> 19) & 0x1f) + 1) * 8;
        fprintf(dump, "mm task=%llu pc=%06x %08x %08x type=%u offset=%03x at=%06x raw:", (unsigned long long)lumiverseRdpTaskTag, m.pc - 8, cmd0, lumiverseGfxWord(m.pc - 4), type, offset, lumiverseGfxSegmentAddress(m, lumiverseGfxWord(m.pc - 4)));
        for(u32 e = 0; e < size / 4 && e < 32; e++) fprintf(dump, " %08x", lumiverseGfxWord(address + e * 4)); fprintf(dump, "\n");
      }
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
        if(lumiverseGfxConker) {
          //Conker: 16-byte lookat rows at 0x00/0x30, then 48-byte entries
          //from 0x60 — colour (3), colour copy, direction (s8 x3), an
          //attenuation parameter byte, 16 zero bytes, the view-space
          //position (s16 x3) twice; entry numLights is the ambient
          if(offset >= 0x60) {
            const u32 size = (((cmd0 >> 19) & 0x1f) + 1) * 8;
            for(u32 e = 0; e * 48 < size; e++) {
              const u32 index = (offset - 0x60) / 48 + e; if(index >= 16) break;
              const u32 at = address + e * 48; auto& l = m.conkerLights[index];
              l.r = lumiverseGfxByte(at + 0); l.g = lumiverseGfxByte(at + 1); l.b = lumiverseGfxByte(at + 2);
              l.dx = lumiverseGfxSByte(at + 8); l.dy = lumiverseGfxSByte(at + 9); l.dz = lumiverseGfxSByte(at + 10);
              l.param = lumiverseGfxByte(at + 12);
              l.px = lumiverseGfxShort(at + 32); l.py = lumiverseGfxShort(at + 34); l.pz = lumiverseGfxShort(at + 36);
            }
          }
          break;
        }
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
        if(FILE* dump = lumiverseGfxVtxDumpFile()) {
          fprintf(dump, "force task=%llu pc=%06x %08x %08x at=%06x raw:", (unsigned long long)lumiverseRdpTaskTag, m.pc - 8, cmd0, cmd1, address);
          for(u32 e = 0; e < 16; e++) fprintf(dump, " %08x", lumiverseGfxWord(address + e * 4)); fprintf(dump, "\n");
        }
        //round 20: Conker's build issues this 64-byte block (index 0x0e) once
        //per bone group of its skinned characters, between the bone's G_MTX
        //and its G_VTX; the block is not a matrix (raw words such as
        //3728fa7f / 8202xxxx, no s16.16 int/frac halves) — treating it as
        //gSPForceMatrix replaced the MVP with garbage and every vertex of
        //the group landed far behind the camera (w = -77602 on the N-logo
        //frame; the class behind 73% of the stray on-screen area). The
        //bone's MV x P is what the LLE stream shows for those vertices.
        if(lumiverseGfxConker && !lumiverseGfxConkerMv14()) {
          //round 21: the block is the batch's packed-normal table — one
          //halfword per vertex-buffer slot (x, y as s8 / 127; z is the
          //vertex's flag byte). Consecutive G_VTX loads of one bone group
          //share one block (the game stores them per 32-slot chunk).
          for(u32 slot = 0; slot < 32; slot++) m.conkerNormalXY[slot] = lumiverseGfxHalf(address + slot * 2);
          break;
        }
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

//round 19: per-task display-list features gathered by the dry-run walkers
//(reset at each walk): opcode counts, vertices loaded, triangles issued.
//They feed the graphics cost model (the modelled RSP duration of an HLE
//task) and the LLE cost log it was fitted on.
struct LumiverseGfxTaskFeatures {
  u32 ops[256] = {};
  u32 commands = 0;
  u32 vertices = 0;
  u32 tris = 0;
};
static LumiverseGfxTaskFeatures lumiverseGfxTaskFeatures;


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

//S2DEX dry-run helpers (defined after the GBI1 walker)
auto lumiverseGfxS2DEXDumpFile() -> FILE*;
auto lumiverseGfxS2DEXDumpTaskWanted() -> bool;
auto lumiverseGfxS2DEXDumpCommand(FILE* fp, u32 pc, u32 cmd0, u32 cmd1, u32 structAt, u32 structBytes, const char* tag) -> void;
auto lumiverseGfxS2DEXStructBytes(u8 opcode, u32 cmd0, bool gbi2) -> u32;
auto lumiverseGfxS2DEXAdmit(u8 opcode, u32 cmd0, u32 cmd1, bool gbi2, u32 structAt, const char*& why) -> bool;

//dry-run walker shared by GBI1 and GBI0 (Fast3D): flow control + segment
//table only; any unsupported opcode fails the whole task over to LLE
auto lumiverseGfxDryRun(u32 pc, LumiverseGfxCensus& census, u32 dialect = LumiverseDialectGBI1, bool bakedRDP = false, u32 gbi0Vertex = LumiverseGBI0VertexStandard) -> bool {
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
  auto& features = lumiverseGfxTaskFeatures;
  features = {};

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

  //S2DEX (round 10): a task may start in S2DEX (Yoshi's Story) or switch
  //into/out of it with G_LOAD_UCODE; while in S2DEX the object commands are
  //admitted per lumiverseGfxS2DEXAdmit and inline RDP triangles are skipped
  bool s2dex = dialect == LumiverseDialectS2DEX;
  FILE* s2dexDump = s2dex && lumiverseGfxS2DEXDumpTaskWanted() ? lumiverseGfxS2DEXDumpFile() : nullptr;
  if(s2dexDump) fprintf(s2dexDump, "task %llu dl=%06x dialect=s2dex\n", (unsigned long long)lumiverseRdpTaskTag, pc);

  u32 steps = 0;
  while(pc && steps++ < 1000000) {
    const u32 cmd0 = lumiverseGfxWord(pc + 0);
    const u32 cmd1 = lumiverseGfxWord(pc + 4);
    pc += 8;
    const u8 opcode = cmd0 >> 24;
    if(dlDump && steps < 20000) fprintf(dlDump, "  %06x: %08x %08x\n", pc - 8, cmd0, cmd1);

    census.counts[opcode]++;
    if(!census.seen[opcode]) { census.seen[opcode] = true; newOpcode = true; }
    features.ops[opcode]++;
    features.commands++;
    if(!s2dex) {
      if(opcode == 0x04) features.vertices += ((cmd0 >> 20) & 0xf) + 1;
      else if(opcode == 0xbf) features.tris += 1;
      else if(opcode == 0xb1) features.tris += dialect == LumiverseDialectGBI0 ? 4 : 2;
    }

    if(opcode == 0xaf) {  //G_LOAD_UCODE
      const s32 target = lumiverseGfxLoadUcodeDialect(cmd1);
      if(s2dexDump) lumiverseGfxS2DEXDumpCommand(s2dexDump, pc - 8, cmd0, cmd1, 0, 0, target == LumiverseDialectS2DEX ? " ->s2dex" : target == LumiverseDialectGBI1 ? " ->f3dex" : " ->?");
      if(target == LumiverseDialectS2DEX) s2dex = true;
      else if(target == LumiverseDialectGBI1 && dialect != LumiverseDialectGBI0) s2dex = false;
      else { lumiverseGfxLogRejectOnce(6, opcode, cmd0, cmd1, "load-ucode target"); supported = false; break; }
      continue;
    }
    if(s2dex) {
      const u32 structBytes = lumiverseGfxS2DEXStructBytes(opcode, cmd0, false);
      const u32 structAt = structBytes ? segmentAddress(cmd1) : 0;
      const char* why = nullptr;
      const bool admitted = lumiverseGfxS2DEXAdmit(opcode, cmd0, cmd1, false, structAt, why);
      if(s2dexDump && steps < 20000) lumiverseGfxS2DEXDumpCommand(s2dexDump, pc - 8, cmd0, cmd1, structAt, structBytes, admitted ? "" : " !");
      if(opcode >= 0xc8 && opcode <= 0xcf) {
        const u32 qwords = lumiverseGfxRDPCommandQwords[opcode & 0x3f];
        if(s2dexDump && steps < 20000) for(u32 q = 1; q < qwords; q++) fprintf(s2dexDump, "  %06x:   %08x %08x\n", pc + (q - 1) * 8, lumiverseGfxWord(pc + (q - 1) * 8), lumiverseGfxWord(pc + (q - 1) * 8 + 4));
        pc += (qwords - 1) * 8;
        continue;
      }
      if(!admitted) {
        static bool logged[256];
        if(!logged[opcode]) {
          logged[opcode] = true;
          fprintf(stderr, "[rsp-hle-gfx] unsupported s2dex op %02x (%s) cmd0=%08x cmd1=%08x at pc=%06x -> LLE fallback\n", opcode, why ? why : "", cmd0, cmd1, pc - 8);
        }
        lumiverseGfxRejectCounts[7]++;
        supported = false;
        break;
      }
      switch(opcode) {  //S2DEX-specific opcodes that overlap GBI1 numbers are consumed here
      case 0x01: case 0x02: case 0x03: case 0x04: case 0x05:
      case 0xb0: case 0xb1: case 0xb2: case 0xc1: case 0xc2: case 0xc3: case 0xc4:
        continue;
      default: break;
      }
      if(!lumiverseGfxOpcodeSupported(opcode)) {
        static bool logged[256];
        if(!logged[opcode]) { logged[opcode] = true; fprintf(stderr, "[rsp-hle-gfx] unsupported s2dex-base op %02x cmd0=%08x cmd1=%08x -> LLE fallback\n", opcode, cmd0, cmd1); }
        supported = false;
        break;
      }
    } else
    if(opcode == 0x07 && dialect == LumiverseDialectGBI0 && gbi0Vertex == LumiverseGBI0VertexPD) {
      continue;  //PD colour-table pointer (no flow-control effect)
    } else
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
      else if((cmd0 & 0xff) == 0x0c) {
        //G_MW_POINTS (Cruis'n USA: 81% of in-game tasks — round 9): the
        //pre-G_MODIFYVTX way of patching one word of a loaded vertex. The
        //offset addresses the microcode's 40-byte internal vertex; the
        //RGBA (0x10) and ST (0x14) fields map onto the G_MODIFYVTX cases the
        //executor already has; screen XY/Z (0x18/0x1c) stay unsupported
        const u32 field = ((cmd0 >> 8) & 0xffff) % 40;
        if(field != 0x10 && field != 0x14) {
          lumiverseGfxLogRejectOnce(2, opcode, cmd0, cmd1, "moveword POINTS (non RGBA/ST field)");
          supported = false;
        }
      }
      else if((cmd0 & 0xff) == 0x00) {  //MW_MATRIX
        lumiverseGfxLogRejectOnce(2, opcode, cmd0, cmd1, "moveword MATRIX");
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

  if(s2dexDump) fflush(s2dexDump);
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

//----------------------------------------------------------------------------
//S2DEX dry-run / census (round 10). Walks the display list with the base
//dialect's flow control (GBI1 for S2DEX 1.xx, GBI2 for S2DEX2), skips the
//inline RDP triangle commands (0xc8..0xcf carry a whole RDP command: the
//CPU computed the coefficients, the microcode only forwards them) and dumps
//every command plus the RDRAM structures the object commands point at into
//the DL dump file, for empirical derivation against LLE's RDP stream.
//----------------------------------------------------------------------------

//bytes of the RDRAM structure an S2DEX object command points at (n64js
//gbi_s2dex.js field layouts; MIT): 0 = no structure
auto lumiverseGfxS2DEXStructBytes(u8 opcode, u32 cmd0, bool gbi2) -> u32 {
  if(!gbi2) {
    switch(opcode) {
    case 0x01: case 0x02: return 40;          //BG_1CYC / BG_COPY (uObjBg)
    case 0x03: case 0x04: case 0xb2: return 24; //OBJ_RECTANGLE / OBJ_SPRITE / OBJ_RECTANGLE_R (uObjSprite)
    case 0x05: return (cmd0 & 0xffff) == 0 ? 24 : 8;  //OBJ_MOVEMEM: matrix / submatrix
    case 0xc1: return 24;                     //OBJ_LOADTXTR (uObjTxtr)
    case 0xc2: case 0xc3: case 0xc4: return 48; //OBJ_LDTX_* (uObjTxtr + uObjSprite)
    default: return 0;
    }
  }
  switch(opcode) {
  case 0x01: case 0x02: case 0xda: return 24;
  case 0x05: return 24;
  case 0x06: case 0x07: case 0x08: return 48;
  case 0x09: case 0x0a: return 40;
  case 0xdc: return (cmd0 & 0xfe) == 0 ? 24 : ((cmd0 & 0xfe) == 2 ? 8 : 0);
  default: return 0;
  }
}

auto lumiverseGfxOpcodeSupportedS2DEX(u8 opcode, bool gbi2) -> bool {
  //RDP passthrough + syncs shared by both variants
  switch(opcode) {
  case 0xe4: case 0xe5: case 0xe6: case 0xe7: case 0xe8:
  case 0xe9: case 0xea: case 0xeb: case 0xec: case 0xed:
  case 0xee: case 0xef: case 0xf0: case 0xf2: case 0xf3:
  case 0xf4: case 0xf5: case 0xf6: case 0xf7: case 0xf8:
  case 0xf9: case 0xfa: case 0xfb: case 0xfc: case 0xfd:
  case 0xfe: case 0xff:
  case 0xc8: case 0xc9: case 0xca: case 0xcb:
  case 0xcc: case 0xcd: case 0xce: case 0xcf:
    return true;
  default: break;
  }
  if(!gbi2) {
    switch(opcode) {
    case 0x00: case 0x06: case 0xb8: case 0xbc: case 0xb4: case 0xb3: case 0xc0:
    case 0xb6: case 0xb7: case 0xb9: case 0xba: case 0xbb:  //GBI1 immediates
    case 0x01: case 0x02: case 0x03: case 0x04: case 0x05:
    case 0xb0: case 0xb1: case 0xb2:
    case 0xc1: case 0xc2: case 0xc3: case 0xc4:
      return true;
    default: return false;
    }
  }
  switch(opcode) {
  case 0x00: case 0xde: case 0xdf: case 0xdb: case 0xdc: case 0xe0: case 0xe1: case 0xf1:
  case 0xe2: case 0xe3:
  case 0x01: case 0x02: case 0x04: case 0x05: case 0x06: case 0x07: case 0x08:
  case 0x09: case 0x0a: case 0x0b: case 0xd5: case 0xda:
    return true;
  default: return false;
  }
}

//S2DEX command dump (LUMIVERSE_ARES_N64_RSP_HLE_S2DEX_DUMP=<path>, first
//..._GFX_DL_DUMP_TASKS tasks after ..._GFX_DL_DUMP_SKIP): every walked
//command plus the RDRAM structure it points at, tagged with the task ordinal
//the RDP stream dump carries — empirical derivation of S2DEX semantics
auto lumiverseGfxS2DEXDumpFile() -> FILE* {
  static FILE* dlDump = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_S2DEX_DUMP");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  return dlDump;
}
auto lumiverseGfxS2DEXDumpTaskWanted() -> bool {
  static const u32 limit = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP_TASKS"); return v ? (u32)::atoi(v) : 400u; }();
  static const u32 skip = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP_SKIP"); return v ? (u32)::atoi(v) : 0u; }();
  static u32 seen = 0;
  if(!lumiverseGfxS2DEXDumpFile()) return false;
  seen++;
  return seen > skip && seen <= skip + limit;
}
auto lumiverseGfxS2DEXDumpCommand(FILE* fp, u32 pc, u32 cmd0, u32 cmd1, u32 structAt, u32 structBytes, const char* tag) -> void {
  fprintf(fp, "  %06x: %08x %08x%s", pc, cmd0, cmd1, tag);
  if(structBytes) {
    fprintf(fp, "  @%06x |", structAt);
    for(u32 b = 0; b < structBytes; b += 2) fprintf(fp, " %04x", lumiverseGfxHalf(structAt + b));
  }
  fprintf(fp, "\n");
}

//dry-run admission of the S2DEX object commands actually implemented:
//sprites (with/without texture load), the matrix movemems, render mode,
//inline RDP triangles; texture loads of type block/TLUT only. Backgrounds
//(BG_1CYC/BG_COPY), rectangles and select-DL are rejected until validated.
auto lumiverseGfxS2DEXAdmit(u8 opcode, u32 cmd0, u32 cmd1, bool gbi2, u32 structAt, const char*& why) -> bool {
  why = nullptr;
  (void)cmd1;
  if(opcode >= 0xc8 && opcode <= 0xcf) return true;
  auto textureOk = [&](u32 at) -> bool {
    const u32 command = lumiverseGfxWord(at) & 0xff;
    if(command == 0x33 || command == 0x30 || command == 0x34) return true;
    why = "s2dex texture type";
    return false;
  };
  if(!gbi2) {
    switch(opcode) {
    case 0x04: case 0xb1: return true;
    case 0x05: { const u32 index = cmd0 & 0xffff; if(index == 0 || index == 2) return true; why = "s2dex movemem index"; return false; }
    case 0xc1: case 0xc2: case 0xc3: case 0xc4: return textureOk(structAt);
    case 0x03: case 0xb2: return true;
    case 0x01: if(lumiverseGfxS2DEXBackground4BitSupported(structAt) || lumiverseGfxS2DEXBackgroundSupported(structAt)) return true; why = "s2dex BG_1CYC (scrolled/scaled/flipped)"; return false;
    case 0x02: if(lumiverseGfxS2DEXBackgroundCopySupported(structAt)) return true; why = "s2dex BG_COPY (scrolled/flipped/non-16-bit)"; return false;
    case 0xb0: why = "s2dex SELECT_DL"; return false;
    default: return true;  //base GBI1 command
    }
  }
  switch(opcode) {
  case 0x02: case 0x0b: return true;
  case 0x05: case 0x06: case 0x07: case 0x08: return textureOk(structAt);
  case 0xdc: return true;  //types 0/2 are the object matrices; others: base GBI2 rules
  case 0x01: case 0xda: return true;
  case 0x04: why = "s2dex2 SELECT_DL"; return false;
  case 0x09: if(lumiverseGfxS2DEXBackground4BitSupported(structAt) || lumiverseGfxS2DEXBackgroundSupported(structAt)) return true; why = "s2dex2 BG_1CYC (scrolled/scaled/flipped)"; return false;
  case 0x0a: {
    if(lumiverseGfxS2DEXBackgroundCopySupported(structAt)) return true;
    //round 21: name the form in the log (imageX/Y 10.5, flip, load, siz)
    static char form[160]; static u32 logged = 0;
    if(logged++ < 3) fprintf(stderr, "[rsp-hle-gfx] s2dex2 BG_COPY struct at %06x: imageX=%04x imageW=%04x frameX=%04x frameW=%04x imageY=%04x imageH=%04x frameY=%04x frameH=%04x load=%04x fmt/siz=%02x%02x flip=%04x tmemW=%04x tmemH=%04x\n",
      structAt, lumiverseGfxHalf(structAt + 0), lumiverseGfxHalf(structAt + 2), lumiverseGfxHalf(structAt + 4), lumiverseGfxHalf(structAt + 6), lumiverseGfxHalf(structAt + 8), lumiverseGfxHalf(structAt + 10), lumiverseGfxHalf(structAt + 12), lumiverseGfxHalf(structAt + 14),
      lumiverseGfxHalf(structAt + 20), lumiverseGfxByte(structAt + 22), lumiverseGfxByte(structAt + 23), lumiverseGfxHalf(structAt + 26), lumiverseGfxHalf(structAt + 28), lumiverseGfxHalf(structAt + 30));
    (void)form;
    why = "s2dex2 BG_COPY (scrolled/flipped/non-16-bit)"; return false;
  }
  case 0xd5: why = "s2dex2 DL_COUNT"; return false;
  default: return true;  //base GBI2 command
  }
}

auto lumiverseGfxOpcodeSupportedGBI2(u8 opcode) -> bool {
  switch(opcode) {
  case 0x08: return lumiverseGfxOp08NoOp();
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
  auto& features = lumiverseGfxTaskFeatures;
  features = {};

  auto segmentAddress = [&](u32 address) -> u32 {
    return (segments[(address >> 24) & 0xf] + (address & 0x00ffffff)) & 0x00ffffff;
  };
  bool s2dex = false;
  FILE* s2dexDump = lumiverseGfxS2DEXDumpTaskWanted() ? lumiverseGfxS2DEXDumpFile() : nullptr;
  if(s2dexDump) fprintf(s2dexDump, "task %llu dl=%06x dialect=gbi2\n", (unsigned long long)lumiverseRdpTaskTag, rootPC);
  //round 19: LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP=<path> for GBI2 too
  //(Conker's custom ops): every walked command of the tasks in the
  //DL_DUMP_SKIP/DL_DUMP_TASKS window, tagged by the RDP task ordinal so the
  //LLE RDP stream dump (RDP_STREAM_DUMP, same tag) lines up per task; an
  //unsupported opcode also dumps the 256 raw qwords that follow it
  static FILE* dlDump = [] () -> FILE* {
    const char* path = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP");
    return path && path[0] ? fopen(path, "w") : nullptr;
  }();
  static const u32 dlDumpLimit = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP_TASKS"); return v ? (u32)::atoi(v) : 400u; }();
  static const u32 dlDumpSkip = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_DL_DUMP_SKIP"); return v ? (u32)::atoi(v) : 0u; }();
  FILE* dl = dlDump && lumiverseRdpTaskTag > dlDumpSkip && lumiverseRdpTaskTag <= dlDumpSkip + dlDumpLimit ? dlDump : nullptr;
  if(dl) fprintf(dl, "task %llu dl=%06x dialect=gbi2\n", (unsigned long long)lumiverseRdpTaskTag, rootPC);

  u32 steps = 0;
  u32 pc = rootPC;
  while(pc && steps++ < 1000000) {
    const u32 cmd0 = lumiverseGfxWord(pc + 0);
    const u32 cmd1 = lumiverseGfxWord(pc + 4);
    pc += 8;
    const u8 opcode = cmd0 >> 24;
    if(dl && steps < 20000) fprintf(dl, "  %06x: %08x %08x%s\n", pc - 8, cmd0, cmd1, stackDepth ? "" : " *");

    census.counts[opcode]++;
    if(!census.seen[opcode]) { census.seen[opcode] = true; newOpcode = true; }
    features.ops[opcode]++;
    features.commands++;
    if(!s2dex) {
      if(opcode == 0x01) features.vertices += (cmd0 >> 12) & 0xff;
      else if(opcode == 0x05) features.tris += 1;
      else if(opcode == 0x06 || opcode == 0x07) features.tris += 2;
    }

    if(opcode == 0xdd) {  //G_LOAD_UCODE (round 10: F3DZEX <-> S2DEX2)
      const s32 target = lumiverseGfxLoadUcodeDialect(cmd1);
      if(s2dexDump) lumiverseGfxS2DEXDumpCommand(s2dexDump, pc - 8, cmd0, cmd1, 0, 0, target == LumiverseDialectS2DEX2 ? " ->s2dex2" : target == LumiverseDialectGBI2 ? " ->gbi2" : " ->?");
      if(target == LumiverseDialectS2DEX2) s2dex = true;
      else if(target == LumiverseDialectGBI2) s2dex = false;
      else { lumiverseGfxLogRejectOnce(6, opcode, cmd0, cmd1, "load-ucode target"); supported = false; break; }
      continue;
    }
    if(s2dex) {
      const u32 structBytes = lumiverseGfxS2DEXStructBytes(opcode, cmd0, true);
      const u32 structAt = structBytes ? segmentAddress(cmd1) : 0;
      const char* why = nullptr;
      const bool admitted = lumiverseGfxS2DEXAdmit(opcode, cmd0, cmd1, true, structAt, why);
      if(s2dexDump && steps < 20000) lumiverseGfxS2DEXDumpCommand(s2dexDump, pc - 8, cmd0, cmd1, structAt, structBytes, admitted ? "" : " !");
      if(opcode >= 0xc8 && opcode <= 0xcf) {
        const u32 qwords = lumiverseGfxRDPCommandQwords[opcode & 0x3f];
        if(s2dexDump && steps < 20000) for(u32 q = 1; q < qwords; q++) fprintf(s2dexDump, "  %06x:   %08x %08x\n", pc + (q - 1) * 8, lumiverseGfxWord(pc + (q - 1) * 8), lumiverseGfxWord(pc + (q - 1) * 8 + 4));
        pc += (qwords - 1) * 8;
        continue;
      }
      if(!admitted) {
        static bool logged[256];
        if(!logged[opcode]) {
          logged[opcode] = true;
          fprintf(stderr, "[rsp-hle-gfx] unsupported s2dex2 op %02x (%s) cmd0=%08x cmd1=%08x at pc=%06x -> LLE fallback\n", opcode, why ? why : "", cmd0, cmd1, pc - 8);
        }
        lumiverseGfxRejectCounts[7]++;
        supported = false;
        break;
      }
      switch(opcode) {  //S2DEX2-specific opcodes that overlap GBI2 numbers are consumed here
      case 0x01: case 0x02: case 0x04: case 0x05: case 0x06: case 0x07: case 0x08:
      case 0x09: case 0x0a: case 0x0b: case 0xd5: case 0xda:
        continue;
      case 0xdc:
        if((cmd0 & 0xfe) == 0 || (cmd0 & 0xfe) == 2) continue;
        break;
      default: break;
      }
    }

    if(lumiverseGfxConker && opcode >= 0x10 && opcode <= 0x1f) continue;  //Conker triangle list (executor)
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
      if(dl) {
        fprintf(dl, "  unsupported %02x at %06x; raw qwords follow:\n", opcode, pc - 8);
        for(u32 q = 0; q < 256; q++) fprintf(dl, "  %06x: %08x %08x\n", pc + q * 8, lumiverseGfxWord(pc + q * 8), lumiverseGfxWord(pc + q * 8 + 4));
        //the segment table + the referenced blocks of the custom command
        fprintf(dl, "  segments:"); for(u32 s = 0; s < 16; s++) fprintf(dl, " %x=%06x", s, segments[s]); fprintf(dl, "\n");
        const u32 ref = segmentAddress(cmd1);
        fprintf(dl, "  cmd1 -> %06x:\n", ref);
        for(u32 q = 0; q < 128; q++) fprintf(dl, "  %06x: %08x %08x\n", ref + q * 8, lumiverseGfxWord(ref + q * 8), lumiverseGfxWord(ref + q * 8 + 4));
        fflush(dl);
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

  if(s2dexDump) fflush(s2dexDump);
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

//three-letter game code from the cartridge header (bytes 0x3b-0x3d; the
//fourth byte is the region) — for microcode images shared between games
//that must be gated differently (Banjo-Tooie vs THPS2)
auto lumiverseGfxCartridgeCode() -> string {
  static const string code = [] {
    string value;
    if(cartridge.rom.size < 0x40) return value;
    for(u32 index = 0x3b; index < 0x3e; index++) value.append((char)cartridge.rom.read<Byte>(index));
    return value;
  }();
  return code;
}

//env knob that defaults ON: only an explicit "0" turns it off
auto lumiverseGfxEnvDefaultOn(const char* name) -> bool {
  const char* value = ::getenv(name);
  return !value || value[0] != '0';
}

#if defined(VULKAN)
//microcode hash -> executor dialect (+ vertex variant, GoldenEye baked-RDP
//admission); false = not whitelisted or opted out by its env knob
auto lumiverseGfxResolveDialect(u64 ucodeHash, u32& dialect, u32& gbi0Vertex, bool& geBaked) -> bool {
  gbi0Vertex = LumiverseGBI0VertexStandard;
  geBaked = false;
  lumiverseGfxConker = ucodeHash == 0x63a15d2f6bdae1f5ull;
  switch(ucodeHash) {
  case LumiverseUcodeF3DEXNoN122:   dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DEX121:      dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DEXNoN100:   dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DEX095:      dialect = LumiverseDialectGBI1; break;
  case LumiverseUcodeF3DZEXNoN208J: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DZEXNoN206H: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DZEXNoN208I: dialect = LumiverseDialectGBI2; break;
  //Banjo-Tooie (U) "F3DEX.NoN fifo 2.08": DE-WHITELISTED in round 8. The
  //intro validated in the M-GBI2 round, but every post-intro display list
  //carries op 0x08 (whole-task LLE fallback) and — worse — the scenes that
  //do run under the GBI2 executor render as black/garbage triangles
  //(cutscene at steps 4500/6000 vs the LLE reference, see the round-8
  //report), and an LLE task after HLE tasks leaves the fifo microcode
  //spinning forever (frozen picture). Opt back in with
  //LUMIVERSE_ARES_N64_BT_GFX_HLE=1 for further work.
  //Round 10: RE-WHITELISTED. With the round-9 fifo/FREEZE writeback fix the
  //round-8 symptoms do not reproduce: 40,000 steps (title, file select,
  //the whole opening cutscene through Gruntilda's sisters, Bottles' ghost
  //and Banjo's house) with 12,097 HLE tasks, 0 fallbacks — no op 0x08/0x20
  //was ever issued — no hang, and a 20,000-step HLE-vs-LLE checkpoint
  //series byte-identical at 2000/9000/11000/17000 and phase-only elsewhere.
  //Gameplay past the cutscene was not reached by the harness script; if the
  //Rare ops appear there the dry-run falls those tasks back per task, and
  //LUMIVERSE_ARES_N64_BT_GFX_HLE=0 opts the game out entirely.
  case LumiverseUcodeF3DEX2NoN208: {
    //Tony Hawk's Pro Skater 2 (U) ships the same microcode image (NTQ);
    //LUMIVERSE_ARES_N64_THPS2_GFX_HLE=0 opts it out separately
    const bool thps2 = lumiverseGfxCartridgeCode() == "NTQ";
    if(thps2 ? !lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_THPS2_GFX_HLE") : !lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_BT_GFX_HLE")) return false;
    dialect = LumiverseDialectGBI2; break;
  }
  case LumiverseUcodeF3DEX2204H:    dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2206:     dialect = LumiverseDialectGBI2; break;
  //Donkey Kong 64 (U) "F3DEX fifo 2.07": RE-WHITELISTED in round 9. The
  //August "hang at the DK Rap" was two fifo bugs, not the microcode: (1)
  //Rare's engine holds DPC FREEZE across the task, and every HLE flush used
  //to restart the fifo at output_buff, dropping all but the last flush of a
  //frozen frame (the DK Rap tasks exceed the 160 KB fifo); (2) after a
  //kick that ended inside an RDP command, paraLLEl's render() dropped the
  //next full-fifo kick outright (queue never compacted) — the frame's
  //SyncFull vanished and the game waited for the DP interrupt forever.
  //Gate: 4000 steps through the DK Rap into the title attract, checkpoints
  //byte-identical to LLE at 1000/2500/4000; LUMIVERSE_ARES_N64_DK64_GFX_HLE=0
  //opts out.
  case LumiverseUcodeF3DEX2207: {
    if(!lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_DK64_GFX_HLE")) return false;
    dialect = LumiverseDialectGBI2; break;
  }
  //Cruis'n World (U) "RSP Gfx ucode F3DEX fifo 2.04" (resident ucode,
  //ucode_size 0): round-9 gate — 0 fallbacks over 4000 steps into the first
  //race, menu/race checkpoints content-identical to LLE, host 135 vs 52
  //steps/s. LUMIVERSE_ARES_N64_CWORLD_GFX_HLE=0 opts out.
  case 0xfdeacd35544ff754ull: {
    if(!lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_CWORLD_GFX_HLE")) return false;
    dialect = LumiverseDialectGBI2; break;
  }
  //San Francisco Rush (U): "RSP Gfx ucode F3DLX.NoN 1.21" — the GBI1
  //command set (the LX line-drawing variant; SF64's F3DEX.NoN 1.21 image is
  //its second, already-whitelisted ucode). Round-9 gate: 0/1793 fallbacks
  //over 4000 steps into the first race, race checkpoints content-identical
  //to LLE (car/timer phase only), host 120 vs 36 steps/s.
  //LUMIVERSE_ARES_N64_RUSH_GFX_HLE=0 opts out.
  case 0x2d3bb1207b5332deull: {
    if(!lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_RUSH_GFX_HLE")) return false;
    dialect = LumiverseDialectGBI1; break;
  }
  case LumiverseUcodeF3DEX2NoN208H: dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2208K:    dialect = LumiverseDialectGBI2; break;
  case LumiverseUcodeF3DEX2208:     dialect = LumiverseDialectGBI2; break;
  //Yoshi's Story (U) S2DEX 1.06 — round 10 (opt-out LUMIVERSE_ARES_N64_YOSHI_GFX_HLE=0)
  case LumiverseUcodeS2DEX106: {
    if(!lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_YOSHI_GFX_HLE")) return false;
    dialect = LumiverseDialectS2DEX; break;
  }
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
    //LUMIVERSE 2026-09-01: device log with this path = 45-70% speed at
    //24-36 ms/field (CPU-bound; GoldenEye is the heaviest title and the
    //device has no JIT) — still faster than the LLE RSP path it replaced,
    //so it stays ON. LUMIVERSE_ARES_N64_GE_HLE=0 forces the LLE path for A/B.
    const char* optOut = ::getenv("LUMIVERSE_ARES_N64_GE_HLE");
    if(optOut && optOut[0] == '0') return false;
    dialect = LumiverseDialectGBI0; geBaked = true; break;
  }
  //Perfect Dark (U): bannerless Rare microcode (hash b8c3bdd1902f32f7,
  //round-9 census). Round 12: GoldenEye's Fast3D dialect (TRI4, baked-RDP
  //runs admitted) plus the PD vertex variant — custom op 0x07 = colour
  //table pointer, 12-byte vertices indexing it (our DL dumps: every VTX is
  //preceded by an 07 whose cmd1 sits 0x150 past the vertex block, VTX
  //lengths are n*12; n64js gbi0.js GBI0PD documents the same layout).
  //Gate: 0 fallbacks over 4225 tasks (7000 steps: logos, intro cutscene,
  //agent menus, Carrington Institute walk-through); checkpoints identical
  //or camera/menu-animation phase against the LLE frames. Host 152-160
  //steps/s vs 91-94 with the LLE RSP. LUMIVERSE_ARES_N64_PD_GFX_HLE=0 opts out.
  case 0xb8c3bdd1902f32f7ull: {
    if(!lumiverseGfxEnvDefaultOn("LUMIVERSE_ARES_N64_PD_GFX_HLE")) return false;
    dialect = LumiverseDialectGBI0; gbi0Vertex = LumiverseGBI0VertexPD; geBaked = true; break;
  }
  //Conker's Bad Fur Day (U): "RSP Gfx ucode F3DEXBG.NoN fifo 2.08" — a
  //custom F3DEX2 build (round 8 census; the engine keeps the ucode resident
  //and dispatches with ucode_size 0). Tried as plain GBI2 behind an opt-in
  //until its dialect differences are validated (LUMIVERSE_ARES_N64_CONKER_GFX_HLE=1).
  case 0x63a15d2f6bdae1f5ull: {
    static const bool optIn = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_CONKER_GFX_HLE"); return v && v[0] == '1'; }();
    if(!optIn) return false;
    dialect = LumiverseDialectGBI2; break;
  }
  default: return false;
  }
  return true;
}

//----------------------------------------------------------------------------
//round 19: graphics task duration model
//----------------------------------------------------------------------------
//With graphics HLE a task completes at dispatch; the LLE microcode takes
//1-8 ms of RSP time per frame, and the Zeldas' audio thread schedules
//around that (round 7: Master Quest title-screen seam clicks 10/14 with
//instant completion vs 1/1 LLE, the deferred-completion knob at a flat
//4 ms halved them). Like the audio executors' models (rounds 14/16/18),
//an HLE graphics task now "runs" for a modelled RSP duration computed from
//its display list: a per-opcode cycle table + per-vertex + per-triangle
//costs + a constant, fitted by least squares on LLE task durations of our
//own emulator (LUMIVERSE_ARES_N64_RSP_HLE_GFX_COST_LOG=1 prints the
//per-task [rsp-hle-gfx-cost] lines the fit reads). The RDP stream is still
//queued at dispatch (the picture is identical); its SyncFull interrupt is
//held until the modelled completion. LUMIVERSE_ARES_N64_RSP_HLE_GFX_COST=0
//restores instant completion.
auto lumiverseGfxCostModel() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_COST"); return !v || v[0] != '0'; }();
  return value;
}
//LUMIVERSE_ARES_N64_RSP_HLE_GFX_COST_SCALE=<percent> (default 100): scales the
//modelled duration (A/B lever for the click gates)
auto lumiverseGfxCostScale() -> u32 {
  static const u32 value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_COST_SCALE"); return v ? (u32)::atoi(v) : 100u; }();
  return value;
}
struct LumiverseGfxCostTable {
  u32 constant;
  u32 perVertex;
  u32 perTri;
  u32 ops[256];
};
//fitted tables (RSP cycles); see the round-19 report for the fit
//fit (round 19, LLE runs of our own emulator, 4000 steps each, generic
//script, yielded segments summed): GBI2 = OoT + Master Quest + Majora +
//Smash (10,448 + 3,780 tasks, rms 5.3% of the mean 5.4 ms task, median
//1.9%); GBI1 = SF64 + Mario Kart 64 (3,680 tasks, rms 0.8% of 18.7 ms —
//Banjo-Kazooie excluded: its tasks stall on the DPC FREEZE its engine
//holds, 58 ms mean); GBI0 = SM64 (1,813 tasks, rms 13% of 32 ms). A flat
//per-command cost was 13.6% / 41% / 18%. Non-negative least squares, so
//collinear opcodes (VTX vs the triangle it feeds) land on one of them.
static const LumiverseGfxCostTable lumiverseGfxCostGBI2 = { 0, 0, 0, {
  0, 0, 0, 0, 0, 1708, 0, 889, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 2710, 1753, 0, 2023, 0, 0, 5715, 0, 0,
  244, 0, 0, 0, 516, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9801,
  0, 0, 0, 598, 121, 0, 0, 0, 0, 0, 0, 1100, 0, 0, 0, 0 } };
static const LumiverseGfxCostTable lumiverseGfxCostGBI1 = { 442, 1934, 1227, {
  0, 818, 0, 2249, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 548, 0, 0, 0, 2188, 0, 0, 0, 0, 139, 0, 0, 0, 0, 131,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 625, 0, 442, 0, 0, 0, 0, 0, 0,
  0, 0, 1893, 44, 0, 0, 0, 215, 0, 0, 0, 0, 1579, 0, 0, 0 } };
//GBI0 (Fast3D) per image: one table per microcode build — fitted together
//they disagree (16.8% rms; SM64 2.0D vs Pilotwings vs Wave Race vs SOTE vs
//Cruis'n USA builds have different per-command costs): SM64 1,813 tasks
//13%, Pilotwings 1,430 tasks 5.7% (76 ms mean!), Wave Race 12%, SOTE 14%,
//Cruis'n USA 6.2%. GoldenEye / Perfect Dark (baked-RDP Fast3D) stay
//instant: no fit (their audio gates were built on instant completion).
static const LumiverseGfxCostTable lumiverseGfxCostSM64 = { 0, 128, 762, {
  0, 3022, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 17413, 0, 0, 0, 0, 0, 62318, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 111946, 3762, 0, 0, 0, 0, 0 } };
static const LumiverseGfxCostTable lumiverseGfxCostPW64 = { 0, 919, 0, {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 5377, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 8190, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 51918, 8165, 0, 0, 0, 0, 0 } };
static const LumiverseGfxCostTable lumiverseGfxCostWR64 = { 0, 0, 0, {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 5320, 0, 0, 0, 0, 0, 0, 0, 55446, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 435, 495006, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 47704, 0, 95734, 0, 0, 0, 0, 6163, 0, 0, 0, 0 } };
static const LumiverseGfxCostTable lumiverseGfxCostSOTE = { 0, 0, 1157, {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 831, 2450, 0, 165, 0, 0, 0, 1157,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 597, 0, 0, 0, 14287, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1394, 2028, 0, 0, 224, 0, 0, 0, 0, 0 } };
static const LumiverseGfxCostTable lumiverseGfxCostCUSA = { 0, 0, 885, {
  0, 0, 0, 1082, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 3206, 0, 913, 0, 0, 0, 1033, 2168, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 4222, 0, 0, 0, 3511, 0, 0, 0, 853, 0, 1615, 0, 0, 273 } };
static const LumiverseGfxCostTable lumiverseGfxCostNone = { 0, 0, 0, {} };
//round 20: Conker's Bad Fur Day (F3DEXBG) — 5,252 LLE tasks of the intro
//+ first area (rms 7.8% of the 30.8 ms mean task, median 7.4%; the GBI2
//table put 279 of 918 logo-scene tasks off by more than 2x). The sixteen
//0x1x opcodes are one packed-triangle command and share a coefficient.
static const LumiverseGfxCostTable lumiverseGfxCostConker = [] {
  LumiverseGfxCostTable t = { 0, 0, 0, {} };
  t.ops[0x06] = 3087; for(u32 op = 0x10; op <= 0x1f; op++) t.ops[op] = 1954;
  t.ops[0xf0] = 3093; t.ops[0xfa] = 2978; t.ops[0xfc] = 14276; t.ops[0xfd] = 620;
  return t;
}();
auto lumiverseGfxCostTableFor(u64 ucodeHash, u32 dialect) -> const LumiverseGfxCostTable& {
  if(ucodeHash == 0x63a15d2f6bdae1f5ull) return lumiverseGfxCostConker;
  if(dialect == LumiverseDialectGBI2 || dialect == LumiverseDialectS2DEX2) return lumiverseGfxCostGBI2;
  if(dialect != LumiverseDialectGBI0) return lumiverseGfxCostGBI1;
  switch(ucodeHash) {
  case LumiverseUcodeFast3DSM64: return lumiverseGfxCostSM64;
  case LumiverseUcodeFast3DPW64: return lumiverseGfxCostPW64;
  case LumiverseUcodeFast3DWR64: return lumiverseGfxCostWR64;
  case LumiverseUcodeFast3DSOTE: return lumiverseGfxCostSOTE;
  case LumiverseUcodeFast3DCUSA: return lumiverseGfxCostCUSA;
  default: return lumiverseGfxCostNone;  //GoldenEye / Perfect Dark: instant
  }
}
auto lumiverseGfxModelledCycles(u64 ucodeHash, u32 dialect, const LumiverseGfxTaskFeatures& f) -> u32 {
  const LumiverseGfxCostTable& t = lumiverseGfxCostTableFor(ucodeHash, dialect);
  u64 cycles = t.constant + (u64)f.vertices * t.perVertex + (u64)f.tris * t.perTri;
  for(u32 op = 0; op < 256; op++) if(f.ops[op]) cycles += (u64)f.ops[op] * t.ops[op];
  cycles = cycles * lumiverseGfxCostScale() / 100;
  if(cycles > 20000000) cycles = 20000000;  //0.32 s: a runaway list never wedges the RSP
  return (u32)cycles;
}

//cost log: the LLE duration (RSP cycles from dispatch to BREAK) of every
//graphics task next to its display-list features — walked at dispatch by
//the same dry-run the HLE uses, so the features are exactly what the model
//sees. Level 1 (census, LLE execution) runs give the fit data.
auto lumiverseGfxCostLog() -> bool {
  static const bool value = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_COST_LOG"); return v && v[0] == '1'; }();
  return value;
}
//A yielded task (the OS asks the RSP for the audio task mid-frame — the
//Zeldas do it every frame: dispatch, yield ~100 us later, audio task, the
//graphics task resumed with OSTask flags bit 0 set) is ONE task: the
//segments' durations are summed and the line is printed at the final
//(non-yielded) BREAK.
static u64 lumiverseGfxTaskStartCycles = 0;
static u64 lumiverseGfxTaskAccumulated = 0;
static u32 lumiverseGfxTaskSegments = 0;
static u32 lumiverseGfxLoggedDataPtr = 0;
static LumiverseGfxTaskFeatures lumiverseGfxLoggedFeatures;
static u32 lumiverseGfxLoggedDialect = 0;
static u64 lumiverseGfxLoggedHash = 0;
static bool lumiverseGfxLoggedWalkable = false;
auto lumiverseGfxNoteTaskDispatch(const u32 task[16], u64 ucodeHash) -> void {
  const u32 dataPtr = task[12] & 0x00ffffff;
  const bool resumed = (task[1] & 1) && lumiverseGfxTaskSegments && dataPtr == lumiverseGfxLoggedDataPtr;
  lumiverseGfxTaskStartCycles = 0;
  if(resumed) {
    lumiverseGfxTaskSegments++;
    lumiverseGfxTaskStartCycles = rsp.profile.cycles ? rsp.profile.cycles : 1;
    return;
  }
  lumiverseGfxTaskAccumulated = 0;
  lumiverseGfxTaskSegments = 0;
  u32 dialect, gbi0Vertex; bool geBaked;
  if(!lumiverseGfxResolveDialect(ucodeHash, dialect, gbi0Vertex, geBaked)) return;
  if(!dataPtr) return;
  static LumiverseGfxCensus census;
  lumiverseGfxLoggedWalkable = dialect == LumiverseDialectGBI2
    ? lumiverseGfxDryRunGBI2(dataPtr, census)
    : lumiverseGfxDryRun(dataPtr, census, dialect, geBaked, gbi0Vertex);
  static const bool shadow = [] { const char* v = ::getenv("LUMIVERSE_ARES_N64_RSP_HLE_GFX_SHADOW"); return v && v[0] == '1'; }();
  if(shadow && lumiverseGfxLoggedWalkable && lumiverseHLELevel() < 2) {
    static LumiverseGfxMachine machine;
    machine.out.fifo = false;
    lumiverseGfxShadowDiscard = true;
    if(dialect == LumiverseDialectGBI2) lumiverseGfxExecuteTaskGBI2(machine, dataPtr);
    else lumiverseGfxExecuteTask(machine, dataPtr, dialect, gbi0Vertex, geBaked);
    lumiverseGfxShadowDiscard = false;
  }
  lumiverseGfxLoggedFeatures = lumiverseGfxTaskFeatures;
  lumiverseGfxLoggedDialect = dialect;
  lumiverseGfxLoggedHash = ucodeHash;
  lumiverseGfxLoggedDataPtr = dataPtr;
  lumiverseGfxTaskSegments = 1;
  lumiverseGfxTaskStartCycles = rsp.profile.cycles ? rsp.profile.cycles : 1;
}
auto lumiverseGfxNoteTaskEnd(u64 cycles, bool yielded) -> void {
  if(!lumiverseGfxTaskStartCycles) return;
  lumiverseGfxTaskAccumulated += cycles - lumiverseGfxTaskStartCycles;
  lumiverseGfxTaskStartCycles = 0;
  if(yielded) return;  //resumed later with OSTask flags bit 0; printed then
  const u64 d = lumiverseGfxTaskAccumulated;
  const u32 segments = lumiverseGfxTaskSegments;
  lumiverseGfxTaskAccumulated = 0;
  lumiverseGfxTaskSegments = 0;
  static u32 lines = 0;
  if(lines++ >= 60000) return;
  const auto& f = lumiverseGfxLoggedFeatures;
  fprintf(stderr, "[rsp-hle-gfx-cost] cycles=%llu dialect=%u walkable=%u segments=%u cmds=%u vtx=%u tri=%u model=%u ops",
    (unsigned long long)d, lumiverseGfxLoggedDialect, (u32)lumiverseGfxLoggedWalkable, segments, f.commands, f.vertices, f.tris,
    lumiverseGfxModelledCycles(lumiverseGfxLoggedHash, lumiverseGfxLoggedDialect, f));
  for(u32 op = 0; op < 256; op++) if(f.ops[op]) fprintf(stderr, " %02x:%u", op, f.ops[op]);
  fprintf(stderr, "\n");
}
#else
auto lumiverseGfxCostLog() -> bool { return false; }
auto lumiverseGfxNoteTaskDispatch(const u32 task[16], u64 ucodeHash) -> void { (void)task; (void)ucodeHash; }
auto lumiverseGfxNoteTaskEnd(u64 cycles, bool yielded) -> void { (void)cycles; (void)yielded; }
#endif

auto lumiverseExecuteGraphicsTask(const u32 task[16], u64 ucodeHash) -> bool {
#if defined(VULKAN)
  u32 dialect;
  u32 gbi0Vertex = LumiverseGBI0VertexStandard;
  bool geBaked = false;
  if(!lumiverseGfxResolveDialect(ucodeHash, dialect, gbi0Vertex, geBaked)) return false;
  if(!vulkan.enable) return false;

  static LumiverseGfxMachine machine;
  static LumiverseGfxCensus census;
  static u64 tasksExecuted = 0;
  static u64 tasksFallback = 0;

  const u32 dataPtr = task[12] & 0x00ffffff;
  if(!dataPtr) return false;

  const bool walkable = dialect == LumiverseDialectGBI2
    ? lumiverseGfxDryRunGBI2(dataPtr, census)
    : lumiverseGfxDryRun(dataPtr, census, dialect, geBaked, gbi0Vertex);
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
  machine.out.fifoPos = fifoStart;
  machine.out.fifoStarted = false;
  machine.out.pendingQwords = 0;
  if(machine.out.fifo) {
    rdp.writeWord(0x0c, 0x0001, rsp);  //DPC_STATUS: clear xbus -> RDRAM source
  }

  //round 19: modelled duration — the SyncFull of the stream queued below is
  //held until the task's completion (RSP::lumiverseHLEDeliverCompletion)
  const u32 modelledCycles = lumiverseGfxCostModel() && lumiverseHLERequestedCompletionCycles <= 0
    ? lumiverseGfxModelledCycles(ucodeHash, dialect, lumiverseGfxTaskFeatures) : 0;
  lumiverseDPInterruptPending = false;
  lumiverseDPInterruptDefer = modelledCycles > 0;
  const bool ok = dialect == LumiverseDialectGBI2
    ? lumiverseGfxExecuteTaskGBI2(machine, dataPtr)
    : lumiverseGfxExecuteTask(machine, dataPtr, dialect, gbi0Vertex, geBaked);
  lumiverseDPInterruptDefer = false;
  if(ok) {
    tasksExecuted++;
    if(modelledCycles > 0) lumiverseHLERequestedCompletionCycles = (s32)modelledCycles;
  } else {
    tasksFallback++;
    if(lumiverseDPInterruptPending) { lumiverseDPInterruptPending = false; mi.raise(MI::IRQ::DP); }
  }

  if(lumiverseGfxLogLevel() >= 1 && ((tasksExecuted + tasksFallback) & 63) == 1) {
    fprintf(stderr,
      "[rsp-hle-gfx] tasks=%llu fallback=%llu tris=%llu clipped=%llu rejected=%llu rejects(stack/b0/mw/mvtx/baked/bakedcut)=%llu/%llu/%llu/%llu/%llu/%llu fifoWrapsFrozen=%llu\n",
      (unsigned long long)tasksExecuted, (unsigned long long)tasksFallback,
      (unsigned long long)machine.trisEmitted, (unsigned long long)machine.trisClipped,
      (unsigned long long)machine.trisRejected,
      (unsigned long long)lumiverseGfxRejectCounts[0], (unsigned long long)lumiverseGfxRejectCounts[1],
      (unsigned long long)lumiverseGfxRejectCounts[2], (unsigned long long)lumiverseGfxRejectCounts[3],
      (unsigned long long)lumiverseGfxRejectCounts[4], (unsigned long long)lumiverseGfxRejectCounts[5],
      (unsigned long long)lumiverseGfxFifoWrapsFrozen);
  }

  return ok;
#else
  (void)task;
  (void)ucodeHash;
  return false;
#endif
}

}  //namespace
