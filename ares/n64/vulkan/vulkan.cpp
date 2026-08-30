#include <n64/n64.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if defined(LUMIVERSE_ARES_STATIC_MOLTENVK)
#include <dlfcn.h>
#endif

namespace ares::Nintendo64 {

Vulkan vulkan;

struct LoggingInterface : Util::LoggingInterface {
  auto log(const char* tag, const char* fmt, va_list va) -> bool {
    char buffer[8192];
    vsnprintf(buffer, sizeof(buffer), fmt, va);
#if defined(LUMIVERSE_ARES_STATIC_MOLTENVK)
    fprintf(stderr, "[%s]: %s", tag, buffer);
#else
  //print(terminal::color::yellow(tag), buffer);
#endif
    return true;
  }
} loggingInterface;

namespace {
//LUMIVERSE: pipeline the SyncFull GPU drain by one sync instead of stalling
//the emulation thread until the GPU has fully finished the current frame.
//Read fresh (not cached) so switching engine profiles between ROM launches
//takes effect within the same process.
auto lumiverseDeferSyncFullWait() -> bool {
  const char* value = std::getenv("LUMIVERSE_ARES_N64_DEFER_SYNCFULL_WAIT");
  return !value || value[0] != '0';
}

//LUMIVERSE: drop a scanout instead of blocking the emulation thread when the
//reader thread is still busy with the previous one.
auto lumiverseNonBlockingScanout() -> bool {
  const char* value = std::getenv("LUMIVERSE_ARES_N64_NONBLOCKING_SCANOUT");
  return !value || value[0] != '0';
}

//LUMIVERSE: emulation-thread stall accounting, reported ~once per second to
//stderr so device runs show exactly where frame time goes.
struct LumiverseStallStats {
  u64 syncWaitNs = 0;
  u64 syncWaitMaxNs = 0;
  u32 syncCount = 0;
  u64 renderNs = 0;      //RDP command enqueue work on the emulation thread
  u64 scanoutNs = 0;     //scanoutAsync submissions on the emulation thread
  u64 windowStartNs = 0;

  static auto nowNs() -> u64 {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  auto recordRender(u64 ns) -> void { renderNs += ns; }
  auto recordScanout(u64 ns) -> void { scanoutNs += ns; }

  auto recordSyncWait(u64 ns) -> void {
    syncWaitNs += ns;
    if(ns > syncWaitMaxNs) syncWaitMaxNs = ns;
    syncCount += 1;

    const u64 now = nowNs();
    if(windowStartNs == 0) windowStartNs = now;
    if(now - windowStartNs >= 1000000000ull) {
      fprintf(stderr,
        "[ares-perf] sync %u/s wait %.1f ms/s (max %.2f) | rdp-enqueue %.1f ms/s | scanout %.1f ms/s\n",
        syncCount,
        syncWaitNs / 1e6,
        syncWaitMaxNs / 1e6,
        renderNs / 1e6,
        scanoutNs / 1e6);
      syncWaitNs = 0;
      syncWaitMaxNs = 0;
      syncCount = 0;
      renderNs = 0;
      scanoutNs = 0;
      windowStartNs = now;
    }
  }
};
LumiverseStallStats lumiverseStallStats;

//LUMIVERSE: pre-compile every VI scanout pipeline variant during the load
//phase. The VI stages are graphics pipelines with no async-compile fallback,
//so on MoltenVK each new variant otherwise costs a ~100ms synchronous Metal
//pipeline build on the emulation thread the first time a game exercises it
//(interlace, gamma, AA-mode changes mid-game).
auto lumiverseVIWarmupEnabled() -> bool {
  const char* value = std::getenv("LUMIVERSE_ARES_VI_WARMUP");
  return !value || value[0] != '0';
}

//LUMIVERSE: how many SyncFulls of GPU latency the emulation thread tolerates
//before blocking. 1 = wait for the previous sync; 2-4 absorb the GPU
//scheduling latency of sharing the device with the RealityKit compositor.
auto lumiverseSyncFullDepth() -> u32 {
  const char* value = std::getenv("LUMIVERSE_ARES_N64_SYNCFULL_DEPTH");
  if(!value) return 1;
  long parsed = std::strtol(value, nullptr, 10);
  if(parsed < 1) return 1;
  if(parsed > 4) return 4;
  return static_cast<u32>(parsed);
}
}

struct Vulkan::Implementation {
  Implementation(u8* data, u32 size);
  ~Implementation();

  ::Vulkan::Context context;
  ::Vulkan::Device device;
  ::RDP::CommandProcessor* processor = nullptr;
  bool deviceReady = false;
  atomic<const char*> crash_error = nullptr;

  struct Validation : public ::RDP::ValidationInterface {
    Implementation& self;
    Validation(Implementation& i) : self(i) {}
    void report_rdp_crash(::RDP::ValidationError err, const char *msg) override {
      self.crash_error = msg;
    }
  } validator{*this};

  //commands are u64 words, but the backend uses u32 swapped words.
  //size and offset are in u64 words.
  u32 buffer[0x10000] = {};
  u32 queueSize = 0;
  u32 queueOffset = 0;

  ::RDP::VIScanoutBuffer scanout;
  //LUMIVERSE: last VI register values the game wrote, so the pipeline
  //warm-up sweep can restore the real config after its synthetic one.
  u32 viShadow[unsigned(::RDP::VIRegister::Count)] = {};
  bool viShadowSet[unsigned(::RDP::VIRegister::Count)] = {};
  bool viWarmupDone = false;
  std::mutex lock;
  std::condition_variable condition;
  u64 pendingSyncFullTimelines[4] = {};
  u32 pendingSyncFullIndex = 0;
  u32 scanoutCount = 0;
  u32 endCount = 0;

  //LUMIVERSE: (re)create only the RDP command processor for a ROM load; the
  //VkInstance/VkDevice live for the process. visionOS eventually denies GPU
  //submissions from re-created Metal devices as "background execution"
  //(IOGPUMetalError → VK_ERROR_DEVICE_LOST → black screen), and device-level
  //shader caches die with the device.
  void createProcessor(u8* data, u32 size);
  void destroyProcessor();
};

auto Vulkan::load(Node::Object) -> bool {
  if (vulkan.enable) {
    Util::set_thread_logging_interface(&loggingInterface);
    if(implementation) {
      implementation->createProcessor(rdram.ram.data, rdram.ram.size);
    } else {
      implementation = new Vulkan::Implementation(rdram.ram.data, rdram.ram.size);
      if(!implementation->deviceReady) {
        //instance/device never came up; safe to throw the shell away
        delete implementation;
        implementation = nullptr;
      }
    }

    if (!implementation || !implementation->processor) {
      platform->status("Vulkan init failed: No RDP rendering support");
      vulkan.enable = false;
    } else {
      platform->status("Vulkan Enabled: using paraLLEl-RDP");
    }
  } else {
    platform->status("Vulkan Disabled: No RDP rendering support");
  }

  return true;
}

auto Vulkan::unload() -> void {
  //LUMIVERSE: keep the instance/device alive for the process lifetime; only
  //the processor (which references the current RDRAM allocation) goes away.
  if (implementation) implementation->destroyProcessor();
}

//RDP command lengths in 64-bit words, indexed by opcode (op >> 24 & 63).
static constexpr u32 lumiverseRDPCommandLength[64] = {
  1, 1, 1, 1, 1, 1, 1, 1, 4, 6,12,14,12,14,20,22,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

//Lumiverse refactor: the dispatch loop shared by render() and the HLE path.
//Returns false when a partial command remains queued for the next call.
auto Vulkan::processQueuedCommands() -> bool {
  u32* buffer = implementation->buffer;
  u32& queueSize = implementation->queueSize;
  u32& queueOffset = implementation->queueOffset;

  while(queueOffset < queueSize) {
    u32 op = buffer[queueOffset * 2];
    u32 code = op >> 24 & 63;
    u32 length = lumiverseRDPCommandLength[code];

    if(queueOffset + length > queueSize) {
      //partial command, keep data around for next processing call
      return false;
    }

    if(code >= 8) {
      implementation->processor->enqueue_command(length * 2, buffer + queueOffset * 2);
    }

    if(::RDP::Op(code) == ::RDP::Op::SyncFull) {
      u64 timeline = implementation->processor->signal_timeline();
      const u64 waitStart = LumiverseStallStats::nowNs();
      if(lumiverseDeferSyncFullWait()) {
        //LUMIVERSE: wait for the SyncFull from `depth` frames ago instead of
        //this one, so emulation overlaps with the GPU finishing this frame.
        //Requires PARALLEL_RDP_ALLOW_EXTERNAL_HOST=1 for correctness: with
        //staged RDRAM uploads the game can overwrite memory the GPU hasn't
        //consumed yet.
        const u32 depth = lumiverseSyncFullDepth();
        const u32 slot = implementation->pendingSyncFullIndex % depth;
        if(implementation->pendingSyncFullTimelines[slot]) {
          implementation->processor->wait_for_timeline(implementation->pendingSyncFullTimelines[slot]);
        }
        implementation->pendingSyncFullTimelines[slot] = timeline;
        implementation->pendingSyncFullIndex++;
      } else {
        implementation->processor->wait_for_timeline(timeline);
      }
      lumiverseStallStats.recordSyncWait(LumiverseStallStats::nowNs() - waitStart);
      rdp.syncFull();
    }

    queueOffset += length;
  }

  queueOffset = 0;
  queueSize = 0;
  return true;
}

//Lumiverse addition: HLE-generated RDP command stream entry point. `data`
//holds complete commands as pairs of words in host byte order (same layout
//the render() loop produces after its RDRAM/DMEM reads). Returns false if
//the stream could not be queued.
auto Vulkan::queueHLECommands(const u32* data, u32 wordCount) -> bool {
  if(!implementation || !implementation->processor) return false;
  if(wordCount == 0 || (wordCount & 1)) return false;

  u32 pairs = wordCount / 2;
  u32& queueSize = implementation->queueSize;
  if(queueSize + pairs >= 0x8000) {
    //queue full: process what is pending first, then retry once
    if(!processQueuedCommands()) return false;
    if(queueSize + pairs >= 0x8000) return false;
  }

  memcpy(implementation->buffer + queueSize * 2, data, wordCount * sizeof(u32));
  queueSize += pairs;
  return processQueuedCommands();
}

auto Vulkan::render() -> bool {
  if(!implementation || !implementation->processor) return false;
  struct RenderTimer {
    u64 start = LumiverseStallStats::nowNs();
    ~RenderTimer() { lumiverseStallStats.recordRender(LumiverseStallStats::nowNs() - start); }
  } renderTimer;

  auto& command = rdp.command;

  u32 current = command.current & ~7;
  u32 end = command.end & ~7;
  u32 length = (end - current) / 8;
  if(current >= end) return true;

  u32* buffer = implementation->buffer;
  u32& queueSize = implementation->queueSize;
  u32& queueOffset = implementation->queueOffset;
  if(queueSize + length >= 0x8000) return true;

  if(!command.source) {
    do {
      buffer[queueSize * 2 + 0] = rdram.ram.read<Word>(current, RBusDevice::DP_DMA); current += 4;
      buffer[queueSize * 2 + 1] = rdram.ram.read<Word>(current, RBusDevice::DP_DMA); current += 4;
      queueSize++;
    } while(--length);
  } else {
    do {
      buffer[queueSize * 2 + 0] = rsp.dmem.read<Word>(current); current += 4;
      buffer[queueSize * 2 + 1] = rsp.dmem.read<Word>(current); current += 4;
      if(system.homebrewMode) {
        rsp.debugger.dmemReadWord(current - 8, 8, "RDP XBUS");
      }
      queueSize++;
    } while(--length);
  }

  if(!processQueuedCommands()) {
    //partial command, keep data around for next processing call
    command.start = command.current = command.end;
    return true;
  }

  command.current = command.end;
  return true;
}

auto Vulkan::frame() -> void {
  if(!implementation || !implementation->processor) return;
  implementation->processor->begin_frame_context();
}

auto Vulkan::writeWord(u32 address, u32 data) -> void {
  if(!implementation || !implementation->processor) return;
  if(address < unsigned(::RDP::VIRegister::Count)) {
    implementation->viShadow[address] = data;
    implementation->viShadowSet[address] = true;
  }
  implementation->processor->set_vi_register(::RDP::VIRegister(address), data);
}

auto Vulkan::warmupVIPipelines() -> void {
  if(!implementation || !implementation->processor) return;
  if(implementation->viWarmupDone) return;
  implementation->viWarmupDone = true;
  if(!lumiverseVIWarmupEnabled()) return;

  using Reg = ::RDP::VIRegister;
  auto* processor = implementation->processor;
  const u64 startNs = LumiverseStallStats::nowNs();

  //identical option derivation to scanoutAsync, so pipeline keys match
  ::RDP::ScanoutOptions options;
  options.downscale_steps = supersampleScanout ? 16 : 0;
  options.persist_frame_on_invalid_input = true;
  if(disableVideoInterfaceProcessing) {
    options.vi = {false, false, true, false, false, false};
  }
  if(!supersampleScanout) {
    options.blend_previous_frame = weaveDeinterlacing;
    options.upscale_deinterlacing = !weaveDeinterlacing;
  } else {
    options.blend_previous_frame = false;
    options.upscale_deinterlacing = true;
  }

  const auto set = [&](Reg reg, u32 value) {
    processor->set_vi_register(reg, value);
  };

  //synthetic NTSC 320x240 config; Control/XScale/YScale/VCurrentLine vary
  set(Reg::Origin, 0x00100000);
  set(Reg::Width, 320);
  set(Reg::Intr, 2);
  set(Reg::Timing, 0x03e52239);
  set(Reg::VSync, 525);
  set(Reg::HSync, 0x00000c15);
  set(Reg::Leap, 0x0c150c15);
  set(Reg::HStart, ::RDP::make_vi_start_register(108, 108 + 640));
  set(Reg::VStart, ::RDP::make_vi_start_register(37, 37 + 474));
  set(Reg::VBurst, 0x000e0204);

  //the sweep axes cover every specialization-constant and shader-variant
  //combination the VI stages key on:
  //  type x aaMode      -> extract_vram spec (TYPE_MASK | META_AA)
  //  aaMode x dither    -> vi_fetch spec (META_AA | DITHER_FILTER)
  //  gammaSel x aaMode  -> vi_scale spec (GAMMA | GAMMA_DITHER | META_SCALE | META_AA)
  //  yBug               -> fetch-bug program variants + vi_scale spec 2
  //  serrate (2 fields) -> vi_blend_fields / vi_deinterlace paths
  //divot filter stays on throughout: it never enters a spec constant, and
  //having the stage run compiles both divot program variants via the yBug axis.
  constexpr u32 aaModes[] = {0u << 8, 2u << 8, 3u << 8};  //META_AA+SCALE, SCALE, neither
  constexpr u32 gammaSels[] = {
    0,
    ::RDP::VI_CONTROL_GAMMA_ENABLE_BIT,
    ::RDP::VI_CONTROL_GAMMA_ENABLE_BIT | ::RDP::VI_CONTROL_GAMMA_DITHER_ENABLE_BIT,
  };

  constexpr u32 bufferCount = 8;
  ::RDP::VIScanoutBuffer buffers[bufferCount];
  u32 submitted = 0;

  const auto sweepScanout = [&](bool field) {
    set(Reg::VCurrentLine, field);
    auto& target = buffers[submitted % bufferCount];
    if(target.fence) target.fence->wait();
    processor->scanout_async_buffer(target, options);
    submitted++;
    if((submitted % 16) == 0) processor->begin_frame_context();
  };

  for(u32 type = 0; type < 2; type++)
  for(u32 aa = 0; aa < 3; aa++)
  for(u32 dither = 0; dither < 2; dither++)
  for(u32 gamma = 0; gamma < 3; gamma++)
  for(u32 yBug = 0; yBug < 2; yBug++)
  for(u32 serrate = 0; serrate < 2; serrate++) {
    const u32 control =
        (type ? ::RDP::VI_CONTROL_TYPE_RGBA8888_BIT : ::RDP::VI_CONTROL_TYPE_RGBA5551_BIT)
      | aaModes[aa]
      | (dither ? ::RDP::VI_CONTROL_DITHER_FILTER_ENABLE_BIT : 0u)
      | gammaSels[gamma]
      | ::RDP::VI_CONTROL_DIVOT_ENABLE_BIT
      | (serrate ? ::RDP::VI_CONTROL_SERRATE_BIT : 0u);
    set(Reg::Control, control);
    set(Reg::XScale, ::RDP::make_vi_scale_register(0x200, 0));
    set(Reg::YScale, ::RDP::make_vi_scale_register(yBug ? 0x200 : 0x400, 0));
    sweepScanout(false);
    if(serrate) sweepScanout(true);  //second field: blend/deinterlace paths
  }

  //two blank scanouts drop the sweep's garbage prev-frame image so the real
  //first frame never blends against it
  set(Reg::Control, 0);
  sweepScanout(false);
  sweepScanout(false);

  for(auto& buffer : buffers) {
    if(buffer.fence) buffer.fence->wait();
  }
  processor->begin_frame_context();

  //restore every register the game has written so far
  for(u32 index = 0; index < unsigned(Reg::Count); index++) {
    if(implementation->viShadowSet[index]) {
      processor->set_vi_register(Reg(index), implementation->viShadow[index]);
    }
  }

  fprintf(stderr, "[ares] VI pipeline warm-up: %u scanouts in %.0f ms\n",
    submitted, (LumiverseStallStats::nowNs() - startNs) / 1e6);
}

auto Vulkan::scanoutAsync(bool field) -> bool {
  if(!implementation || !implementation->processor) return false;
  warmupVIPipelines();
  struct ScanoutTimer {
    u64 start = LumiverseStallStats::nowNs();
    ~ScanoutTimer() { lumiverseStallStats.recordScanout(LumiverseStallStats::nowNs() - start); }
  } scanoutTimer;

  { //LUMIVERSE: if the reader thread is still consuming the previous scanout,
    //drop this frame's scanout instead of stalling the emulation thread
    //(unless the strict profile asked for the original blocking behavior).
    std::unique_lock<std::mutex> lock{implementation->lock};
    if(lumiverseNonBlockingScanout()) {
      if(implementation->scanoutCount != implementation->endCount) return false;
    } else {
      implementation->condition.wait(lock, [this]() {
        return implementation->scanoutCount == implementation->endCount;
      });
    }
  }

  implementation->processor->set_vi_register(::RDP::VIRegister::VCurrentLine, field);

  //0 steps if scanning out at upscaled resolution.
  //each downscale step reduces output resolution to [width, height] * max(1, upscale >> downscale_steps)
  ::RDP::ScanoutOptions options;
  options.downscale_steps = supersampleScanout ? 16 : 0;
  options.persist_frame_on_invalid_input = true;  //this is a compatibility hack, but I'm not sure what for ...
  if(disableVideoInterfaceProcessing) {
    options.vi = {false, false, true, false, false, false};
  }
  if(!supersampleScanout){
    options.blend_previous_frame = weaveDeinterlacing;
    options.upscale_deinterlacing = !weaveDeinterlacing;
  }
  else {
    options.blend_previous_frame = false;
    options.upscale_deinterlacing = true;
  }


  if(implementation->scanout.fence) {
    implementation->scanout.fence->wait();
  }
  implementation->processor->scanout_async_buffer(implementation->scanout, options);
  implementation->scanoutCount++;
  return true;
}

auto Vulkan::mapScanoutRead(const u8*& rgba, u32& width, u32& height) -> void {
  if(!implementation || !implementation->scanout.fence || !implementation->scanout.width || !implementation->scanout.height) {
    rgba = nullptr;
    width = 0;
    height = 0;
  } else {
    implementation->scanout.fence->wait();
    rgba = (const u8*)implementation->device.map_host_buffer(*implementation->scanout.buffer, ::Vulkan::MEMORY_ACCESS_READ_BIT);
    width = implementation->scanout.width;
    height = implementation->scanout.height;
  }
}

auto Vulkan::unmapScanoutRead() -> void {
  if(implementation && implementation->scanout.buffer) {
    implementation->device.unmap_host_buffer(*implementation->scanout.buffer, ::Vulkan::MEMORY_ACCESS_READ_BIT);
  }
}

auto Vulkan::endScanout() -> void {
  if(implementation) {
    //notify main thread that we're done reading
    std::lock_guard<std::mutex> lock{implementation->lock};
    implementation->endCount++;
    implementation->condition.notify_one();
  }
}

auto Vulkan::crashed() -> const char* {
  if(implementation) return implementation->crash_error;
  return nullptr;
}

Vulkan::Implementation::Implementation(u8* data, u32 size) {
#if defined(LUMIVERSE_ARES_STATIC_MOLTENVK)
  auto getInstanceProcAddr = reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(RTLD_DEFAULT, "vkGetInstanceProcAddr"));
  if(!::Vulkan::Context::init_loader(getInstanceProcAddr)) {
    platform->status("Vulkan loader init failed");
    return;
  }
#else
  if(!::Vulkan::Context::init_loader(nullptr)) {
    platform->status("Vulkan loader init failed");
    return;
  }
#endif
  if(!context.init_instance_and_device(nullptr, 0, nullptr, 0, 0)) {
    platform->status("Vulkan instance/device init failed");
    return;
  }
  device.set_context(context);
  device.init_frame_contexts(3);
  deviceReady = true;

  //Persistent VkPipelineCache (Lumiverse): MoltenVK stores converted Metal
  //shaders in the cache blob, so reloading it removes the 100-140 ms
  //"Stalled compile" mid-game hitches from the second session onward.
  if(const char* cachePath = ::getenv("LUMIVERSE_ARES_PIPELINE_CACHE")) {
    std::vector<uint8_t> blob;
    if(FILE* cacheFile = fopen(cachePath, "rb")) {
      fseek(cacheFile, 0, SEEK_END);
      long cacheSize = ftell(cacheFile);
      fseek(cacheFile, 0, SEEK_SET);
      if(cacheSize > 0) {
        blob.resize((size_t)cacheSize);
        if(fread(blob.data(), 1, blob.size(), cacheFile) != blob.size()) blob.clear();
      }
      fclose(cacheFile);
    }
    //init_pipeline_cache validates the header (device UUID + hash) and
    //falls back to an empty cache when the blob doesn't match this GPU.
    if(!device.init_pipeline_cache(blob.empty() ? nullptr : blob.data(), blob.size())) {
      fprintf(stderr, "[ares] pipeline cache init failed (continuing without)\n");
    } else {
      fprintf(stderr, "[ares] pipeline cache loaded: %zu bytes from %s\n", blob.size(), cachePath);
    }
  }

  createProcessor(data, size);
}

void Vulkan::Implementation::createProcessor(u8* data, u32 size) {
  destroyProcessor();

  ::RDP::CommandProcessorFlags flags = 0;
  switch(vulkan.internalUpscale) {
  case 2: flags |= ::RDP::COMMAND_PROCESSOR_FLAG_UPSCALING_2X_BIT; break;
  case 4: flags |= ::RDP::COMMAND_PROCESSOR_FLAG_UPSCALING_4X_BIT; break;
  case 8: flags |= ::RDP::COMMAND_PROCESSOR_FLAG_UPSCALING_8X_BIT; break;
  }

  if(vulkan.internalUpscale > 1) {
    flags |= ::RDP::COMMAND_PROCESSOR_FLAG_SUPER_SAMPLED_DITHER_BIT;
    //rasky: this is explicitly disabled because we want to make sure we don't
    // read back the super sampled version, as it can cause artifacts. We want
    // parallelRDP to also produce a 1x render to use for readbacks.
    //flags |= ::RDP::COMMAND_PROCESSOR_FLAG_SUPER_SAMPLED_READ_BACK_BIT;
  }

  processor = new ::RDP::CommandProcessor(device, data, 0, size, size / 2, flags);
  if(!processor->device_is_supported()) {
    platform->status("Vulkan RDP device support check failed");
    delete processor;
    processor = nullptr;
    return;
  }

  processor->set_validation_interface(&validator);
}

void Vulkan::Implementation::destroyProcessor() {
  //~CommandProcessor drains the GPU (idle()), so the RDRAM pointer it holds
  //stays valid for its whole lifetime. Reset per-ROM state alongside it.
  if(processor) {
    delete processor;
    processor = nullptr;

    //Persist the pipeline cache so shaders compiled this session skip the
    //Metal conversion next time (see the load in initializeDevice).
    if(const char* cachePath = ::getenv("LUMIVERSE_ARES_PIPELINE_CACHE")) {
      size_t cacheSize = device.get_pipeline_cache_size();
      if(cacheSize > 0) {
        std::vector<uint8_t> blob(cacheSize);
        if(device.get_pipeline_cache_data(blob.data(), blob.size())) {
          if(FILE* cacheFile = fopen(cachePath, "wb")) {
            fwrite(blob.data(), 1, blob.size(), cacheFile);
            fclose(cacheFile);
            fprintf(stderr, "[ares] pipeline cache saved: %zu bytes\n", blob.size());
          }
        }
      }
    }
  }
  scanout = {};
  for(auto& timeline : pendingSyncFullTimelines) timeline = 0;
  pendingSyncFullIndex = 0;
  scanoutCount = 0;
  endCount = 0;
  queueSize = 0;
  queueOffset = 0;
}

Vulkan::Implementation::~Implementation() {
  destroyProcessor();
}

}
