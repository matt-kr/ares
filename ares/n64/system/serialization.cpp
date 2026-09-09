static const string SerializerVersion = "lumi-state-1";

auto System::serialize(bool synchronize) -> serializer {
  rsp.lumiverseAsyncDrain();
  #if defined(VULKAN)
  if(_vulkanNeedsLoad) { vulkan.load(node); _vulkanNeedsLoad = false; }
  vulkan.synchronizeState();
  #endif
  serializer s;

  u32  signature = SerializerSignature;
  char version[16] = {};
  char description[512] = {};
  memory::copy(&version, (const char*)SerializerVersion, SerializerVersion.size());

  s(signature);
  s(synchronize);
  s(version);
  s(description);
  bool gpu = false;
  #if defined(VULKAN)
  gpu = vulkan.enable;
  #endif
  s(gpu);
  auto banks = controllerPakBankCount;
  s(banks);

  serialize(s, synchronize);
  return s;
}

auto System::unserialize(serializer& s) -> bool {
  u32  signature = 0;
  bool synchronize = true;
  char version[16] = {};
  char description[512] = {};

  s(signature);
  s(synchronize);
  s(version);
  s(description);

  if(signature != SerializerSignature) return false;
  if(string{version} != SerializerVersion) return false;
  bool savedGPU = false;
  s(savedGPU);
  bool currentGPU = false;
  #if defined(VULKAN)
  currentGPU = vulkan.enable;
  #endif
  auto banks = controllerPakBankCount;
  s(banks);
  if(savedGPU != currentGPU || banks != controllerPakBankCount) return false;

  rsp.lumiverseAsyncDrain();
  #if defined(VULKAN)
  vulkan.synchronizeState();
  #endif
  if(synchronize) power(/* reset = */ false);
  #if defined(VULKAN)
  if(_vulkanNeedsLoad) { vulkan.load(node); _vulkanNeedsLoad = false; }
  #endif
  serialize(s, synchronize);
  return true;
}

auto System::serialize(serializer& s, bool synchronize) -> void {
  s(queue);
  s(cartridge);
  s(controllerPort1);
  s(controllerPort2);
  s(controllerPort3);
  s(controllerPort4);
  s(rdram);
  s(mi);
  s(vi);
  s(ai);
  s(pi);
  s(pif);
  s(cic);
  s(ri);
  s(si);
  s(cpu);
  s(rdp);
  s(rsp);
  s(dd);
  s(aleck64);
  #if defined(VULKAN)
  vulkan.serialize(s);
  #endif
}
