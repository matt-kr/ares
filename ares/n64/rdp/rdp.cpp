#include <n64/n64.hpp>

namespace ares::Nintendo64 {

RDP rdp;
extern bool lumiverseDPInterruptDefer;    //io.cpp (round 19): hold SyncFull's interrupt during an HLE graphics task's modelled duration
extern bool lumiverseDPInterruptPending;
auto lumiverseGfxSyncHoldEnabled() -> bool;  //io.cpp (round 22)
#include "render.cpp"
#include "io.cpp"
#include "debugger.cpp"
#include "serialization.cpp"

auto RDP::load(Node::Object parent) -> void {
  node = parent->append<Node::Object>("RDP");
  debugger.load(node);
}

auto RDP::unload() -> void {
  debugger = {};
  node.reset();
}

auto RDP::crash(const char *reason) -> void {
  debug(unusual, "[RDP] software triggered a hardware bug; RDP crashed and will stop responding. Reason: ", reason);
  //Lumiverse (round 15): the notice above is invisible without a debugger;
  //a latched RDP crash is a permanent hang for a fifo/xbus microcode
  //(DPC_STATUS reads freeze + busy forever), so say so on stderr where the
  //smoke harness and the device console see it
  fprintf(stderr, "[rdp] RDP crashed (latched, will not respond again): %s\n", reason ? reason : "?");
  command.crashed = 1;
  //guard against asynchronous reporting of crash state. We want the RDP to report that it's busy forever
  command.pipeBusy = 1;
  command.bufferBusy = 1;
}

auto RDP::main() -> void {
  const u32 clocks = system.frequency();
  while(Thread::clock < 0) {
    step(clocks);
    command.clock += clocks / 3;
  }
}

auto RDP::power(bool reset) -> void {
  Thread::reset();
  command = {};
  edge = {};
  shade = {};
  texture = {};
  zbuffer = {};
  rectangle = {};
  other = {};
  fog = {};
  blend = {};
  primitive = {};
  environment = {};
  combine = {};
  tlut = {};
  load_ = {};
  tileSize = {};
  tile = {};
  set = {};
  primitiveDepth = {};
  scissor = {};
  convert = {};
  key = {};
  fillRectangle_ = {};
  io.bist = {};
  io.test = {};
}

}
