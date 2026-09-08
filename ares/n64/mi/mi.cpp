#include <n64/n64.hpp>

namespace ares::Nintendo64 {
auto lumiverseSpTraceOn() -> bool;  //rdp/io.cpp (round 16): LUMIVERSE_ARES_N64_SP_TRACE
auto lumiverseSpTrace(const char* who, const char* op, const char* reg, u32 value, u64 extra) -> void;

MI mi;
#include "io.cpp"
#include "debugger.cpp"
#include "serialization.cpp"

auto MI::load(Node::Object parent) -> void {
  node = parent->append<Node::Object>("MI");

  debugger.load(node);
}

auto MI::unload() -> void {
  node.reset();
  debugger = {};
}

auto MI::raise(IRQ source) -> void {
  //Lumiverse diagnostic: LUMIVERSE_ARES_N64_SP_IRQ_LOG=1 logs each SP
  //interrupt with the CPU timer, for serial-vs-async completion timing diffs
  if(source == IRQ::SP) {
    static int spIrqLog = [] {
      const char* value = ::getenv("LUMIVERSE_ARES_N64_SP_IRQ_LOG");
      return value ? ::atoi(value) : 0;
    }();
    if(spIrqLog) {
      static unsigned long long raises = 0;
      fprintf(stderr, "[sp-irq] %llu count=%llu\n",
        ++raises, (unsigned long long)(u64)cpu.scc.count);
    }
  }
  debugger.interrupt((u32)source);
  if(unlikely(lumiverseSpTraceOn())) lumiverseSpTrace("mi", "RAISE", source == IRQ::SP ? "SP" : source == IRQ::SI ? "SI" : source == IRQ::AI ? "AI" : source == IRQ::VI ? "VI" : source == IRQ::PI ? "PI" : "DP", 0, 0);
  switch(source) {
  case IRQ::SP: irq.sp.line = 1; break;
  case IRQ::SI: irq.si.line = 1; break;
  case IRQ::AI: irq.ai.line = 1; break;
  case IRQ::VI: irq.vi.line = 1; break;
  case IRQ::PI: irq.pi.line = 1; break;
  case IRQ::DP: irq.dp.line = 1; break;
  }
  poll();
}

auto MI::lower(IRQ source) -> void {
  if(unlikely(lumiverseSpTraceOn())) lumiverseSpTrace("mi", "LOWER", source == IRQ::SP ? "SP" : source == IRQ::SI ? "SI" : source == IRQ::AI ? "AI" : source == IRQ::VI ? "VI" : source == IRQ::PI ? "PI" : "DP", 0, 0);
  switch(source) {
  case IRQ::SP: irq.sp.line = 0; break;
  case IRQ::SI: irq.si.line = 0; break;
  case IRQ::AI: irq.ai.line = 0; break;
  case IRQ::VI: irq.vi.line = 0; break;
  case IRQ::PI: irq.pi.line = 0; break;
  case IRQ::DP: irq.dp.line = 0; break;
  }
  poll();
}

auto MI::poll() -> void {
  bool line = 0;
  line |= irq.sp.line & irq.sp.mask;
  line |= irq.si.line & irq.si.mask;
  line |= irq.ai.line & irq.ai.mask;
  line |= irq.vi.line & irq.vi.mask;
  line |= irq.pi.line & irq.pi.mask;
  line |= irq.dp.line & irq.dp.mask;
  cpu.setInterruptPending(CPU::Interrupt::RCP, line);
}

auto MI::power(bool reset) -> void {
  irq = {};
  io = {};
}

}
