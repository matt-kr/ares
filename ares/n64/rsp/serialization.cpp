auto RSP::serialize(serializer& s) -> void {
  //Lumiverse addition: a save state must capture a quiescent RSP; wait for
  //any in-flight async audio task before touching core state
  lumiverseAsyncDrain();
  Thread::serialize(s);
  auto pod = [&](auto& value) {
    static_assert(std::is_trivially_copyable_v<std::remove_reference_t<decltype(value)>>);
    s(std::span<u8>((u8*)&value, sizeof(value)));
  };
  s(lumiverseHLEPendingCycles); s(lumiverseDPInterruptDelay);
  s(lumiverseHLERequestedCompletionCycles); s(lumiverseHLEPendingType);
  s(lumiverseHLELastDispatchType); s(lumiverseHLELastDispatchDataPtr);
  s(lumiverseHLEYieldedTaskDataPtr); s(lumiverseHLEYieldedRemaining); s(lumiverseHLEYieldedDataPtr);
  s(lumiverseDPInterruptDefer); s(lumiverseDPInterruptPending);
  pod(lumiverseAudioMachine); pod(lumiverseAudioStateSlots);
  pod(lumiverseAudioDeferred); s(lumiverseAudioDeferredCount);
  s(lumiverseAudioDeferredData); s(lumiverseAudioDeferredBytes); s(lumiverseAudioDeferredCommands);
  s(lumiverseAudioCmdCostPrefix); s(lumiverseAudioCmdCostCount);
  s(lumiverseAudioNaLastBlockLanes);
  #if defined(VULKAN)
  pod(lumiverseSavedGfxMachine);
  s(lumiverseGfxPacedActive); s(lumiverseGfxDrainAccum); s(lumiverseGfxDrainTicks);
  s(lumiverseGfxSpillWords); s(lumiverseGfxSpillCount); s(lumiverseGfxSpillOffset);
  s(lumiverseGfxStallCycles); s(lumiverseGfxDrainWordsPerTick); s(lumiverseGfxDrainAll);
  bool hasSpill = lumiverseGfxSpillOut != nullptr;
  s(hasSpill);
  if(s.reading()) {
    lumiverseGfxSpillOut = hasSpill ? &lumiverseSavedGfxMachine.out : nullptr;
    lumiverseGfxDraining = false;
    lumiverseGfxLateArmed = false;
  }
  #endif
  s(dmem);
  s(imem);

  s(pipeline.address);
  s(pipeline.instruction);
  s(pipeline.clocks);
  s(pipeline.singleIssue);
  for(auto& p : pipeline.previous) {
    s(p.load);
    s(p.rWrite);
    s(p.vWrite);
  }
  s(pipeline.current.store);
  s(pipeline.current.branch);
  s(pipeline.current.rRead);
  s(pipeline.current.vRead);

  s(dma.pending);
  s(dma.current);
  s(dma.busy.read);
  s(dma.busy.write);
  s(dma.full.read);
  s(dma.full.write);
  s(dma.clock);

  s(status.semaphore);
  s(status.halted);
  s(status.broken);
  s(status.full);
  s(status.singleStep);
  s(status.interruptOnBreak);
  s(status.signal);

  for(auto& r : ipu.r) s(r.u32);
  s(ipu.pc);

  s(branch.pc);
  s(branch.nextpc);
  s(branch.state);
  s(branch.nstate);

  for(auto& r : vpu.r) s(r);
  s(vpu.acch);
  s(vpu.accm);
  s(vpu.accl);
  s(vpu.vcoh);
  s(vpu.vcol);
  s(vpu.vcch);
  s(vpu.vccl);
  s(vpu.vce);
  s(vpu.divin);
  s(vpu.divout);
  s(vpu.divdp);

  if constexpr(Accuracy::RSP::Recompiler) {
    recompiler.reset();
  }
}

auto RSP::DMA::Regs::serialize(serializer& s) -> void {
  s(pbusRegion);
  s(pbusAddress);
  s(dramAddress);
  s(length);
  s(skip);
  s(count);
  s(originPc);
  s(originCpu);
}

auto RSP::r128::serialize(serializer& s) -> void {
  s(u128.lo);
  s(u128.hi);
}
