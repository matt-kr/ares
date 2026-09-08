/* Copyright (c) 2020 Themaister
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <atomic>

#ifdef PARALLEL_RDP_SHADER_DIR
#include "global_managers.hpp"
#endif

namespace RDP
{
class CommandProcessor;
class CommandRing
{
public:
	void init(
#ifdef PARALLEL_RDP_SHADER_DIR
			Granite::Global::GlobalManagersHandle global_handles,
#endif
			CommandProcessor *processor, unsigned count);
	~CommandRing();
	void drain();

	void enqueue_command(unsigned num_words, const uint32_t *words);
	// LUMIVERSE: enqueue a run of commands under one lock acquisition and one
	// wake-up (the per-command lock + notify was 11% of the emulation thread
	// in Star Wars: Rogue Squadron missions). Same ring layout and ordering.
	struct BatchedCommand { unsigned num_words; const uint32_t *words; };
	void enqueue_commands(unsigned count, const BatchedCommand *commands);

private:
	CommandProcessor *processor = nullptr;
	std::thread thr;
	std::mutex lock;
	std::condition_variable cond;

	std::vector<uint32_t> ring;
	// LUMIVERSE (round 16): single-producer / single-consumer ring without a
	// mutex on the hot path. The producer (emulation thread) writes the words
	// and release-stores write_count; the consumer (this thread) acquire-loads
	// it, copies the run out, release-stores read_count; completed_count is
	// published after the run was processed. The mutex/condvar are only used
	// to park the consumer when the ring is empty (after a short spin) and to
	// block drain() / a full ring; the producer takes the lock only when the
	// consumer says it is parked. LUMIVERSE_ARES_N64_RDP_RING=0 restores the
	// upstream mutex-per-run ring (the round-13 batched form).
	std::atomic<uint64_t> write_count{0};
	std::atomic<uint64_t> read_count{0};
	std::atomic<uint64_t> completed_count{0};
	std::atomic<bool> consumer_parked{false};
	std::atomic<bool> drain_waiting{false};
	bool lockfree = true;
	void wait_for_space(unsigned words);
	void wake_consumer();

	void thread_loop();
	void teardown_thread();
#ifdef PARALLEL_RDP_SHADER_DIR
	Granite::Global::GlobalManagersHandle global_handles;
#endif
};
}
