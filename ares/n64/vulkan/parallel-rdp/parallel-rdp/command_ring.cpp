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

#include <chrono>
#include "command_ring.hpp"
#include "rdp_device.hpp"
#include "thread_id.hpp"
#include <assert.h>

namespace RDP
{
void CommandRing::init(
#ifdef PARALLEL_RDP_SHADER_DIR
		Granite::Global::GlobalManagersHandle global_handles_,
#endif
		CommandProcessor *processor_, unsigned count)
{
	assert((count & (count - 1)) == 0);
	teardown_thread();
	processor = processor_;
	ring.resize(count);
	write_count = 0;
	read_count = 0;
	completed_count = 0;
	consumer_parked = false;
	drain_waiting = false;
	{
		const char *v = ::getenv("LUMIVERSE_ARES_N64_RDP_RING");
		lockfree = !v || v[0] != '0';
	}
#ifdef PARALLEL_RDP_SHADER_DIR
	global_handles = std::move(global_handles_);
#endif
	thr = std::thread(&CommandRing::thread_loop, this);
}

void CommandRing::teardown_thread()
{
	if (thr.joinable())
	{
		enqueue_command(0, nullptr);
		thr.join();
	}
}

CommandRing::~CommandRing()
{
	teardown_thread();
}

void CommandRing::drain()
{
	std::unique_lock<std::mutex> holder{lock};
	if (lockfree)
	{
		drain_waiting.store(true, std::memory_order_seq_cst);
		cond.wait(holder, [this]() {
			return write_count.load(std::memory_order_acquire) == completed_count.load(std::memory_order_acquire);
		});
		drain_waiting.store(false, std::memory_order_seq_cst);
		return;
	}
	cond.wait(holder, [this]() {
		return write_count == completed_count;
	});
}

// LUMIVERSE: producer side of the lock-free ring — wait (spinning, then
// yielding) until num_words + 1 slots are free; the consumer frees space
// with a release-store of read_count after copying a run out
void CommandRing::wait_for_space(unsigned words)
{
	const uint64_t w = write_count.load(std::memory_order_relaxed);
	unsigned spins = 0;
	while (w + words + 1 > read_count.load(std::memory_order_acquire) + ring.size())
	{
		if (++spins > 64)
			std::this_thread::yield();
	}
}

// LUMIVERSE: after publishing, wake the consumer only if it parked itself
// (it sets consumer_parked under the lock and re-checks the ring before
// waiting, so a publish that lands between its check and its wait is seen
// either by the re-check or by this notify)
void CommandRing::wake_consumer()
{
	if (consumer_parked.load(std::memory_order_seq_cst))
	{
		std::lock_guard<std::mutex> holder{lock};
		cond.notify_one();
	}
}

void CommandRing::enqueue_command(unsigned num_words, const uint32_t *words)
{
	if (lockfree)
	{
		wait_for_space(num_words);
		size_t mask = ring.size() - 1;
		uint64_t w = write_count.load(std::memory_order_relaxed);
		ring[w++ & mask] = num_words;
		for (unsigned i = 0; i < num_words; i++)
			ring[w++ & mask] = words[i];
		write_count.store(w, std::memory_order_seq_cst);
		wake_consumer();
		return;
	}
	std::unique_lock<std::mutex> holder{lock};
	cond.wait(holder, [this, num_words]() {
		return write_count + num_words + 1 <= read_count + ring.size();
	});

	size_t mask = ring.size() - 1;
	ring[write_count++ & mask] = num_words;
	for (unsigned i = 0; i < num_words; i++)
		ring[write_count++ & mask] = words[i];

	cond.notify_one();
}

void CommandRing::enqueue_commands(unsigned count, const BatchedCommand *commands)
{
	if (!count)
		return;
	if (lockfree)
	{
		size_t mask = ring.size() - 1;
		uint64_t w = write_count.load(std::memory_order_relaxed);
		for (unsigned c = 0; c < count; c++)
		{
			unsigned num_words = commands[c].num_words;
			const uint32_t *words = commands[c].words;
			if (w + num_words + 1 > read_count.load(std::memory_order_acquire) + ring.size())
			{
				// publish what is queued so far so the consumer can make room
				write_count.store(w, std::memory_order_seq_cst);
				wake_consumer();
				wait_for_space(num_words);
			}
			ring[w++ & mask] = num_words;
			for (unsigned i = 0; i < num_words; i++)
				ring[w++ & mask] = words[i];
		}
		write_count.store(w, std::memory_order_seq_cst);
		wake_consumer();
		return;
	}
	std::unique_lock<std::mutex> holder{lock};
	size_t mask = ring.size() - 1;
	for (unsigned c = 0; c < count; c++)
	{
		unsigned num_words = commands[c].num_words;
		const uint32_t *words = commands[c].words;
		cond.wait(holder, [this, num_words]() {
			return write_count + num_words + 1 <= read_count + ring.size();
		});
		ring[write_count++ & mask] = num_words;
		for (unsigned i = 0; i < num_words; i++)
			ring[write_count++ & mask] = words[i];
	}
	cond.notify_one();
}

void CommandRing::thread_loop()
{
	Util::register_thread_index(0);

#ifdef PARALLEL_RDP_SHADER_DIR
	// Here to let the RDP play nice with full Granite.
	// When we move to standalone Granite, we won't need to interact with global subsystems like this.
	Granite::Global::set_thread_context(*global_handles);
	global_handles.reset();
#endif

	std::vector<uint32_t> tmp_buffer;
	tmp_buffer.reserve(64);
	// LUMIVERSE: drain up to a small run of commands per lock acquisition
	// (each command's words are copied contiguously; lengths kept aside)
	std::vector<uint32_t> batch_lengths;
	batch_lengths.reserve(64);
	size_t mask = ring.size() - 1;
	static const bool batch = [] { const char *v = ::getenv("LUMIVERSE_ARES_N64_RDP_BATCH"); return !v || v[0] != '0'; }();

	// LUMIVERSE: spin budget before parking (iterations of an acquire load).
	// Round 16, Star Wars: Rogue Squadron mission (quiet host, r16j): with a
	// 2000-iteration spin the consumer parks between the bursts of a display
	// list and nearly every producer run pays the park/notify mutex round
	// trip (564 of 7566 emulation-thread samples in std::mutex::lock, the
	// same as the upstream mutex ring; rdp-enqueue 131-139 ms/s, 63-72
	// steps/s); with 200000 (roughly 100-200 us) the consumer stays hot
	// through a display list and parks only between frames: rdp-enqueue
	// 33 ms/s, mission 77-84 steps/s (+20%), command stream byte-identical.
	// The ring size (RDP_RING_WORDS 4096 vs 65536) made no difference.
	static const unsigned spin_iterations = [] {
		const char *v = ::getenv("LUMIVERSE_ARES_N64_RDP_RING_SPIN");
		return v ? (unsigned)atoi(v) : 200000u;
	}();

	for (;;)
	{
		bool is_idle = false;
		batch_lengths.clear();
		if (lockfree)
		{
			// wait for data: spin, then park on the condvar with the idle timeout
			bool have = false;
			uint64_t r = read_count.load(std::memory_order_relaxed);
			for (unsigned i = 0; i < spin_iterations; i++)
			{
				if (write_count.load(std::memory_order_acquire) > r) { have = true; break; }
			}
			if (!have)
			{
				std::unique_lock<std::mutex> holder{lock};
				consumer_parked.store(true, std::memory_order_seq_cst);
				if (write_count.load(std::memory_order_seq_cst) > r)
					have = true;
				else
					have = cond.wait_for(holder, std::chrono::microseconds(500), [this, r]() { return write_count.load(std::memory_order_acquire) > r; });
				consumer_parked.store(false, std::memory_order_seq_cst);
			}
			if (have)
			{
				tmp_buffer.clear();
				const uint64_t w = write_count.load(std::memory_order_acquire);
				do
				{
					uint32_t num_words = ring[r++ & mask];
					size_t base = tmp_buffer.size();
					tmp_buffer.resize(base + num_words);
					for (uint32_t i = 0; i < num_words; i++)
						tmp_buffer[base + i] = ring[r++ & mask];
					batch_lengths.push_back(num_words);
					if (num_words == 0)
						break;
				} while (batch && w > r && batch_lengths.size() < 64 && tmp_buffer.size() < 1024);
				read_count.store(r, std::memory_order_release);
			}
			else
			{
				tmp_buffer.resize(1);
				tmp_buffer[0] = uint32_t(Op::MetaIdle) << 24;
				batch_lengths.push_back(1);
				is_idle = true;
			}
		}
		else
		{
			std::unique_lock<std::mutex> holder{lock};
			if (cond.wait_for(holder, std::chrono::microseconds(500), [this]() { return write_count > read_count; }))
			{
				tmp_buffer.clear();
				do
				{
					uint32_t num_words = ring[read_count++ & mask];
					size_t base = tmp_buffer.size();
					tmp_buffer.resize(base + num_words);
					for (uint32_t i = 0; i < num_words; i++)
						tmp_buffer[base + i] = ring[read_count++ & mask];
					batch_lengths.push_back(num_words);
					// a zero-length command is the teardown marker: stop the run there
					if (num_words == 0)
						break;
				} while (batch && write_count > read_count && batch_lengths.size() < 64 && tmp_buffer.size() < 1024);
			}
			else
			{
				// If we don't receive commands at a steady pace,
				// notify rendering thread that we should probably kick some work.
				tmp_buffer.resize(1);
				tmp_buffer[0] = uint32_t(Op::MetaIdle) << 24;
				batch_lengths.push_back(1);
				is_idle = true;
			}
		}

		bool teardown = false;
		size_t offset = 0;
		for (uint32_t num_words : batch_lengths)
		{
			if (num_words == 0)
			{
				teardown = true;
				break;
			}
			processor->enqueue_command_direct(num_words, tmp_buffer.data() + offset);
			offset += num_words;
		}
		if (!is_idle)
		{
			if (lockfree)
			{
				completed_count.store(read_count.load(std::memory_order_relaxed), std::memory_order_seq_cst);
				if (drain_waiting.load(std::memory_order_seq_cst))
				{
					std::lock_guard<std::mutex> holder{lock};
					cond.notify_all();
				}
			}
			else
			{
				std::lock_guard<std::mutex> holder{lock};
				completed_count.store(read_count.load(std::memory_order_relaxed), std::memory_order_relaxed);
				cond.notify_one();
			}
		}
		if (teardown)
			break;
	}
}
}
