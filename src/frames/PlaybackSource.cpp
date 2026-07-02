#include "PlaybackSource.h"

#include <algorithm>

namespace {

// Playback frames decoded ahead of the render thread, per eye. Bounds the
// prefetcher's memory to ~(depth + 1) full-frame RGB buffers per eye.
constexpr int kPrefetchDepth = 4;

// Recently shown/decoded frames kept for instant back-and-forth stepping.
// Full-res RGB frames are large (several MB each), so the depth stays small;
// 16 comfortably covers the +-10 shift-step range of the transport.
constexpr int kSeekCacheDepth = 16;

} // namespace

//--------------------------------------------------------------
void PlaybackSource::FramePrefetcher::start(const std::vector<std::string> * files,
	bool loop, int depth, const std::string & tag, std::size_t startCursor) {
	playlist = files;
	loopPlayback = loop;
	name = tag;
	cursor = (playlist && startCursor < playlist->size()) ? startCursor : 0;
	for (int i = 0; i < std::max(1, depth); ++i) {
		empty.send(ofPixels());
	}
	startThread();
}

//--------------------------------------------------------------
void PlaybackSource::FramePrefetcher::stop() {
	stopping = true;
	empty.close();
	filled.close();
	if (isThreadRunning()) {
		waitForThread(false);
	}
}

//--------------------------------------------------------------
bool PlaybackSource::FramePrefetcher::tryGet(ofPixels & out, bool & stale) {
	Frame f;
	if (!filled.tryReceive(f)) {
		return false;
	}
	std::uint64_t current;
	{
		std::lock_guard<std::mutex> lk(cursorMutex);
		current = generation;
	}
	stale = (f.generation != current);
	out = std::move(f.pixels);
	return true;
}

//--------------------------------------------------------------
void PlaybackSource::FramePrefetcher::recycle(ofPixels && buf) {
	empty.send(std::move(buf));
}

//--------------------------------------------------------------
void PlaybackSource::FramePrefetcher::reseek(std::size_t newCursor) {
	{
		std::lock_guard<std::mutex> lk(cursorMutex);
		cursor = newCursor;
		++generation; // anything decoded before this point is now stale.
	}
	// Recycle frames already queued from the old position so their buffers go
	// straight back to the pool. A frame the decoder is producing right now
	// still lands in `filled` afterwards; its stale generation tag makes the
	// consumer recycle it on arrival.
	Frame f;
	while (filled.tryReceive(f)) {
		empty.send(std::move(f.pixels));
	}
}

//--------------------------------------------------------------
void PlaybackSource::FramePrefetcher::threadedFunction() {
	ofPixels buf;
	bool haveBuf = false;
	while (true) {
		if (!haveBuf) {
			if (!empty.receive(buf)) {
				break; // pool closed by stop().
			}
			haveBuf = true;
		}
		if (!playlist || playlist->empty()) {
			break;
		}

		std::size_t idx = 0;
		std::uint64_t gen = 0;
		bool exhausted = false;
		{
			std::lock_guard<std::mutex> lk(cursorMutex);
			if (cursor >= playlist->size()) {
				if (loopPlayback) {
					cursor = 0;
				} else {
					exhausted = true;
				}
			}
			if (!exhausted) {
				idx = cursor++;
				gen = generation;
			}
		}
		if (exhausted) {
			// Non-looping list ran out: the main thread holds the last frame.
			// Stay alive (idle) so a later reseek() can reposition us.
			if (stopping) {
				break;
			}
			ofSleepMillis(5);
			continue;
		}

		const std::string & path = (*playlist)[idx];
		if (ofLoadImage(buf, path)) {
			// Match the live camera's RGB output for an identical pipeline.
			if (buf.getNumChannels() != 3) {
				buf.setNumChannels(3);
			}
			Frame f;
			f.pixels = std::move(buf);
			f.generation = gen;
			if (!filled.send(std::move(f))) {
				break; // queue closed by stop().
			}
			haveBuf = false; // buf was moved out; fetch a fresh one.
		} else {
			ofLogWarning("PlaybackSource") << name
				<< ": failed to load playback frame " << path;
			// keep the buffer and try the next frame.
		}
	}
}

//--------------------------------------------------------------
PlaybackSource::PlaybackSource(Config cfg)
	: config(std::move(cfg)) {
	if (!config.files.empty()) {
		prefetcher = std::make_unique<FramePrefetcher>();
		prefetcher->start(&config.files, config.loop, kPrefetchDepth, config.name, 0);
	}
}

//--------------------------------------------------------------
PlaybackSource::~PlaybackSource() {
	// Join the decoder before `config.files` (its playlist) is destroyed.
	prefetcher.reset();
}

//--------------------------------------------------------------
bool PlaybackSource::stage() {
	// Clears the per-tick "new frame" flag and pulls the next decoded frame
	// into the holding slot. Frames decoded before the latest reseek carry a
	// stale generation tag; their buffers are recycled and we ask again.
	frameNew = false;
	if (!prefetcher) {
		return false;
	}
	while (!hasPending) {
		ofPixels px;
		bool stale = false;
		if (!prefetcher->tryGet(px, stale)) {
			break;
		}
		if (stale) {
			prefetcher->recycle(std::move(px));
			continue;
		}
		pendingPixels = std::move(px);
		hasPending = true;
	}
	return hasPending;
}

//--------------------------------------------------------------
void PlaybackSource::commit() {
	if (!hasPending || !prefetcher) {
		return;
	}

	ofPixels next = std::move(pendingPixels);
	hasPending = false;

	// Keep the staged buffer and return the previous one to the pool so the
	// decoder can reuse its allocation.
	ofPixels previous = std::move(currentPixels);

	// The prefetcher delivers frames in playlist order, so the displayed index
	// simply advances (wrapping with loop playback) in lockstep with it.
	const int count = frameCount();
	const int nextIndex = count > 0 ? (currentIndex + 1) % count : -1;
	setCurrentFrame(std::move(next), nextIndex);
	prefetcher->recycle(std::move(previous));

	cacheInsert(currentIndex, currentPixels);
}

//--------------------------------------------------------------
void PlaybackSource::seek(int index) {
	const int count = frameCount();
	if (count <= 0) {
		return;
	}
	index = ((index % count) + count) % count; // wrap into [0, count)

	// Cache hit: instant. Miss: synchronous decode on the calling (main)
	// thread - the transport's step/scrub path is driven by discrete key
	// presses, so the one-off decode cost is acceptable and keeps stepping
	// frame-accurate in both directions.
	ofPixels px;
	if (!cacheLookup(index, px)) {
		if (!ofLoadImage(px, config.files[index])) {
			ofLogWarning("PlaybackSource") << config.name
				<< ": failed to load playback frame " << config.files[index];
			return;
		}
		if (px.getNumChannels() != 3) {
			px.setNumChannels(3); // match the live-camera / prefetcher RGB pipeline.
		}
		cacheInsert(index, px);
	}

	setCurrentFrame(std::move(px), index);

	// The forward prefetcher is now out of position: drop any staged frame
	// (returning its buffer to the pool) and reposition the running decoder so
	// streaming resumes with the frame after the sought one.
	if (hasPending && prefetcher) {
		prefetcher->recycle(std::move(pendingPixels));
	}
	hasPending = false;
	if (prefetcher) {
		prefetcher->reseek(static_cast<std::size_t>((index + 1) % count));
	}
}

//--------------------------------------------------------------
void PlaybackSource::setCurrentFrame(ofPixels && px, int index) {
	currentPixels = std::move(px);
	currentIndex = index;

	// Texture upload stays on the (GL) main thread.
	if (!currentTexture.isAllocated()
		|| static_cast<int>(currentTexture.getWidth()) != static_cast<int>(currentPixels.getWidth())
		|| static_cast<int>(currentTexture.getHeight()) != static_cast<int>(currentPixels.getHeight())) {
		currentTexture.allocate(currentPixels);
	}
	currentTexture.loadData(currentPixels);
	frameNew = true;
}

//--------------------------------------------------------------
void PlaybackSource::cacheInsert(int index, const ofPixels & px) {
	++cacheTick;
	for (auto & e : seekCache) {
		if (e.index == index) {
			e.pixels = px;
			e.lastUse = cacheTick;
			return;
		}
	}
	if (static_cast<int>(seekCache.size()) < kSeekCacheDepth) {
		seekCache.push_back({index, cacheTick, px});
		return;
	}
	// Evict the least recently used entry.
	auto victim = std::min_element(seekCache.begin(), seekCache.end(),
		[](const CacheEntry & a, const CacheEntry & b) { return a.lastUse < b.lastUse; });
	victim->index = index;
	victim->lastUse = cacheTick;
	victim->pixels = px;
}

//--------------------------------------------------------------
bool PlaybackSource::cacheLookup(int index, ofPixels & out) {
	for (auto & e : seekCache) {
		if (e.index == index) {
			e.lastUse = ++cacheTick;
			out = e.pixels;
			return true;
		}
	}
	return false;
}
