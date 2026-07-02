#pragma once

#include "FrameSource.h"

#include "ofMain.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

/// Disk-playback frame source: streams an ordered, index-aligned list of
/// recorded frames (built by PlaybackTimeline) with a background decoder.
///
/// Forward playback is two-phase so the app can keep every eye locked to the
/// same frame index: stage() pulls the next decoded frame into a holding slot
/// and returns true once one is ready; commit() swaps it in and uploads the
/// texture. The PlaybackController only commits after *every* eye has a frame
/// staged, so per-eye decode-rate differences can never desync them.
///
/// Interactive scrubbing uses seek(): an LRU cache of recently shown/decoded
/// frames makes back-and-forth stepping instant, and the background prefetcher
/// is repositioned in place (reseek) instead of being torn down and restarted.
class PlaybackSource : public IFrameSource {
public:
	struct Config {
		std::vector<std::string> files; ///< Index-aligned frame paths (absolute).
		bool loop = true;
		std::string name = "eye"; ///< Log tag.
	};

	explicit PlaybackSource(Config cfg);
	~PlaybackSource() override;

	// IFrameSource
	void update() override {} // advanced externally via stage/commit/seek.
	bool isFrameNew() const override { return frameNew; }
	const ofPixels & pixels() const override { return currentPixels; }
	const ofTexture & texture() const override { return currentTexture; }
	bool isPlayback() const override { return true; }
	int frameCount() const override { return static_cast<int>(config.files.size()); }
	/// Absolute index of the frame currently displayed, or -1 before the first
	/// frame has been committed.
	int frameIndex() const override { return currentIndex; }
	std::string currentFilePath() const override {
		return (currentIndex >= 0 && currentIndex < frameCount())
			? config.files[currentIndex] : std::string();
	}

	/// Clears the per-tick "new frame" flag and pulls the next decoded frame
	/// into the holding slot (if not already staged from an earlier tick where
	/// the other eye wasn't ready). Returns whether a frame is staged and ready.
	bool stage();
	/// Swaps the staged frame in as the current frame and uploads the texture.
	/// Main/GL thread only.
	void commit();
	/// Random-access seek used by the interactive transport (pause + frame
	/// stepping). Serves the frame from the LRU cache when possible, otherwise
	/// decodes synchronously on the calling (GL main) thread. Repositions the
	/// background prefetcher so forward playback resumes cleanly from here.
	/// `index` is wrapped into [0, frameCount).
	void seek(int index);

private:
	/// Background frame decoder. JPEG/PNG decode is the playback bottleneck,
	/// so it is pulled off the render thread: this thread reads ahead through
	/// the ordered frame list and decodes into a bounded pool of reusable
	/// ofPixels buffers. The main thread pops the next decoded frame
	/// (non-blocking), uploads it to a texture, and returns the buffer. The
	/// `empty` pool caps how many frames may be in flight, so a fast disk
	/// cannot run the decoder ahead without bound.
	///
	/// reseek() repositions the running thread (instead of destroy + recreate):
	/// frames are generation-tagged so anything decoded from the old position
	/// is recognized as stale by the consumer and its buffer recycled.
	class FramePrefetcher : public ofThread {
	public:
		struct Frame {
			ofPixels pixels;
			std::uint64_t generation = 0;
		};

		~FramePrefetcher() override { stop(); }

		void start(const std::vector<std::string> * files, bool loop, int depth,
			const std::string & tag, std::size_t startCursor);
		void stop();
		/// Non-blocking: moves the next decoded frame into `out` if one is
		/// ready. Returns the generation check result via `stale`; the caller
		/// recycles stale frames and asks again.
		bool tryGet(ofPixels & out, bool & stale);
		/// Returns a consumed buffer to the pool for reuse.
		void recycle(ofPixels && buf);
		/// Repositions the decode cursor so the next delivered frame is
		/// `cursor`. In-flight/queued frames from the old position are
		/// invalidated (generation bump) and their buffers recycled as the
		/// consumer drains them.
		void reseek(std::size_t cursor);

	protected:
		void threadedFunction() override;

	private:
		ofThreadChannel<ofPixels> empty; ///< Reusable buffer pool (bounds prefetch depth).
		ofThreadChannel<Frame> filled;   ///< Decoded frames ready for the main thread.
		const std::vector<std::string> * playlist = nullptr;
		bool loopPlayback = true;
		std::string name;

		std::mutex cursorMutex;
		std::size_t cursor = 0;
		std::uint64_t generation = 0; ///< Bumped by every reseek().
		std::atomic<bool> stopping{false};
	};

	/// Single place that installs `px` as the current frame: swap, texture
	/// (re)allocate on size change, upload, mark new. De-dups what commit()
	/// and seek() previously each did on their own.
	void setCurrentFrame(ofPixels && px, int index);
	/// Inserts a copy of `px` into the LRU seek cache (evicting the least
	/// recently used entry when full).
	void cacheInsert(int index, const ofPixels & px);
	/// Copies the cached frame for `index` into `out` if present (refreshing
	/// its LRU stamp).
	bool cacheLookup(int index, ofPixels & out);

	Config config;

	ofPixels currentPixels;
	ofTexture currentTexture;
	bool frameNew = false;
	int currentIndex = -1;

	// Holding slot for a decoded-but-not-yet-shown frame, so the controller
	// can wait until every eye has one ready before committing them together.
	ofPixels pendingPixels;
	bool hasPending = false;

	std::unique_ptr<FramePrefetcher> prefetcher;

	// LRU cache of recently shown/decoded frames so interactive back-and-forth
	// stepping (the annotation workflow) never re-decodes. Bounded; full-res
	// RGB frames are large, so the depth stays small.
	struct CacheEntry {
		int index = -1;
		std::uint64_t lastUse = 0;
		ofPixels pixels;
	};
	std::vector<CacheEntry> seekCache;
	std::uint64_t cacheTick = 0;
};
