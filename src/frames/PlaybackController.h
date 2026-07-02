#pragma once

#include "PlaybackSource.h"
#include "PlaybackTimeline.h"

#include <string>
#include <vector>

/// App-level playback coordinator. Owns the shared timeline (scanned once for
/// all eyes) and drives the transport across every eye's PlaybackSource:
/// two-phase lockstep advance, pause, and frame-accurate stepping - all eyes
/// always hold the same frame index.
class PlaybackController {
public:
	struct Settings {
		std::string folder = "recordings"; ///< Recordings root, relative to bin/data.
		float fps = 0.0f;                  ///< Frame-rate cap; 0 = uncapped.
	};

	/// Scans the shared timeline. Call before creating the streams so their
	/// PlaybackSources can be built from files().
	void setup(const Settings & s, const std::vector<std::string> & eyeNames);

	/// Index-aligned frame files for one eye (empty when setup() was skipped
	/// or the eye had no recordings).
	const std::vector<std::string> & files(const std::string & eyeName) const {
		return timeline.files(eyeName);
	}

	/// Registers each stream's playback source, in stream order. Pass nullptr
	/// for live streams; the transport only activates when EVERY stream is
	/// fed from disk (mixed setups keep the live behavior untouched).
	void attach(const std::vector<PlaybackSource *> & playbackSources);

	/// True when the transport applies (every stream is disk playback).
	bool active() const { return allPlayback; }

	/// Picks up a changed fps cap (config reload).
	void setFps(float fps) { settings.fps = fps; }

	/// Per-tick lockstep advance: stage every eye's next decoded frame, then
	/// commit them together only when *all* are ready, so per-eye decode-rate
	/// differences can't desync the frame indices. No-op when inactive,
	/// paused, or gated by the fps cap.
	void update();

	/// Pause the transport (if playing) and seek every eye by `delta` frames,
	/// keeping them locked to a single shared index. Negative steps back.
	void step(int delta);

	bool paused() const { return isPaused; }
	void togglePaused();

private:
	Settings settings;
	PlaybackTimeline timeline;
	std::vector<PlaybackSource *> sources;
	bool allPlayback = false;
	// Interactive transport: spacebar toggles this; the arrow keys step frames
	// (auto-pausing). While paused, frame changes come solely from step().
	bool isPaused = false;
	// Fixed-grid schedule for the fps cap. Only the *attempt* is time-gated;
	// the schedule advances solely on a successful commit, so the effective
	// rate is min(target fps, decode throughput) and a slow decoder never
	// makes us silently skip ahead.
	float nextFrameTime = 0.0f;
};
