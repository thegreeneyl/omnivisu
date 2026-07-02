#include "PlaybackController.h"

#include "ofMain.h"

//--------------------------------------------------------------
void PlaybackController::setup(const Settings & s,
	const std::vector<std::string> & eyeNames) {
	settings = s;
	timeline.scan(settings.folder, eyeNames);
}

//--------------------------------------------------------------
void PlaybackController::attach(const std::vector<PlaybackSource *> & playbackSources) {
	sources = playbackSources;
	allPlayback = !sources.empty();
	for (const auto * src : sources) {
		if (src == nullptr) {
			allPlayback = false;
			break;
		}
	}
	if (!allPlayback) {
		sources.clear();
	}
}

//--------------------------------------------------------------
void PlaybackController::update() {
	if (!allPlayback || isPaused) {
		return;
	}

	if (settings.fps > 0.0f && ofGetElapsedTimef() < nextFrameTime) {
		return;
	}

	bool allStaged = true;
	for (auto * src : sources) {
		// Call stage on every source (side effects), never short-circuit.
		allStaged = src->stage() && allStaged;
	}
	if (!allStaged) {
		return;
	}
	for (auto * src : sources) {
		src->commit();
	}

	if (settings.fps > 0.0f) {
		const float now = ofGetElapsedTimef();
		const float interval = 1.0f / settings.fps;
		nextFrameTime += interval;
		// Resync after a stall instead of bursting a catch-up sequence.
		if (nextFrameTime < now) {
			nextFrameTime = now + interval;
		}
	}
}

//--------------------------------------------------------------
void PlaybackController::step(int delta) {
	if (!allPlayback) {
		return;
	}

	// Stepping always implies a paused transport so the sought frame holds.
	isPaused = true;

	const int count = sources[0]->frameCount();
	if (count <= 0) {
		return;
	}

	// Compute one shared target index from the reference eye so every eye
	// stays locked to the same frame (the per-eye lists are index-aligned).
	const int base = sources[0]->frameIndex();
	const int target = (((base + delta) % count) + count) % count;
	for (auto * src : sources) {
		src->seek(target);
	}
	ofLogNotice("omnivisu") << "playback frame " << target << " / " << (count - 1);
}

//--------------------------------------------------------------
void PlaybackController::togglePaused() {
	isPaused = !isPaused;
	ofLogNotice("omnivisu") << (isPaused ? "playback paused" : "playback resumed");
}
