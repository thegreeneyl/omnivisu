#pragma once

#include "ofMain.h"

#include <string>

/// Abstract frame supplier for one eye stream. Exactly one concrete source
/// feeds an EyeCameraStream: LiveCameraSource (IDS camera) or PlaybackSource
/// (on-disk image sequence). Everything downstream (detection, grading,
/// render) reads frames through this interface and stays source-agnostic.
class IFrameSource {
public:
	virtual ~IFrameSource() = default;

	/// Per-tick poll on the main/GL thread. Live capture pumps the grabber
	/// here; playback is advanced externally (PlaybackController lockstep), so
	/// its implementation is a no-op.
	virtual void update() = 0;

	/// True while the current frame has not been superseded/consumed.
	/// Live: edge-triggered per grabber update. Playback: level-triggered
	/// (stays true until the next stage attempt), preserving the existing
	/// "keep re-detecting the held frame while paused" behavior.
	virtual bool isFrameNew() const = 0;

	virtual const ofPixels & pixels() const = 0;
	virtual const ofTexture & texture() const = 0;

	/// True when frames come from disk (transport controls apply).
	virtual bool isPlayback() const = 0;

	// Transport introspection; meaningful for playback only. Live-safe
	// defaults so callers can query without dynamic casts.
	virtual int frameCount() const { return 0; }
	virtual int frameIndex() const { return -1; }
	virtual std::string currentFilePath() const { return {}; }
};
