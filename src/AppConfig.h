#pragma once

#include "ofMain.h"

#include <string>

/// Loads the application-level config.json. Holds general settings (initial
/// display mode), streaming target settings (config-only for now), and exposes
/// the "mask" sub-block as raw JSON for MaskLayout to consume.
class AppConfig {
public:
	struct StreamingConfig {
		bool enabled = false;
		std::string targetIp = "127.0.0.1";
		int targetPort = 12345;
		std::string compression = "none"; ///< "none" (raw RGB) or "jpeg".
		int packetPayloadBytes = 1440;    ///< UDP payload chunk size (sans header).
		int fpsLimit = 0;                 ///< Max send rate; 0 = unlimited.
		bool asyncReadback = true;        ///< PBO async FBO readback (+1 frame latency, recovers fps).
	};

	/// Dev-only: replay recorded image sequences instead of the live camera.
	/// When enabled the grabber is never opened; frames are read from
	/// <folder>/<session>/eye_<name>/ image sequences across all sessions.
	struct PlaybackConfig {
		bool enabled = false;
		std::string folder = "recordings"; ///< Root (relative to bin/data).
		bool loop = true;                  ///< Restart at the first frame when the last session ends.
		float fps = 0.0f;                  ///< Target playback rate; 0 = uncapped (as fast as decode/display allow).
	};

	/// Dev-only: where raw camera frames are written while recording.
	struct RecordingConfig {
		std::string folder = "recordings"; ///< Root (relative to bin/data).
		std::string format = "jpg";        ///< Image sequence extension ("jpg" or "png").
	};

	/// Parses the given config file. Returns false (and logs) if the file is
	/// missing or unparseable; in that case all getters return their defaults.
	bool load(const std::string & path);

	bool isLoaded() const { return loaded; }

	/// "mask" or "side_by_side". Drives the initial m-toggle state.
	const std::string & getDisplayMode() const { return displayMode; }
	bool startsInMaskMode() const { return displayMode != "side_by_side"; }

	const StreamingConfig & getStreaming() const { return streaming; }

	const PlaybackConfig & getPlayback() const { return playback; }
	const RecordingConfig & getRecording() const { return recording; }

	/// The "mask" sub-object, passed to MaskLayout::load(). Empty if absent.
	const ofJson & getMaskJson() const { return maskJson; }

	/// The "mouth" sub-object, passed to Mouth::load(). Empty if absent.
	const ofJson & getMouthJson() const { return mouthJson; }

private:
	ofJson maskJson = ofJson::object();
	ofJson mouthJson = ofJson::object();
	std::string displayMode = "mask";
	StreamingConfig streaming;
	PlaybackConfig playback;
	RecordingConfig recording;
	bool loaded = false;
};
