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

	/// Parses the given config file. Returns false (and logs) if the file is
	/// missing or unparseable; in that case all getters return their defaults.
	bool load(const std::string & path);

	bool isLoaded() const { return loaded; }

	/// "mask" or "side_by_side". Drives the initial m-toggle state.
	const std::string & getDisplayMode() const { return displayMode; }
	bool startsInMaskMode() const { return displayMode != "side_by_side"; }

	const StreamingConfig & getStreaming() const { return streaming; }

	/// The "mask" sub-object, passed to MaskLayout::load(). Empty if absent.
	const ofJson & getMaskJson() const { return maskJson; }

private:
	ofJson maskJson = ofJson::object();
	std::string displayMode = "mask";
	StreamingConfig streaming;
	bool loaded = false;
};
