#include "AppConfig.h"

//--------------------------------------------------------------
bool AppConfig::load(const std::string & path) {
	loaded = false;

	ofFile file(path);
	if (!file.exists()) {
		ofLogWarning("AppConfig") << "config file not found: " << path
			<< " - using defaults";
		return false;
	}

	ofJson json;
	try {
		json = ofLoadJson(path);
	} catch (const std::exception & e) {
		ofLogError("AppConfig") << "failed to parse " << path << ": " << e.what();
		return false;
	}
	if (json.is_null() || json.empty()) {
		ofLogError("AppConfig") << "empty or invalid JSON: " << path;
		return false;
	}

	if (json.contains("general")) {
		const auto & g = json["general"];
		displayMode = g.value("display_mode", displayMode);
	}

	if (json.contains("streaming")) {
		const auto & s = json["streaming"];
		streaming.enabled = s.value("enabled", streaming.enabled);
		streaming.targetIp = s.value("target_ip", streaming.targetIp);
		streaming.targetPort = s.value("target_port", streaming.targetPort);
		streaming.compression = s.value("compression", streaming.compression);
		streaming.packetPayloadBytes = s.value("packet_payload_bytes", streaming.packetPayloadBytes);
		streaming.fpsLimit = s.value("fps_limit", streaming.fpsLimit);
		streaming.asyncReadback = s.value("async_readback", streaming.asyncReadback);
	}

	if (json.contains("playback")) {
		const auto & p = json["playback"];
		playback.enabled = p.value("enabled", playback.enabled);
		playback.folder = p.value("folder", playback.folder);
		playback.loop = p.value("loop", playback.loop);
		playback.fps = p.value("fps", playback.fps);
	}

	if (json.contains("recording")) {
		const auto & r = json["recording"];
		recording.folder = r.value("folder", recording.folder);
		recording.format = r.value("format", recording.format);
	}

	if (json.contains("mask")) {
		maskJson = json["mask"];
	}

	if (json.contains("mouth")) {
		mouthJson = json["mouth"];
	}

	loaded = true;
	ofLogNotice("AppConfig") << "loaded " << path << " (display_mode=" << displayMode
		<< ", streaming=" << (streaming.enabled ? "on" : "off")
		<< " -> " << streaming.targetIp << ":" << streaming.targetPort
		<< ", playback=" << (playback.enabled ? "on" : "off") << ")";
	return true;
}
