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
	}

	if (json.contains("mask")) {
		maskJson = json["mask"];
	}

	loaded = true;
	ofLogNotice("AppConfig") << "loaded " << path << " (display_mode=" << displayMode
		<< ", streaming=" << (streaming.enabled ? "on" : "off")
		<< " -> " << streaming.targetIp << ":" << streaming.targetPort << ")";
	return true;
}
