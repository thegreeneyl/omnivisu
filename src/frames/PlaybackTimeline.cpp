#include "PlaybackTimeline.h"

#include "ofMain.h"

#include <algorithm>
#include <cstddef>

const std::vector<std::string> PlaybackTimeline::kEmpty;

namespace {

/// Sorted list of image files inside one eye subfolder.
std::vector<std::string> listEyeFrames(const std::string & dir) {
	std::vector<std::string> files;
	ofDirectory d(dir);
	if (!d.exists()) {
		return files;
	}
	d.allowExt("jpg");
	d.allowExt("jpeg");
	d.allowExt("png");
	d.listDir();
	d.sort();
	for (std::size_t i = 0; i < d.size(); ++i) {
		files.push_back(d.getPath(i));
	}
	return files;
}

} // namespace

//--------------------------------------------------------------
bool PlaybackTimeline::scan(const std::string & rootFolder,
	const std::vector<std::string> & eyeNames) {
	perEye.clear();
	for (const auto & name : eyeNames) {
		perEye[name]; // ensure every requested eye has a (possibly empty) list.
	}

	const std::string root = ofToDataPath(rootFolder, true);
	ofDirectory rootDir(root);
	if (!rootDir.exists()) {
		ofLogWarning("PlaybackTimeline") << "playback folder not found: " << root;
		return false;
	}

	rootDir.listDir();
	std::vector<std::string> sessions;
	for (std::size_t i = 0; i < rootDir.size(); ++i) {
		if (rootDir.getFile(i).isDirectory()) {
			sessions.push_back(rootDir.getPath(i));
		}
	}
	std::sort(sessions.begin(), sessions.end());

	for (const auto & session : sessions) {
		// Frames of every eye_* subfolder in this session (including eyes not
		// requested), because the alignment clamp considers all of them.
		std::map<std::string, std::vector<std::string>> sessionFrames;
		ofDirectory sessionDir(session);
		sessionDir.listDir();
		std::size_t clamp = 0;
		bool haveAny = false;
		for (std::size_t i = 0; i < sessionDir.size(); ++i) {
			if (!sessionDir.getFile(i).isDirectory()) {
				continue;
			}
			const std::string dirName = sessionDir.getName(i);
			if (dirName.rfind("eye_", 0) != 0) {
				continue;
			}
			std::vector<std::string> frames = listEyeFrames(sessionDir.getPath(i));
			if (frames.empty()) {
				continue; // an empty eye folder neither contributes nor clamps.
			}
			clamp = haveAny ? std::min(clamp, frames.size()) : frames.size();
			haveAny = true;
			sessionFrames[dirName.substr(4)] = std::move(frames);
		}
		if (!haveAny) {
			continue;
		}

		for (const auto & name : eyeNames) {
			auto it = sessionFrames.find(name);
			if (it == sessionFrames.end()) {
				continue;
			}
			auto & list = perEye[name];
			for (std::size_t i = 0; i < clamp; ++i) {
				list.push_back(it->second[i]);
			}
		}
	}

	bool any = false;
	for (const auto & name : eyeNames) {
		const std::size_t n = perEye[name].size();
		any = any || n > 0;
		ofLogNotice("PlaybackTimeline") << name << ": " << n
			<< " index-aligned frames from " << sessions.size()
			<< " session(s) in " << root;
	}
	return any;
}

//--------------------------------------------------------------
const std::vector<std::string> & PlaybackTimeline::files(const std::string & eyeName) const {
	const auto it = perEye.find(eyeName);
	return it != perEye.end() ? it->second : kEmpty;
}

//--------------------------------------------------------------
int PlaybackTimeline::frameCount() const {
	std::size_t best = 0;
	for (const auto & kv : perEye) {
		best = std::max(best, kv.second.size());
	}
	return static_cast<int>(best);
}
