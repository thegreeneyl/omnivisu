#pragma once

#include <map>
#include <string>
#include <vector>

/// Shared playback timeline: scans the recordings folder ONCE and builds one
/// index-aligned frame-file list per eye. Frame index N of every eye always
/// belongs to the same recorded moment, because each session is clamped to the
/// shortest eye_* sequence it contains (the cameras are unsynchronized and
/// record slightly different frame counts; surplus tail frames are simply not
/// queued - recordings on disk are left untouched).
///
/// This replaces the previous per-stream folder scan, where every eye
/// re-walked all sibling eye folders just to compute the same clamp.
/// Pure data + filesystem reads; no GL, no threads, trivially testable.
class PlaybackTimeline {
public:
	/// Scans `rootFolder` (relative to bin/data): sessions are its timestamped
	/// subfolders (sorted lexically == chronological with the YYMMDD-HHMMSS
	/// naming), each containing one eye_<name> image-sequence subfolder per
	/// eye. Builds the per-eye lists for `eyeNames` (e.g. {"left", "right"}).
	/// Returns true when at least one frame was found for any eye.
	bool scan(const std::string & rootFolder, const std::vector<std::string> & eyeNames);

	/// Index-aligned frame files for one eye (absolute paths). Empty when the
	/// eye had no frames or scan() was never called.
	const std::vector<std::string> & files(const std::string & eyeName) const;

	/// Frames in the longest per-eye list (the reference eye's count).
	int frameCount() const;

private:
	std::map<std::string, std::vector<std::string>> perEye;
	static const std::vector<std::string> kEmpty;
};
