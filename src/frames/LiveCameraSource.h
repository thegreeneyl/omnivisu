#pragma once

#if !defined(OMNIVISU_NO_CAMERA)

#include "FrameSource.h"

#include "ofMain.h"
#include "ofxIdsPeak.h"

#include <cstdio>
#include <memory>
#include <string>

/// Live IDS-camera frame source. Owns the ofxIdsPeak grabber (and thereby the
/// camera parameter group used by the GUI/JSON) plus raw-frame recording.
///
/// The object is constructed even when the stream runs disk playback, so the
/// grabber's parameter group keeps existing and the per-eye JSON round-trips
/// unchanged; open() is simply never called in that case.
class LiveCameraSource : public IFrameSource {
public:
	struct Config {
		std::string name = "eye";        ///< Log tag.
		std::string recordingFormat = "jpg";
	};

	explicit LiveCameraSource(Config cfg)
		: config(std::move(cfg)) {}
	~LiveCameraSource() override { stopRecording(); }

	/// Camera publishes RGB color frames so the rendered FBO stays RGB; the
	/// detection worker desaturates internally. Call before open().
	void applyLowLatencyPreset() {
		grabber_.applyLowLatencyPreset(ofxIdsPeak::OutputFormat::RGB);
	}
	/// Translates the JSON "camera" block (select_by / value) into the
	/// grabber's device selector so this eye binds to a specific physical
	/// camera (by serial/model/index). Call before open().
	void selectDevice(const std::string & selectBy, const std::string & selectValue);
	/// Opens the camera. Returns false when initialization failed.
	bool open() { return grabber_.setup(); }

	// IFrameSource
	void update() override {
		grabber_.update();
		// Capture the raw (uncropped, ungraded) frame before anything
		// downstream touches it, so playback reproduces the live pipeline
		// exactly.
		if (recorder && grabber_.isFrameNew()) {
			recorder->enqueue(grabber_.getPixels());
		}
	}
	bool isFrameNew() const override { return grabber_.isFrameNew(); }
	const ofPixels & pixels() const override { return grabber_.getPixels(); }
	const ofTexture & texture() const override { return grabber_.getTexture(); }
	bool isPlayback() const override { return false; }

	/// Direct grabber access for camera-specific needs: the GUI/JSON parameter
	/// group, parameter batching around bulk deserializes, and the frame/error
	/// counters in the diagnostic log.
	ofxIdsPeak::Grabber & grabber() { return grabber_; }
	const ofxIdsPeak::Grabber & grabber() const { return grabber_; }

	/// Records the raw camera frames to an image sequence in `dir` (relative
	/// to bin/data). Writing happens on a background thread so capture is
	/// never stalled.
	void startRecording(const std::string & dir);
	void stopRecording();
	bool isRecording() const { return recorder != nullptr; }

private:
	/// Background frame saver. Owns its own thread and channel so a fresh
	/// instance is created per recording session (an ofThreadChannel cannot be
	/// reopened once closed). Writes zero-padded image sequences.
	class FrameRecorder : public ofThread {
	public:
		~FrameRecorder() override { stop(); }
		void start(const std::string & dir, const std::string & format) {
			outDir = dir;
			fmt = format.empty() ? "jpg" : format;
			startThread();
		}
		void stop() {
			channel.close();
			if (isThreadRunning()) {
				waitForThread(false);
			}
		}
		void enqueue(const ofPixels & px) { channel.send(px); }

	protected:
		void threadedFunction() override {
			ofPixels px;
			std::size_t idx = 0;
			while (channel.receive(px)) {
				char name[48];
				std::snprintf(name, sizeof(name), "/frame_%06zu.", idx++);
				ofSaveImage(px, outDir + name + fmt, OF_IMAGE_QUALITY_BEST);
			}
		}

	private:
		ofThreadChannel<ofPixels> channel;
		std::string outDir;
		std::string fmt = "jpg";
	};

	Config config;
	ofxIdsPeak::Grabber grabber_;
	std::unique_ptr<FrameRecorder> recorder; ///< Non-null only while recording.
};

#endif // !defined(OMNIVISU_NO_CAMERA)
