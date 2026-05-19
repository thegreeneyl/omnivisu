#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxIdsPeak.h"
#include "OneEuroFilter.h"

#include <opencv2/imgproc.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>

/// One camera stream: capture (main thread), eye detection (worker thread),
/// full-frame letterboxed render to FBO (main thread).
/// Simplified: detects only the eye bounding box + center. Angle, pupil,
/// gaze, blink and squint detection have been removed for debugging clarity.
class EyeCameraStream : protected ofThread {
public:
	EyeCameraStream() = default;
	~EyeCameraStream() override;
	EyeCameraStream(const EyeCameraStream &) = delete;
	EyeCameraStream & operator=(const EyeCameraStream &) = delete;
	EyeCameraStream(EyeCameraStream &&) = delete;
	EyeCameraStream & operator=(EyeCameraStream &&) = delete;

	struct Config {
		std::string name = "eye";
		glm::ivec2 fboSize = {672, 504};
		std::string cascadeFile = "haarcascade_eye.xml";
		bool mirrorX = true;
	};

	struct Result {
		bool present = false;
		float confidence = 0.0f;
		glm::vec2 eyeCenter = {0, 0};
		ofRectangle eyeBox;
		float detectedEyeWidthPx = 0.0f;
	};

	bool setup(const Config & cfg);
	void update();
	void draw(float x, float y, float w, float h) const;
	void drawDebug(float x, float y, float w, float h) const;

	const ofFbo & getTargetFbo() const { return targetFbo; }
	Result getResult() const {
		std::lock_guard<std::mutex> lk(stateMutex);
		return result;
	}
	const ofxIdsPeak::Grabber & getGrabber() const { return grabber; }
	ofParameterGroup & getParameters() { return parameters.group; }
	const std::string & getName() const { return config.name; }

	std::uint64_t getDetectionCount() const { return detectionsRun.load(); }
	std::uint64_t getValidDetectionCount() const { return detectionsValid.load(); }

private:
	struct RawDetection {
		bool valid = false;
		ofRectangle eyeBox;
		glm::vec2 eyeCenter = {0, 0};
		float confidence = 0.0f;
	};

	struct Parameters {
		ofParameterGroup group;
		ofParameter<int> haarMinWidth{"haar min width", 500, 8, 1000};
		ofParameter<int> haarMinHeight{"haar min height", 350, 8, 1000};
		ofParameter<float> haarScale{"haar scale", 1.08f, 1.01f, 1.5f};
		ofParameter<int> haarNeighbors{"haar neighbors", 5, 0, 20};
		ofParameter<int> presentOnFrames{"present on frames", 4, 1, 30};
		ofParameter<int> presentOffFrames{"present off frames", 10, 1, 60};
		ofParameter<float> euroMinCutoff{"euro min cutoff", 0.5f, 0.01f, 5.0f};
		ofParameter<float> euroBeta{"euro beta", 0.007f, 0.0f, 0.1f};
		ofParameter<bool> enableEyeTracking{"enable eye tracking", true};
		ofParameter<bool> showDebugOverlay{"show debug overlay", true};

		ofParameterGroup gradingGroup{"grading"};
		ofParameter<bool> enableGrading{"enable grading", true};
		ofParameter<float> gradeExposure{"exposure (stops)", 0.0f, -2.0f, 2.0f};
		ofParameter<float> gradeBrightness{"brightness", 0.0f, -0.5f, 0.5f};
		ofParameter<float> gradeContrast{"contrast", 1.0f, 0.0f, 2.0f};
		ofParameter<float> gradeGamma{"gamma", 1.0f, 0.3f, 3.0f};
		ofParameter<float> gradeSaturation{"saturation", 1.0f, 0.0f, 2.0f};
	};

	struct FrameJob {
		ofPixels pixels;
		float captureTime = 0.0f;
	};

	void setupParameters();
	bool detectEye(const ofPixels & pixels, RawDetection & out);
	ofxCvBlob pickBestEyeBlob(const std::vector<ofxCvBlob> & blobs) const;
	void applySmoothing(const RawDetection & raw, float dt);
	void renderTargetFbo();
	void threadedFunction() override;
	void resetWorkerStateLocked();
	bool buildGradeShader(bool useRect);
	void drawCameraIntoFbo(float drawX, float drawY, float drawW, float drawH);

	Config config;
	Parameters parameters;
	Result result;

	ofxIdsPeak::Grabber grabber;
	ofFbo targetFbo;
	ofShader gradeShader;
	bool gradeShaderLoaded = false;
	bool gradeShaderUsesRect = false;
	ofxCvColorImage cvColor;
	ofxCvGrayscaleImage cvGray;
	ofxCvHaarFinder eyeFinder;

	RawDetection lastRaw;
	ofRectangle lastEyeBox;
	bool hasLastEyeBox = false;

	int presentHitStreak = 0;
	int presentMissStreak = 0;
	bool presentSmoothed = false;
	bool presentSmoothedPrev = false;
	float workerLastFrameTime = 0.0f;

	ofThreadChannel<FrameJob> frameJobs;
	mutable std::mutex stateMutex;

	OneEuroFilter filterEyeX;
	OneEuroFilter filterEyeY;

	std::atomic<std::uint64_t> detectionsRun{0};
	std::atomic<std::uint64_t> detectionsValid{0};
};
