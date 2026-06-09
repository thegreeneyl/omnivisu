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
		std::string paramsFile;
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
	ofParameterGroup & getGrabberParameters() { return grabber.parameters.group; }
	ofParameterGroup & getTrackingParameters() { return parameters.trackingGroup; }
	ofParameterGroup & getGradingParameters() { return parameters.gradingGroup; }
	ofParameterGroup & getViewParameters() { return parameters.viewGroup; }
	const std::string & getName() const { return config.name; }

	bool saveParameters() const;
	bool loadParameters();

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

		ofParameterGroup trackingGroup{"tracking"};
		ofParameter<bool> enableEyeTracking{"enable eye tracking", true};
		ofParameter<bool> showDebugOverlay{"show debug overlay", true};
		ofParameter<int> trackingDownscale{"tracking downscale", 8, 1, 16};
		ofParameter<int> haarMinWidth{"haar min width", 400, 8, 1000};
		ofParameter<int> haarMinHeight{"haar min height", 250, 8, 1000};
		ofParameter<float> haarScale{"haar scale", 1.01f, 1.001f, 1.5f};
		ofParameter<int> haarNeighbors{"haar neighbors", 5, 0, 20};
		ofParameter<int> presentOnFrames{"present on frames", 4, 1, 30};
		ofParameter<int> presentOffFrames{"present off frames", 10, 1, 60};
		ofParameter<float> euroMinCutoff{"euro min cutoff", 0.5f, 0.01f, 5.0f};
		ofParameter<float> euroBeta{"euro beta", 0.007f, 0.0f, 0.1f};

		ofParameterGroup gradingGroup{"grading"};
		ofParameter<bool> enableGrading{"enable grading", true};
		ofParameter<float> gradeExposure{"exposure (stops)", 0.0f, -2.0f, 2.0f};
		ofParameter<float> gradeBrightness{"brightness", 0.0f, -0.5f, 0.5f};
		ofParameter<float> gradeContrast{"contrast", 1.6f, 0.0f, 2.0f};
		ofParameter<float> gradeGamma{"gamma", 1.0f, 0.3f, 3.0f};
		ofParameter<float> gradeSaturation{"saturation", 1.0f, 0.0f, 2.0f};

		ofParameterGroup viewGroup{"view"};
		ofParameter<float> viewScale{"scale", 2.0f, 0.1f, 10.0f};
		// fit to fill ON: scale is a multiplier on the aspect-fit baseline that
		// shrinks the whole frame into the FBO. OFF: scale is the exact camera
		// pixel -> FBO pixel factor (1.0 == one camera pixel per FBO pixel).
		ofParameter<bool> fitToFill{"fit to fill", true};
		ofParameter<bool> followEye{"follow eye", false};
		ofParameter<float> followSmoothing{"follow smoothing", 0.5f, 0.0f, 1.0f};

		// FBO (per-eye render target) dimensions. Read from JSON at setup time
		// and applied to Config::fboSize before allocation. Not added to the
		// runtime GUI because changing them only takes effect on startup.
		ofParameterGroup fboGroup{"fbo"};
		ofParameter<int> fboWidth{"width", 672, 1, 8192};
		ofParameter<int> fboHeight{"height", 504, 1, 8192};
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
	std::string paramsFilePath() const;

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
	ofPixels detectionPixels; ///< Worker-only scratch buffer for downsampled tracking input.

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

	// Eye-follow smoothing state. <0 marks an uninitialized smoothed center so
	// the first valid detection snaps in (no lurch from origin) and subsequent
	// detections low-pass toward it.
	float followSmoothedX = -1.0f;
	float followSmoothedY = -1.0f;
	float lastFollowTime = 0.0f;

	// Layout actually used by the most recent renderTargetFbo() call. Shared
	// with drawDebug() so overlays stay aligned through scale and follow shifts.
	struct DrawLayout {
		float x = 0.0f;
		float y = 0.0f;
		float w = 0.0f;
		float h = 0.0f;
		bool valid = false;
	};
	DrawLayout lastLayout;

	std::atomic<std::uint64_t> detectionsRun{0};
	std::atomic<std::uint64_t> detectionsValid{0};
};
