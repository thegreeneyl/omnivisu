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
/// Detection runs one of two selectable strategies (see Parameters::detectorMode):
///   - iris: classical iris/limbus ellipse fit (default) -> center, radius, fit
///           quality, plus the data later phases use for gaze/openness.
///   - haar: the legacy Haar eye cascade, kept for A/B comparison.
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

		// Iris/limbus detection (iris detector). In iris mode eyeCenter mirrors
		// irisCenter; in haar mode the iris fields stay zero.
		glm::vec2 irisCenter = {0, 0};
		glm::vec2 irisSize = {0, 0};   ///< Fitted ellipse full axes (w,h) in source px.
		float irisAngleDeg = 0.0f;     ///< Ellipse rotation, degrees.
		float irisRadiusPx = 0.0f;     ///< Mean radius = (irisSize.x + irisSize.y) / 4.
		float fitQuality = 0.0f;       ///< 0..1 ellipse-fit confidence (0 for haar).

		// Phase 2 expression signals (populated later; defaults are neutral).
		glm::vec2 gaze = {0, 0};       ///< Normalized iris offset vs neutral.
		float openness = 1.0f;         ///< 0 = closed .. 1 = wide open.
		bool blink = false;
	};

	bool setup(const Config & cfg);
	void update();
	void draw(float x, float y, float w, float h) const;
	void drawDebug(float x, float y, float w, float h) const;
	/// Draws the full (uncropped) graded camera frame aspect-fit into the rect,
	/// with the detection overlay on top. Used by the raw side-by-side debug view.
	void drawRawDebug(float x, float y, float w, float h);

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
	/// One iris ellipse candidate, in *source* pixel coordinates (already mapped
	/// back up from the downscaled detection image).
	struct IrisCandidate {
		glm::vec2 center = {0, 0};
		glm::vec2 size = {0, 0};   ///< Full ellipse axes (w, h).
		float angleDeg = 0.0f;
		float radius = 0.0f;       ///< Mean radius = (size.x + size.y) / 4.
		float fitQuality = 0.0f;   ///< 0..1.
		ofRectangle box;
	};

	struct RawDetection {
		bool valid = false;
		ofRectangle eyeBox;
		glm::vec2 eyeCenter = {0, 0};
		float confidence = 0.0f;

		// Iris fields (iris detector only; zero in haar mode).
		glm::vec2 irisCenter = {0, 0};
		glm::vec2 irisSize = {0, 0};
		float irisAngleDeg = 0.0f;
		float irisRadiusPx = 0.0f;
		float fitQuality = 0.0f;

		// All candidates considered this frame, in *source* px, for the debug
		// overlay ("show candidates"). Boxes are Haar eye boxes (modes 1/2);
		// irises are every gated ellipse (all modes).
		std::vector<ofRectangle> candidateBoxes;
		std::vector<IrisCandidate> candidateIris;
	};

	struct Parameters {
		ofParameterGroup group;

		ofParameterGroup trackingGroup{"tracking"};
		ofParameter<bool> enableEyeTracking{"enable eye tracking", true};
		ofParameter<bool> showDebugOverlay{"show debug overlay", true};
		// Active detector: 0 = pure iris/limbus, 1 = legacy Haar cascade,
		// 2 = Haar eye box with iris located inside it (default).
		ofParameter<int> detectorMode{"detector mode", 2, 0, 2};
		ofParameter<int> trackingDownscale{"tracking downscale", 8, 1, 16};
		ofParameter<int> haarMinWidth{"haar min width", 400, 8, 1000};
		ofParameter<int> haarMinHeight{"haar min height", 250, 8, 1000};
		ofParameter<float> haarScale{"haar scale", 1.01f, 1.001f, 1.5f};
		ofParameter<int> haarNeighbors{"haar neighbors", 5, 0, 20};
		// Iris detector tuning. The expected radius is in *source* pixels for
		// intuitive tuning; the detector converts to detection scale internally.
		// threshold mode: 0 = adaptive, 1 = Otsu, 2 = fixed (iris threshold).
		// preferred side: 0 = none, 1 = left half, 2 = right half.
		ofParameter<int> irisExpectedRadius{"iris expected radius", 120, 4, 300};
		ofParameter<float> irisRadiusTolerance{"iris radius tolerance", 0.6f, 0.05f, 1.0f};
		ofParameter<int> irisThresholdMode{"iris threshold mode", 1, 0, 2};
		ofParameter<int> irisThreshold{"iris threshold", 70, 0, 255};
		ofParameter<int> irisBlur{"iris blur", 5, 1, 31};
		ofParameter<float> irisMinFitQuality{"iris min fit quality", 0.35f, 0.0f, 1.0f};
		// Roundness gate: minor/major axis ratio a candidate must reach (1 = a
		// perfect circle). Higher rejects elongated ellipses.
		ofParameter<float> irisMinCircularity{"iris min circularity", 0.7f, 0.0f, 1.0f};
		ofParameter<int> irisPreferredSide{"iris preferred side", 0, 0, 2};
		// Debug: draw every candidate eye box and iris ellipse (not just the
		// chosen one) on the raw camera overlay.
		ofParameter<bool> debugShowCandidates{"debug show candidates", false};
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

		// Which physical camera this eye binds to. Read from JSON at setup time
		// and translated into the grabber's device selector before it opens.
		// Startup-only, so not added to the runtime GUI.
		ofParameterGroup cameraGroup{"camera"};
		ofParameter<std::string> selectBy{"select by", "serial"};
		ofParameter<std::string> selectValue{"value", ""};
	};

	struct FrameJob {
		ofPixels pixels;
		float captureTime = 0.0f;
	};

	void setupParameters();
	bool detectEye(const ofPixels & pixels, RawDetection & out);
	/// Haar cascade detection on the prepared `cvGray` image. Fills `out` and
	/// returns true on a valid hit. (Legacy path, kept for A/B comparison.)
	bool detectHaar(int downscale, int srcW, int srcH, RawDetection & out);
	/// Iris/limbus detection on the prepared `cvGray` image: threshold ->
	/// contours -> fitEllipse -> size/fit gating -> best candidate.
	bool detectIris(int downscale, RawDetection & out);
	/// Combined detector: Haar gives candidate eye boxes; the iris detector runs
	/// inside each box. Boxes without a valid iris are discarded (false-positive
	/// rejection). Eye box = anchor, iris = gaze source.
	bool detectEyeBoxIris(int downscale, int srcW, int srcH, RawDetection & out);
	/// Iris candidate finder shared by the iris and combined detectors. Runs on
	/// `gray` (a full or ROI detection image); `offX`/`offY` is the ROI origin in
	/// detection px. Emits candidates in *source* pixels. Clears `out` first.
	void collectIrisCandidates(const cv::Mat & gray, int downscale,
		float offX, float offY, std::vector<IrisCandidate> & out) const;
	ofxCvBlob pickBestEyeBlob(const std::vector<ofxCvBlob> & blobs) const;
	IrisCandidate pickBestIris(const std::vector<IrisCandidate> & cands) const;
	/// True if a source-px x lies on the configured preferred half. Mirror-aware,
	/// so the half matches what the user sees in the (mirrored) display. Returns
	/// true for every x when no preference is set (preferred side == 0).
	bool onPreferredSide(float srcX, float srcW) const;
	void applySmoothing(const RawDetection & raw, float dt);
	void renderTargetFbo();
	void threadedFunction() override;
	void resetWorkerStateLocked();
	bool buildGradeShader(bool useRect);
	void drawCameraIntoFbo(float drawX, float drawY, float drawW, float drawH);
	/// Shared detection overlay (box, center cross, presence/worker stats),
	/// mapping source pixels into the given on-screen image rect. dispX/dispY is
	/// the display-quad origin used for the state/heartbeat text.
	void drawDetectionOverlay(float dispX, float dispY,
		float imgX, float imgY, float imgW, float imgH,
		float srcW, float srcH) const;
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
	// Eye-box size smoothing (mode 2) so the stable anchor never breathes with
	// noisy Haar box dimensions.
	OneEuroFilter filterBoxW;
	OneEuroFilter filterBoxH;
	// Iris-center smoothing, kept separate from the box anchor so the iris stays
	// responsive (gaze source) while the box holds still.
	OneEuroFilter filterIrisX;
	OneEuroFilter filterIrisY;

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
