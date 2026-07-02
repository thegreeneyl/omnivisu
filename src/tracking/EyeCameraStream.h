#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "FrameSource.h"
#include "PlaybackSource.h"
#if !defined(OMNIVISU_NO_CAMERA)
#include "LiveCameraSource.h"
#endif
#include "OneEuroFilter.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

/// One eye stream: frame acquisition through an IFrameSource (live IDS camera
/// or disk playback, chosen at setup), eye detection (worker thread), and the
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

		// Frame source. When playbackEnabled is set (or the build defines
		// OMNIVISU_NO_CAMERA) the camera is never opened and frames come from
		// playbackFiles (this eye's index-aligned list, built by the shared
		// PlaybackTimeline). recordingFormat is only used while recording the
		// live camera.
		bool playbackEnabled = false;
		std::vector<std::string> playbackFiles;
		bool playbackLoop = true;
		std::string recordingFormat = "jpg";
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
	/// Normalized horizontal gaze of the pupil within the eye opening, in
	/// DISPLAY orientation: 0 = centered, negative = looking left, positive =
	/// looking right (mirror-corrected so it matches what the user sees).
	/// Defined as (irisCenter.x - eyeCenter.x) / (eyeBox.width / 2), i.e. the
	/// pupil center relative to the (offset-shifted) eye center. Returns 0 when
	/// there is no current detection or no fitted iris.
	float getGazeX() const;

#if !defined(OMNIVISU_NO_CAMERA)
	const ofxIdsPeak::Grabber & getGrabber() const { return liveSource->grabber(); }
	ofParameterGroup & getGrabberParameters() { return liveSource->grabber().parameters.group; }
#endif
	ofParameterGroup & getParameters() { return parameters.group; }
	ofParameterGroup & getTrackingParameters() { return parameters.trackingGroup; }
	ofParameterGroup & getGradingParameters() { return parameters.gradingGroup; }
	ofParameterGroup & getViewParameters() { return parameters.viewGroup; }
	ofParameterGroup & getCropParameters() { return parameters.cropGroup; }
	const std::string & getName() const { return config.name; }

	bool saveParameters() const;
	bool loadParameters();

	/// True when this stream is fed by on-disk recordings (camera not opened).
	bool isPlayback() const { return playback; }

	/// The playback source when in playback mode, nullptr in live capture.
	/// Transport (lockstep advance, stepping, seeking) is coordinated across
	/// eyes by the app-level PlaybackController through this pointer.
	PlaybackSource * playbackSource() { return playbackSrc.get(); }

	/// Recording of the raw camera frames to an image sequence in `dir`
	/// (relative to bin/data). No-op in playback mode. Writing happens on a
	/// background thread so capture is never stalled.
	void startRecording(const std::string & dir);
	void stopRecording();
	bool isRecording() const {
#if !defined(OMNIVISU_NO_CAMERA)
		return liveSource && liveSource->isRecording();
#else
		return false;
#endif
	}

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
		float darkness = 0.0f;     ///< 0..1: how much darker the interior is than
		                           ///< the surrounding ring (specular-highlight
		                           ///< pixels excluded). Higher = more iris-like.
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
		float darkness = 0.0f; ///< Chosen iris darkness margin, 0..1 (see IrisCandidate).

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
		// 2 = Haar eye box with classical iris located inside it (default),
		// 3 = Haar eye box with a MediaPipe-Iris ONNX model locating the iris
		//     inside it (falls back to mode 2 if the model is missing/fails),
		// 4 = MediaPipe-Iris ONNX model run directly on a centered region of
		//     the frame, with NO Haar step. Useful when Haar misses on a
		//     tightly-framed single-eye camera. Reports no detection if the
		//     model is unavailable.
		// 5 = Haar-seeded, iris-TRACKED DNN (recommended over 3): Haar is only
		//     used to (re)acquire; once locked, each frame crops a stable ROI
		//     around the eye and runs the model there, so per-frame Haar
		//     brittleness (the "downscale value" cliff) and extreme-gaze
		//     dropouts no longer break tracking. Falls back to mode 2 if the
		//     model is unavailable.
		ofParameter<int> detectorMode{"detector mode", 2, 0, 5};
		// Path (relative to bin/data) of the ONNX iris-landmark model used by
		// detector modes 3 and 4. Loaded lazily the first time it is needed.
		// Leave the file absent to make mode 3 transparently behave like mode 2.
		ofParameter<std::string> irisDnnModel{"iris dnn model", "models/iris_landmark.onnx"};
		// Mode 4 only: the search region is a centered rectangle covering this
		// fraction of the (cropped) frame. 1.0 = whole frame; lower values zoom
		// into the center so the eye fills more of the model's 64x64 input.
		ofParameter<float> irisRoiScale{"iris roi scale", 1.0f, 0.1f, 1.0f};
		// Mode 5 (tracked DNN) tuning:
		// - track margin: the tracked ROI side = max(seed eye-box side) * this.
		//   Larger = more room for the iris to travel at extreme gaze before it
		//   leaves the crop (at the cost of a smaller eye in the 64x64 input).
		// - track min quality: quality gate WHILE tracking. Deliberately looser
		//   than iris_min_fit_quality so a partly eyelid-occluded corner-gaze
		//   iris is kept instead of dropped.
		// - track lost frames: consecutive tracking failures tolerated before
		//   the tracker gives up and falls back to Haar re-acquisition.
		ofParameter<float> irisTrackMargin{"iris track margin", 1.8f, 1.0f, 4.0f};
		ofParameter<float> irisTrackMinQuality{"iris track min quality", 0.08f, 0.0f, 1.0f};
		ofParameter<int> irisTrackLostFrames{"iris track lost frames", 6, 1, 60};
		ofParameter<int> trackingDownscale{"tracking downscale", 8, 1, 16};
		ofParameter<int> haarMinWidth{"haar min width", 400, 8, 1000};
		ofParameter<int> haarMinHeight{"haar min height", 250, 8, 1000};
		ofParameter<float> haarScale{"haar scale", 1.01f, 1.001f, 1.5f};
		ofParameter<int> haarNeighbors{"haar neighbors", 5, 0, 20};
		// When set (mode 3, and mode 5 re-acquisition), run the Haar cascade at
		// several image resolutions each frame and merge the hits, instead of
		// only at 1/tracking_downscale. This smooths out the discontinuous "some
		// downscale values find nothing" behaviour of a single-scale cascade at
		// the cost of a few extra (cheap) detections per frame.
		ofParameter<bool> haarMultiScale{"haar multi scale", false};
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
		// Darkness gate: how much darker the ellipse interior must be than the
		// surrounding ring (0..1, normalized so 1.0 == ~128 gray levels). Rejects
		// bright features / specular reflections that pass the shape gates.
		ofParameter<float> irisMinDarkness{"iris min darkness", 0.05f, 0.0f, 1.0f};
		ofParameter<int> irisPreferredSide{"iris preferred side", 0, 0, 2};
		// Manual correction for the eye-box anchor, in *source* pixels. Haar eye
		// boxes sit slightly off; tune per eye. Positive X moves the center right
		// in the (mirror-aware) displayed image, positive Y moves it down. Shifts
		// the eye box/follow anchor; the iris is unaffected.
		ofParameter<float> eyeCenterOffsetX{"eye center offset x", 0.0f, -200.0f, 200.0f};
		ofParameter<float> eyeCenterOffsetY{"eye center offset y", 0.0f, -200.0f, 200.0f};
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

		// Per-edge crop of the raw camera frame, as a fraction of the full
		// sensor dimension (0 = no crop, 0.9 = trim 90% off that edge). Applied
		// to the frame *before* detection and rendering, so it both reduces the
		// inter-camera overlap seen by the detector and changes the displayed
		// image ratio. Directions are in DISPLAY orientation (mirror-aware), so
		// "right" is the right edge of what the user sees.
		ofParameterGroup cropGroup{"crop"};
		ofParameter<float> cropTop{"top", 0.0f, 0.0f, 0.9f};
		ofParameter<float> cropRight{"right", 0.0f, 0.0f, 0.9f};
		ofParameter<float> cropBottom{"bottom", 0.0f, 0.0f, 0.9f};
		ofParameter<float> cropLeft{"left", 0.0f, 0.0f, 0.9f};

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
	/// Null-safe reads through the active frame source (live camera or disk
	/// playback). Everything downstream reads frames through these.
	const ofPixels & sourcePixels() const;
	const ofTexture & sourceTexture() const;
	bool sourceFrameNew() const;
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
	/// Runs the Haar eye cascade on `graySrc` (full-res grayscale) at several
	/// down-sampling factors around tracking_downscale and appends every hit,
	/// mapped to source px, to `boxes`. Used by the multi-scale seeding path so
	/// a single Haar scale-cliff can't drop the eye. Uses its own classifier
	/// (irisHaarCascade) so the default single-scale eyeFinder path is untouched.
	void collectHaarBoxesMultiScale(const cv::Mat & graySrc, int srcW, int srcH,
		std::vector<ofRectangle> & boxes);
	/// Combined DNN detector (mode 3): Haar gives candidate eye boxes; a
	/// MediaPipe-Iris ONNX model locates the iris on the full-resolution RGB
	/// crop of each box. Boxes whose iris fails the plausibility gate are
	/// discarded. Falls back to detectEyeBoxIris() whenever the model is
	/// unavailable, so enabling mode 3 can never break tracking. `src` is the
	/// full (cropped) source frame in source-pixel coordinates.
	bool detectEyeBoxIrisDnn(const ofPixels & src, int downscale, int srcW, int srcH,
		RawDetection & out);
	/// Haar-free DNN detector (mode 4): runs the iris ONNX model on a single
	/// centered ROI of the full-resolution frame (size = parameters.irisRoiScale
	/// of the frame). The ROI itself is reported as the eye box (there is no
	/// Haar eye-opening estimate), so gaze is normalized against the search
	/// region. Reports no detection when the model is unavailable.
	bool detectIrisDnn(const ofPixels & src, int srcW, int srcH, RawDetection & out);
	/// Tracked DNN detector (mode 5): Haar (via detectEyeBoxIrisDnn) only
	/// (re)acquires the eye; once locked, crops a stable ROI around the eye and
	/// runs the model there every frame, following the iris without touching
	/// Haar. Falls back to detectEyeBoxIris() if the model is unavailable.
	bool detectIrisTracked(const ofPixels & src, int downscale, int srcW, int srcH,
		RawDetection & out);
	/// Runs the iris ONNX model on `roi` (a source-pixel rectangle) of the
	/// full-res `srcMat`, filling `out` (center/size/radius in source px, plus a
	/// symmetry-and-size fit quality). Returns false on any inference problem.
	bool computeIrisInRoi(const cv::Mat & srcMat, const ofRectangle & roi,
		IrisCandidate & out);
	/// Lazily loads the iris ONNX model named by parameters.irisDnnModel.
	/// Returns true once the net is ready; false (with a one-shot warning) when
	/// the file is missing or fails to parse. Runs on the detection worker.
	bool ensureIrisNet();
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
	void drawCameraIntoFbo(float drawX, float drawY, float drawW, float drawH,
		const ofRectangle & srcRect);
	/// Translates the per-edge crop fractions into a source-pixel sub-rectangle
	/// of the full frame (fullW x fullH). Mirror-aware: left/right are swapped
	/// when config.mirrorX is set so the directions match the displayed image.
	ofRectangle computeCropRectSource(float fullW, float fullH) const;
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

	// Frame acquisition. Exactly one source is *active* (`source`); which one
	// is decided once at setup by config.playbackEnabled / OMNIVISU_NO_CAMERA.
	// On camera builds the LiveCameraSource object always exists - even in
	// playback mode - so the grabber's parameter group keeps living in the
	// per-eye JSON/GUI unchanged; its camera is simply never opened then.
#if !defined(OMNIVISU_NO_CAMERA)
	std::unique_ptr<LiveCameraSource> liveSource;
#endif
	std::unique_ptr<PlaybackSource> playbackSrc; ///< Non-null only in playback mode.
	IFrameSource * source = nullptr; ///< The active source (points at one of the above).

	ofFbo targetFbo;
	ofShader gradeShader;
	bool gradeShaderLoaded = false;
	bool gradeShaderUsesRect = false;
	ofxCvColorImage cvColor;
	ofxCvGrayscaleImage cvGray;
	ofxCvHaarFinder eyeFinder;
	// Second copy of the eye cascade used only by the multi-scale seeding path
	// (collectHaarBoxesMultiScale). Kept separate from eyeFinder so the default
	// single-scale detection is byte-for-byte unchanged.
	cv::CascadeClassifier irisHaarCascade;

	// MediaPipe-Iris ONNX model for detector mode 3. Loaded lazily on the
	// worker thread the first time mode 3 runs, and only used there, so no
	// synchronization is needed. `irisNetTriedPath` records the last path we
	// attempted so the "missing/failed" warning is logged only once per path.
	cv::dnn::Net irisNet;
	bool irisNetReady = false;
	std::string irisNetTriedPath;
	ofPixels detectionPixels; ///< Worker-only scratch buffer for downsampled tracking input.

	RawDetection lastRaw;
	ofRectangle lastEyeBox;
	bool hasLastEyeBox = false;

	// Mode 5 tracker state (worker-thread only, like lastEyeBox). While
	// irisTrackValid is set we skip Haar and crop around irisTrackEyeCenter.
	// irisTrackEyeBox is the eye box captured at acquisition and is kept as the
	// stable gaze-normalization reference so gaze scale matches mode 3.
	bool irisTrackValid = false;
	glm::vec2 irisTrackEyeCenter{0.0f, 0.0f};
	ofRectangle irisTrackEyeBox;
	int irisTrackMiss = 0;

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

	/// True when frames come from disk (mirrors source->isPlayback()).
	bool playback = false;

	// Dimensions (px) of the cropped frame handed to the most recent detection
	// job, so the worker normalizes distances against the cropped frame rather
	// than the full sensor. 0 until the first cropped frame is dispatched.
	std::atomic<int> activeCropW{0};
	std::atomic<int> activeCropH{0};
};
