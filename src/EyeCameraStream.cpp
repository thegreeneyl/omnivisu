#include "EyeCameraStream.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr float kPi = 3.14159265358979323846f;

float clamp01(float v) {
	return std::max(0.0f, std::min(1.0f, v));
}

ofxIdsPeak::DeviceSelect deviceSelectFromString(const std::string & s) {
	if (s == "serial") return ofxIdsPeak::DeviceSelect::Serial;
	if (s == "model") return ofxIdsPeak::DeviceSelect::Model;
	if (s == "index") return ofxIdsPeak::DeviceSelect::Index;
	return ofxIdsPeak::DeviceSelect::Any;
}

float rectDistanceSq(const ofRectangle & a, const ofRectangle & b) {
	const float ax = a.x + a.width * 0.5f;
	const float ay = a.y + a.height * 0.5f;
	const float bx = b.x + b.width * 0.5f;
	const float by = b.y + b.height * 0.5f;
	const float dx = ax - bx;
	const float dy = ay - by;
	return dx * dx + dy * dy;
}
} // namespace

//--------------------------------------------------------------
void EyeCameraStream::setupParameters() {
	parameters.group.setName(config.name);
	parameters.group.add(grabber.parameters.group);

	parameters.trackingGroup.clear();
	parameters.trackingGroup.setName("tracking");
	parameters.trackingGroup.add(parameters.enableEyeTracking);
	parameters.trackingGroup.add(parameters.showDebugOverlay);
	parameters.trackingGroup.add(parameters.detectorMode);
	parameters.trackingGroup.add(parameters.trackingDownscale);
	parameters.trackingGroup.add(parameters.haarMinWidth);
	parameters.trackingGroup.add(parameters.haarMinHeight);
	parameters.trackingGroup.add(parameters.haarScale);
	parameters.trackingGroup.add(parameters.haarNeighbors);
	parameters.trackingGroup.add(parameters.irisExpectedRadius);
	parameters.trackingGroup.add(parameters.irisRadiusTolerance);
	parameters.trackingGroup.add(parameters.irisThresholdMode);
	parameters.trackingGroup.add(parameters.irisThreshold);
	parameters.trackingGroup.add(parameters.irisBlur);
	parameters.trackingGroup.add(parameters.irisMinFitQuality);
	parameters.trackingGroup.add(parameters.irisMinCircularity);
	parameters.trackingGroup.add(parameters.irisMinDarkness);
	parameters.trackingGroup.add(parameters.irisPreferredSide);
	parameters.trackingGroup.add(parameters.eyeCenterOffsetX);
	parameters.trackingGroup.add(parameters.eyeCenterOffsetY);
	parameters.trackingGroup.add(parameters.debugShowCandidates);
	parameters.trackingGroup.add(parameters.presentOnFrames);
	parameters.trackingGroup.add(parameters.presentOffFrames);
	parameters.trackingGroup.add(parameters.euroMinCutoff);
	parameters.trackingGroup.add(parameters.euroBeta);
	parameters.group.add(parameters.trackingGroup);

	parameters.gradingGroup.clear();
	parameters.gradingGroup.setName("grading");
	parameters.gradingGroup.add(parameters.enableGrading);
	parameters.gradingGroup.add(parameters.gradeExposure);
	parameters.gradingGroup.add(parameters.gradeBrightness);
	parameters.gradingGroup.add(parameters.gradeContrast);
	parameters.gradingGroup.add(parameters.gradeGamma);
	parameters.gradingGroup.add(parameters.gradeSaturation);
	parameters.group.add(parameters.gradingGroup);

	parameters.viewGroup.clear();
	parameters.viewGroup.setName("view");
	parameters.viewGroup.add(parameters.viewScale);
	parameters.viewGroup.add(parameters.fitToFill);
	parameters.viewGroup.add(parameters.followEye);
	parameters.viewGroup.add(parameters.followSmoothing);
	parameters.group.add(parameters.viewGroup);

	parameters.cropGroup.clear();
	parameters.cropGroup.setName("crop");
	parameters.cropGroup.add(parameters.cropTop);
	parameters.cropGroup.add(parameters.cropRight);
	parameters.cropGroup.add(parameters.cropBottom);
	parameters.cropGroup.add(parameters.cropLeft);
	parameters.group.add(parameters.cropGroup);

	parameters.fboGroup.clear();
	parameters.fboGroup.setName("fbo");
	parameters.fboGroup.add(parameters.fboWidth);
	parameters.fboGroup.add(parameters.fboHeight);
	parameters.group.add(parameters.fboGroup);

	parameters.cameraGroup.clear();
	parameters.cameraGroup.setName("camera");
	parameters.cameraGroup.add(parameters.selectBy);
	parameters.cameraGroup.add(parameters.selectValue);
	parameters.group.add(parameters.cameraGroup);
}

//--------------------------------------------------------------
bool EyeCameraStream::setup(const Config & cfg) {
	config = cfg;
	setupParameters();

	// Camera publishes RGB color frames so the rendered FBO stays RGB; the
	// detection worker desaturates internally via ofxCvGrayscaleImage.
	grabber.applyLowLatencyPreset(ofxIdsPeak::OutputFormat::RGB);

	// Load tunable params from JSON before the grabber initializes so the
	// camera comes up with the persisted exposure/gain/WB/etc. values.
	loadParameters();

	// Translate the JSON "camera" block into the grabber's device selector so
	// this eye binds to a specific physical camera (by serial/model/index).
	const ofxIdsPeak::DeviceSelect selectMode = deviceSelectFromString(parameters.selectBy.get());
	const std::string selectValue = parameters.selectValue.get();
	int selectIndex = 0;
	if (selectMode == ofxIdsPeak::DeviceSelect::Index) {
		try {
			selectIndex = std::stoi(selectValue);
		} catch (const std::exception &) {
			ofLogWarning("EyeCameraStream") << config.name
				<< ": camera select index '" << selectValue << "' is not an integer; using 0";
		}
	}
	grabber.setDeviceSelector(selectMode, selectValue, selectIndex);

	const bool grabberOk = grabber.setup();
	if (!grabberOk) {
		ofLogError("EyeCameraStream") << config.name << ": failed to initialize IDS peak camera";
	}

	const of::filesystem::path cascadePath = ofToDataPath(config.cascadeFile, true);
	if (!ofFile::doesFileExist(cascadePath)) {
		ofLogError("EyeCameraStream")
			<< config.name << ": cascade not found: " << cascadePath
			<< " (copy haarcascade_eye.xml into bin/data/)";
		return false;
	}

	try {
		eyeFinder.setup(config.cascadeFile);
	} catch (const cv::Exception & e) {
		ofLogError("EyeCameraStream")
			<< config.name << ": failed to load cascade " << cascadePath << ": " << e.what();
		return false;
	}
	eyeFinder.setScaleHaar(parameters.haarScale);
	eyeFinder.setNeighbors(parameters.haarNeighbors.get());

	cvColor.setUseTexture(false);
	cvGray.setUseTexture(false);

	// FBO dimensions come from the per-eye JSON (loadParameters above). Fall
	// back to the Config defaults if the file omits them (params default to
	// 672x504, matching Config::fboSize).
	config.fboSize.x = std::max(1, parameters.fboWidth.get());
	config.fboSize.y = std::max(1, parameters.fboHeight.get());
	const int w = config.fboSize.x;
	const int h = config.fboSize.y;
	targetFbo.allocate(w, h, GL_RGB);

	const float minCutoff = parameters.euroMinCutoff;
	const float beta = parameters.euroBeta;
	filterEyeX.setup(minCutoff, beta);
	filterEyeY.setup(minCutoff, beta);
	filterBoxW.setup(minCutoff, beta);
	filterBoxH.setup(minCutoff, beta);
	filterIrisX.setup(minCutoff, beta);
	filterIrisY.setup(minCutoff, beta);

	workerLastFrameTime = ofGetElapsedTimef();
	lastFollowTime = workerLastFrameTime;
	startThread();
	ofLogNotice("EyeCameraStream") << config.name << ": detection worker thread started";

	// Shader is (re)built lazily the first time we render, when the grabber's
	// texture target is known (depends on ofGetUsingArbTex() and driver).
	gradeShaderLoaded = false;
	return grabberOk;
}

//--------------------------------------------------------------
bool EyeCameraStream::buildGradeShader(bool useRect) {
	std::string defines;
	if (useRect) {
		defines = "#define USE_RECT 1\n";
	}

	const std::string vert =
		"#version 150\n"
		+ defines +
		"uniform mat4 modelViewProjectionMatrix;\n"
		"uniform mat4 textureMatrix;\n"
		"in vec4 position;\n"
		"in vec2 texcoord;\n"
		"out vec2 vTexCoord;\n"
		"void main() {\n"
		"    vTexCoord = (textureMatrix * vec4(texcoord, 0.0, 1.0)).xy;\n"
		"    gl_Position = modelViewProjectionMatrix * position;\n"
		"}\n";

	const std::string frag =
		"#version 150\n"
		+ defines +
		"#ifdef USE_RECT\n"
		"uniform sampler2DRect tex0;\n"
		"#else\n"
		"uniform sampler2D tex0;\n"
		"#endif\n"
		"uniform float uExposure;\n"
		"uniform float uBrightness;\n"
		"uniform float uContrast;\n"
		"uniform float uGamma;\n"
		"uniform float uSaturation;\n"
		"in vec2 vTexCoord;\n"
		"out vec4 outColor;\n"
		"void main() {\n"
		"    vec3 c = texture(tex0, vTexCoord).rgb;\n"
		"    c *= exp2(uExposure);\n"
		"    c += vec3(uBrightness);\n"
		"    c = (c - vec3(0.5)) * uContrast + vec3(0.5);\n"
		"    c = pow(max(c, vec3(0.0)), vec3(1.0 / uGamma));\n"
		"    float luma = dot(c, vec3(0.299, 0.587, 0.114));\n"
		"    c = mix(vec3(luma), c, uSaturation);\n"
		"    c = clamp(c, vec3(0.0), vec3(1.0));\n"
		"    outColor = vec4(c, 1.0);\n"
		"}\n";

	gradeShader.unload();
	bool ok = true;
	ok = ok && gradeShader.setupShaderFromSource(GL_VERTEX_SHADER, vert);
	ok = ok && gradeShader.setupShaderFromSource(GL_FRAGMENT_SHADER, frag);
	ok = ok && gradeShader.bindDefaults();
	ok = ok && gradeShader.linkProgram();

	if (ok) {
		ofLogNotice("EyeCameraStream") << config.name
			<< ": grade shader built (sampler="
			<< (useRect ? "sampler2DRect" : "sampler2D") << ")";
	} else {
		ofLogWarning("EyeCameraStream") << config.name
			<< ": grade shader failed to build; falling back to direct draw";
	}
	gradeShaderUsesRect = useRect;
	return ok;
}

//--------------------------------------------------------------
std::string EyeCameraStream::paramsFilePath() const {
	return config.paramsFile.empty() ? config.name + ".json" : config.paramsFile;
}

//--------------------------------------------------------------
ofRectangle EyeCameraStream::computeCropRectSource(float fullW, float fullH) const {
	if (fullW <= 0.0f || fullH <= 0.0f) {
		return ofRectangle(0.0f, 0.0f, std::max(0.0f, fullW), std::max(0.0f, fullH));
	}

	float top = clamp01(parameters.cropTop.get());
	float right = clamp01(parameters.cropRight.get());
	float bottom = clamp01(parameters.cropBottom.get());
	float left = clamp01(parameters.cropLeft.get());

	// Never crop the frame away entirely: keep at least a thin sliver on each
	// axis by scaling opposing edges down if they together reach the full size.
	constexpr float kMaxAxis = 0.95f;
	if (left + right > kMaxAxis) {
		const float s = kMaxAxis / (left + right);
		left *= s;
		right *= s;
	}
	if (top + bottom > kMaxAxis) {
		const float s = kMaxAxis / (top + bottom);
		top *= s;
		bottom *= s;
	}

	// Crop directions are display-oriented; the sensor is non-mirrored, so swap
	// left/right when the display is mirrored (matches eye_center_offset_x).
	const float srcLeftFrac = config.mirrorX ? right : left;
	const float srcRightFrac = config.mirrorX ? left : right;

	const float x = std::round(srcLeftFrac * fullW);
	const float y = std::round(top * fullH);
	const float w = std::max(1.0f, std::round(fullW - (srcLeftFrac + srcRightFrac) * fullW));
	const float h = std::max(1.0f, std::round(fullH - (top + bottom) * fullH));
	return ofRectangle(x, y, w, h);
}

//--------------------------------------------------------------
bool EyeCameraStream::loadParameters() {
	const auto path = ofToDataPath(paramsFilePath(), true);
	if (!ofFile::doesFileExist(path)) {
		ofLogError("EyeCameraStream") << config.name
			<< ": params file not found at " << path
			<< " - using bare ofParameter declarations";
		return false;
	}
	const ofJson json = ofLoadJson(path);
	// At runtime the grabber's capture thread is live; a bulk ofDeserialize into
	// the parameter group would fire every grabber listener (a storm of
	// applyParameters() plus a reconfigureOutputFormat() buffer realloc) and
	// crash. Bracket the write so the camera parameters are applied once, safely.
	// During initial setup the grabber is not yet initialized, so this is a no-op.
	const bool live = grabber.isInitialized();
	if (live) {
		grabber.beginParameterBatch();
	}
	ofDeserialize(json, parameters.group);
	if (live) {
		grabber.endParameterBatch();
	}
	ofLogNotice("EyeCameraStream") << config.name << ": params loaded from " << path;
	return true;
}

//--------------------------------------------------------------
bool EyeCameraStream::saveParameters() const {
	const auto path = ofToDataPath(paramsFilePath(), true);
	ofJson json;
	ofSerialize(json, parameters.group);
	if (!ofSavePrettyJson(path, json)) {
		ofLogError("EyeCameraStream") << config.name << ": failed to save " << path;
		return false;
	}
	ofLogNotice("EyeCameraStream") << config.name << ": params saved to " << path;
	return true;
}

//--------------------------------------------------------------
EyeCameraStream::~EyeCameraStream() {
	frameJobs.close();
	if (isThreadRunning()) {
		stopThread();
		waitForThread(false);
	}
}

//--------------------------------------------------------------
void EyeCameraStream::resetWorkerStateLocked() {
	// Caller must hold stateMutex.
	result = {};
	lastRaw = {};
}

//--------------------------------------------------------------
ofxCvBlob EyeCameraStream::pickBestEyeBlob(const std::vector<ofxCvBlob> & blobs) const {
	if (blobs.empty()) {
		return {};
	}

	// Normalize against the cropped detection frame (what the worker actually
	// sees), falling back to the full sensor before the first cropped dispatch.
	const float frameW = activeCropW.load() > 0 ? static_cast<float>(activeCropW.load())
		: static_cast<float>(grabber.getPixels().getWidth());
	const float frameH = activeCropH.load() > 0 ? static_cast<float>(activeCropH.load())
		: static_cast<float>(grabber.getPixels().getHeight());
	const ofRectangle frame(0, 0, frameW, frameH);
	const float frameDiagSq = frame.width * frame.width + frame.height * frame.height;

	ofxCvBlob best = blobs.front();
	float bestScore = -1.0f;

	for (const auto & blob : blobs) {
		const float area = blob.boundingRect.width * blob.boundingRect.height;
		const float areaNorm = area / std::max(1.0f, frame.width * frame.height);

		float temporalScore = 0.0f;
		if (hasLastEyeBox) {
			const float distSq = rectDistanceSq(blob.boundingRect, lastEyeBox);
			const float distNorm = distSq / std::max(1.0f, frameDiagSq);
			temporalScore = 1.0f - clamp01(distNorm * 4.0f);
		}

		const float score = areaNorm * 0.55f + temporalScore * 0.45f;
		if (score > bestScore) {
			bestScore = score;
			best = blob;
		}
	}

	return best;
}

//--------------------------------------------------------------
bool EyeCameraStream::detectEye(const ofPixels & pixels, RawDetection & out) {
	out = {};

	if (!pixels.isAllocated() || pixels.getWidth() <= 0 || pixels.getHeight() <= 0) {
		return false;
	}

	// Optionally downsample the tracking input. Display path is unaffected -
	// this only resizes the worker copy of the frame before the cascade runs.
	const int srcW = static_cast<int>(pixels.getWidth());
	const int srcH = static_cast<int>(pixels.getHeight());
	const int downscale = std::max(1, parameters.trackingDownscale.get());
	const int detW = std::max(1, srcW / downscale);
	const int detH = std::max(1, srcH / downscale);

	const ofPixels * detectionInput = &pixels;
	if (downscale > 1) {
		// ofPixels::resize bilinear path is not implemented in OF 0.12, so we
		// downsample with OpenCV. INTER_AREA is the recommended filter for
		// downsampling: it averages source pixels, giving cleaner low-pass
		// behavior than nearest-neighbor without ringing artifacts.
		const ofImageType ofType = pixels.getImageType();
		if (!detectionPixels.isAllocated()
			|| static_cast<int>(detectionPixels.getWidth()) != detW
			|| static_cast<int>(detectionPixels.getHeight()) != detH
			|| detectionPixels.getImageType() != ofType) {
			detectionPixels.allocate(detW, detH, ofType);
		}

		const int channels = static_cast<int>(pixels.getNumChannels());
		const int cvType = (channels == 1) ? CV_8UC1
						: (channels == 3) ? CV_8UC3
						: (channels == 4) ? CV_8UC4 : 0;
		if (cvType == 0) {
			ofLogWarning("EyeCameraStream") << "tracking downscale: unsupported channel count "
				<< channels << ", falling back to full-resolution detection";
		} else {
			cv::Mat srcMat(srcH, srcW, cvType,
				const_cast<unsigned char *>(pixels.getData()));
			cv::Mat dstMat(detH, detW, cvType, detectionPixels.getData());
			cv::resize(srcMat, dstMat, cv::Size(detW, detH), 0, 0, cv::INTER_AREA);
			detectionInput = &detectionPixels;
		}
	}

	// ofxCvImage::setFromPixels(const ofPixels&) ignores channel count and copies
	// raw bytes, which corrupts the grayscale image when given RGB input. Route
	// RGB through ofxCvColorImage so operator= performs the proper color->gray.
	if (detectionInput->getNumChannels() >= 3) {
		if (!cvColor.bAllocated
			|| static_cast<int>(cvColor.getWidth()) != detW
			|| static_cast<int>(cvColor.getHeight()) != detH) {
			cvColor.allocate(detW, detH);
		}
		// Reallocate cvGray to match the new size too, otherwise the operator=
		// rejects the assignment with "region of interest mismatch" the first
		// time the user changes tracking downscale at runtime.
		if (!cvGray.bAllocated
			|| static_cast<int>(cvGray.getWidth()) != detW
			|| static_cast<int>(cvGray.getHeight()) != detH) {
			cvGray.allocate(detW, detH);
		}
		cvColor.setFromPixels(*detectionInput);
		cvGray = cvColor;
	} else {
		if (!cvGray.bAllocated
			|| static_cast<int>(cvGray.getWidth()) != detW
			|| static_cast<int>(cvGray.getHeight()) != detH) {
			cvGray.allocate(detW, detH);
		}
		cvGray.setFromPixels(*detectionInput);
	}

	// Dispatch to the selected detector. All operate on the prepared `cvGray`
	// detection image and report results in source-pixel coordinates.
	const int mode = parameters.detectorMode.get();
	if (mode == 1) {
		return detectHaar(downscale, srcW, srcH, out);
	}
	if (mode == 2) {
		return detectEyeBoxIris(downscale, srcW, srcH, out);
	}
	return detectIris(downscale, out);
}

//--------------------------------------------------------------
bool EyeCameraStream::detectHaar(int downscale, int srcW, int srcH, RawDetection & out) {
	eyeFinder.setScaleHaar(parameters.haarScale);
	eyeFinder.setNeighbors(parameters.haarNeighbors);

	// User-facing haar min size is in *source pixels* for intuitive tuning,
	// so we divide by the downscale factor before passing to the cascade.
	const int minW = std::max(1, parameters.haarMinWidth.get() / downscale);
	const int minH = std::max(1, parameters.haarMinHeight.get() / downscale);
	const int n = eyeFinder.findHaarObjects(cvGray, minW, minH);
	if (n <= 0 || eyeFinder.blobs.empty()) {
		return false;
	}

	// Cascade returns blobs in detection-image coordinates. Map back to source
	// pixels so the picker, smoothing, and overlay all work in the same space.
	if (downscale > 1) {
		const float s = static_cast<float>(downscale);
		for (auto & blob : eyeFinder.blobs) {
			blob.boundingRect.x *= s;
			blob.boundingRect.y *= s;
			blob.boundingRect.width *= s;
			blob.boundingRect.height *= s;
			blob.centroid.x *= s;
			blob.centroid.y *= s;
		}
	}

	for (const auto & blob : eyeFinder.blobs) {
		out.candidateBoxes.push_back(blob.boundingRect); // debug overlay.
	}

	const ofxCvBlob eyeBlob = pickBestEyeBlob(eyeFinder.blobs);
	if (eyeBlob.boundingRect.width <= 0.0f || eyeBlob.boundingRect.height <= 0.0f) {
		return false;
	}

	out.eyeBox = eyeBlob.boundingRect;
	out.eyeCenter = {eyeBlob.centroid.x, eyeBlob.centroid.y};
	out.valid = true;

	const float areaNorm = (out.eyeBox.width * out.eyeBox.height)
		/ std::max(1.0f, static_cast<float>(srcW * srcH));
	out.confidence = clamp01(areaNorm * 25.0f);

	lastEyeBox = out.eyeBox;
	hasLastEyeBox = true;
	return true;
}

//--------------------------------------------------------------
void EyeCameraStream::collectIrisCandidates(const cv::Mat & gray, int downscale,
	float offX, float offY, std::vector<IrisCandidate> & out) const {
	out.clear();
	if (gray.empty() || gray.cols < 5 || gray.rows < 5) {
		return;
	}

	int blurK = std::max(1, parameters.irisBlur.get());
	if ((blurK % 2) == 0) {
		++blurK; // Gaussian kernel size must be odd.
	}
	cv::Mat blurred;
	if (blurK > 1) {
		cv::GaussianBlur(gray, blurred, cv::Size(blurK, blurK), 0.0);
	} else {
		blurred = gray;
	}

	// The iris/limbus is darker than the surrounding sclera/skin, so we threshold
	// the *dark* pixels (BINARY_INV) into blobs.
	const float expectedDet = std::max(1.0f,
		parameters.irisExpectedRadius.get() / static_cast<float>(std::max(1, downscale)));
	cv::Mat bin;
	const int tmode = parameters.irisThresholdMode.get();
	if (tmode == 0) {
		int block = std::max(3, static_cast<int>(expectedDet));
		if ((block % 2) == 0) {
			++block; // adaptiveThreshold block size must be odd.
		}
		// Block size must stay smaller than the (possibly small ROI) image.
		const int maxBlock = (std::min(gray.cols, gray.rows) - 1) | 1;
		block = std::max(3, std::min(block, maxBlock));
		cv::adaptiveThreshold(blurred, bin, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
			cv::THRESH_BINARY_INV, block, 5.0);
	} else if (tmode == 1) {
		cv::threshold(blurred, bin, 0.0, 255.0, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	} else {
		cv::threshold(blurred, bin, parameters.irisThreshold.get(), 255.0, cv::THRESH_BINARY_INV);
	}

	// Morphological open+close to drop specks and seal the iris blob. Kernel is a
	// small fraction of the expected iris so it never erases the iris itself.
	const int kn = std::max(1, static_cast<int>(expectedDet) / 8);
	const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * kn + 1, 2 * kn + 1));
	cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);
	cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.empty()) {
		return;
	}

	const float tol = clamp01(parameters.irisRadiusTolerance.get());
	const float minR = expectedDet * (1.0f - tol);
	const float maxR = expectedDet * (1.0f + tol);
	const float minArea = kPi * minR * minR * 0.3f;
	const float s = static_cast<float>(downscale);

	out.reserve(contours.size());
	for (const auto & c : contours) {
		if (c.size() < 5) {
			continue; // fitEllipse requires at least 5 points.
		}
		const double area = cv::contourArea(c);
		if (area < minArea) {
			continue;
		}
		const cv::RotatedRect e = cv::fitEllipse(c);
		const float a = e.size.width;
		const float b = e.size.height;
		if (a <= 0.0f || b <= 0.0f) {
			continue;
		}
		const float rmean = (a + b) * 0.25f;
		if (rmean < minR || rmean > maxR) {
			continue; // fixed-scale size gate: kills chin/ear/eyebrow blobs.
		}

		// Roundness gate: reject elongated ellipses outright. aspect is the
		// minor/major axis ratio (1 = perfect circle).
		const float aspect = std::min(a, b) / std::max(a, b);
		if (aspect < parameters.irisMinCircularity.get()) {
			continue;
		}

		// Fit quality: how round (aspect), how compact (circularity), and how
		// well the contour area matches its fitted ellipse area.
		const double perim = cv::arcLength(c, true);
		const float circ = (perim > 0.0)
			? static_cast<float>(4.0 * kPi * area / (perim * perim))
			: 0.0f;
		const float ellipseArea = kPi * (a * 0.5f) * (b * 0.5f);
		const float areaRatio = (ellipseArea > 0.0f)
			? std::min(static_cast<float>(area), ellipseArea) / std::max(static_cast<float>(area), ellipseArea)
			: 0.0f;
		const float fitQuality = clamp01(0.4f * aspect + 0.3f * clamp01(circ) + 0.3f * areaRatio);

		// Darkness check: an iris/pupil is darker than the surrounding eyeball.
		// Compare the mean intensity of the ellipse interior against a ring just
		// outside it, excluding near-saturated (specular highlight) pixels so a
		// glint inside the pupil can't brighten the interior. This is what stops
		// the detector from latching onto reflections that merely look round.
		float darkness = 0.0f;
		{
			constexpr double kHighlightCutoff = 245.0; // ignore specular glints.
			cv::Mat innerMask = cv::Mat::zeros(blurred.size(), CV_8UC1);
			cv::ellipse(innerMask, cv::RotatedRect(e.center, cv::Size2f(a * 0.7f, b * 0.7f), e.angle),
				cv::Scalar(255), -1);
			cv::Mat irisMask = cv::Mat::zeros(blurred.size(), CV_8UC1);
			cv::ellipse(irisMask, e, cv::Scalar(255), -1);
			cv::Mat outerMask = cv::Mat::zeros(blurred.size(), CV_8UC1);
			cv::ellipse(outerMask, cv::RotatedRect(e.center, cv::Size2f(a * 1.8f, b * 1.8f), e.angle),
				cv::Scalar(255), -1);
			cv::Mat ringMask = outerMask & ~irisMask;

			cv::Mat notHighlight = blurred < kHighlightCutoff;
			const cv::Mat innerValid = innerMask & notHighlight;
			const cv::Mat ringValid = ringMask & notHighlight;
			if (cv::countNonZero(innerValid) > 0 && cv::countNonZero(ringValid) > 0) {
				const double innerMean = cv::mean(blurred, innerValid)[0];
				const double ringMean = cv::mean(blurred, ringValid)[0];
				// Normalize so ~128 gray levels of margin maps to 1.0.
				darkness = clamp01(static_cast<float>(ringMean - innerMean) / 128.0f);
			}
		}
		if (darkness < parameters.irisMinDarkness.get()) {
			continue; // interior not darker than surroundings -> likely a reflection.
		}

		// Map ROI-local detection px -> full source px: (local + off) * downscale.
		const float cxDet = e.center.x + offX;
		const float cyDet = e.center.y + offY;
		IrisCandidate cand;
		cand.center = {cxDet * s, cyDet * s};
		cand.size = {a * s, b * s};
		cand.angleDeg = e.angle;
		cand.darkness = darkness;
		cand.radius = rmean * s;
		cand.fitQuality = fitQuality;
		cand.box = ofRectangle((cxDet - a * 0.5f) * s, (cyDet - b * 0.5f) * s, a * s, b * s);
		out.push_back(cand);
	}
}

//--------------------------------------------------------------
bool EyeCameraStream::detectIris(int downscale, RawDetection & out) {
	// `cvGray` holds the (downscaled) single-channel detection image. Wrap its
	// pixels in a cv::Mat (no copy) for the OpenCV pipeline.
	ofPixels & gp = cvGray.getPixels();
	const int detW = static_cast<int>(gp.getWidth());
	const int detH = static_cast<int>(gp.getHeight());
	if (detW <= 0 || detH <= 0 || gp.getNumChannels() != 1 || gp.getData() == nullptr) {
		return false;
	}
	cv::Mat gray(detH, detW, CV_8UC1, gp.getData());

	std::vector<IrisCandidate> cands;
	collectIrisCandidates(gray, downscale, 0.0f, 0.0f, cands);
	out.candidateIris = cands; // debug overlay: show every candidate.
	if (cands.empty()) {
		return false;
	}

	const IrisCandidate best = pickBestIris(cands);
	// Presence gate: require a confident-enough ellipse. The size band was
	// already enforced above; preferred-side is folded into the scorer.
	if (best.radius <= 0.0f || best.fitQuality < parameters.irisMinFitQuality.get()) {
		return false;
	}

	out.eyeBox = best.box;
	out.eyeCenter = best.center;
	out.irisCenter = best.center;
	out.irisSize = best.size;
	out.irisAngleDeg = best.angleDeg;
	out.irisRadiusPx = best.radius;
	out.fitQuality = best.fitQuality;
	out.darkness = best.darkness;
	out.confidence = best.fitQuality;
	out.valid = true;

	lastEyeBox = out.eyeBox;
	hasLastEyeBox = true;
	return true;
}

//--------------------------------------------------------------
bool EyeCameraStream::detectEyeBoxIris(int downscale, int srcW, int srcH, RawDetection & out) {
	// 1. Haar gives candidate eye-opening boxes (in detection px).
	eyeFinder.setScaleHaar(parameters.haarScale);
	eyeFinder.setNeighbors(parameters.haarNeighbors);
	const int minW = std::max(1, parameters.haarMinWidth.get() / downscale);
	const int minH = std::max(1, parameters.haarMinHeight.get() / downscale);
	const int n = eyeFinder.findHaarObjects(cvGray, minW, minH);
	if (n <= 0 || eyeFinder.blobs.empty()) {
		return false;
	}

	ofPixels & gp = cvGray.getPixels();
	const int detW = static_cast<int>(gp.getWidth());
	const int detH = static_cast<int>(gp.getHeight());
	if (detW <= 0 || detH <= 0 || gp.getNumChannels() != 1 || gp.getData() == nullptr) {
		return false;
	}
	cv::Mat gray(detH, detW, CV_8UC1, gp.getData());

	const float s = static_cast<float>(downscale);
	const ofRectangle frame(0, 0, srcW, srcH);
	const float frameDiagSq = frame.width * frame.width + frame.height * frame.height;

	// 2. For each box, locate the best iris inside it. Boxes without a valid
	// iris are discarded - this is what removes chin/ear/eyebrow false positives.
	bool found = false;
	float bestScore = -std::numeric_limits<float>::max();
	RawDetection bestOut;
	std::vector<IrisCandidate> cands;
	std::vector<ofRectangle> allBoxes;   // debug: every Haar box considered.
	std::vector<IrisCandidate> allIris;  // debug: every iris ellipse found.
	for (const auto & blob : eyeFinder.blobs) {
		// Clamp the Haar box to the detection image before cropping the ROI.
		int rx = static_cast<int>(std::floor(blob.boundingRect.x));
		int ry = static_cast<int>(std::floor(blob.boundingRect.y));
		int rw = static_cast<int>(std::ceil(blob.boundingRect.width));
		int rh = static_cast<int>(std::ceil(blob.boundingRect.height));
		rx = std::max(0, std::min(rx, detW - 1));
		ry = std::max(0, std::min(ry, detH - 1));
		rw = std::max(1, std::min(rw, detW - rx));
		rh = std::max(1, std::min(rh, detH - ry));
		if (rw < 5 || rh < 5) {
			continue;
		}

		// Eye box + its center in source px (the stable anchor).
		const ofRectangle eyeBox(blob.boundingRect.x * s, blob.boundingRect.y * s,
			blob.boundingRect.width * s, blob.boundingRect.height * s);
		const glm::vec2 boxCenter(eyeBox.x + eyeBox.width * 0.5f,
			eyeBox.y + eyeBox.height * 0.5f);
		allBoxes.push_back(eyeBox);

		const cv::Mat roi = gray(cv::Rect(rx, ry, rw, rh));
		collectIrisCandidates(roi, downscale, static_cast<float>(rx), static_cast<float>(ry), cands);
		allIris.insert(allIris.end(), cands.begin(), cands.end());
		if (cands.empty()) {
			continue;
		}

		const IrisCandidate iris = pickBestIris(cands);
		if (iris.radius <= 0.0f || iris.fitQuality < parameters.irisMinFitQuality.get()) {
			continue; // no valid iris in this box -> reject the box.
		}

		// 3. Score the (box, iris) pair: iris confidence + box temporal proximity,
		// with the preferred side as a dominant, near-hard preference so the
		// correct eye wins when two are in frame.
		float temporalScore = 0.0f;
		if (hasLastEyeBox) {
			const float distNorm = rectDistanceSq(eyeBox, lastEyeBox) / std::max(1.0f, frameDiagSq);
			temporalScore = 1.0f - clamp01(distNorm * 4.0f);
		}
		const float sidePenalty = onPreferredSide(boxCenter.x, frame.width) ? 0.0f : -10.0f;
		const float score = iris.fitQuality * 0.4f
			+ iris.darkness * 0.3f
			+ temporalScore * 0.3f
			+ sidePenalty;

		if (score > bestScore) {
			bestScore = score;
			bestOut = {};
			bestOut.eyeBox = eyeBox;
			bestOut.eyeCenter = boxCenter;
			bestOut.irisCenter = iris.center;
			bestOut.irisSize = iris.size;
			bestOut.irisAngleDeg = iris.angleDeg;
			bestOut.irisRadiusPx = iris.radius;
			bestOut.fitQuality = iris.fitQuality;
			bestOut.darkness = iris.darkness;
			bestOut.confidence = iris.fitQuality;
			bestOut.valid = true;
			found = true;
		}
	}

	// Always publish the candidate lists for the debug overlay, even when no box
	// survived the gates, so the user can see what was rejected.
	if (!found) {
		out.candidateBoxes = std::move(allBoxes);
		out.candidateIris = std::move(allIris);
		return false;
	}

	out = bestOut;
	out.candidateBoxes = std::move(allBoxes);
	out.candidateIris = std::move(allIris);
	lastEyeBox = out.eyeBox;
	hasLastEyeBox = true;
	return true;
}

//--------------------------------------------------------------
EyeCameraStream::IrisCandidate EyeCameraStream::pickBestIris(
	const std::vector<IrisCandidate> & cands) const {
	if (cands.empty()) {
		return {};
	}

	// Normalize against the cropped detection frame (see pickBestEyeBlob).
	const float frameW = activeCropW.load() > 0 ? static_cast<float>(activeCropW.load())
		: static_cast<float>(grabber.getPixels().getWidth());
	const float frameH = activeCropH.load() > 0 ? static_cast<float>(activeCropH.load())
		: static_cast<float>(grabber.getPixels().getHeight());
	const ofRectangle frame(0, 0, frameW, frameH);
	const float frameDiagSq = frame.width * frame.width + frame.height * frame.height;
	const float expected = std::max(1.0f, static_cast<float>(parameters.irisExpectedRadius.get()));

	IrisCandidate best = cands.front();
	float bestScore = -std::numeric_limits<float>::max();
	for (const auto & c : cands) {
		// Size closeness: 1 when the radius matches the expected iris radius.
		const float sizeScore = clamp01(1.0f - std::abs(c.radius - expected) / std::max(1.0f, expected));

		float temporalScore = 0.0f;
		if (hasLastEyeBox) {
			const float distNorm = rectDistanceSq(c.box, lastEyeBox) / std::max(1.0f, frameDiagSq);
			temporalScore = 1.0f - clamp01(distNorm * 4.0f);
		}

		// Preferred side acts as a dominant, near-hard preference: a candidate on
		// the wrong half is pushed far below any right-half candidate, so the
		// chosen eye flips reliably when two are visible. Among same-side
		// candidates, size + fit + temporal proximity still decide.
		const float sidePenalty = onPreferredSide(c.center.x, frame.width) ? 0.0f : -10.0f;

		const float score = sizeScore * 0.25f
			+ c.fitQuality * 0.25f
			+ c.darkness * 0.25f
			+ temporalScore * 0.25f
			+ sidePenalty;
		if (score > bestScore) {
			bestScore = score;
			best = c;
		}
	}

	return best;
}

//--------------------------------------------------------------
bool EyeCameraStream::onPreferredSide(float srcX, float srcW) const {
	const int side = parameters.irisPreferredSide.get();
	if (side == 0) {
		return true; // no preference: every candidate qualifies.
	}
	// Detection runs on the non-mirrored source, but the user picks a side based
	// on the (possibly mirrored) display. Convert to a displayed [0,1] x so the
	// preference matches what they see.
	const float nx = (srcW > 0.0f) ? srcX / srcW : 0.5f;
	const float dispNx = config.mirrorX ? (1.0f - nx) : nx;
	return (side == 1) ? (dispNx < 0.5f) : (dispNx >= 0.5f);
}

//--------------------------------------------------------------
float EyeCameraStream::getGazeX() const {
	std::lock_guard<std::mutex> lk(stateMutex);
	// Only report a live gaze while the eye is present; otherwise neutral so the
	// driven feature relaxes to its centered state during dropouts.
	if (!result.present) {
		return 0.0f;
	}
	const float halfW = result.eyeBox.width * 0.5f;
	if (halfW <= 1.0f || result.irisRadiusPx <= 0.0f) {
		return 0.0f; // no eye box or no fitted iris (e.g. haar mode).
	}
	float gaze = (result.irisCenter.x - result.eyeCenter.x) / halfW;
	// eyeCenter/irisCenter live in non-mirrored source px; flip so positive
	// always means "looking right" in the mirrored display the user sees.
	if (config.mirrorX) {
		gaze = -gaze;
	}
	return gaze;
}

//--------------------------------------------------------------
void EyeCameraStream::applySmoothing(const RawDetection & raw, float dt) {
	if (raw.valid) {
		++presentHitStreak;
		presentMissStreak = 0;
	} else {
		++presentMissStreak;
		presentHitStreak = 0;
	}

	if (!presentSmoothed && presentHitStreak >= parameters.presentOnFrames) {
		presentSmoothed = true;
	}
	if (presentSmoothed && presentMissStreak >= parameters.presentOffFrames) {
		presentSmoothed = false;
		// PRESENT -> LOST transition: drop the temporal anchor so the next
		// detection picks the strongest blob by area only. Prevents a single
		// false positive (chin, mouth, etc.) from sticking via temporal score.
		hasLastEyeBox = false;
	}

	result.present = presentSmoothed;

	// Important: only overwrite eyeBox/eyeCenter/confidence on a *valid* raw.
	// Otherwise we keep the last good values so the overlay doesn't collapse
	// to (0,0) during transient detection misses.
	if (raw.valid) {
		result.detectedEyeWidthPx = raw.eyeBox.width;
		result.confidence = raw.confidence;
		result.irisSize = raw.irisSize;
		result.irisAngleDeg = raw.irisAngleDeg;
		result.irisRadiusPx = raw.irisRadiusPx;
		result.fitQuality = raw.fitQuality;

		// The eye anchor is the box *center*; smoothing the center plus the box
		// size (rather than copying raw.eyeBox) gives a rock-steady anchor while
		// the iris stays free to move with gaze. A manual per-eye offset (source
		// px) corrects the Haar box's slight bias; X is mirror-aware so positive
		// always moves the anchor right in the displayed image.
		const float offX = parameters.eyeCenterOffsetX.get() * (config.mirrorX ? -1.0f : 1.0f);
		const float offY = parameters.eyeCenterOffsetY.get();
		const float rawBoxCx = raw.eyeBox.x + raw.eyeBox.width * 0.5f + offX;
		const float rawBoxCy = raw.eyeBox.y + raw.eyeBox.height * 0.5f + offY;

		if (presentSmoothed && !presentSmoothedPrev) {
			filterEyeX.reset(rawBoxCx);
			filterEyeY.reset(rawBoxCy);
			filterBoxW.reset(raw.eyeBox.width);
			filterBoxH.reset(raw.eyeBox.height);
			filterIrisX.reset(raw.irisCenter.x);
			filterIrisY.reset(raw.irisCenter.y);
		}

		float cx = rawBoxCx;
		float cy = rawBoxCy;
		float bw = raw.eyeBox.width;
		float bh = raw.eyeBox.height;
		glm::vec2 iris = raw.irisCenter;
		if (presentSmoothed) {
			cx = filterEyeX.filter(rawBoxCx, dt);
			cy = filterEyeY.filter(rawBoxCy, dt);
			bw = filterBoxW.filter(raw.eyeBox.width, dt);
			bh = filterBoxH.filter(raw.eyeBox.height, dt);
			iris.x = filterIrisX.filter(raw.irisCenter.x, dt);
			iris.y = filterIrisY.filter(raw.irisCenter.y, dt);
		}

		result.eyeCenter = {cx, cy};
		result.eyeBox = ofRectangle(cx - bw * 0.5f, cy - bh * 0.5f, bw, bh);
		result.irisCenter = iris;
	}

	presentSmoothedPrev = presentSmoothed;
}

//--------------------------------------------------------------
void EyeCameraStream::renderTargetFbo() {
	if (!grabber.getTexture().isAllocated()) {
		return;
	}

	const float fboW = static_cast<float>(config.fboSize.x);
	const float fboH = static_cast<float>(config.fboSize.y);

	// Crop sub-rectangle of the full sensor frame; all fit/follow math below
	// works in cropped-source space so the FBO shows only the cropped image.
	const ofRectangle cropRect = computeCropRectSource(
		grabber.getTexture().getWidth(), grabber.getTexture().getHeight());
	const float srcW = cropRect.width;
	const float srcH = cropRect.height;
	float drawW = fboW;
	float drawH = fboH;
	float drawX = 0.0f;
	float drawY = 0.0f;
	if (srcW > 0.0f && srcH > 0.0f) {
		const float s = std::max(0.0001f, parameters.viewScale.get());
		if (parameters.fitToFill.get()) {
			// Aspect-preserving fit: shrink the whole camera frame to fit the
			// FBO, then apply view scale as a multiplier on that baseline.
			// scale > 1 zooms in (image is cropped by the FBO); scale < 1 zooms out.
			const float srcAspect = srcW / srcH;
			const float fboAspect = fboW / fboH;
			if (srcAspect > fboAspect) {
				drawW = fboW;
				drawH = fboW / srcAspect;
			} else {
				drawH = fboH;
				drawW = fboH * srcAspect;
			}
			drawW *= s;
			drawH *= s;
		} else {
			// Exact camera scale: view scale is the literal camera pixel -> FBO
			// pixel factor, so scale == 1.0 draws the frame at native resolution
			// (one camera pixel per FBO pixel) regardless of FBO/camera aspect.
			drawW = srcW * s;
			drawH = srcH * s;
		}
		drawX = (fboW - drawW) * 0.5f;
		drawY = (fboH - drawH) * 0.5f;

		// Eye-follow: shift the source image so the (smoothed) eye center sits
		// at the FBO center, then clamp so the FBO is always fully covered.
		// The mirror is symmetric for centering, so the same formula and clamp
		// apply whether config.mirrorX is true or false.
		const float now = ofGetElapsedTimef();
		const float dt = std::max(0.001f, std::min(0.25f, now - lastFollowTime));
		lastFollowTime = now;

		if (parameters.followEye.get()) {
			const Result snap = getResult();
			const bool haveDetection = (snap.eyeBox.width > 0.0f && snap.eyeBox.height > 0.0f);
			if (haveDetection) {
				const float targetX = snap.eyeCenter.x;
				const float targetY = snap.eyeCenter.y;
				if (followSmoothedX < 0.0f) {
					// First sample: snap rather than animate in from (0, 0).
					followSmoothedX = targetX;
					followSmoothedY = targetY;
				} else {
					// Time-based exponential low-pass; slider == 0 disables smoothing,
					// slider == 1 maps to a ~0.5 s time constant.
					const float smoothing = std::min(0.999f, std::max(0.0f, parameters.followSmoothing.get()));
					const float tau = smoothing * 0.5f;
					const float alpha = (tau > 1e-4f) ? (1.0f - std::exp(-dt / tau)) : 1.0f;
					followSmoothedX += alpha * (targetX - followSmoothedX);
					followSmoothedY += alpha * (targetY - followSmoothedY);
				}
			}

			if (followSmoothedX >= 0.0f) {
				drawX = fboW * 0.5f - (followSmoothedX / srcW) * drawW;
				drawY = fboH * 0.5f - (followSmoothedY / srcH) * drawH;
			}
		} else {
			// Reset so a future enable re-snaps to the next valid detection.
			followSmoothedX = -1.0f;
			followSmoothedY = -1.0f;
		}

		// Clamp so the camera image always fully covers the FBO (no black
		// background visible). When the scaled image is smaller than the FBO
		// in either axis, follow is impossible without exposing black, so we
		// fall back to centered drawing along that axis.
		if (drawW >= fboW) {
			drawX = std::min(0.0f, std::max(fboW - drawW, drawX));
		} else {
			drawX = (fboW - drawW) * 0.5f;
		}
		if (drawH >= fboH) {
			drawY = std::min(0.0f, std::max(fboH - drawH, drawY));
		} else {
			drawY = (fboH - drawH) * 0.5f;
		}
	}

	lastLayout = {drawX, drawY, drawW, drawH, true};

	targetFbo.begin();
	ofClear(0, 0, 0, 255);
	ofPushMatrix();
	if (config.mirrorX) {
		ofTranslate(fboW, 0.0f);
		ofScale(-1.0f, 1.0f);
	}
	drawCameraIntoFbo(drawX, drawY, drawW, drawH, cropRect);
	ofPopMatrix();
	targetFbo.end();
}

//--------------------------------------------------------------
void EyeCameraStream::drawCameraIntoFbo(float drawX, float drawY, float drawW, float drawH,
		const ofRectangle & srcRect) {
	const ofTexture & tex = grabber.getTexture();

	// Lazy-build shader once the texture target is known. Rebuild if the target
	// flips between rect / 2D (driver/GL setting changes).
	const bool needsRect = (tex.getTextureData().textureTarget == GL_TEXTURE_RECTANGLE_ARB);
	if (!gradeShaderLoaded || gradeShaderUsesRect != needsRect) {
		gradeShaderLoaded = buildGradeShader(needsRect);
	}

	// drawSubsection samples only the cropped sub-rectangle of the sensor frame,
	// so the crop is applied to the displayed image (FBO + raw debug view).
	const bool useShader = parameters.enableGrading && gradeShaderLoaded;
	if (!useShader) {
		tex.drawSubsection(drawX, drawY, drawW, drawH,
			srcRect.x, srcRect.y, srcRect.width, srcRect.height);
		return;
	}

	gradeShader.begin();
	gradeShader.setUniformTexture("tex0", tex, 0);
	gradeShader.setUniform1f("uExposure", parameters.gradeExposure.get());
	gradeShader.setUniform1f("uBrightness", parameters.gradeBrightness.get());
	gradeShader.setUniform1f("uContrast", parameters.gradeContrast.get());
	gradeShader.setUniform1f("uGamma", std::max(0.01f, parameters.gradeGamma.get()));
	gradeShader.setUniform1f("uSaturation", parameters.gradeSaturation.get());
	tex.drawSubsection(drawX, drawY, drawW, drawH,
		srcRect.x, srcRect.y, srcRect.width, srcRect.height);
	gradeShader.end();
}

//--------------------------------------------------------------
void EyeCameraStream::threadedFunction() {
	FrameJob job;
	while (frameJobs.receive(job)) {
		// Coalesce: if the main thread already enqueued newer frames, jump to the latest.
		FrameJob newer;
		while (frameJobs.tryReceive(newer)) {
			job = std::move(newer);
		}

		++detectionsRun;

		if (!parameters.enableEyeTracking) {
			// Reset worker-only smoothing state so re-enabling tracking starts fresh.
			presentSmoothed = false;
			presentSmoothedPrev = false;
			presentHitStreak = 0;
			presentMissStreak = 0;
			hasLastEyeBox = false;
			std::lock_guard<std::mutex> lk(stateMutex);
			resetWorkerStateLocked();
			continue;
		}

		const float dt = std::max(1.0f / 1000.0f, job.captureTime - workerLastFrameTime);
		workerLastFrameTime = job.captureTime;

		// Pick up live changes to smoothing parameters from the GUI.
		const float minCutoff = std::max(0.001f, parameters.euroMinCutoff.get());
		const float beta = std::max(0.0f, parameters.euroBeta.get());
		filterEyeX.setParams(minCutoff, beta);
		filterEyeY.setParams(minCutoff, beta);
		filterBoxW.setParams(minCutoff, beta);
		filterBoxH.setParams(minCutoff, beta);
		filterIrisX.setParams(minCutoff, beta);
		filterIrisY.setParams(minCutoff, beta);

		RawDetection raw;
		detectEye(job.pixels, raw);

		if (raw.valid) {
			++detectionsValid;
		}

		std::lock_guard<std::mutex> lk(stateMutex);
		if (!parameters.enableEyeTracking) {
			resetWorkerStateLocked();
			continue;
		}
		lastRaw = raw;
		applySmoothing(raw, dt);
	}
}

//--------------------------------------------------------------
void EyeCameraStream::update() {
	grabber.update();

	if (!grabber.isFrameNew()) {
		renderTargetFbo();
		return;
	}

	if (!parameters.enableEyeTracking) {
		std::lock_guard<std::mutex> lk(stateMutex);
		if (result.present || lastRaw.valid) {
			resetWorkerStateLocked();
		}
		FrameJob discarded;
		while (frameJobs.tryReceive(discarded)) {}
		renderTargetFbo();
		return;
	}

	{
		FrameJob discarded;
		while (frameJobs.tryReceive(discarded)) {}
	}

	FrameJob job;
	// Crop the raw frame *before* detection so the worker never sees the
	// inter-camera overlap (the opposite eye), and so detection coordinates
	// already live in the same cropped space the FBO/overlay display.
	const ofPixels & full = grabber.getPixels();
	const ofRectangle cr = computeCropRectSource(full.getWidth(), full.getHeight());
	if (cr.x <= 0.0f && cr.y <= 0.0f
		&& cr.width >= full.getWidth() && cr.height >= full.getHeight()) {
		job.pixels = full; // no crop: avoid an extra copy.
	} else {
		full.cropTo(job.pixels, static_cast<int>(cr.x), static_cast<int>(cr.y),
			static_cast<int>(cr.width), static_cast<int>(cr.height));
	}
	activeCropW = static_cast<int>(job.pixels.getWidth());
	activeCropH = static_cast<int>(job.pixels.getHeight());
	job.captureTime = ofGetElapsedTimef();
	frameJobs.send(std::move(job));

	renderTargetFbo();
}

//--------------------------------------------------------------
void EyeCameraStream::draw(float x, float y, float w, float h) const {
	if (!targetFbo.isAllocated()) {
		return;
	}
	targetFbo.draw(x, y, w, h);
}

//--------------------------------------------------------------
void EyeCameraStream::drawDebug(float x, float y, float w, float h) const {
	// The cropped eye-FBO view intentionally draws no detection overlay; the raw
	// camera view (drawRawDebug) is the dedicated debug view that carries it.
	draw(x, y, w, h);
}

//--------------------------------------------------------------
void EyeCameraStream::drawRawDebug(float x, float y, float w, float h) {
	// The raw view shows the cropped frame, so its aspect ratio and the overlay
	// mapping both use the cropped dimensions, not the full sensor.
	const ofRectangle cropRect = computeCropRectSource(
		grabber.getPixels().getWidth(), grabber.getPixels().getHeight());
	const float srcW = cropRect.width;
	const float srcH = cropRect.height;
	if (srcW <= 0.0f || srcH <= 0.0f) {
		ofPushStyle();
		ofSetColor(255);
		ofDrawBitmapString(config.name + " — waiting for camera", x + 20.0f, y + 20.0f);
		ofPopStyle();
		return;
	}

	// Aspect-fit the *whole* camera frame into the destination rect (no eye
	// follow / zoom), so this view shows the full graded sensor image with the
	// detection overlay mapped in source-pixel space.
	const float srcAspect = srcW / srcH;
	const float dstAspect = (h > 0.0f) ? (w / h) : srcAspect;
	float imgW = w;
	float imgH = h;
	if (srcAspect > dstAspect) {
		imgW = w;
		imgH = w / srcAspect;
	} else {
		imgH = h;
		imgW = h * srcAspect;
	}
	const float imgX = x + (w - imgW) * 0.5f;
	const float imgY = y + (h - imgH) * 0.5f;

	if (grabber.getTexture().isAllocated()) {
		ofPushStyle();
		ofSetColor(255);
		ofPushMatrix();
		if (config.mirrorX) {
			// Mirror horizontally within the fit rect so orientation matches the
			// FBO view and the overlay's mirror-aware source->display mapping.
			ofTranslate(imgX + imgX + imgW, 0.0f);
			ofScale(-1.0f, 1.0f);
		}
		// Reuses the grade shader path so the raw view honours the same color
		// grading as the cropped FBO view (or direct draw when grading is off).
		drawCameraIntoFbo(imgX, imgY, imgW, imgH, cropRect);
		ofPopMatrix();
		ofPopStyle();
	}

	if (parameters.showDebugOverlay) {
		drawDetectionOverlay(x, y, imgX, imgY, imgW, imgH, srcW, srcH);
	}
}

//--------------------------------------------------------------
void EyeCameraStream::drawDetectionOverlay(float dispX, float dispY,
		float imgX, float imgY, float imgW, float imgH,
		float srcW, float srcH) const {
	if (srcW <= 0.0f || srcH <= 0.0f) {
		return;
	}

	// Snapshot worker-published state under lock for race-free overlay drawing.
	Result snapResult;
	RawDetection snapLastRaw;
	{
		std::lock_guard<std::mutex> lk(stateMutex);
		snapResult = result;
		snapLastRaw = lastRaw;
	}

	const std::uint64_t runs = detectionsRun.load();
	const std::uint64_t hits = detectionsValid.load();

	const bool mirror = config.mirrorX;
	const float dispScaleX = imgW / srcW;
	const float dispScaleY = imgH / srcH;

	auto mapSrcToDispX = [&](float px) {
		const float nx = mirror ? (srcW - px) : px;
		return imgX + nx * dispScaleX;
	};
	auto mapSrcToDispY = [&](float py) {
		return imgY + py * dispScaleY;
	};

	const bool trackingOn = parameters.enableEyeTracking;
	const bool haveDetection = trackingOn && (snapResult.present || snapLastRaw.valid);

	ofPushStyle();

	// Debug: every candidate box (yellow) and iris ellipse (cyan) considered this
	// frame, including rejected ones. The chosen box/iris is drawn solid below.
	if (trackingOn && parameters.debugShowCandidates.get()) {
		ofNoFill();
		ofSetLineWidth(1.0f);
		ofSetColor(255, 230, 0, 130);
		for (const auto & b : snapLastRaw.candidateBoxes) {
			const float bx = mirror ? (srcW - (b.x + b.width)) : b.x;
			ofDrawRectangle(imgX + bx * dispScaleX, imgY + b.y * dispScaleY,
				b.width * dispScaleX, b.height * dispScaleY);
		}
		ofSetColor(0, 230, 255, 150);
		for (const auto & ic : snapLastRaw.candidateIris) {
			if (ic.size.x <= 0.0f || ic.size.y <= 0.0f) {
				continue;
			}
			const float ecx = mapSrcToDispX(ic.center.x);
			const float ecy = mapSrcToDispY(ic.center.y);
			const float eang = mirror ? -ic.angleDeg : ic.angleDeg;
			ofPushMatrix();
			ofTranslate(ecx, ecy);
			ofRotateDeg(eang);
			ofDrawEllipse(0.0f, 0.0f, ic.size.x * dispScaleX, ic.size.y * dispScaleY);
			ofPopMatrix();
		}
	}

	if (haveDetection) {
		const ofRectangle & box = snapResult.eyeBox.width > 0 ? snapResult.eyeBox : snapLastRaw.eyeBox;
		const glm::vec2 center = (snapResult.eyeBox.width > 0)
			? snapResult.eyeCenter
			: snapLastRaw.eyeCenter;

		const float boxW = box.width;
		const float boxH = box.height;

		const float dispBoxW = boxW * dispScaleX;
		const float dispBoxH = boxH * dispScaleY;
		const float boxLeftSrc = mirror ? (srcW - (box.x + boxW)) : box.x;
		const float dispBoxX = imgX + boxLeftSrc * dispScaleX;
		const float dispBoxY = imgY + box.y * dispScaleY;

		const float dispCenterX = mapSrcToDispX(center.x);
		const float dispCenterY = mapSrcToDispY(center.y);

		if (snapResult.present) {
			ofSetColor(0, 255, 0, 220);
		} else {
			ofSetColor(255, 80, 80, 200);
		}

		ofNoFill();
		ofSetLineWidth(2.0f);
		if (boxW > 0.0f && boxH > 0.0f) {
			ofDrawRectangle(dispBoxX, dispBoxY, dispBoxW, dispBoxH);
		}

		const float cross = 12.0f;
		ofDrawLine(dispCenterX - cross, dispCenterY, dispCenterX + cross, dispCenterY);
		ofDrawLine(dispCenterX, dispCenterY - cross, dispCenterX, dispCenterY + cross);

		// Fitted iris ellipse (iris detector only; haar leaves irisRadiusPx == 0).
		const bool useResIris = snapResult.irisRadiusPx > 0.0f;
		const float irisR = useResIris ? snapResult.irisRadiusPx : snapLastRaw.irisRadiusPx;
		const glm::vec2 irisSz = useResIris ? snapResult.irisSize : snapLastRaw.irisSize;
		const float irisAng = useResIris ? snapResult.irisAngleDeg : snapLastRaw.irisAngleDeg;
		const glm::vec2 irisC = useResIris ? snapResult.irisCenter : snapLastRaw.irisCenter;
		if (irisR > 0.0f && irisSz.x > 0.0f && irisSz.y > 0.0f) {
			const float ecx = mapSrcToDispX(irisC.x);
			const float ecy = mapSrcToDispY(irisC.y);
			const float ew = irisSz.x * dispScaleX;
			const float eh = irisSz.y * dispScaleY;
			// Mirroring flips the apparent rotation direction.
			const float eang = mirror ? -irisAng : irisAng;
			ofSetColor(80, 180, 255, 230);
			ofPushMatrix();
			ofTranslate(ecx, ecy);
			ofRotateDeg(eang);
			ofNoFill();
			ofSetLineWidth(2.0f);
			ofDrawEllipse(0.0f, 0.0f, ew, eh);
			ofPopMatrix();

			// Iris-center dot and a faint line from the eye-box center to the
			// iris center: a live preview of the future gaze vector.
			ofSetColor(80, 180, 255, 120);
			ofSetLineWidth(1.0f);
			ofDrawLine(dispCenterX, dispCenterY, ecx, ecy);
			ofFill();
			ofSetColor(80, 180, 255, 230);
			ofDrawCircle(ecx, ecy, 3.0f);
			ofNoFill();
		}

		ofSetColor(255, 255, 255, 230);
		std::string boxInfo = "box " + ofToString((int)boxW) + "x" + ofToString((int)boxH)
			+ "  center (" + ofToString((int)center.x) + "," + ofToString((int)center.y) + ")";
		ofDrawBitmapStringHighlight(boxInfo, dispBoxX, std::max(dispY + 12.0f, dispBoxY - 6.0f));
	}

	// Presence + worker heartbeat at the top of the displayed quad.
	std::string state;
	if (!trackingOn) {
		state = "TRACKING OFF";
		ofSetColor(120, 120, 120, 230);
	} else if (snapResult.present) {
		state = "PRESENT";
		ofSetColor(0, 200, 0, 230);
	} else {
		state = snapLastRaw.valid ? "LOST (stale)" : "LOST";
		ofSetColor(220, 60, 60, 230);
	}
	ofDrawBitmapStringHighlight(config.name + " : " + state, dispX + 6, dispY + 16);

	ofSetColor(255, 255, 0, 230);
	const int dmode = parameters.detectorMode.get();
	const char * detName = (dmode == 1) ? "haar" : (dmode == 2) ? "boxiris" : "iris";
	const float fitShown = snapResult.fitQuality > 0.0f ? snapResult.fitQuality : snapLastRaw.fitQuality;
	std::string info = std::string("det=") + detName
		+ "  fit=" + ofToString(fitShown, 2)
		+ "  dark=" + ofToString(snapLastRaw.darkness, 2)
		+ "  cand=" + ofToString((unsigned)snapLastRaw.candidateBoxes.size())
		+ "b/" + ofToString((unsigned)snapLastRaw.candidateIris.size()) + "i"
		+ "  runs=" + ofToString((unsigned long long)runs)
		+ "  hits=" + ofToString((unsigned long long)hits)
		+ "  conf=" + ofToString(snapResult.confidence, 2);
	ofDrawBitmapStringHighlight(info, dispX + 6, dispY + 36);

	ofPopStyle();
}
