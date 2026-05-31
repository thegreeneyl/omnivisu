#include "EyeCameraStream.h"

#include <algorithm>
#include <cmath>

namespace {
float clamp01(float v) {
	return std::max(0.0f, std::min(1.0f, v));
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
	parameters.trackingGroup.add(parameters.trackingDownscale);
	parameters.trackingGroup.add(parameters.haarMinWidth);
	parameters.trackingGroup.add(parameters.haarMinHeight);
	parameters.trackingGroup.add(parameters.haarScale);
	parameters.trackingGroup.add(parameters.haarNeighbors);
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
	parameters.viewGroup.add(parameters.followEye);
	parameters.viewGroup.add(parameters.followSmoothing);
	parameters.group.add(parameters.viewGroup);
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

	const int w = config.fboSize.x;
	const int h = config.fboSize.y;
	targetFbo.allocate(w, h, GL_RGB);

	const float minCutoff = parameters.euroMinCutoff;
	const float beta = parameters.euroBeta;
	filterEyeX.setup(minCutoff, beta);
	filterEyeY.setup(minCutoff, beta);

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
bool EyeCameraStream::loadParameters() {
	const auto path = ofToDataPath(paramsFilePath(), true);
	if (!ofFile::doesFileExist(path)) {
		ofLogError("EyeCameraStream") << config.name
			<< ": params file not found at " << path
			<< " - using bare ofParameter declarations";
		return false;
	}
	const ofJson json = ofLoadJson(path);
	ofDeserialize(json, parameters.group);
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

	const ofRectangle frame(0, 0, grabber.getPixels().getWidth(), grabber.getPixels().getHeight());
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
		result.eyeBox = raw.eyeBox;
		result.detectedEyeWidthPx = raw.eyeBox.width;
		result.confidence = raw.confidence;

		if (presentSmoothed && !presentSmoothedPrev) {
			filterEyeX.reset(raw.eyeCenter.x);
			filterEyeY.reset(raw.eyeCenter.y);
		}

		if (presentSmoothed) {
			result.eyeCenter.x = filterEyeX.filter(raw.eyeCenter.x, dt);
			result.eyeCenter.y = filterEyeY.filter(raw.eyeCenter.y, dt);
		} else {
			result.eyeCenter = raw.eyeCenter;
		}
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

	const float srcW = grabber.getTexture().getWidth();
	const float srcH = grabber.getTexture().getHeight();
	float drawW = fboW;
	float drawH = fboH;
	float drawX = 0.0f;
	float drawY = 0.0f;
	if (srcW > 0.0f && srcH > 0.0f) {
		const float srcAspect = srcW / srcH;
		const float fboAspect = fboW / fboH;
		if (srcAspect > fboAspect) {
			drawW = fboW;
			drawH = fboW / srcAspect;
		} else {
			drawH = fboH;
			drawW = fboH * srcAspect;
		}
		// Apply user-controlled view scale on top of the aspect-preserving fit.
		// scale > 1 zooms in (image is cropped by the FBO); scale < 1 zooms out.
		const float s = std::max(0.0001f, parameters.viewScale.get());
		drawW *= s;
		drawH *= s;
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
	drawCameraIntoFbo(drawX, drawY, drawW, drawH);
	ofPopMatrix();
	targetFbo.end();
}

//--------------------------------------------------------------
void EyeCameraStream::drawCameraIntoFbo(float drawX, float drawY, float drawW, float drawH) {
	const ofTexture & tex = grabber.getTexture();

	// Lazy-build shader once the texture target is known. Rebuild if the target
	// flips between rect / 2D (driver/GL setting changes).
	const bool needsRect = (tex.getTextureData().textureTarget == GL_TEXTURE_RECTANGLE_ARB);
	if (!gradeShaderLoaded || gradeShaderUsesRect != needsRect) {
		gradeShaderLoaded = buildGradeShader(needsRect);
	}

	const bool useShader = parameters.enableGrading && gradeShaderLoaded;
	if (!useShader) {
		tex.draw(drawX, drawY, drawW, drawH);
		return;
	}

	gradeShader.begin();
	gradeShader.setUniformTexture("tex0", tex, 0);
	gradeShader.setUniform1f("uExposure", parameters.gradeExposure.get());
	gradeShader.setUniform1f("uBrightness", parameters.gradeBrightness.get());
	gradeShader.setUniform1f("uContrast", parameters.gradeContrast.get());
	gradeShader.setUniform1f("uGamma", std::max(0.01f, parameters.gradeGamma.get()));
	gradeShader.setUniform1f("uSaturation", parameters.gradeSaturation.get());
	tex.draw(drawX, drawY, drawW, drawH);
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
	job.pixels = grabber.getPixels();
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
	draw(x, y, w, h);

	if (!parameters.showDebugOverlay) {
		return;
	}

	const float srcW = grabber.getPixels().getWidth();
	const float srcH = grabber.getPixels().getHeight();
	if (srcW <= 0 || srcH <= 0) {
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

	// The image is letterboxed inside the FBO, which is then drawn at (x, y, w, h).
	// Compute the on-screen rect of the image itself so overlays map back to source pixels.
	const bool mirror = config.mirrorX;
	const float fboW = static_cast<float>(config.fboSize.x);
	const float fboH = static_cast<float>(config.fboSize.y);

	float imgX = x;
	float imgY = y;
	float imgW = w;
	float imgH = h;
	if (fboW > 0.0f && fboH > 0.0f) {
		float drawX, drawY, drawW, drawH;
		if (lastLayout.valid) {
			// Use the same layout renderTargetFbo() applied, so the overlay
			// follows the camera through scale and eye-follow shifts.
			drawX = lastLayout.x;
			drawY = lastLayout.y;
			drawW = lastLayout.w;
			drawH = lastLayout.h;
		} else {
			// Fallback for the very first frame, before renderTargetFbo has run:
			// centered, scale-aware letterbox fit (matches the no-follow path).
			const float srcAspect = srcW / srcH;
			const float fboAspect = fboW / fboH;
			drawW = fboW;
			drawH = fboH;
			if (srcAspect > fboAspect) {
				drawW = fboW;
				drawH = fboW / srcAspect;
			} else {
				drawH = fboH;
				drawW = fboH * srcAspect;
			}
			const float s = std::max(0.0001f, parameters.viewScale.get());
			drawW *= s;
			drawH *= s;
			drawX = (fboW - drawW) * 0.5f;
			drawY = (fboH - drawH) * 0.5f;
		}
		const float fboToDispX = w / fboW;
		const float fboToDispY = h / fboH;
		imgX = x + drawX * fboToDispX;
		imgY = y + drawY * fboToDispY;
		imgW = drawW * fboToDispX;
		imgH = drawH * fboToDispY;
	}

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

		ofSetColor(255, 255, 255, 230);
		std::string boxInfo = "box " + ofToString((int)boxW) + "x" + ofToString((int)boxH)
			+ "  center (" + ofToString((int)center.x) + "," + ofToString((int)center.y) + ")";
		ofDrawBitmapStringHighlight(boxInfo, dispBoxX, std::max(y + 12.0f, dispBoxY - 6.0f));
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
	ofDrawBitmapStringHighlight(config.name + " : " + state, x + 6, y + 16);

	ofSetColor(255, 255, 0, 230);
	std::string info = "worker runs=" + ofToString((unsigned long long)runs)
		+ "  hits=" + ofToString((unsigned long long)hits)
		+ "  conf=" + ofToString(snapResult.confidence, 2);
	ofDrawBitmapStringHighlight(info, x + 6, y + 36);

	ofPopStyle();
}
