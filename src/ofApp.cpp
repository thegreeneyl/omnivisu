#include "ofApp.h"

#include <algorithm>
#include <array>
#include <sstream>

namespace {

/// Draws a stream's FBO into dst with a "cover" fit (preserve aspect, crop
/// overflow) so the eye opening is fully filled with no distortion or
/// background. GL scissor clips the cropped overflow to the eye rect.
/// viewportHeight is the height of the current render target (window height
/// when drawing to screen, FBO height when drawing into an FBO) and is used to
/// flip dst.y into GL's bottom-left scissor origin.
void drawStreamCover(const EyeCameraStream & stream, const ofRectangle & dst, float viewportHeight) {
	const auto & fbo = stream.getTargetFbo();
	if (!fbo.isAllocated() || dst.width <= 0.0f || dst.height <= 0.0f) {
		return;
	}
	const float fboW = fbo.getWidth();
	const float fboH = fbo.getHeight();
	if (fboW <= 0.0f || fboH <= 0.0f) {
		return;
	}

	const float scale = std::max(dst.width / fboW, dst.height / fboH);
	const float drawW = fboW * scale;
	const float drawH = fboH * scale;
	const float drawX = dst.x + (dst.width - drawW) * 0.5f;
	const float drawY = dst.y + (dst.height - drawH) * 0.5f;

	ofPushStyle();
	ofSetColor(255);
	glEnable(GL_SCISSOR_TEST);
	// GL scissor origin is bottom-left, so flip the y of the dst rect.
	glScissor(static_cast<GLint>(dst.x),
		static_cast<GLint>(viewportHeight - (dst.y + dst.height)),
		static_cast<GLsizei>(dst.width),
		static_cast<GLsizei>(dst.height));
	fbo.draw(drawX, drawY, drawW, drawH);
	glDisable(GL_SCISSOR_TEST);
	ofPopStyle();
}

} // namespace

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("omnivisu");
	ofSetVerticalSync(true);
	ofBackground(0);

	appConfig.load("config.json");
	viewMode = appConfig.startsInMaskMode() ? ViewMode::Mask : ViewMode::EyeFbo;

	// streams[0] is the LEFT eye, streams[1] the RIGHT eye. This positional
	// mapping matches MaskLayout's left/right openings; each eye's JSON binds
	// to a specific physical camera via its "camera" block.
	struct EyeSpec {
		std::string name;
		std::string paramsFile;
	};
	const std::array<EyeSpec, 2> eyes = {{
		{"left", "eye_left.json"},
		{"right", "eye_right.json"},
	}};
	for (const auto & eye : eyes) {
		EyeCameraStream::Config cfg;
		cfg.name = eye.name;
		cfg.paramsFile = eye.paramsFile;
		cfg.fboSize = {672, 504};
		cfg.mirrorX = true;

		auto stream = std::make_unique<EyeCameraStream>();
		if (!stream->setup(cfg)) {
			ofLogError("omnivisu") << "Failed to initialize " << eye.name << " camera stream";
		}
		streams.push_back(std::move(stream));
	}

	guis.reserve(streams.size());
	for (size_t i = 0; i < streams.size(); ++i) {
		auto panel = std::make_unique<ofxPanel>();
		panel->setup(streams[i]->getName());
		panel->add(streams[i]->getGrabberParameters());
		panel->add(streams[i]->getTrackingParameters());
		panel->add(streams[i]->getGradingParameters());
		panel->add(streams[i]->getViewParameters());
		panel->add(streams[i]->getCropParameters());
		guis.push_back(std::move(panel));
	}
	layoutGuis();

	maskLoaded = maskLayout.load(appConfig.getMaskJson());
	if (!maskLoaded) {
		ofLogWarning("omnivisu") << "mask layout not loaded; falling back to side-by-side view";
		viewMode = ViewMode::EyeFbo;
	}

	mouthLoaded = mouth.load(appConfig.getMouthJson());

	applyStreamingConfig(appConfig.getStreaming());
}

//--------------------------------------------------------------
void ofApp::applyStreamingConfig(const AppConfig::StreamingConfig & sc) {
	streamingEnabled = sc.enabled;
	if (!streamingEnabled) {
		streamSender.close();
		return;
	}

	// Combined stream FBO is a fixed 828x280 (independent of config), so only
	// allocate it once.
	if (!streamFbo.isAllocated()) {
		ofFboSettings fboSettings;
		fboSettings.width = kStreamWidth;
		fboSettings.height = kStreamHeight;
		fboSettings.internalformat = GL_RGB;
		fboSettings.numSamples = 0;
		fboSettings.useDepth = false;
		fboSettings.useStencil = false;
		streamFbo.allocate(fboSettings);
		streamFbo.begin();
		ofClear(0, 0, 0, 255);
		streamFbo.end();
	}

	streamAsyncReadback = sc.asyncReadback;
	streamFpsLimit = sc.fpsLimit;
	if (!streamPixels.isAllocated()) {
		streamPixels.allocate(kStreamWidth, kStreamHeight, OF_PIXELS_RGB);
	}
	if (streamAsyncReadback && !streamPbo[0].isAllocated()) {
		const size_t bytes = static_cast<size_t>(kStreamWidth) * kStreamHeight * 3; // GL_RGB
		streamPbo[0].allocate(bytes, GL_STREAM_READ);
		streamPbo[1].allocate(bytes, GL_STREAM_READ);
	}
	streamPboIndex = 0;
	streamPboPrimed = false;
	nextStreamSampleTime = 0.0f;

	EyeStreamSender::Config senderCfg;
	senderCfg.targetIp = sc.targetIp;
	senderCfg.targetPort = sc.targetPort;
	senderCfg.compression = sc.compression;
	senderCfg.packetPayloadBytes = sc.packetPayloadBytes;
	senderCfg.fpsLimit = sc.fpsLimit;
	if (!streamSender.setup(senderCfg)) {
		ofLogError("omnivisu") << "failed to start eye stream sender";
		streamingEnabled = false;
	}
}

//--------------------------------------------------------------
void ofApp::reloadConfig() {
	appConfig.load("config.json");

	// Re-apply display mode + mask layout from disk.
	maskLoaded = maskLayout.load(appConfig.getMaskJson());
	// Preserve the currently selected view across reloads; only drop out of the
	// mask view if the mask layout is no longer available.
	if (!maskLoaded && viewMode == ViewMode::Mask) {
		viewMode = ViewMode::EyeFbo;
		ofLogWarning("omnivisu") << "mask layout not loaded on reload; using side-by-side view";
	}

	mouthLoaded = mouth.load(appConfig.getMouthJson());

	// Re-apply streaming: this tears down and restarts the sender so changes to
	// target ip/port, compression, payload size, fps cap, async readback, or the
	// enabled flag all take effect immediately.
	applyStreamingConfig(appConfig.getStreaming());
	ofLogNotice("omnivisu") << "config.json reloaded";
}

//--------------------------------------------------------------
void ofApp::layoutGuis() {
	const float screenW = ofGetWidth();
	const float screenH = ofGetHeight();
	const int n = static_cast<int>(streams.size());
	if (n == 0 || guis.size() != streams.size()) {
		return;
	}

	const float slotW = screenW / static_cast<float>(n);
	const float pad = 10.0f;

	for (int i = 0; i < n; ++i) {
		const float texW = streams[i]->getTargetFbo().getWidth() > 0
			? static_cast<float>(streams[i]->getTargetFbo().getWidth())
			: 672.0f;
		const float texH = streams[i]->getTargetFbo().getHeight() > 0
			? static_cast<float>(streams[i]->getTargetFbo().getHeight())
			: 504.0f;
		const float scale = std::min(slotW / texW, screenH / texH);
		const float drawW = texW * scale;
		const float drawH = texH * scale;
		const float drawX = slotW * static_cast<float>(i) + (slotW - drawW) * 0.5f;
		const float drawY = (screenH - drawH) * 0.5f;
		guis[i]->setPosition(drawX + pad, drawY + pad);
	}
}

//--------------------------------------------------------------
void ofApp::update() {
	for (const auto & stream : streams) {
		stream->update();
	}

	// Drive the mouth width from gaze: streams[0] = left eye -> left edge,
	// streams[1] = right eye -> right edge.
	if (mouthLoaded) {
		const float leftGaze = streams.size() > 0 ? streams[0]->getGazeX() : 0.0f;
		const float rightGaze = streams.size() > 1 ? streams[1]->getGazeX() : 0.0f;
		mouth.setGaze(leftGaze, rightGaze);
		mouth.update(static_cast<float>(ofGetLastFrameTime()));
	}

	const float now = ofGetElapsedTimef();
	if (now - lastFpsLogTime >= 1.0f) {
		std::ostringstream msg;
		msg << "app fps: " << ofGetFrameRate();
		for (size_t i = 0; i < streams.size(); ++i) {
			const auto & g = streams[i]->getGrabber();
			const auto r = streams[i]->getResult();
			msg << " | [" << i << "] frames=" << g.getFrameCounter()
				<< " err=" << g.getErrorCounter()
				<< " worker_runs=" << streams[i]->getDetectionCount()
				<< " hits=" << streams[i]->getValidDetectionCount()
				<< " present=" << (r.present ? 1 : 0)
				<< " conf=" << ofToString(r.confidence, 2)
				<< " fit=" << ofToString(r.fitQuality, 2);
		}
		if (streamCpuSamples > 0) {
			msg << " | stream_cpu_us=" << static_cast<long>(streamCpuUsSum / streamCpuSamples)
				<< " (n=" << streamCpuSamples << ", async=" << (streamAsyncReadback ? 1 : 0) << ")";
			streamCpuUsSum = 0.0;
			streamCpuSamples = 0;
		}
		ofLogNotice("omnivisu") << msg.str();
		lastFpsLogTime = now;
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	const float screenW = ofGetWidth();
	const float screenH = ofGetHeight();
	const int n = static_cast<int>(streams.size());

	if (viewMode == ViewMode::Mask && maskLoaded) {
		drawMasked();
	} else if (n == 0) {
		ofSetColor(255);
		ofDrawBitmapString("omnivisu — no camera streams", 20, 20);
	} else {
		const float slotW = screenW / static_cast<float>(n);

		for (int i = 0; i < n; ++i) {
			auto & stream = streams[i];

			if (viewMode == ViewMode::RawCamera) {
				// Full graded camera frame for the slot; the stream aspect-fits
				// the whole sensor image and draws the detection overlay on top.
				stream->drawRawDebug(slotW * static_cast<float>(i), 0.0f, slotW, screenH);
				continue;
			}

			const glm::ivec2 fboSize = stream->getTargetFbo().getWidth() > 0
				? glm::ivec2(stream->getTargetFbo().getWidth(), stream->getTargetFbo().getHeight())
				: glm::ivec2(672, 504);

			const float texW = static_cast<float>(fboSize.x);
			const float texH = static_cast<float>(fboSize.y);
			const float scale = std::min(slotW / texW, screenH / texH);
			const float drawW = texW * scale;
			const float drawH = texH * scale;
			const float drawX = slotW * static_cast<float>(i) + (slotW - drawW) * 0.5f;
			const float drawY = (screenH - drawH) * 0.5f;

			if (stream->getTargetFbo().isAllocated()) {
				stream->drawDebug(drawX, drawY, drawW, drawH);
			} else {
				ofSetColor(255);
				ofDrawBitmapString(stream->getName() + " — waiting for camera", drawX + 20, drawY + 20);
			}
		}
	}

	if (showGui) {
		for (auto & panel : guis) {
			panel->draw();
		}
	}

	if (showFps) {
		ofSetColor(255);
		ofDrawBitmapString(ofToString(ofGetFrameRate(), 2), 20, ofGetHeight() - 20);
	}

	updateStream();
}

//--------------------------------------------------------------
void ofApp::updateStream() {
	if (!streamingEnabled || !streamFbo.isAllocated()) {
		return;
	}

	// Decouple the expensive stream render+readback from the display rate. When
	// capped, most display frames skip this work entirely so the window holds
	// its full refresh rate.
	if (streamFpsLimit > 0) {
		const float now = ofGetElapsedTimef();
		if (now < nextStreamSampleTime) {
			return;
		}
		const float interval = 1.0f / static_cast<float>(streamFpsLimit);
		nextStreamSampleTime += interval;
		// If we fell more than one interval behind (e.g. after a stall), resync
		// to "now" instead of bursting a catch-up sequence of frames.
		if (nextStreamSampleTime < now) {
			nextStreamSampleTime = now + interval;
		}
	}

	const std::uint64_t streamStartUs = ofGetElapsedTimeMicros();
	const int n = static_cast<int>(streams.size());
	const float vpH = static_cast<float>(kStreamHeight);

	streamFbo.begin();
	ofClear(0, 0, 0, 255);
	if (n > 0) {
		drawStreamCover(*streams[0],
			ofRectangle(0.0f, 0.0f, kStreamEyeWidth, kStreamHeight), vpH);
	}
	if (n > 1) {
		drawStreamCover(*streams[1],
			ofRectangle(kStreamEyeWidth, 0.0f, kStreamEyeWidth, kStreamHeight), vpH);
	}
	streamFbo.end();

	if (streamAsyncReadback) {
		glPixelStorei(GL_PACK_ALIGNMENT, 1);          // defensive; 828*3 is already 4-aligned
		streamFbo.copyTo(streamPbo[streamPboIndex]);  // async glReadPixels into PBO, returns immediately

		const int prev = 1 - streamPboIndex;
		if (streamPboPrimed) {
			unsigned char * p = streamPbo[prev].map<unsigned char>(GL_READ_ONLY);
			if (p) {
				memcpy(streamPixels.getData(), p, streamPixels.size());
				streamPbo[prev].unmap();
				streamSender.submit(streamPixels);
			}
		}
		streamPboIndex = prev;
		streamPboPrimed = true;
	} else {
		streamFbo.readToPixels(streamPixels);
		streamSender.submit(streamPixels);
	}

	streamCpuUsSum += static_cast<double>(ofGetElapsedTimeMicros() - streamStartUs);
	++streamCpuSamples;
}

//--------------------------------------------------------------
void ofApp::drawMasked() {
	ofClear(0);

	const MaskLayout::ScreenLayout sl = maskLayout.compute(ofGetWidth(), ofGetHeight());
	const int n = static_cast<int>(streams.size());

	// Positional mapping: streams[0] = left eye, streams[1] = right eye.
	const float vpH = static_cast<float>(ofGetWindowHeight());
	if (n > 0) {
		drawStreamCover(*streams[0], sl.leftEyeScreen, vpH);
	}
	if (n > 1) {
		drawStreamCover(*streams[1], sl.rightEyeScreen, vpH);
	}

	ofEnableAlphaBlending();
	maskLayout.drawMask(sl);

	// The mouth lives in the same image-pixel space as the mask, so it reuses
	// the cover-fit transform and draws on top of the mask.
	if (mouthLoaded) {
		mouth.draw(sl.scale, sl.imgX, sl.imgY);
	}
}

//--------------------------------------------------------------
void ofApp::exit() {
	streamSender.close();
	streams.clear();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'g' || key == 'G') {
		showGui = !showGui;
	} else if (key == 'm' || key == 'M') {
		// Cycle: mask overlay -> cropped eye FBOs -> raw graded camera. Skip the
		// mask view when no mask layout is loaded.
		switch (viewMode) {
		case ViewMode::Mask:
			viewMode = ViewMode::EyeFbo;
			break;
		case ViewMode::EyeFbo:
			viewMode = ViewMode::RawCamera;
			break;
		case ViewMode::RawCamera:
		default:
			viewMode = maskLoaded ? ViewMode::Mask : ViewMode::EyeFbo;
			break;
		}
	} else if (key == 'f' || key == 'F') {
		showFps = !showFps;
	} else if (key == 's' || key == 'S') {
		for (const auto & stream : streams) {
			stream->saveParameters();
		}
	} else if (key == 'r' || key == 'R') {
		for (const auto & stream : streams) {
			stream->loadParameters();
		}
		reloadConfig();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
	layoutGuis();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {
}
