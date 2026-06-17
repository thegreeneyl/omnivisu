#include "ofApp.h"

#include <algorithm>
#include <array>
#include <sstream>

namespace {

/// Draws a stream's FBO into dst with a "cover" fit (preserve aspect, crop
/// overflow) so the eye opening is fully filled with no distortion or
/// background. GL scissor clips the cropped overflow to the eye rect.
void drawStreamCover(const EyeCameraStream & stream, const ofRectangle & dst) {
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
		static_cast<GLint>(ofGetWindowHeight() - (dst.y + dst.height)),
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
	maskMode = appConfig.startsInMaskMode();

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
		guis.push_back(std::move(panel));
	}
	layoutGuis();

	maskLoaded = maskLayout.load(appConfig.getMaskJson());
	if (!maskLoaded) {
		ofLogWarning("omnivisu") << "mask layout not loaded; falling back to side-by-side view";
		maskMode = false;
	}
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
				<< " conf=" << ofToString(r.confidence, 2);
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

	if (maskMode && maskLoaded) {
		drawMasked();
	} else if (n == 0) {
		ofSetColor(255);
		ofDrawBitmapString("omnivisu — no camera streams", 20, 20);
	} else {
		const float slotW = screenW / static_cast<float>(n);

		for (int i = 0; i < n; ++i) {
			const auto & stream = streams[i];
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
}

//--------------------------------------------------------------
void ofApp::drawMasked() {
	ofClear(0);

	const MaskLayout::ScreenLayout sl = maskLayout.compute(ofGetWidth(), ofGetHeight());
	const int n = static_cast<int>(streams.size());

	// Positional mapping: streams[0] = left eye, streams[1] = right eye.
	if (n > 0) {
		drawStreamCover(*streams[0], sl.leftEyeScreen);
	}
	if (n > 1) {
		drawStreamCover(*streams[1], sl.rightEyeScreen);
	}

	ofEnableAlphaBlending();
	maskLayout.drawMask(sl);
}

//--------------------------------------------------------------
void ofApp::exit() {
	streams.clear();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'g' || key == 'G') {
		showGui = !showGui;
	} else if (key == 'm' || key == 'M') {
		if (maskLoaded) {
			maskMode = !maskMode;
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
