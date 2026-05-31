#include "ofApp.h"

#include <array>
#include <sstream>

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("omnivisu");
	ofSetVerticalSync(true);
	ofBackground(0);

	const std::array<std::string, 2> names = {"eye0", "eye1"};
	for (const auto & name : names) {
		EyeCameraStream::Config cfg;
		cfg.name = name;
		cfg.fboSize = {672, 504};
		cfg.mirrorX = true;

		auto stream = std::make_unique<EyeCameraStream>();
		if (!stream->setup(cfg)) {
			ofLogError("omnivisu") << "Failed to initialize " << name << " camera stream";
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

	if (n == 0) {
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
void ofApp::exit() {
	streams.clear();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'g' || key == 'G') {
		showGui = !showGui;
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
