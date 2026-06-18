#pragma once

#include "ofMain.h"
#include "ofBufferObject.h"
#include "ofxGui.h"
#include "AppConfig.h"
#include "EyeCameraStream.h"
#include "EyeStreamSender.h"
#include "MaskLayout.h"

#include <memory>
#include <vector>

class ofApp : public ofBaseApp {
public:
	void setup() override;
	void update() override;
	void draw() override;
	void exit() override;

	void keyPressed(int key) override;
	void keyReleased(int key) override;
	void mouseMoved(int x, int y) override;
	void mouseDragged(int x, int y, int button) override;
	void mousePressed(int x, int y, int button) override;
	void mouseReleased(int x, int y, int button) override;
	void mouseEntered(int x, int y) override;
	void mouseExited(int x, int y) override;
	void windowResized(int w, int h) override;
	void dragEvent(ofDragInfo dragInfo) override;
	void gotMessage(ofMessage msg) override;

private:
	void layoutGuis();
	void drawMasked();
	void updateStream();
	void applyStreamingConfig(const AppConfig::StreamingConfig & sc);
	void reloadConfig();

	std::vector<std::unique_ptr<EyeCameraStream>> streams;
	std::vector<std::unique_ptr<ofxPanel>> guis;
	AppConfig appConfig;
	MaskLayout maskLayout;
	// View cycled by the 'm' key: mask overlay -> cropped eye FBOs -> raw graded
	// camera (each with the detection overlay where applicable).
	enum class ViewMode { Mask, EyeFbo, RawCamera };
	ViewMode viewMode = ViewMode::Mask;
	bool maskLoaded = false;
	bool showGui = false;
	bool showFps = false;
	float lastFpsLogTime = 0.0f;

	// Eye video streaming. The combined stream FBO holds the two eyes
	// side-by-side (left half | right half), each cover-fit like the mask.
	static constexpr int kStreamEyeWidth = 414;
	static constexpr int kStreamHeight = 280;
	static constexpr int kStreamWidth = kStreamEyeWidth * 2; // 828
	bool streamingEnabled = false;
	ofFbo streamFbo;
	ofPixels streamPixels;
	EyeStreamSender streamSender;

	// Asynchronous FBO readback via double-buffered PBOs. Avoids the
	// glReadPixels pipeline stall at the cost of one frame of latency.
	bool streamAsyncReadback = true;
	ofBufferObject streamPbo[2];
	int streamPboIndex = 0;
	bool streamPboPrimed = false;

	// Cap how often the (GPU-side) stream render + readback runs, decoupled
	// from the display frame rate. 0 = every frame. Lets the display hold
	// 60fps while the stream samples at e.g. 30fps. nextStreamSampleTime is a
	// fixed-grid schedule so the achieved rate approximates the target even
	// when the display rate is not an exact multiple of it.
	int streamFpsLimit = 0;
	float nextStreamSampleTime = 0.0f;

	// Rolling CPU cost of updateStream() for the per-second diagnostic log.
	double streamCpuUsSum = 0.0;
	int streamCpuSamples = 0;
};
