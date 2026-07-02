#include "Mouth.h"

#include <cmath>

namespace {

/// Reads a color channel, defaulting to the current value, and clamps it into
/// the valid 0..255 range (the config may carry out-of-range values).
int readChannel(const ofJson & j, const char * key, int fallback) {
	return static_cast<int>(ofClamp(j.value(key, fallback), 0, 255));
}

} // namespace

//--------------------------------------------------------------
bool Mouth::load(const ofJson & json) {
	loaded = false;

	if (json.is_null() || json.empty()) {
		ofLogWarning("Mouth") << "empty or missing mouth config block; using defaults";
		return false;
	}

	if (json.contains("size")) {
		const auto & s = json["size"];
		size.x = s.value("w", size.x);
		size.y = s.value("h", size.y);
	}
	if (json.contains("anchor")) {
		const auto & a = json["anchor"];
		anchor.x = a.value("x", anchor.x);
		anchor.y = a.value("y", anchor.y);
	}
	if (json.contains("color")) {
		const auto & c = json["color"];
		color.r = readChannel(c, "r", color.r);
		color.g = readChannel(c, "g", color.g);
		color.b = readChannel(c, "b", color.b);
		color.a = readChannel(c, "a", color.a);
	}
	if (json.contains("control")) {
		const auto & ctrl = json["control"];
		gazeInMin = ctrl.value("gaze_in_min", gazeInMin);
		gazeInMax = ctrl.value("gaze_in_max", gazeInMax);
		leftEdgeExtend = ctrl.value("left_edge_extend", leftEdgeExtend);
		leftEdgeRetract = ctrl.value("left_edge_retract", leftEdgeRetract);
		rightEdgeRetract = ctrl.value("right_edge_retract", rightEdgeRetract);
		rightEdgeExtend = ctrl.value("right_edge_extend", rightEdgeExtend);
		smoothing = ofClamp(ctrl.value("smoothing", smoothing), 0.0f, 1.0f);
	}

	loaded = true;
	ofLogNotice("Mouth") << "loaded size (" << size.x << "x" << size.y
		<< "), anchor (" << anchor.x << ", " << anchor.y << "), color ("
		<< static_cast<int>(color.r) << ", " << static_cast<int>(color.g) << ", "
		<< static_cast<int>(color.b) << ", " << static_cast<int>(color.a)
		<< "), gaze in [" << gazeInMin << ", " << gazeInMax << "]";
	return true;
}

//--------------------------------------------------------------
void Mouth::setGaze(float leftGazeX, float rightGazeX) {
	leftGazeTarget = leftGazeX;
	rightGazeTarget = rightGazeX;
}

//--------------------------------------------------------------
void Mouth::update(float dt) {
	if (smoothing <= 0.0f || dt <= 0.0f) {
		leftGazeSmoothed = leftGazeTarget;
		rightGazeSmoothed = rightGazeTarget;
		return;
	}
	// Time-based exponential low-pass; slider 1.0 -> ~0.5 s time constant.
	const float tau = smoothing * 0.5f;
	const float alpha = 1.0f - std::exp(-dt / tau);
	leftGazeSmoothed += alpha * (leftGazeTarget - leftGazeSmoothed);
	rightGazeSmoothed += alpha * (rightGazeTarget - rightGazeSmoothed);
}

//--------------------------------------------------------------
float Mouth::gazeToParam(float gaze) const {
	const float span = gazeInMax - gazeInMin;
	if (std::abs(span) < 1e-6f) {
		return 0.5f;
	}
	return ofClamp((gaze - gazeInMin) / span, 0.0f, 1.0f);
}

//--------------------------------------------------------------
void Mouth::draw(float scale, float imgX, float imgY) const {
	if (!loaded) {
		return;
	}

	const float halfW = size.x * 0.5f;

	// Each edge slides with its eye's gaze. Offsets are in half-width units
	// relative to the center, so multiply by halfW to land in image-pixel space.
	const float leftOffset = ofLerp(leftEdgeExtend, leftEdgeRetract, gazeToParam(leftGazeSmoothed));
	const float rightOffset = ofLerp(rightEdgeRetract, rightEdgeExtend, gazeToParam(rightGazeSmoothed));

	float leftEdgeImg = anchor.x + leftOffset * halfW;
	float rightEdgeImg = anchor.x + rightOffset * halfW;
	// Guard against the edges crossing (e.g. extreme/overlapping ranges).
	if (rightEdgeImg < leftEdgeImg) {
		rightEdgeImg = leftEdgeImg;
	}

	// Apply the mask's cover-fit transform to land in screen space. Height is
	// fixed and vertically centered on the anchor.
	const float screenX = imgX + leftEdgeImg * scale;
	const float screenW = (rightEdgeImg - leftEdgeImg) * scale;
	const float screenY = imgY + (anchor.y - size.y * 0.5f) * scale;
	const float screenH = size.y * scale;

	ofPushStyle();
	ofEnableAlphaBlending();
	ofFill();
	ofSetColor(color);
	ofDrawRectangle(screenX, screenY, screenW, screenH);
	ofPopStyle();
}
