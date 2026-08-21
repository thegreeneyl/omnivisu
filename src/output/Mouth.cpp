#include "Mouth.h"

#include <algorithm>
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
	if (json.contains("lights")) {
		const auto & l = json["lights"];
		lights.x = std::max(1, l.value("w", lights.x));
		lights.y = std::max(1, l.value("h", lights.y));
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

	allocateLightFbos();

	loaded = true;
	ofLogNotice("Mouth") << "loaded size (" << size.x << "x" << size.y
		<< "), lights (" << lights.x << "x" << lights.y
		<< "), anchor (" << anchor.x << ", " << anchor.y << "), color ("
		<< static_cast<int>(color.r) << ", " << static_cast<int>(color.g) << ", "
		<< static_cast<int>(color.b) << ", " << static_cast<int>(color.a)
		<< "), gaze in [" << gazeInMin << ", " << gazeInMax << "]";
	return true;
}

//--------------------------------------------------------------
void Mouth::allocateLightFbos() {
	const int ssW = lights.x * kSupersample;
	const int ssH = lights.y * kSupersample;

	// Skip when already allocated at the right size (config reload with an
	// unchanged grid).
	if (lightsFbo.isAllocated()
		&& static_cast<int>(lightsFbo.getWidth()) == lights.x
		&& static_cast<int>(lightsFbo.getHeight()) == lights.y
		&& lightsSsFbo.isAllocated()
		&& static_cast<int>(lightsSsFbo.getWidth()) == ssW) {
		return;
	}

	// GL_TEXTURE_2D is required explicitly: OF's desktop default is a rectangle
	// texture target, which cannot have mipmaps (needed for the downsample).
	ofFboSettings ss;
	ss.width = ssW;
	ss.height = ssH;
	ss.internalformat = GL_RGBA;
	ss.textureTarget = GL_TEXTURE_2D;
	ss.numSamples = 0;
	ss.useDepth = false;
	ss.useStencil = false;
	lightsSsFbo.allocate(ss);
	lightsSsFbo.begin();
	ofClear(0, 0, 0, 0);
	lightsSsFbo.end();
	// Mipmaps give a true area average when downsampling: kSupersample is a
	// power of two, so the lights.x x lights.y mip level is the exact mean of
	// each light's kSupersample^2 block. Plain GL_LINEAR minification would
	// only sample 4 of the 64 covered texels. Generate once here so the
	// texture "has" mipmaps — setTextureMinMagFilter silently ignores mipmap
	// filters on textures without them.
	lightsSsFbo.getTexture().generateMipmap();
	lightsSsFbo.getTexture().setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);

	ofFboSettings lf;
	lf.width = lights.x;
	lf.height = lights.y;
	lf.internalformat = GL_RGBA;
	lf.textureTarget = GL_TEXTURE_2D;
	lf.numSamples = 0;
	lf.useDepth = false;
	lf.useStencil = false;
	lightsFbo.allocate(lf);
	// Nearest-neighbor so the on-screen upscale shows hard LED cells.
	lightsFbo.getTexture().setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);

	lightsFbo.begin();
	ofClear(0, 0, 0, 0);
	lightsFbo.end();

	lightsPixels.allocate(lights.x, lights.y, OF_PIXELS_RGBA);
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
	} else {
		// Time-based exponential low-pass; slider 1.0 -> ~0.5 s time constant.
		const float tau = smoothing * 0.5f;
		const float alpha = 1.0f - std::exp(-dt / tau);
		leftGazeSmoothed += alpha * (leftGazeTarget - leftGazeSmoothed);
		rightGazeSmoothed += alpha * (rightGazeTarget - rightGazeSmoothed);
	}

	renderLights();
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
void Mouth::renderLights() {
	if (!loaded || !lightsFbo.isAllocated() || !lightsSsFbo.isAllocated()) {
		return;
	}

	// Each edge slides with its eye's gaze. Offsets are in half-width units
	// relative to the center, so the full mouth width spans -1..+1 — which maps
	// directly onto the light row: offset -1 = light 0's left edge, +1 = the
	// last light's right edge.
	const float leftOffset = ofLerp(leftEdgeExtend, leftEdgeRetract, gazeToParam(leftGazeSmoothed));
	const float rightOffset = ofLerp(rightEdgeRetract, rightEdgeExtend, gazeToParam(rightGazeSmoothed));

	const float ssW = lightsSsFbo.getWidth();
	const float ssH = lightsSsFbo.getHeight();
	float leftPx = (leftOffset * 0.5f + 0.5f) * ssW;
	float rightPx = (rightOffset * 0.5f + 0.5f) * ssW;
	// Guard against the edges crossing (e.g. extreme/overlapping ranges).
	if (rightPx < leftPx) {
		rightPx = leftPx;
	}

	// Supersampled pass. The background is cleared to the mouth COLOR with
	// alpha 0 (not transparent black) so the downsample average only dilutes
	// alpha, never the color: a half-covered light keeps the full RGB and gets
	// half the alpha, which blends linearly with coverage on screen.
	lightsSsFbo.begin();
	ofClear(color.r, color.g, color.b, 0);
	ofPushStyle();
	ofDisableAlphaBlending();
	ofFill();
	ofSetColor(color);
	ofDrawRectangle(leftPx, 0.0f, rightPx - leftPx, ssH);
	ofPopStyle();
	lightsSsFbo.end();

	// Area-downsample into the light grid via the mipmap chain (see
	// allocateLightFbos). Blending stays off: this is a resolve, not a
	// composite.
	lightsSsFbo.getTexture().generateMipmap();
	lightsFbo.begin();
	ofClear(0, 0, 0, 0);
	ofPushStyle();
	ofDisableAlphaBlending();
	ofSetColor(255);
	lightsSsFbo.draw(0.0f, 0.0f, lightsFbo.getWidth(), lightsFbo.getHeight());
	ofPopStyle();
	lightsFbo.end();
}

//--------------------------------------------------------------
const ofPixels & Mouth::getLightPixels() {
	if (lightsFbo.isAllocated()) {
		lightsFbo.readToPixels(lightsPixels);
	}
	return lightsPixels;
}

//--------------------------------------------------------------
void Mouth::draw(float scale, float imgX, float imgY) const {
	if (!loaded || !lightsFbo.isAllocated()) {
		return;
	}

	// The destination is the FULL mouth footprint (the light row always spans
	// size.x); gaze now moves the lit region inside the light grid instead of
	// resizing the drawn rectangle. Nearest-neighbor filtering (set at
	// allocation) turns each texel into a hard LED cell on screen.
	const float screenX = imgX + (anchor.x - size.x * 0.5f) * scale;
	const float screenY = imgY + (anchor.y - size.y * 0.5f) * scale;
	const float screenW = size.x * scale;
	const float screenH = size.y * scale;

	ofPushStyle();
	ofEnableAlphaBlending();
	ofSetColor(255);
	lightsFbo.draw(screenX, screenY, screenW, screenH);
	ofPopStyle();
}
