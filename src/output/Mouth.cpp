#include "Mouth.h"

#include <algorithm>
#include <cmath>

namespace {

/// Reads a color channel, defaulting to the current value, and clamps it into
/// the valid 0..255 range (the config may carry out-of-range values).
int readChannel(const ofJson & j, const char * key, int fallback) {
	return static_cast<int>(ofClamp(j.value(key, fallback), 0, 255));
}

/// Maps a raw value into 0..1 over [lo, hi], clamped. Degenerate range = 0.5.
float normalizeParam(float v, float lo, float hi) {
	const float span = hi - lo;
	if (std::abs(span) < 1e-6f) {
		return 0.5f;
	}
	return ofClamp((v - lo) / span, 0.0f, 1.0f);
}

} // namespace

//--------------------------------------------------------------
void Mouth::QuantizedChannel::reset(float param) {
	const int count = std::max(1, stateCount);
	state = std::clamp(static_cast<int>(std::floor(param * count)), 0, count - 1);
	dwell = 1e9f; // free to switch immediately.
}

//--------------------------------------------------------------
bool Mouth::QuantizedChannel::advance(float param, float dt) {
	dwell += dt;
	const int count = std::max(1, stateCount);
	if (count == 1) {
		state = 0;
		return false;
	}
	state = std::clamp(state, 0, count - 1);

	// Schmitt trigger: the current band, expanded by the hysteresis margin on
	// both sides. Only an input clearly outside it may switch states.
	const float band = 1.0f / count;
	const float lo = state * band - hysteresis * band;
	const float hi = (state + 1) * band + hysteresis * band;
	if (param >= lo && param <= hi) {
		return false;
	}
	// Dwell gate: even a clear crossing must wait out the minimum hold time,
	// which caps the state-change rate during fast sweeps.
	if (dwell < minDwellSeconds) {
		return false;
	}
	state = std::clamp(static_cast<int>(std::floor(param * count)), 0, count - 1);
	dwell = 0.0f;
	return true;
}

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
		gazeXMin = ctrl.value("gaze_x_min", gazeXMin);
		gazeXMax = ctrl.value("gaze_x_max", gazeXMax);
		gazeYMin = ctrl.value("gaze_y_min", gazeYMin);
		gazeYMax = ctrl.value("gaze_y_max", gazeYMax);
		positionStates = std::max(1, ctrl.value("position_states", positionStates));
		if (ctrl.contains("widths") && ctrl["widths"].is_array()
			&& !ctrl["widths"].empty()) {
			widths.clear();
			for (const auto & w : ctrl["widths"]) {
				// Never below 1 light: the mouth must always show something.
				widths.push_back(std::max(1, w.get<int>()));
			}
		}
		hysteresis = ofClamp(ctrl.value("hysteresis", hysteresis), 0.0f, 1.0f);
		minDwellSeconds = std::max(0.0f, ctrl.value("min_dwell_seconds", minDwellSeconds));
		transitionSeconds = std::max(0.0f, ctrl.value("transition_seconds", transitionSeconds));
		smoothing = std::max(0.0f, ctrl.value("smoothing", smoothing));
	}

	allocateLightFbos();

	// Configure the channels and start them at their neutral (centered) state,
	// with the edges snapped onto the resulting target (no startup animation).
	positionChannel.stateCount = positionStates;
	positionChannel.hysteresis = hysteresis;
	positionChannel.minDwellSeconds = minDwellSeconds;
	positionChannel.reset(0.5f);
	widthChannel.stateCount = static_cast<int>(widths.size());
	widthChannel.hysteresis = hysteresis;
	widthChannel.minDwellSeconds = minDwellSeconds;
	widthChannel.reset(0.5f);
	computeTargetEdges();
	currentLeftLight = targetLeftLight;
	currentRightLight = targetRightLight;

	loaded = true;
	std::string widthsStr;
	for (size_t i = 0; i < widths.size(); ++i) {
		widthsStr += (i ? "," : "") + ofToString(widths[i]);
	}
	ofLogNotice("Mouth") << "loaded size (" << size.x << "x" << size.y
		<< "), lights (" << lights.x << "x" << lights.y
		<< "), anchor (" << anchor.x << ", " << anchor.y
		<< "), positions " << positionStates << ", widths [" << widthsStr
		<< "], hysteresis " << hysteresis << ", dwell " << minDwellSeconds
		<< "s, transition " << transitionSeconds << "s";
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
void Mouth::setGaze(const glm::vec2 & leftGaze, bool leftPresent,
	const glm::vec2 & rightGaze, bool rightPresent) {
	// Average over the present eyes; with none present relax to neutral, which
	// drives both channels back to their centered states.
	if (leftPresent && rightPresent) {
		gazeTarget = (leftGaze + rightGaze) * 0.5f;
	} else if (leftPresent) {
		gazeTarget = leftGaze;
	} else if (rightPresent) {
		gazeTarget = rightGaze;
	} else {
		gazeTarget = {0.0f, 0.0f};
	}
}

//--------------------------------------------------------------
void Mouth::computeTargetEdges() {
	const int gridW = std::max(1, lights.x);
	int w = 1;
	if (!widths.empty()) {
		const int idx = std::clamp(widthChannel.state, 0, static_cast<int>(widths.size()) - 1);
		w = widths[idx];
	}
	w = std::clamp(w, 1, gridW);

	// Position states spread evenly over the travel that remains for this
	// width, so every state is a whole-light target fully inside the grid.
	const float p = positionStates > 1
		? static_cast<float>(positionChannel.state) / static_cast<float>(positionStates - 1)
		: 0.5f;
	const int left = static_cast<int>(std::lround(static_cast<float>(gridW - w) * p));
	targetLeftLight = static_cast<float>(left);
	targetRightLight = static_cast<float>(left + w);
}

//--------------------------------------------------------------
void Mouth::update(float dt) {
	if (!loaded) {
		return;
	}

	// Input low-pass (time constant = smoothing seconds): removes tracker
	// noise before quantization without delaying state decisions much.
	if (smoothing <= 0.0f || dt <= 0.0f) {
		gazeSmoothed = gazeTarget;
	} else {
		const float alpha = 1.0f - std::exp(-dt / smoothing);
		gazeSmoothed += alpha * (gazeTarget - gazeSmoothed);
	}

	// Quantize both channels with hysteresis + dwell.
	const float posParam = normalizeParam(gazeSmoothed.x, gazeXMin, gazeXMax);
	const float widthParam = normalizeParam(gazeSmoothed.y, gazeYMin, gazeYMax);
	const bool posChanged = positionChannel.advance(posParam, dt);
	const bool widthChanged = widthChannel.advance(widthParam, dt);
	if (posChanged || widthChanged) {
		computeTargetEdges();
		ofLogNotice("Mouth") << "state -> pos " << positionChannel.state << "/"
			<< (positionStates - 1) << ", width " << widthChannel.state << "/"
			<< (static_cast<int>(widths.size()) - 1)
			<< " => lights [" << targetLeftLight << ".." << targetRightLight << ")";
	}

	// Ease the current edges toward the (integer) target. Time constant is
	// transition_seconds / 3 so a move is ~95% done after transition_seconds.
	if (transitionSeconds <= 0.0f || dt <= 0.0f) {
		currentLeftLight = targetLeftLight;
		currentRightLight = targetRightLight;
	} else {
		const float alpha = 1.0f - std::exp(-dt / (transitionSeconds / 3.0f));
		currentLeftLight += alpha * (targetLeftLight - currentLeftLight);
		currentRightLight += alpha * (targetRightLight - currentRightLight);
	}

	renderLights();
}

//--------------------------------------------------------------
void Mouth::renderLights() {
	if (!loaded || !lightsFbo.isAllocated() || !lightsSsFbo.isAllocated()) {
		return;
	}

	const float ssW = lightsSsFbo.getWidth();
	const float ssH = lightsSsFbo.getHeight();
	// Edges live in light units; one light = kSupersample supersample pixels.
	float leftPx = ofClamp(currentLeftLight * kSupersample, 0.0f, ssW);
	float rightPx = ofClamp(currentRightLight * kSupersample, 0.0f, ssW);
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
	// size.x); the state machine moves the lit region inside the light grid.
	// Nearest-neighbor filtering (set at allocation) turns each texel into a
	// hard LED cell on screen.
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
