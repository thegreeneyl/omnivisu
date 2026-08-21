#pragma once

#include "ofMain.h"

#include <vector>

/// JSON-driven mouth feature living in the same image-pixel coordinate space as
/// the mask (and the eye openings). Defined by a center anchor, a base size, and
/// an RGBA fill color. Drawn using the mask's cover-fit transform (scale +
/// clamped image offset) so it stays locked to the face at any window size.
///
/// Control model: the combined (averaged-over-present-eyes) gaze drives two
/// independently quantized channels with hysteresis, so the mouth holds
/// deliberate poses instead of tracking the eyes continuously:
///   - horizontal gaze -> POSITION state (looking right moves the mouth right),
///   - vertical gaze   -> WIDTH state (looking up widens, down narrows; the
///     width list is config-driven and never below 1 light).
/// The resulting target always lands on whole lights of the grid; the current
/// edges ease smoothly toward it, which reads as lights handing over.
///
/// The continuous mouth is rasterized through a discrete light grid (config
/// "lights", default 14x1 — one texel per physical LED): the bar is drawn into
/// a supersampled FBO, area-downsampled into the tiny lights FBO (so partially
/// covered lights get fractional brightness), and the preview upscales that
/// grid with nearest-neighbor filtering into the size/anchor footprint. What
/// you see on the mask is therefore exactly what the LED strip can express.
class Mouth {
public:
	/// Loads the mouth from the "mouth" JSON block (size, lights grid, center
	/// anchor, color, and an optional "control" sub-block for the state
	/// mapping). Returns false (and logs) if the block is empty; the defaults
	/// below are kept in that case. (Re)allocates the light-grid FBOs.
	bool load(const ofJson & mouthJson);

	bool isLoaded() const { return loaded; }

	/// Feeds the latest per-eye normalized gaze (x: negative = looking left,
	/// positive = right; y: positive = looking up) plus eye presence. The mouth
	/// averages the PRESENT eyes; with both absent it relaxes to neutral.
	void setGaze(const glm::vec2 & leftGaze, bool leftPresent,
		const glm::vec2 & rightGaze, bool rightPresent);

	/// Advances input smoothing, the state machines, and the edge easing by dt
	/// seconds, then re-rasterizes the light grid.
	void update(float dt);

	/// Draws the light-grid FBO (nearest-neighbor upscaled, so each light shows
	/// as a hard cell) using the mask's image->screen transform, i.e. the scale
	/// and (imgX, imgY) offset produced by MaskLayout::compute().
	void draw(float scale, float imgX, float imgY) const;

	/// The discrete light grid the mouth was rasterized into (lights.w x
	/// lights.h texels, one per physical light). This is the future DMX source.
	const ofFbo & getLightsFbo() const { return lightsFbo; }

	/// Reads the current light grid back to the CPU (lights.w x lights.h RGBA
	/// pixels, row-major). Intended for the future DMX/stream path; not called
	/// per-frame by the app yet.
	const ofPixels & getLightPixels();

private:
	/// One quantized control channel: splits the normalized 0..1 input into
	/// `stateCount` equal bands and holds the current state until the input
	/// leaves the current band by more than `hysteresis` band-widths AND the
	/// state has been held for at least `minDwellSeconds` (Schmitt trigger +
	/// dwell gate — kills boundary ping-pong and hectic sweeps).
	struct QuantizedChannel {
		int stateCount = 2;
		float hysteresis = 0.35f;     ///< Fraction of one band width.
		float minDwellSeconds = 0.25f;

		int state = 0;
		float dwell = 1e9f; ///< Seconds in the current state (starts "expired").

		/// Resets to the state whose band contains `param` (no hysteresis).
		void reset(float param);
		/// Advances by dt with the current normalized input; returns true when
		/// the state changed.
		bool advance(float param, float dt);
	};

	/// (Re)allocates the lights + supersample FBOs to the current grid size.
	void allocateLightFbos();

	/// Recomputes the integer target edges (in light units) from the current
	/// channel states: width from the widths list, position spread across the
	/// travel left for that width. Always whole lights, always inside the grid.
	void computeTargetEdges();

	/// Rasterizes the current (eased) mouth edges into the light grid:
	/// supersampled draw, then area-downsample into lightsFbo.
	void renderLights();

	glm::vec2 size{3000.0f, 200.0f};    ///< Base width/height in image-pixel space.
	glm::ivec2 lights{14, 1};           ///< Discrete light grid (texels = physical lights).
	glm::vec2 anchor{2000.0f, 2500.0f}; ///< Center in image-pixel space.
	ofColor color{255, 255, 255, 100};  ///< Fill color with alpha.

	// Control config (JSON "control" block).
	float gazeXMin = -0.5f;             ///< Horizontal gaze mapped to position param 0 (left).
	float gazeXMax = 0.5f;              ///< Horizontal gaze mapped to position param 1 (right).
	float gazeYMin = -0.4f;             ///< Vertical gaze mapped to width param 0 (narrowest).
	float gazeYMax = 0.4f;              ///< Vertical gaze mapped to width param 1 (widest).
	int positionStates = 5;             ///< Number of horizontal position states.
	std::vector<int> widths{2, 5, 9, 14}; ///< Width states in lights, narrow -> wide (>= 1).
	float hysteresis = 0.35f;           ///< Band-exit margin, fraction of one band.
	float minDwellSeconds = 0.25f;      ///< Minimum time between state changes.
	float transitionSeconds = 0.15f;    ///< Ease duration toward a new target (~95%).
	float smoothing = 0.2f;             ///< Input low-pass time constant in seconds (0 = raw).

	// Runtime state.
	glm::vec2 gazeTarget{0.0f, 0.0f};   ///< Latest averaged gaze input.
	glm::vec2 gazeSmoothed{0.0f, 0.0f}; ///< Low-passed gaze input.
	QuantizedChannel positionChannel;
	QuantizedChannel widthChannel;
	float targetLeftLight = 0.0f;       ///< Integer-valued target edges, light units.
	float targetRightLight = 1.0f;
	float currentLeftLight = 0.0f;      ///< Eased edges the rasterizer draws.
	float currentRightLight = 1.0f;

	/// Supersampling factor: the continuous bar is drawn at lights * kSupersample
	/// resolution, then averaged down so edge lights get fractional brightness.
	/// MSAA on a 14x1 target would give far too few samples per light.
	static constexpr int kSupersample = 8;

	ofFbo lightsFbo;   ///< lights.w x lights.h — one texel per physical light.
	ofFbo lightsSsFbo; ///< Supersample target (lights * kSupersample).
	ofPixels lightsPixels; ///< CPU copy of lightsFbo, filled by getLightPixels().

	bool loaded = false;
};
