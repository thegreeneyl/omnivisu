#pragma once

#include "ofMain.h"

/// JSON-driven mouth feature living in the same image-pixel coordinate space as
/// the mask (and the eye openings). Defined by a center anchor, a base size, and
/// an RGBA fill color. Drawn using the mask's cover-fit transform (scale +
/// clamped image offset) so it stays locked to the face at any window size.
///
/// The mouth's WIDTH is gaze-driven: the left edge follows the left eye and the
/// right edge follows the right eye, each sliding horizontally with the pupil's
/// position within its eye opening. The height is fixed (config size.h).
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
	/// anchor, color, and an optional "control" sub-block for the gaze mapping).
	/// Returns false (and logs) if the block is empty; the defaults below are
	/// kept in that case. (Re)allocates the light-grid FBOs.
	bool load(const ofJson & mouthJson);

	bool isLoaded() const { return loaded; }

	/// Feeds the latest per-eye normalized horizontal gaze (display orientation:
	/// negative = looking left, positive = looking right, 0 = centered). These
	/// are the targets that update() smooths toward.
	void setGaze(float leftGazeX, float rightGazeX);

	/// Advances the edge smoothing by dt seconds.
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
	/// Maps a raw gaze value to a 0..1 travel parameter via the input range.
	float gazeToParam(float gaze) const;

	/// (Re)allocates the lights + supersample FBOs to the current grid size.
	void allocateLightFbos();

	/// Rasterizes the current smoothed mouth edges into the light grid:
	/// supersampled draw, then area-downsample into lightsFbo.
	void renderLights();

	glm::vec2 size{3000.0f, 200.0f};    ///< Base width/height in image-pixel space.
	glm::ivec2 lights{14, 1};           ///< Discrete light grid (texels = physical lights).
	glm::vec2 anchor{2000.0f, 2500.0f}; ///< Center in image-pixel space.
	ofColor color{255, 255, 255, 100};  ///< Fill color with alpha.

	// Gaze -> edge-position mapping. Offsets are in half-width units relative to
	// the center: -1 = the rectangle's left edge, +1 = its right edge, 0 = center.
	// As gaze sweeps left->right, each edge lerps from its first to its second
	// value, so by default the right edge travels from 1/3 left of center out to
	// the right edge, and the left edge from the left edge in to 1/3 right of
	// center.
	float gazeInMin = -0.5f;            ///< Gaze mapped to travel param 0 (full left).
	float gazeInMax = 0.5f;             ///< Gaze mapped to travel param 1 (full right).
	float leftEdgeExtend = -1.0f;       ///< Left edge when looking left (param 0).
	float leftEdgeRetract = 0.333f;     ///< Left edge when looking right (param 1).
	float rightEdgeRetract = -0.333f;   ///< Right edge when looking left (param 0).
	float rightEdgeExtend = 1.0f;       ///< Right edge when looking right (param 1).
	float smoothing = 0.0f;             ///< 0 = none .. 1 = ~0.5 s time constant.

	float leftGazeTarget = 0.0f;
	float rightGazeTarget = 0.0f;
	float leftGazeSmoothed = 0.0f;
	float rightGazeSmoothed = 0.0f;

	/// Supersampling factor: the continuous bar is drawn at lights * kSupersample
	/// resolution, then averaged down so edge lights get fractional brightness.
	/// MSAA on a 14x1 target would give far too few samples per light.
	static constexpr int kSupersample = 8;

	ofFbo lightsFbo;   ///< lights.w x lights.h — one texel per physical light.
	ofFbo lightsSsFbo; ///< Supersample target (lights * kSupersample).
	ofPixels lightsPixels; ///< CPU copy of lightsFbo, filled by getLightPixels().

	bool loaded = false;
};
