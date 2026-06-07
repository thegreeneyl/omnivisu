#pragma once

#include "ofMain.h"

#include <string>

/// JSON-driven full-screen mask. Holds a mask PNG with transparent eye
/// openings and per-eye rects defined in image-pixel space, then "cover"-fits
/// the image to the window (no black bars) and maps the eye rects to screen.
class MaskLayout {
public:
	struct ScreenLayout {
		float scale = 1.0f;
		float imgX = 0.0f;
		float imgY = 0.0f;
		ofRectangle leftEyeScreen;
		ofRectangle rightEyeScreen;
	};

	/// Loads the layout JSON and the referenced PNG. Returns false (and logs)
	/// if the JSON is missing/unparseable or the image fails to load.
	bool load(const std::string & path);

	bool isLoaded() const { return loaded; }

	/// Computes the cover-fit scale, clamped image offset, and per-eye screen
	/// rects for the given window size.
	ScreenLayout compute(float screenW, float screenH) const;

	/// Draws the mask PNG full-screen using the supplied layout.
	void drawMask(const ScreenLayout & layout) const;

	int getLeftStreamIndex() const { return leftStreamIndex; }
	int getRightStreamIndex() const { return rightStreamIndex; }

private:
	ofImage maskImage;
	glm::ivec2 imageSize{4000, 4000};
	glm::vec2 anchor{2000.0f, 2000.0f};
	ofRectangle leftEye;
	ofRectangle rightEye;
	int leftStreamIndex = 0;
	int rightStreamIndex = 1;
	bool loaded = false;
};
