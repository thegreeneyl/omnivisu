#include "MaskLayout.h"

#include <algorithm>

namespace {

ofRectangle readRect(const ofJson & j, const ofRectangle & fallback) {
	ofRectangle r = fallback;
	if (j.contains("x")) r.x = j.value("x", r.x);
	if (j.contains("y")) r.y = j.value("y", r.y);
	if (j.contains("w")) r.width = j.value("w", r.width);
	if (j.contains("h")) r.height = j.value("h", r.height);
	return r;
}

ofRectangle mapRect(const ofRectangle & src, float scale, float imgX, float imgY) {
	return ofRectangle(imgX + src.x * scale,
		imgY + src.y * scale,
		src.width * scale,
		src.height * scale);
}

} // namespace

//--------------------------------------------------------------
bool MaskLayout::load(const std::string & path) {
	loaded = false;

	ofFile file(path);
	if (!file.exists()) {
		ofLogWarning("MaskLayout") << "layout file not found: " << path;
		return false;
	}

	ofJson json;
	try {
		json = ofLoadJson(path);
	} catch (const std::exception & e) {
		ofLogError("MaskLayout") << "failed to parse " << path << ": " << e.what();
		return false;
	}
	if (json.is_null() || json.empty()) {
		ofLogError("MaskLayout") << "empty or invalid JSON: " << path;
		return false;
	}

	const std::string imageName = json.value("mask_image", std::string("260408-TGE-OMNI-FrantalView.png"));
	if (!maskImage.load(imageName)) {
		ofLogError("MaskLayout") << "failed to load mask image: " << imageName;
		return false;
	}

	// Prefer the actual loaded image size; fall back to JSON image_size.
	imageSize.x = static_cast<int>(maskImage.getWidth());
	imageSize.y = static_cast<int>(maskImage.getHeight());
	if (json.contains("image_size")) {
		const auto & is = json["image_size"];
		if (imageSize.x <= 0) imageSize.x = is.value("w", 4000);
		if (imageSize.y <= 0) imageSize.y = is.value("h", 4000);
	}

	leftEye = readRect(json.value("left_eye", ofJson::object()),
		ofRectangle(1030, 1073, 414, 280));
	rightEye = readRect(json.value("right_eye", ofJson::object()),
		ofRectangle(2551, 1071, 414, 280));

	if (json.contains("left_eye")) {
		leftStreamIndex = json["left_eye"].value("stream", leftStreamIndex);
	}
	if (json.contains("right_eye")) {
		rightStreamIndex = json["right_eye"].value("stream", rightStreamIndex);
	}

	// Anchor defaults to the image center, then to JSON anchor if present.
	anchor = glm::vec2(imageSize.x * 0.5f, imageSize.y * 0.5f);
	if (json.contains("anchor")) {
		const auto & a = json["anchor"];
		anchor.x = a.value("x", anchor.x);
		anchor.y = a.value("y", anchor.y);
	}

	loaded = true;
	ofLogNotice("MaskLayout") << "loaded " << imageName << " (" << imageSize.x
		<< "x" << imageSize.y << "), anchor (" << anchor.x << ", " << anchor.y << ")";
	return true;
}

//--------------------------------------------------------------
MaskLayout::ScreenLayout MaskLayout::compute(float screenW, float screenH) const {
	ScreenLayout out;

	const float imgW = static_cast<float>(imageSize.x);
	const float imgH = static_cast<float>(imageSize.y);
	if (imgW <= 0.0f || imgH <= 0.0f) {
		return out;
	}

	// Cover fit: scale so the image fully covers the window (no black bars).
	const float scale = std::max(screenW / imgW, screenH / imgH);
	const float scaledW = imgW * scale;
	const float scaledH = imgH * scale;

	// Place the image so the anchor maps to the window center, then clamp so
	// no image edge leaves a gap (offsets are <= 0 and >= screen - scaled).
	const float desiredX = screenW * 0.5f - anchor.x * scale;
	const float desiredY = screenH * 0.5f - anchor.y * scale;
	const float imgX = std::min(0.0f, std::max(screenW - scaledW, desiredX));
	const float imgY = std::min(0.0f, std::max(screenH - scaledH, desiredY));

	out.scale = scale;
	out.imgX = imgX;
	out.imgY = imgY;
	out.leftEyeScreen = mapRect(leftEye, scale, imgX, imgY);
	out.rightEyeScreen = mapRect(rightEye, scale, imgX, imgY);
	return out;
}

//--------------------------------------------------------------
void MaskLayout::drawMask(const ScreenLayout & layout) const {
	if (!loaded) {
		return;
	}
	const float scaledW = imageSize.x * layout.scale;
	const float scaledH = imageSize.y * layout.scale;
	ofSetColor(255);
	maskImage.draw(layout.imgX, layout.imgY, scaledW, scaledH);
}
