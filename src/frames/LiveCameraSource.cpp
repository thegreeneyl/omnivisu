#if !defined(OMNIVISU_NO_CAMERA)

#include "LiveCameraSource.h"

#include <exception>

namespace {

ofxIdsPeak::DeviceSelect deviceSelectFromString(const std::string & s) {
	if (s == "serial") return ofxIdsPeak::DeviceSelect::Serial;
	if (s == "model") return ofxIdsPeak::DeviceSelect::Model;
	if (s == "index") return ofxIdsPeak::DeviceSelect::Index;
	return ofxIdsPeak::DeviceSelect::Any;
}

} // namespace

//--------------------------------------------------------------
void LiveCameraSource::selectDevice(const std::string & selectBy,
	const std::string & selectValue) {
	const ofxIdsPeak::DeviceSelect selectMode = deviceSelectFromString(selectBy);
	int selectIndex = 0;
	if (selectMode == ofxIdsPeak::DeviceSelect::Index) {
		try {
			selectIndex = std::stoi(selectValue);
		} catch (const std::exception &) {
			ofLogWarning("LiveCameraSource") << config.name
				<< ": camera select index '" << selectValue << "' is not an integer; using 0";
		}
	}
	grabber_.setDeviceSelector(selectMode, selectValue, selectIndex);
}

//--------------------------------------------------------------
void LiveCameraSource::startRecording(const std::string & dir) {
	stopRecording();
	// dir is relative to bin/data; create it (recursively) before writing.
	ofDirectory::createDirectory(dir, true, true);
	recorder = std::make_unique<FrameRecorder>();
	recorder->start(dir, config.recordingFormat);
	ofLogNotice("LiveCameraSource") << config.name << ": recording to " << dir;
}

//--------------------------------------------------------------
void LiveCameraSource::stopRecording() {
	if (recorder) {
		recorder->stop();
		recorder.reset();
		ofLogNotice("LiveCameraSource") << config.name << ": recording stopped";
	}
}

#endif // !defined(OMNIVISU_NO_CAMERA)
