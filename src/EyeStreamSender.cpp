#include "EyeStreamSender.h"

#include "EyeStreamProtocol.h"

#include <algorithm>
#include <cstring>
#include <random>

EyeStreamSender::~EyeStreamSender() {
	close();
}

//--------------------------------------------------------------
bool EyeStreamSender::setup(const Config & cfg) {
	close();
	config = cfg;

	if (!udp.Create()) {
		ofLogError("EyeStreamSender") << "failed to create UDP socket";
		return false;
	}
	if (!udp.Connect(config.targetIp.c_str(), static_cast<unsigned short>(config.targetPort))) {
		ofLogError("EyeStreamSender") << "failed to connect to "
			<< config.targetIp << ":" << config.targetPort;
		udp.Close();
		return false;
	}
	udp.SetNonBlocking(true);
	udp.SetSendBufferSize(4 * 1024 * 1024);

	if (config.packetPayloadBytes < 1) {
		config.packetPayloadBytes = 1440;
	}

	// New random session id per run so the receiver can tell a restarted sender
	// apart from the previous one (frameId restarts at 0 every run).
	std::random_device rd;
	sessionId = (static_cast<std::uint32_t>(rd()) << 16) ^ static_cast<std::uint32_t>(rd());
	if (sessionId == 0) {
		sessionId = 1;
	}

	socketReady = true;
	stopping = false;
	frameCounter = 0;
	lastSendTime = 0.0f;
	startThread();

	ofLogNotice("EyeStreamSender") << "streaming to " << config.targetIp << ":"
		<< config.targetPort << " (compression=" << config.compression
		<< ", payload=" << config.packetPayloadBytes
		<< ", fps_limit=" << config.fpsLimit
		<< ", session=" << sessionId << ")";
	return true;
}

//--------------------------------------------------------------
void EyeStreamSender::submit(const ofPixels & rgbPixels) {
	if (!socketReady) {
		return;
	}
	{
		std::lock_guard<std::mutex> lk(frameMutex);
		pendingFrame = rgbPixels; // overwrites any not-yet-sent frame
		hasPending = true;
	}
	frameCv.notify_one();
}

//--------------------------------------------------------------
void EyeStreamSender::close() {
	if (isThreadRunning()) {
		{
			std::lock_guard<std::mutex> lk(frameMutex);
			stopping = true;
		}
		frameCv.notify_one();
		waitForThread(false, 2000);
	}
	if (socketReady) {
		udp.Close();
		socketReady = false;
	}
}

//--------------------------------------------------------------
void EyeStreamSender::threadedFunction() {
	// Persistent buffer reused across iterations. swap() ping-pongs ownership
	// with pendingFrame, so neither buffer is ever left dangling and we avoid
	// per-frame (re)allocation. NOTE: do NOT use `frame = std::move(pendingFrame)`
	// here: ofPixels' move-assignment leaves the moved-from object pointing at
	// the same buffer (it only clears pixelsOwner), so submit()'s next copy would
	// reuse a freed buffer -> heap use-after-free.
	ofPixels frame;
	while (true) {
		{
			std::unique_lock<std::mutex> lk(frameMutex);
			frameCv.wait(lk, [this] { return hasPending || stopping; });
			if (stopping) {
				return;
			}
			frame.swap(pendingFrame);
			hasPending = false;
		}

		if (config.fpsLimit > 0) {
			const float minInterval = 1.0f / static_cast<float>(config.fpsLimit);
			const float now = ofGetElapsedTimef();
			const float wait = minInterval - (now - lastSendTime);
			if (wait > 0.0f) {
				ofSleepMillis(static_cast<int>(wait * 1000.0f));
			}
			lastSendTime = ofGetElapsedTimef();
		}

		sendFrame(frame);
	}
}

//--------------------------------------------------------------
void EyeStreamSender::sendFrame(const ofPixels & pixels) {
	if (!pixels.isAllocated() || pixels.getWidth() <= 0 || pixels.getHeight() <= 0) {
		return;
	}

	const std::uint16_t w = static_cast<std::uint16_t>(pixels.getWidth());
	const std::uint16_t h = static_cast<std::uint16_t>(pixels.getHeight());

	const char * payload = nullptr;
	std::size_t payloadSize = 0;
	std::uint8_t format = eyestream::kFormatRawRgb;

	ofBuffer jpegBuf;
	if (config.compression == "jpeg") {
		ofSaveImage(pixels, jpegBuf, OF_IMAGE_FORMAT_JPEG, OF_IMAGE_QUALITY_HIGH);
		payload = jpegBuf.getData();
		payloadSize = jpegBuf.size();
		format = eyestream::kFormatJpeg;
	} else {
		payload = reinterpret_cast<const char *>(pixels.getData());
		payloadSize = static_cast<std::size_t>(pixels.getTotalBytes());
	}

	if (payload == nullptr || payloadSize == 0) {
		return;
	}

	const std::size_t chunk = static_cast<std::size_t>(config.packetPayloadBytes);
	const std::size_t totalPackets = (payloadSize + chunk - 1) / chunk;
	if (totalPackets == 0 || totalPackets > 0xFFFF) {
		ofLogWarning("EyeStreamSender") << "frame too large for protocol ("
			<< payloadSize << " bytes); dropping";
		return;
	}

	const std::uint32_t frameId = frameCounter++;
	packetBuf.resize(eyestream::kHeaderBytes + chunk);

	for (std::size_t i = 0; i < totalPackets; ++i) {
		const std::size_t offset = i * chunk;
		const std::size_t thisPayload = std::min(chunk, payloadSize - offset);

		eyestream::PacketHeader header;
		header.sessionId = sessionId;
		header.frameId = frameId;
		header.totalBytes = static_cast<std::uint32_t>(payloadSize);
		header.payloadOffset = static_cast<std::uint32_t>(offset);
		header.width = w;
		header.height = h;
		header.format = format;
		header.totalPackets = static_cast<std::uint16_t>(totalPackets);
		header.packetIndex = static_cast<std::uint16_t>(i);
		header.payloadBytes = static_cast<std::uint16_t>(thisPayload);

		std::memcpy(packetBuf.data(), &header, eyestream::kHeaderBytes);
		std::memcpy(packetBuf.data() + eyestream::kHeaderBytes, payload + offset, thisPayload);

		udp.Send(packetBuf.data(), static_cast<int>(eyestream::kHeaderBytes + thisPayload));
	}
}
