#pragma once

#include "ofMain.h"
#include "ofxNetwork.h"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

/// Streams RGB frames to a remote receiver over UDP. The GL/main thread hands
/// off the latest frame via submit(); a dedicated worker thread packetizes and
/// sends it so networking never stalls rendering. Only the most recent frame is
/// ever kept pending (older frames are dropped) to minimize latency.
class EyeStreamSender : public ofThread {
public:
	struct Config {
		std::string targetIp = "127.0.0.1";
		int targetPort = 12345;
		std::string compression = "none"; ///< "none" (raw RGB) or "jpeg".
		int packetPayloadBytes = 1440;
		int fpsLimit = 0; ///< Max send rate; 0 = unlimited.
	};

	~EyeStreamSender() override;

	/// Opens the UDP socket and starts the worker thread. Returns false on
	/// socket failure (and logs); submit() then becomes a no-op.
	bool setup(const Config & cfg);

	/// Copies the latest frame for transmission. Must be RGB (3 channels).
	/// Safe to call every frame from the main thread.
	void submit(const ofPixels & rgbPixels);

	/// Stops the worker thread and closes the socket.
	void close();

	bool isActive() const { return socketReady; }

private:
	void threadedFunction() override;
	void sendFrame(const ofPixels & pixels);

	Config config;
	ofxUDPManager udp;
	bool socketReady = false;

	std::mutex frameMutex;
	std::condition_variable frameCv;
	ofPixels pendingFrame;
	bool hasPending = false;
	bool stopping = false;

	std::uint32_t sessionId = 0;
	std::uint32_t frameCounter = 0;
	float lastSendTime = 0.0f;
	std::vector<char> packetBuf;
};
