#pragma once

#include <cstdint>

/// Shared wire protocol for the eye video UDP stream. This header MUST stay
/// byte-for-byte identical in the sender (omnivisu) and receiver
/// (omnivisu_receiver) projects.
///
/// Each UDP datagram is one PacketHeader immediately followed by `payloadBytes`
/// of frame data. A full frame is split across `totalPackets` datagrams that
/// share the same `frameId`; the receiver reassembles by frameId and discards
/// any frame it cannot complete. All fields are little-endian (both ends are
/// little-endian x86/arm64, so we send the struct as-is without byte swapping).
///
/// `sessionId` is a random value chosen once per sender run. `frameId` is only
/// monotonic *within* a session and resets to 0 every time the sender restarts,
/// so the receiver MUST key its newest-frame-wins ordering on sessionId and
/// reset all reassembly state whenever the sessionId changes; otherwise a
/// restarted sender's low frameIds look "stale" and get dropped forever.
namespace eyestream {

constexpr std::uint32_t kMagic = 0x45594553; // "EYES"
constexpr std::uint8_t kFormatRawRgb = 0;
constexpr std::uint8_t kFormatJpeg = 1;

#pragma pack(push, 1)
struct PacketHeader {
	std::uint32_t magic = kMagic;
	std::uint32_t sessionId = 0; ///< Unique per sender run; receiver resets on change.
	std::uint32_t frameId = 0;
	std::uint32_t totalBytes = 0;    ///< Total payload size of the whole frame.
	std::uint32_t payloadOffset = 0; ///< Byte offset of this chunk within the frame.
	std::uint16_t width = 0;
	std::uint16_t height = 0;
	std::uint16_t totalPackets = 0;
	std::uint16_t packetIndex = 0;
	std::uint16_t payloadBytes = 0;
	std::uint8_t format = kFormatRawRgb;
};
#pragma pack(pop)

constexpr int kHeaderBytes = static_cast<int>(sizeof(PacketHeader));

} // namespace eyestream
