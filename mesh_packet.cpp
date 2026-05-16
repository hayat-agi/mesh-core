#include "mesh_packet.h"

// Simple CRC16-CCITT calculation (polynomial 0x1021)
static uint16_t calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void packet_init(Packet& p) {
    p.msg_id = 0;
    p.src_addr = 0;
    p.dst_addr = 0;
    p.prev_hop = 0;
    p.next_hop = 0;
    p.ttl = 0;
    p.hop_count = 0;
    p.seq_num = 0;
    p.type = 0;
    p.ai_priority = 0;
    p.payload_len = 0;
    p.path_len = 0;
    for (int i = 0; i < MESH_PATH_MAX; ++i) {
        p.path[i] = 0;
    }
    for (int i = 0; i < MAX_PAYLOAD_LEN; ++i) {
        p.payload[i] = 0;
    }
    p.crc = 0;
}

// Write a little-endian uint16_t to buffer
static inline void write_u16_le(uint8_t* buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

// Write a little-endian uint32_t to buffer
static inline void write_u32_le(uint8_t* buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

// Read a little-endian uint16_t from buffer
static inline uint16_t read_u16_le(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

// Read a little-endian uint32_t from buffer
static inline uint32_t read_u32_le(const uint8_t* buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

// Header size is everything before the payload, including the hop_path
// metadata (path_len byte + MESH_PATH_MAX u16 entries, fixed on the wire).
// Core: msg_id(4) + src(2) + dst(2) + prev(2) + next(2) + ttl(1) + hop(1)
//       + seq(2) + type(1) + ai(1) + plen(1) = 19 bytes
// Path: path_len(1) + path[MESH_PATH_MAX](2 * 8 = 16) = 17 bytes
// Total: 36 bytes
#define HEADER_SIZE 36

size_t packet_serialize(const Packet& p, uint8_t* buf, size_t buf_size) {
    uint8_t plen = p.payload_len;
    if (plen > MAX_PAYLOAD_LEN) {
        plen = MAX_PAYLOAD_LEN;
    }

    size_t total_size = HEADER_SIZE + plen + 2; // +2 for CRC
    if (buf_size < total_size) {
        return 0; // buffer too small
    }

    size_t offset = 0;
    write_u32_le(&buf[offset], p.msg_id); offset += 4;
    write_u16_le(&buf[offset], p.src_addr); offset += 2;
    write_u16_le(&buf[offset], p.dst_addr); offset += 2;
    write_u16_le(&buf[offset], p.prev_hop); offset += 2;
    write_u16_le(&buf[offset], p.next_hop); offset += 2;
    buf[offset++] = p.ttl;
    buf[offset++] = p.hop_count;
    write_u16_le(&buf[offset], p.seq_num); offset += 2;
    buf[offset++] = p.type;
    buf[offset++] = p.ai_priority;
    buf[offset++] = plen;

    // hop_path metadata: fixed-size on wire so deserialize stays stateless.
    uint8_t pn = p.path_len > MESH_PATH_MAX ? MESH_PATH_MAX : p.path_len;
    buf[offset++] = pn;
    for (uint8_t i = 0; i < MESH_PATH_MAX; ++i) {
        write_u16_le(&buf[offset], p.path[i]); offset += 2;
    }

    // Payload
    for (size_t i = 0; i < plen; ++i) {
        buf[offset++] = p.payload[i];
    }

    // CRC is over the header (incl. hop_path) and payload
    uint16_t crc = calculate_crc16(buf, offset);
    write_u16_le(&buf[offset], crc); offset += 2;

    return offset;
}

bool packet_deserialize(Packet& p, const uint8_t* buf, size_t len) {
    if (len < HEADER_SIZE + 2) {
        return false; // Not enough data for header + CRC
    }

    size_t offset = 0;
    p.msg_id = read_u32_le(&buf[offset]); offset += 4;
    p.src_addr = read_u16_le(&buf[offset]); offset += 2;
    p.dst_addr = read_u16_le(&buf[offset]); offset += 2;
    p.prev_hop = read_u16_le(&buf[offset]); offset += 2;
    p.next_hop = read_u16_le(&buf[offset]); offset += 2;
    p.ttl = buf[offset++];
    p.hop_count = buf[offset++];
    p.seq_num = read_u16_le(&buf[offset]); offset += 2;
    p.type = buf[offset++];
    p.ai_priority = buf[offset++];

    uint8_t plen = buf[offset++];
    if (plen > MAX_PAYLOAD_LEN) {
        return false; // Invalid payload length
    }

    // hop_path: fixed-size on wire, validate path_len bound
    uint8_t pn = buf[offset++];
    if (pn > MESH_PATH_MAX) {
        return false;
    }
    p.path_len = pn;
    for (uint8_t i = 0; i < MESH_PATH_MAX; ++i) {
        p.path[i] = read_u16_le(&buf[offset]); offset += 2;
    }

    if (len < HEADER_SIZE + plen + 2) {
        return false; // Not enough data for the specified payload length + CRC
    }

    // Read Payload
    for (size_t i = 0; i < plen; ++i) {
        p.payload[i] = buf[offset++];
    }
    p.payload_len = plen;

    // Validate CRC
    uint16_t expected_crc = read_u16_le(&buf[offset]);
    // The CRC is calculated over buf from 0 to HEADER_SIZE + plen
    uint16_t calculated_crc = calculate_crc16(buf, HEADER_SIZE + plen);

    if (expected_crc != calculated_crc) {
        return false; // CRC mismatch
    }

    return true;
}
