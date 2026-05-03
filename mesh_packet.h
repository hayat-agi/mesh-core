#ifndef MESH_PACKET_H
#define MESH_PACKET_H

#include <stdint.h>
#include <stddef.h>

#define MAX_PAYLOAD_LEN 64

// Packet struct as defined in the PRD
typedef struct __attribute__((packed)) {
    uint32_t msg_id;       // Unique message ID for duplicate detection

    uint16_t src_addr;     // Original source node
    uint16_t dst_addr;     // Final destination node

    uint16_t prev_hop;     // Last forwarding node
    uint16_t next_hop;     // Next intended hop (0xFFFF = broadcast)

    uint8_t  ttl;          // Decrement at each hop; drop at zero
    uint8_t  hop_count;    // Increment at each hop (for diagnostics / metrics)

    uint16_t seq_num;      // Per-source packet sequence number
    uint8_t  type;         // DATA / RREQ / RREP / RERR / BEACON / ACK

    int8_t   rssi;

    uint8_t  ai_priority;  // 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL
    uint8_t  payload_len;  // Actual payload length in bytes
    uint8_t  payload[MAX_PAYLOAD_LEN];  // Application payload buffer (V1 fixed size)

    uint16_t crc;          // CRC16 over header + payload_len bytes (excluding crc field)
} Packet;

void packet_init(Packet& p);
size_t packet_serialize(const Packet& p, uint8_t* buf, size_t buf_size);
bool packet_deserialize(Packet& p, const uint8_t* buf, size_t len);

#endif // MESH_PACKET_H
