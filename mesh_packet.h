#ifndef MESH_PACKET_H
#define MESH_PACKET_H

#include <stdint.h>
#include <stddef.h>

// Paket başlığı + payload toplamda LoRa SX127x MTU'suna sığmali (255 byte).
// Başlık ~13 byte, framing 3 byte => payload için ~239 byte kalıyor.
// Geniş mesaj + household JSON özelliklerini desteklemek için 200'e çıkarıldı.
#define MAX_PAYLOAD_LEN 200

// Ordered chain of node addresses a packet has traversed. Source seeds
// path[0] = LOCAL_ADDR, path_len = 1; each relay appends its address
// before forwarding so the gateway can uplink the full chain. 8 supports
// a 7-hop mesh, well beyond the planned 5-node layout. Fixed-size on the
// wire to keep deserialization stateless.
#define MESH_PATH_MAX 8

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

    uint8_t  path_len;                  // # of valid entries in path[]
    uint16_t path[MESH_PATH_MAX];       // chain of traversed node addrs

    uint8_t  payload[MAX_PAYLOAD_LEN];  // Application payload buffer (V1 fixed size)

    uint16_t crc;          // CRC16 over header + payload_len bytes (excluding crc field)
} Packet;

void packet_init(Packet& p);
size_t packet_serialize(const Packet& p, uint8_t* buf, size_t buf_size);
bool packet_deserialize(Packet& p, const uint8_t* buf, size_t len);

#endif // MESH_PACKET_H
