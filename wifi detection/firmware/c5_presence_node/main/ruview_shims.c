/**
 * @file ruview_shims.c
 * @brief Link-time stand-ins for the RuView modules edge_processing.c calls.
 *
 * RuView's edge pipeline normally hands its packets to stream_sender (UDP to
 * the sensing server). This node has no AP to join, so the packets are decoded
 * here and printed on the USB console for the host dashboard instead.
 */

#include <stdio.h>
#include <string.h>

#include "edge_processing.h"
#include "csi_collector.h"
#include "mmwave_sensor.h"
#include "nvs_config.h"
#include "stream_sender.h"
#include "wasm_runtime.h"

#include "presence_node.h"

nvs_config_t g_nvs_config;

uint8_t csi_collector_get_node_id(void)
{
    return presence_node_id();
}

int stream_sender_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len < sizeof(uint32_t)) return -1;

    uint32_t magic;
    memcpy(&magic, data, sizeof(magic));

    if (magic == EDGE_VITALS_MAGIC && len >= sizeof(edge_vitals_pkt_t)) {
        edge_vitals_pkt_t pkt;
        memcpy(&pkt, data, sizeof(pkt));
        /* EDGE,flags,motion_energy,presence_score,n_persons,br_bpm,hr_bpm,rssi,ts_ms,dsp_hz */
        printf("EDGE,%u,%.5f,%.5f,%u,%.1f,%.1f,%d,%lu,%.1f\n",
               (unsigned)pkt.flags,
               pkt.motion_energy,
               pkt.presence_score,
               (unsigned)pkt.n_persons,
               pkt.breathing_rate / 100.0f,
               pkt.heartrate / 10000.0f,
               (int)pkt.rssi,
               (unsigned long)pkt.timestamp_ms,
               edge_get_sample_rate_hz());
    } else if (magic == EDGE_FEATURE_MAGIC && len >= sizeof(edge_feature_pkt_t)) {
        edge_feature_pkt_t pkt;
        memcpy(&pkt, data, sizeof(pkt));
        printf("FEAT,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               (unsigned)pkt.seq,
               pkt.features[0], pkt.features[1], pkt.features[2], pkt.features[3],
               pkt.features[4], pkt.features[5], pkt.features[6], pkt.features[7]);
    }
    return (int)len;
}

bool mmwave_sensor_get_state(mmwave_state_t *state)
{
    (void)state;
    return false;  /* No mmWave radar on this node. */
}

void wasm_runtime_on_frame(const float *phases, const float *amplitudes,
                           const float *variances, uint16_t n_sc,
                           const edge_vitals_pkt_t *vitals)
{
    (void)phases; (void)amplitudes; (void)variances; (void)n_sc; (void)vitals;
}
