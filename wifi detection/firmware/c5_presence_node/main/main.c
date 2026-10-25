/**
 * @file main.c
 * @brief ESP32-C5 presence node — multi-peer WiFi CSI mesh with RuView edge DSP.
 *
 * Every node runs this same image. Each node broadcasts an ESP-NOW beacon at
 * PN_TX_HZ on a fixed channel and captures the CSI of the beacons it hears from
 * every other node, so N nodes measure all N*(N-1) directed links.
 *
 * Each beacon also carries the sender's latest amplitude measurement of up to
 * PN_MAX_REPORTS of its own links. A node on USB prints those relayed reports,
 * so a single USB-connected "gateway" node delivers every link in the mesh,
 * including links between nodes that are only powered.
 *
 * The CSI of the first peer heard also feeds RuView's edge pipeline
 * (edge_processing.c, vendored unmodified) at its designed 20 Hz rate.
 *
 * Console lines (USB Serial/JTAG):
 *   HELLO,<mac>,<channel>
 *   CSIA,<tx_mac>,<seq>,<t_us>,<rssi>,<noise_floor>,<n_sc>,<hex amplitude bytes>
 *   RLY,<rx_mac>,<tx_mac>,<amp_seq>,<rssi>,<n_sc>,<hex amplitude bytes>
 *   EDGE,... / FEAT,...   (see ruview_shims.c)
 *   STAT,<mac>,<first_peer>,<channel>,<tx_ok>,<tx_fail>,<beacons_rx>,<csi_rx>,<dropped>
 *   PEERS,<mac>,<peer_mac>:<csi_count>:<rssi>,...
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "edge_processing.h"
#include "presence_node.h"

static const char *TAG = "presence_node";

#define PN_CHANNEL          6       /**< 2.4 GHz channel shared by all nodes. */
#define PN_TX_HZ            50      /**< ESP-NOW beacons per second per node. */
#define PN_EDGE_HZ          20      /**< RuView edge DSP design rate (CONFIG_EDGE_DSP_SAMPLE_HZ default). */
#define PN_BEACON_MAGIC     0x4E505652u  /**< "RVPN" */
#define PN_BEACON_VERSION   2
#define PN_MAX_PEERS        8
#define PN_MAX_SC           64      /**< Amplitude bins kept per link (C5 HT20 gives 57). */
#define PN_MAX_REPORTS      3       /**< Link reports per beacon: 14 + 3 * 74 = 236 B <= 250 B. */
#define PN_MAX_CSI_BYTES    256
#define PN_QUEUE_LEN        32
#define PN_RELAY_MIN_US     35000   /**< Print each relayed link at most ~28 Hz. */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t uptime_ms;
    uint8_t  version;
    uint8_t  n_reports;
} pn_beacon_hdr_t;

typedef struct __attribute__((packed)) {
    uint8_t  tx_mac[6];     /**< Node whose beacons the sender measured. */
    int8_t   rssi;
    uint8_t  n_sc;
    uint16_t amp_seq;
    uint8_t  amp[PN_MAX_SC];
} pn_report_t;

typedef struct {
    uint8_t  mac[6];
    uint32_t beacons;
    uint32_t csi;
    int8_t   rssi;
    uint8_t  n_sc;
    uint16_t amp_seq;
    uint8_t  amp[PN_MAX_SC];
    /* Relay bookkeeping for reports this peer sends about its own links. */
    uint8_t  relay_tx[PN_MAX_PEERS][6];
    uint16_t relay_seq[PN_MAX_PEERS];
    int64_t  relay_us[PN_MAX_PEERS];
    uint8_t  n_relay;
} pn_peer_t;

typedef enum { PN_ITEM_CSI, PN_ITEM_RELAY } pn_item_kind_t;

typedef struct {
    pn_item_kind_t kind;
    uint8_t  peer_idx;          /**< CSI: transmitting peer. */
    uint8_t  rx_mac[6];         /**< RELAY: node that measured the link. */
    uint8_t  tx_mac[6];         /**< RELAY: node it measured. */
    int64_t  t_us;
    uint32_t seq;
    int8_t   rssi;
    int8_t   noise_floor;
    uint8_t  first_word_invalid;
    uint16_t len;               /**< CSI: raw I/Q bytes. RELAY: amplitude bins. */
    uint8_t  buf[PN_MAX_CSI_BYTES];
} pn_item_t;

static const uint8_t s_broadcast[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static uint8_t           s_own_mac[6];
static pn_peer_t         s_peers[PN_MAX_PEERS];
static volatile uint8_t  s_n_peers;
static portMUX_TYPE      s_peer_lock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t     s_queue;

static volatile uint32_t s_tx_ok;
static volatile uint32_t s_tx_fail;
static volatile uint32_t s_beacons_rx;
static volatile uint32_t s_csi_rx;
static volatile uint32_t s_dropped;
static int64_t           s_next_edge_us;

uint8_t presence_node_id(void)
{
    return s_own_mac[5];
}

static int find_peer(const uint8_t *mac)
{
    uint8_t n = s_n_peers;
    for (int i = 0; i < n; i++) {
        if (memcmp(s_peers[i].mac, mac, 6) == 0) return i;
    }
    return -1;
}

/* WiFi task only (ESP-NOW receive callback), so additions never race each other. */
static int add_peer(const uint8_t *mac)
{
    int idx = find_peer(mac);
    if (idx >= 0 || s_n_peers >= PN_MAX_PEERS) return idx;
    idx = s_n_peers;
    memset(&s_peers[idx], 0, sizeof(s_peers[idx]));
    memcpy(s_peers[idx].mac, mac, 6);
    s_n_peers = idx + 1;
    return idx;
}

static void queue_item(const pn_item_t *item)
{
    if (xQueueSend(s_queue, item, 0) != pdTRUE) s_dropped++;
}

/* ---- ESP-NOW ---------------------------------------------------------- */

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || data == NULL || len < 12) return;

    uint32_t magic;
    memcpy(&magic, data, sizeof(magic));
    if (magic != PN_BEACON_MAGIC) return;

    int idx = add_peer(info->src_addr);
    if (idx < 0) return;
    pn_peer_t *sender = &s_peers[idx];
    sender->beacons++;
    s_beacons_rx++;

    if (len < (int)sizeof(pn_beacon_hdr_t)) return;  /* version 1 beacon: no reports */
    const pn_beacon_hdr_t *hdr = (const pn_beacon_hdr_t *)data;
    if (hdr->version < 2) return;

    int64_t now_us = esp_timer_get_time();
    static pn_item_t item;  /* WiFi task only; copied into the queue. */
    for (int r = 0; r < hdr->n_reports; r++) {
        size_t off = sizeof(pn_beacon_hdr_t) + (size_t)r * sizeof(pn_report_t);
        if (off + sizeof(pn_report_t) > (size_t)len) break;
        pn_report_t rep;
        memcpy(&rep, data + off, sizeof(rep));
        /* This node measures the reverse of that link itself. */
        if (memcmp(rep.tx_mac, s_own_mac, 6) == 0 || rep.n_sc == 0) continue;

        int slot = -1;
        for (int k = 0; k < sender->n_relay; k++) {
            if (memcmp(sender->relay_tx[k], rep.tx_mac, 6) == 0) { slot = k; break; }
        }
        if (slot < 0) {
            if (sender->n_relay >= PN_MAX_PEERS) continue;
            slot = sender->n_relay++;
            memcpy(sender->relay_tx[slot], rep.tx_mac, 6);
            sender->relay_seq[slot] = (uint16_t)(rep.amp_seq - 1);
            sender->relay_us[slot] = 0;
        }
        if (sender->relay_seq[slot] == rep.amp_seq) continue;               /* no new data */
        if (now_us - sender->relay_us[slot] < PN_RELAY_MIN_US) continue;    /* rate limit */
        sender->relay_seq[slot] = rep.amp_seq;
        sender->relay_us[slot] = now_us;

        item.kind = PN_ITEM_RELAY;
        memcpy(item.rx_mac, info->src_addr, 6);
        memcpy(item.tx_mac, rep.tx_mac, 6);
        item.seq = rep.amp_seq;
        item.rssi = rep.rssi;
        item.len = rep.n_sc > PN_MAX_SC ? PN_MAX_SC : rep.n_sc;
        memcpy(item.buf, rep.amp, item.len);
        queue_item(&item);
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    static uint8_t frame[sizeof(pn_beacon_hdr_t) + PN_MAX_REPORTS * sizeof(pn_report_t)];
    pn_beacon_hdr_t *hdr = (pn_beacon_hdr_t *)frame;
    hdr->magic = PN_BEACON_MAGIC;
    hdr->version = PN_BEACON_VERSION;
    uint8_t rr = 0;  /* round-robin start when there are more peers than report slots */
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / PN_TX_HZ);

    while (1) {
        hdr->seq++;
        hdr->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);

        uint8_t n = 0, np = s_n_peers;
        taskENTER_CRITICAL(&s_peer_lock);
        for (uint8_t k = 0; k < np && n < PN_MAX_REPORTS; k++) {
            const pn_peer_t *p = &s_peers[(rr + k) % np];
            if (p->n_sc == 0) continue;
            pn_report_t *rep = (pn_report_t *)(frame + sizeof(pn_beacon_hdr_t) + n * sizeof(pn_report_t));
            memcpy(rep->tx_mac, p->mac, 6);
            rep->rssi = p->rssi;
            rep->n_sc = p->n_sc;
            rep->amp_seq = p->amp_seq;
            memcpy(rep->amp, p->amp, PN_MAX_SC);
            n++;
        }
        taskEXIT_CRITICAL(&s_peer_lock);
        if (np > PN_MAX_REPORTS) rr = (uint8_t)((rr + PN_MAX_REPORTS) % np);
        hdr->n_reports = n;

        size_t len = sizeof(pn_beacon_hdr_t) + n * sizeof(pn_report_t);
        if (esp_now_send(s_broadcast, frame, len) == ESP_OK) {
            s_tx_ok++;
        } else {
            s_tx_fail++;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

/* ---- CSI capture ------------------------------------------------------ */

static void csi_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (info == NULL || info->buf == NULL || info->len == 0) return;
    int idx = find_peer(info->mac);
    if (idx < 0) return;

    uint16_t len = info->len > PN_MAX_CSI_BYTES ? PN_MAX_CSI_BYTES : info->len;
    int64_t now_us = esp_timer_get_time();
    s_csi_rx++;
    s_peers[idx].csi++;

    /* RuView's edge pipeline tracks one link: the first peer heard. */
    if (idx == 0 && now_us >= s_next_edge_us) {
        edge_enqueue_csi((const uint8_t *)info->buf, len,
                         info->rx_ctrl.rssi, info->rx_ctrl.channel);
        s_next_edge_us += 1000000 / PN_EDGE_HZ;
        if (s_next_edge_us < now_us) s_next_edge_us = now_us + 1000000 / PN_EDGE_HZ;
    }

    static pn_item_t item;  /* WiFi task only; copied into the queue. */
    item.kind = PN_ITEM_CSI;
    item.peer_idx = (uint8_t)idx;
    item.t_us = now_us;
    item.seq = s_peers[idx].csi;
    item.len = len;
    item.rssi = info->rx_ctrl.rssi;
    item.noise_floor = info->rx_ctrl.noise_floor;
    item.first_word_invalid = info->first_word_invalid;
    memcpy(item.buf, info->buf, len);
    queue_item(&item);
}

static char *put_hex(char *out, const uint8_t *data, uint16_t n)
{
    static const char hex[] = "0123456789abcdef";
    for (uint16_t i = 0; i < n; i++) {
        *out++ = hex[data[i] >> 4];
        *out++ = hex[data[i] & 0x0f];
    }
    return out;
}

static void print_task(void *arg)
{
    (void)arg;
    static pn_item_t item;
    static uint8_t amp[PN_MAX_SC];
    static char line[96 + PN_MAX_SC * 2];

    while (1) {
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) continue;
        int n;
        if (item.kind == PN_ITEM_CSI) {
            pn_peer_t *p = &s_peers[item.peer_idx];
            uint16_t n_sc = item.len / 2;
            if (n_sc > PN_MAX_SC) n_sc = PN_MAX_SC;
            for (uint16_t i = 0; i < n_sc; i++) {
                float re = (int8_t)item.buf[2 * i], im = (int8_t)item.buf[2 * i + 1];
                float a = sqrtf(re * re + im * im) + 0.5f;
                amp[i] = a > 255.0f ? 255 : (uint8_t)a;
            }
            if (item.first_word_invalid && n_sc >= 2) amp[0] = amp[1] = 0;

            /* Latest measurement of this link, shared in our beacons. */
            taskENTER_CRITICAL(&s_peer_lock);
            memcpy(p->amp, amp, n_sc);
            p->n_sc = (uint8_t)n_sc;
            p->rssi = item.rssi;
            p->amp_seq++;
            taskEXIT_CRITICAL(&s_peer_lock);

            n = snprintf(line, sizeof(line), "CSIA," MACSTR ",%lu,%lld,%d,%d,%u,",
                         MAC2STR(p->mac), (unsigned long)item.seq, (long long)item.t_us,
                         (int)item.rssi, (int)item.noise_floor, (unsigned)n_sc);
            n = (int)(put_hex(line + n, amp, n_sc) - line);
        } else {
            n = snprintf(line, sizeof(line), "RLY," MACSTR "," MACSTR ",%u,%d,%u,",
                         MAC2STR(item.rx_mac), MAC2STR(item.tx_mac), (unsigned)item.seq,
                         (int)item.rssi, (unsigned)item.len);
            n = (int)(put_hex(line + n, item.buf, item.len) - line);
        }
        line[n++] = '\n';
        fwrite(line, 1, n, stdout);
    }
}

/* ---- Setup ------------------------------------------------------------ */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY));
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(WIFI_IF_STA, &protocols));
    wifi_bandwidths_t bandwidths = { .ghz_2g = WIFI_BW_HT20, .ghz_5g = WIFI_BW_HT20 };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(PN_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_own_mac));
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer = {
        .channel = PN_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, s_broadcast, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* 802.11b DSSS frames carry no CSI; send HT20 MCS0 so every frame has an HT-LTF. */
    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_HT20,
        .rate = WIFI_PHY_RATE_MCS0_LGI,
        .ersu = false,
        .dcm = false,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate));
}

static void csi_init(void)
{
    /* Promiscuous MGMT capture, as in RuView's csi_collector: ESP-NOW frames are
     * vendor action (management) frames. */
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));

    wifi_csi_config_t csi_config;
    memset(&csi_config, 0, sizeof(csi_config));
    csi_config.enable = 1U;
    csi_config.acquire_csi_legacy = 1U;
    csi_config.acquire_csi_ht20 = 1U;
    csi_config.acquire_csi_ht40 = 1U;
    csi_config.acquire_csi_su = 1U;
    csi_config.acquire_csi_mu = 1U;
    csi_config.acquire_csi_dcm = 1U;
    csi_config.acquire_csi_beamformed = 1U;
    csi_config.acquire_csi_force_lltf = 0U;
    csi_config.acquire_csi_vht = 1U;
    csi_config.acquire_csi_he_stbc_mode = 2U;
    csi_config.val_scale_cfg = 0U;
    csi_config.dump_ack_en = 0U;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static void print_status(void)
{
    char peers[PN_MAX_PEERS * 32];
    int n = 0;
    uint8_t np = s_n_peers;
    for (uint8_t i = 0; i < np; i++) {
        n += snprintf(peers + n, sizeof(peers) - n, ",%02x:%02x:%02x:%02x:%02x:%02x:%lu:%d",
                      MAC2STR(s_peers[i].mac), (unsigned long)s_peers[i].csi, (int)s_peers[i].rssi);
    }
    printf("HELLO," MACSTR ",%d\n", MAC2STR(s_own_mac), PN_CHANNEL);
    if (np > 0) {
        printf("STAT," MACSTR "," MACSTR ",%d,%lu,%lu,%lu,%lu,%lu\n",
               MAC2STR(s_own_mac), MAC2STR(s_peers[0].mac), PN_CHANNEL,
               (unsigned long)s_tx_ok, (unsigned long)s_tx_fail,
               (unsigned long)s_beacons_rx, (unsigned long)s_csi_rx, (unsigned long)s_dropped);
    } else {
        printf("STAT," MACSTR ",none,%d,%lu,%lu,%lu,%lu,%lu\n",
               MAC2STR(s_own_mac), PN_CHANNEL,
               (unsigned long)s_tx_ok, (unsigned long)s_tx_fail,
               (unsigned long)s_beacons_rx, (unsigned long)s_csi_rx, (unsigned long)s_dropped);
    }
    printf("PEERS," MACSTR "%s\n", MAC2STR(s_own_mac), peers);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_queue = xQueueCreate(PN_QUEUE_LEN, sizeof(pn_item_t));
    configASSERT(s_queue != NULL);

    wifi_init();

    /* RuView edge pipeline, tier 1: presence + motion + vitals, no WASM / compression.
     * presence_thresh = 0 selects RuView's 60 s adaptive ambient calibration. */
    edge_config_t edge_cfg = {
        .tier = 1,
        .presence_thresh = 0.0f,
        .fall_thresh = 15.0f,
        .vital_window = 256,
        .vital_interval_ms = 250,
        .top_k_count = EDGE_TOP_K,
        .power_duty = 100,
    };
    ESP_ERROR_CHECK(edge_processing_init(&edge_cfg));

    espnow_init();
    csi_init();

    xTaskCreate(print_task, "pn_print", 4096, NULL, 3, NULL);
    xTaskCreate(tx_task, "espnow_tx", 3072, NULL, 6, NULL);

    ESP_LOGI(TAG, "presence node up: mac=" MACSTR " channel=%d tx=%d Hz, beacon v%d",
             MAC2STR(s_own_mac), PN_CHANNEL, PN_TX_HZ, PN_BEACON_VERSION);

    while (1) {
        print_status();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
