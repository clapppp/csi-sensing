#include <stdio.h>
#include <stdlib.h>              // abs
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"      // 콜백 → 출력 태스크로 데이터 넘길 큐
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"           // esp_timer_get_time: 부팅 후 마이크로초
#include "nvs_flash.h"
#include "lwip/sockets.h"        // UDP 소켓 (socket, sendto)

#define CSI_CHANNEL 11           // sender와 같게. 수신기가 이 채널로 AP를 엶
// PC 연결: 수신기가 AP를 열고, 접속한 PC로 UDP 전송
#define AP_SSID     "csi-recv"
#define AP_PASS     "csisensing"   // WPA2라 8자 이상
#define PC_PORT     5005           // PC가 받을 UDP 포트
#define BATCH       5              // CSI 몇 건을 UDP 한 번에 묶을지 (5 x 272B = 1360B, 한 패킷 최대 1472B 안)
// CSI 스케일 설정
// CSI_MANU_SCALE 0: 자동 스케일 (패킷마다 스케일이 바뀌어서 진폭이 같이 흔들림)
// CSI_MANU_SCALE 1: shift 값으로 스케일 고정 (0~15, 1 클수록 값이 2배. 너무 크면 ±127에서 잘림)
#define CSI_MANU_SCALE 1
#define CSI_SHIFT      2         // 보정 시작값
// 시작 후 CALIB_S초 동안은 CSI를 출력하지 않고 shift만 맞춘 뒤 고정 (측정 중에는 안 바꿈)
// 1초마다 CSI 숫자 크기의 상위 1%를 보고: 잘림이 있거나 CALIB_HI 초과면 shift -1, CALIB_LO 미만이면 +1
#define CALIB_S        5
#define CALIB_LO       50
#define CALIB_HI       100
// sender 로그에 찍힌 MAC으로 바꿔 넣기
static const uint8_t SENDER_MAC[6] = {0x20, 0x9b, 0xa9, 0x92, 0xf4, 0x3c};
static const char *TAG = "recv";

/* 콜백에서 전송 태스크로 넘길 데이터 한 건 */
typedef struct {
    uint32_t seq;                // CSI 받은 순간의 last_seq (큐에서 밀려도 이 CSI와 짝이 안 어긋나게 여기서 담음)
    uint32_t lost;               // CSI 받은 순간의 누적 손실 수
    uint32_t ts;                 // 수신 시각 (부팅 후 마이크로초)
    int8_t   rssi;               // 수신 신호 세기 (dBm)
    uint8_t  fwi;                // first_word_invalid: 1이면 CSI 첫 4바이트가 하드웨어 한계로 무효
    uint16_t len;                // CSI 바이트 수
    int8_t   buf[384];           // CSI 원본 (ESP32 최대 길이보다 넉넉하게)
} csi_pkt_t;

/* UDP로 보내는 CSI 한 건 (272바이트, 리틀엔디안). 예전 시리얼 한 줄과 같은 항목 */
typedef struct __attribute__((packed)) {
    uint32_t seq;                // 마지막으로 받은 ESP-NOW seq
    uint32_t lost;               // 누적 손실 수
    uint32_t ts;                 // 수신 시각 (부팅 후 마이크로초)
    int8_t   rssi;
    uint8_t  fwi;
    uint16_t len;                // 실제 CSI 바이트 수 (보통 256)
    int8_t   csi[256];           // CSI 원본 (허수, 실수 순서 쌍)
} csi_rec_t;

static QueueHandle_t csi_q;
static volatile uint32_t last_seq = 0, lost = 0;  // volatile: 콜백과 다른 태스크가 같이 접근
static volatile bool got_first = false;
static volatile uint32_t pc_ip = 0;               // 접속한 PC의 IP (0이면 아직 없음 → 안 보냄)

/* ===== PC가 AP에 붙어 IP를 받으면 기억, 끊기면 지움 ===== */
static void ap_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        pc_ip = ((ip_event_ap_staipassigned_t *)data)->ip.addr;
        ESP_LOGI(TAG, "PC connected: " IPSTR ", sending UDP to port %d", IP2STR(&((ip_event_ap_staipassigned_t *)data)->ip), PC_PORT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        pc_ip = 0;
        ESP_LOGI(TAG, "PC disconnected");
    }
}

/* ===== CSI 콜백: WiFi 드라이버 태스크에서 호출됨 → 짧게 끝내야 함 ===== */
static void csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;
    if (memcmp(info->mac, SENDER_MAC, 6) != 0) return;   // 우리 sender 패킷만 통과

    csi_pkt_t p;
    p.seq  = last_seq;
    p.lost = lost;
    p.ts   = info->rx_ctrl.timestamp;
    p.rssi = info->rx_ctrl.rssi;
    p.fwi  = info->first_word_invalid;
    p.len  = info->len > sizeof(p.buf) ? sizeof(p.buf) : info->len;
    memcpy(p.buf, info->buf, p.len);
    xQueueSend(csi_q, &p, 0);    // 큐가 꽉 차면 기다리지 않고 버림, 여기서 써주면 print_task 에서 큐에서 꺼내 출력
}

/* ===== ESP-NOW 수신 콜백: payload의 seq로 손실 계산 ===== */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < 4 || memcmp(info->src_addr, SENDER_MAC, 6) != 0) return;
    uint32_t seq;
    memcpy(&seq, data, 4);       // sender가 보낸 4바이트를 다시 uint32_t로
    if (got_first && seq > last_seq + 1) lost += seq - last_seq - 1;
    last_seq = seq;
    got_first = true;
}

/* 받을때 csi 설정. sender와 같게 맞춰야 함 + 연산 방법들. 보정 중에 shift만 바꿔 다시 적용 */
static wifi_csi_config_t csi_cfg = {
    .lltf_en           = true,   // L-LTF로 CSI 계산 (64 서브캐리어)
    .htltf_en          = true,   // HT-LTF로도 계산 (11n 패킷일 때)
    .stbc_htltf2_en    = false,  // STBC 안 씀
    .ltf_merge_en      = false,  // L-LTF와 HT-LTF를 평균 내지 않고 따로 받음
    .channel_filter_en = false,  // 인접 서브캐리어 평활화 끔 → 원본값
    .manu_scale        = CSI_MANU_SCALE,  // 1이면 스케일 고정, 0이면 자동
    .shift             = CSI_SHIFT,       // 고정일 때 쓸 스케일 (manu_scale 0이면 무시됨)
};

/* 1초 동안 모은 |CSI 숫자| 분포로 shift를 한 단계 조정. 상위 1% 값을 돌려줌 */
static int calib_step(uint32_t hist[129], uint32_t n)
{
    uint32_t acc = 0;
    int p99 = 0;
    for (int v = 128; v >= 0; v--) {                     // 큰 값부터 세서 상위 1% 위치 찾기
        acc += hist[v];
        if (acc * 100 >= n) { p99 = v; break; }
    }
    bool clipped = hist[127] + hist[128] > n / 1000;      // 0.1% 넘게 ±127에 걸리면 잘림으로 봄
    int s = csi_cfg.shift;
    if ((clipped || p99 > CALIB_HI) && s > 0) s--;
    else if (p99 < CALIB_LO && s < 15) s++;
    if (s != csi_cfg.shift) {
        csi_cfg.shift = s;
        ESP_ERROR_CHECK(esp_wifi_set_csi(false));         // 끄고 설정 바꾼 뒤 다시 켬
        ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
        ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    }
    return p99;
}

/* ===== 전송 태스크: 큐에서 꺼내 BATCH건씩 PC로 UDP 전송 (처음 CALIB_S초는 shift 보정만) ===== */
static void print_task(void *arg)
{
    csi_pkt_t p;
    uint32_t hist[129] = {0}, n = 0;                     // |CSI 숫자| 0~128 개수
    int64_t t0 = esp_timer_get_time(), next = t0 + 1000000;
    bool calibrating = CSI_MANU_SCALE;                    // 자동 스케일이면 보정 안 함
    int p99 = 0;
    static csi_rec_t batch[BATCH];                        // 묶음 (static: 1.3KB라 태스크 스택 대신 전역 메모리)
    int nb = 0;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);   // UDP 소켓
    if (sock < 0) { ESP_LOGE(TAG, "socket failed"); abort(); }
    struct sockaddr_in dst = { .sin_family = AF_INET, .sin_port = htons(PC_PORT) };
    while (1) {
        if (!xQueueReceive(csi_q, &p, portMAX_DELAY)) continue;   // csi_cb가 큐에 넣을 떄까지 기다림
        if (calibrating) {
            for (int i = 4; i < p.len; i++) hist[abs(p.buf[i])]++;  // 첫 4바이트(무효일 수 있음)는 뺌
            n += p.len > 4 ? p.len - 4 : 0;
            int64_t now = esp_timer_get_time();
            if (now < next) continue;
            if (n) p99 = calib_step(hist, n);
            memset(hist, 0, sizeof(hist));
            n = 0;
            next += 1000000;
            if (now - t0 >= CALIB_S * 1000000LL) {
                calibrating = false;
                ESP_LOGI(TAG, "calib done: shift %d (last 1s top1%% |csi| %d)", csi_cfg.shift, p99);
            }
            continue;                                     // 보정 중에는 출력 안 함
        }
        // 한 건을 묶음에 쌓고, BATCH건 차면 방금 CSI를 받은 직후 바로 전송 → 송신기 다음 패킷(10ms 뒤) 전에 끝남
        csi_rec_t *r = &batch[nb++];
        r->seq = p.seq; r->lost = p.lost; r->ts = p.ts;
        r->rssi = p.rssi; r->fwi = p.fwi;
        r->len = p.len > sizeof(r->csi) ? sizeof(r->csi) : p.len;
        memset(r->csi, 0, sizeof(r->csi));
        memcpy(r->csi, p.buf, r->len);
        if (nb < BATCH) continue;
        nb = 0;
        if (!pc_ip) continue;                             // PC 아직 안 붙었으면 버림
        dst.sin_addr.s_addr = pc_ip;
        sendto(sock, batch, sizeof(batch), 0, (struct sockaddr *)&dst, sizeof(dst));   // 실패해도 다시 안 보냄 (놓치면 끝, PC가 seq로 확인)
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();                   // AP용 네트워크 (192.168.4.1, PC에 IP 나눠주는 DHCP 서버 포함)
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, ap_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, ap_event, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.csi_enable = 1;          // CSI 기능 켜기 (esp 설정의 CONFIG_ESP_WIFI_CSI_ENABLED 대신 코드에서). 안 켜지면 csi_init에서 멈춤
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));     // AP 모드: PC가 여기에 접속. AP는 절전이 없어서 set_ps 필요 없음
    wifi_config_t ap = {
        .ap = {
            .ssid = AP_SSID,
            .password = AP_PASS,
            .channel = CSI_CHANNEL,                       // sender와 같은 채널로 AP를 열어야 CSI도 계속 받음
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 1,                          // PC 한 대만
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20));   // 20MHz 고정 (sender와 같게)
}

static void csi_init(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));  // espnow 콜백과 비교해 거의 같은 순간에 호출됨.
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //큐 생성후 print task 생성
    csi_q = xQueueCreate(32, sizeof(csi_pkt_t));          // 최대 32건 버퍼링, 램 힙공간에 공간 확보해놓음
    if (!csi_q) { ESP_LOGE(TAG, "queue alloc failed"); abort(); }  // 메모리 부족하면 바로 알 수 있게
    wifi_init();                                          // 네트워크(lwIP)가 먼저 켜져야 태스크에서 소켓을 만들 수 있음
    xTaskCreate(print_task, "print", 4096, NULL, 5, NULL); // 스택 4KB, 우선순위 5, 스레드 만들기와 같음
    ESP_ERROR_CHECK(esp_now_init()); // 받기만 하니 송신쪽에선 패킷 설정이 있었지만 여기선 기본설정으로
    //두가지 콜백 등록
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    csi_init();

    ESP_LOGI(TAG, "receiver ready, ch %d, csi manu_scale %d, calibrating shift for %ds from %d, free heap %u",
             CSI_CHANNEL, CSI_MANU_SCALE, CSI_MANU_SCALE ? CALIB_S : 0, CSI_SHIFT, (unsigned)esp_get_free_heap_size());
    // app_main은 여기서 return해도 됨. 콜백과 print_task가 계속 동작함
}