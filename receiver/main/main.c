#include <stdio.h>
#include <stdlib.h>              // abs
#include <string.h>
#include <inttypes.h>            // PRIu32: uint32_t를 printf로 찍을 때 쓰는 형식 매크로
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

#define CSI_CHANNEL 11           // sender와 같게
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
static const uint8_t SENDER_MAC[6] = {0xc8, 0x85, 0x41, 0xe2, 0x98, 0xf0};
static const char *TAG = "recv";

/* 콜백에서 출력 태스크로 넘길 데이터 한 건 */
typedef struct {
    uint32_t ts;                 // 수신 시각 (부팅 후 마이크로초)
    int8_t   rssi;               // 수신 신호 세기 (dBm)
    uint8_t  fwi;                // first_word_invalid: 1이면 CSI 첫 4바이트가 하드웨어 한계로 무효
    uint16_t len;                // CSI 바이트 수
    int8_t   buf[384];           // CSI 원본 (ESP32 최대 길이보다 넉넉하게)
} csi_pkt_t;

static QueueHandle_t csi_q;
static volatile uint32_t last_seq = 0, lost = 0;  // volatile: 콜백과 다른 태스크가 같이 접근
static volatile bool got_first = false;

/* ===== CSI 콜백: WiFi 드라이버 태스크에서 호출됨 → 짧게 끝내야 함 ===== */
static void csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;
    if (memcmp(info->mac, SENDER_MAC, 6) != 0) return;   // 우리 sender 패킷만 통과

    csi_pkt_t p;
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

/* ===== 출력 태스크: 큐에서 꺼내 시리얼로 출력 (처음 CALIB_S초는 shift 보정만) ===== */
static void print_task(void *arg)
{
    csi_pkt_t p;
    uint32_t hist[129] = {0}, n = 0;                     // |CSI 숫자| 0~128 개수
    int64_t t0 = esp_timer_get_time(), next = t0 + 1000000;
    bool calibrating = CSI_MANU_SCALE;                    // 자동 스케일이면 보정 안 함
    int p99 = 0;
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
        printf("CSI,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%d,%u,%u,[",
               last_seq, lost, p.ts, p.rssi, p.fwi, p.len);
        for (int i = 0; i < p.len; i++) printf("%d ", p.buf[i]);
        printf("]\n");
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_NONE));
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
    xTaskCreate(print_task, "print", 4096, NULL, 5, NULL); // 스택 4KB, 우선순위 5, 스레드 만들기와 같음

    wifi_init();
    ESP_ERROR_CHECK(esp_now_init()); // 받기만 하니 송신쪽에선 패킷 설정이 있었지만 여기선 기본설정으로
    //두가지 콜백 등록
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    csi_init();

    ESP_LOGI(TAG, "receiver ready, ch %d, csi manu_scale %d, calibrating shift for %ds from %d, free heap %u",
             CSI_CHANNEL, CSI_MANU_SCALE, CSI_MANU_SCALE ? CALIB_S : 0, CSI_SHIFT, (unsigned)esp_get_free_heap_size());
    // app_main은 여기서 return해도 됨. 콜백과 print_task가 계속 동작함
}