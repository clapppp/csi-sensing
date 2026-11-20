#include <stdio.h>
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
#include "nvs_flash.h"

#define CSI_CHANNEL 11           // sender와 같게
// CSI 스케일 설정
// CSI_MANU_SCALE 0: 자동 스케일 (패킷마다 스케일이 바뀌어서 진폭이 같이 흔들림)
#define CSI_MANU_SCALE 0
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

/* ===== 출력 태스크: 큐에서 꺼내 시리얼로 출력 ===== */
static void print_task(void *arg)
{
    csi_pkt_t p;
    while (1) {
        if (xQueueReceive(csi_q, &p, portMAX_DELAY)) {   // csi_cb가 큐에 넣을 떄까지 기다림
            printf("CSI,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%d,%u,%u,[",
                   last_seq, lost, p.ts, p.rssi, p.fwi, p.len);
            for (int i = 0; i < p.len; i++) printf("%d ", p.buf[i]);
            printf("]\n");
        }
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
    // 받을때 csi 설정. sender와 같게 맞춰야 함 + 연산 방법들
    wifi_csi_config_t c = {
        .lltf_en           = true,   // L-LTF로 CSI 계산 (64 서브캐리어)
        .htltf_en          = true,   // HT-LTF로도 계산 (11n 패킷일 때)
        .stbc_htltf2_en    = false,  // STBC 안 씀
        .ltf_merge_en      = false,  // L-LTF와 HT-LTF를 평균 내지 않고 따로 받음
        .channel_filter_en = false,  // 인접 서브캐리어 평활화 끔 → 원본값
        .manu_scale        = CSI_MANU_SCALE,  // 1이면 스케일 고정, 0이면 자동
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&c));
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

    ESP_LOGI(TAG, "receiver ready, ch %d, csi manu_scale %d, free heap %u",
             CSI_CHANNEL, CSI_MANU_SCALE, (unsigned)esp_get_free_heap_size());
    // app_main은 여기서 return해도 됨. 콜백과 print_task가 계속 동작함
}