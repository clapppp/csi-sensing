/* ===== 헤더: 쓸 함수/타입 선언을 가져옴 ===== */
#include <stdio.h>               // printf 등 표준 입출력
#include <string.h>              // memcpy 등 메모리 함수
#include "freertos/FreeRTOS.h"   // FreeRTOS 기본 타입/매크로 (TickType_t, pdMS_TO_TICKS)
#include "freertos/task.h"       // 태스크 함수 (vTaskDelayUntil, xTaskGetTickCount)
#include "esp_wifi.h"            // WiFi 드라이버 API
#include "esp_now.h"             // ESP-NOW API
#include "esp_event.h"           // 이벤트 루프 API
#include "esp_netif.h"           // 네트워크 인터페이스(TCP/IP 스택) API
#include "esp_mac.h"             // MAC 주소 출력용 매크로 (MACSTR, MAC2STR)
#include "esp_log.h"             // ESP_LOGI/ESP_LOGW 로그 함수
#include "nvs_flash.h"           // NVS(플래시 key-value 저장소) API

/* ===== 설정값 ===== */
#define CSI_CHANNEL   11         // 사용할 WiFi 채널. receiver도 같은 값이어야 패킷을 받음
#define SEND_HZ       100        // 1초에 보낼 패킷 수 (= CSI 샘플링 주파수)

static const char *TAG = "sender";   // 로그 앞에 붙는 태그 문자열
static const uint8_t BCAST[ESP_NOW_ETH_ALEN] =   // ESP_NOW_ETH_ALEN = 6 (MAC 길이)
    {0xff,0xff,0xff,0xff,0xff,0xff};             // 브로드캐스트 주소: 모든 기기가 수신 대상

/* ===== WiFi 초기화 ===== */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());              // TCP/IP 스택(lwIP) 초기화
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // 기본 이벤트 루프 생성. WiFi 드라이버가 상태 변화를 여기로 알림

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // WiFi 드라이버 기본 설정값(버퍼 개수 등)을 담은 구조체
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));           // 그 설정으로 드라이버 초기화 (메모리 할당, 내부 태스크 생성)
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // WiFi 설정을 플래시가 아니라 RAM에만 저장 (재부팅 시 매번 새로 설정)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));       // STA(클라이언트) 모드. 공유기에 연결은 안 하고 무선부를 켜는 용도
    ESP_ERROR_CHECK(esp_wifi_start());              // 무선부(RF) 실제로 켜기

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // 절전 모드 끔. 켜져 있으면 RF가 주기적으로 잠들어 송신 간격이 흔들림
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20)); // 채널 폭 20MHz로 고정
    ESP_ERROR_CHECK(esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_NONE)); // 채널 11로 고정. 보조 채널 없음(40MHz 안 씀). start 뒤에 호출해야 함
}

/* ===== ESP-NOW 초기화 ===== */
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());                // ESP-NOW 프로토콜 초기화. WiFi가 start된 상태여야 함
    esp_now_peer_info_t peer = {                    // 송신 대상(peer) 정보 구조체
        .channel = CSI_CHANNEL,                     // 이 peer와 통신할 채널
        .ifidx   = WIFI_IF_STA,                     // 어느 인터페이스로 보낼지 (STA)
        .encrypt = false,                           // 암호화 안 함 (브로드캐스트는 암호화 불가)
    };
    memcpy(peer.peer_addr, BCAST, ESP_NOW_ETH_ALEN); // peer 주소 칸에 브로드캐스트 MAC 6바이트 복사
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));       // peer 목록에 등록. 등록 안 된 주소로는 send가 실패함
    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_HT20,     // 11n, 20MHz → 프리앰블에 L-LTF + HT-LTF가 들어감
        .rate    = WIFI_PHY_RATE_MCS0_LGI, // 가장 느리고 견고한 11n 속도
        .ersu    = false,
        .dcm     = false,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(BCAST, &rate));
}

/* ===== 진입점 ===== */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();               // NVS 초기화. WiFi 드라이버가 RF 보정값을 여기 저장함
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||         // NVS 공간이 꽉 찼거나
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {     // 이전 펌웨어가 다른 NVS 포맷으로 써 놨으면
        ESP_ERROR_CHECK(nvs_flash_erase());         // NVS 영역을 지우고
        ret = nvs_flash_init();                     // 다시 초기화
    }
    ESP_ERROR_CHECK(ret);                           // 그래도 실패면 여기서 멈춤

    wifi_init();                                    // 위에서 정의한 WiFi 초기화
    espnow_init();                                  // 위에서 정의한 ESP-NOW 초기화

    uint8_t mac[6];                                 // MAC 주소 받을 6바이트 배열
    esp_wifi_get_mac(WIFI_IF_STA, mac);             // STA 인터페이스의 MAC을 mac 배열에 채움
    ESP_LOGI(TAG, "sender MAC " MACSTR ", ch %d, %d Hz", // MACSTR = "%02x:%02x:..." 형식 문자열
             MAC2STR(mac), CSI_CHANNEL, SEND_HZ);       // MAC2STR = 배열을 6개 인자로 펼쳐 줌

    uint32_t seq = 0;                               // 시퀀스 번호 (receiver가 손실 확인용)
    TickType_t last = xTaskGetTickCount();          // 현재 시각(틱 단위). 주기 계산의 기준점
    const TickType_t period = pdMS_TO_TICKS(1000 / SEND_HZ); // 10ms를 틱 수로 변환 (기본 100Hz 틱이면 1틱)

    while (1) {                                     // 무한 루프: app_main이 끝나지 않게 함
        esp_err_t err = esp_now_send(BCAST,         // 목적지: 브로드캐스트
                                     (uint8_t *)&seq, // 보낼 데이터의 시작 주소 (seq 변수를 바이트 배열로 취급)
                                     sizeof(seq));  // 보낼 길이: 4바이트
        if (err != ESP_OK)                          // 송신 큐에 못 넣었으면
            ESP_LOGW(TAG, "send fail: %s", esp_err_to_name(err)); // 경고 로그 (에러 코드를 문자열로)
        seq++;                                      // 다음 번호
        vTaskDelayUntil(&last, period);             // last + period 시각까지 대기하고 last를 갱신
    }
}