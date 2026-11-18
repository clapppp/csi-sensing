#!/bin/sh
# receiver ESP32 시리얼을 정해진 시간(기본 10초) 동안 받아서 CSI만 CSV로 저장하고 자동 종료.
#
# 받는 동안에는 cat이 시리얼 원본을 임시 파일에 그대로 쓰기만 하고, 끝난 뒤에 한 번에 CSV로 변환함.
# (읽으면서 변환하면 변환이 밀릴 때 cat도 같이 멈춰서 시리얼 데이터가 버려지기 때문)
#
# 사용법:
#   ./csi_log.sh /dev/cu.usbserial-XXXX        # 10초 저장
#   ./csi_log.sh /dev/cu.usbserial-XXXX 30     # 30초 저장
#   BAUD=115200 ./csi_log.sh ...               # 보드레이트 변경 (기본 921600)
# 파일은 항상 tools 폴더에 csi_<날짜_시각>.csv 로 저장됨. 중간에 Ctrl+C 하면 그때까지 받은 것만 변환함.
#
# CSV 컬럼: seq,lost,ts_us,rssi,first_word_invalid,len,csi   (csi = int8 값들을 공백으로 구분한 한 칸)
# 포트 찾기: ls /dev/cu.*   (리눅스는 ls /dev/ttyUSB*)
# idf.py monitor 가 같은 포트를 열고 있으면 안 됨.

PORT="$1"
DUR="${2:-10}"
# 어디서 실행하든 이 스크립트가 있는 tools 폴더에 저장
OUT="$(cd "$(dirname "$0")" && pwd)/csi_$(date +%Y%m%d_%H%M%S).csv"
BAUD="${BAUD:-921600}"

if [ -z "$PORT" ]; then
    echo "usage: $0 <serial-port> [seconds]" >&2
    exit 1
fi
if [ ! -e "$PORT" ]; then
    echo "[csi_log] 포트가 없음: $PORT   (ls /dev/cu.* 로 확인)" >&2
    exit 1
fi

RAW="$(mktemp "${TMPDIR:-/tmp}/csi_raw.XXXXXX")" || exit 1
READER=""
cleanup() { [ -n "$READER" ] && kill "$READER" 2>/dev/null; rm -f "$RAW"; }
trap cleanup EXIT
trap ':' INT                                   # Ctrl+C는 sleep만 끊고 아래 변환은 계속 진행

# 1) 받기: 포트를 연 채로 보드레이트 설정 후 원본을 그대로 파일에 씀 (다른 일은 안 함)
( stty "$BAUD" raw -echo || exit 1; exec cat ) < "$PORT" > "$RAW" &
READER=$!

echo "[csi_log] $PORT @ $BAUD, ${DUR}s 저장 ..." >&2
sleep "$DUR"
kill "$READER" 2>/dev/null
wait "$READER" 2>/dev/null
READER=""
echo "[csi_log] 받은 원본 $(wc -c < "$RAW" | tr -d ' ') bytes, CSV 변환 중" >&2

# 2) 변환: 원본 파일을 한 번에 읽어서 CSV로
# LC_ALL=C: 포트 열 때 들어오는 깨진 바이트(한글 로케일에서 awk가 에러 냄)를 그냥 바이트로 취급
LC_ALL=C awk -v out="$OUT" '
    BEGIN {
        print "seq,lost,ts_us,rssi,first_word_invalid,len,csi" > out
    }
    { sub(/\r$/, "") }
    /^CSI,/ {
        i = index($0, "[")
        if (i == 0 || $0 !~ /\]$/) { bad++; next }      # 잘린 줄
        head = substr($0, 5, i - 5)                     # "seq,lost,ts,rssi,fwi,len,"
        body = substr($0, i + 1); sub(/ *\]$/, "", body)   # [ ] 사이 "v0 v1 v2 ..."
        nf = split(head, h, ",")
        if (nf < 6 || split(body, v, " ") != h[6] + 0) { bad++; next }   # 값 개수 불일치
        for (j = 1; j <= h[6]; j++)                     # 숫자 아닌 값(예: 글자 빠져서 "-"만 남음) 섞이면 버림
            if (v[j] !~ /^-?[0-9]+$/) { bad++; next }
        print head "\"" body "\"" > out
        n++
        next
    }
    { print }                                           # 부팅 로그 등은 화면에만
    END { printf "[csi_log] done: %d rows (%d broken skipped) -> %s\n", n + 0, bad + 0, out > "/dev/stderr" }
' < "$RAW"
