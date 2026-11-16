#!/bin/sh
# receiver ESP32 시리얼을 정해진 시간(기본 10초) 동안 한 줄씩 읽어서 CSI만 CSV로 저장하고 자동 종료.
#
# 사용법:
#   ./csi_log.sh /dev/cu.usbserial-XXXX                # 10초, tools/csi_<날짜_시각>.csv
#   ./csi_log.sh /dev/cu.usbserial-XXXX run1.csv       # tools/run1.csv 로 저장
#   ./csi_log.sh /dev/cu.usbserial-XXXX ~/data/a.csv   # 경로를 주면 그 위치에 저장
#   ./csi_log.sh /dev/cu.usbserial-XXXX run1.csv 30    # 30초 기록
#   BAUD=115200 ./csi_log.sh ...                       # 보드레이트 변경 (기본 921600)
#
# CSV 컬럼: seq,lost,ts_us,rssi,len,csi   (csi = int8 값들을 공백으로 구분한 한 칸)
# 포트 찾기: ls /dev/cu.*   (리눅스는 ls /dev/ttyUSB*)
# idf.py monitor 가 같은 포트를 열고 있으면 안 됨.

PORT="$1"
OUT="${2:-csi_$(date +%Y%m%d_%H%M%S).csv}"
# 파일 이름만 주면(경로에 / 없음) 어디서 실행하든 이 스크립트가 있는 tools 폴더에 저장
case "$OUT" in
    */*) ;;
    *)   OUT="$(cd "$(dirname "$0")" && pwd)/$OUT" ;;
esac
DUR="${3:-10}"
BAUD="${BAUD:-921600}"

if [ -z "$PORT" ]; then
    echo "usage: $0 <serial-port> [output.csv] [seconds]" >&2
    exit 1
fi

FIFO="$(mktemp -u "${TMPDIR:-/tmp}/csi_log.XXXXXX")"
mkfifo "$FIFO" || exit 1

# 1) 시리얼 리더: 포트를 연 채로 보드레이트 설정 후 cat → FIFO
( stty "$BAUD" raw -echo || exit 1; exec cat ) < "$PORT" > "$FIFO" &
READER=$!

# 2) 타이머: DUR초 뒤 리더를 죽이면 awk가 EOF를 받고 깔끔하게 끝남
( sleep "$DUR"; kill "$READER" 2>/dev/null ) &
TIMER=$!

cleanup() { kill "$READER" "$TIMER" 2>/dev/null; rm -f "$FIFO"; }
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM          # 중간에 Ctrl+C 해도 그때까지 저장된 건 남음

echo "[csi_log] $PORT @ $BAUD, ${DUR}s -> $OUT" >&2

# 3) 한 줄씩 읽어 CSV 변환 (줄마다 flush → 실시간 기록)
awk -v out="$OUT" '
    BEGIN { print "seq,lost,ts_us,rssi,len,csi" > out; fflush(out) }
    { sub(/\r$/, "") }
    /^CSI,/ {
        i = index($0, "[")
        if (i == 0 || $0 !~ /\]$/) { bad++; next }      # 잘린 줄
        head = substr($0, 5, i - 5)                     # "seq,lost,ts,rssi,len,"
        body = substr($0, i + 1); sub(/ *\]$/, "", body)
        nf = split(head, h, ",")
        if (nf < 5 || split(body, v, " ") != h[5] + 0) { bad++; next }   # 값 개수 불일치
        print head "\"" body "\"" > out; fflush(out)
        if (++n % 50 == 0) printf "\r[csi_log] %d rows", n > "/dev/stderr"
        next
    }
    { print }                                           # 부팅 로그 등은 화면에만
    END { printf "\n[csi_log] done: %d rows, %d broken skipped -> %s\n", n + 0, bad + 0, out > "/dev/stderr" }
' < "$FIFO"
