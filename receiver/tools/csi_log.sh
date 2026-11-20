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
#   DELAY=10 ./csi_log.sh ...                  # 시작 전 대기 시간 변경 (기본 3초, 자세 잡거나 나갈 시간)
#   LABEL=1 ./csi_log.sh ...                   # 호흡 정답 레이블 같이 기록 (아래 설명)
# 파일은 항상 tools 폴더에 csi_<날짜_시각>.csv 로 저장됨. 중간에 Ctrl+C 하면 그때까지 받은 것만 변환함.
#
# CSV 컬럼: seq,lost,ts_us,rssi,first_word_invalid,len,csi   (csi = int8 값들을 공백으로 구분한 한 칸)
# 포트 찾기: ls /dev/cu.*   (리눅스는 ls /dev/ttyUSB*)
# idf.py monitor 가 같은 포트를 열고 있으면 안 됨.
#
# 호흡 레이블 (LABEL=1):
#   측정하는 동안 이 터미널에서  ← 를 누르고 있으면 들숨,  → 를 누르고 있으면 날숨,  아무것도 안 누르면 무호흡.
#   키를 누르고 있으면 키 반복 입력이 계속 들어오는 걸로 "누르는 중"을 판단함 (터미널은 키 떼는 순간을 못 받음).
#   CSV 맨 뒤에 host_time(PC 시각, 초), label(in / ex / none) 컬럼이 추가됨. 파일은 CSV 하나만 생김.

PORT="$1"
DUR="${2:-10}"
# 어디서 실행하든 이 스크립트가 있는 tools 폴더에 저장
BASE="$(cd "$(dirname "$0")" && pwd)/csi_$(date +%Y%m%d_%H%M%S)"
OUT="$BASE.csv"
BAUD="${BAUD:-921600}"
DELAY="${DELAY:-3}"                 # 실행 후 측정 시작까지 기다리는 시간(초)
LABEL="${LABEL:-0}"
GAP="${GAP:-0.7}"                   # 같은 키 입력 사이가 이보다 벌어지면 "뗐다"로 봄 (macOS 키 반복 시작 지연보다 길어야 함)

if [ -z "$PORT" ]; then
    echo "usage: $0 <serial-port> [seconds]" >&2
    exit 1
fi
if [ ! -e "$PORT" ]; then
    echo "[csi_log] 포트가 없음: $PORT   (ls /dev/cu.* 로 확인)" >&2
    exit 1
fi

now() { perl -MTime::HiRes=time -e 'printf "%.3f\n", time'; }

# 0) 시작 전 대기: 실행하고 자세 잡거나 방에서 나갈 시간 (이 동안은 아무것도 안 받음, Ctrl+C로 취소 가능)
i="$DELAY"
while [ "$i" -gt 0 ]; do
    printf '\r[csi_log] %d초 뒤 측정 시작...  ' "$i" >&2
    sleep 1
    i=$((i - 1))
done
printf '\r[csi_log] 측정 시작              \a\n' >&2   # \a: 터미널 알림음

RAW="$(mktemp "${TMPDIR:-/tmp}/csi_raw.XXXXXX")" || exit 1
KEYS="$(mktemp "${TMPDIR:-/tmp}/csi_keys.XXXXXX")" || exit 1   # 키 입력 기록 (변환 끝나면 지움)
READER=""
TTYSAVE=""
cleanup() {
    [ -n "$READER" ] && kill "$READER" 2>/dev/null
    [ -n "$TTYSAVE" ] && stty "$TTYSAVE" < /dev/tty 2>/dev/null   # 키 입력 모드 원래대로
    rm -f "$RAW" "$KEYS"
}
trap cleanup EXIT
trap ':' INT                                   # Ctrl+C는 대기만 끊고 아래 변환은 계속 진행

# 1) 받기: 포트를 연 채로 보드레이트 설정 후 원본을 그대로 파일에 씀 (다른 일은 안 함)
( stty "$BAUD" raw -echo || exit 1; exec cat ) < "$PORT" > "$RAW" &
READER=$!
T0="$(now)"                                    # 받기 시작한 PC 시각 (레이블 시간 맞추는 기준)

echo "[csi_log] $PORT @ $BAUD, ${DUR}s 저장 ..." >&2
if [ "$LABEL" = 1 ]; then
    echo "[csi_log] ← 누르고 있기 = 들숨,  → 누르고 있기 = 날숨,  안 누르기 = 무호흡" >&2
    TTYSAVE="$(stty -g < /dev/tty)"
    stty -icanon -echo min 0 time 1 < /dev/tty   # 엔터 없이 키 하나씩 바로 받기, 화면에 안 찍기
    # 키 입력 기록: 화살표가 들어올 때마다 "PC시각,in|ex" 한 줄. DUR초 지나면 끝
    perl -MTime::HiRes=time -e '
        my ($dur, $out) = @ARGV; my $end = time + $dur;
        open(my $tty, "<", "/dev/tty") or die; open(my $o, ">", $out) or die;
        select((select($o), $| = 1)[0]); $| = 1;
        print $o "host_time,key\n";
        my ($buf, $last, $lt) = ("", "", 0);
        while ((my $t = time) < $end) {
            my $rin = ""; vec($rin, fileno($tty), 1) = 1;
            if (select($rin, undef, undef, 0.05) > 0) {
                sysread($tty, my $c, 64) or next; $buf .= $c;
                while ($buf =~ s/\e[\[O]([CD])//) {          # → = ESC[C, ← = ESC[D
                    my $k = $1 eq "D" ? "in" : "ex";
                    printf $o "%.3f,%s\n", time, $k; ($last, $lt) = ($k, time);
                }
                $buf = "" if length($buf) > 16;              # 다른 키는 무시
            }
            my $st = ($last ne "" && time - $lt < 0.7) ? ($last eq "in" ? "들숨 <<" : "날숨 >>") : "무호흡  ";
            printf STDERR "\r[csi_log] %4.0fs 남음   %s   ", $end - time, $st;
        }
        print STDERR "\n";
    ' "$DUR" "$KEYS"
    stty "$TTYSAVE" < /dev/tty; TTYSAVE=""
else
    sleep "$DUR"
fi
kill "$READER" 2>/dev/null
wait "$READER" 2>/dev/null
READER=""
printf '\a' >&2                                # 측정 끝 알림음
echo "[csi_log] 받은 원본 $(wc -c < "$RAW" | tr -d ' ') bytes, CSV 변환 중" >&2

# 2) 변환: 원본 파일을 한 번에 읽어서 CSV로
# LC_ALL=C: 포트 열 때 들어오는 깨진 바이트(한글 로케일에서 awk가 에러 냄)를 그냥 바이트로 취급
# LABEL=1이면 키 기록 파일을 먼저 읽어서 들숨/날숨 구간을 만들고, CSI 줄마다 그 시각의 레이블을 붙임
if [ "$LABEL" = 1 ]; then KEYIN="$KEYS"; else KEYIN=/dev/null; fi
LC_ALL=C awk -v out="$OUT" -v label="$LABEL" -v t0="$T0" -v gap="$GAP" -v kf="$KEYIN" '
    FILENAME == kf {                                    # 첫 번째 파일 = 키 기록 (LABEL=0이면 /dev/null이라 비어 있음)
        if (FNR > 1) { split($0, e, ","); ne++; et[ne] = e[1] + 0; ek[ne] = e[2] }
        next
    }
    !built {                                            # CSI 읽기 전에 한 번: 키 기록 → 구간
        built = 1
        rep = 0; nr = 0                                 # 키 반복 간격 추정 (0.2초 미만 간격들의 평균)
        for (i = 2; i <= ne; i++) if (ek[i] == ek[i-1] && et[i] - et[i-1] < 0.2) { rep += et[i] - et[i-1]; nr++ }
        rep = nr ? rep / nr : 0.09
        for (i = 1; i <= ne; i++) {
            if (i == 1 || ek[i] != ek[i-1] || et[i] - et[i-1] > gap) { ns++; ss[ns] = et[i]; sk[ns] = ek[i] }
            se[ns] = et[i] + rep                        # 구간 끝 = 마지막 키 입력 + 반복 간격
        }
        p = 1
        hdr = "seq,lost,ts_us,rssi,first_word_invalid,len,csi"
        print (label == 1 ? hdr ",host_time,label" : hdr) > out
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
        if (label == 1) {
            if (ts0 == "") ts0 = h[3]
            ht = t0 + (h[3] - ts0) / 1000000            # 이 CSI의 PC 시각 = 받기 시작 시각 + receiver 시간 경과
            while (p <= ns && se[p] < ht) p++
            lb = (p <= ns && ss[p] <= ht) ? sk[p] : "none"
            cnt[lb]++
            print head "\"" body "\"," sprintf("%.3f", ht) "," lb > out
        } else
            print head "\"" body "\"" > out
        n++
        next
    }
    { print }                                           # 부팅 로그 등은 화면에만
    END {
        if (!built) print "seq,lost,ts_us,rssi,first_word_invalid,len,csi" > out   # CSI가 한 줄도 없을 때
        printf "[csi_log] done: %d rows (%d broken skipped) -> %s\n", n + 0, bad + 0, out > "/dev/stderr"
        if (label == 1) printf "[csi_log] label: in %d, ex %d, none %d rows (키 입력 %d개, 구간 %d개) -> %s\n", cnt["in"], cnt["ex"], cnt["none"], ne, ns, out > "/dev/stderr"
    }
' "$KEYIN" - < "$RAW"
