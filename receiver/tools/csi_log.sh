#!/bin/sh
# ./csi_log.sh /dev/cu.usbserial-XXXX 30     30초 측정 (기본 10초), tools/csi_<날짜_시각>.csv 에 받는 즉시 씀
# LABEL=1 ./csi_log.sh ...                    ← 누르고 있기 = 들숨(in), → = 날숨(ex), 안 누르기 = 무호흡(none)
# BAUD=921600 DELAY=3 GAP=0.7                 보드레이트, 시작 전 대기(초), 같은 키 입력이 GAP초 넘게 끊기면 뗀 걸로 봄

OUT="$(cd "$(dirname "$0")" && pwd)/csi_$(date +%Y%m%d_%H%M%S).csv"
i="${DELAY:-3}"; while [ "$i" -gt 0 ]; do printf '\r%d초 뒤 시작 ' "$i" >&2; sleep 1; i=$((i - 1)); done   # 자세 잡을 시간 카운트다운
printf '\r측정 시작      \a\n' >&2                    # 시작 알림음
trap ':' INT                                           # Ctrl+C는 perl만 멈추고 아래 stty 복구까지 진행
[ "$LABEL" = 1 ] && S="$(stty -g)" && stty -icanon -echo min 0 time 1   # 엔터 없이 키 바로 받기, 화면에 안 찍기

# 포트를 연 채로 보드레이트 설정(stty) → 같은 연결을 perl이 이어받음(exec)
# perl은 시리얼(fd 0)과 키보드(/dev/tty)를 select로 같이 기다리다가 들어오는 대로 처리
( stty "${BAUD:-921600}" raw -echo; exec perl -MTime::HiRes=time -e '
    my ($dur, $out, $label, $gap) = @ARGV;
    my $stop; $SIG{INT} = sub { $stop = 1 };
    my $t0 = time;                                      # 받기 시작한 PC 시각 (측정 시간 재는 기준)
    open my $o, ">", $out; select((select($o), $| = 1)[0]);   # 줄마다 바로 디스크에
    print $o "seq,lost,ts_us,rssi,first_word_invalid,len,csi", ($label ? ",host_time,label" : ""), "\n";
    my $tty; open $tty, "<", "/dev/tty" if $label;

    my ($sb, $kb, $n, $ts0, $ta, @pend, @seg) = ("", "", 0);  # 시리얼 버퍼, 키 버퍼, 줄 수, ESP32 시각 기준, 첫 줄 도착 시각, 레이블 대기 줄, 키 구간 [키, 시작, 마지막 입력]
    my $rep = 0.09;                                     # 키 반복 간격 (실제 간격으로 계속 갱신)

    # 시각 t의 레이블: t를 포함하는 구간(시작 ~ 마지막 입력 + 반복 간격)의 키
    sub lab { my $t = shift; for (reverse @seg) { next if $_->[1] > $t; return $t <= $_->[2] + $rep ? $_->[0] : "none" } return "none" }
    # t + gap 이 지난 줄은 구간이 더 안 바뀌니 확정해서 씀 (인자 1이면 전부)
    sub flush { my $all = shift; while (@pend && ($all || time > $pend[0][0] + $gap + 0.1)) { my ($t, $r) = @{ shift @pend }; printf $o "%s,%.3f,%s\n", $r, $t, lab($t) } }

    while (!$stop && time < $t0 + $dur) {
        my $rin = ""; vec($rin, 0, 1) = 1; vec($rin, fileno $tty, 1) = 1 if $tty;
        select(my $rout = $rin, undef, undef, 0.05);

        if (vec($rout, 0, 1) && sysread STDIN, my $c, 65536) {
            $sb .= $c;
            while ($sb =~ s/^(.*?)\r?\n//) {
                my ($h, $b) = $1 =~ /^CSI,((?:-?\d+,){6})\[(.*?) *\]$/ or next;    # h = "seq,lost,ts,rssi,fwi,len," 숫자 6개 딱 (반쪽 줄이 앞에 붙은 건 버림)
                next unless $b =~ /^-?\d+( -?\d+)*$/ && split(/ /, $b) == (split /,/, $h)[5];   # 잘리거나 깨진 줄 버림
                $n++;
                if (!$label) { print $o "$h\"$b\"\n"; next }
                my $ts = (split /,/, $h)[2]; $ts0 //= $ts; $ta //= time;              # 기준점 = 첫 정상 줄의 ESP32 시각과 실제 도착 시각
                push @pend, [$ta + (($ts - $ts0) % 2**32) / 1e6, "$h\"$b\""];   # PC 시각 = 첫 줄 도착 + ESP32 경과 (uint32 한 바퀴 넘어가도 OK)
            }
        }

        if ($tty && vec($rout, fileno $tty, 1) && sysread $tty, my $c, 64) {
            $kb .= $c;
            while ($kb =~ s/\e[\[O]([CD])//) {                                     # ← = ESC[D, → = ESC[C
                my ($k, $t, $s) = ($1 eq "D" ? "in" : "ex", time, $seg[-1]);
                if ($s && $s->[0] eq $k && $t - $s->[2] <= $gap) {                  # 같은 키가 gap 안에 또 옴 = 계속 누르는 중
                    $rep = 0.9 * $rep + 0.1 * ($t - $s->[2]) if $t - $s->[2] < 0.2;
                    $s->[2] = $t;
                } else { push @seg, [$k, $t, $t] }                                  # 키 바뀜 or 끊겼다 다시 누름 = 새 구간
            }
            $kb = "" if length $kb > 16;                                            # 다른 키는 무시
        }

        flush(0) if $label;
        my $s = $seg[-1];
        printf STDERR "\r%4.0fs  %d줄  %s   ", $t0 + $dur - time, $n, !$label ? "" : $s && time - $s->[2] < $gap ? $s->[0] : "none";
    }
    my $s = $seg[-1]; $s->[2] = time if $s && time - $s->[2] < $gap;   # 끝날 때 gap 안이면 아직 누르는 중 (다음 반복 입력이 오기 전에 끝난 것)
    flush(1) if $label;
    print STDERR "\n$n rows -> $out\n";
' "${2:-10}" "$OUT" "$LABEL" "${GAP:-0.7}" ) < "$1"

[ -n "$S" ] && stty "$S"
printf '\a' >&2                                        # 끝 알림음
