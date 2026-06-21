#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "usage: $0 <binary> <start-addr> [output]" >&2
    echo "example: $0 workloads/ubuntu/ubuntu.dtb 0xfffff000 workloads/ubuntu/ubuntu.dtb.writes" >&2
}

if (( $# < 2 || $# > 3 )); then
    usage
    exit 2
fi

input=$1
addr=${2#0x}
addr=${addr#0X}

if [[ ! -f "$input" || ! "$addr" =~ ^[0-9a-fA-F]+$ ]]; then
    usage
    exit 2
fi

emit() {
    perl -e '
        use strict;
        use warnings;

        my ($file, $addr_text) = @ARGV;
        my $addr = hex($addr_text);

        open my $fh, "<:raw", $file or die "$file: $!";

        while (1) {
            my $buf = "";
            my $n = read($fh, $buf, 8);
            die "read $file: $!" unless defined $n;
            last if $n == 0;

            $buf .= "\0" x (8 - $n);
            printf "W%08x %016x\n", $addr, unpack("Q<", $buf);
            $addr += 8;
        }
    ' "$input" "$addr"
}

if (( $# == 3 )); then
    emit > "$3"
else
    emit
fi
