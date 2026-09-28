#!/usr/bin/env bash
# Read the integrity log (rv_errlog via rv_soc_top's window at 0x1000_E000) from a board that
# is running Linux, over ssh, and name what fired:
#
#   tools/errlog-read.sh <board-ip>
#     errlog: clean  sticky=0000000000000000
#     errlog: FAULT  sticky=0000000000000040 first=dcache.replay (bit 6) at cycle 0x2f1a3b0c00
#             bit 6  dcache.replay      a replay owed with no fill in flight
#
# The bit names are the INTEGRITY LOG tables in rv_dcache.v (D$), rv_icache.v (I$) and ooo2_lsu.v,
# in the SoC's fixed assignment (D$ [15:0], I$ [31:16], LSU [47:32]); keep them in step. On a
# bitstream without the log the window reads zero and this says so instead of "clean".
set -u
IP=${1:?board ip}
# The window is read with 64-BIT LOADS. That is the contract rv_soc_top's windows serve: a
# byte load through the device path returns lane 0 replicated ('LLLLLLLL' for the magic,
# 2026-09-21), and python's struct/mmap read is byte loads on this strict-align userland. So
# the reader is a 20-line C program (/var/tmp/errlog/rdwin.c on the NFS root, built on the
# board on first use) that does `volatile uint64_t` loads: `rdwin <phys> <nwords>`.
W=$(timeout 60 ssh -o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=no "tommy@$IP" \
      'cd /var/tmp/errlog && { [ -x rdwin ] || gcc -O1 -o rdwin rdwin.c; } && sudo ./rdwin 0x1000e000 3' 2>/dev/null
) || { echo "errlog: unreadable (no ssh/gcc//dev/mem on $IP)"; exit 2; }
[ -n "$W" ] || { echo "errlog: unreadable (empty read from $IP)"; exit 2; }
set -- $W
MAGIC=$1; STICKY=$2; FIRST=$3
[ "${MAGIC: -8}" = "4552524c" ] || { echo "errlog: absent (window reads $MAGIC: a bitstream without rv_errlog)"; exit 2; }
CACHE=(orphan slot wdone beat two span tag_reuse dead_wait ro_race vv_no_pv vhit_other)
CDESC=("a fill beat for no MSHR" "a read response nothing awaits" "a write completion for no write sent" "beats out of order within a burst" "two pvalid copies of one physical line" "VA and PA disagree in the page offset" "a requester tag reused while outstanding" "a waiter parked on an MSHR that is not live" "a fill beat into a line being read out" "vvalid without pvalid" "a current-epoch virtual hit on another physical line")
ICACHE=(dual unused1 l2_orphan align skid_full pa_offset l2_both stamp_dup rc_both)
IDESC=("one line hits in both ways" "(unused)" "an L2 answer with no read outstanding" "a pair not 16-byte aligned" "a request into a full skid" "VA and PA disagree in the page offset" "a demand read and a prefetch outstanding together" "stamping a line the other way already holds" "one physical line resident in both ways")
FE=(ring_head ring_over ring_hold ring_gen ring_orphan ring_marks pq_full pq_empty pair_end fetch_cap fetch_pc bundle slot_order be_zombie be_realloc fp_orphan)
FDESC=("the fetch ring's head is not the fetch PC" "fetch consumed more halfwords than the ring holds" "the ring holds more than it reserved" "a kept I\$ answer from a stale generation" "an I\$ answer with no request in flight" "marks held + in flight != predictions queued" "a prediction pushed into a full queue" "the aligner popped an empty prediction queue" "a pair ends before the stream's address" "fetch's page cap disagrees with its oracle" "fetch's pc_q and ipc_q disagree" "a malformed bundle (slot without predecessor, fault beside a slot)" "a slot consumed out of order" "a writeback to a physical register that is not pending (a squashed op's result)" "a physical register allocated while still pending" "an FPU result with nothing in flight, or one overwriting an unconsumed result")
LSU=(dev_spec dev_span tag_reissue tag_orphan tag_reuse ld_done req_early m_spec)
LDESC=("a speculative access to a non-DRAM address" "a non-DRAM access straddling an 8-byte word" "a load tag reissued while its response is outstanding" "a fast response carrying a tag nothing is waiting on" "a tag lands and restarts in one cycle" "pt_ld_done disagrees with pt_done & ~store" "req_early on an access that is not a translate-only load" "an AMO or CBO started off the ROB head")
name() {   # <bit> -> unit.name and description
   local b=$1
   if   [ "$b" -lt 16 ]; then echo "dcache.${CACHE[$b]:-?}|${CDESC[$b]:-}"
   elif [ "$b" -lt 32 ]; then echo "icache.${ICACHE[$((b-16))]:-?}|${IDESC[$((b-16))]:-}"
   elif [ "$b" -lt 48 ]; then echo "lsu.${LSU[$((b-32))]:-?}|${LDESC[$((b-32))]:-}"
   else echo "frontend.${FE[$((b-48))]:-?}|${FDESC[$((b-48))]:-}"; fi
}
if [ "$STICKY" = "0000000000000000" ]; then echo "errlog: clean  sticky=$STICKY"; exit 0; fi
fidx=$(( (16#$FIRST >> 48) & 0xff )); fcyc=$(( 16#$FIRST & 0xffffffffffff ))
fn=$(name $fidx)
printf 'errlog: FAULT  sticky=%s first=%s (bit %d) at cycle 0x%x\n' "$STICKY" "${fn%%|*}" "$fidx" "$fcyc"
for b in $(seq 0 63); do
   if [ $(( (16#$STICKY >> b) & 1 )) -eq 1 ]; then n=$(name $b); printf '        bit %-2d %-20s %s\n' "$b" "${n%%|*}" "${n#*|}"; fi
done
exit 1
