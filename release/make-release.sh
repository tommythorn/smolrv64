#!/usr/bin/env bash
# Assemble a SmolRV64 release: for each platform its bitstream and its device trees (SD-card root and
# NFS root), the OpenSBI+Linux payload every platform boots, nfs-root.sh, a manifest and SHA256SUMS.
#
#   release/make-release.sh <tag> <fw_payload.bin> <rk build> <hdmi build> [--publish]
#
# A build is a directory holding <project>.bit and meta.txt: a met bitcache entry, or the output of
# tools/monitor-eco.sh. The files land in ./smolrv64-<tag>/; --publish then creates the GitHub
# release <tag> with them (gh), as a draft to review before it is published.
#
# The NFS trees name no server: the kernel mounts /srv/smolrv64 from the DHCP reply's next-server,
# the server the monitor's `go` loads the payload from.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
[ $# -ge 4 ] || { sed -n '2,13p' "$0"; exit 2; }
tag=$1 fw=$(realpath "$2") rk=$(realpath "$3") hdmi=$(realpath "$4") publish=${5:-}
out=$PWD/smolrv64-$tag
[ ! -e "$out" ] || { echo "$out exists" >&2; exit 1; }
mkdir -p "$out"
NFSROOT=/srv/smolrv64
DIV8=$(sed -n 's/^DIV8 *?= *//p' "$here/workloads/ubuntu/Makefile")

# meta.txt's commit: a bitcache entry's "commit:", a monitor ECO's "base commit:".
rtl() { sed -n 's/^\(base \)\{0,1\}commit: *\([0-9a-f]\{8\}\).*/\2/p' "$1/meta.txt"; }
{
    echo "SmolRV64 $tag, assembled $(date '+%F') from $(git -C "$here" log -1 --format='%h %s')"
    echo
    echo "fw_payload.bin: OpenSBI and $(strings "$fw" | grep -m1 -oE '^Linux version [^ ]+ .*PREEMPT.*[0-9]{4}$' || strings "$fw" | grep -m1 -oE '^Linux version [^ ]+')"
} > "$out/MANIFEST.txt"
cp "$fw" "$out/fw_payload.bin"
for p in rk-xcku5p:"$rk":rk_xcku5p: hdmi-xcku5p:"$hdmi":hdmi_xcku5p:--hdmi; do
    IFS=: read -r name dir proj flag <<< "$p"
    [ -f "$dir/$proj.bit" ] || { echo "$dir has no $proj.bit" >&2; exit 1; }
    grep -qE '^(met: *yes|base wns: *[0-9])' "$dir/meta.txt" || { echo "$dir: not a met build" >&2; exit 1; }
    cp "$dir/$proj.bit" "$out/smolrv64-$name.bit"
    for root in sd nfs; do
        r=$([ $root = sd ] && echo --sd-root || echo --nfsroot=$NFSROOT)
        python3 "$here/tools/check-dts-timebase.py" --gen "$DIV8" $flag $r --rtl="$(rtl "$dir")" \
            "$here/workloads/ubuntu/ubuntu-nfs.dts.in" "$out/$name-$root.dts" > /dev/null
        dtc -q -I dts -O dtb -o "$out/$name-$root.dtb" "$out/$name-$root.dts"
        rm "$out/$name-$root.dts"
    done
    { echo; echo "smolrv64-$name.bit:"; sed 's/^/  /' "$dir/meta.txt"; } >> "$out/MANIFEST.txt"
done
cp "$here/release/nfs-root.sh" "$out/"
(cd "$out" && sha256sum -- * > SHA256SUMS)
ls -la "$out"
if [ "$publish" = --publish ]; then
    notes=$(mktemp)
    {
        echo "Boot Ubuntu 25.04 on the RK-XCKU5P-F v1.2 or the HDMI-XCKU5P: [Running it on a board](https://github.com/tommythorn/smolrv64#running-it-on-a-board)."
        echo; echo '```'; cat "$out/MANIFEST.txt"; echo '```'
    } > "$notes"
    # gh finds the repository from the checkout's remote, so it runs there.
    (cd "$here" && gh release create "$tag" --draft --target "$(git rev-parse HEAD)" \
        --title "SmolRV64 $tag" --notes-file "$notes" "$out"/*)
    rm "$notes"
fi
