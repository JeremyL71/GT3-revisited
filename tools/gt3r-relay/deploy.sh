#!/bin/sh
# Builds and installs gt3r-relay on a fresh Debian/Ubuntu server over SSH.
#
#   tools/gt3r-relay/deploy.sh root@203.0.113.7          # first install or update
#   tools/gt3r-relay/deploy.sh root@203.0.113.7 --dry-run  # only build the source bundle
#
# Copies only the relay's own sources (no game code or data), installs g++ and cmake,
# builds, installs /usr/local/bin/gt3r-relay (and gt3r-relay-bench) and the systemd unit, opens UDP 47631 in
# ufw when ufw is active, and (re)starts the service. Run it again to update. The game versions the
# relay announces live in /etc/gt3r-relay/versions.conf (set-versions.sh); an update keeps them.
set -eu

target=${1:-}
[ -n "$target" ] || { echo "usage: $0 user@host [--dry-run]" >&2; exit 2; }
root=$(cd "$(dirname "$0")/../.." && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

mkdir -p "$work/gt3r-relay/src/net" "$work/gt3r-relay/src/base" "$work/gt3r-relay/src/sim" "$work/gt3r-relay/tools"
cp "$root"/src/net/socket.* "$root"/src/net/protocol.* "$root"/src/net/rendezvous.* "$root"/src/net/relay_server.* \
   "$work/gt3r-relay/src/net/"
cp "$root/src/base/bytes.h" "$work/gt3r-relay/src/base/"
cp "$root/src/sim/sim_types.h" "$work/gt3r-relay/src/sim/"
cp -R "$root/tools/gt3r-relay" "$work/gt3r-relay/tools/"
COPYFILE_DISABLE=1 tar --no-xattrs -C "$work" -czf "$work/gt3r-relay.tgz" gt3r-relay
echo "bundle: $(du -h "$work/gt3r-relay.tgz" | cut -f1)"

if [ "${2:-}" = "--dry-run" ]; then
    tar -tzf "$work/gt3r-relay.tgz"
    exit 0
fi

# One shared connection for every step (one key approval, no reconnects).
ctl="/tmp/gt3r-ssh-%C"
opts="-o ControlMaster=auto -o ControlPath=$ctl -o ControlPersist=120 -o ConnectTimeout=20 -o ServerAliveInterval=10"
for try in 1 2 3; do
    ssh $opts "$target" true && break
    [ "$try" = 3 ] && { echo "cannot reach $target over SSH" >&2; exit 1; }
    echo "SSH attempt $try failed, retrying..." >&2
    sleep 5
done
trap 'ssh $opts -O exit "$target" 2>/dev/null; rm -rf "$work"' EXIT
scp -q $opts "$work/gt3r-relay.tgz" "$target:/tmp/gt3r-relay.tgz"
ssh $opts "$target" 'sh -s' <<'REMOTE'
set -eu
SUDO=""; [ "$(id -u)" -eq 0 ] || SUDO=sudo
export DEBIAN_FRONTEND=noninteractive
command -v g++ >/dev/null && command -v cmake >/dev/null || {
    $SUDO apt-get update -q
    $SUDO apt-get install -y -q --no-install-recommends g++ cmake make
}
rm -rf /tmp/gt3r-relay && tar -C /tmp -xzf /tmp/gt3r-relay.tgz
cmake -S /tmp/gt3r-relay/tools/gt3r-relay -B /tmp/gt3r-relay/build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build /tmp/gt3r-relay/build -j 2 --target gt3r-relay
$SUDO install -m 755 /tmp/gt3r-relay/build/gt3r-relay /usr/local/bin/gt3r-relay
# The load test too (docs/dev/ONLINE.md "Benchmark"); optional, it never blocks the relay.
if cmake --build /tmp/gt3r-relay/build -j 2 --target gt3r-relay-bench >/dev/null 2>&1; then
    $SUDO install -m 755 /tmp/gt3r-relay/build/gt3r-relay-bench /usr/local/bin/gt3r-relay-bench
else
    echo "note: gt3r-relay-bench did not build (the relay is fine)" >&2
fi
$SUDO install -m 644 /tmp/gt3r-relay/tools/gt3r-relay/gt3r-relay.service /etc/systemd/system/gt3r-relay.service
# Game versions (set-versions.sh writes the file; an update keeps it).
$SUDO install -d -m 755 /etc/gt3r-relay
if command -v ufw >/dev/null && $SUDO ufw status | grep -q active; then
    $SUDO ufw allow 47631/udp >/dev/null
fi
$SUDO systemctl daemon-reload
$SUDO systemctl enable gt3r-relay >/dev/null 2>&1
$SUDO systemctl restart gt3r-relay
sleep 1
$SUDO systemctl --no-pager --lines=4 status gt3r-relay | sed -n '1,9p'
[ -f /etc/gt3r-relay/versions.conf ] || echo "note: no /etc/gt3r-relay/versions.conf yet: every game version may play (tools/gt3r-relay/set-versions.sh)"
rm -rf /tmp/gt3r-relay /tmp/gt3r-relay.tgz
REMOTE
echo "done: players enter $(echo "$target" | sed 's/.*@//') under Relay server"
