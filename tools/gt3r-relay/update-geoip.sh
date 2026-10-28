#!/bin/sh
# Fetches DB-IP's free "IP to Country Lite" list (https://db-ip.com, CC BY 4.0) for
# gt3r-relay --geoip, then reloads the relay (sessions stay). deploy.sh installs it as
# /usr/local/bin/gt3r-relay-update-geoip and runs it monthly (gt3r-relay-geoip.timer).
#
#   gt3r-relay-update-geoip [/var/lib/gt3r-relay/country.csv]
#
# A failed download keeps the list there is. Needs curl, gunzip and GNU date.
set -eu

out=${1:-/var/lib/gt3r-relay/country.csv}
dir=$(dirname "$out")
mkdir -p "$dir"
tmp=$(mktemp "$dir/.country.XXXXXX")
trap 'rm -f "$tmp" "$tmp.gz"' EXIT

# This month's list, else last month's (a new one comes out early in the month).
this=$(date -u +%Y-%m)
last=$(date -u -d "$(date -u +%Y-%m-15) -1 month" +%Y-%m)
for month in "$this" "$last"; do
    if curl -fsS --max-time 300 -o "$tmp.gz" "https://download.db-ip.com/free/dbip-country-lite-$month.csv.gz"; then
        gunzip -c "$tmp.gz" >"$tmp"
        chmod 644 "$tmp"
        mv "$tmp" "$out"
        echo "country list $month: $(wc -l <"$out") ranges in $out"
        if systemctl is-active --quiet gt3r-relay 2>/dev/null; then systemctl reload gt3r-relay; fi
        exit 0
    fi
done
echo "could not fetch the DB-IP country list for $this or $last (kept the one there is)" >&2
exit 1
