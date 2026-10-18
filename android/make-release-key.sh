#!/bin/sh
# Creates the project's Android release signing key (once). The certificate
# names the project, not a person. Keep the .jks and its password safe (a
# password manager): every future release must be signed with this same key,
# or installed copies can't update.
#
#   android/make-release-key.sh [out.jks]
#
# Then store it as GitHub Actions secrets:
#   base64 -i gt3r-release.jks | gh secret set ANDROID_KEYSTORE_BASE64
#   gh secret set ANDROID_KEYSTORE_PASSWORD
set -eu
out="${1:-gt3r-release.jks}"
if [ -e "$out" ]; then
    echo "$out already exists; not overwriting it." >&2
    exit 1
fi
keytool -genkeypair -v -keystore "$out" -storetype PKCS12 -alias gt3r \
    -keyalg RSA -keysize 4096 -validity 36500 \
    -dname "CN=GT3 Revisited, O=GT3 Revisited"
echo "Created $out (alias gt3r). Do not commit it."
