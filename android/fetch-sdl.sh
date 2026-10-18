#!/bin/sh
# Fetches the SDL3 source used for both the Java glue and the native build.
set -e
cd "$(dirname "$0")"
TAG=release-3.4.12
if [ ! -d third_party/SDL ]; then
  git clone --depth 1 --branch "$TAG" https://github.com/libsdl-org/SDL.git third_party/SDL
fi
