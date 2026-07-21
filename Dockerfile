# syntax=docker/dockerfile:1.7
#
# Mednafen (Doomsday One RRDC fork) build + run container.
#
# Dual-purpose, matching the sibling ports' toolchain-image pattern:
#   * CI (build.yml) mounts a fresh checkout and rebuilds/tests inside it, so
#     the apt toolchain is provisioned once (on a Dockerfile change) instead of
#     on every push.
#   * Downstream consumers (e.g. pac-man-fx's headless functional tests) can
#     pull the image and run the baked, RRDC-enabled emulator at
#     /usr/local/bin/mednafen — no host setup, no from-source build.
#
# Unlike the pac-man-fx image there is no from-source cross-compiler here:
# Mednafen builds against stock Debian dev packages (autotools + SDL2 + FLAC +
# zlib). A PC-FX BIOS is NEVER baked in (licensing) — supply it at runtime.
#
# Build:  docker build -t ghcr.io/doomsdayonecom/mednafen-builder:dev .
#         (from a checkout cloned with --recursive: the RRDC core is a submodule)
# Run:    docker run --rm ghcr.io/doomsdayonecom/mednafen-builder:dev mednafen --help

FROM debian:bookworm-slim

ARG DEBIAN_FRONTEND=noninteractive

# Build toolchain (autotools + gettext for AM_GNU_GETTEXT), the emulator's dev
# deps (SDL2, FLAC, zlib), and the headless test kit (Xvfb + ImageMagick for
# frame grabs, python3 + pytest for the RRDC conformance suite).
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        autoconf \
        automake \
        libtool \
        gettext \
        autopoint \
        pkg-config \
        git \
        ca-certificates \
        libsdl2-dev \
        libflac-dev \
        zlib1g-dev \
        xvfb \
        imagemagick \
        python3 \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

# Regenerate the build system from tracked inputs (we don't commit the autotools
# output), then build the RRDC-enabled emulator and install it on PATH. The
# submodule is expected to be present (checkout --recursive / COPY); the update
# is a best-effort fallback and is fine to fail when .git isn't in the context.
RUN git submodule update --init --recursive 2>/dev/null || true; \
    ./autogen.sh && \
    ./configure --enable-rrdc && \
    make -j"$(nproc)" && \
    install -Dm755 src/mednafen /usr/local/bin/mednafen && \
    make clean

CMD ["mednafen", "--help"]
