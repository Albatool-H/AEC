#!/usr/bin/env bash
set -euo pipefail

CODE="$(cd "$(dirname "$0")" && pwd)"
PREFIX="${NDNSIM_DIR:-/opt/ndnsim}"
BASE="$PREFIX/root/ndnSIM"
WORK="${NDNSIM_SRC:-/opt/src}"

NS3_REPO=https://github.com/named-data-ndnSIM/ns-3-dev.git
NS3_TAG=ns-3.23-dev-ndnSIM-2.1
NDNSIM_REPO=https://github.com/named-data-ndnSIM/ndnSIM.git
NDNSIM_TAG=ndnSIM-2.1

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends software-properties-common
add-apt-repository -y universe
apt-get update
apt-get install -y --no-install-recommends \
    git ca-certificates wget patch pkg-config cmake make \
    build-essential g++ gcc \
    python2 python2-dev python-setuptools \
    python3 python3-dev python3-pip python3-setuptools \
    libsqlite3-dev libcrypto++-dev libboost-all-dev libssl-dev \
    libxml2-dev fonts-urw-base35

pip3 install --no-cache-dir "numpy==1.24.4" "matplotlib==3.7.5" "pillow==10.4.0" \
    "networkx==3.1" "vowpalwabbit==9.10.0"

mkdir -p "$WORK"
cd "$WORK"

git clone --depth 1 --branch "$NS3_TAG" "$NS3_REPO" ns-3

mkdir -p ns-3/src
git clone --recursive "$NDNSIM_REPO" ns-3/src/ndnSIM
git -C ns-3/src/ndnSIM checkout "$NDNSIM_TAG"
git -C ns-3/src/ndnSIM submodule update --init --recursive

patch -d ns-3 -p1 < "$CODE/patches/ns-3.patch"
patch -d ns-3/src/ndnSIM -p1 < "$CODE/patches/ndnSIM.patch"

cd "$WORK/ns-3"
python2 ./waf configure \
    --build-profile=optimized \
    --prefix="$BASE"
python2 ./waf -j"$(nproc)"
python2 ./waf install

if [ ! -e "$BASE/lib64" ]; then
    ln -s lib "$BASE/lib64"
fi

LIC="$PREFIX/licenses"
mkdir -p "$LIC/ns-3" "$LIC/ndnSIM" "$LIC/ndn-cxx"
for f in LICENSE COPYING COPYING.md AUTHORS README.md; do
    [ -f "$WORK/ns-3/$f" ] && cp "$WORK/ns-3/$f" "$LIC/ns-3/" || true
    [ -f "$WORK/ns-3/src/ndnSIM/$f" ] && cp "$WORK/ns-3/src/ndnSIM/$f" "$LIC/ndnSIM/" || true
    [ -f "$WORK/ns-3/src/ndnSIM/ndn-cxx/$f" ] && cp "$WORK/ns-3/src/ndnSIM/ndn-cxx/$f" "$LIC/ndn-cxx/" || true
done
find "$LIC" -type f | sort

find "$BASE/lib" -name '*.so' -exec strip --strip-unneeded {} + 2>/dev/null || true
chmod -R a+rX "$PREFIX" "$WORK"
apt-get clean
rm -rf /var/lib/apt/lists/*

du -sh "$BASE"
