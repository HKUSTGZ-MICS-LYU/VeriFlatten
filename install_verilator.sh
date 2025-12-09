#!/bin/bash

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    URL="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2025-12-09/oss-cad-suite-linux-x64-20251209.tgz"
    ARCHIVE_NAME="oss-cad-suite-linux-x64-20251209.tgz"
    EXPECTED_SHA256="28d2d6cdb7ebef09f0e5972895f154fbc770b2ee7a054aea327c84875053a9c5"
elif [ "$ARCH" = "aarch64" ]; then
    URL="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2025-12-09/oss-cad-suite-linux-arm64-20251209.tgz"
    ARCHIVE_NAME="oss-cad-suite-linux-arm64-20251209.tgz"
    EXPECTED_SHA256="4c474dcc8dc7da7780f3a898396009184bd6bc3b797f81c24c974237149998a4"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

INSTALL_DIR="$(pwd)/oss-cad-suite"

# Check if already installed
if [ -d "$INSTALL_DIR" ]; then
    echo "oss-cad-suite directory already exists at $INSTALL_DIR"
    echo "Skipping installation."
    exit 0
fi

echo "Downloading oss-cad-suite from $URL..."
wget -c "$URL" -O "$ARCHIVE_NAME"

if [ $? -ne 0 ]; then
    echo "Download failed, please check network or link."
    exit 1
fi

echo "Verifying checksum..."
echo "$EXPECTED_SHA256  $ARCHIVE_NAME" | sha256sum -c -
if [ $? -ne 0 ]; then
    echo "Checksum verification failed!"
    exit 1
fi

echo "Extracting..."
tar -xzf "$ARCHIVE_NAME"
TAR_EXIT_CODE=$?

# Clean up archive immediately
rm "$ARCHIVE_NAME"

if [ $TAR_EXIT_CODE -ne 0 ]; then
    echo "Extraction failed."
    exit 1
fi

# ==========================================
# Aggressive Cleanup for Verilator Only
# ==========================================
echo "Pruning installation to keep only Verilator..."
cd "$INSTALL_DIR" || exit

# 1. Clean BIN directory
# Keep only files starting with "verilator" (verilator, verilator_bin, verilator_coverage, etc.)
# We use 'find' to delete everything that does NOT match the pattern.
find bin/ -type f -not -name "verilator*" -delete
find bin/ -type l -not -name "verilator*" -delete

# 2. Clean SHARE directory
# Verilator needs 'share/verilator' (contains include files like verilated.cpp).
# Remove all other folders in share/ (like yosys, ghdl, trellis, etc.)
find share/ -mindepth 1 -maxdepth 1 -type d -not -name "verilator" -exec rm -rf {} +

# 3. Clean Root directories
# Remove folders that Verilator definitely doesn't use.
# We KEEP 'lib' because binaries in 'bin' often depend on shared libraries in 'lib'.
# We KEEP 'libexec' as requested.
rm -rf py3bin examples super_prove environment environment.fish manifest.json

cd ..

echo "Installation and cleanup complete."
echo "Verilator path: $INSTALL_DIR/bin/verilator"