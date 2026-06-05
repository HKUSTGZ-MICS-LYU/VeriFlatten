#!/bin/bash
#
# Install OSS CAD Suite (full, no pruning) + sv2v
#
# Keeps everything from oss-cad-suite — all tools, all libraries,
# all share files, etc. This is the complete installation.
#

set -euo pipefail

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
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Check if already installed
if [ -d "$INSTALL_DIR" ]; then
    echo "oss-cad-suite directory already exists at $INSTALL_DIR"
    echo "Skipping installation."
else
    echo "Downloading oss-cad-suite from $URL..."
    cd "$SCRIPT_DIR"
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

    echo "oss-cad-suite extracted in full (no pruning)."
fi

# ==========================================
# Install sv2v from source
# ==========================================
SV2V_BIN="$INSTALL_DIR/bin/sv2v"

if [ -f "$SV2V_BIN" ]; then
    echo "sv2v already installed at $SV2V_BIN"
else
    echo "Installing sv2v from source..."

    # Check if stack is available
    if ! command -v stack &> /dev/null; then
        echo "Error: 'stack' is not installed. Please install Haskell Stack first."
        echo "  See: https://docs.haskellstack.org/en/stable/install_and_upgrade/"
        exit 1
    fi

    # Create temporary build directory
    BUILD_DIR="$(pwd)/sv2v_build"
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    # Clone sv2v repository
    echo "Cloning sv2v repository..."
    git clone https://github.com/zachjs/sv2v.git
    if [ $? -ne 0 ]; then
        echo "Failed to clone sv2v repository."
        rm -rf "$BUILD_DIR"
        exit 1
    fi

    cd sv2v

    # Configure stack to use China mirror (faster download in China)
    echo "Configuring stack to use China mirror..."
    mkdir -p ~/.stack
    cat > ~/.stack/config.yaml << 'EOF'
# China mirror for faster download
setup-info-locations:
  - http://mirrors.tuna.tsinghua.edu.cn/stackage/stack-setup.yaml

packages:
  hackage:
    url: http://mirrors.tuna.tsinghua.edu.cn/hackage/
EOF

    # Build sv2v
    echo "Building sv2v (this may take a while)..."
    make
    if [ $? -ne 0 ]; then
        echo "Failed to build sv2v."
        rm -rf "$BUILD_DIR"
        exit 1
    fi

    # Copy binary to installation directory
    echo "Installing sv2v binary..."
    if [ -f "bin/sv2v" ]; then
        cp bin/sv2v "$SV2V_BIN"
    else
        echo "Error: sv2v binary not found in bin/"
        rm -rf "$BUILD_DIR"
        exit 1
    fi
    chmod +x "$SV2V_BIN"

    # Clean up build directory
    cd ..
    rm -rf "$BUILD_DIR"

    echo "sv2v installed at $SV2V_BIN"
fi

echo ""
echo "Installation complete."
echo "  oss-cad-suite: $INSTALL_DIR (full)"
echo "  Verilator: $INSTALL_DIR/bin/verilator"
echo "  yosys:     $INSTALL_DIR/bin/yosys"
echo "  sby:       $INSTALL_DIR/bin/sby"
echo "  sv2v:      $INSTALL_DIR/bin/sv2v"
