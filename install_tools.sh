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
    echo "Skipping verilator installation."
else
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
    # sv2v is built in bin/ subdirectory
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
echo "  Verilator: $INSTALL_DIR/bin/verilator"
echo "  sv2v:      $INSTALL_DIR/bin/sv2v"