#!/bin/bash
# Install QGroundControl (Latest Version)
# Downloads and sets up QGroundControl AppImage

set -e

echo "==========================================="
echo "  Installing QGroundControl"
echo "==========================================="
echo ""
echo "This will:"
echo "  - Download latest QGroundControl AppImage"
echo "  - Install to ~/Applications"
echo "  - Create desktop shortcut"
echo "  - Make it executable"
echo ""
echo "Estimated time: 2-5 minutes (depends on download speed)"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

# Create Applications directory
APPS_DIR="$HOME/Applications"
mkdir -p "$APPS_DIR"

echo ""
echo "Step 1/5: Installing dependencies..."
sudo apt update
sudo apt install -y \
    libfuse2 \
    fuse \
    wget \
    curl \
    desktop-file-utils

echo ""
echo "Step 2/5: Finding latest QGroundControl release..."

# Get latest release URL from GitHub
LATEST_RELEASE_URL=$(curl -s https://api.github.com/repos/mavlink/qgroundcontrol/releases/latest | grep "browser_download_url.*AppImage" | grep -v "md5" | cut -d '"' -f 4)

if [ -z "$LATEST_RELEASE_URL" ]; then
    echo "❌ Could not find latest release"
    echo "   Falling back to stable version 4.4.0..."
    LATEST_RELEASE_URL="https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.0/QGroundControl.AppImage"
fi

echo "   Download URL: $LATEST_RELEASE_URL"

QGC_PATH="$APPS_DIR/QGroundControl.AppImage"

if [ -f "$QGC_PATH" ]; then
    echo "⚠️  QGroundControl already exists at $QGC_PATH"
    read -p "Overwrite? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping download"
        QGC_EXISTS=true
    else
        echo ""
        echo "Downloading QGroundControl AppImage..."
        echo "   (This may take a few minutes...)"
        wget --show-progress -O "$QGC_PATH" "$LATEST_RELEASE_URL"
        QGC_EXISTS=false
    fi
else
    echo ""
    echo "Downloading QGroundControl AppImage..."
    echo "   (This may take a few minutes...)"
    wget --show-progress -O "$QGC_PATH" "$LATEST_RELEASE_URL"
    QGC_EXISTS=false
fi

echo ""
echo "Step 3/5: Extracting AppImage (FUSE not available in Docker)..."
chmod +x "$QGC_PATH"

# Extract AppImage since FUSE is not available in containers
cd "$APPS_DIR"
if [ -d "QGroundControl" ]; then
    echo "   Removing old extracted version..."
    rm -rf QGroundControl
fi

echo "   Extracting AppImage contents..."
"$QGC_PATH" --appimage-extract > /dev/null 2>&1
mv squashfs-root QGroundControl

# Create wrapper script
QGC_WRAPPER="$APPS_DIR/qgroundcontrol"
cat > "$QGC_WRAPPER" << 'WRAPPER_EOF'
#!/bin/bash
cd ~/Applications/QGroundControl
./AppRun "$@"
WRAPPER_EOF
chmod +x "$QGC_WRAPPER"

echo "   ✅ AppImage extracted to $APPS_DIR/QGroundControl"

# Keep the original AppImage for reference
echo "   Original AppImage kept at: $QGC_PATH"

echo ""
echo "Step 4/5: Creating desktop shortcut..."

# Create desktop entry
DESKTOP_FILE="$HOME/.local/share/applications/qgroundcontrol.desktop"
mkdir -p "$HOME/.local/share/applications"

cat > "$DESKTOP_FILE" << 'EOF'
[Desktop Entry]
Version=1.0
Type=Application
Name=QGroundControl
Comment=Ground Control Station for PX4 and ArduPilot
Exec=@@WRAPPER_PATH@@
Icon=qgroundcontrol
Terminal=false
Categories=Development;Science;
StartupNotify=true
EOF

# Replace placeholder with actual path
sed -i "s|@@WRAPPER_PATH@@|$QGC_WRAPPER|g" "$DESKTOP_FILE"

# Also create desktop shortcut
DESKTOP_SHORTCUT="$HOME/Desktop/QGroundControl.desktop"
cp "$DESKTOP_FILE" "$DESKTOP_SHORTCUT"
chmod +x "$DESKTOP_SHORTCUT"

# Copy icon from extracted directory
echo ""
echo "Step 5/5: Setting up icon..."
ICON_SOURCE="$APPS_DIR/QGroundControl/usr/share/icons/hicolor/128x128/apps/qgroundcontrol.png"
if [ -f "$ICON_SOURCE" ]; then
    ICON_DIR="$HOME/.local/share/icons/hicolor/128x128/apps"
    mkdir -p "$ICON_DIR"
    cp "$ICON_SOURCE" "$ICON_DIR/"
    echo "   ✅ Icon installed"
else
    echo "   ⚠️  Icon not found (will use default)"
fi

echo ""
echo "==========================================="
echo "  ✅ QGroundControl Installation Complete!"
echo "==========================================="
echo ""
echo "Installed at:"
echo "  $APPS_DIR/QGroundControl/ (extracted)"
echo "  $QGC_PATH (original AppImage)"
echo ""
echo "To launch QGroundControl:"
echo "  1. Double-click desktop icon"
echo "  2. Search in applications menu"
echo "  3. Run from terminal:"
echo "     qgroundcontrol"
echo "     (or: $QGC_WRAPPER)"
echo ""
echo "Note: AppImage extracted because FUSE is not available"
echo "      in Docker containers. This is normal and working."
echo ""
echo "Connection settings for Docker container:"
echo "  - UDP: Auto-connect on port 14550 (default)"
echo "  - TCP: Manually add connection"
echo "         Type: TCP"
echo "         Server: localhost"
echo "         Port: 5760"
echo ""
echo "To update QGroundControl:"
echo "  - Just run this script again"
echo "  - Or download manually from:"
echo "    https://qgroundcontrol.com"
echo ""
echo "==========================================="
