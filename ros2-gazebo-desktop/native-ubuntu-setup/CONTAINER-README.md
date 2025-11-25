# Welcome to Ubuntu Desktop Container!

## 🎉 All Installation Scripts Are Pre-Loaded

You don't need to copy anything - everything is ready to use!

## Quick Start

### Option 1: Automated Installation (Recommended)

```bash
~/install-all.sh
```

This runs all installation steps automatically (~1-1.5 hours).

### Option 2: Manual Step-by-Step

```bash
~/installation-scripts/00-check-system.sh
~/installation-scripts/01-install-ros2-jazzy.sh
~/installation-scripts/02-install-gazebo-harmonic.sh
~/installation-scripts/03-install-px4-deps.sh
~/installation-scripts/04-setup-px4.sh
~/installation-scripts/05-install-microros.sh
```

Then configure:
```bash
~/configure-environment.sh
source ~/.bashrc
```

## What's Included

✅ All installation scripts in `~/installation-scripts/`
✅ Configuration script: `~/configure-environment.sh`
✅ Test script: `~/test-installation.sh`
✅ Automated installer: `~/install-all.sh`
✅ Complete documentation in `~/installation-docs/`
✅ Quick reference: `~/QUICK-REFERENCE.md`

## Desktop Shortcuts

Check your Desktop - you'll find shortcuts to:
- `install-all.sh` - Click to start automated installation
- `installation-scripts/` folder
- `docs/` folder with guides

## After Installation

Once installation completes, use these commands:

```bash
gz-default    # Terminal 1: Start Gazebo
px4-sitl      # Terminal 2: Start PX4 SITL
microros      # Terminal 3: Start micro-ROS agent
px4-topics    # Terminal 4: Monitor ROS 2 topics
```

## Documentation

- **Installation Guide**: `~/installation-docs/INSTALLATION-GUIDE.md`
- **Usage Guide**: `~/installation-docs/USAGE-GUIDE.md`
- **Quick Reference**: `~/QUICK-REFERENCE.md`

## File Locations

| Item | Location |
|------|----------|
| Installation Scripts | `~/installation-scripts/` |
| Documentation | `~/installation-docs/` |
| Automated Installer | `~/install-all.sh` |
| Environment Setup | `~/configure-environment.sh` |
| Testing | `~/test-installation.sh` |
| PX4 Workspace | `~/px4_workspace/` (persistent) |

## Persistent Storage

The `~/px4_workspace/` directory is mounted from the host machine.
Everything in this directory survives container restarts!

## Need Help?

Check the documentation:
```bash
cat ~/installation-docs/INSTALLATION-GUIDE.md
cat ~/QUICK-REFERENCE.md
```

Or view the installation log after running:
```bash
cat ~/installation.log
```

## Ready to Begin?

Just run:
```bash
~/install-all.sh
```

And grab some coffee ☕ - installation takes about 1-1.5 hours!
