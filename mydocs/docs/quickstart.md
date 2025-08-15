# ROS 2 Installation Guide
This guide explains how to install **ROS 2 (Humble)** on **Windows 11**, **WSL2**, and **Jetson (Ubuntu)** environments.


## 1. Installing ROS 2 on Windows 11
### 1.1 Prerequisites
- Windows 11 (64-bit)
- [Chocolatey](https://chocolatey.org/install) installed
- PowerShell running in **Administrator mode**

### 1.2 Installation Steps
```powershell
# Update Chocolatey
choco upgrade chocolatey

# Install ROS 2 Humble
choco install ros-humble-desktop -y

# Install additional tools
choco install git cmake python -y



## On Wsl2

## On Jetson