# DeviceShred — Secure Removable Wiper

DeviceShred is a Qt-based desktop application for **securely wiping removable storage devices** (USB sticks, SD cards, external drives, etc.) on Linux.
<img width="1102" height="744" alt="screenshot" src="https://github.com/user-attachments/assets/34de7be9-666e-4733-afb7-ee17c92dae29" />

##  Features

* **System disks hidden** → prevents accidental formatting.
* **Wipe methods**:

  * Fill with zeros
  * Fill with random data (`/dev/urandom`)
* **TRIM/BLKDISCARD support** → faster cleanup on supported devices.
* **Configurable block size** (256 KiB – 16 MiB).
* **Modern, animated UI** (Qt Widgets + QSS).

## 🛠 Build Instructions

### Requirements

* Qt 5 or 6 (with development packages)
* C++17 compatible compiler (g++, clang)

On Ubuntu/Debian-based systems:

```bash
sudo apt install qtbase5-dev qt5-qmake g++ make
```

On Arch-based systems:

```bash
sudo pacman -S base-devel qt5-base clang

```

### Steps



1. Generate Makefile with `qmake`:

```bash
qmake DeviceShred.pro
```

2. Build with `make`:

```bash
make
```

3. Run (with root privileges):

```bash
sudo ./DeviceShred
```


