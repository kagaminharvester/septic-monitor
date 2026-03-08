# Flashing Instructions for Septic Tank Monitor (LILYGO TTGO T-Display)

## Prerequisites
- LILYGO TTGO T-Display connected via USB OTG cable
- Python 3 + esptool installed
- Compiled firmware: `.pio/build/ttgo-t-display/firmware.bin`

## Flash from Linux/Mac/Windows PC
```bash
# Install esptool
pip install esptool

# Find the serial port (COM on Windows, /dev/ttyUSB* or /dev/ttyACM* on Linux/Mac)
# Then flash:
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
```

## Flash from Android/Termux
```bash
# Make sure USB permission is granted
termux-usb -l
termux-usb -r /dev/bus/usb/001/002

# Flash via esptool with USB FD trick
python -m esptool --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
```

## Verify Flash
After flashing, the board will reboot. Check the TFT display:
- Should show "SEPTIC MONITOR" title
- Tank gauge with fill animation
- WiFi connection status (AP mode if no WiFi configured)

## Configure WiFi
1. Open web browser
2. If connected to WiFi: visit `http://<board-ip>/`
3. If in AP mode: connect to WiFi network **"SepticMonitor"** (password: **septic1234**)
4. Visit `http://192.168.4.1/`
5. Go to Settings tab, enter WiFi credentials and save

## First Measurement
1. Click "Measure Now" button on web GUI, or
2. Press top button on board (GPIO0)
3. Board will scan for 30 seconds (configurable)
4. Tank level appears on TFT and web dashboard

## Demo Mode (Live Scanning)
Click "Start Demo" button to continuously scan without waiting for scheduled time.
Press bottom button on board (GPIO35) to toggle demo mode.

## Scheduled Measurements
Configure in Settings:
- **Once daily**: 6 AM
- **Twice daily**: 6 AM + 6 PM (default)
- **Every 6 hours**: 12 AM, 6 AM, 12 PM, 6 PM
- **Manual only**: disabled

## API Endpoints
- `GET /api/status` — current tank level
- `POST /api/measure` — trigger measurement
- `POST /api/demo` — toggle demo mode
- `GET/POST /api/settings` — load/save config
- `GET /api/log` — measurement history

## Troubleshooting
**Board won't boot after flash:**
- Hold RESET button 3 seconds, then release
- Check TFT backlight (GPIO4) brightness

**Can't connect to WiFi:**
- Board falls back to AP mode after 15s timeout
- Connect to "SepticMonitor" AP to reconfigure

**Measurement always fails:**
- Check JSN-SR04T wiring (Trigger=GPIO26, Echo=GPIO25)
- Verify sensor can see liquid surface (no obstruction)
- Try manual calibration in demo mode

**Web GUI shows "Offline":**
- Check WiFi connection on board (displayed on TFT bottom)
- Refresh page and wait 5 seconds for status update
