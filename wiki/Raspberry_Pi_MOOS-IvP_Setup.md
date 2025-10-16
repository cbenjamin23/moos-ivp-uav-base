# Raspberry Pi MOOS-IvP Setup from macOS for UAV Projects

**Note:** Please change hostname and filenames to match your project


## 0) Prereqs on your Mac
Install Raspberry Pi Imager (from raspberrypi.com).

Install Homebrew (if you don’t have it):
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Install ssh-copy-id via Homebrew:
```bash
brew install ssh-copy-id
```


## 1) Flash the SD card with Raspberry Pi Imager
1. Insert the SD card → open Raspberry Pi Imager.
2. Choose OS: e.g., Raspberry Pi OS Lite (64-bit).
3. Choose Storage: select your SD card.
4. Click ⚙ (settings/advanced) and set:
   - Hostname: `skywalker1.local`
   - Enable SSH: on (password auth is fine for first login)
   - Username: `uav`
   - Password: (choose a strong one)
   - Wi-Fi: SSID + password
   - Wireless LAN country: `CA`
   - Locale: timezone + keyboard (`us`)
5. Save → Write the image. Eject card.


## 2) First boot & test SSH
Insert SD into the Pi, power it, wait ~1–2 minutes.  
From your Mac:
```bash
ssh uav@skywalker1.local
```
Enter password from step 1.


## 3) Generate an SSH key on your Mac
Create a dedicated key so you don’t overwrite existing ones:
```bash
ssh-keygen -t ed25519 -C "pi-skywalker" -f ~/.ssh/id_ed25519_skywalker1
```
Passphrase (optional). Files created:
- Private key: `~/.ssh/id_ed25519_skywalker1`
- Public key: `~/.ssh/id_ed25519_skywalker1.pub`


## 4) Copy your public key to the Pi
```bash
ssh-copy-id -i ~/.ssh/id_ed25519_skywalker1.pub uav@skywalker1.local
```


## 5) Make SSH seamless
Edit `~/.ssh/config`:
```bash
Host skywalker1
    HostName skywalker1.local
    User uav
    IdentityFile ~/.ssh/id_ed25519_skywalker1
    IdentitiesOnly yes
    AddKeysToAgent yes
    UseKeychain yes
```
Then run:
```bash
ssh-add --apple-use-keychain ~/.ssh/id_ed25519_skywalker1
```


## 6) Test passwordless login
```bash
ssh skywalker1
```
If password still required, fix permissions on Pi:
```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
chown -R uav:uav ~/.ssh
```


## 7) Update and upgrade packages
```bash
sudo apt-get update
sudo apt-get upgrade -y
sudo systemctl disable --now packagekit   # optional
```


## 8) Install useful packages
```bash
sudo apt-get --assume-yes install emacs wget members cmake subversion screen ncdu watch git
```


## 9) Install & build core repos
### 9.1 System prep
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential git curl file cmake ninja-build pkg-config
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

### 9.2 Install uav-common
Note: For now, contact chbenj36@gmail.com to request access
```bash
cd ~
git clone https://github.com/cbenjamin23/uav-common.git
cd ~/uav-common
./build.sh
```
Add to `~/.bashrc`:
```bash
git -C ~/uav-common pull --quiet --ff-only && source ~/uav-common/bashrc_common.sh
```

### 9.3 Build moos-ivp related repos
Follow additional wiki instructions to build the following repos on your pi:
- moos-ivp-swarm (private)
- moos-ivp-uav-base (this)
- MAVSDK (inside moos-ivp-uav-base), 
- Note: generally use the `minrobot` option, when presented, to build repos on the pi as it simply excludes gui-related apps.

## 10) To Be Continued
