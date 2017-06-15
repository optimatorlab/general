# general
Our first repository

## Requirements:
- Ubuntu 14.04

## Instructions:
1) On the host machine, open a terminal window and enter:
```
sudo apt-get install git
```

2) Now, clone this repository.  Note:  It will create a subdirectory named "general" in your home directory.
```
cd ~
git clone https://github.com/optimatorlab/general
```

### If you want to run the auto-installer:
```
cd general
chmod +x *.sh
./auto_install.sh
```

- If/when prompted to remove write-protected regular files, you may enter "Y" (without the quotes).

### If you want to do the manual install:
Open [manual_install.md](manual_install.md) and follow the instructions.


### Troubleshooting

- WebGL on Chromium:
    - Open Chromium
    - Type chrome://flags in the address bar
    - CTRL-f and type "rendering list".  "Override software rendering list" will come up.
    - Click "Enable" and relaunch the browser (relaunch button appears at bottom of browser).
