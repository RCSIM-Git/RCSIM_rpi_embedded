# Definicje Usług systemd dla RCSIM

### `rcsim-supervisor.service`
```ini
[Unit]
Description=RCSIM Supervisor Service
After=network-online.target
[Service]
Type=simple
User=pi
ExecStart=/home/pi/rcsim_project/venv/bin/python3 /home/pi/rcsim_project/core/supervisor.py
Restart=always
[Install]
WantedBy=multi-user.target
```

### `rcsim-telemetry.service`
```ini
[Unit]
Description=RCSIM Telemetry Main Service
After=rcsim-supervisor.service
BindsTo=rcsim-supervisor.service
[Service]
Type=simple
User=pi
ExecStart=/home/pi/rcsim_project/venv/bin/python3 /home/pi/rcsim_project/core/main_service.py
Restart=on-failure
[Install]
WantedBy=multi-user.target
```
