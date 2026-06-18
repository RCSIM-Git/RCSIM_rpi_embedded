# 🍓 Moduł Wdrożeniowy RCSIM (Raspberry Pi 5 + Hailo-8)

Niniejsze repozytorium zawiera oprogramowanie pokładowe oraz infrastrukturę wdrożeniową dla autonomicznego pojazdu **RCSIM (Race Ready Autonomous System)** uruchamianego bezpośrednio na **Raspberry Pi 5** z akceleratorem **Hailo-8 / Hailo-8L**.

System odpowiada za bezpośrednią kontrolę nad fizycznym pojazdem RC, przetwarzanie danych sensorycznych w czasie rzeczywistym, wnioskowanie AI na NPU (Neural Processing Unit), planowanie trasy (SLAM/Cartographer/A*/Pure Pursuit) oraz dwukierunkową transmisję niskopoźnieniową (WebRTC/UDP) z systemem GCS (Ground Control Station) na PC.

---

## 🚀 Główne Odpowiedzialności i Funkcje

1. **Sterowanie i Kontrola Sprzętu (Hardware I/O)**
   - Integracja z kontrolerem PWM **PCA9685** (obsługa skrętu oraz przepustnicy pojazdu).
   - Obsługa czujników IMU oraz odbiorników GPS (protokoły NMEA, konfiguracja poprawek **RTK/NTRIP**).
   - Odczyt aparatury RC przez parser **CRSF (Crossfire)**.

2. **Detekcja i Wnioskowanie AI (Hailo-8 / Hailo-8L)**
   - Sprzętowa akceleracja End-to-End Regression (RCSIM) na koprocesorze Hailo-8 za pomocą pakietów `.hef`.

3. **Komunikacja Niskolatencyjna (Streaming & Telemetry)**
   - Transmisja wideo H.264 z modułu kamery IMX219 za pośrednictwem natywnego potoku **MediaMTX** (WebRTC/WHEP oraz RTSP).
   - Niskopoziomowy protokół fragmentacji pakietów map i telemetrii (**Chunking**) zapobiegający problemom z limitem MTU (maksymalnie 1100 bajtów na pakiet w celu uniknięcia fragmentacji IP).
   - Wsparcie dla standardu **MAVLink** do integracji z zewnętrznymi autopilotami/kontrolerami lotu.

4. **Nawigacja Autonomiczna i SLAM**
   - **CostmapManager**: Zarządzanie siatką zajętości (occupancy grid) w czasie rzeczywistym na podstawie odczytów LiDAR.
   - **Global Planner**: Wyznaczanie optymalnej ścieżki za pomocą algorytmu **A***.
   - **Local & Reactive Planner**: Bezpieczne omijanie przeszkód i podążanie za wyznaczoną ścieżką przy użyciu algorytmu **Pure Pursuit**.
   - **Maszyna Stanów (State Machine)** i **Safety Supervisor**: Niezależny strażnik bezpieczeństwa monitorujący sygnał życia (heartbeat), przeciążenia IMU oraz odległość od przeszkód (Failsafe z automatycznym zatrzymaniem pojazdu).

---

## 📂 Struktura Projektu (`rpi_project_source`)

```bash
rpi_project_source/
├── core/                        # Rdzeń systemu operacyjnego RPi
│   ├── main_service.py          # Główna usługa zarządzająca cyklem życia aplikacji
│   ├── supervisor.py            # Nadzór nad procesami pokładowymi i wątkami
│   ├── safety_supervisor.py     # Hard Safety Rules, obsługa Failsafe i czujników zderzeniowych
│   ├── webrtc_manager.py        # Mostek WebRTC (WHEP) do wysyłania wideo i odbierania komend
│   ├── chunking.py              # Dzielenie dużych pakietów map i SLAM (poniżej MTU 1100)
│   ├── crsf_parser.py           # Odczyt danych z aparatury sterującej RC
│   └── mavlink_service.py       # Usługa komunikacji MAVLink
│
├── modules/                     # Sterowniki peryferiów i detekcja AI
│   ├── ai_manager.py            # Zarządzanie wnioskowaniem Hailo-8/8L i ładowaniem modeli .hef
│   ├── camera_manager.py        # Klient RTSP odbierający strumień wideo z MediaMTX
│   ├── pca9685.py               # Sterownik PWM I2C dla serwomechanizmów i silnika
│   ├── gps.py                   # Obsługa GPS LC29H oraz klienta poprawek RTK (NTRIP)
│   │
│   └── planners/                # Podsystem Planowania i Autonomii
│       ├── costmap_manager.py   # Tworzenie siatki zajętości i transformata odległości (Distance Transform)
│       ├── astar_planner.py     # Globalny planer trasy na mapie 2D
│       ├── pure_pursuit_planner.py # Podążanie za ścieżką z dynamicznym lookahead
│       ├── reactive_planner.py  # Unikanie kolizji w bliskim zasięgu
│       └── local_planner.py     # Fasada integrująca sensory z planerami
│
├── deployment/                  # Skrypty startowe, pliki systemd i konfiguracja Docker
└── tests/                       # Testy jednostkowe i integracyjne (pytest)
```

---

## 🛠️ Uruchomienie i Wdrożenie (Docker)

Oprogramowanie jest w pełni skonteneryzowane, co zapewnia powtarzalność środowiska na Raspberry Pi w trybie *headless* (bez serwera X11).

### Wymagania wstępne:
- Zainstalowany Docker oraz Docker Compose na Raspberry Pi 5.
- Zainstalowane sterowniki koprocesora Hailo RT (jeśli używany jest akcelerator NPU).
- Zainstalowany i uruchomiony serwer **MediaMTX** na systemie operacyjnym RPi.

### Szybki start:

1. **Konfiguracja**
   Wszystkie ustawienia (IP stacji bazowej PC, limity PWM, parametry Pure Pursuit, logowanie NTRIP) znajdują się w pliku `rpi_project_source/config.json`. Przed uruchomieniem upewnij się, że plik ma poprawną strukturę JSON.

2. **Budowanie i uruchamianie kontenera**
   ```bash
   cd rpi_project_source
   # Zbuduj obraz Docker
   docker-compose build
   # Uruchom usługi w tle
   docker-compose up -d
   ```

3. **Sprawdzanie stanu aplikacji**
   ```bash
   docker-compose logs -f
   ```

---

## 🔌 Połączenia Sprzętowe i Schemat Kabli

Aby poprawnie skonfigurować fizyczny pojazd, podłącz czujniki, kontrolery i urządzenia peryferyjne do złącza GPIO Raspberry Pi 5 zgodnie z poniższym schematem:

### 1. Połączenia magistrali I2C (PCA9685 & IMU GY-87/BMX160)
Sterownik PWM oraz IMU dzielą wspólnie magistralę I2C (Piny 3 i 5).
| Urządzenie | Pin urządzenia | Pin Raspberry Pi 5 / Nazwa | Opis |
|---|---|---|---|
| **PCA9685 (PWM)** | VCC | Pin 1 (3.3V) | Zasilanie układu logicznego |
| **PCA9685 (PWM)** | GND | Pin 9 (GND) | Masa układu logicznego |
| **PCA9685 (PWM)** | SDA | Pin 3 (GPIO 2 / SDA) | Linia danych |
| **PCA9685 (PWM)** | SCL | Pin 5 (GPIO 3 / SCL) | Linia zegarowa |
| **GY-87 IMU** | VCC | Pin 17 (3.3V) | Zasilanie czujnika |
| **GY-87 IMU** | GND | Pin 25 (GND) | Masa czujnika |
| **GY-87 IMU** | SDA | Pin 3 (GPIO 2 / SDA) | Współdzielona linia danych |
| **GY-87 IMU** | SCL | Pin 5 (GPIO 3 / SCL) | Współdzielona linia zegarowa |

*Uwaga: Podłącz serwo skrętu do kanału 0, a regulator ESC (silnik napędowy) do kanału 1 sterownika PCA9685.*

### 2. Połączenia szeregowe i USB (GPS, LiDAR, Odbiornik RC/MAVLink)
| Urządzenie | Pin / Port urządzenia | Pin / Port Raspberry Pi 5 | Port systemowy | Opis |
|---|---|---|---|---|
| **GPS LC29H** | TX | Pin 10 (GPIO 15 / RXD0) | `/dev/ttyAMA0` (UART0) | Odbiór telemetrii GPS (RX) |
| **GPS LC29H** | RX | Pin 8 (GPIO 14 / TXD0) | `/dev/ttyAMA0` (UART0) | Wysyłanie konfiguracji GPS (TX) |
| **CRSF / MAVLink** | TX | Pin 21 (GPIO 9 / RXD3) | `/dev/ttyAMA3` (UART3) | Odbiór sterowania i telemetrii (RX) |
| **CRSF / MAVLink** | RX | Pin 24 (GPIO 8 / TXD3) | `/dev/ttyAMA3` (UART3) | Wysyłanie telemetrii (TX) |
| **LiDAR LD08** | Złącze USB | Port USB 2.0 / 3.0 | `/dev/rcsim/lidar` | Podpięty przez przejściówkę USB-to-UART |

*Zawsze upewnij się, że ścieżki portów szeregowych w pliku `config.json` odpowiadają fizycznej konfiguracji sprzętowej.*

---

## 🔌 Połączenie i Weryfikacja Statusu Działania

### 1. Połączenie z Raspberry Pi (przez Terminal/CMD)
Aby zalogować się do Raspberry Pi z komputera PC za pomocą Wiersza Poleceń (Windows) lub Terminala (Linux/macOS):
```bash
# Połącz się za pomocą klienta SSH (zastąp 'pi' i IP swoimi danymi)
ssh pi@<IP_RASPBERRY_PI>
```
Jeśli używasz **Tailscale VPN**, zastąp `<IP_RASPBERRY_PI>` adresem IP Tailscale przypisanym do RPi (np. `100.x.x.x`).

### 2. Sprawdzanie czy program działa prawidłowo
Po zalogowaniu się na urządzenie uruchom poniższe polecenia, aby zweryfikować stan aplikacji:

- **Sprawdzenie czy kontener Docker działa:**
  ```bash
  docker ps
  # Powinieneś zobaczyć uruchomiony kontener o nazwie "rcsim_industrial"
  ```
- **Podgląd logów aplikacji w czasie rzeczywistym:**
  ```bash
  docker logs -f rcsim_industrial
  # Szukaj komunikatów typu "All checks passed. Starting supervisor..." oraz regularnych odczytów telemetrii/sensorów.
  ```
- **Weryfikacja serwera wideo MediaMTX:**
  Upewnij się, że serwer mediów odpowiedzialny za RTSP/WebRTC działa poprawnie:
  ```bash
  sudo systemctl status mediamtx
  # Sprawdzenie aktywnych ścieżek strumienia (powinna pokazać się camera_ai):
  curl http://localhost:9997/v3/paths/list | jq
  ```
- **Dostęp do zasobów sprzętowych (I2C/Serial/NPU):**
  Sprawdź czy kontener ma dostęp do magistral i koprocesora:
  ```bash
  # Czy system widzi akcelerator Hailo NPU:
  hailortcli fw-control identify
  # Czy magistrala I2C widzi kontroler PWM PCA9685 (powinien być na adresie 0x40):
  i2cdetect -y 1
  ```

---

## ⚠️ Standardy Jakości i Bezpieczeństwa (dla Deweloperów)

Wszyscy kontrybutorzy rozwijający ten moduł muszą bezwzględnie przestrzegać poniższych reguł bezpieczeństwa sprzętowego i programistycznego:

1. **Zabezpieczenia Hard-Safety:**
   Nigdy nie modyfikuj ani nie usuwaj logiki bezpieczeństwa w `safety_supervisor.py`. Watchdog, reakcja na brak sygnału (Heartbeat Failsafe) oraz natychmiastowe zatrzymanie pojazdu (Emergency Stop) chronią fizyczny sprzęt przed zniszczeniem.
2. **Limit MTU:**
   Przesyłanie pakietów telemetrii i map powyżej 1100 bajtów doprowadzi do fragmentacji IP i zerwania transmisji wideo. Używaj wyłącznie klasy `chunking.py` przy przesyłaniu dużych struktur danych.
3. **Prawidłowe Zwalnianie Zasobów:**
   Wątki, sockety UDP, połączenia I2C oraz uchwyty do modeli Hailo NPU muszą być bezpiecznie zamykane w metodach `cleanup()` / `stop()`.
4. **Brak Sekretów:**
   Nigdy nie umieszczaj haseł ani rzeczywistych kluczy w plikach konfiguracyjnych `config.json`. Do celów produkcyjnych używaj bezpiecznych zmiennych środowiskowych.
