Aby stworzyć własny, spersonalizowany obraz systemu operacyjnego (**Custom OS Image / RCSIM OS**) na bazie kodu i konfiguracji znajdujących się w katalogu `C:\Users\Mateusz\Desktop\RCSIM27.04monacoSLAM\RCSIM_RPI`, musisz przejść przez proces przygotowania systemu na karcie SD, wdrożenia aplikacji kontenerowej RCSIM, a następnie zrzucenia i skompresowania tego systemu do pliku `.img`. 

Poniżej znajduje się kompletna instrukcja krok po kroku, jak to zrobić.

---

### KROK 1: Przygotowanie bazowego systemu operacyjnego
Aplikacja wdrożeniowa RCSIM dla Raspberry Pi 5 działa w oparciu o konteneryzację Docker na systemie operacyjnym Debian/Raspberry Pi OS (64-bit).

1. Pobierz i zainstaluj **Raspberry Pi Imager** na swoim komputerze PC.
2. Włóż czystą kartę SD do czytnika.
3. W Raspberry Pi Imager wybierz:
   * **Urządzenie:** Raspberry Pi 5
   * **System operacyjny:** Raspberry Pi OS (64-bit) (wersja Bookworm lub najnowsza kompatybilna z Pythonem 3.13)
4. Kliknij ikonę zębatki (Ustawienia zaawansowane) i skonfiguruj:
   * **Nazwę hosta** (np. `rcsim-node`)
   * **Włącz SSH** (wybierz uwierzytelnianie hasłem lub kluczem publicznym)
   * **Użytkownika i hasło** (np. użytkownik: `pi`)
   * **Konfigurację sieci bezprzewodowej WiFi** (SSID i hasło, aby urządzenie połączyło się z siecią po starcie)
5. Kliknij **ZAPISZ** (Write) i poczekaj na zakończenie procesu.

---

### KROK 2: Wdrożenie kodu RCSIM na Raspberry Pi
Teraz musisz wgrać i uruchomić kod ze swojego folderu deweloperskiego `RCSIM_RPI\RCSIM_rpi_embedded\rpi_project_source` na uruchomionym Raspberry Pi. Możesz to zrobić na dwa sposoby:

#### Opcja A: Automatycznie przez RCSIM Deployment Tool (Zalecana)
1. Uruchom narzędzie **RCSIM RPi5 Deployment Tool** na komputerze PC (skrypt `RCsimRPi5deploymentapp.py` lub skompilowany plik `.exe` z folderu `RCSIM_deployment_tool`).
2. W sekcji **Source Directory** wskaż ścieżkę do plików źródłowych RPi:
   `C:\Users\Mateusz\Desktop\RCSIM27.04monacoSLAM\RCSIM_RPI\RCSIM_rpi_embedded\rpi_project_source`
3. Podaj adres IP przydzielony Twojemu Raspberry Pi przez router oraz dane logowania SSH.
4. Kliknij **START DEPLOYMENT**. Narzędzie automatycznie:
   * Zainstaluje Dockera i Docker Compose na Raspberry Pi.
   * Skopiuje cały kod źródłowy, pliki usług i konfigurację sprzętową.
   * Uruchomi pobieranie i kompilację kontenera na bazie `Dockerfile`.
   * Skonfiguruje autostart kontenera przy starcie systemu.

#### Opcja B: Ręcznie przez Terminal (SSH)
1. Połącz się z Raspberry Pi przez SSH:
   ```bash
   ssh pi@<IP_RASPBERRY_PI>
   ```
2. Zainstaluj Dockera:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh && sh get-docker.sh
   sudo usermod -aG docker $USER
   ```
3. Skopiuj zawartość folderu `rpi_project_source` z PC do katalogu `~/rpi_project_source` na Raspberry Pi (np. za pomocą protokołu SFTP / programu FileZilla).
4. Przejdź do katalogu na Raspberry Pi i zbuduj kontenery:
   ```bash
   cd ~/rpi_project_source
   docker compose up --build -d
   ```

---

### KROK 3: Włączenie interfejsów sprzętowych (Hardware Overlays)
RCSIM wymaga bezpośredniego dostępu do magistral I2C, UART, kamery CSI oraz akceleratora Hailo-8. Upewnij się, że w pliku `/boot/firmware/config.txt` na Raspberry Pi włączone są odpowiednie nakładki systemowe:
```ini
dtparam=i2c_arm=on
dtparam=uart0=on
dtoverlay=hailo-ctl
```

---

### KROK 4: Zrzucenie gotowego systemu do pliku `.img` (Klonowanie)
Gdy system na Raspberry Pi działa stabilnie, kontenery uruchamiają się automatycznie przy starcie, a diagnostyka przechodzi pomyślnie, możesz zamienić tę kartę SD w swój własny instalacyjny plik OS `.img`.

1. Wyłącz bezpiecznie Raspberry Pi komendą:
   ```bash
   sudo poweroff
   ```
2. Wyjmij kartę SD z Raspberry Pi i włóż ją z powrotem do komputera PC.
3. Otwórz program **Win32 Disk Imager** (na systemie Windows):
   * W polu **Device** wybierz literę dysku swojej karty SD.
   * W polu **Image File** kliknij ikonę folderu i wybierz miejsce zapisu oraz nazwę pliku, np. `RCSIM_OS.img`.
   * Kliknij przycisk **Read** (Odczytaj). Program skopiuje całą strukturę karty SD do jednego pliku obrazu na Twoim komputerze.

---

### KROK 5: Optymalizacja rozmiaru obrazu (Opcjonalnie - PiShrink)
Obraz zrzucony z karty SD ma dokładnie taki sam rozmiar jak fizyczna karta (np. 32GB lub 64GB), nawet jeśli większość miejsca jest pusta. Aby zmniejszyć rozmiar pliku `.img` do absolutnego minimum (np. 4-6GB), użyj narzędzia **PiShrink** (wymaga systemu Linux lub WSL na Windows):

1. Zainstaluj PiShrink na maszynie Linux/WSL:
   ```bash
   wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
   chmod +x pishrink.sh
   sudo mv pishrink.sh /usr/local/bin/
   ```
2. Uruchom kompresję obrazu:
   ```bash
   sudo pishrink.sh -s RCSIM_OS.img
   ```
Narzędzie usunie niewykorzystane sektory i ustawi flagę automatycznego rozszerzania partycji (auto-resize) przy pierwszym uruchomieniu na nowym urządzeniu.

---

### KROK 6: Dystrybucja i Flashowanie
Twój własny, dedykowany system operacyjny jest gotowy. Teraz możesz wgrać go na dowolną kartę SD dokładnie według kroków, które opisałeś:
1. Otwórz **Raspberry Pi Imager**.
2. Jako system operacyjny wybierz **Custom Image** (Własny obraz) i wskaż stworzony plik `RCSIM_OS.img`.
3. Wybierz kartę SD i kliknij **WRITE**.
4. Po wgraniu dodaj swoje pliki konfiguracyjne do partycji rozruchowej w folderze `config`.




Open Raspberry Pi Imager

Regarding RPI model - just leave it

Choose Operating system - select last options - custom image

Select image of RCSIM OS (file ending .img you downloaded in step 2)

Select a disk to flush - your SD Card

Follow on screen instructions and flush SD card

Add your configuration files to the RCSIM_OS partition on SD card (config directory)

Turn on the device and wait 60sec - if you personalized OS image (Wifi, hostname, etc) you will need to power cycle the device to apply OS changes.

Turn on the device again and check what version you have via the menu Diagnostics → System info

Go RUN !!