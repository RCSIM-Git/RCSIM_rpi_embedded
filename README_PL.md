# 🍓 Moduł RCSIMDEPLOY (Raspberry Pi Deployment)

Ten katalog to trzon oprogramowania oraz infrastruktury instalacyjnej docelowo wdrażanej na urządzeniu pokładowym w robocie (Raspberry Pi 5). 

## Odpowiedzialność (Zadania Główne)
1. Dostarczenie kodu komunikacji (I2C) ze sterownikami sprzętowymi (np. PCA9685).
2. Obsługa akceleracji sprzętowej **Hailo-8** z wykorzystaniem pakietów `.hef` dla modelu.
3. Przepływ WebRTC (strumieniowanie wideo h264 z modułu kamery IMX219) wraz z protokołem fragmentacji UDP.
4. Środowisko zarządzane jest skryptami Docker (`docker-compose.yml`, `Dockerfile`) - projekt wymaga kompatybilności z przestrzeniami bez środowisk okienkowych (headless).

## Struktura:
* `rpi_project_source/core/`: Rdzenna logika RPi (w tym `chunking` i złącze WebRTC).
* `rpi_project_source/modules/`: Konfiguracja kontrolerów obwodowych i detekcji obrazu (Hailo).
* `rpi_project_source/deployment/`: Helpery i logiki startowe dockera.
* `rpi_project_source/tests/`: Testy jednostkowe dla oprogramowania roboczej maliny.
