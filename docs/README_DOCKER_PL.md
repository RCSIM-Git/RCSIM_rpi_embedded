# Instrukcja Wdrożenia Docker na Raspberry Pi (RCSIM)

Ten dokument opisuje, jak zbudować i uruchomić aplikację RCSIM na Raspberry Pi przy użyciu Dockera.

## Wymagania

Na Raspberry Pi muszą być zainstalowane:
- **Docker**: `curl -fsSL https://get.docker.com -o get-docker.sh && sh get-docker.sh`
- **Docker Compose**: Zazwyczaj jest wtyczką do Dockera (`docker compose`).

## Struktura Katalogów

Upewnij się, że masz następującą strukturę na RPi (katalog `rpi_project_source`):

```text
rpi_project_source/
├── docker-compose.yml
├── Dockerfile
├── requirements.txt
├── core/
│   ├── supervisor.py
│   └── ...
└── modules/
    └── ...
```

## Uruchomienie

1. **Przejdź do katalogu**:
   ```bash
   cd rpi_project_source
   ```

2. **Zbuduj i uruchom kontenery**:
   Użyj polecenia `docker compose up` z flagą `--build`, aby wymusić przebudowanie obrazu po zmianach w kodzie.
   ```bash
   docker compose up --build -d
   ```
   - `-d`: Uruchamia w tle (detached setup).

3. **Podgląd logów**:
   ```bash
   docker compose logs -f
   ```

4. **Zatrzymanie**:
   ```bash
   docker compose down
   ```

## Notatki Techniczne

- **Uprawnienia Sprzętowe**: Kontener działa w trybie `privileged: true` i `network_mode: "host"`, co jest wymagane do dostępu do GPIO, I2C, kamery i portów szeregowych.
- **Audio**: Zainstalowano `portaudio19-dev` oraz `pulse/alsa` libs w obrazie, aby obsłużyć mikrofon/głośnik. Upewnij się, że na hoście (RPi) audio nie jest zablokowane przez inny proces.
- **WebRTC**: Porty UDP są dynamiczne, ale dzięki `network_mode: "host"` nie trzeba ich ręcznie mapować.

## Rozwiązywanie Problemów

- **Błąd "ModuleNotFoundError"**: Upewnij się, że skopiowałeś wszystkie pliki `core` z PC do `rpi_project_source/core`.
- **Błąd Audio**: Sprawdź, czy użytkownik na RPi należy do grupy `audio`.
