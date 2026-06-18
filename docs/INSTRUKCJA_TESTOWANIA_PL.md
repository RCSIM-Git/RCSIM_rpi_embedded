# Instrukcja Testowania - Naprawiony Deployment

## Co zostało naprawione?

Wprowadzono **3 kluczowe poprawki** diagnostyczne, które pomogą zidentyfikować przyczynę błędu "DEPLOYMENT FAILED (Code: 1)":

### 1️⃣ Ulepszony entrypoint.sh
- Dodano testy weryfikacyjne przed startem aplikacji
- Sprawdzanie czy PySide6 importuje się poprawnie
- Czytelne komunikaty błędów

### 2️⃣ Weryfikacja w Dockerfile
- Build zatrzyma się jeśli PySide6 nie zainstaluje się poprawnie
- Fail-fast approach - wcześniejsze wykrycie problemów

### 3️⃣ Lepsza diagnostyka w deployment_logic.py
- Automatyczne pobieranie logów kontenera przy błędzie
- Sprawdzanie czy kontener rzeczywiście działa

---

## Jak Przetestować?

### Krok 1: Uruchom RCSIM Deployment Tool
```bash
cd RCSIM_deployment_tool/RCSIM_deployment_tool
python RCsimRPi5deploymentapp.py
```

### Krok 2: Wprowadź Dane i Deploy
1. **Tailscale IP RPi:** (np. 100.x.x.x)
2. **SSH User:** (np. pi)
3. **SSH Password:** (hasło do RPi)
4. **Kliknij:** "Deploy to Raspberry Pi"

### Krok 3: Obserwuj Logi
Teraz zobaczysz **dużo więcej informacji**:

#### ✅ Scenariusz Sukcesu:
```
[ENTRYPOINT] Starting RCSIM Container...
[ENTRYPOINT] Python version: Python 3.11.x
[ENTRYPOINT] Testing Python imports...
✓ PySide6 OK
[ENTRYPOINT] All checks passed. Starting supervisor...
✓ Container is RUNNING. Streaming initial logs...
--- DEPLOYMENT COMPLETED SUCCESSFULLY! ---
```

#### ❌ Scenariusz Błędu - Otrzymasz Szczegóły:
```
[ENTRYPOINT] ERROR: PySide6 import failed!
ModuleNotFoundError: No module named 'PySide6.QtCore'
```
lub
```
✗ Container NOT running. Fetching error logs...
[szczegółowe logi z kontenera - ostatnie 50 linii]
ERROR: Container failed to start!
```

---

## Co Zrobić Dalej?

### A) Jeśli Deployment Zakończy Się Sukcesem ✅
Świetnie! System działa. Możesz:
- Uruchomić RCSIM PC App i połączyć się z robotem
- Sprawdzić telemetrię i video stream

### B) Jeśli Dostaniesz Błąd ❌
**Skopiuj CAŁY output z okna deployment (logi)** i:
1. Przeczytaj sekcję dotyczącą rozwiązywania problemów w pliku `docs/README_DOCKER.md`.
2. Lub prześlij logi do głównego programisty w celu dalszej diagnozy.

---

## Szybka Diagnostyka Manualna (SSH)

Jeśli chcesz zobaczyć co się dzieje na RPi:

```bash
# Połącz się z RPi
ssh twoj_user@rpi_tailscale_ip

# Sprawdź status kontenera
docker ps -a

# Zobacz logi kontenera
docker logs rcsim_industrial

# Jeśli kontener nie działa - zobacz ostatnie 100 linii
docker logs --tail 100 rcsim_industrial
```

---

## FAQ - Najczęstsze Problemy

### Q: Build przeszedł, ale kontener nie startuje
**A:** To najprawdopodobniej problem z PySide6 lub brakiem bibliotek systemowych Qt.  
Zobacz sekcję diagnozy błędów w `docs/README_DOCKER.md`.

### Q: "ERROR: PySide6 import failed during build!"
**A:** PySide6 nie zainstalowało się z pip. Możliwe rozwiązania:
1. Sprawdź połączenie z `piwheels.org`
2. Rozważ instalację systemową: `python3-pyside6.qtcore`

### Q: "Container rcsim_industrial not running"
**A:** Kontener wystartował, ale od razu się wyłączył. Logi pokażą przyczynę (dostępne teraz automatycznie).

---

## Pliki Zmodyfikowane

    `RCSIM_RPI/RCSIM_rpi_embedded/rpi_project_source/entrypoint.sh` - diagnostyka startu  
    `RCSIM_RPI/RCSIM_rpi_embedded/rpi_project_source/Dockerfile` - weryfikacja PySide6  
    `RCSIM_deployment_tool/RCSIM_deployment_tool/core/deployment_logic.py` - pobieranie logów  
    `RCSIM_RPI/RCSIM_rpi_embedded/docs/README_DOCKER.md` - szczegółowa dokumentacja Docker  

---

## Kontakt / Dalsze Kroki

Po uruchomieniu deployment:
1. **Sukces?** → Świetnie! System gotowy do pracy
2. **Błąd?** → Skopiuj logi i sprawdź `docs/README_DOCKER.md`
3. **Dalej nie działa?** → Prześlij logi w celu diagnozy

**Powodzenia!** 🚀
