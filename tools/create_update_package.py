import os
import zipfile

SOURCE_DIR = "rpi_project_source"
OUTPUT_FILENAME = "RCSIM_RPi_Update.zip"
EXCLUDES = {
    "__pycache__",
    ".git",
    ".idea",
    ".mypy_cache",
    ".pytest_cache",
    "debug_logs",
    "venv",
    ".env",
}


def zip_directory(source_dir, output_filename):
    with zipfile.ZipFile(output_filename, "w", zipfile.ZIP_DEFLATED) as zipf:
        for root, dirs, files in os.walk(source_dir):
            # Exclude directories
            dirs[:] = [d for d in dirs if d not in EXCLUDES]

            for file in files:
                if file in EXCLUDES or file.endswith(".pyc") or file.endswith(".log"):
                    continue

                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, os.path.dirname(source_dir))

                print(f"Adding {arcname}...")
                zipf.write(file_path, arcname)


if __name__ == "__main__":
    if os.path.exists(OUTPUT_FILENAME):
        os.remove(OUTPUT_FILENAME)

    print(f"Zipping {SOURCE_DIR} into {OUTPUT_FILENAME}...")
    zip_directory(SOURCE_DIR, OUTPUT_FILENAME)
    print("Done!")
