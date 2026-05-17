import os
import zipfile

# This script creates a deployment package for the ROS2 unified system.
# It packages the content of 'ros2_deploy' and includes the 'ros2_ws' from the project root.

PROJECT_ROOT = ".."  # Assuming we are in RCSIMDEPLOY
SOURCE_DIR = "ros2_deploy"
ROS2_WS_ROOT = os.path.join(PROJECT_ROOT, "ros2_ws")
OUTPUT_FILENAME = "RCSIM_ROS2_Update.zip"

EXCLUDES = {
    "__pycache__",
    ".git",
    ".idea",
    ".mypy_cache",
    ".pytest_cache",
    "debug_logs",
    "venv",
    ".env",
    "build",
    "install",
    "log",
}


def zip_directory(source_dir, zipf, arcname_prefix=""):
    for root, dirs, files in os.walk(source_dir):
        # Exclude directories
        dirs[:] = [d for d in dirs if d not in EXCLUDES]

        for file in files:
            if file in EXCLUDES or file.endswith(".pyc") or file.endswith(".log"):
                continue

            file_path = os.path.join(root, file)
            # Calculate path inside zip
            rel_path = os.path.relpath(file_path, source_dir)
            arcname = os.path.join(arcname_prefix, rel_path)

            # print(f"Adding {arcname}...")
            zipf.write(file_path, arcname)


if __name__ == "__main__":
    if os.path.exists(OUTPUT_FILENAME):
        os.remove(OUTPUT_FILENAME)

    print(f"Creating ROS2 update package: {OUTPUT_FILENAME}...")

    with zipfile.ZipFile(OUTPUT_FILENAME, "w", zipfile.ZIP_DEFLATED) as zipf:
        # 1. Add everything from ros2_deploy folder
        print(f"Adding base deployment files from {SOURCE_DIR}...")
        zip_directory(SOURCE_DIR, zipf, arcname_prefix="ros2_deploy")

        # 2. Add the ros2_ws source code
        print(f"Adding ROS2 workspace from {ROS2_WS_ROOT}...")
        zip_directory(
            ROS2_WS_ROOT, zipf, arcname_prefix=os.path.join("ros2_deploy", "ros2_ws")
        )

    print("Package created successfully!")
