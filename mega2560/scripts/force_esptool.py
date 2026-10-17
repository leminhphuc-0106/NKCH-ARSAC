Import("env")

# The installed pioarduino platform resolves the tool-esptoolpy package
# directory as an executable on this Windows installation.  Use the verified
# console entry point installed in PlatformIO's virtual environment instead.
# UPLOADER matters for `upload`; OBJCOPY covers the image-build step.
ESPE = r"C:\Users\LENOVO\.platformio\penv\Scripts\esptool.exe"
env.Replace(OBJCOPY=ESPE)
env.Replace(UPLOADER=ESPE)

# esptool's progress bar uses U+2588, which the legacy cp1252 Windows console
# cannot encode; both the uploader's stdout and PlatformIO's echo of it then
# abort with UnicodeEncodeError.  Suppress the progress output.
env.Append(UPLOADERFLAGS=["--no-progress"])
