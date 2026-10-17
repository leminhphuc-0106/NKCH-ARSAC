using System.Globalization;
using System.Text.RegularExpressions;

namespace RobotHmi.Communication;

// Keeps the HMI baudrate, the ESP32 firmware source and the PlatformIO monitor
// speed in step.  Only the baudrate number is rewritten in each file; every
// other byte (comments, formatting, unrelated settings) is preserved.
//
// Note: changing main.cpp only updates the firmware SOURCE.  The physical ESP32
// keeps running its old firmware until it is rebuilt and uploaded, which the
// caller must communicate to the user.
internal static class BaudConfigSync
{
    public static string? ProjectRoot => FindProjectRoot(AppContext.BaseDirectory);

    // Reads the currently configured firmware baudrate, preferring the value
    // actually used by main.cpp (SERIAL_BAUD) and falling back to the PlatformIO
    // monitor speed.
    public static int? ReadConfiguredBaud(string? root)
    {
        if (string.IsNullOrEmpty(root)) return null;
        if (TryReadMacroValue(Path.Combine(root, "mega2560", "src", "main.cpp"), out var baud))
            return baud;
        if (TryReadMonitorSpeed(Path.Combine(root, "mega2560", "platformio.ini"), out baud))
            return baud;
        return null;
    }

    public static bool TrySync(int baud, out string message, out bool changed) => TrySyncAt(ProjectRoot, baud, out message, out changed);

    public static bool TrySyncAt(string? root, int baud, out string message, out bool changed)
    {
        message = string.Empty;
        changed = false;
        if (baud <= 0)
        {
            message = "Unsupported baudrate.";
            return false;
        }
        if (string.IsNullOrEmpty(root))
        {
            message = "Project root not found; firmware configuration was not changed.";
            return false;
        }

        var mainCpp = Path.Combine(root, "mega2560", "src", "main.cpp");
        var platformio = Path.Combine(root, "mega2560", "platformio.ini");
        if (!File.Exists(mainCpp) || !File.Exists(platformio))
        {
            message = "Firmware configuration files were not found; nothing was changed.";
            return false;
        }

        try
        {
            changed = RewriteBaud(mainCpp, new Regex(@"(#define\s+SERIAL_BAUD\s+)\d+"), baud, "SERIAL_BAUD");
            changed |= RewriteBaud(platformio, new Regex(@"(?m)^(\s*monitor_speed\s*=\s*)\d+"), baud, "monitor_speed");

            message = changed
                ? $"Baudrate configuration updated to {baud}.\r\n"
                  + $"HMI serial setting: {baud}\r\n"
                  + $"Firmware source: {baud}\r\n"
                  + $"PlatformIO monitor: {baud}\r\n\r\n"
                  + "Please rebuild and upload the ESP32 firmware before reconnecting."
                : $"Baudrate already {baud}; no configuration changes were made.";
            return true;
        }
        catch (Exception ex)
        {
            message = "Failed to update baudrate configuration: " + ex.Message;
            return false;
        }
    }

    private static string? FindProjectRoot(string start)
    {
        var directory = new DirectoryInfo(start);
        for (var i = 0; i < 12 && directory is not null; i++, directory = directory.Parent)
        {
            if (File.Exists(Path.Combine(directory.FullName, "mega2560", "src", "main.cpp"))
                && File.Exists(Path.Combine(directory.FullName, "mega2560", "platformio.ini")))
                return directory.FullName;
        }
        return null;
    }

    private static bool TryReadMacroValue(string path, out int baud)
    {
        baud = 0;
        if (!File.Exists(path)) return false;
        var match = Regex.Match(File.ReadAllText(path), @"#define\s+SERIAL_BAUD\s+(\d+)");
        return match.Success && int.TryParse(match.Groups[1].Value, NumberStyles.Integer, CultureInfo.InvariantCulture, out baud);
    }

    private static bool TryReadMonitorSpeed(string path, out int baud)
    {
        baud = 0;
        if (!File.Exists(path)) return false;
        var match = Regex.Match(File.ReadAllText(path), @"(?m)^\s*monitor_speed\s*=\s*(\d+)");
        return match.Success && int.TryParse(match.Groups[1].Value, NumberStyles.Integer, CultureInfo.InvariantCulture, out baud);
    }

    // Replaces the single baudrate capture group in the file.  Returns true only
    // when the file actually changed, so an unchanged selection is a no-op.
    private static bool RewriteBaud(string path, Regex regex, int baud, string label)
    {
        var text = File.ReadAllText(path);
        var match = regex.Match(text);
        if (!match.Success)
            throw new InvalidOperationException($"{label} was not found in {Path.GetFileName(path)}");

        var updated = regex.Replace(text, "${1}" + baud.ToString(CultureInfo.InvariantCulture), 1);
        if (updated == text)
            return false;

        WriteAtomically(path, updated);
        return true;
    }

    private static void WriteAtomically(string path, string content)
    {
        var directory = Path.GetDirectoryName(path)!;
        var temp = Path.Combine(directory, Path.GetRandomFileName() + ".tmp");
        File.WriteAllText(temp, content);
        try
        {
            File.Replace(temp, path, null);
        }
        catch
        {
            // Fall back to an overwrite copy if Replace is unsupported here.
            File.Copy(temp, path, true);
        }
        finally
        {
            if (File.Exists(temp)) File.Delete(temp);
        }
    }
}
