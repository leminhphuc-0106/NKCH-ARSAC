using Microsoft.Win32;

namespace RobotHmi.Communication;

// Identifies the ESP32's USB-to-serial adapter (CH340/CH341 on the DOIT ESP32
// DevKit V1) from Windows device metadata, so the HMI can select the real board
// instead of the first enumerated port (which may be Bluetooth or anything
// else).  Uses the device registry (no extra dependencies).
internal static class SerialPortLocator
{
    // QinHeng USB-to-serial bridge hardware IDs used by the CH340 family.
    private static readonly string[] Ch34xHardwareIds =
    {
        "VID_1A86&PID_7523", // CH340
        "VID_1A86&PID_5523", // CH341
        "VID_1A86&PID_55D4"  // CH9102 (same driver family)
    };

    // COM ports currently bound to a CH34x USB-serial adapter.
    public static IReadOnlyList<string> FindEsp32Ports()
    {
        var ports = new List<string>();
        try
        {
            using var usb = Registry.LocalMachine.OpenSubKey(@"SYSTEM\CurrentControlSet\Enum\USB");
            if (usb is null) return ports;
            foreach (var vidPid in usb.GetSubKeyNames())
            {
                if (!Ch34xHardwareIds.Any(id => vidPid.StartsWith(id, StringComparison.OrdinalIgnoreCase)))
                    continue;
                using var device = usb.OpenSubKey(vidPid);
                if (device is null) continue;
                foreach (var instance in device.GetSubKeyNames())
                {
                    using var inst = device.OpenSubKey(instance);
                    using var deviceParameters = inst?.OpenSubKey("Device Parameters");
                    if (deviceParameters?.GetValue("PortName") is string port && !string.IsNullOrWhiteSpace(port))
                        ports.Add(port);
                }
            }
        }
        catch
        {
            // Registry access failure is treated as "no device found"; the caller
            // falls back to explicit manual selection with a diagnostic.
        }
        return ports.Distinct(StringComparer.OrdinalIgnoreCase)
                    .OrderBy(p => p, StringComparer.OrdinalIgnoreCase)
                    .ToArray();
    }
}
