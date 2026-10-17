using RobotHmi.Models;

namespace RobotHmi.Communication;

// Mirrors mega2560/src/main.cpp: 55 AA, two signed big-endian int16 counts,
// followed by the low byte of their four-byte sum.
internal sealed class TelemetryParser
{
    private readonly byte[] data = new byte[4];
    private int state;
    private int index;

    public bool Push(byte value, out TelemetryData telemetry)
    {
        telemetry = default;
        switch (state)
        {
            case 0: state = value == 0x55 ? 1 : 0; return false;
            case 1:
                state = value == 0xAA ? 2 : value == 0x55 ? 1 : 0;
                if (state == 2) index = 0;
                return false;
            case 2:
                data[index++] = value;
                if (index == data.Length) state = 3;
                return false;
            default:
                var checksum = unchecked((byte)(data[0] + data[1] + data[2] + data[3]));
                state = 0;
                if (value != checksum) return false;
                telemetry = new TelemetryData(
                    unchecked((short)((data[0] << 8) | data[1])),
                    unchecked((short)((data[2] << 8) | data[3])), DateTime.UtcNow);
                return true;
        }
    }
}

// Parses the ESP32 -> PC status frame
// (55 AB CODE 00 00 00 CHECKSUM) used to report whether a LINE/CIRCLE command
// was accepted, rejected or finished.  Telemetry frames (55 AA) are handled by
// TelemetryParser and are ignored here.
internal sealed class StatusParser
{
    private readonly byte[] data = new byte[4];
    private int state;
    private int index;

    public bool Push(byte value, out byte code)
    {
        code = 0;
        switch (state)
        {
            case 0: state = value == 0x55 ? 1 : 0; return false;
            case 1:
                state = value == 0xAB ? 2 : value == 0x55 ? 1 : 0;
                if (state == 2) index = 0;
                return false;
            case 2:
                data[index++] = value;
                if (index == data.Length) state = 3;
                return false;
            default:
                var checksum = unchecked((byte)(data[0] + data[1] + data[2] + data[3]));
                state = 0;
                if (value != checksum) return false;
                code = data[0];
                return true;
        }
    }
}
