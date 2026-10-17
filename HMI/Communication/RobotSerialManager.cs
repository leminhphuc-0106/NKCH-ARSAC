using System.IO.Ports;
using RobotHmi.Models;

namespace RobotHmi.Communication;

internal sealed class RobotSerialManager : IDisposable
{
    private readonly object gate = new();
    private readonly TelemetryParser parser = new();
    private readonly StatusParser statusParser = new();
    private SerialPort? port;

    public event EventHandler<TelemetryData>? TelemetryReceived;
    public event EventHandler<byte>? StatusReceived;
    public event EventHandler<string>? CommunicationError;
    public bool IsConnected { get { lock (gate) return port is { IsOpen: true }; } }

    public void Connect(string portName, int baudRate)
    {
        Disconnect();
        var next = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
        {
            ReadTimeout = 500,
            WriteTimeout = 500
        };
        next.DataReceived += OnDataReceived;
        try { next.Open(); }
        catch { next.DataReceived -= OnDataReceived; next.Dispose(); throw; }
        lock (gate) port = next;
    }

    // PC -> ESP32 command packet, exactly 11 bytes:
    //   [0..1]  header 0xAA 0x55
    //   [2..3]  joint 1 desired count (int16, MSB first)
    //   [4..5]  joint 2 desired count (int16, MSB first)
    //   [6..7]  joint 1 dynamics torque (int16, MSB first, 1 LSB = 0.001 N*m)
    //   [8..9]  joint 2 dynamics torque (int16, MSB first, 1 LSB = 0.001 N*m)
    //   [10]    checksum = (sum of bytes 0..9) mod 256
    public void SendJointCounts(short count1, short count2) => SendJointCountsWithTorque(count1, count2, 0, 0);

    public void SendJointCountsWithTorque(short count1, short count2, short torque1Lsb, short torque2Lsb)
    {
        var frame = new byte[11];
        frame[0] = 0xAA; frame[1] = 0x55;
        frame[2] = (byte)(count1 >> 8); frame[3] = (byte)count1;
        frame[4] = (byte)(count2 >> 8); frame[5] = (byte)count2;
        frame[6] = (byte)(torque1Lsb >> 8); frame[7] = (byte)torque1Lsb;
        frame[8] = (byte)(torque2Lsb >> 8); frame[9] = (byte)torque2Lsb;
        byte sum = 0;
        for (var i = 0; i < 10; i++) sum = unchecked((byte)(sum + frame[i]));
        frame[10] = sum;
        lock (gate)
        {
            if (port is not { IsOpen: true }) throw new InvalidOperationException("Robot is not connected.");
            port.Write(frame, 0, frame.Length);
        }
    }

    // Control frame (distinct 2nd header byte 0x56): AA 56 CMD 00 00 00 CMD
    //   CMD 0x01 = STOP (emergency stop), 0x02 = HOME (homing)
    public void SendControl(byte command)
    {
        var frame = new byte[] { 0xAA, 0x56, command, 0x00, 0x00, 0x00, command };
        lock (gate)
        {
            if (port is not { IsOpen: true }) throw new InvalidOperationException("Robot is not connected.");
            port.Write(frame, 0, frame.Length);
        }
    }

    // High-level trajectory command line parsed by the ESP32 with the existing
    // MControl command parser, e.g. "LINE x1 y1 z1 TO x2 y2 z2" or
    // "CIRCLE x y z r".  The ESP32 generates and executes the trajectory.
    public void SendText(string command)
    {
        var frame = System.Text.Encoding.ASCII.GetBytes(command + "\n");
        lock (gate)
        {
            if (port is not { IsOpen: true }) throw new InvalidOperationException("Robot is not connected.");
            port.Write(frame, 0, frame.Length);
        }
    }

    private void OnDataReceived(object sender, SerialDataReceivedEventArgs args)
    {
        try
        {
            SerialPort? source;
            lock (gate) source = port;
            if (source is not { IsOpen: true }) return;
            var available = source.BytesToRead;
            if (available <= 0) return;
            var bytes = new byte[available];
            var read = source.Read(bytes, 0, bytes.Length);
            for (var i = 0; i < read; i++)
            {
                if (parser.Push(bytes[i], out var telemetry)) TelemetryReceived?.Invoke(this, telemetry);
                if (statusParser.Push(bytes[i], out var status)) StatusReceived?.Invoke(this, status);
            }
        }
        catch (Exception ex) { CommunicationError?.Invoke(this, ex.Message); }
    }

    public void Disconnect()
    {
        SerialPort? old;
        lock (gate) { old = port; port = null; }
        if (old is null) return;
        old.DataReceived -= OnDataReceived;
        try { if (old.IsOpen) old.Close(); } finally { old.Dispose(); }
    }

    public void Dispose() => Disconnect();
}
