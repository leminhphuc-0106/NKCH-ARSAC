using System.Globalization;
using RobotHmi.Models;

namespace RobotHmi.Logging;

internal enum TrajectoryType
{
    Point,
    Line,
    Circle
}

internal sealed class DataLogger
{
    private readonly string projectRoot;
    private readonly string dataRoot;
    private readonly object gate = new();
    private string? customDataRoot;
    private string path;
    private TrajectoryType activeTrajectory = TrajectoryType.Point;

    // The ESP32 samples and transmits one telemetry frame every
    // TELEMETRY_PERIOD_MS (10 ms), so the CSV time base is advanced by this fixed
    // interval per logged sample.  Using the firmware's sampling interval instead
    // of the HMI arrival clock keeps each row 10 ms apart: the Windows serial
    // driver delivers telemetry in buffered bursts, so arrival timestamps
    // collapse many frames onto the same ~1 ms instant.
    private const double SampleIntervalSeconds = 0.010;
    private long sampleCount;

    public DataLogger()
    {
        var assemblyLocation = typeof(DataLogger).Assembly.Location;
        var assemblyDirectory = Path.GetDirectoryName(assemblyLocation)!;
        projectRoot = Path.GetFullPath(Path.Combine(assemblyDirectory, "..", "..", "..", "..", ".."));
        dataRoot = Path.Combine(projectRoot, "Data");
        foreach (var trajectoryType in Enum.GetValues<TrajectoryType>())
            Directory.CreateDirectory(Path.Combine(dataRoot, trajectoryType.ToString()));
        path = BuildPath(activeTrajectory);
        EnsureHeader(path);
    }

    public string DirectoryPath => Path.GetDirectoryName(path)!;
    public string FilePath => path;
    public bool DirectoryExists => Directory.Exists(Path.GetDirectoryName(path));
    public bool FileExists => File.Exists(path);

    public void SetTrajectoryType(TrajectoryType trajectoryType)
    {
        lock (gate)
        {
            activeTrajectory = trajectoryType;
            sampleCount = 0;
            path = BuildExecutionPath(trajectoryType);
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            ResetHeader(path);
        }
    }

    public bool TrySetCustomDataRoot(string? value, out string? error)
    {
        error = null;
        lock (gate)
        {
            if (string.IsNullOrWhiteSpace(value))
            {
                customDataRoot = null;
                path = BuildPath(activeTrajectory);
                Directory.CreateDirectory(Path.GetDirectoryName(path)!);
                EnsureHeader(path);
                return true;
            }

            try
            {
                var candidate = value.Trim();
                if (candidate.Length == 0)
                {
                    customDataRoot = null;
                    path = BuildPath(activeTrajectory);
                    Directory.CreateDirectory(Path.GetDirectoryName(path)!);
                    EnsureHeader(path);
                    return true;
                }

                var root = Path.GetFullPath(candidate);
                if (File.Exists(root) && !string.IsNullOrEmpty(Path.GetDirectoryName(root)))
                    root = Path.GetDirectoryName(root)!;

                if (!Directory.Exists(root))
                    Directory.CreateDirectory(root);

                customDataRoot = root;
                path = BuildPath(activeTrajectory);
                Directory.CreateDirectory(Path.GetDirectoryName(path)!);
                EnsureHeader(path);
                return true;
            }
            catch (Exception ex)
            {
                error = "Invalid CSV folder path: " + ex.Message;
                return false;
            }
        }
    }

    public string[] GetCsvFiles()
    {
        var root = customDataRoot ?? dataRoot;
        if (!Directory.Exists(root)) return Array.Empty<string>();
        return Directory.GetFiles(root, "*.csv", SearchOption.AllDirectories)
            .OrderBy(file => file, StringComparer.OrdinalIgnoreCase)
            .ToArray();
    }

    public bool IsWritable()
    {
        try
        {
            using var stream = new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.ReadWrite);
            return true;
        }
        catch
        {
            return false;
        }
    }

    // referenceAndTorque carries the PC-computed reference derivatives and
    // dynamics feedforward for the reference sample in effect for this row:
    // (refDq1, refDq2, refDdq1, refDdq2, refTau1, refTau2, haveTorque).
    // The PID correction, total commanded torque and PWM are PC-side ESTIMATES
    // computed for logging only (the ESP32 does not return them): pidPwm1/2 (+
    // havePwm), tauPid1/2, tauCmd1/2 (+ havePid).  The dynamics torque is the PC
    // model feedforward, not a measured torque.
    public void Log(TelemetryData data, double q1, double q2, double x, double y, double? targetQ1, double? targetQ2,
                    double refDq1, double refDq2, double refDdq1, double refDdq2, double refTau1, double refTau2, bool haveTorque,
                    double pidPwm1, double pidPwm2, bool havePwm,
                    double tauPid1, double tauPid2, double tauCmd1, double tauCmd2, bool havePid)
    {
        var e1 = targetQ1.HasValue ? targetQ1.Value - q1 : (double?)null;
        var e2 = targetQ2.HasValue ? targetQ2.Value - q2 : (double?)null;
        var total = e1.HasValue && e2.HasValue ? Math.Sqrt(e1.Value * e1.Value + e2.Value * e2.Value) : (double?)null;
        string F(double? value) => value?.ToString("G17", CultureInfo.InvariantCulture) ?? "";
        var tau1 = haveTorque ? F(refTau1) : "";
        var tau2 = haveTorque ? F(refTau2) : "";
        lock (gate)
        {
            var elapsedSeconds = sampleCount * SampleIntervalSeconds;
            sampleCount++;
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            EnsureHeader(path);
            File.AppendAllText(path, string.Join(',', elapsedSeconds.ToString("F3", CultureInfo.InvariantCulture), data.Count1, data.Count2,
                F(q1), F(q2), F(x), F(y), F(targetQ1), F(targetQ2), F(e1), F(e2), F(total),
                F(refDq1), F(refDq2), F(refDdq1), F(refDdq2), tau1, tau2,
                F(havePid ? tauPid1 : (double?)null), F(havePid ? tauPid2 : (double?)null),
                F(havePid ? tauCmd1 : (double?)null), F(havePid ? tauCmd2 : (double?)null),
                F(havePwm ? pidPwm1 : (double?)null), F(havePwm ? pidPwm2 : (double?)null)) + Environment.NewLine);
        }
    }

    private string BuildPath(TrajectoryType trajectoryType)
    {
        var root = customDataRoot ?? dataRoot;
        return Path.Combine(root, trajectoryType.ToString(), $"{trajectoryType}.csv");
    }

    // One CSV file per execution: a timestamped file inside the existing
    // per-trajectory folder.  A numeric suffix guarantees uniqueness so an
    // earlier execution's file is never overwritten or appended to.
    private string BuildExecutionPath(TrajectoryType trajectoryType)
    {
        var root = customDataRoot ?? dataRoot;
        var folder = Path.Combine(root, trajectoryType.ToString());
        var stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture);
        var candidate = Path.Combine(folder, $"{trajectoryType}_{stamp}.csv");
        var suffix = 1;
        while (File.Exists(candidate))
            candidate = Path.Combine(folder, $"{trajectoryType}_{stamp}_{suffix++}.csv");
        return candidate;
    }

    private static string BuildHeader() => "time,Count1,Count2,Theta1Rad,Theta2Rad,XMetres,YMetres,TargetTheta1Rad,TargetTheta2Rad,Error1Rad,Error2Rad,TotalErrorRad,"
        + "RefVel1_Rads,RefVel2_Rads,RefAcc1_Rads2,RefAcc2_Rads2,TauDyn1_Nm,TauDyn2_Nm,TauPID1_Nm,TauPID2_Nm,TauCmd1_Nm,TauCmd2_Nm,Pwm1,Pwm2" + Environment.NewLine;

    private static void EnsureHeader(string targetPath)
    {
        if (!File.Exists(targetPath)) File.WriteAllText(targetPath, BuildHeader());
    }

    private static void ResetHeader(string targetPath)
    {
        File.WriteAllText(targetPath, BuildHeader());
    }
}
