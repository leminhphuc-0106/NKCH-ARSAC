using System.Runtime.InteropServices;

namespace RobotHmi.Robot;

internal static class InverseKinematics
{
    [DllImport("RobotKinematicsNative.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern int RobotCalculateInverseKinematics(int x, int y, out float q1, out float q2);

    [DllImport("RobotKinematicsNative.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern int RobotCalculatePoint(int x, int y, out float q1, out float q2, out short count1, out short count2);

    [DllImport("RobotKinematicsNative.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern int RobotPlanLine(int x1, int y1, int x2, int y2, short[] counts, int maxSamples, out int sampleCount, out float duration);

    [DllImport("RobotKinematicsNative.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern int RobotPlanCircle(int cx, int cy, int radius, short[] counts, int maxSamples, out int sampleCount, out float duration);

    [DllImport("RobotKinematicsNative.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern int RobotComputeTorque(float q1, float q2, float dq1, float dq2, float ddq1, float ddq2, out float t1, out float t2);

    [DllImport("RobotKinematicsNative.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern void RobotQuintic(float u, float T, out float s, out float sd, out float sdd);

    // 2R inverse dynamics (N*m) for one reference sample.  q/dq/ddq in rad units.
    public static bool TryComputeTorque(double q1, double q2, double dq1, double dq2, double ddq1, double ddq2, out double t1, out double t2)
    {
        var ok = RobotComputeTorque((float)q1, (float)q2, (float)dq1, (float)dq2, (float)ddq1, (float)ddq2, out var n1, out var n2) != 0;
        t1 = n1; t2 = n2;
        return ok;
    }

    // Quintic time scaling and analytic derivatives (single C++ definition).
    public static (double S, double Sd, double Sdd) Quintic(double u, double T)
    {
        RobotQuintic((float)u, (float)T, out var s, out var sd, out var sdd);
        return (s, sd, sdd);
    }

    public static bool TryCalculate(int x, int y, out double q1, out double q2)
    {
        var result = RobotCalculateInverseKinematics(x, y, out var nativeQ1, out var nativeQ2) != 0;
        q1 = nativeQ1; q2 = nativeQ2;
        return result;
    }

    // Full native point calculation: existing C++ solution filtering plus the
    // shared C++ count conversion.  The returned counts are sent to the ESP32
    // unchanged.
    public static bool TryCalculatePoint(int x, int y, out double q1, out double q2, out short count1, out short count2)
    {
        var result = RobotCalculatePoint(x, y, out var nativeQ1, out var nativeQ2, out count1, out count2) != 0;
        q1 = nativeQ1; q2 = nativeQ2;
        return result;
    }

    // Line Motion: the shared C++ engine samples the Cartesian line, validates
    // every point with the existing RRplanar_in filter (continuous solution
    // branch) and converts to desired encoder counts.  The HMI only forwards
    // the returned counts over the existing serial path.
    public static bool TryPlanLine(int x1, int y1, int x2, int y2, out short[] counts, out int sampleCount, out double duration)
    {
        const int maxSamples = 4096;
        var buffer = new short[maxSamples * 2];
        var result = RobotPlanLine(x1, y1, x2, y2, buffer, maxSamples, out sampleCount, out var nativeDuration) != 0;
        counts = buffer;
        duration = nativeDuration;
        return result;
    }

    // Circle Motion: same shared C++ engine and count conversion as Line Motion.
    public static bool TryPlanCircle(int cx, int cy, int radius, out short[] counts, out int sampleCount, out double duration)
    {
        const int maxSamples = 4096;
        var buffer = new short[maxSamples * 2];
        var result = RobotPlanCircle(cx, cy, radius, buffer, maxSamples, out sampleCount, out var nativeDuration) != 0;
        counts = buffer;
        duration = nativeDuration;
        return result;
    }
}
