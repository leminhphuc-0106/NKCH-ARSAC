namespace RobotHmi.Robot;

internal static class RobotMath
{
    // Existing firmware/calculate.cpp defines encoder counts per complete 2π revolution.
    public const int CountsPerRevolution1 = 11880;
    public const int CountsPerRevolution2 = 1980;
    public const double Link1Metres = 0.20;
    public const double Link2Metres = 0.15;

    public static double CountToJoint1Angle(short count) => count * 2.0 * Math.PI / CountsPerRevolution1;
    public static double CountToJoint2Angle(short count) => count * 2.0 * Math.PI / CountsPerRevolution2;

    // Joint mechanical limits (see DATAA::RRplanar_in in Calculate_Lib.cpp):
    //   Joint 1: 0 <= q1 <= pi       -> counts [0, CountsPerRevolution1/2]
    //   Joint 2: -170deg <= q2 <= pi -> counts [-935, CountsPerRevolution2/2]
    // Both ranges are shorter than one full revolution, so each orientation has a
    // single admissible representation in the continuous encoder-count space.
    public const int MinCount1 = 0;
    public const int MaxCount1 = (CountsPerRevolution1 / 2) + 100;
    public const int MinCount2 = -1010;
    public const int MaxCount2 = (CountsPerRevolution2 / 2) + 20;

    // Shift a wrapped target count to the equivalent that is BOTH inside the
    // joint's allowed count range [minCount,maxCount] AND nearest the continuous
    // reference count (measured encoder).  Because the allowed range is shorter
    // than a revolution, at most one equivalent qualifies, so the commanded
    // physical joint position can never leave its limits by a whole revolution,
    // while the encoder measurement itself remains continuous.
    public static short UnwrapCountWithinLimits(short target, short reference, int countsPerRev, int minCount, int maxCount)
    {
        var best = (short)Math.Clamp((int)target, minCount, maxCount);
        var bestDistance = long.MaxValue;
        for (var k = -1; k <= 1; k++)
        {
            var candidate = (long)target + (long)k * countsPerRev;
            if (candidate < minCount || candidate > maxCount) continue;
            var distance = Math.Abs(candidate - reference);
            if (distance < bestDistance) { bestDistance = distance; best = (short)candidate; }
        }
        return best;
    }

    public static short UnwrapCount1(short target, short reference) => UnwrapCountWithinLimits(target, reference, CountsPerRevolution1, MinCount1, MaxCount1);
    public static short UnwrapCount2(short target, short reference) => UnwrapCountWithinLimits(target, reference, CountsPerRevolution2, MinCount2, MaxCount2);

    // True when a measured encoder position lies inside the permitted joint
    // range.  Used to refuse to start a trajectory from an out-of-range measured
    // pose (the joint passed a limit, or the homing/encoder zero is wrong).
    public static bool WithinJointLimits(short count1, short count2) =>
        count1 >= MinCount1 && count1 <= MaxCount1 && count2 >= MinCount2 && count2 <= MaxCount2;

    // Project initial condition, taken from the existing sources:
    // calculate.cpp sets currentQ1 = 0 and currentQ2 = PI, and the firmware
    // starts joint 2 at PWM_NUM2 / 2 counts (= CountsPerRevolution2 / 2, i.e.
    // q2 = PI).  The HMI must not re-derive this on its own.
    public const short InitialCount1 = 0;
    public const short InitialCount2 = CountsPerRevolution2 / 2;
    public static double InitialJoint1Angle => CountToJoint1Angle(InitialCount1);
    public static double InitialJoint2Angle => CountToJoint2Angle(InitialCount2);

    // Position of joint 1 (elbow) for the current q1, from the shared geometry.
    public static (double X, double Y) Joint1Position(double q1) =>
        (Link1Metres * Math.Cos(q1), Link1Metres * Math.Sin(q1));

    public static (double X, double Y) ForwardKinematics(double q1, double q2) =>
        (Link1Metres * Math.Cos(q1) + Link2Metres * Math.Cos(q1 + q2),
         Link1Metres * Math.Sin(q1) + Link2Metres * Math.Sin(q1 + q2));
}
