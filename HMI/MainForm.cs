using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using RobotHmi.Charts;
using RobotHmi.Communication;
using RobotHmi.Controls;
using RobotHmi.Logging;
using RobotHmi.Models;
using RobotHmi.Robot;

namespace RobotHmi;

internal sealed class MainForm : Form
{
    private static readonly Color Canvas = Color.FromArgb(13, 18, 24);
    private static readonly Color Surface = Color.FromArgb(22, 29, 38);
    private static readonly Color SurfaceRaised = Color.FromArgb(29, 38, 49);
    private static readonly Color Border = Color.FromArgb(50, 63, 78);
    private static readonly Color TextPrimary = Color.FromArgb(232, 238, 244);
    private static readonly Color TextMuted = Color.FromArgb(145, 160, 176);
    private static readonly Color Accent = Color.FromArgb(62, 190, 190);
    private static readonly Color Warning = Color.FromArgb(237, 174, 73);
    private static readonly Color Danger = Color.FromArgb(220, 92, 92);

    private readonly RobotSerialManager serial = new();
    private readonly DataLogger logger = new();
    private readonly ComboBox portBox = new() { DropDownStyle = ComboBoxStyle.DropDownList };
    private readonly ComboBox baudBox = new() { DropDownStyle = ComboBoxStyle.DropDownList, Width = 92 };
    private readonly Button connectButton = new() { Text = "CONNECT", AutoSize = true };
    private readonly Label stateLabel = new() { AutoSize = true, ForeColor = Color.White, Padding = new Padding(8, 5, 8, 5) };
    private readonly Label connectionDetail = new() { AutoSize = true, ForeColor = Color.FromArgb(145, 162, 180), Margin = new Padding(8, 8, 10, 0) };
    private readonly Label statusMessage = new() { AutoSize = true, ForeColor = Color.FromArgb(145, 162, 180), Margin = new Padding(18, 7, 0, 0) };
    private readonly System.Windows.Forms.Timer portTimer = new() { Interval = 1500 };
    private readonly Dictionary<string, Label> readouts = new();
    private readonly TextBox pointX = Input(); private readonly TextBox pointY = Input();
    private readonly TextBox lineX1 = Input(); private readonly TextBox lineY1 = Input(); private readonly TextBox lineX2 = Input(); private readonly TextBox lineY2 = Input();
    private readonly TextBox circleX = Input(); private readonly TextBox circleY = Input(); private readonly TextBox circleR = Input();
    private readonly RealtimeChart joint1Chart = new() { ChartTitle = "Joint 1 angle: target vs actual", Unit = "rad", MultiSeries = true };
    private readonly RealtimeChart joint2Chart = new() { ChartTitle = "Joint 2 angle: target vs actual", Unit = "rad", MultiSeries = true };
    private readonly RealtimeChart count1Chart = new() { ChartTitle = "Joint 1 encoder count", Unit = "count" };
    private readonly RealtimeChart count2Chart = new() { ChartTitle = "Joint 2 encoder count", Unit = "count" };
    private readonly RobotView robotView = new();
    private readonly ListBox csvFileList = new() { Dock = DockStyle.Fill, BackColor = SurfaceRaised, ForeColor = TextPrimary, BorderStyle = BorderStyle.FixedSingle, Margin = new Padding(4) };
    private readonly TextBox csvPathBox = new()
    {
        Dock = DockStyle.Fill,
        Height = 34,
        Margin = new Padding(0, 0, 0, 4),
        ForeColor = TextPrimary,
        BackColor = SurfaceRaised,
        BorderStyle = BorderStyle.FixedSingle,
        Font = new Font("Segoe UI", 9F),
        ShortcutsEnabled = true,
        ReadOnly = false,
        ScrollBars = ScrollBars.None,
        Multiline = false,
        AcceptsTab = false,
        TabStop = true
    };
    private string committedCsvPath = string.Empty;
    private readonly DateTime chartStart = DateTime.UtcNow;
    // The HMI selects the ESP32's CH340 USB-serial port from Windows device
    // metadata (SerialPortLocator) instead of a fixed COM number, which becomes
    // stale when Windows re-enumerates the device.  A stale value is never used
    // and an arbitrary port (e.g. Bluetooth) is never auto-selected.
    private string lastPortDiagnostic = "";
    private double? targetQ1, targetQ2;
    // Cartesian TARGET for Robot View, in metres, taken from the commanded
    // POINT input (not from actual telemetry).
    private (double X, double Y)? targetPoint;
    private string robotState = "DISCONNECTED";
    // HOME control state.  The operator now initiates HOME/STOP from the screen;
    // homingActive guards against overlapping HOME requests until the firmware
    // reports the outcome (55 AB 51/52/53).
    private Button? homeButton;
    private bool homingActive;
    private readonly List<string> csvFilePaths = new();
    // PC reference generation / transmission period (100 Hz).  Trajectory
    // progression is driven by the monotonic streamClock, so a late timer tick
    // still yields the correct time-based reference value; the timer only
    // triggers the send.  Kept separate from the ESP32 telemetry (10 ms) and
    // ESP32 PID (1 ms) periods.
    private const int ReferencePeriodMs = 10;
    // A point-to-point move is refused if encoder feedback is older than this.
    private const int TelemetryFreshnessMs = 200;
    // A commanded Cartesian path start (line start / circle entry) must agree
    // with the measured position within this tolerance before streaming begins:
    // the first streamed sample is the path start, so a mismatch would step the
    // robot there instead of following the path.
    private const double PathStartToleranceCm = 3.0;
    // CSV logging stops after the measured end-effector stays within
    // PathStartToleranceCm of the final reference for this many consecutive
    // telemetry frames (debounces a transient pass-through), or after this
    // bounded settle time if it never converges (so the file cannot grow
    // without bound).  Both reuse the existing project tolerances/intervals.
    private const int CompletionHoldSamples = 10;
    private const int SettleTimeoutMs = 3000;
    private readonly System.Windows.Forms.Timer streamTimer = new() { Interval = ReferencePeriodMs };
    private long lastTelemetryTicks;
    private readonly Stopwatch streamClock = new();
    private short[]? streamCounts;
    private int streamSampleCount;
    private double streamDuration = 1.0;
    private TrajectoryType streamTrajectory = TrajectoryType.Point;
    // The PC generates the time-varying reference (point, line, circle) and
    // streams successive desired-count samples to the ESP32, which only follows
    // the latest received reference.  The ESP32 does not interpolate.
    private readonly bool streamSendsCounts = true;

    // CSV logging lifecycle (Idle -> Logging -> Completed/Stopped).  Rows are
    // appended only while a trajectory execution is active.  The generator
    // emitting its final reference is NOT sufficient: logging continues until the
    // measured position settles on the final reference (or the bounded settle
    // time elapses), so the last samples are captured and idle rows are not.
    // All of these are touched only on the UI thread (telemetry is marshalled
    // through BeginInvoke), so no extra locking is required.
    private bool logActive;
    private bool streamGenerationDone;
    // Independent lifecycle flag for the REAL-TIME MONITORING angle charts.  The
    // charts sample only while a trajectory is active and freeze (keep their
    // final data) at completion/STOP.  This is deliberately separate from
    // logActive (CSV) and from telemetry reception, which both continue.
    private bool chartActive;
    private long runStartTicks;
    // Actual-signal monitoring completion (independent of the desired trajectory
    // and of CSV logging): the measured position must be stable for a hold, with
    // a bounded timeout so memory stays finite.
    private const int MonitorHoldSamples = 20;
    private const int MonitorTimeoutMs = 8000;
    private int monStableSamples;
    private long monStartTicks;
    private double monLastXCm, monLastYCm;
    private bool monHasLast;
    private bool haveFinalTarget;
    private double finalTargetXCm;
    private double finalTargetYCm;
    private int reachedSamples;
    private long settleStartTicks;

    // Reference derivatives + dynamics feedforward.  The command packet carries
    // the reference joint angles (counts) and the 2R inverse-dynamics torque for
    // the same sample.  Torque fixed-point: 1 LSB = 0.001 N*m (int16 => +-32.767
    // N*m).  Derivatives come from the streamed sample grid and the analytic
    // quintic time law (chain rule), consistent for POINT, LINE and CIRCLE.
    private const double TorqueLsbPerNm = 1000.0;
    private const short TorqueLsbMax = short.MaxValue;
    private double refDq1, refDq2, refDdq1, refDdq2, refTau1, refTau2;
    private bool haveRefTorque;

    // ---- Logging-only PID / PWM estimates (CSV only; never sent to the ESP32) ----
    // The ESP32 PID runs on the COUNT error (target count - measured count) and
    // its output is the BTS7960 PWM command; the gains below are the firmware's
    // setPID() values (mega2560/src/main.cpp).  The PC reproduces that equation
    // only for CSV logging, using its own integral/derivative state so the live
    // controller/timing is untouched.  The PWM full scale is 2^PWM_RESOLUTION
    // (=256 at 8 bits).  TauPID is the torque EQUIVALENT of that PWM command using
    // the firmware's torque<->PWM normalization scale (TAU_SCALE_*_NM); it is an
    // estimate, not the ESP32's actual output.
    private const double PidKp1 = 0.5, PidKi1 = 0.05, PidKd1 = 0.3;   // encode1.setPID
    private const double PidKp2 = 0.05, PidKi2 = 0.01, PidKd2 = 0.03; // encode2.setPID
    private const double PwmFullScale = 256.0;                        // 2^PWM_RESOLUTION
    // PROCESSING::running clamps the command to +/-(resolution-1) = +/-255.
    private const double PwmMaxDuty = PwmFullScale - 1.0;
    private const double TauScale1Nm = 1.382, TauScale2Nm = 0.205;    // firmware TAU_SCALE_*
    private double logPidInt1, logPidInt2, logPidErrPrev1, logPidErrPrev2;
    private bool logPidPrevValid1, logPidPrevValid2;

    // Latest measured encoder feedback, used to capture the start position of a
    // PC-generated point-to-point reference.  Defaults match the firmware's
    // startup pose until the first telemetry frame arrives.
    private short lastActualCount1 = RobotMath.InitialCount1;
    private short lastActualCount2 = RobotMath.InitialCount2;
    // Original duration scaling (MControl TIME*_PER_90 seconds per PI/2 travel)
    // used to time the PC-generated quintic reference.
    private const double TimePer90Motor1 = 1.8;
    private const double TimePer90Motor2 = 2.0;

    public MainForm()
    {
        Text = "2-DOF Robot Control";
        FormBorderStyle = FormBorderStyle.Sizable;
        MinimumSize = new Size(1100, 900);
        StartPosition = FormStartPosition.CenterScreen;
        BackColor = Canvas;
        ForeColor = TextPrimary;
        Font = new Font("Segoe UI", 9F);
        Controls.Add(BuildContent());
        serial.TelemetryReceived += (_, data) => QueueTelemetry(data);
        serial.StatusReceived += (_, code) => QueueRobotStatus(code);
        serial.CommunicationError += (_, error) => QueueStatus("ERROR", error);
        streamTimer.Tick += StreamTick;
        portTimer.Tick += (_, _) => RefreshPorts(); portTimer.Start();
        RefreshPorts();
        baudBox.Items.AddRange(new object[] { "9600", "57600", "115200", "921600" });
        // Show the baudrate currently configured in the firmware source so the
        // HMI and the ESP32 stay in step across restarts.
        var configuredBaud = BaudConfigSync.ReadConfiguredBaud(BaudConfigSync.ProjectRoot);
        var configuredText = configuredBaud?.ToString(CultureInfo.InvariantCulture);
        if (configuredText is not null && !baudBox.Items.Contains(configuredText)) baudBox.Items.Add(configuredText);
        baudBox.SelectedItem = configuredText is not null && baudBox.Items.Contains(configuredText) ? configuredText : "921600";
        baudBox.SelectedIndexChanged += (_, _) => ApplySelectedBaud();
        SetStatus("DISCONNECTED", "Select a COM port and connect.");
        FormClosing += (_, _) => { portTimer.Stop(); streamTimer.Stop(); StopLogging(false); EndMonitoring("app closing"); serial.Dispose(); };
    }

    private Control BuildTopBar()
    {
        var bar = new Panel { Dock = DockStyle.Top, Height = 72, BackColor = Color.FromArgb(18, 24, 33), Padding = new Padding(20, 8, 10, 8) };
        var title = new Label { Text = "2-DOF ROBOT CONTROL", AutoSize = true, ForeColor = Color.FromArgb(240, 245, 250), Font = new Font("Segoe UI", 18, FontStyle.Bold), Dock = DockStyle.Top };
        //var subtitle = new Label { Text = "ESP32 Motion Control & Real-Time Monitoring", AutoSize = true, ForeColor = Color.FromArgb(145, 162, 180), Font = new Font("Segoe UI", 9), Dock = DockStyle.Top };
        title.MouseDown += DragWindow; bar.MouseDown += DragWindow;
        var text = new Panel { Dock = DockStyle.Left, Width = 470 }; text.Controls.Add(title); //text.Controls.Add(subtitle); 
        var tools = new FlowLayoutPanel { Dock = DockStyle.Right, AutoSize = true, WrapContents = false, Padding = new Padding(0, 13, 2, 0) };
        tools.Controls.AddRange(new Control[] { stateLabel, ButtonFor("â€”", (_, _) => WindowState = FormWindowState.Minimized), ButtonFor("â–¡", (_, _) => WindowState = WindowState == FormWindowState.Maximized ? FormWindowState.Normal : FormWindowState.Maximized), ButtonFor("Ã—", (_, _) => Close(), Color.IndianRed) });
        bar.Controls.Add(tools); bar.Controls.Add(text); return bar;
    }

    private Control BuildContent()
    {
        var content = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 3, BackColor = Canvas, Padding = new Padding(8, 10, 8, 0) };
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 70));
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 25));
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 5));
        content.Controls.Add(BuildDashboard(), 0, 0);
        content.Controls.Add(BuildCommandPanel(), 0, 1);
        var footer = new Panel { Dock = DockStyle.Fill, BackColor = Surface };
        footer.Controls.Add(statusMessage);
        stateLabel.Dock = DockStyle.Right;
        footer.Controls.Add(stateLabel);
        content.Controls.Add(footer, 0, 2);
        return content;
    }

    private Control BuildCommandPanel()
    {
        var panel = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2, BackColor = Canvas, Padding = new Padding(0, 0, 0, 0) };
        // panel.RowStyles.Add(new RowStyle(SizeType.Absolute, 22)); panel.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        // var heading = new Label { Text = "COMMAND CENTER", Dock = DockStyle.Fill, ForeColor = TextMuted, Font = new Font("Segoe UI", 8F, FontStyle.Bold), TextAlign = ContentAlignment.MiddleLeft };
        // panel.Controls.Add(heading, 0, 0);
        var commands = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 5, RowCount = 1, Margin = new Padding(0) };
        // Relative columns let the command groups use the window width without narrowing Motion Safety.
        foreach (var width in new[] { 21f, 19f, 19f, 22f, 19f }) commands.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, width));
        commands.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        commands.Controls.Add(BuildConnectionGroup(), 0, 0);
        commands.Controls.Add(BuildMotionGroup(), 1, 0);
        commands.Controls.Add(BuildPointGroup(), 2, 0);
        commands.Controls.Add(BuildLineGroup(), 3, 0);
        commands.Controls.Add(BuildCircleGroup(), 4, 0);
        panel.Controls.Add(commands, 0, 1);
        return panel;
    }

    private GroupBox BuildConnectionGroup()
    {
        var group = Group("CONNECTION");
        var grid = FormGrid();

        AddField(grid, "COM port:", portBox, 0);
        AddField(grid, "Baud rate:", baudBox, 1);

        var refreshButton = ButtonFor("REFRESH", (_, _) => RefreshPorts());

        // Make CONNECT visually identical to REFRESH.
        connectButton.Text = "CONNECT";
        connectButton.AutoSize = true;
        connectButton.Dock = DockStyle.None;
        connectButton.Anchor = AnchorStyles.Left;
        connectButton.Height = refreshButton.Height;
        connectButton.FlatStyle = refreshButton.FlatStyle;
        connectButton.BackColor = refreshButton.BackColor;
        connectButton.ForeColor = refreshButton.ForeColor;
        connectButton.FlatAppearance.BorderColor = refreshButton.FlatAppearance.BorderColor;
        connectButton.FlatAppearance.BorderSize = refreshButton.FlatAppearance.BorderSize;
        connectButton.Margin = refreshButton.Margin;
        connectButton.Padding = refreshButton.Padding;
        connectButton.Font = refreshButton.Font;
        connectButton.Size = refreshButton.PreferredSize;

        connectButton.Click += ConnectClicked;

        // Add 15 px to the button rows.
        grid.RowStyles[2] = new RowStyle(SizeType.Absolute, 35F);
        grid.RowStyles[3] = new RowStyle(SizeType.Absolute, 35F);

        grid.Controls.Add(refreshButton, 1, 2);
        grid.Controls.Add(connectButton, 1, 3);

        group.Controls.Add(grid);
        return group;
    }

    private GroupBox BuildMotionGroup()
    {
        var group = Group("MOTION SAFETY");
        var row = new FlowLayoutPanel { Dock = DockStyle.Fill, Padding = new Padding(4), AutoSize = false, WrapContents = false };
        homeButton = ButtonFor("HOME", HomeRequested);
        row.Controls.Add(homeButton); row.Controls.Add(ButtonFor("STOP", StopRequested, Color.IndianRed)); group.Controls.Add(row); return group;
    }

    private GroupBox BuildPointGroup()
    {
        var group = Group("MOVE TO POINT"); var grid = FormGrid();
        AddField(grid, "X:", pointX, 0); AddField(grid, "Y:", pointY, 1);
        var run = ButtonFor("RUN POINT", (_, _) => RunPoint()); grid.Controls.Add(run, 1, 2);
        WireEnter(pointX, pointY); WireEnter(pointY, run); group.Controls.Add(grid); return group;
    }

    private GroupBox BuildLineGroup()
    {
        var group = Group("LINE MOTION"); var grid = FormGrid();
        AddField(grid, "Start X:", lineX1, 0); AddField(grid, "Start Y:", lineY1, 1); AddField(grid, "End X:", lineX2, 2); AddField(grid, "End Y:", lineY2, 3);
        var run = ButtonFor("RUN LINE", (_, _) => RunLine()); grid.Controls.Add(run, 1, 4);
        WireEnter(lineX1, lineY1); WireEnter(lineY1, lineX2); WireEnter(lineX2, lineY2); WireEnter(lineY2, run); group.Controls.Add(grid); return group;
    }

    private GroupBox BuildCircleGroup()
    {
        var group = Group("CIRCLE MOTION");
        var grid = FormGrid();

        AddField(grid, "Center X:", circleX, 0);
        AddField(grid, "Center Y:", circleY, 1);
        AddField(grid, "Radius:", circleR, 2);

        var run = ButtonFor("RUN CIRCLE", (_, _) => RunCircle());
        grid.Controls.Add(run, 1, 3);
        WireEnter(circleX, circleY);
        WireEnter(circleY, circleR);
        WireEnter(circleR, run);

        group.Controls.Add(grid);
        return group;
    }

    private Control BuildMonitor()
    {
        // Only the two joint-angle desired-vs-actual plots remain, filling the
        // whole monitoring area: a single 2-column row so both charts share the
        // width and height evenly and resize with the container (no clipping,
        // truncation or empty rows).
        var monitor = new TableLayoutPanel { Dock = DockStyle.Fill, AutoSize = false, GrowStyle = TableLayoutPanelGrowStyle.FixedSize, ColumnCount = 2, RowCount = 1, Padding = new Padding(2) };
        for (var i = 0; i < 2; i++) monitor.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50f));
        monitor.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        joint1Chart.MinimumSize = joint2Chart.MinimumSize = Size.Empty;
        joint1Chart.Dock = joint2Chart.Dock = DockStyle.Fill;
        monitor.Controls.Add(joint1Chart, 0, 0);
        monitor.Controls.Add(joint2Chart, 1, 0);
        return monitor;
    }

    private Control BuildDashboard()
    {
        var dashboard = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 2, BackColor = BackColor };
        dashboard.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 56));
        dashboard.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 44));
        dashboard.RowStyles.Add(new RowStyle(SizeType.Percent, 48));
        dashboard.RowStyles.Add(new RowStyle(SizeType.Percent, 52));

        var robotCard = new GroupBox { Text = "ROBOT VIEW", Dock = DockStyle.Fill, Padding = new Padding(8), ForeColor = Color.FromArgb(62, 190, 190), BackColor = Color.FromArgb(22, 29, 38) };
        robotCard.Controls.Add(robotView);
        dashboard.Controls.Add(robotCard, 0, 0);

        var statusCard = new GroupBox { Text = "LIVE STATUS", Dock = DockStyle.Fill, Padding = new Padding(10), ForeColor = Color.FromArgb(62, 190, 190), BackColor = Color.FromArgb(22, 29, 38) };
        var statusLayout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2, BackColor = Color.FromArgb(22, 29, 38) };
        statusLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 68));
        statusLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 32));
        var table = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 3, BackColor = Color.FromArgb(22, 29, 38) };
        for (var i = 0; i < 3; i++)
        {
            table.RowStyles.Add(new RowStyle(SizeType.Percent, 33.34f));
        }
        table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        AddDashboardValue(table, "J1 ANGLE", "q1", 0, 0); AddDashboardValue(table, "J2 ANGLE", "q2", 1, 0);
        AddDashboardValue(table, "J1 COUNT", "count1", 0, 1); AddDashboardValue(table, "J2 COUNT", "count2", 1, 1);
        AddDashboardValue(table, "ACTUAL X (cm)", "x", 0, 2); AddDashboardValue(table, "ACTUAL Y (cm)", "y", 1, 2);
        readouts["e1"] = new Label(); readouts["e2"] = new Label(); readouts["et"] = new Label();
        statusLayout.Controls.Add(table, 0, 0);
        statusLayout.Controls.Add(BuildCsvLogSection(), 0, 1);
        statusCard.Controls.Add(statusLayout); dashboard.Controls.Add(statusCard, 1, 0);

        var monitorCard = new GroupBox { Text = "REAL-TIME MONITORING", Dock = DockStyle.Fill, Padding = new Padding(8), ForeColor = Color.FromArgb(62, 190, 190), BackColor = Color.FromArgb(22, 29, 38) };
        monitorCard.Controls.Add(BuildMonitor()); dashboard.Controls.Add(monitorCard, 0, 1); dashboard.SetColumnSpan(monitorCard, 2);
        return dashboard;
    }

    private void AddDashboardValue(TableLayoutPanel table, string title, string key, int column, int row)
    {
        var cell = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1, BackColor = Color.FromArgb(29, 38, 49), Margin = new Padding(4) };
        cell.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize)); cell.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        cell.Controls.Add(new Label { Text = title + ":", Dock = DockStyle.Fill, ForeColor = Color.FromArgb(145, 160, 176), TextAlign = ContentAlignment.MiddleLeft, Padding = new Padding(8, 0, 4, 0), AutoSize = true }, 0, 0);
        var output = new Label { Text = "--", Dock = DockStyle.Fill, AutoEllipsis = true, ForeColor = Color.FromArgb(232, 238, 244), Font = new Font("Segoe UI", 11F, FontStyle.Bold), TextAlign = ContentAlignment.MiddleRight, Padding = new Padding(2, 0, 8, 0) };
        readouts[key] = output; cell.Controls.Add(output, 1, 0); table.Controls.Add(cell, column, row);
    }

    private Control BuildCsvLogSection()
    {
        var section = new GroupBox { Text = "CSV LOG FILES", Dock = DockStyle.Fill, ForeColor = Accent, BackColor = Surface, Margin = new Padding(0, 8, 0, 0) };
        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2, Padding = new Padding(8), BackColor = Surface };
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        section.Controls.Add(layout);

        var pathRow = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            ColumnCount = 2,
            RowCount = 1,
            Margin = new Padding(0, 0, 0, 4),
            BackColor = Surface,
            Padding = new Padding(0)
        };
        pathRow.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        pathRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        pathRow.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        var pathLabel = new Label
        {
            Text = "Path:",
            AutoSize = true,
            Anchor = AnchorStyles.Left,
            ForeColor = TextMuted,
            Margin = new Padding(0, 0, 8, 0),
            TextAlign = ContentAlignment.MiddleLeft
        };
        pathLabel.TextAlign = ContentAlignment.MiddleLeft;

        committedCsvPath = logger.DirectoryPath;
        csvPathBox.Text = committedCsvPath;
        csvPathBox.KeyDown += CsvPathKeyDown;
        csvPathBox.Leave += (_, _) => CommitCsvPath();
        csvPathBox.MouseDown += CsvPathMouseDown;
        csvPathBox.Anchor = AnchorStyles.Left | AnchorStyles.Right;
        csvPathBox.Margin = new Padding(0, 0, 0, 0);
        csvPathBox.MinimumSize = new Size(0, 26);

        pathRow.Controls.Add(pathLabel, 0, 0);
        pathRow.Controls.Add(csvPathBox, 1, 0);
        pathRow.SetColumnSpan(csvPathBox, 1);
        pathRow.SetRowSpan(csvPathBox, 1);

        layout.Controls.Add(pathRow, 0, 0);
        layout.Controls.Add(csvFileList, 0, 1);
        RefreshCsvFiles();
        return section;
    }

    private void CsvPathMouseDown(object? sender, MouseEventArgs e)
    {
        if ((Control.ModifierKeys & Keys.Control) == Keys.Control)
        {
            var target = csvPathBox.Text.Trim();
            if (string.IsNullOrWhiteSpace(target)) { SetStatus("ERROR", "CSV path is empty."); return; }

            try
            {
                var fullPath = Path.GetFullPath(target);
                if (Directory.Exists(fullPath))
                {
                    Process.Start(new ProcessStartInfo { FileName = fullPath, UseShellExecute = true });
                    return;
                }

                if (File.Exists(fullPath))
                {
                    Process.Start(new ProcessStartInfo { FileName = "explorer.exe", Arguments = $"/select,\"{fullPath}\"", UseShellExecute = true });
                    return;
                }

                var parent = Path.GetDirectoryName(fullPath);
                if (!string.IsNullOrEmpty(parent) && Directory.Exists(parent))
                {
                    Process.Start(new ProcessStartInfo { FileName = parent, UseShellExecute = true });
                    return;
                }

                SetStatus("ERROR", $"CSV path does not exist: {target}");
            }
            catch (Exception ex)
            {
                SetStatus("ERROR", "Could not open CSV path: " + ex.Message);
            }
        }
    }

    private void CsvPathKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.KeyCode == Keys.Enter)
        {
            e.Handled = true;
            e.SuppressKeyPress = true;

            if (CommitCsvPath())
            {
                csvFileList.Focus();
            }
        }
    }

    private bool CommitCsvPath()
    {
        var candidate = csvPathBox.Text?.Trim();
        if (string.IsNullOrWhiteSpace(candidate))
        {
            SetStatus("ERROR", "CSV path cannot be empty.");
            csvPathBox.Text = committedCsvPath;
            return false;
        }

        try
        {
            var normalized = NormalizeCsvRootPath(candidate);
            if (!logger.TrySetCustomDataRoot(normalized, out var error))
            {
                SetStatus("ERROR", error ?? "Invalid CSV path.");
                csvPathBox.Text = committedCsvPath;
                return false;
            }

            committedCsvPath = normalized;
            csvPathBox.Text = normalized;
            RefreshCsvFiles();
            SetStatus(robotState, $"CSV path updated: {normalized}");
            return true;
        }
        catch (Exception ex)
        {
            SetStatus("ERROR", "Invalid CSV path: " + ex.Message);
            csvPathBox.Text = committedCsvPath;
            return false;
        }
    }

    private static string NormalizeCsvRootPath(string value)
    {
        var fullPath = Path.GetFullPath(value.Trim());
        var folder = Path.GetFileName(fullPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
        if (string.Equals(folder, TrajectoryType.Point.ToString(), StringComparison.OrdinalIgnoreCase)
            || string.Equals(folder, TrajectoryType.Line.ToString(), StringComparison.OrdinalIgnoreCase)
            || string.Equals(folder, TrajectoryType.Circle.ToString(), StringComparison.OrdinalIgnoreCase))
        {
            var parent = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(parent)) return parent;
        }

        if (File.Exists(fullPath))
        {
            var directory = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(directory)) return NormalizeCsvRootPath(directory);
        }

        return fullPath;
    }

    private void RefreshCsvFiles()
    {
        csvFileList.BeginUpdate();
        csvFileList.Items.Clear();
        csvFilePaths.Clear();
        var files = logger.GetCsvFiles();
        if (files.Length == 0)
        {
            csvFileList.Items.Add("No CSV files yet.");
            csvFileList.Enabled = false;
        }
        else
        {
            foreach (var file in files) { csvFileList.Items.Add(Path.GetFileName(file)); csvFilePaths.Add(file); }
            csvFileList.Enabled = true;
        }
        csvFileList.EndUpdate();
    }

    private void RunPoint()
    {
        if (homingActive) { SetStatus("ERROR", "Homing is in progress; wait for it to complete before running a trajectory."); return; }
        logger.SetTrajectoryType(TrajectoryType.Point);
        ResumeCharts();
        BeginErrorRun(TrajectoryType.Point);
        if (!TryCoordinate(pointX, out var x) || !TryCoordinate(pointY, out var y)) return;
        if (!InverseKinematics.TryCalculatePoint(x, y, out var q1, out var q2, out var count1, out var count2)) { SetStatus("ERROR", "Point is outside the C++ kinematics workspace."); return; }
        if (!serial.IsConnected) { SetStatus("ERROR", "Connect before sending a point command."); return; }
        // Safe start policy: a PC-generated point reference must start from the
        // measured position.  Refuse the move if feedback is stale rather than
        // silently assuming the robot is at its nominal initial position.
        if (!FeedbackIsFresh())
        {
            SetStatus("ERROR", "No recent encoder feedback; point move rejected. Verify the ESP32 is connected and streaming telemetry.");
            return;
        }
        if (!FeedbackWithinJointLimits()) return;
        // Show the requested Cartesian target immediately, from the user's X/Y
        // (cm -> m). This does not depend on live telemetry, so the marker
        // stays on Robot View even before/without encoder feedback.
        targetPoint = (x / 100.0, y / 100.0);
        robotView.SetTarget(targetPoint);
        // PC-side generation: quintic-interpolated joint reference from the
        // current measured position to the requested target.  The reference is
        // streamed to the ESP32, which only follows the latest sample.
        // Unwrap the IK target counts to the equivalent representation nearest the
        // measured encoder counts.  The IK/count conversion returns angles wrapped
        // to (-pi,pi], but the encoder counts are continuous (free-running), so a
        // wrapped target can be many revolutions from the measured position and
        // would command a multi-turn move (and produce large TargetTheta* values).
        // Unwrapping relative to the measured state commands the shortest
        // equivalent rotation while preserving the same physical (FK) target.
        count1 = RobotMath.UnwrapCount1(count1, lastActualCount1);
        count2 = RobotMath.UnwrapCount2(count2, lastActualCount2);
        var duration = PointDurationSeconds(lastActualCount1, lastActualCount2, count1, count2);
        var refCounts = BuildJointReference(lastActualCount1, lastActualCount2, count1, count2, duration, out var refSamples);
        StartStreaming(TrajectoryType.Point, refCounts, refSamples, duration, null,
            $"Point sent: ({x}, {y}) cm  /  q1 {(q1 * 180 / Math.PI):F1} Â°  /  q2 {(q2 * 180 / Math.PI):F1} Â°");
    }

    // Duration for a PC-generated joint move using the original MControl
    // TIME*_PER_90 scaling (seconds per PI/2 of joint travel).
    private static double PointDurationSeconds(short start1, short start2, short target1, short target2)
    {
        var dq1 = Math.Abs(RobotMath.CountToJoint1Angle(target1) - RobotMath.CountToJoint1Angle(start1));
        var dq2 = Math.Abs(RobotMath.CountToJoint2Angle(target2) - RobotMath.CountToJoint2Angle(start2));
        var t1 = dq1 * TimePer90Motor1 / (Math.PI / 2);
        var t2 = dq2 * TimePer90Motor2 / (Math.PI / 2);
        return Math.Max(0.1, Math.Max(t1, t2));
    }

    // Linear joint-count samples on a fixed ReferencePeriodMs grid from start to
    // target; the existing quintic time-scaling (StreamTick profile) is applied
    // to the sample index, producing the smooth reference without a second
    // interpolation method.  Timing comes from the monotonic streamClock, not
    // from these sample indices.
    private static short[] BuildJointReference(short start1, short start2, short target1, short target2, double duration, out int sampleCount)
    {
        var periodSeconds = ReferencePeriodMs / 1000.0;
        sampleCount = Math.Max(2, (int)Math.Ceiling(duration / periodSeconds) + 1);
        var counts = new short[sampleCount * 2];
        for (var i = 0; i < sampleCount; i++)
        {
            var f = (double)i / (sampleCount - 1);
            counts[2 * i] = (short)Math.Round(start1 + (target1 - start1) * f);
            counts[2 * i + 1] = (short)Math.Round(start2 + (target2 - start2) * f);
        }
        return counts;
    }

    // True when encoder feedback is recent enough to trust the measured state.
    private bool FeedbackIsFresh() => Environment.TickCount64 - lastTelemetryTicks <= TelemetryFreshnessMs;

    // The streamed targets are always kept inside the joint ranges, but the
    // measured position is free-running feedback and can lie outside them (the
    // joint was pushed past a limit, or the homing/encoder zero is wrong).
    // Refuse to start a trajectory from such an out-of-range pose instead of
    // driving from an unsafe configuration; the operator must re-home / move the
    // robot back to a valid pose.  The measured value is never clamped or
    // normalized - only rejected while it is invalid, so normal operation is
    // unaffected when the measured position is valid.
    private bool FeedbackWithinJointLimits()
    {
        if (RobotMath.WithinJointLimits(lastActualCount1, lastActualCount2)) return true;
        SetStatus("ERROR", $"Measured joint position is outside the permitted range (J1 {lastActualCount1} counts in [{RobotMath.MinCount1}, {RobotMath.MaxCount1}], J2 {lastActualCount2} counts in [{RobotMath.MinCount2}, {RobotMath.MaxCount2}]). Re-home / move the robot to a valid pose before starting a trajectory.");
        return false;
    }

    // Measured end-effector position in cm, from the latest encoder feedback and
    // the shared forward kinematics.
    private (double X, double Y) MeasuredPointCm()
    {
        var q1 = RobotMath.CountToJoint1Angle(lastActualCount1);
        var q2 = RobotMath.CountToJoint2Angle(lastActualCount2);
        var (x, y) = RobotMath.ForwardKinematics(q1, q2);
        return (x * 100.0, y * 100.0);
    }

    // Validate feedback freshness and that the robot is at the commanded path
    // start before a Cartesian trajectory begins streaming.
    private bool RequirePathStart(double targetXCm, double targetYCm, string label)
    {
        if (!FeedbackIsFresh())
        {
            SetStatus("ERROR", $"No recent encoder feedback; {label} move rejected. Verify the ESP32 is connected and streaming telemetry.");
            return false;
        }
        if (!FeedbackWithinJointLimits()) return false;
        var (mx, my) = MeasuredPointCm();
        var distance = Math.Sqrt((mx - targetXCm) * (mx - targetXCm) + (my - targetYCm) * (my - targetYCm));
        if (distance > PathStartToleranceCm)
        {
            SetStatus("ERROR", $"{label} start ({targetXCm:F0}, {targetYCm:F0}) cm is {distance:F1} cm from the measured position ({mx:F1}, {my:F1}) cm. Move to the path start first (e.g. a Point command).");
            return false;
        }
        return true;
    }

    // Line Motion: validate the Cartesian endpoints, let the shared C++ engine
    // plan the whole path (existing RRplanar_in + count conversion), then stream
    // the resulting desired joint counts over the existing serial path.  The HMI
    // performs no inverse kinematics of its own.
    private void RunLine()
    {
        if (homingActive) { SetStatus("ERROR", "Homing is in progress; wait for it to complete before running a trajectory."); return; }
        logger.SetTrajectoryType(TrajectoryType.Line);
        ResumeCharts();
        BeginErrorRun(TrajectoryType.Line);
        if (!TryCoordinate(lineX1, out var x1) || !TryCoordinate(lineY1, out var y1)
            || !TryCoordinate(lineX2, out var x2) || !TryCoordinate(lineY2, out var y2)) return;
        if (!serial.IsConnected) { SetStatus("ERROR", "Connect before sending a line command."); return; }
        // The straight path starts at (x1,y1); require the robot to be there
        // (with fresh feedback) so the first streamed sample is continuous.
        if (!RequirePathStart(x1, y1, "Line")) return;
        if (!InverseKinematics.TryPlanLine(x1, y1, x2, y2, out var counts, out var sampleCount, out var duration))
        { SetStatus("ERROR", "Line is outside the C++ kinematics workspace."); return; }
        // The PC generates the Cartesian line reference (shared C++ planner +
        // inverse kinematics) and streams the desired joint counts to the ESP32.
        var command = $"LINE {x1} {y1} 0 TO {x2} {y2} 0";
        StartStreaming(TrajectoryType.Line, counts, sampleCount, duration, command,
            $"Line ({x1}, {y1}) -> ({x2}, {y2})  /  PC-generated reference");
    }

    // Circle Motion: same shared C++ engine, validation and count conversion as
    // Line Motion.
    private void RunCircle()
    {
        if (homingActive) { SetStatus("ERROR", "Homing is in progress; wait for it to complete before running a trajectory."); return; }
        logger.SetTrajectoryType(TrajectoryType.Circle);
        ResumeCharts();
        BeginErrorRun(TrajectoryType.Circle);
        if (!TryCoordinate(circleX, out var cx) || !TryCoordinate(circleY, out var cy)
            || !TryCoordinate(circleR, out var radius)) return;
        if (radius <= 0) { circleR.BackColor = Color.FromArgb(89, 47, 52); circleR.Focus(); SetStatus("ERROR", "Circle radius must be positive."); return; }
        if (!serial.IsConnected) { SetStatus("ERROR", "Connect before sending a circle command."); return; }
        // planCircle starts at the circle's angle-0 point (cx+radius, cy); require
        // the robot to be there (with fresh feedback) for a continuous start.
        if (!RequirePathStart(cx + radius, cy, "Circle")) return;
        if (!InverseKinematics.TryPlanCircle(cx, cy, radius, out var counts, out var sampleCount, out var duration))
        { SetStatus("ERROR", "Circle is outside the C++ kinematics workspace."); return; }
        // The PC generates the Cartesian circle reference (shared C++ planner +
        // inverse kinematics) and streams the desired joint counts to the ESP32.
        var command = $"CIRCLE {cx} {cy} 0 {radius}";
        StartStreaming(TrajectoryType.Circle, counts, sampleCount, duration, command,
            $"Circle center ({cx}, {cy}) r={radius}  /  PC-generated reference");
    }

    private void StartStreaming(TrajectoryType type, short[] counts, int sampleCount, double duration, string? command, string message)
    {
        if (sampleCount <= 0 || counts.Length < sampleCount * 2)
        {
            SetStatus("ERROR", "Trajectory produced no samples.");
            return;
        }
        streamCounts = counts;
        streamSampleCount = sampleCount;
        streamDuration = Math.Max(duration, 0.2);
        streamTrajectory = type;
        // Reference the first sample immediately for the tracking-error readouts
        // and Robot View while the ESP32 executes the command.
        targetQ1 = RobotMath.CountToJoint1Angle(counts[0]);
        targetQ2 = RobotMath.CountToJoint2Angle(counts[1]);
        targetPoint = RobotMath.ForwardKinematics(targetQ1.Value, targetQ2.Value);
        robotView.SetTarget(targetPoint);
        // Only line/circle send the legacy parameter text; the reference itself
        // is streamed as desired-count frames for every trajectory type.
        if (!string.IsNullOrEmpty(command))
        {
            try { serial.SendText(command); }
            catch (Exception ex) { streamCounts = null; StopLogging(false); SetStatus("ERROR", "Send failed: " + ex.Message); return; }
        }
        // The last interpolated sample is the completion target used to decide
        // when the execution has actually finished moving.
        var last = streamSampleCount - 1;
        var fq1 = RobotMath.CountToJoint1Angle(counts[2 * last]);
        var fq2 = RobotMath.CountToJoint2Angle(counts[2 * last + 1]);
        var (fx, fy) = RobotMath.ForwardKinematics(fq1, fq2);
        finalTargetXCm = fx * 100.0; finalTargetYCm = fy * 100.0;
        haveFinalTarget = true;
        StartLoggingExecution();
        streamClock.Restart();
        streamTimer.Start();
        SetStatus("MOVING", message);
    }

    // Begin (or restart) the CSV logging lifecycle for one execution.
    private void StartLoggingExecution()
    {
        logActive = true;
        streamGenerationDone = false;
        reachedSamples = 0;
        haveRefTorque = false;
        refDq1 = refDq2 = refDdq1 = refDdq2 = refTau1 = refTau2 = 0;
        // Fresh logging-only PID state for this execution.
        logPidInt1 = logPidInt2 = logPidErrPrev1 = logPidErrPrev2 = 0;
        logPidPrevValid1 = logPidPrevValid2 = false;
        // Start a fresh angle-chart sampling run with a per-trajectory time base.
        chartActive = true;
        runStartTicks = Environment.TickCount64;
        monStableSamples = 0;
        monHasLast = false;
    }

    // End the CSV logging lifecycle.  DataLogger appends each row with an
    // open/write/close, so stopping the appends finalises (flushes) the file; the
    // completed file is left on disk for inspection and plotting.
    private void StopLogging(bool completed)
    {
        if (!logActive) return;
        logActive = false;
        streamGenerationDone = false;
        reachedSamples = 0;
        _ = completed;
        // NOTE: the angle-chart (Real-Time Monitoring) lifecycle is deliberately
        // NOT tied to CSV logging.  It is ended separately by EndMonitoring()
        // only when the actual signal has settled (or on an explicit
        // stop/reset/disconnect), so finishing the desired trajectory or the CSV
        // file does not truncate the actual curve.
    }

    // End the Real-Time Monitoring session (freeze the angle charts, retaining
    // their data).  Called when the actual signal is reliably complete or on an
    // explicit STOP/disconnect - never merely because the desired trajectory or
    // CSV logging finished.
    private string monitorEndReason = "";
    private void EndMonitoring(string reason)
    {
        if (!chartActive) return;
        chartActive = false;
        monStableSamples = 0;
        monHasLast = false;
        monitorEndReason = reason;
    }

    // Compute the desired reference (joint positions, reference derivatives and
    // inverse-dynamics feed-forward) for normalized quintic progress u from the
    // streamed sample grid, and update the shared fields used for transmission
    // and CSV logging.  Extracted so the CSV logger can refresh the desired
    // reference at each telemetry row (below) instead of only on stream-timer
    // ticks - Windows Forms timers can be starved by the telemetry message
    // queue, which left the logged desired/derivative columns stale.
    private void ComputeReferenceAt(double u, out int index, out short count1, out short count2, out short tauLsb1, out short tauLsb2)
    {
        // Existing quintic time-scaling (the project's interQuintic profile):
        // the PC generates the time-varying reference before transmitting it.
        var eased = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        index = (int)Math.Round(eased * (streamSampleCount - 1));
        if (index < 0) index = 0;
        if (index >= streamSampleCount) index = streamSampleCount - 1;
        count1 = streamCounts![2 * index];
        count2 = streamCounts![2 * index + 1];
        // Reference derivatives for THIS reference sample, from the actual
        // streamed sample grid and the analytic quintic time law (chain rule):
        //   idx(t) = s(t) * (N-1),  q_ref = q_sample(idx)
        //   qdot = (dq/didx) * idxdot;  qddot = (d2q/didx2)*idxdot^2 + (dq/didx)*idxddot
        var rq1 = RobotMath.CountToJoint1Angle(count1);
        var rq2 = RobotMath.CountToJoint2Angle(count2);
        var (_, sDot, sDdot) = InverseKinematics.Quintic(u, streamDuration);
        var idxPrev = Math.Max(0, index - 1);
        var idxNext = Math.Min(streamSampleCount - 1, index + 1);
        var q1Prev = RobotMath.CountToJoint1Angle(streamCounts![2 * idxPrev]);
        var q1Next = RobotMath.CountToJoint1Angle(streamCounts![2 * idxNext]);
        var q2Prev = RobotMath.CountToJoint2Angle(streamCounts![2 * idxPrev + 1]);
        var q2Next = RobotMath.CountToJoint2Angle(streamCounts![2 * idxNext + 1]);
        var q1p = (q1Next - q1Prev) / 2.0; // dq/didx (central)
        var q2p = (q2Next - q2Prev) / 2.0;
        var q1pp = q1Next - 2.0 * rq1 + q1Prev; // d2q/didx2
        var q2pp = q2Next - 2.0 * rq2 + q2Prev;
        var didt = (streamSampleCount - 1) * sDot;
        var d2idt2 = (streamSampleCount - 1) * sDdot;
        refDq1 = q1p * didt;
        refDq2 = q2p * didt;
        refDdq1 = q1pp * didt * didt + q1p * d2idt2;
        refDdq2 = q2pp * didt * didt + q2p * d2idt2;
        haveRefTorque = InverseKinematics.TryComputeTorque(rq1, rq2, refDq1, refDq2, refDdq1, refDdq2, out refTau1, out refTau2)
            && double.IsFinite(refTau1) && double.IsFinite(refTau2);
        if (!haveRefTorque)
        {
            // No valid dynamics for this sample: use zero torque rather than a
            // stale or non-finite value (the field is evaluation-only anyway).
            refTau1 = refTau2 = 0;
        }
        // Signed fixed-point torque, saturating at the int16 boundary.
        tauLsb1 = (short)Math.Clamp(Math.Round(refTau1 * TorqueLsbPerNm), -32767.0, TorqueLsbMax);
        tauLsb2 = (short)Math.Clamp(Math.Round(refTau2 * TorqueLsbPerNm), -32767.0, TorqueLsbMax);
        // The trajectory reference at the current time drives the live error
        // readouts and the desired columns/TotalErrorRad logged to CSV.
        targetQ1 = RobotMath.CountToJoint1Angle(count1);
        targetQ2 = RobotMath.CountToJoint2Angle(count2);
    }

    private void StreamTick(object? sender, EventArgs e)
    {
        if (streamCounts is null) { streamTimer.Stop(); return; }
        var elapsed = streamClock.Elapsed.TotalSeconds;
        var u = streamDuration <= 0 ? 1.0 : elapsed / streamDuration;
        if (u > 1.0) u = 1.0;
        ComputeReferenceAt(u, out var index, out var count1, out var count2, out var tauLsb1, out var tauLsb2);
        // Transmit the reference sample (joint counts + dynamics torque) in one
        // 11-byte packet.  The ESP32 follows the counts with its PID; the torque
        // field is decoded and mapped to an isolated pwmNorm (not used to drive
        // the motors in this task).
        if (streamSendsCounts)
        {
            try { serial.SendJointCountsWithTorque(count1, count2, tauLsb1, tauLsb2); }
            catch (Exception ex) { streamTimer.Stop(); streamCounts = null; StopLogging(false); SetStatus("ERROR", "Send failed: " + ex.Message); return; }
        }
        // Record the Setpoint trajectory when each planned reference sample is
        // generated, on the shared execution-relative time base.
        if (chartActive)
        {
            var targetTime = (Environment.TickCount64 - runStartTicks) / 1000.0;
            joint1Chart.AddSampleTo("Setpoint", targetTime, targetQ1!.Value);
            joint2Chart.AddSampleTo("Setpoint", targetTime, targetQ2!.Value);
        }
        targetPoint = RobotMath.ForwardKinematics(targetQ1!.Value, targetQ2!.Value);
        robotView.SetTarget(targetPoint);
        SetStatus("MOVING", $"{streamTrajectory}: sample {index + 1}/{streamSampleCount}");
        if (u >= 1.0)
        {
            // Generator finished emitting its final reference.  Logging is NOT
            // stopped here: the robot may still be moving to the final target.
            streamTimer.Stop();
            streamCounts = null;
            streamGenerationDone = true;
            settleStartTicks = Environment.TickCount64;
            // The desired trajectory has finished.  Monitoring continues (actual
            // samples keep being plotted) until the actual signal settles.
            monStartTicks = Environment.TickCount64;
            monStableSamples = 0;
            monHasLast = false;
            SetStatus("MOVING", $"{streamTrajectory}: settling on target...");
        }
    }

    private void ResumeCharts()
    {
        // Count charts keep their live rolling window.  Joint-angle charts are
        // multi-series: history is preserved, never cleared here.
        count1Chart.ResumeLive(); count2Chart.ResumeLive();
    }

    // Start a new execution's position traces; the previous run's traces stay
    // visible on the joint-angle charts until the next run begins.  The type is
    // no longer needed now that the tracking-error charts are removed.
    private void BeginErrorRun(TrajectoryType type)
    {
        _ = type;
        joint1Chart.ResetSeries();
        joint2Chart.ResetSeries();
    }

    private void HomeRequested(object? sender, EventArgs e)
    {
        // Ignore repeated clicks while a HOME request/homing is already active.
        if (homingActive) return;
        if (!serial.IsConnected) { SetStatus("ERROR", "Connect before homing."); return; }
        homingActive = true;
        if (homeButton is not null) homeButton.Enabled = false;
        try
        {
            serial.SendControl(0x02);
            // Do not claim homing started/completed here: the ESP32 reports the
            // real outcome with a status frame (55 AB 51/52/53).
            SetStatus("CONNECTED", "HOME requested; awaiting ESP32 status...");
        }
        catch (Exception ex)
        {
            homingActive = false; if (homeButton is not null) homeButton.Enabled = true;
            SetStatus("ERROR", "HOME failed: " + ex.Message);
        }
    }

    private void StopRequested(object? sender, EventArgs e)
    {
        streamTimer.Stop(); streamCounts = null;
        // A trajectory abort is not a successful completion: finalize logging but
        // never mark it complete.
        StopLogging(false);
        EndMonitoring("STOP");
        // A STOP aborts any homing; release the HOME guard so the operator can
        // re-home deliberately afterwards.
        homingActive = false; if (homeButton is not null) homeButton.Enabled = true;
        if (!serial.IsConnected) { SetStatus("ERROR", "Connect before STOP."); return; }
        try
        {
            serial.SendControl(0x01);
            SetStatus("STOPPED", "STOP sent. Motion latched; enter a new target and press RUN to start a new trajectory.");
        }
        catch (Exception ex) { SetStatus("ERROR", "STOP failed: " + ex.Message); }
    }

    // Applies a newly selected baudrate to the HMI serial setting and to the
    // firmware SOURCE configuration (main.cpp SERIAL_BAUD and platformio.ini
    // monitor_speed).  It never claims the physical ESP32 has changed: changing
    // the source requires a rebuild and upload.
    private void ApplySelectedBaud()
    {
        if (baudBox.SelectedItem is not string text
            || !int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var baud))
            return;

        if (serial.IsConnected)
        {
            serial.Disconnect();
            connectButton.Text = "CONNECT";
            portBox.Enabled = baudBox.Enabled = true;
        }

        if (!BaudConfigSync.TrySync(baud, out var message, out var changed))
        {
            SetStatus("ERROR", message);
            return;
        }

        stateLabel.AccessibleDescription = message;
        SetStatus("CONFIG", changed
            ? $"Baudrate {baud}: HMI serial, main.cpp SERIAL_BAUD and platformio monitor_speed updated. Rebuild and upload the ESP32 firmware before reconnecting."
            : $"Baudrate already {baud}; no configuration changes were made.");
    }

    private void ConnectClicked(object? sender, EventArgs e)
    {
        if (serial.IsConnected) { streamTimer.Stop(); streamCounts = null; StopLogging(false); EndMonitoring("disconnected"); serial.Disconnect(); connectButton.Text = "CONNECT"; portBox.Enabled = baudBox.Enabled = true; SetStatus("DISCONNECTED", "Serial connection closed."); return; }
        if (portBox.SelectedItem is not string port || baudBox.SelectedItem is not string baud) { SetStatus("ERROR", "Select a COM port and baud rate."); return; }
        try { serial.Connect(port, int.Parse(baud, CultureInfo.InvariantCulture)); connectButton.Text = "DISCONNECT"; portBox.Enabled = baudBox.Enabled = false; SetStatus("CONNECTED", $"Connected to {port} at {baud} 8N1."); }
        catch (Exception ex) { SetStatus("ERROR", "Connection failed: " + ex.Message); }
    }

    private void QueueTelemetry(TelemetryData data) { if (!IsDisposed && IsHandleCreated) BeginInvoke(new Action(() => UpdateTelemetry(data))); }
    private void QueueStatus(string state, string message) { if (!IsDisposed && IsHandleCreated) BeginInvoke(new Action(() => SetStatus(state, message))); }
    private void QueueRobotStatus(byte code) { if (!IsDisposed && IsHandleCreated) BeginInvoke(new Action(() => UpdateRobotStatus(code))); }

    // ESP32 status frames (55 AB ...): a trajectory command being accepted is not
    // the same as execution succeeding, so rejection/abort update the UI too.
    private void UpdateRobotStatus(byte code)
    {
        switch (code)
        {
            case 0x21: SetStatus("MOVING", "ESP32 accepted LINE; executing with MControl."); break;
            case 0x31: SetStatus("MOVING", "ESP32 accepted CIRCLE; executing with MControl."); break;
            case 0x22:
            case 0x32:
                streamTimer.Stop(); streamCounts = null;
                StopLogging(false); // rejected trajectory: not a completion
                EndMonitoring("rejected");
                SetStatus("ERROR", "ESP32 rejected the trajectory (outside workspace or invalid parameters).");
                break;
            case 0x40:
                SetStatus("CONNECTED", $"{streamTrajectory} trajectory complete (ESP32 reported DONE).");
                break;
            case 0x50:
                streamTimer.Stop(); streamCounts = null;
                StopLogging(false); // emergency stop: not a successful completion
                EndMonitoring("stopped");
                SetStatus("STOPPED", "ESP32 confirmed STOP; motors are held stopped.");
                break;
            case 0x51:
                // Homing refused because the home/limit switches are not wired.
                homingActive = false; if (homeButton is not null) homeButton.Enabled = true;
                SetStatus("ERROR", "ESP32: HOME unavailable (home/limit switches not wired or verified).");
                break;
            case 0x52:
                homingActive = true; if (homeButton is not null) homeButton.Enabled = false;
                SetStatus("HOMING", "ESP32 confirmed homing is active.");
                break;
            case 0x53:
                homingActive = false; if (homeButton is not null) homeButton.Enabled = true;
                SetStatus("CONNECTED", "ESP32 reported homing complete.");
                break;
            case 0x54:
                // Homing aborted (timeout or switch fault): both motors stopped.
                homingActive = false; if (homeButton is not null) homeButton.Enabled = true;
                SetStatus("ERROR", "ESP32 reported homing failed/aborted (timeout or switch fault); motors stopped.");
                break;
        }
    }
    private void UpdateTelemetry(TelemetryData data)
    {
        var q1 = RobotMath.CountToJoint1Angle(data.Count1); var q2 = RobotMath.CountToJoint2Angle(data.Count2); var (x, y) = RobotMath.ForwardKinematics(q1, q2);
        lastActualCount1 = data.Count1; lastActualCount2 = data.Count2;
        lastTelemetryTicks = Environment.TickCount64;
        // Refresh the desired reference / derivatives / feed-forward for THIS
        // telemetry row's time from the same streamed sample grid.  This keeps
        // the logged desired columns aligned with the telemetry time even when
        // the stream-timer callback was delayed (Windows Forms timers can be
        // starved by the telemetry message queue), so the CSV never records a
        // stale desired snapshot.  During settling streamCounts is null and the
        // final reference is held (already correct).
        if (logActive && streamCounts is not null && !streamGenerationDone)
        {
            var refElapsed = streamClock.Elapsed.TotalSeconds;
            var refU = streamDuration <= 0 ? 1.0 : refElapsed / streamDuration;
            if (refU < 0) refU = 0; else if (refU > 1) refU = 1;
            ComputeReferenceAt(refU, out _, out _, out _, out _, out _);
        }
        var e1 = targetQ1 - q1; var e2 = targetQ2 - q2; var total = e1.HasValue && e2.HasValue ? Math.Sqrt(e1.Value * e1.Value + e2.Value * e2.Value) : (double?)null;
        SetReadout("count1", data.Count1.ToString(CultureInfo.InvariantCulture)); SetReadout("count2", data.Count2.ToString(CultureInfo.InvariantCulture));
        SetReadout("q1", (q1 * 180 / Math.PI).ToString("F3", CultureInfo.InvariantCulture)); SetReadout("q2", (q2 * 180 / Math.PI).ToString("F3", CultureInfo.InvariantCulture));
        // x, y are kept in metres for the robot view and CSV logging; the Live
        // Status readouts are shown in centimetres (m -> cm = *100) with one
        // decimal place.  The internal x/y values are unchanged.
        SetReadout("x", (x * 100.0).ToString("F1", CultureInfo.InvariantCulture)); SetReadout("y", (y * 100.0).ToString("F1", CultureInfo.InvariantCulture)); SetReadout("e1", F(e1)); SetReadout("e2", F(e2)); SetReadout("et", F(total));
        robotView.SetPose(q1, q2);
        var seconds = (data.Timestamp - chartStart).TotalSeconds; count1Chart.AddSample(seconds, data.Count1); count2Chart.AddSample(seconds, data.Count2);
        // Angle charts, on a per-trajectory elapsed time base (seconds).
        // Actual = measured encoder angle, from telemetry only.  Target = the
        // planned joint reference, recorded in StreamTick at each generated
        // reference sample; during the settle phase the reference is held, so it
        // is extended here until CSV logging finishes.  Neither is reconstructed
        // from the other.  Both freeze (retaining data) when the execution ends;
        // a later RUN resets them (BeginErrorRun).
        if (chartActive)
        {
            var runSeconds = (Environment.TickCount64 - runStartTicks) / 1000.0;
            joint1Chart.AddSampleTo("Actual", runSeconds, q1);
            joint2Chart.AddSampleTo("Actual", runSeconds, q2);
            if (streamGenerationDone && targetQ1.HasValue && targetQ2.HasValue)
            {
                joint1Chart.AddSampleTo("Setpoint", runSeconds, targetQ1.Value);
                joint2Chart.AddSampleTo("Setpoint", runSeconds, targetQ2.Value);
            }
        }
        // Actual-signal completion, tracked independently of the desired
        // trajectory and of CSV logging.  Once the desired reference is done the
        // charts stay live (actual keeps plotting) until the measured position is
        // stable for a hold, or a bounded timeout is reached.
        if (chartActive && streamGenerationDone)
        {
            var (mcx, mcy) = MeasuredPointCm();
            if (!monHasLast) { monLastXCm = mcx; monLastYCm = mcy; monHasLast = true; }
            var moved = Math.Sqrt((mcx - monLastXCm) * (mcx - monLastXCm) + (mcy - monLastYCm) * (mcy - monLastYCm));
            monLastXCm = mcx; monLastYCm = mcy;
            if (moved <= PathStartToleranceCm) monStableSamples++; else monStableSamples = 0;
            if (monStableSamples >= MonitorHoldSamples)
            {
                EndMonitoring("actual settled");
                SetStatus("CONNECTED", $"{streamTrajectory} monitoring complete: actual settled.");
            }
            else if (Environment.TickCount64 - monStartTicks > MonitorTimeoutMs)
            {
                EndMonitoring("timeout - actual not settled");
                SetStatus("CONNECTED", $"{streamTrajectory} monitoring stopped: timeout (actual not settled).");
            }
        }
        // CSV logging lifecycle: append rows ONLY while an execution is active.
        // Append first so the final samples are not lost, then evaluate
        // completion (generator done AND measured position settled on the final
        // reference) and stop logging.  Idle telemetry is consequently ignored.
        if (logActive)
        {
            // Logging-only PID/PWM estimates.  These reproduce the ESP32 PID on
            // the COUNT error with the firmware gains, on the PC's own integral/
            // derivative state (never the live controller).  They are NOT sent to
            // the robot and are ESTIMATES, not the ESP32's actual outputs.
            double pidPwm1 = 0, pidPwm2 = 0, tauPid1 = 0, tauPid2 = 0, tauCmd1 = 0, tauCmd2 = 0;
            var havePwm = false;
            var havePidEstimate = false;
            if (targetQ1.HasValue && targetQ2.HasValue)
            {
                var dT = ReferencePeriodMs / 1000.0;
                var targetCount1 = Math.Round(targetQ1.Value * RobotMath.CountsPerRevolution1 / (2.0 * Math.PI));
                var targetCount2 = Math.Round(targetQ2.Value * RobotMath.CountsPerRevolution2 / (2.0 * Math.PI));
                var err1 = targetCount1 - data.Count1;
                var err2 = targetCount2 - data.Count2;
                logPidInt1 += err1 * dT; logPidInt2 += err2 * dT;
                var dErr1 = logPidPrevValid1 ? (err1 - logPidErrPrev1) / dT : 0.0;
                var dErr2 = logPidPrevValid2 ? (err2 - logPidErrPrev2) / dT : 0.0;
                var out1 = PidKp1 * err1 + PidKi1 * logPidInt1 + PidKd1 * dErr1;
                var out2 = PidKp2 * err2 + PidKi2 * logPidInt2 + PidKd2 * dErr2;
                logPidErrPrev1 = err1; logPidErrPrev2 = err2; logPidPrevValid1 = logPidPrevValid2 = true;
                pidPwm1 = Math.Clamp(Math.Round(out1), -PwmMaxDuty, PwmMaxDuty);
                pidPwm2 = Math.Clamp(Math.Round(out2), -PwmMaxDuty, PwmMaxDuty);
                havePwm = true;
                tauPid1 = pidPwm1 / 255.0 * TauScale1Nm;
                tauPid2 = pidPwm2 / 255.0 * TauScale2Nm;
                tauCmd1 = (haveRefTorque ? refTau1 : 0) + tauPid1;
                tauCmd2 = (haveRefTorque ? refTau2 : 0) + tauPid2;
                havePidEstimate = true;
            }

            logger.Log(data, q1, q2, x, y, targetQ1, targetQ2,
                refDq1, refDq2, refDdq1, refDdq2, refTau1, refTau2, haveRefTorque,
                pidPwm1, pidPwm2, havePwm, tauPid1, tauPid2, tauCmd1, tauCmd2, havePidEstimate);
            if (streamGenerationDone && haveFinalTarget)
            {
                var (mx, my) = MeasuredPointCm();
                var dx = mx - finalTargetXCm; var dy = my - finalTargetYCm;
                var distance = Math.Sqrt(dx * dx + dy * dy);
                if (distance <= PathStartToleranceCm) reachedSamples++;
                else reachedSamples = 0;
                if (reachedSamples >= CompletionHoldSamples)
                {
                    StopLogging(true);
                    SetStatus("CONNECTED", $"{streamTrajectory} complete: logging stopped.");
                }
                else if (Environment.TickCount64 - settleStartTicks > SettleTimeoutMs)
                {
                    StopLogging(false);
                    SetStatus("CONNECTED", $"{streamTrajectory} logging stopped (settle timeout).");
                }
            }
        }
        RefreshCsvFiles();
        if (robotState == "CONNECTED") SetStatus("CONNECTED", "Receiving valid encoder telemetry.");
    }

    private void RefreshPorts()
    {
        if (serial.IsConnected) return;
        var selected = portBox.SelectedItem as string;
        var ports = System.IO.Ports.SerialPort.GetPortNames().OrderBy(p => p, StringComparer.OrdinalIgnoreCase).ToArray();
        var espPorts = SerialPortLocator.FindEsp32Ports().Where(p => ports.Contains(p, StringComparer.OrdinalIgnoreCase)).ToArray();

        var itemsChanged = !ports.SequenceEqual(portBox.Items.Cast<string>(), StringComparer.OrdinalIgnoreCase);
        if (itemsChanged) { portBox.Items.Clear(); portBox.Items.AddRange(ports); }

        // Keep the operator's choice if it is still present; otherwise select the
        // single identified CH340 board.  Never fall back to an arbitrary
        // enumerated port (Bluetooth/other), and never guess when the device is
        // absent or ambiguous - the operator must choose explicitly.
        if (selected is not null && ports.Contains(selected, StringComparer.OrdinalIgnoreCase)) portBox.SelectedItem = selected;
        else if (espPorts.Length == 1) portBox.SelectedItem = espPorts[0];
        else if (itemsChanged) portBox.SelectedItem = null;

        string diagnostic;
        if (ports.Length == 0) diagnostic = "No serial ports found. Connect the ESP32 (CH340) USB cable.";
        else if (espPorts.Length == 0) diagnostic = "No ESP32 (CH340) serial device found. Connect the board, then press REFRESH.";
        else if (espPorts.Length > 1) diagnostic = $"Multiple ESP32 (CH340) devices found ({string.Join(", ", espPorts)}); select the correct port manually.";
        else diagnostic = "";
        if (diagnostic != lastPortDiagnostic)
        {
            lastPortDiagnostic = diagnostic;
            if (diagnostic.Length > 0) SetStatus("DISCONNECTED", diagnostic);
        }
    }
    private void SetStatus(string state, string message)
    {
        robotState = state; stateLabel.Text = state; stateLabel.BackColor = state is "ERROR" ? Color.Firebrick : state is "MOVING" ? Color.DarkOrange : state is "DISCONNECTED" ? Color.DimGray : Color.SeaGreen;
        stateLabel.AccessibleDescription = message;
        statusMessage.Text = message;
        statusMessage.ForeColor = state == "ERROR" ? Danger : TextMuted;
        connectionDetail.Text = portBox.SelectedItem is string port ? $"{port}  |  {baudBox.SelectedItem ?? "-"} baud" : "NO SERIAL PORT";
    }
    private void SetReadout(string key, string value) => readouts[key].Text = value;
    private static string F(double? value) => value?.ToString("F6", CultureInfo.InvariantCulture) ?? "";
    private static TextBox Input() => new() { Dock = DockStyle.Fill, TextAlign = HorizontalAlignment.Center, BackColor = SurfaceRaised, ForeColor = TextPrimary, BorderStyle = BorderStyle.FixedSingle, Margin = new Padding(4, 3, 0, 3) };
    private static Label Output() => new() { Dock = DockStyle.Fill, AutoSize = false, TextAlign = ContentAlignment.MiddleRight, ForeColor = TextPrimary, BackColor = SurfaceRaised, Padding = new Padding(6, 0, 6, 0) };
    private static GroupBox Group(string text) => new() { Text = text, Dock = DockStyle.Fill, Height = 210, Padding = new Padding(10), Margin = new Padding(0, 0, 0, 10), ForeColor = Accent, BackColor = Surface };
    private static TableLayoutPanel FormGrid() { var table = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 5 }; table.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize)); table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100)); for (var i = 0; i < 5; i++) table.RowStyles.Add(new RowStyle(SizeType.Absolute, 30)); return table; }
    private static void AddField(TableLayoutPanel table, string label, Control control, int row) { table.Controls.Add(LabelFor(label), 0, row); table.Controls.Add(control, 1, row); }
    private static Label LabelFor(string text) => new() { Text = text, AutoSize = true, Anchor = AnchorStyles.Left, ForeColor = TextMuted, Margin = new Padding(4, 5, 4, 5) };
    private static Button ButtonFor(string text, EventHandler action, Color? color = null) { var button = new Button { Text = text, AutoSize = true, Height = 35, FlatStyle = FlatStyle.Flat, BackColor = color ?? SurfaceRaised, ForeColor = TextPrimary, FlatAppearance = { BorderColor = Border, BorderSize = 1 }, Margin = new Padding(4, 3, 4, -2), Padding = new Padding(8, 0, 8, 0), Font = new Font("Segoe UI", 8F, FontStyle.Bold) }; button.Click += action; return button; }
    private static void WireEnter(Control current, Control next) => current.KeyDown += (_, e) => { if (e.KeyCode == Keys.Enter) { next.Focus(); e.SuppressKeyPress = true; } };
    private static bool TryCoordinate(TextBox box, out int value) { if (int.TryParse(box.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out value)) { box.BackColor = SurfaceRaised; return true; } box.BackColor = Color.FromArgb(89, 47, 52); box.Focus(); return false; }
    [DllImport("user32.dll")] private static extern bool ReleaseCapture();
    [DllImport("user32.dll")] private static extern IntPtr SendMessage(IntPtr handle, int message, int wParam, int lParam);
    private void DragWindow(object? sender, MouseEventArgs e) { if (e.Button == MouseButtons.Left) { ReleaseCapture(); SendMessage(Handle, 0xA1, 2, 0); } }
}

