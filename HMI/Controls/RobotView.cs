using RobotHmi.Robot;

namespace RobotHmi.Controls;

internal sealed class RobotView : Control
{
    // Initial pose comes from the shared calculation source (calculate.cpp /
    // firmware initial condition exposed by RobotMath), not a local assumption.
    private double q1 = RobotMath.InitialJoint1Angle;
    private double q2 = RobotMath.InitialJoint2Angle;
    private (double X, double Y)? target;
    private PointF viewportCenter;
    private Size previousClientSize;
    private float zoom = 1f;
    private bool isPanning;
    private Point panStart;

    public RobotView()
    {
        DoubleBuffered = true;
        BackColor = Color.FromArgb(16, 23, 31);
        ForeColor = Color.FromArgb(220, 228, 238);
        MinimumSize = new Size(280, 280);
        Dock = DockStyle.Fill;
        TabStop = true;
    }

    public void SetPose(double joint1, double joint2)
    {
        q1 = joint1; q2 = joint2; Invalidate();
    }

    // The TARGET marker is driven by the commanded Cartesian coordinate (the
    // POINT input), not by live telemetry, so it stays visible across repaints,
    // CSV loads and while the robot is still moving toward it.
    public void SetTarget((double X, double Y)? targetPosition)
    {
        target = targetPosition; Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        var g = e.Graphics;
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        EnsureViewportCenter();
        var center = viewportCenter;
        var scale = BaseScale() * zoom;
        PointF Map(double x, double y) => new(center.X + (float)(x * scale), center.Y - (float)(y * scale));
        using var grid = new Pen(Color.FromArgb(35, 52, 66), 1);
        var gridSpacing = GridSpacing(scale);
        var xStart = center.X % gridSpacing;
        var yStart = center.Y % gridSpacing;
        for (var x = xStart; x < Width; x += gridSpacing) g.DrawLine(grid, x, 0, x, Height);
        for (var y = yStart; y < Height; y += gridSpacing) g.DrawLine(grid, 0, y, Width, y);
        using var axis = new Pen(Color.FromArgb(88, 128, 145), 1.4f);
        g.DrawLine(axis, 0, center.Y, Width, center.Y); g.DrawLine(axis, center.X, 0, center.X, Height);
        if (center.Y >= 0 && center.Y < Height) TextRenderer.DrawText(g, "X", Font, new Point(Width - 18, Math.Clamp((int)center.Y + 4, 0, Height - Font.Height)), Color.FromArgb(135, 190, 210));
        if (center.X >= 0 && center.X < Width) TextRenderer.DrawText(g, "Y", Font, new Point(Math.Clamp((int)center.X + 4, 0, Width - 18), 4), Color.FromArgb(135, 190, 210));

        var elbow = RobotMath.Joint1Position(q1);
        var endEffector = RobotMath.ForwardKinematics(q1, q2);
        var joint2 = Map(elbow.X, elbow.Y);
        var end = Map(endEffector.X, endEffector.Y);
        using var link1 = new Pen(Color.FromArgb(55, 185, 235), 8) { StartCap = System.Drawing.Drawing2D.LineCap.Round, EndCap = System.Drawing.Drawing2D.LineCap.Round };
        using var link2 = new Pen(Color.FromArgb(75, 215, 165), 8) { StartCap = System.Drawing.Drawing2D.LineCap.Round, EndCap = System.Drawing.Drawing2D.LineCap.Round };
        using var jointBrush = new SolidBrush(Color.FromArgb(235, 242, 250));
        using var endBrush = new SolidBrush(Color.FromArgb(255, 105, 100));
        g.DrawLine(link1, center, joint2); g.DrawLine(link2, joint2, end);
        g.FillEllipse(jointBrush, center.X - 7, center.Y - 7, 14, 14); g.FillEllipse(jointBrush, joint2.X - 6, joint2.Y - 6, 12, 12); g.FillEllipse(endBrush, end.X - 6, end.Y - 6, 12, 12);
        TextRenderer.DrawText(g, "J1", Font, new Point((int)center.X + 8, (int)center.Y + 8), ForeColor);
        TextRenderer.DrawText(g, "J2", Font, new Point((int)joint2.X + 8, (int)joint2.Y + 8), ForeColor);
        TextRenderer.DrawText(g, "END EFFECTOR", Font, new Point((int)end.X + 8, (int)end.Y + 8), ForeColor);
        // Draw the TARGET marker last so it can never be hidden by the links,
        // joint or end-effector overlay when the robot reaches the target.
        if (target is { } point)
        {
            var p = Map(point.X, point.Y);
            using var targetPen = new Pen(Color.FromArgb(255, 190, 70), 2) { DashStyle = System.Drawing.Drawing2D.DashStyle.Dash };
            g.DrawEllipse(targetPen, p.X - 7, p.Y - 7, 14, 14);
            TextRenderer.DrawText(g, "TARGET", Font, new Point((int)p.X + 8, (int)p.Y - 10), Color.FromArgb(255, 205, 100));
        }
    }

    protected override void OnResize(EventArgs e)
    {
        base.OnResize(e);
        if (previousClientSize.Width > 0 && previousClientSize.Height > 0)
        {
            viewportCenter.X += (ClientSize.Width - previousClientSize.Width) / 2f;
            viewportCenter.Y += (ClientSize.Height - previousClientSize.Height) / 2f;
        }
        previousClientSize = ClientSize;
    }

    protected override void OnMouseEnter(EventArgs e)
    {
        base.OnMouseEnter(e);
        Focus();
    }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        base.OnMouseWheel(e);
        EnsureViewportCenter();
        var oldScale = BaseScale() * zoom;
        var worldX = (e.X - viewportCenter.X) / oldScale;
        var worldY = (viewportCenter.Y - e.Y) / oldScale;
        zoom = Math.Clamp(zoom * (e.Delta > 0 ? 1.15f : 1f / 1.15f), .45f, 4f);
        var newScale = BaseScale() * zoom;
        viewportCenter = new PointF(e.X - worldX * newScale, e.Y + worldY * newScale);
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        if (e.Button != MouseButtons.Middle) return;
        EnsureViewportCenter();
        isPanning = true;
        panStart = e.Location;
        Capture = true;
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (!isPanning) return;
        viewportCenter.X += e.X - panStart.X;
        viewportCenter.Y += e.Y - panStart.Y;
        panStart = e.Location;
        Invalidate();
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);
        if (e.Button != MouseButtons.Middle) return;
        isPanning = false;
        Capture = false;
    }

    private void EnsureViewportCenter()
    {
        if (viewportCenter != PointF.Empty) return;
        viewportCenter = new PointF(ClientSize.Width / 2f, ClientSize.Height / 2f + ClientSize.Height * .08f);
        previousClientSize = ClientSize;
    }

    private float BaseScale() => Math.Max(1f, Math.Min(Width, Height) * .38f / (float)(RobotMath.Link1Metres + RobotMath.Link2Metres));

    private static float GridSpacing(float scale)
    {
        var desiredWorldStep = 36f / scale;
        var exponent = MathF.Floor(MathF.Log10(desiredWorldStep));
        var power = MathF.Pow(10, exponent);
        var normalized = desiredWorldStep / power;
        var step = normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10;
        return step * power * scale;
    }
}
