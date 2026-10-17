using System.Globalization;

namespace RobotHmi.Charts;

internal sealed class RealtimeChart : Control
{
    // Single-series rolling buffer (encoder-count charts).
    private readonly Queue<(double Time, double Value)> values = new();

    // Multi-series history (tracking-error charts).  Each trajectory run adds a
    // new series; previous runs are retained.
    private readonly List<Series> seriesList = new();
    private Series? currentSeries;

    private bool liveMode = true;
    private string emptyMessage = "AWAITING TELEMETRY";
    private float viewportZoom = 1f;
    private PointF viewportOffset;
    private bool isPanning;
    private Point panStart;
    public string ChartTitle { get; init; } = "Signal";
    public string Unit { get; init; } = "";
    public double WindowSeconds { get; init; } = 10;

    // When true the chart keeps every run as a separate coloured series instead
    // of a single rolling window.
    public bool MultiSeries { get; init; }

    private sealed class Series
    {
        public Series(string name, Color color) { Name = name; Color = color; }
        public string Name { get; }
        public Color Color { get; }
        public List<(double Time, double Value)> Points { get; } = new();
    }

    private static readonly Color[] Palette =
    {
        Color.FromArgb(62, 190, 190), Color.FromArgb(224, 158, 66), Color.FromArgb(198, 108, 178),
        Color.FromArgb(126, 200, 96), Color.FromArgb(96, 150, 226), Color.FromArgb(228, 96, 96),
        Color.FromArgb(200, 190, 90), Color.FromArgb(150, 130, 220)
    };

    // Slightly smaller font for the X-axis tick labels and the TIME (S) caption
    // so the whole time axis fits inside the (short) chart cell without clipping.
    private static readonly Font AxisFont = new("Segoe UI", 7.5f, FontStyle.Regular, GraphicsUnit.Point);

    public RealtimeChart()
    {
        DoubleBuffered = true;
        BackColor = Color.FromArgb(16, 23, 31);
        ForeColor = Color.FromArgb(218, 228, 236);
        Margin = new Padding(6);
        MinimumSize = new Size(220, 140);
        TabStop = true;
    }

    // Start a new named series (multi-series charts).  Previous series remain.
    public void BeginSeries(string name)
    {
        if (!MultiSeries) { ResumeLive(); return; }
        currentSeries = new Series(name, Palette[seriesList.Count % Palette.Length]);
        seriesList.Add(currentSeries);
        liveMode = false;
        Invalidate();
    }

    public void AddSample(double time, double value)
    {
        if (MultiSeries)
        {
            if (currentSeries is null) BeginSeries("run");
            currentSeries!.Points.Add((time, value));
            Invalidate();
            return;
        }
        values.Enqueue((time, value));
        if (liveMode)
            while (values.Count > 0 && values.Peek().Time < time - WindowSeconds) values.Dequeue();
        Invalidate();
    }

    // Append a sample to a named series, creating it on first use.  Used to plot
    // two aligned series (e.g. desired vs actual) inside one chart.
    public void AddSampleTo(string seriesName, double time, double value)
    {
        var target = seriesList.FirstOrDefault(s => s.Name == seriesName);
        if (target is null)
        {
            target = new Series(seriesName, Palette[seriesList.Count % Palette.Length]);
            seriesList.Add(target);
        }
        target.Points.Add((time, value));
        Invalidate();
    }

    // Clear every series so a new execution starts a fresh dataset.  Existing
    // traces are preserved until this is called (i.e. until the next run starts).
    public void ResetSeries()
    {
        seriesList.Clear();
        currentSeries = null;
        Invalidate();
    }

    // Replace (single-series) or append as a named series (multi-series).
    public void LoadSeries(IReadOnlyList<(double Time, double Value)> series)
    {
        if (MultiSeries)
        {
            var named = new Series($"csv #{seriesList.Count + 1}", Palette[seriesList.Count % Palette.Length]);
            named.Points.AddRange(series);
            seriesList.Add(named);
            currentSeries = named;
            liveMode = false;
            Invalidate();
            return;
        }
        liveMode = false;
        values.Clear();
        foreach (var sample in series) values.Enqueue(sample);
        Invalidate();
    }

    public void ShowEmpty(string message)
    {
        if (MultiSeries)
        {
            if (seriesList.Count == 0) emptyMessage = message;
            Invalidate();
            return;
        }
        liveMode = true;
        values.Clear();
        emptyMessage = message;
        Invalidate();
    }

    public void ResumeLive()
    {
        if (MultiSeries)
        {
            // Never destroy history; a new run simply starts another series.
            liveMode = false;
            return;
        }
        liveMode = true;
        emptyMessage = "AWAITING TELEMETRY";
        values.Clear();
        Invalidate();
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        var graphics = e.Graphics;
        graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        var titleBounds = new Rectangle(4, 2, Math.Max(0, Width - 8), 22);
        TextRenderer.DrawText(graphics, ChartTitle, Font, titleBounds, ForeColor,
            TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.SingleLine | TextFormatFlags.EndEllipsis);
        var plot = PlotBounds();
        using var axis = new Pen(Color.FromArgb(93, 113, 128));
        graphics.DrawRectangle(axis, plot);

        // Collect the samples to draw.
        List<(double Time, double Value, Color Color)> draws = new();
        if (MultiSeries)
        {
            foreach (var s in seriesList)
                foreach (var p in s.Points) draws.Add((p.Time, p.Value, s.Color));
        }
        else
        {
            foreach (var p in values) draws.Add((p.Time, p.Value, Color.FromArgb(62, 190, 190)));
        }

        if (draws.Count == 0)
        {
            TextRenderer.DrawText(graphics, emptyMessage, Font, plot, Color.FromArgb(116, 137, 151),
                TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.SingleLine | TextFormatFlags.EndEllipsis);
            return;
        }

        var earliest = draws.Min(p => p.Time);
        var newest = draws.Max(p => p.Time);
        var oldest = !MultiSeries && liveMode ? Math.Max(0, newest - WindowSeconds) : earliest;
        var window = !MultiSeries && liveMode ? WindowSeconds : Math.Max(newest - earliest, 1e-9);
        var min = draws.Min(p => p.Value); var max = draws.Max(p => p.Value);
        if (Math.Abs(max - min) < 1e-9) { max += 1; min -= 1; }
        var padding = (max - min) * .1; min -= padding; max += padding;
        // Background grid drawn behind the data.  Its vertical lines are drawn at
        // exactly the same pixel positions as the time tick marks (one shared
        // time->X mapping), so ticks and grid always coincide.
        DrawGrid(graphics, plot, oldest, window);
        PointF ToPoint(double time, double value) => new(
            (float)(plot.Left + (time - oldest) / window * plot.Width),
            (float)(plot.Bottom - (value - min) / (max - min) * plot.Height));

        var state = graphics.Save();
        graphics.SetClip(plot);
        graphics.Transform = new System.Drawing.Drawing2D.Matrix(
            viewportZoom, 0, 0, viewportZoom, viewportOffset.X, viewportOffset.Y);
        if (MultiSeries)
        {
            foreach (var s in seriesList)
            {
                if (s.Points.Count < 2) continue;
                using var line = new Pen(s.Color, 1.6f);
                graphics.DrawLines(line, s.Points.Select(p => ToPoint(p.Time, p.Value)).ToArray());
            }
        }
        else if (draws.Count > 1)
        {
            using var line = new Pen(Color.FromArgb(62, 190, 190), 1.8f);
            graphics.DrawLines(line, draws.Select(p => ToPoint(p.Time, p.Value)).ToArray());
        }
        graphics.Restore(state);

        var labelBounds = new Rectangle(2, plot.Top, Math.Max(0, plot.Left - 5), Font.Height);
        TextRenderer.DrawText(graphics, $"{max:G4} {Unit}", Font, labelBounds, Color.FromArgb(145, 160, 176), TextFormatFlags.Right | TextFormatFlags.EndEllipsis);
        labelBounds.Y = plot.Bottom - Font.Height;
        TextRenderer.DrawText(graphics, $"{min:G4}", Font, labelBounds, Color.FromArgb(145, 160, 176), TextFormatFlags.Right | TextFormatFlags.EndEllipsis);
        // X axis: elapsed time in seconds with adaptive tick spacing (recomputed
        // on each paint, so it follows zoom, pan and resize).
        DrawTimeAxis(graphics, plot, oldest, window);
        TextRenderer.DrawText(graphics, "TIME (S)", AxisFont, new Rectangle(plot.Left, plot.Bottom + 4 + AxisFont.Height + 2, plot.Width, AxisFont.Height), Color.FromArgb(145, 160, 176), TextFormatFlags.HorizontalCenter | TextFormatFlags.SingleLine);

        if (MultiSeries && seriesList.Count > 0) DrawLegend(graphics, plot);
    }

    // One authoritative time->X mapping for everything (series, grid, ticks).
    // It accounts for the zoom/pan viewport transform so data and axis stay
    // aligned.  Returns the visible ticks (time value + pixel X).
    private static float TimeToX(double t, Rectangle plot, double oldest, double window, float zoom, PointF offset)
        => (float)(zoom * (plot.Left + (t - oldest) / window * plot.Width) + offset.X);

    private List<(double T, float X)> VisibleTimeTicks(Rectangle plot, double oldest, double window, out double step)
    {
        var ticks = new List<(double, float)>();
        step = 1.0;
        if (plot.Width <= 0 || !(window > 0)) return ticks;
        float ScreenXToTime(float sx) => (float)(oldest + (((sx - viewportOffset.X) / viewportZoom) - plot.Left) / plot.Width * window);
        var tLeft = ScreenXToTime(plot.Left);
        var tRight = ScreenXToTime(plot.Right);
        if (tRight < tLeft) (tLeft, tRight) = (tRight, tLeft);
        var span = (double)tRight - tLeft;
        if (!(span > 0) || !double.IsFinite(span)) return ticks;
        step = NiceTimeStep(span, plot.Width);
        for (var t = Math.Ceiling(tLeft / step) * step; t <= tRight + 1e-9; t += step)
        {
            var sx = TimeToX(t, plot, oldest, window, viewportZoom, viewportOffset);
            if (sx < plot.Left - 0.5f || sx > plot.Right + 0.5f) continue;
            ticks.Add((t, sx));
        }
        return ticks;
    }

    // Background grid; vertical lines share the tick X positions exactly.
    private void DrawGrid(Graphics graphics, Rectangle plot, double oldest, double window)
    {
        using var grid = new Pen(Color.FromArgb(42, 57, 70));
        foreach (var (_, sx) in VisibleTimeTicks(plot, oldest, window, out _))
            graphics.DrawLine(grid, sx, plot.Top, sx, plot.Bottom);
        for (var i = 1; i <= 4; i++)
        {
            var y = plot.Top + plot.Height * i / 5f;
            graphics.DrawLine(grid, plot.Left, y, plot.Right, y);
        }
    }

    // Draw the numeric time tick marks and labels in seconds, at the same X
    // positions used by the grid (so labels line up with the vertical grid).
    private void DrawTimeAxis(Graphics graphics, Rectangle plot, double oldest, double window)
    {
        var ticks = VisibleTimeTicks(plot, oldest, window, out var step);
        if (ticks.Count == 0) return;
        var decimals = step < 0.5 ? 2 : step < 1.0 ? 1 : 0;
        using var tickPen = new Pen(Color.FromArgb(93, 113, 128));
        var labelColor = Color.FromArgb(145, 160, 176);
        var lastRight = double.NegativeInfinity;
        foreach (var (t, sx) in ticks)
        {
            graphics.DrawLine(tickPen, sx, plot.Bottom, sx, plot.Bottom + 4);
            var text = t.ToString("F" + decimals, CultureInfo.InvariantCulture);
            var size = TextRenderer.MeasureText(text, AxisFont);
            var x = sx - size.Width / 2.0;
            if (x < plot.Left) x = plot.Left;
            if (x > lastRight + 4)
            {
                TextRenderer.DrawText(graphics, text, AxisFont, new Rectangle((int)x, plot.Bottom + 4, size.Width + 2, AxisFont.Height),
                    labelColor, TextFormatFlags.NoPadding);
                lastRight = x + size.Width;
            }
        }
    }

    // A "nice" 1/2/5x10^n tick interval giving roughly one label per ~70 px.
    private static double NiceTimeStep(double span, int widthPixels)
    {
        var target = Math.Max(2, widthPixels / 70);
        var raw = span / target;
        var magnitude = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        var normalized = raw / magnitude;
        var nice = normalized <= 1 ? 1.0 : normalized <= 2 ? 2.0 : normalized <= 5 ? 5.0 : 10.0;
        return nice * magnitude;
    }

    private void DrawLegend(Graphics graphics, Rectangle plot)
    {
        var y = plot.Top + 4;
        // Legend text box widened so series labels are not ellipsized; kept
        // inside the plot area.
        var legendWidth = Math.Min(140, Math.Max(60, plot.Width / 3));
        var swatchX = plot.Right - legendWidth - 4;
        foreach (var s in seriesList)
        {
            if (s.Points.Count == 0) continue;
            using var swatch = new SolidBrush(s.Color);
            graphics.FillRectangle(swatch, swatchX, y + 3, 8, 8);
            TextRenderer.DrawText(graphics, s.Name, Font, new Rectangle(swatchX + 12, y, legendWidth - 12, 14),
                Color.FromArgb(200, 212, 222), TextFormatFlags.Left | TextFormatFlags.EndEllipsis);
            y += 14;
            if (y > plot.Bottom - 14) break;
        }
    }

    protected override void OnMouseEnter(EventArgs e)
    {
        base.OnMouseEnter(e);
        Focus();
    }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        base.OnMouseWheel(e);
        if (!PlotBounds().Contains(e.Location)) return;
        var nextZoom = Math.Clamp(viewportZoom * (e.Delta > 0 ? 1.15f : 1f / 1.15f), .5f, 3f);
        var ratio = nextZoom / viewportZoom;
        viewportOffset = new PointF(e.X - ratio * (e.X - viewportOffset.X), e.Y - ratio * (e.Y - viewportOffset.Y));
        viewportZoom = nextZoom;
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        if (e.Button != MouseButtons.Middle || !PlotBounds().Contains(e.Location)) return;
        isPanning = true;
        panStart = e.Location;
        Capture = true;
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);
        if (!isPanning) return;
        viewportOffset.X += e.X - panStart.X;
        viewportOffset.Y += e.Y - panStart.Y;
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

    // Reserve the bottom strip from the axis font metrics (tick marks + two text
    // rows + padding) so the time axis always fits, at any control size.
    private int AxisBlockHeight => 2 * AxisFont.Height + 8;

    private Rectangle PlotBounds() => Rectangle.FromLTRB(58, 28, Math.Max(59, Width - 12), Math.Max(29, Height - AxisBlockHeight));
}
