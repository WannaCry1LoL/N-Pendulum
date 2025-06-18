namespace Pendulum;
using Timer = System.Windows.Forms.Timer;
using System.Drawing.Drawing2D;
using System.Diagnostics;
using Math;
public partial class Form1 : Form
{
	private readonly Panel _mainArea = new DoubleBufferedPanel();
	private Bitmap _bitmap;
	private long _lastUpdate;
	private readonly Timer _timer = new()
	{
		Interval = 1
	};
	private readonly Stopwatch _stopwatch = new();
	
	public Form1(int amount)
	{
		var pendulum = NPendulumBuilder
			.Create()
			.AddNRandom(amount, NumberRange<double>.FromVariation(0, 0.5 * double.Pi))
			.Build();
		
		InitializeComponent();
		
		ResizeRedraw = true;
		
		Resize += (sender, args) =>
		{
			_bitmap?.Dispose();
			_mainArea.Size = ClientSize;
			_bitmap = new Bitmap(_mainArea.ClientSize.Width, _mainArea.ClientSize.Height);
			_mainArea.BackgroundImage = _bitmap;
			pendulum.ClearPoints();
		};
		
		FormClosing += (sender, args) =>
		{
			_timer.Stop();
			pendulum.Dispose();
		};

		_mainArea.Paint += (sender, args) =>
		{
			using var bitGraphics = Graphics.FromImage(_bitmap);
			var g = args.Graphics;
			g.SmoothingMode = SmoothingMode.AntiAlias;
			pendulum.Draw(g, bitGraphics,
				new PointF(_mainArea.ClientSize.Width / 2.0f, _mainArea.ClientSize.Height / 2.0f));
		};
		
		_timer.Tick += (sender, args) =>
		{
			var dt = _stopwatch.ElapsedMilliseconds - _lastUpdate;
			pendulum.Update(dt / 2000.0);
			_lastUpdate = _stopwatch.ElapsedMilliseconds;
			_mainArea.Invalidate();
		};
		
		_mainArea.Size = ClientSize;
		_bitmap = new Bitmap(_mainArea.ClientSize.Width, _mainArea.ClientSize.Height);
		_mainArea.BackgroundImage = _bitmap;
		_lastUpdate = 0;
		_timer.Start();
		_stopwatch.Start();
		Controls.Add(_mainArea);
		
	}
}