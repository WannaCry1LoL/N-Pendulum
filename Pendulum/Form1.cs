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
	
	private readonly Stopwatch _stopwatch = new();
	private readonly CancellationTokenSource _cts = new();
	public Form1(int amount)
	{
		var pendulum = NPendulumBuilder.Heart;
		
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
			_cts.Cancel();
			_stopwatch.Stop();
			pendulum.Dispose();
		};
		
		_mainArea.Paint += (sender, args) =>
		{
			using var bitGraphics = Graphics.FromImage(_bitmap);
			var g = args.Graphics;
			g.SmoothingMode = SmoothingMode.AntiAlias;
			bitGraphics.SmoothingMode = SmoothingMode.AntiAlias;
			pendulum.Draw(g, bitGraphics,
				new PointF(_mainArea.ClientSize.Width / 2.0f, _mainArea.ClientSize.Height / 2.0f));
		};

		Task.Run(() =>
		{
			var task = _cts.Token;
			while (!task.IsCancellationRequested)
			{
				pendulum.Update(0.001);
				if (_mainArea.IsHandleCreated)
					_mainArea.BeginInvoke(() => _mainArea.Invalidate());
				Thread.Sleep(10);
			}
		});
		
		_mainArea.Size = ClientSize;
		_bitmap = new Bitmap(_mainArea.ClientSize.Width, _mainArea.ClientSize.Height);
		_mainArea.BackgroundImage = _bitmap;
		_lastUpdate = 0;
		_stopwatch.Start();
		Controls.Add(_mainArea);
		
	}
}