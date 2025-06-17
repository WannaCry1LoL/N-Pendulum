namespace Pendulum;
using System.Diagnostics;
using Math;
using Timer = System.Windows.Forms.Timer;
public partial class Form1 : Form
{
	private readonly Panel _mainArea = new DoubleBufferedPanel();
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
			_mainArea.Size = ClientSize;
			pendulum.ClearPoints();
		};
		
		FormClosing += (sender, args) =>
		{
			_timer.Stop();
			pendulum.Dispose();
		};
		
		_mainArea.Paint += (sender, args) =>
			pendulum.Draw(args.Graphics, new PointF(_mainArea.ClientSize.Width / 2.0f, _mainArea.ClientSize.Height / 2.0f));
		
		_timer.Tick += (sender, args) =>
		{
			var dt = _stopwatch.ElapsedMilliseconds - _lastUpdate;
			pendulum.Update(dt / 2000.0);
			_lastUpdate = _stopwatch.ElapsedMilliseconds;
			_mainArea.Invalidate();
		};
		
		_lastUpdate = 0;
		_timer.Start();
		_stopwatch.Start();
		Controls.Add(_mainArea);
		
		_mainArea.Size = ClientSize;
	}
}