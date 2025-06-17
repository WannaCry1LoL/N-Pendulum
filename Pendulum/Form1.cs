using System.Diagnostics;

namespace Pendulum;
using Pendulum.Math;
public partial class Form1 : Form
{
	private Panel MainArea = new DoubleBufferedPanel();
	private NPendulum pendulum = new NPendulum();
	private DateTime lastUpdate;
	public Form1()
	{
		var timer = new System.Windows.Forms.Timer()
		{
			Interval = 1
		};
		timer.Tick += (sender, args) =>
		{
			var dt = DateTime.Now - lastUpdate;
			pendulum.Update(dt.TotalMilliseconds / 2000.0 );
			lastUpdate = DateTime.Now;
			MainArea.Invalidate();
		};
		pendulum
			.AddN(2, 0.45 * double.Pi, 0.0);
		InitializeComponent();
		ResizeRedraw = true;
		Resize += (sender, args) =>
		{
			MainArea.Size = ClientSize;
		};
		MainArea.Paint += (sender, args) =>
		{
			pendulum.Draw(args.Graphics, new PointF(MainArea.ClientSize.Width / 2, MainArea.ClientSize.Height / 2));
		};
		Controls.Add(MainArea);
		lastUpdate = DateTime.Now;
		timer.Start();
	}
}