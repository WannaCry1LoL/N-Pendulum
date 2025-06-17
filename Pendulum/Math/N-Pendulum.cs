namespace Pendulum.Math;

public class NPendulum : IDisposable
{
	private const double g = -9.81; 
	private List<Pendulum> _pendulums = [];
	private List<PointF> _points = [];
	private Pen PointPen = new Pen(Color.Red, 1);
	private Pen p = new(Color.Black, 3);
	private SolidBrush b = new(Color.Red);
	public NPendulum() { }
	
	public NPendulum Add(Pendulum a)
	{
		_pendulums.Add(a);
		return this;
	}

	public NPendulum Add(double theta, double thetaDot)
	{
		_pendulums.Add(new Pendulum(1.0, 1.0, theta, thetaDot));
		return this;
	}

	public NPendulum AddN(int n, double theta, double thetaDot)
	{
		for (int i = 0; i < n; i++)
		{
			Add(theta, thetaDot);
		}
		return this;
	}

	public NPendulum Pop(out Pendulum popped)
	{
		if (_pendulums.Count == 0) throw new InvalidOperationException("Cannot pop from empty list");
		popped = _pendulums[^1];
		_pendulums.RemoveAt(_pendulums.Count - 1);
		return this;
	}

	public NPendulum AddRange(params Pendulum[] values)
	{
		_pendulums.AddRange(values);
		return this;
	}

	private static double[,] CreateMatrix(double[] thetas)
	{
		var n = thetas.Length;
		var mat = new double[n, n];
		for (int i = 0; i < n; i++)
		{
			var theta1 = thetas[i];
			for (int j = 0; j < n; j++)
			{
				mat[i, j] = (n - int.Max(i, j)) * double.Cos(theta1 - thetas[j]);
			}
		}
		return mat;
	}
	
	private static double[] CreateVector(double[] thetas, double[] thetaDots)
	{
		var n = thetas.Length;
		var res = new double[n];
		for (int i = 0; i < n; i++)
		{
			var sum = 0.0;
			var theta = thetas[i];
			for (int j = 0; j < n; j++)
			{
				var theta2 = thetas[j];
				var theta2dot = thetaDots[j];
				sum -= (n - int.Max(i, j)) * double.Sin(theta - theta2) * theta2dot * theta2dot;
			}

			sum -= g * (n - i) * double.Sin(theta);
			res[i] = sum;
		}
		return res;
	}

	private static (double[], double[]) F(double[] thetas, double[] thetadots)
	{
		var A = CreateMatrix(thetas);
		var b = CreateVector(thetas, thetadots);
		var solved = LUSolve.Eliminate(A, b);
		return (thetadots, solved);
	}

	private static double[] RkShorthand(double[] current, double[] baseValues, double mult) => 
		current
			.Select((x, i) => baseValues[i] + x * mult)
			.ToArray();

	private static (double[], double[]) ApplyRk((double[], double[]) current, double[] firstBase, double[] secondBase, double mult)
		=> F(RkShorthand(current.Item1, firstBase, mult), RkShorthand(current.Item2, secondBase, mult));

	private static (double[] NewThetas, double[] NewThetaDots) RungeKutta4(double dt, double[] thetas, double[] thetadots)
	{
		var k1 = F(thetas, thetadots);
		var k2 = ApplyRk(k1, thetas, thetadots, 0.5 * dt);
		var k3 = ApplyRk(k2, thetas, thetadots, 0.5 * dt);
		var k4 = ApplyRk(k3, thetas, thetadots, 1 * dt);
		var newThetas = thetas.Select((x, i) => x + dt / 6 * (k1.Item1[i] + (k2.Item1[i] + k3.Item1[i]) * 2 + k4.Item1[i])).ToArray();
		var newThetaDots = thetadots.Select((x, i) => x + dt / 6 * (k1.Item2[i] + (k2.Item2[i] + k3.Item2[i]) * 2 + k4.Item2[i])).ToArray();
		
		return (newThetas, newThetaDots);
	}

	public void Update(double dt)
	{
		var thetas = _pendulums.Select(x => x.Theta).ToArray();
		var thetadots = _pendulums.Select(x => x.ThetaDot).ToArray();
		var res = RungeKutta4(dt, thetas, thetadots);
		for (int i = 0; i < _pendulums.Count; i++)
		{
			var pendulum = _pendulums[i];
			pendulum.Theta = res.NewThetas[i];
			pendulum.ThetaDot = res.NewThetaDots[i];
		}
	}
	
	private const double ArmLength = 500;
	private const double CircleSize = 20;

	private PointF[] Positions(PointF initial)
	{
		var result = new PointF[_pendulums.Count + 1];
		result[0] = initial;
		(double x, double y) = (initial.X, initial.Y);
		var actuallength = ArmLength / _pendulums.Count;
		for (int i = 0; i < _pendulums.Count; i++)
		{
			var res = double.SinCos(_pendulums[i].Theta);
			x += res.Sin * actuallength;
			y -= res.Cos * actuallength;
			result[i + 1] = new PointF((float)x, (float)y);
		}
		return result;
	}
	public void Draw(Graphics g, PointF start)
	{
		var pos = Positions(start);
		g.DrawLines(p, pos);
		var last = pos[^1];
		_points.Add(last);
		if (_points.Count > 1)
			g.DrawLines(PointPen, _points.ToArray());
		g.FillEllipse(b, last.X - (int)(CircleSize / 2), last.Y - (int)(CircleSize / 2), (int)CircleSize, (int)CircleSize);
	}
	public void Dispose()
	{
		PointPen.Dispose();
		p.Dispose();
		b.Dispose();
	}
}