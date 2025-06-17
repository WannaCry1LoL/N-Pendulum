namespace Pendulum.Math;

public class NPendulum : IDisposable
{
	private const double Gravity = -9.81;
	private const double ArmLength = 500;
	private const float CircleSize = 20;
	private const float CircleOffset = (CircleSize / 2);
	
	private readonly List<PointF> _points = [];
	
	private readonly double[] _thetas;
	private readonly double[] _thetaDots;
	private readonly double[,] _matrix;
	private readonly double[] _vector;
	private readonly PointF[] _positions;
	
	private readonly int _n;
	private readonly double _armlength;
	
	private readonly Pen _pointPen = new(Color.Red, 1);
	private readonly Pen _linePen = new(Color.Black, 3);
	private readonly SolidBrush _circleBrush = new(Color.Red);

	private bool _canAddPoint = true;

	public NPendulum(double[] initialTheta, double[] initialThetaDot)
	{
		_thetas = new double[initialTheta.Length];
		_thetaDots = new double[initialTheta.Length];
		
		_n = initialTheta.Length;
		_armlength = ArmLength / _n;
		_matrix = new double[_n, _n];
		_vector = new double[_n];
		_positions = new PointF[_n + 1];
		
		initialTheta.CopyTo(_thetas);
		initialThetaDot.CopyTo(_thetaDots);
	}
	
	public void ClearPoints() => _points.Clear();
	
	private void PopulateMatrix(double[] thetas)
	{
		for (int i = 0; i < _n; i++)
		{
			var theta1 = thetas[i];
			for (int j = 0; j < _n; j++)
			{
				_matrix[i, j] = (_n - int.Max(i, j)) * double.Cos(theta1 - thetas[j]);
			}
		}
	}
	
	private void PopulateVector(double[] thetas, double[] thetaDots)
	{
		for (int i = 0; i < _n; i++)
		{
			var sum = 0.0;
			var theta = thetas[i];
			for (int j = 0; j < _n; j++)
			{
				var theta2 = thetas[j];
				var theta2dot = thetaDots[j];
				sum -= (_n - int.Max(i, j)) * double.Sin(theta - theta2) * theta2dot * theta2dot;
			}

			sum -= Gravity * (_n - i) * double.Sin(theta);
			_vector[i] = sum;
		}
	}

	private (double[], double[]) F(double[] thetas, double[] thetaDots)
	{
		PopulateMatrix(thetas);
		PopulateVector(thetas, thetaDots);
		var solved = LUSolve.Eliminate(_matrix, _vector);
		return (thetaDots, solved);
	}
	
	private double[] RkShorthand(double[] current, double[] baseValues, double mult)
	{
		var res = new double[_n];
		for (int i = 0; i < _n; i++)
			res[i] = baseValues[i] + mult * current[i];
		return res;
	}
	
	private (double[], double[]) ApplyRk((double[], double[]) current, double[] firstBase, double[] secondBase, double mult)
		=> F(RkShorthand(current.Item1, firstBase, mult), RkShorthand(current.Item2, secondBase, mult));

	private void RungeKutta4(double dt, double[] thetas, double[] thetadots)
	{
		var k1 = F(thetas, thetadots);
		var k2 = ApplyRk(k1, thetas, thetadots, 0.5 * dt);
		var k3 = ApplyRk(k2, thetas, thetadots, 0.5 * dt);
		var k4 = ApplyRk(k3, thetas, thetadots, 1 * dt);
		for (int i = 0; i < _n; i++)
		{
			thetas[i] += dt / 6 * (k1.Item1[i] + (k2.Item1[i] + k3.Item1[i]) * 2 + k4.Item1[i]);
			thetadots[i] += dt / 6 * (k1.Item2[i] + (k2.Item2[i] + k3.Item2[i]) * 2 + k4.Item2[i]);
		}
	}

	public void Update(double dt)
	{
		RungeKutta4(dt, _thetas, _thetaDots);
		_canAddPoint = true;
	}

	private void Positions(PointF initial)
	{
		_positions[0] = initial;
		(double x, double y) = (initial.X, initial.Y);
		for (int i = 0; i < _n; i++)
		{
			var res = double.SinCos(_thetas[i]);
			x += res.Sin * _armlength;
			y -= res.Cos * _armlength;
			_positions[i + 1] = new PointF((float)x, (float)y);
		}
	}
	
	public void Draw(Graphics g, PointF start)
	{
		Positions(start);
		g.DrawLines(_linePen, _positions);
		var last = _positions[^1];
		if (_canAddPoint)
		{
			_points.Add(last);
			_canAddPoint = false;
		}
		if (_points.Count > 1)
			g.DrawLines(_pointPen, _points.ToArray());
		g.FillEllipse(_circleBrush, last.X - CircleOffset, last.Y - CircleOffset, CircleSize, CircleSize);
	}
	public void Dispose()
	{
		_pointPen.Dispose();
		_linePen.Dispose();
		_circleBrush.Dispose();
	}
}