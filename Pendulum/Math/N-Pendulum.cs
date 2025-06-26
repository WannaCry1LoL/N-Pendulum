#define RUNGEKUTTA

using System.Runtime.CompilerServices;

namespace Pendulum.Math;

#if !(RUNGEKUTTA || LEAPFROG || SYMEULER)
#error Define a solver method
#endif

public class NPendulum : IDisposable
{
	private const double Gravity = -9.81;
	private const double ArmLength = 500;
	private const float CircleSize = 20;
	private const float CircleOffset = (CircleSize / 2);
	
	private PointF? _previous;
	
	private readonly double[] _thetas;
	private readonly double[] _thetaDots;
	private readonly double[,] _matrix;
	private readonly double[] _vector;
	private readonly PointF[] _positions;
#if RUNGEKUTTA
	private readonly double[][] _rkBuffers;
	private readonly double[][] _rkSolutionBuffers;
#elif LEAPFROG	
	private readonly double[] _accelBuffer;
	private readonly double[] _newAccelBuffer;
#elif SYMEULER
	private readonly double[] _solutionBuffer;
#endif
	private readonly int _n;
	private readonly double _armlength;
	
	private readonly Pen _pointPen = Pens.Red;
	private readonly Pen _linePen = new(Color.Black, 3);
	private readonly SolidBrush _circleBrush = new(Color.Red);

	private readonly NxNLUSolver _luSolver;

	public NPendulum(double[] initialTheta, double[] initialThetaDot)
	{
		_thetas = new double[initialTheta.Length];
		_thetaDots = new double[initialTheta.Length];
		
		_n = initialTheta.Length;
		_armlength = ArmLength / _n;
		_matrix = new double[_n, _n];
		_vector = new double[_n];
		_positions = new PointF[_n + 1];
		_luSolver = new NxNLUSolver(_n);
#if RUNGEKUTTA
		_rkBuffers = Utils.CreateFixedSizeBuffers<double>(4, _n);
		_rkSolutionBuffers = Utils.CreateFixedSizeBuffers<double>(4, _n);
#elif LEAPFROG
		_accelBuffer = new double[_n];
		_newAccelBuffer = new double[_n];
#elif SYMEULER
		_solutionBuffer = new double[_n];
#endif
		
		initialTheta.CopyTo(_thetas);
		initialThetaDot.CopyTo(_thetaDots);
	}
	
	public void ClearPoints() => _previous = null;

	private void PopulateMatrix(double[] thetas)
	{
		Parallel.For(0, _n, i =>
		{
			var theta1 = thetas[i];
			for (int j = 0; j < _n; j++)
			{
				_matrix[i, j] = (_n - int.Max(i, j)) * double.Cos(theta1 - thetas[j]);
			}
		});
	}
	
	private void PopulateVector(double[] thetas, double[] thetaDots)
	{
		Parallel.For(0, _n, (i) =>
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
		});
	}

	private void Populate(double[] thetas, double[] thetaDots)
	{
		Parallel.For(0, _n, i =>
		{
			var sum = 0.0;
			var theta = thetas[i];
			for (int j = 0; j < _n; j++)
			{
				var theta2 = thetas[j];
				var theta2dot = thetaDots[j];
				var res = double.SinCos(theta - theta2);
				var weird = _n - int.Max(i, j);
				sum -= weird * res.Sin * theta2dot * theta2dot;
				_matrix[i, j] = weird * res.Cos;
			}
			
			sum -= Gravity * (_n - i) * double.Sin(theta);
			_vector[i] = sum;
		});
	}
	
#if RUNGEKUTTA
	private (double[], double[]) F(double[] thetas, double[] thetaDots, double[] solution)
	{
		Populate(thetas, thetaDots);
		_luSolver.Eliminate(_matrix, _vector, solution);
		return (thetaDots, solution);
	}
	
	private double[] RkShorthand(double[] current, double[] baseValues, double mult, double[] output)
	{
		Parallel.For(0, _n, i =>
		{
			output[i] = baseValues[i] + mult * current[i];
		});
		return output;
	}
	
	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private (double[], double[]) ApplyRk((double[], double[]) current, double[] firstBase, double[] secondBase, double mult, double[] secondBuffer, double[] solutionBuffer)
		=> F(RkShorthand(current.Item1, firstBase, mult, _rkBuffers[0]), RkShorthand(current.Item2, secondBase, mult, secondBuffer), solutionBuffer);

	private void RungeKutta4(double dt, double[] thetas, double[] thetaDots)
	{
		var k1 = F(thetas, thetaDots, _rkSolutionBuffers[0]);
		var k2 = ApplyRk(k1, thetas, thetaDots, 0.5 * dt, _rkBuffers[1], _rkSolutionBuffers[1]);
		var k3 = ApplyRk(k2, thetas, thetaDots, 0.5 * dt, _rkBuffers[2], _rkSolutionBuffers[2]);
		var k4 = ApplyRk(k3, thetas, thetaDots, 1 * dt, _rkBuffers[3], _rkSolutionBuffers[3]);
		Parallel.For(0, _n, i =>
		{
			thetas[i] += dt / 6 * (k1.Item1[i] + (k2.Item1[i] + k3.Item1[i]) * 2 + k4.Item1[i]);
			thetaDots[i] += dt / 6 * (k1.Item2[i] + (k2.Item2[i] + k3.Item2[i]) * 2 + k4.Item2[i]);
		});
	}
#elif SYMEULER
	private void SymplecticEuler(double dt, double[] thetas, double[] thetaDots)
	{
		Populate(thetas, thetaDots);

		_luSolver.Eliminate(_matrix, _vector, _solutionBuffer);

		Parallel.For(0, _n, i =>
		{
			thetaDots[i] += dt * _solutionBuffer[i];
			thetas[i] += dt * thetaDots[i];
		});
	}
#elif LEAPFROG
	private void Leapfrog(double dt, double[] thetas, double[] thetaDots)
	{
		Populate(thetas, thetaDots);
		_luSolver.Eliminate(_matrix, _vector, _accelBuffer);
		Parallel.For(0, _n, i =>
		{
			thetaDots[i] += 0.5 * dt * _accelBuffer[i];
			thetas[i] += dt * thetaDots[i];
		});
		
		Populate(thetas, thetaDots);
		
		_luSolver.Eliminate(_matrix, _vector, _newAccelBuffer);

		Parallel.For(0, _n, i =>
		{
			thetaDots[i] += 0.5 * dt * _newAccelBuffer[i];
		});
	}
#endif
	public void Update(double dt)
	{
#if RUNGEKUTTA
		RungeKutta4(dt, _thetas, _thetaDots);
#elif SYMEULER
		SymplecticEuler(dt, _thetas, _thetaDots);
#elif LEAPFROG
		Leapfrog(dt, _thetas, _thetaDots);
#endif
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
	
	public void Draw(Graphics g, Graphics bitmapG, PointF start)
	{
		Positions(start);
		var last = _positions[^1];
		
		if (_previous is not null)
		{
			bitmapG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
			bitmapG.DrawLine(_pointPen, _previous.Value, last);
		}
		
		g.DrawLines(_linePen, _positions);
		g.FillEllipse(_circleBrush, last.X - CircleOffset, last.Y - CircleOffset, CircleSize, CircleSize);
		
		_previous = last;
	}
	
	public void Dispose()
	{
		_linePen.Dispose();
		_circleBrush.Dispose();
	}
}