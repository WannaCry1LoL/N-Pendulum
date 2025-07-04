using Pendulum.Math.Solvers;

namespace Pendulum.Math;

public enum SolverType
{
	RungeKutta4,
	SymplecticEuler,
	Leapfrog
}

public class NPendulum : IDisposable
{
	private const double Gravity = -9.81;
	private const double ArmLength = 500;
	private const float CircleSize = 20;
	private const float CircleOffset = (CircleSize / 2);
	
	private PointF? _previous;
	
	private readonly IPendulumSolver _solver;
	
	private readonly double[] _thetas;
	private readonly double[] _thetaDots;
	private readonly PointF[] _positions;
	private readonly int _n;
	private readonly double _armlength;
	
	private readonly Pen _pointPen = Pens.Red;
	private readonly Pen _linePen = new(Color.Black, 3);
	private readonly SolidBrush _circleBrush = new(Color.Red);

	public NPendulum(double[] initialTheta, double[] initialThetaDot, SolverType solverType = SolverType.RungeKutta4)
	{
		_n = initialTheta.Length;
		
		_thetas = new double[_n];
		_thetaDots = new double[_n];

		_solver = solverType switch
		{
			SolverType.RungeKutta4 => new RungeKutta4(_n, Gravity),
			SolverType.SymplecticEuler => new SymplecticEuler(_n, Gravity),
			SolverType.Leapfrog => new Leapfrog(_n, Gravity),
			_ => throw new InvalidOperationException("Unknown solver type"),
		};
		
		_armlength = ArmLength / _n;
		_positions = new PointF[_n + 1];
		
		initialTheta.CopyTo(_thetas);
		initialThetaDot.CopyTo(_thetaDots);
	}
	
	public void ClearPoints() => _previous = null;
	
	public void Update(double dt)
	{
		_solver.Solve(dt, _thetas, _thetaDots);
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