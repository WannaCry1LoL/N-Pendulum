namespace Pendulum.Math;

public abstract class PendulumSolverBase : IPendulumSolver
{
	protected readonly int N;
	protected readonly double Gravity;
	protected readonly double[] Vector;
	protected readonly double[,] Matrix;
	protected readonly NxNLUSolver LuSolver;
	
	public abstract void Solve(double dt, double[] thetas, double[] thetaDots);
	
	protected void Populate(double[] thetas, double[] thetaDots)
	{
		Parallel.For(0, N, i =>
		{
			var sum = 0.0;
			var theta = thetas[i];
			for (var j = 0; j < N; j++)
			{
				var theta2 = thetas[j];
				var theta2dot = thetaDots[j];
				var res = double.SinCos(theta - theta2);
				var weird = N - int.Max(i, j);
				sum -= weird * res.Sin * theta2dot * theta2dot;
				Matrix[i, j] = weird * res.Cos;
			}
			
			sum -= Gravity * (N - i) * double.Sin(theta);
			Vector[i] = sum;
		});
	}
	
	protected PendulumSolverBase(int n, double g)
	{
		N = n;
		Gravity = g;
		Vector = new double[n];
		Matrix = new double[n, n];
		LuSolver = new NxNLUSolver(n);
	}
}