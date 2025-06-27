namespace Pendulum.Math;

public class Leapfrog : PendulumSolverBase
{
	protected readonly double[] _accelBuffer;
	protected readonly double[] _newAccelBuffer;

	public Leapfrog(int n, double g) : base(n, g)
	{
		_accelBuffer = new double[n];
		_newAccelBuffer = new double[n];
	}
	
	public override void Solve(double dt, double[] thetas, double[] thetaDots)
	{
		Populate(thetas, thetaDots);
		LuSolver.Eliminate(Matrix, Vector, _accelBuffer);
		Parallel.For(0, N, i =>
		{
			thetaDots[i] += 0.5 * dt * _accelBuffer[i];
			thetas[i] += dt * thetaDots[i];
		});
		
		Populate(thetas, thetaDots);
		
		LuSolver.Eliminate(Matrix, Vector, _newAccelBuffer);

		Parallel.For(0, N, i =>
		{
			thetaDots[i] += 0.5 * dt * _newAccelBuffer[i];
		});
	}
}