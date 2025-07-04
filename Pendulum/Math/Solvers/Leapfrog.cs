namespace Pendulum.Math.Solvers;

public class Leapfrog : PendulumSolverBase
{
	protected readonly double[] AccelBuffer;
	protected readonly double[] NewAccelBuffer;

	public Leapfrog(int n, double g) : base(n, g)
	{
		AccelBuffer = new double[n];
		NewAccelBuffer = new double[n];
	}
	
	public override void Solve(double dt, double[] thetas, double[] thetaDots)
	{
		Populate(thetas, thetaDots);
		LuSolver.Eliminate(Matrix, Vector, AccelBuffer);
		Parallel.For(0, N, i =>
		{
			thetaDots[i] += 0.5 * dt * AccelBuffer[i];
			thetas[i] += dt * thetaDots[i];
		});
		
		Populate(thetas, thetaDots);
		
		LuSolver.Eliminate(Matrix, Vector, NewAccelBuffer);

		Parallel.For(0, N, i =>
		{
			thetaDots[i] += 0.5 * dt * NewAccelBuffer[i];
		});
	}
}