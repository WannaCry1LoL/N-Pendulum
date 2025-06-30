namespace Pendulum.Math;

public class SymplecticEuler : PendulumSolverBase
{
	protected readonly double[] SolutionBuffer;

	public SymplecticEuler(int n, double g) : base(n, g)
	{
		SolutionBuffer = new double[n];
	}
	
	public override void Solve(double dt, double[] thetas, double[] thetaDots)
	{
		Populate(thetas, thetaDots);

		LuSolver.Eliminate(Matrix, Vector, SolutionBuffer);

		Parallel.For(0, N, i =>
		{
			thetas[i] += dt * thetaDots[i];
			thetaDots[i] += dt * SolutionBuffer[i];
		});
	}
}