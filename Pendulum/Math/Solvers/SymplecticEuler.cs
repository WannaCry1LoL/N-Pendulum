namespace Pendulum.Math.Solvers;

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
		for (int i = 0; i < N; i++)
		{
			thetaDots[i] += dt * SolutionBuffer[i];
			thetas[i] += dt * thetaDots[i];
		}
	}
}