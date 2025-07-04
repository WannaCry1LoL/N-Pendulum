namespace Pendulum.Math.Solvers;

public interface IPendulumSolver
{
	public void Solve(double dt, double[] thetas, double[] thetaDots);
}