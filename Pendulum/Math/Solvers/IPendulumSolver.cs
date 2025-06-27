namespace Pendulum.Math;

public interface IPendulumSolver
{
	public void Solve(double dt, double[] thetas, double[] thetaDots);
}