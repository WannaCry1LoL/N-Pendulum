using System.Runtime.CompilerServices;

namespace Pendulum.Math.Solvers;

public class RungeKutta4 : PendulumSolverBase
{
	protected readonly double[][] RkBuffers;
	protected readonly double[][] RkSolutionBuffers;

	public RungeKutta4(int n, double g) : base(n, g)
	{
		RkBuffers = Utils.CreateFixedSizeBuffers<double>(4, N);
		RkSolutionBuffers = Utils.CreateFixedSizeBuffers<double>(4, N);
	}
	
	private void F(double[] thetas, double[] thetaDots, double[] solution)
	{
		Populate(thetas, thetaDots);
		LuSolver.Eliminate(Matrix, Vector, solution);
	}
	
	private double[] RkShorthand(double[] current, double[] baseValues, double mult, double[] output)
	{
		for (int i = 0; i < N; i++)
			output[i] = baseValues[i] + mult * current[i];
		return output;
	}

	public override void Solve(double dt, double[] thetas, double[] thetaDots)
	{
		var (vel1, vel2, vel3, vel4) = (thetaDots, RkBuffers[1], RkBuffers[2], RkBuffers[3]);
		var (acc1, acc2, acc3, acc4) = (RkSolutionBuffers[0], RkSolutionBuffers[1], RkSolutionBuffers[2], RkSolutionBuffers[3]);

		F(thetas, thetaDots, RkSolutionBuffers[0]);
		F(
			RkShorthand(vel1, thetas, 0.5 * dt, RkBuffers[0]),
			RkShorthand(acc1, thetaDots, 0.5 * dt, RkBuffers[1]),
			RkSolutionBuffers[1]
		);
		F(
			RkShorthand(vel2, thetas, 0.5 * dt, RkBuffers[0]),
			RkShorthand(acc2, thetaDots, 0.5 * dt, RkBuffers[2]),
			RkSolutionBuffers[2]
		);
		F(
			RkShorthand(vel3, thetas, 1 * dt, RkBuffers[0]),
			RkShorthand(acc3, thetaDots, 1 * dt, RkBuffers[3]), 
			RkSolutionBuffers[3]
		);
		
		for (int i = 0; i < N; i++) {
			thetas[i] += dt / 6 * (vel1[i] + (vel2[i] + vel3[i]) * 2 + vel4[i]);
			thetaDots[i] += dt / 6 * (acc1[i] + (acc2[i] + acc3[i]) * 2 + acc4[i]);
		}
	}
}