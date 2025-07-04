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
	
	private (double[], double[]) F(double[] thetas, double[] thetaDots, double[] solution)
	{
		Populate(thetas, thetaDots);
		LuSolver.Eliminate(Matrix, Vector, solution);
		return (thetaDots, solution);
	}
	
	private double[] RkShorthand(double[] current, double[] baseValues, double mult, double[] output)
	{
		Parallel.For(0, N, i =>
		{
			output[i] = baseValues[i] + mult * current[i];
		});
		return output;
	}
	
	[MethodImpl(MethodImplOptions.AggressiveInlining)]
	private (double[], double[]) ApplyRk((double[], double[]) current, double[] firstBase, double[] secondBase, double mult, double[] secondBuffer, double[] solutionBuffer)
		=> F(RkShorthand(current.Item1, firstBase, mult, RkBuffers[0]), RkShorthand(current.Item2, secondBase, mult, secondBuffer), solutionBuffer);

	public override void Solve(double dt, double[] thetas, double[] thetaDots)
	{
		var k1 = F(thetas, thetaDots, RkSolutionBuffers[0]);
		var k2 = ApplyRk(k1, thetas, thetaDots, 0.5 * dt, RkBuffers[1], RkSolutionBuffers[1]);
		var k3 = ApplyRk(k2, thetas, thetaDots, 0.5 * dt, RkBuffers[2], RkSolutionBuffers[2]);
		var k4 = ApplyRk(k3, thetas, thetaDots, 1 * dt, RkBuffers[3], RkSolutionBuffers[3]);
		Parallel.For(0, N, i =>
		{
			thetas[i] += dt / 6 * (k1.Item1[i] + (k2.Item1[i] + k3.Item1[i]) * 2 + k4.Item1[i]);
			thetaDots[i] += dt / 6 * (k1.Item2[i] + (k2.Item2[i] + k3.Item2[i]) * 2 + k4.Item2[i]);
		});
	}
}