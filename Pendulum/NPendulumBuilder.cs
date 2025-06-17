using Pendulum.Math;

namespace Pendulum;

public class NPendulumBuilder
{
	private List<Math.Pendulum> _pendulums = [];
	
	public static NPendulumBuilder Create() => new();

	public NPendulumBuilder Add(double theta, double thetaDot)
	{
		_pendulums.Add(new Math.Pendulum(theta, thetaDot));
		return this;
	}

	public NPendulumBuilder Add(Math.Pendulum pendulum)
	{
		_pendulums.Add(pendulum);
		return this;
	}

	public NPendulumBuilder AddSimilar(double theta, double thetaDot, double thetaVariation, double thetaDotVariation)
	{
		var thetaRange = NumberRange<double>.FromVariation(theta, thetaVariation);
		var thetaDotRange = NumberRange<double>.FromVariation(thetaDot, thetaDotVariation); 
		Add(thetaRange.Map(Random.Shared.NextDouble(), 0, 1), thetaDotRange.Map(Random.Shared.NextDouble(), 0, 1));
		return this;
	}

	public NPendulumBuilder AddRandom(NumberRange<double>? thetaRange = null, NumberRange<double>? thetaDotRange = null)
	{
		var thetaBounds = thetaRange ??  new NumberRange<double>(0.0, double.Tau);
		var thetaDotBounds = thetaDotRange ?? new NumberRange<double>(0.0, 0.0);
		var theta = thetaBounds.Map(Random.Shared.NextDouble(), 0, 1);
		var thetaDot = thetaDotBounds.Map(Random.Shared.NextDouble(), 0, 1);
		Add(theta, thetaDot);
		return this;
	}
	
	public NPendulumBuilder AddRange(params Math.Pendulum[] pendulums)
	{
		_pendulums.AddRange(pendulums);
		return this;
	}
	
	public NPendulumBuilder AddNSame(int n, double theta, double thetaDot)
	{
		for (int i = 0; i < n; i++)
		{
			Add(theta, thetaDot);
		}
		return this;
	}	
	
	public NPendulumBuilder AddNSimilar(int n, double theta, double thetaDot, double thetaVariation, double thetaDotVariation)
	{
		for (int i = 0; i < n; i++)
		{
			AddSimilar(theta, thetaDot, thetaVariation, thetaDotVariation);
		}
		return this;
	}
	
	public NPendulumBuilder AddNRandom(int n, NumberRange<double>? thetaRange = null, NumberRange<double>? thetaDotRange = null)
	{
		for (int i = 0; i < n; i++)
		{
			AddRandom(thetaRange, thetaDotRange);
		}
		return this;
	}
	
	public NPendulum Build()
	{
		if (_pendulums.Count == 0) 
			throw new InvalidOperationException("No pendulums added to the builder.");
		var thetas = _pendulums.Select(x => x.Theta).ToArray();
		var thetaDots = _pendulums.Select(x => x.ThetaDot).ToArray();
		return new NPendulum(thetas, thetaDots);
	}
}