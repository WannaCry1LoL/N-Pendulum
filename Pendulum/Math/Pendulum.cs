namespace Pendulum.Math;

public class Pendulum
{
	public double Length, Mass, ThetaDot;

	public double Theta
	{
		get;
		set => field = value.Mod(float.Tau);
	}

	
	public Pendulum(double length, double mass, double theta, double thetaDot = 0) => (Length, Mass, Theta, ThetaDot) = (length, mass, theta, thetaDot);
}
