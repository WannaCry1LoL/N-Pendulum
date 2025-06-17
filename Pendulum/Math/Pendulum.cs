namespace Pendulum.Math;

public struct Pendulum
{
	public double ThetaDot;

	public double Theta
	{
		get;
		set => field = value.Mod(double.Tau);
	}
	
	public Pendulum(double theta, double thetaDot = 0) => (Theta, ThetaDot) = (theta, thetaDot);
}
