namespace Pendulum.Math;
public static class OurExtensions
{
	public static double Mod(this double value, double right) => value - right * double.Floor(value / right);
	
}