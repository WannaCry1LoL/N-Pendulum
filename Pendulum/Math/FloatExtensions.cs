using System.Numerics;
namespace Pendulum.Math;
public delegate T Mapper<T>(T input); 
public static class OurExtensions
{
	public static double Mod(this double value, double right) => value - right * double.Floor(value / right);
	
}