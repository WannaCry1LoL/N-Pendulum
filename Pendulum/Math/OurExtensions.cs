using System.Numerics;

namespace Pendulum.Math;
public static class OurExtensions
{
	public static T Mod<T>(this T value, T right) where T : IFloatingPoint<T> => value - right * T.Floor(value / right);
}