using System.Numerics;
using System.Runtime.InteropServices;


namespace Pendulum.Math;
enum ComparisonTriState
{
	Smaller = -1,
	Equal = 0,
	Greater = 1,
}

public readonly record struct NumberRange<T> where T : INumberBase<T>, IComparable<T>
{
	public readonly T Min, Max;
	
	public NumberRange(T min, T max) => (Min, Max) = (min, max);
	private static ComparisonTriState Compare(T left, T right) => (ComparisonTriState)left.CompareTo(right);
	
	public T ClampLower(T val) => Compare(val, Min) < ComparisonTriState.Equal ? Min : val;
	public T ClampUpper(T val) => Compare(val, Max) > ComparisonTriState.Equal ? Max : val;
	public T Clamp(T val) => ClampUpper(ClampLower(val));
	
	public T Map(T val, T firstMin, T firstMax) => (val - firstMin)/(firstMax - firstMin) * (Max - Min) + Min;
	
	public static NumberRange<T> FromVariation(T center, T lower, T upper) => new(center - lower, center + upper);
	public static NumberRange<T> FromVariation(T center, T variation) => FromVariation(center, variation, variation);
}


