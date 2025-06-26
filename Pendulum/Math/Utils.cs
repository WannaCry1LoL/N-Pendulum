namespace Pendulum.Math;

public static class Utils
{
	public static T[][] CreateFixedSizeBuffers<T>(int n, int size)
	{
		var res = new T[n][];
		for (int i = 0; i < n; i++)
		{
			res[i] = new T[size];
		}
		return res;
	}
}