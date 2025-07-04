namespace Pendulum.Math;

public class NxNLUSolver
{
	private readonly int _n;
	private readonly double[,] _uBuffer;
	private readonly double[,] _lBuffer;
	private readonly double[] _uTransformBuffer;

	public NxNLUSolver(int n)
	{
		_n = n;
		_uBuffer = new double[n, n];
		_lBuffer = new double[n, n];
		_uTransformBuffer = new double[n];
	}
	public void Decompose(double[,] source)
	{
		for (var i = 0; i < _n; i++)
		{
			for (var k = i; k < _n; k++)
			{
				double sum = 0;

				for (var j = 0; j < i; j++)
				{
					sum += _lBuffer[i, j] * _uBuffer[j, k];
				}

				_uBuffer[i, k] = source[i, k] - sum;
			}

			for (var k = i; k < _n; k++)
			{
				if (i == k)
				{
					_lBuffer[i, i] = 1;
				}
				else
				{
					double sum = 0;

					for (var j = 0; j < i; j++)
					{
						sum += _lBuffer[k, j] * _uBuffer[j, i];
					}

					_lBuffer[k, i] = (source[k, i] - sum) / _uBuffer[i, i];
				}
			}
		}
	}
	
	public void Eliminate(double[,] matrix, ReadOnlySpan<double> coefficients, Span<double> solution)
	{
		Decompose(matrix);

		for (var i = 0; i < _n; i++)
		{
			double pivotPointSum = 0;

			for (var j = 0; j < i; j++)
			{
				pivotPointSum += _uTransformBuffer[j] * _lBuffer[i, j];
			}

			_uTransformBuffer[i] = coefficients[i] - pivotPointSum;
		}

		for (var i = _n - 1; i >= 0; i--)
		{
			double pivotPointSum = 0;

			for (var j = i + 1; j < _n; j++)
			{
				pivotPointSum += solution[j] * _uBuffer[i, j];
			}

			solution[i] = (_uTransformBuffer[i] - pivotPointSum) / _uBuffer[i, i];
		}
	}
}