namespace Pendulum;

static class Program
{
	/// <summary>
	///  The main entry point for the application.
	/// </summary>
	[STAThread]
	static void Main(string[] args)
	{
		var amount = 2;
		if (args.Length >= 1)
		{
			amount = int.Parse(args[0]);
		}
		ApplicationConfiguration.Initialize();
		Application.Run(new Form1(amount));
	}
}