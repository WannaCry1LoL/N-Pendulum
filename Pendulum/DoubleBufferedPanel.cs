namespace Pendulum;

public class DoubleBufferedPanel : Panel
{
	public DoubleBufferedPanel() : base()
	{
		SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.DoubleBuffer | ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint, true );
	}

}