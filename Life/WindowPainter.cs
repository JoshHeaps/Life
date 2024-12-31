namespace Life;

public class WindowPainter(PhysicsEngine engine)
{
    public void Paint(PaintEventArgs e)
    {
        var graphics = e.Graphics;
        Dictionary<ParticleColor, Func<Brush>> brushes = new()
        {
            { ParticleColor.Red, () => new SolidBrush(Color.Red) },
            { ParticleColor.Yellow, () => new SolidBrush(Color.Orange) },
            { ParticleColor.Green, () => new SolidBrush(Color.Green) },
            { ParticleColor.Cyan, () => new SolidBrush(Color.Teal) },
            { ParticleColor.Blue, () => new SolidBrush(Color.Blue) },
            { ParticleColor.Magenta, () => new SolidBrush(Color.Magenta) },
        };

        graphics.Clear(Color.White);

        var cells = engine.GetParticles();

        foreach (var cell in cells)
        {
            Brush brush = brushes[cell.Color]();

            try
            {
                graphics.FillRectangle(brush, new((int)cell.Position.X, (int)cell.Position.Y, 3, 3));
            }
            catch { }
        }
    }
}
