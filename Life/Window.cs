using System.Diagnostics;
using System.Reflection;
using System.Runtime.InteropServices;

namespace Life
{
    public partial class Window : Form
    {
        public bool IsWindowOpen { get; set; } = true;
        public int FrameRate { get; } = 120;

        private bool _canPaint = true;
        private readonly Stopwatch _timer;
        private readonly WindowPainter _painter;
        private readonly PhysicsEngine _engine;

        public Window(WindowPainter painter, PhysicsEngine engine)
        {
            _painter = painter;
            _engine = engine;

            InitializeComponent();
            DoubleBuffered = true;
            CreateRules(true);

            _timer = Stopwatch.StartNew();

            Task.Run(() =>
            {
                while (IsWindowOpen)
                {
                    Stopwatch stopwatch = Stopwatch.StartNew();

                    while (stopwatch.Elapsed < TimeSpan.FromMilliseconds(1000 / FrameRate))
                        ;

                    _canPaint = true;
                }
            });
        }

        private void CreateRules(bool useTest = false)
        {
            if (useTest)
            {
                CreateTestRules();

                return;
            }

            var colors = Enum.GetValues(typeof(ParticleColor)).Cast<ParticleColor>().ToList();

            foreach (var cell in colors)
            {
                Particle.Rules.Add(cell, []);
            }

            foreach (var cell in colors)
            {
                foreach (var cell2 in colors)
                {
                    var rng = new Random();
                    Particle.Rules[cell].Add(cell2, (decimal)(Math.Sqrt(Math.Sqrt(rng.NextDouble())) * 0.0005) * (rng.Next(2) == 0 ? -1 : 1));
                }
            }

            for (int i = 0; i < 100; i++)
            {
                var rng = new Random();

                foreach (var cell in colors)
                    _engine.AddBody(new Particle(cell, new(rng.Next(ClientSize.Width), rng.Next(ClientSize.Height))));
            }
        }

        private void CreateTestRules()
        {
            Particle.Rules.Add(ParticleColor.Red, []);
            Particle.Rules.Add(ParticleColor.Green, []);

            decimal force = 0.005M;

            Particle.Rules[ParticleColor.Red].Add(ParticleColor.Red, -force);
            Particle.Rules[ParticleColor.Red].Add(ParticleColor.Green, force);
            Particle.Rules[ParticleColor.Green].Add(ParticleColor.Red, -force);
            Particle.Rules[ParticleColor.Green].Add(ParticleColor.Green, force);


            for (int i = 0; i < 200; i++)
            {
                var rng = new Random();
                //_engine.AddBody(new Particle(ParticleColor.Red, new(rng.Next(Size.Width), rng.Next(Size.Height))));
                _engine.AddBody(new Particle(ParticleColor.Green, new(rng.Next(Size.Width), rng.Next(Size.Height))));
            }
        }

        private void UpdateVariables()
        {
            PhysicsEngine.ScreenHeight = ClientSize.Height;
            PhysicsEngine.ScreenWidth = ClientSize.Width;
            _engine.Update((float)_timer.Elapsed.TotalSeconds * 2f);
        }

        private void Window_Paint(object sender, PaintEventArgs e)
        {
            _painter.Paint(e);
        }

        private void Window_FormClosing(object sender, FormClosingEventArgs e)
        {
            IsWindowOpen = false;
        }

        private void Window_Shown(object sender, EventArgs e)
        {
            while (IsWindowOpen)
            {
                RefreshPanel();
                DoEvents();
            }
        }

        private void RefreshPanel()
        {
            if (!_canPaint)
                return;

            UpdateVariables();
            Refresh();
            _canPaint = false;
        }

        private void DoEvents()
        {
            while (!_canPaint)
            {
                Application.DoEvents();
            }
        }

        private void Window_SizeChanged(object sender, EventArgs e)
        {
            RefreshPanel();
        }

        private void Window_Resize(object sender, EventArgs e)
        {
            RefreshPanel();
        }

        private void Window_ResizeBegin(object sender, EventArgs e)
        {
            RefreshPanel();
        }

        private void Window_ResizeEnd(object sender, EventArgs e)
        {
            RefreshPanel();
        }

        private void Window_Load(object sender, EventArgs e)
        {
            AllocConsole();
        }

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern bool AllocConsole();
    }
}
