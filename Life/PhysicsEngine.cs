using System.Numerics;

namespace Life;

public class PhysicsEngine
{
    private readonly List<RigidBody> _bodies = [];
    private readonly List<Particle> _particles = [];
    private static readonly float _drag = 0.50f;

    public static float ScreenWidth { get; set; }
    public static float ScreenHeight { get; set; }

    public void AddBody(RigidBody body)
    {
        if (body is Particle particle)
            _particles.Add(particle);

        _bodies.Add(body);
    }

    public void AddBodyRange(IEnumerable<RigidBody> bodies)
    {
        foreach (RigidBody b in bodies)
            AddBody(b);
    }

    public List<RigidBody> GetBodies() => _bodies;

    public List<Particle> GetParticles() => _particles;

    public void Update(float deltaTime)
    {
        float cellSize = 100f;
        float distanceThreshold = 200f; // Example: 10 units

        ApplyInteractionForces(cellSize, distanceThreshold);

        foreach (var body in _bodies)
        {
            body.Update(deltaTime);

            // Check collisions
            foreach (var other in _bodies)
            {
                if (body != other && body.CheckCollision(other)) // Example radii
                {
                    body.ResolveCollision(other);
                }
            }

            body.ApplyDamping(_drag, _drag);
        }
    }

    private Dictionary<(int X, int Y), List<Particle>> BuildSpatialGrid(float cellSize)
    {
        var grid = new Dictionary<(int, int), List<Particle>>();

        foreach (var particle in _particles)
        {
            var cell = ((int)(particle.Position.X / cellSize), (int)(particle.Position.Y / cellSize));

            if (!grid.TryGetValue(cell, out List<Particle>? value))
                value = [];

            value.Add(particle);
            grid.TryAdd(cell, value);
        }

        return grid;
    }

    private void ApplyInteractionForces(float cellSize, float distanceThreshold)
    {
        var grid = BuildSpatialGrid(cellSize);

        Parallel.ForEach(grid, cell =>
        {
            foreach (var particle in cell.Value)
            {
                UpdateNeighboringParticles(grid, cell, particle, distanceThreshold);
            }
        });
    }

    private static void UpdateNeighboringParticles(Dictionary<(int X, int Y), List<Particle>> grid, KeyValuePair<(int X, int Y), List<Particle>> cell, Particle particle, float distanceThreshold)
    {
        var maxX = grid.Max(x => x.Key.X);
        var maxY = grid.Max(y => y.Key.Y);

        // Check neighboring cells
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                (int X, int Y) neighborCell = (cell.Key.X + dx, cell.Key.Y + dy);

                if (neighborCell.X < 0)
                    neighborCell = (maxX, neighborCell.Y);
                else if (neighborCell.X > maxX)
                    neighborCell = (0, neighborCell.Y);

                if (neighborCell.Y < 0)
                    neighborCell = (neighborCell.X, maxY);
                else if (neighborCell.Y > maxY)
                    neighborCell = (neighborCell.X, 0);

                if (!grid.ContainsKey(neighborCell)) continue;

                foreach (var other in grid[neighborCell])
                {
                    if (particle == other) continue;
                    particle.ApplyInteractionForce(other, distanceThreshold);
                }
            }
        }
    }

    public static Vector2 GetShortestWrappedDistance(Vector2 position1, Vector2 position2)
    {
        float dx = position2.X - position1.X;
        float dy = position2.Y - position1.Y;

        // Wrap distances in X
        if (Math.Abs(dx) > ScreenWidth / 2)
            dx -= Math.Sign(dx) * ScreenWidth;

        // Wrap distances in Y
        if (Math.Abs(dy) > ScreenHeight / 2)
            dy -= Math.Sign(dy) * ScreenHeight;

        return new Vector2(dx, dy);
    }

    public List<List<Particle>> DBSCAN(List<Particle> particles, float eps, int minPoints, out HashSet<Particle> noise)
    {
        var clusters = new List<List<Particle>>();
        var visited = new HashSet<Particle>();
        noise = [];

        foreach (var particle in particles)
        {
            if (visited.Contains(particle)) continue;

            visited.Add(particle);

            // Find neighbors within the radius
            var neighbors = particles.Where(p => Vector2.Distance(p.Position, particle.Position) <= eps).ToList();

            if (neighbors.Count < minPoints)
            {
                noise.Add(particle); // Mark as noise
            }
            else
            {
                // Create a new cluster
                var cluster = new List<Particle>();
                clusters.Add(cluster);
                ExpandCluster(cluster, particle, neighbors, particles, visited, eps, minPoints);
            }
        }

        return clusters;
    }

    private void ExpandCluster(
        List<Particle> cluster,
        Particle particle,
        List<Particle> neighbors,
        List<Particle> particles,
        HashSet<Particle> visited,
        float eps,
        int minPoints)
    {
        cluster.Add(particle);

        for (int i = 0; i < neighbors.Count; i++)
        {
            var neighbor = neighbors[i];

            if (!visited.Contains(neighbor))
            {
                visited.Add(neighbor);

                // Find neighbors of the neighbor
                var neighborNeighbors = particles.Where(p => Vector2.Distance(p.Position, neighbor.Position) <= eps).ToList();
                if (neighborNeighbors.Count >= minPoints)
                {
                    neighbors.AddRange(neighborNeighbors.Where(n => !neighbors.Contains(n)));
                }
            }

            if (!cluster.Contains(neighbor))
            {
                cluster.Add(neighbor);
            }
        }
    }
}