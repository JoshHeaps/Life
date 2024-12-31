using System.Numerics;

namespace Life;

public class Particle(ParticleColor color, Vector2 initialPosition) : RigidBody(initialPosition, 1, 0)
{
    /// <summary>
    /// Contains information on how each cell color will interact with other cell colors (negative value is repelled by, positive is attracted to)
    /// </summary>
    public static Dictionary<ParticleColor, Dictionary<ParticleColor, decimal>> Rules { get; set; } = [];
    private Dictionary<ParticleColor, decimal> _rules => Rules[color];

    public ParticleColor Color => color;

    public void ApplyInteractionForce(Particle other, float distanceThreshold)
    {
        Vector2 direction = PhysicsEngine.GetShortestWrappedDistance(this.Position, other.Position);
        float distanceSquared = direction.LengthSquared();

        if (distanceSquared > distanceThreshold * distanceThreshold) return;

        distanceSquared += 10;

        // Calculate force based on the rule and distance
        if (_rules.TryGetValue(other.Color, out decimal rule))
        {
            if (distanceSquared < 15)
            {
                distanceSquared *= -0.25F;
                rule = Math.Abs(rule);
            }

            float forceMagnitude = (float)((double)rule / distanceSquared);
            Vector2 force = Vector2.Normalize(direction) * forceMagnitude;

            this.ApplyForce(force);
        }
    }
}

public enum ParticleColor
{
    Red,
    Yellow,
    Green,
    Cyan,
    Blue,
    Magenta,
}