using System.Numerics;

namespace Life;

public class RigidBody
{
    // Physical properties
    public Vector2 Position { get; private set; }
    public Vector2 Velocity { get; private set; }
    public Vector2 Acceleration { get; protected set; }
    public float Mass { get; private set; }
    public float InverseMass => Mass > 0 ? 1f / Mass : 0f;
    public Vector2 Force { get; private set; }

    // Rotational properties
    public float Rotation { get; private set; } // Angle in radians
    public float AngularVelocity { get; private set; }
    public float Torque { get; private set; }
    public float Inertia { get; private set; } // Moment of inertia
    public float InverseInertia => Inertia > 0 ? 1f / Inertia : 0f;

    public int Radius { get; private set; } = 3;

    // Constructor
    public RigidBody(Vector2 initialPosition, float mass, float inertia)
    {
        Position = initialPosition;
        Velocity = Vector2.Zero;
        Acceleration = Vector2.Zero;
        Force = Vector2.Zero;

        Rotation = 0f;
        AngularVelocity = 0f;
        Torque = 0f;

        Mass = mass;
        Inertia = inertia;
    }

    // Apply a force to the body
    public void ApplyForce(Vector2 force, Vector2? applicationPoint = null)
    {
        Force += force;
        if (applicationPoint.HasValue)
        {
            Vector2 r = applicationPoint.Value - Position; // Relative position
            Torque += r.X * force.Y - r.Y * force.X; // 2D cross product
        }
    }

    // Apply an impulse to the body
    public void ApplyImpulse(Vector2 impulse, Vector2? applicationPoint = null)
    {
        Velocity += impulse * InverseMass;
        if (applicationPoint.HasValue)
        {
            Vector2 r = applicationPoint.Value - Position;
            AngularVelocity += (r.X * impulse.Y - r.Y * impulse.X) * InverseInertia;
        }
    }

    // Update the physics
    public void Update(float deltaTime)
    {
        // Update linear motion
        Acceleration = Force * InverseMass;
        Velocity += Acceleration * deltaTime;
        Position += Velocity * deltaTime;

        while (Position.X < 0) Position = new(Position.X + PhysicsEngine.ScreenWidth, Position.Y);
        while (Position.X > PhysicsEngine.ScreenWidth) Position = new(Position.X - PhysicsEngine.ScreenWidth, Position.Y);
        while (Position.Y < 0) Position = new(Position.X, Position.Y + PhysicsEngine.ScreenHeight);
        while (Position.Y > PhysicsEngine.ScreenHeight) Position = new(Position.X, Position.Y - PhysicsEngine.ScreenHeight);

        // Update angular motion
        float angularAcceleration = Torque * InverseInertia;
        AngularVelocity += angularAcceleration * deltaTime;
        Rotation += AngularVelocity * deltaTime;

        // Reset forces for the next frame
        Force = Vector2.Zero;
        Torque = 0f;
    }

    public bool CheckCollision(RigidBody other)
    {
        float distance = Vector2.Distance(Position, other.Position);
        return distance < (3 + 3);
    }

    public void ResolveCollision(RigidBody other, float restitution = 0.0f)
    {
        // Get the distance vector between the particles
        Vector2 distanceVector = this.Position - other.Position;

        // Calculate the distance between particles
        float distance = distanceVector.Length();

        // Minimum allowable distance between particles (prevent overlap)
        float minDistance = this.Radius + other.Radius;

        // If the particles are too close (penetrating)
        if (distance < minDistance)
        {
            // Calculate the penetration depth
            float penetrationDepth = minDistance - distance;

            // Normalize the distance vector (to get the direction of separation)
            Vector2 separationDirection = Vector2.Normalize(distanceVector);

            // Apply a fraction of the penetration depth correction to prevent overshoot
            float correctionFactor = 0.5f;  // Apply half the penetration depth correction
            this.Position += separationDirection * (penetrationDepth * correctionFactor);  // Move this particle
            other.Position -= separationDirection * (penetrationDepth * correctionFactor); // Move the other particle
        }
    }

    public void ApplyDamping(float linearDamping, float angularDamping)
    {
        Velocity *= 1f - linearDamping;
        AngularVelocity *= 1f - angularDamping;
    }
}
