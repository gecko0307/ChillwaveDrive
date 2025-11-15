module vehicle;

import std.stdio;
import std.algorithm;
import std.math;
import dagon;
import dagon.ext.newton;
import wheel;

Vector3f boxInertia(Vector3f halfSize, float mass)
{
    float x2 = halfSize.x * halfSize.x;
    float y2 = halfSize.y * halfSize.y;
    float z2 = halfSize.z * halfSize.z;
    float Ixx = (mass / 3.0) * (y2 + z2);
    float Iyy = (mass / 3.0) * (x2 + z2);
    float Izz = (mass / 3.0) * (x2 + y2);
    return Vector3f(Ixx, Iyy, Izz);
}

class Vehicle: EntityComponent
{
    NewtonPhysicsWorld world;
    NewtonCollisionShape chassisShape;
    NewtonRigidBody chassisBody;
    Array!Wheel wheels;
    
    float torqueDirection = 1.0f; // -1.0f or 1.0f
    float throttle = 0.0f; // 0.0f..1.0f
    float steeringInput = 0.0f; // -1.0f..1.0f
    float maxSteeringAngle = 45.0f;
    float maxTorque = 5000.0f;
    
    bool accelerating = false;
    bool brake = false;
    
    float movementDirection = 0.0f;
    
    Matrix4x4f prevTransformation;
    
    this(NewtonPhysicsWorld world, Entity entity, NewtonCollisionShape shape, float mass, int materialID)
    {
        super(world.eventManager, entity);
        this.world = world;
        
        this.chassisShape = shape;
        
        this.chassisBody = world.createDynamicBody(this.chassisShape, mass);
        this.chassisBody.position = entity.position;
        this.chassisBody.rotation = entity.rotation;
        this.chassisBody.transformation =
            translationMatrix(entity.position) *
            entity.rotation.toMatrix4x4;
        this.chassisBody.raycastable = false;
        this.chassisBody.groupId = materialID;
        
        NewtonBodySetContinuousCollisionMode(chassisBody.newtonBody, 1);
        NewtonBodySetMatrix(chassisBody.newtonBody, chassisBody.transformation.arrayof.ptr);
        
        NewtonMaterialSetDefaultFriction(world.newtonWorld, 0, materialID, 0.2f, 0.2f);
        NewtonMaterialSetDefaultElasticity(world.newtonWorld, 0, materialID, 0.2f);
        
        prevTransformation = Matrix4x4f.identity;
    }
    
    ~this()
    {
        wheels.free();
    }
    
    Wheel addWheel(Vector3f suspensionPosition, float radius, float facing)
    {
        Wheel wheel = New!Wheel(suspensionPosition, facing, this);
        wheel.radius = radius;
        wheels.append(wheel);
        return wheel;
    }
    
    void setInertia(float mass, Vector3f itertia)
    {
        NewtonBodySetMassMatrix(chassisBody.newtonBody, mass, itertia.x, itertia.y, itertia.z);
    }
    
    Vector3f position() @property
    {
        return chassisBody.position.xyz;
    }
    
    Quaternionf rotation() @property
    {
        return chassisBody.rotation;
    }
    
    Matrix4x4f transformation() @property
    {
        return chassisBody.transformation;
    }
    
    Vector3f longitudinalAxis()
    {
        return chassisBody.transformation.forward;
    }
    
    Vector3f lateralAxis()
    {
        return chassisBody.transformation.right;
    }
    
    Vector3f verticalAxis()
    {
        return chassisBody.transformation.up;
    }
    
    Vector3f velocity() @property
    {
        return chassisBody.velocity;
    }
    
    float speed() @property
    {
        return chassisBody.velocity.length;
    }
    
    float speedKMH() @property
    {
        return chassisBody.velocity.length * 3.6;
    }
    
    void accelerate(float direction, float delta)
    {
        brake = (movementDirection < 0.0f && direction > 0.0f) ||
                (movementDirection > 0.0f && direction < 0.0f);
        
        torqueDirection = direction;
        
        if (throttle < 1.0f)
            throttle += delta;
        else
            throttle = 1.0f;
        
        accelerating = true;
    }
    
    void idle()
    {
        accelerating = false;
        brake = false;
    }
    
    void steer(float input)
    {
        steeringInput += input;
        
        if (steeringInput > 1.0f)
            steeringInput = 1.0f;
        if (steeringInput < -1.0f)
            steeringInput = -1.0f;
    }
    
    float lateralSpeedKMH() @property
    {
        Vector3f rightVector = chassisBody.transformation.right;
        return abs(dot(chassisBody.velocity, rightVector)) * 3.6;
    }
    
    float longitudinalSpeedKMH() @property
    {
        Vector3f forwardVector = chassisBody.transformation.forward;
        return abs(dot(chassisBody.velocity, forwardVector)) * 3.6;
    }
    
    float lateralSlip() @property
    {
        float lateralSpeed = abs(dot(chassisBody.velocity, chassisBody.transformation.right));
        return clamp((lateralSpeed - 6.0f) / 6.0f, 0.0f, 1.0f);
    }
    
    float longitudinalSlip() @property
    {
        if (brake) return 1.0f;
        
        float res = 0.0f;
        foreach(wheel; wheels)
        {
            if (wheel.onGround)
            {
                res += clamp(wheel.slipRatio, 0.0f, 1.0f);
            }
        }
        return res / wheels.length;
    }
    
    override void update(Time t)
    {
        float speed = speedKMH;
        
        float ackermann = 5.0f;
        float steeringAngleInner = maxSteeringAngle * steeringInput;
        float steeringAngleOuter = (maxSteeringAngle - ackermann) * steeringInput;
        
        if (steeringInput < 0.0f)
        {
            wheels[0].steeringAngle = steeringAngleInner;
            wheels[1].steeringAngle = steeringAngleOuter;
        }
        else
        {
            wheels[0].steeringAngle = steeringAngleOuter;
            wheels[1].steeringAngle = steeringAngleInner;
        }
        
        float torque = 0.0f;
        if (accelerating)
        {
            float spd = speedKMH;
            float decreaseFactor = lerp(1.0f, 0.9f, clamp((spd - 80.0f) / (200.0f - 80.0f), 0.0f, 1.0f));
            torque = maxTorque * decreaseFactor * throttle * torqueDirection;
        }
        
        foreach(w; wheels)
        {
            w.torque = torque * w.torqueSplitRatio;
            w.brake = brake;
            w.update(t.delta);
        }
        
        chassisBody.update(t.delta);

        entity.prevTransformation = prevTransformation;

        entity.position = chassisBody.position.xyz;
        entity.transformation = chassisBody.transformation * scaleMatrix(entity.scaling);
        entity.invTransformation = entity.transformation.inverse;
        entity.rotation = chassisBody.rotation;

        entity.absoluteTransformation = entity.transformation;
        entity.invAbsoluteTransformation = entity.invTransformation;
        entity.prevAbsoluteTransformation = entity.prevTransformation;

        prevTransformation = entity.transformation;
        
        float steeringDecreaseStep = 1.8f * t.delta;
        if (steeringInput > steeringDecreaseStep)
            steeringInput -= steeringDecreaseStep;
        else if (steeringInput < -steeringDecreaseStep)
            steeringInput += steeringDecreaseStep;
        else
            steeringInput = 0.0f;
        
        movementDirection = (dot(velocity.normalized, longitudinalAxis) < 0.0f)? -1.0f : 1.0f;
        
        if (!accelerating)
        {
            throttle = 0.0f;
        }
    }
}
