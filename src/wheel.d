module wheel;

import std.stdio;
import std.algorithm;
import std.math;
import dagon;
import dagon.ext.newton;
import vehicle;
import pacejka;

struct Suspension
{
    float minLength;
    float maxLength;
    float stiffness; 
    float damping;
    float compression;
    float length;
    float lengthPrev;
}

class Wheel: Owner, NewtonRaycaster
{
    Vehicle vehicle;
    Vector3f position = Vector3f(0.0f, 0.0f, 0.0f);
    Suspension suspension;
    float radius = 0.35f;
    float steeringAngle = 0.0f;
    float camberAngle = 0.0f;
    float facing = 0.0f;
    float normalForce = 0.0f;
    float tractionForce = 0.0f;
    float lateralFrictionForce = 0.0f;
    float staticLateralFrictionForce = 0.0f;
    float longitudinalFrictionForce = 0.0f;
    float load = 0.25f;
    float slipAngle = 0.0f;
    float slipRatio = 0.0f;
    float torque = 0.0f;
    float torqueSplitRatio = 0.0f;
    float angularVelocity = 0.0f;
    float roll = 0.0f;
    float invInertia = 0.8f;
    float staticFrictionCoefficient = 0.99f;
    float lateralDynamicFrictionCoefficient = 1.0f;
    float longitudinalDynamicFrictionCoefficient = 1.0f;
    Quaternionf steering = Quaternionf.identity;
    bool brake = false;
    
    float maxRayDistance = 1000.0f;
    protected float closestHitRayParam = 1.0f;
    Vector3f groundPosition;
    Vector3f groundNormal;
    bool onGround = false;
    
    PacejkaModel tyreModel;
    
    this(Vector3f position, float facing, Vehicle vehicle)
    {
        super(vehicle);
        this.vehicle = vehicle;
        this.position = position;
        this.facing = facing;
        
        suspension.minLength = 0.2f;
        suspension.maxLength = 0.3f;
        suspension.stiffness = 100.0f;
        suspension.damping = 10.0f;
        suspension.compression = 0.0f;
        suspension.length = 0.0f;
        suspension.lengthPrev = 0.0f;
    }
    
    float onRayHit(NewtonRigidBody nbody, Vector3f hitPoint, Vector3f hitNormal, float t)
    {
        if (t < closestHitRayParam)
        {
            groundPosition = hitPoint;
            groundNormal = hitNormal;
            closestHitRayParam = t;
            return t;
        }
        else
        {
            return 1.0f;
        }
    }
    
    bool raycast(Vector3f pstart, Vector3f pend)
    {
        closestHitRayParam = 1.0f;
        vehicle.world.raycast(pstart, pend, this);
        groundPosition = pstart + (pend - pstart).normalized * maxRayDistance * closestHitRayParam;
        return (closestHitRayParam < 1.0f);
    }
    
    void update(double dt)
    {
        camberAngle = clamp(camberAngle, -4.0f, 4.0f);
        
        Vector3f upVectorWorld = verticalAxis();
        Vector3f rayDir = -upVectorWorld;
        Vector3f suspPosition = position * vehicle.chassisBody.transformation;
        
        steering = rotationQuaternion!float(Axis.y, degtorad(steeringAngle));
        
        Vector3f forwardAxis = longitudinalAxis();
        Vector3f sideAxis = lateralAxis();
        Vector3f forcePosition = tyreContactPoint();
        
        bool hitGround = raycast(suspPosition, suspPosition + rayDir * maxRayDistance);
        if (!hitGround)
        {
            hitGround = true;
            groundPosition = suspPosition;
            groundPosition.y = 0.0f;
            groundNormal = Vector3f(0.0f, 1.0f, 0.0f);
        }
        
        float suspToGround = distance(suspPosition, groundPosition);
        
        angularVelocity = 0.0f;
        float angularAcceleration = 0.0f;
        
        if (!hitGround || (suspToGround > suspension.maxLength + radius)) // wheel is in air
        {
            onGround = false;
            
            suspension.lengthPrev = suspension.maxLength;
            suspension.length = suspension.maxLength;
            suspension.compression = 0.0f;
            
            normalForce = 0.0f;
            tractionForce = 0.0f;
            lateralFrictionForce = 0.0f;
            longitudinalFrictionForce = 0.0f;
            
            slipAngle = 0.0f;
            slipRatio = 0.0f;
            
            angularVelocity = torque / radius * invInertia * dt;
        }
        else // suspension is compressed
        {
            onGround = true;
            
            suspension.lengthPrev = suspension.length;
            suspension.length = max(0.0f, suspToGround - radius);
            suspension.compression = suspension.maxLength - suspension.length;
            
            // Normal force
            float wheelLoad = vehicle.chassisBody.mass * load;
            float springForce = suspension.compression * suspension.stiffness;
            float compressionSpeed = suspension.lengthPrev - suspension.length;
            float dampingForce = (compressionSpeed * suspension.damping) / dt;
            normalForce = (springForce + dampingForce) * wheelLoad;
            
            vehicle.chassisBody.addForceAtPos(groundNormal * normalForce, forcePosition);
            
            float chassisSpeed = vehicle.speed;
            Vector3f chassisVelocity = vehicle.velocity;
            Vector3f wheelVelocity = vehicle.chassisBody.pointVelocity(forcePosition);
            float wheelSpeed = wheelVelocity.length;
            float lateralSpeed = dot(wheelVelocity, sideAxis);
            float longitudinalDir = (dot(vehicle.chassisBody.velocity.normalized, forwardAxis) > 0.0f) ? 1.0f : -1.0f;
            
            float longitudinalSpeed = dot(wheelVelocity, forwardAxis);
            
            if (brake)
            {
                // Block the wheel
                angularAcceleration = 0.0f;
                angularVelocity = 0.0f;
                slipRatio = 1.0f;
            }
            else if (abs(torque) > 0.0f)
            {
                // Apply torque
                tractionForce = torque / radius * invInertia;
                vehicle.chassisBody.addForceAtPos(forwardAxis * tractionForce, forcePosition);
                angularAcceleration = tractionForce * 0.1f;
                slipRatio = clamp(abs((angularVelocity * radius) / max2(abs(longitudinalSpeed), 0.00001f)), 0.0f, 1.0f);
            }
            else
            {
                // Free spin
                angularVelocity = longitudinalSpeed / radius * invInertia;
                angularAcceleration = 0.0f;
                slipRatio = 0.0f;
            }
            
            slipAngle = atan(lateralSpeed / max2(abs(longitudinalSpeed), 0.00001f));
            
            // Friction force
            float idleThreshold = 1.0f;
            // speedFactor interpolates between static (0.0) and dynamic (1.0) friction
            float speedFactor = clamp(wheelSpeed / idleThreshold, 0.0f, 1.0f);
            float staticLateralFrictionForce = lateralSpeed / dt * wheelLoad * staticFrictionCoefficient;
            float dynamicLateralFrictionForce = tyreModel.lateralForce(normalForce, slipAngle, degtorad(camberAngle)) * lateralDynamicFrictionCoefficient;
            lateralFrictionForce = lerp(staticLateralFrictionForce, dynamicLateralFrictionForce, speedFactor);
            longitudinalFrictionForce = tyreModel.longitudinalForce(normalForce, slipRatio) * longitudinalDynamicFrictionCoefficient;
            vehicle.chassisBody.addForceAtPos(-sideAxis * lateralFrictionForce, forcePosition);
            vehicle.chassisBody.addForceAtPos(-forwardAxis * longitudinalDir * longitudinalFrictionForce, forcePosition);
        }
        
        angularVelocity += angularAcceleration * dt;
        if (abs(angularVelocity) > 0.5f)
        {
            float angularVelocityVisual = clamp(angularVelocity, -10.0f, 10.0f);
            roll += angularVelocityVisual * dt;
            roll = fmod(roll, 2.0f * PI);
        }
    }
    
    Vector3f tyreContactPoint() const
    {
        Vector3f tyreBottom = position - Vector3f(0.0f, suspension.length + radius, 0.0f);
        return tyreBottom * vehicle.chassisBody.transformation;
    }
    
    Vector3f verticalAxis()
    {
        return vehicle.verticalAxis;
    }
    
    Vector3f lateralAxis()
    {
        return steering.rotate(vehicle.lateralAxis * facing).normalized;
    }
    
    Vector3f longitudinalAxis()
    {
        return steering.rotate(vehicle.longitudinalAxis).normalized;
    }
    
    Vector3f localWheelPosition()
    {
        return position - Vector3f(0.0f, suspension.length, 0.0f);
    }
    
    Quaternionf localRotation()
    {
        float facingAngle = 90.0f - 90.0f * facing;
        return
            rotationQuaternion!float(Axis.y, degtorad(facingAngle + steeringAngle)) *
            rotationQuaternion!float(Axis.z, degtorad(-camberAngle)) *
            rotationQuaternion!float(Axis.x, roll * facing);
    }
}
