module vehicle;

import std.stdio;

import std.math;
import dagon;
import dagon.ext.newton;
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
    float angle = 0.0f;
    float facing = 0.0f;
    float normalForce = 0.0f;
    float load = 0.25f;
    float grip = 1.0f;
    float slipAngle = 0.0f;
    float slipRatio = 0.0f;
    float torque = 0.0f;
    float angularVelocity = 0.0f;
    float roll = 0.0f;
    Quaternionf steering = Quaternionf.identity;
    
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
        
        suspension.minLength = 0.1f;
        suspension.maxLength = 0.25f;
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
        Vector3f upVectorWorld = vehicle.verticalAxis();
        Vector3f downVectorWorld = -upVectorWorld;
        
        Vector3f positionWorld = position * vehicle.chassisBody.transformation;
        
        steering = rotationQuaternion!float(Axis.y, degtorad(angle));
        
        bool hitGround = raycast(positionWorld, positionWorld + downVectorWorld * maxRayDistance);
        float suspToGround = distance(positionWorld, groundPosition);
        
        if (!hitGround || (suspToGround > suspension.maxLength + radius)) // wheel is in air
        {
            onGround = false;
            
            suspension.lengthPrev = suspension.maxLength;
            suspension.length = suspension.maxLength;
            suspension.compression = 0.0f;
            
            normalForce = 0.0f;
            
            slipAngle = 0.0f;
            slipRatio = 0.0f;
            
            angularVelocity = torque * dt;
        }
        else // suspension is compressed
        {
            onGround = true;
            
            suspension.lengthPrev = suspension.length;
            suspension.length = suspToGround - radius;
            if (suspension.length < suspension.minLength)
                suspension.length = suspension.minLength;
            suspension.compression = suspension.maxLength - suspension.length;
            
            float wheelLoad = vehicle.chassisBody.mass * load;
            float springForce = suspension.compression * suspension.stiffness;
            float compressionSpeed = suspension.lengthPrev - suspension.length;
            float dampingForce = (compressionSpeed * suspension.damping) / dt;
            normalForce = (springForce + dampingForce) * wheelLoad;
            vehicle.chassisBody.addForceAtPos(upVectorWorld * normalForce, positionWorld);
            
            Vector3f forwardAxis = longitudinalAxis();
            Vector3f sideAxis = lateralAxis();
            Vector3f forcePosition = tyreContactPoint();
            
            if (abs(torque) > 0.0f)
            {
                float tractionForce = torque / radius;
                vehicle.chassisBody.addForceAtPos(forwardAxis * tractionForce, forcePosition);
            }
            
            // Friction
            Vector3f tyreVelocity = vehicle.chassisBody.pointVelocity(forcePosition);
            float lateralSpeed = dot(tyreVelocity, sideAxis);
            float longitudinalSpeed = dot(tyreVelocity, forwardAxis);
            slipAngle = atan2(lateralSpeed, abs(longitudinalSpeed));
            slipRatio = 1.0f - clamp((angularVelocity * radius) / max2(abs(longitudinalSpeed), 0.00001f), 0.0f, 1.0f);
            float latFForce = tyreModel.lateralForce(normalForce, slipAngle, 0.0f);
            float longFForce = tyreModel.longitudinalForce(normalForce, slipRatio);
            vehicle.chassisBody.addForceAtPos(sideAxis * -latFForce, forcePosition);
            vehicle.chassisBody.addForceAtPos(forwardAxis * longFForce, forcePosition);
            
            angularVelocity = longitudinalSpeed / radius;
        }
        
        roll += radtodeg(angularVelocity) * dt;
        if (roll > 360.0f)
            roll -= 360.0f;
    }
    
    Vector3f tyreContactPoint() const
    {
        Vector3f tyreBottom = position - Vector3f(0.0f, suspension.length + radius, 0.0f);
        return tyreBottom * vehicle.chassisBody.transformation;
    }
    
    Vector3f lateralAxis()
    {
        return steering.rotate(vehicle.chassisBody.transformation.right * facing).normalized;
    }
    
    Vector3f longitudinalAxis()
    {
        return steering.rotate(vehicle.chassisBody.transformation.forward).normalized;
    }
    
    float getLateralFrictionForce()
    {
        Vector3f wheelVelocity = vehicle.chassisBody.pointVelocity(tyreContactPoint);
        float lateralSpeed = abs(dot(wheelVelocity, lateralAxis));
        return lerp(10000.0f, 5000.0f, clamp((lateralSpeed - 2.0f) / (8.0f - 2.0f), 0.0f, 1.0f));
    }
    
    float getLongitudinalFrictionForce()
    {
        return 100.0f;
    }
    
    Vector3f localWheelPosition()
    {
        return position - Vector3f(0.0f, suspension.length, 0.0f);
    }
    
    Quaternionf localRotation()
    {
        return
            rotationQuaternion!float(Axis.y, degtorad(angle * 0.5f)) *
            rotationQuaternion!float(Axis.x, degtorad(roll));
    }
}

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
    Wheel[4] wheels;
    
    float torque = 0.0f;
    float maxTorque = 5000.0f;
    
    float steeringAngle = 0.0f;
    float maxSteeringAngle = 40.0f;
    
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
        //this.chassisBody.collisionCallback = &onCollision;
        
        NewtonBodySetContinuousCollisionMode(chassisBody.newtonBody, 1);
        NewtonBodySetMatrix(chassisBody.newtonBody, chassisBody.transformation.arrayof.ptr);
        
        NewtonMaterialSetDefaultFriction(world.newtonWorld, 0, materialID, 0.2f, 0.2f);
        NewtonMaterialSetDefaultElasticity(world.newtonWorld, 0, materialID, 0.2f);
        //NewtonMaterialSetCollisionCallback(world.newtonWorld, materialID, world.defaultGroupId, null, &chassisContactsProcess);
        
        float suspensionPos = -0.1f;
        wheels[0] = New!Wheel(Vector3f(-0.8f, suspensionPos, +1.0f), -1.0f, this);
        wheels[1] = New!Wheel(Vector3f(+0.8f, suspensionPos, +1.0f), +1.0f, this);
        wheels[2] = New!Wheel(Vector3f(-0.8f, suspensionPos, -1.0f), -1.0f, this);
        wheels[3] = New!Wheel(Vector3f(+0.8f, suspensionPos, -1.0f), +1.0f, this);
        
        prevTransformation = Matrix4x4f.identity;
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
    
    Vector3f forwardAxis()
    {
        return chassisBody.transformation.forward;
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
    
    bool gas = false;
    
    void accelerate(float t)
    {
        torque += t;
        gas = true;
    }
    
    void steer(float angle)
    {
        steeringAngle += angle;
        
        if (steeringAngle > maxSteeringAngle)
            steeringAngle = maxSteeringAngle;
        if (steeringAngle < -maxSteeringAngle)
            steeringAngle = -maxSteeringAngle;
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
        float torqueSign = sign(torque);
        float absTorque = abs(torque);
        if (absTorque > maxTorque)
            absTorque = maxTorque;
        torque = absTorque * torqueSign;
        
        if (!gas)
        {
            float torqueDecreaseStep = 5000.0f * t.delta;
            if (absTorque > 0.0f)
                absTorque -= torqueDecreaseStep;
            else
                absTorque = 0.0f;
            torque = absTorque * torqueSign;
        }
        
        gas = false;
        
        float steeringDecreaseStep = 80.0f * t.delta;
        if (steeringAngle > steeringDecreaseStep)
            steeringAngle -= steeringDecreaseStep;
        else if (steeringAngle < -steeringDecreaseStep)
            steeringAngle += steeringDecreaseStep;
        else
            steeringAngle = 0.0f;
        
        wheels[0].torque = torque * 0.5f;
        wheels[1].torque = torque * 0.5f;
        wheels[0].angle = steeringAngle;
        wheels[1].angle = steeringAngle;
        
        foreach(w; wheels)
        {
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
    }
}
