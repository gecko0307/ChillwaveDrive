module vehicle;

import std.stdio;
import std.algorithm;
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
    float tractionForce = 0.0f;
    float lateralFrictionForce = 0.0f;
    float staticLateralFrictionForce = 0.0f;
    float longitudinalFrictionForce = 0.0f;
    float load = 0.25f;
    float grip = 0.6f;
    float slipAngle = 0.0f;
    float slipRatio = 0.0f;
    float torque = 0.0f;
    float angularVelocity = 0.0f;
    float roll = 0.0f;
    float invInertia = 0.8f;
    float staticFrictionCoefficient = 0.95f;
    float lateralDynamicFrictionCoefficient = 0.75f;
    float longitudinalDynamicFrictionCoefficient = 0.75f;
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
        
        suspension.minLength = 0.2f;
        suspension.maxLength = 0.3f;
        suspension.stiffness = 100.0f;
        suspension.damping = 8.0f;
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
        
        Vector3f suspPosition = position * vehicle.chassisBody.transformation;
        
        steering = rotationQuaternion!float(Axis.y, degtorad(angle));
        
        Vector3f forwardAxis = longitudinalAxis();
        Vector3f sideAxis = lateralAxis();
        Vector3f forcePosition = tyreContactPoint();
        
        bool hitGround = raycast(suspPosition, suspPosition + downVectorWorld * maxRayDistance);
        float suspToGround = distance(suspPosition, groundPosition);
        
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
            suspension.length = suspToGround - radius;
            if (suspension.length < suspension.minLength)
                suspension.length = suspension.minLength;
            suspension.compression = suspension.maxLength - suspension.length;
            
            // Normal force
            float wheelLoad = vehicle.chassisBody.mass * load;
            float springForce = suspension.compression * suspension.stiffness;
            float compressionSpeed = suspension.lengthPrev - suspension.length;
            float dampingForce = (compressionSpeed * suspension.damping) / dt;
            normalForce = (springForce + dampingForce) * wheelLoad;
            vehicle.chassisBody.addForceAtPos(upVectorWorld * normalForce, forcePosition);
            
            float chassisSpeed = vehicle.speed;
            Vector3f chassisVelocity = vehicle.velocity;
            Vector3f tyreVelocity = vehicle.chassisBody.pointVelocity(forcePosition);
            float lateralSpeed = dot(tyreVelocity, sideAxis);
            float longitudinalDir = (dot(vehicle.chassisBody.velocity.normalized, forwardAxis) > 0.0f) ? 1.0f : -1.0f;
            float longitudinalSpeed = dot(chassisVelocity, forwardAxis);
            
            // Forward force
            if (abs(torque) > 0.0f)
            {
                tractionForce = torque / radius * grip * invInertia;
                vehicle.chassisBody.addForceAtPos(forwardAxis * tractionForce, forcePosition);
                angularVelocity = tractionForce * dt;
                slipRatio = clamp(abs((angularVelocity * radius) / max2(abs(longitudinalSpeed), 0.00001f)), 0.0f, 1.0f);
            }
            else
            {
                angularVelocity = longitudinalSpeed / radius * invInertia;
                slipRatio = 0.0f;
            }
            
            slipAngle = atan2(lateralSpeed, abs(longitudinalSpeed));
            
            // Friction force
            float idleThreshold = 0.5f;
            float speedFactor = clamp(chassisSpeed / idleThreshold, 0.0f, 1.0f);
            float staticLateralFrictionForce = lateralSpeed / dt * wheelLoad * staticFrictionCoefficient;
            float dynamicLateralFrictionForce = tyreModel.lateralForce(normalForce, slipAngle, 0.0f) * lateralDynamicFrictionCoefficient;
            lateralFrictionForce = lerp(staticLateralFrictionForce, dynamicLateralFrictionForce, speedFactor);
            longitudinalFrictionForce = tyreModel.longitudinalForce(normalForce, slipRatio) * longitudinalDynamicFrictionCoefficient;
            vehicle.chassisBody.addForceAtPos(-sideAxis * lateralFrictionForce, forcePosition);
            vehicle.chassisBody.addForceAtPos(-forwardAxis * longitudinalDir * longitudinalFrictionForce, forcePosition);
        }
        
        float angularVelocityVisual = clamp(angularVelocity, -15.0f, 15.0f);
        roll += angularVelocityVisual * dt;
        roll = fmod(roll, 2.0f * PI);
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
        float facingAngle = 90.0f - 90.0f * facing;
        return
            rotationQuaternion!float(Axis.y, degtorad(facingAngle + angle * 0.5f)) *
            rotationQuaternion!float(Axis.x, roll * facing);
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
    
    float engineHPower = 150.0f;
    float engineTorque = 0.0f;
    float maxRPM = 6000.0f;
    // 1st gear: 3.5-4.5
    // 2nd gear: 2.0-2.5
    // 3rd gear: 1.5-1.8
    // 4th gear: 1.0-1.3
    // 5th gear: 0.8-1.0
    float[5] gearRatio = [3.5, 2.0, 1.5, 1.0, 0.8];
    uint gear = 0;
    float torquePerWheel = 0.0f;
    
    float throttle = 0.0f;
    float direction = 1.0f;
    
    float steeringAngle = 0.0f;
    float maxSteeringAngle = 45.0f;
    
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
        
        float suspensionPos = 0.2f;
        wheels[0] = New!Wheel(Vector3f(-0.75f, suspensionPos, +1.35f), -1.0f, this);
        wheels[1] = New!Wheel(Vector3f(+0.75f, suspensionPos, +1.35f), +1.0f, this);
        wheels[2] = New!Wheel(Vector3f(-0.75f, suspensionPos, -1.35f), -1.0f, this);
        wheels[3] = New!Wheel(Vector3f(+0.75f, suspensionPos, -1.35f), +1.0f, this);
        
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
    
    void setDirection(float dir)
    {
        direction = dir;
    }
    
    void pullAccelerator(float delta)
    {
        if (throttle < 1.0f)
            throttle += delta;
        else
            throttle = 1.0f;
    }
    
    void releaseAccelerator(float delta)
    {
        if (throttle > 0.0f)
            throttle -= delta;
        else
            throttle = 0.0f;
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
        float steeringDecreaseStep = 80.0f * t.delta;
        if (steeringAngle > steeringDecreaseStep)
            steeringAngle -= steeringDecreaseStep;
        else if (steeringAngle < -steeringDecreaseStep)
            steeringAngle += steeringDecreaseStep;
        else
            steeringAngle = 0.0f;
        
        wheels[0].angle = steeringAngle;
        wheels[1].angle = steeringAngle;
        
        float diffGearRatio = 3.91f;
        float transmissionEfficiency = 0.9f;
        float differentialEfficiency = 0.9f;
        float torqueLoss = transmissionEfficiency * differentialEfficiency;
        
        // TODO:
        //float wheelRPM = (abs(wheel.angularVelocity) * 60.0f) / (2.0f * M_PI);
        //float rpm = wheelRPM * gearRatio[gear] * diffGearRatio;
        
        float rpm = 2000.0f;
        
        if (rpm > maxRPM)
            rpm = maxRPM;
        
        // TODO:
        //float engineTorque = getTorqueFromRPM(rpm);
        
        engineTorque = (engineHPower * 5252.0f) / rpm;
        
        torquePerWheel = engineTorque * throttle * direction * gearRatio[gear] * diffGearRatio * 0.5f * torqueLoss;
        
        wheels[0].torque = torquePerWheel;
        wheels[1].torque = torquePerWheel;
        
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
