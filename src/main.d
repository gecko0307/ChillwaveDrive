module main;

import std.stdio;
import std.conv;
import std.meta;
import std.math;
import dagon;
import dagon.ext.newton;
import soloud;
import vehicle;
import view;

class GameScene: Scene
{
    VehicleDemoGame game;
    Soloud audio;
    
    GLTFAsset aTrack;
    
    Camera camera;
    VehicleViewComponent vehicleView;
    
    NewtonPhysicsWorld physicsWorld;
    
    OBJAsset aChassis;
    OBJAsset aWheel;
    TextureAsset aTexParticleDust;
    
    Entity eCar;
    Entity eWheel1, eWheel2, eWheel3, eWheel4;
    Vehicle car;
    
    ParticleSystem particleSystem;
    Emitter emitterLeft;
    Emitter emitterRight;
    
    Wav sfxEngine;
    Wav sfxSquealLoop;
    Wav[2] sfxHit;
    
    int engineVoice;
    int squealVoice;
    int hitVoice;

    this(VehicleDemoGame game)
    {
        super(game);
        this.game = game;
        this.audio = game.audio;
    }

    override void beforeLoad()
    {
        aTrack = addGLTFAsset("data/track/track.gltf");
        aChassis = addOBJAsset("data/car/chassis.obj");
        aWheel = addOBJAsset("data/car/wheel.obj");
        aTexParticleDust = addTextureAsset("data/particles/dust.png");
        
        // Sounds
        sfxEngine = Wav.create();
        sfxEngine.load("data/sounds/engine.wav");
        sfxEngine.setVolume(0.2f);
        sfxEngine.set3dDistanceDelay(true);
        
        sfxSquealLoop = Wav.create();
        sfxSquealLoop.load("data/sounds/squeal.wav");
        sfxSquealLoop.setVolume(0.5f);
        sfxSquealLoop.set3dDistanceDelay(true);
        
        sfxHit[0] = Wav.create();
        sfxHit[0].load("data/sounds/hit1.wav");
        sfxHit[0].setVolume(0.5f);
        sfxHit[0].set3dDistanceDelay(true);
        
        sfxHit[1] = Wav.create();
        sfxHit[1].load("data/sounds/hit2.wav");
        sfxHit[1].setVolume(0.5f);
        sfxHit[1].set3dDistanceDelay(true);
    }

    override void afterLoad()
    {
        game.deferredRenderer.ssaoEnabled = true;
        game.deferredRenderer.ssaoPower = 6.0f;
        game.postProcessingRenderer.fStop = 1.0;
        game.postProcessingRenderer.glowEnabled = true;
        game.postProcessingRenderer.glowThreshold = 1.0f;
        game.postProcessingRenderer.glowIntensity = 0.25f;
        game.postProcessingRenderer.glowRadius = 7;
        game.postProcessingRenderer.exposure = 1.0f;
        game.postProcessingRenderer.tonemapper = Tonemapper.AgX_Punchy;
        game.postProcessingRenderer.fxaaEnabled = true;
        game.postProcessingRenderer.motionBlurEnabled = true;
        game.postProcessingRenderer.motionBlurFramerate = 60;
        
        environment.backgroundColor = Color4f(0.7f, 0.8f, 1.0f, 1.0f);
        environment.fogColor = environment.backgroundColor;
        environment.fogStart = 10.0f;
        environment.fogEnd = 100.0f;
        
        environment.ambientColor = Color4f(0.7f, 0.8f, 1.0f, 1.0f);
        environment.ambientEnergy = 0.5f;
        
        physicsWorld = New!NewtonPhysicsWorld(eventManager, assetManager);
        
        camera = addCamera();
        camera.fov = 80.0f;
        game.renderer.activeCamera = camera;
        
        auto sun = addLight(LightType.Sun);
        sun.color = Color4f(1.0f, 0.9f, 0.8f, 1.0f);
        sun.shadowEnabled = true;
        sun.energy = 10.0f;
        sun.turn(220.0f);
        sun.pitch(-20.0f);
        sun.scatteringEnabled = false;
        sun.scattering = 0.2f;
        sun.mediumDensity = 0.1f;
        sun.scatteringUseShadow = false;
        sun.scatteringMaxRandomStepOffset = 0.5f;
        environment.sun = sun;
        
        auto eSky = addEntity();
        auto psync = New!PositionSync(eventManager, eSky, camera);
        eSky.drawable = New!ShapeBox(Vector3f(1.0f, 1.0f, 1.0f), assetManager);
        eSky.scaling = Vector3f(100.0f, 100.0f, 100.0f);
        eSky.layer = EntityLayer.Background;
        eSky.gbufferMask = 1.0f;
        eSky.material = New!Material(assetManager);
        eSky.material.depthWrite = false;
        eSky.material.useCulling = false;
        eSky.material.useFog = false;
        auto skyShader = New!RayleighShader(assetManager);
        eSky.material.shader = skyShader;
        
        // Track
        aTrack.markTransparentEntities();
        useEntity(aTrack.rootEntity);
        foreach(node; aTrack.nodes)
        {
            useEntity(node.entity);
        }
        foreach(mat; aTrack.materials)
        {
            mat.useCulling = false;
        }
        
        auto eTrack = addEntity();
        auto trackShape = New!NewtonMeshShape(aTrack, physicsWorld);
        eTrack.makeStaticBody(physicsWorld, trackShape);
        
        // Car
        eCar = addEntity();
        eCar.blurMask = 0.0f;
        eCar.position = Vector3f(0.0f, 26.0f, 210.0f);
        Vector3f chassisSizeBottom = Vector3f(1.75f, 0.5f, 3.0f);
        Vector3f chassisSizeTop = Vector3f(1.5f, 0.5f, 1.5f);
        Vector3f chassisTopPosition = Vector3f(0.0f, 0.5f, -0.25f);
        eCar.drawable = aChassis.mesh;
        eCar.material = addMaterial();
        eCar.material.baseColorFactor = Color4f(1.0f, 0.4f, 0.3f, 1.0f);
        eCar.material.roughnessFactor = 0.01f;
        eCar.blurMask = 0.0f;
        
        auto chassisShapeBottom = New!NewtonBoxShape(chassisSizeBottom, physicsWorld);
        auto chassisShapeTop = New!NewtonBoxShape(chassisSizeTop, physicsWorld);
        chassisShapeTop.setTransformation(translationMatrix(chassisTopPosition));
        auto chassisShape = New!NewtonCompoundShape(cast(NewtonCollisionShape[])[chassisShapeBottom, chassisShapeTop], physicsWorld);
        float carMass = 2000.0f;
        car = New!Vehicle(physicsWorld, eCar, chassisShape, carMass, 1);
        car.chassisBody.centerOfMass = Vector3f(0.0f, -0.5f, 0.0f); // -0.5f
        car.setInertia(carMass, boxInertia(chassisSizeBottom * Vector3f(1.0f, 0.0f, 1.0f), carMass));
        
        auto wheelMaterial = addMaterial();
        wheelMaterial.baseColorFactor = Color4f(0.4f, 0.4f, 0.4f, 1.0f);
        
        float wheelScale = 0.35f / 0.4f;
        Vector3f wheelScaleV = Vector3f(wheelScale, wheelScale, wheelScale);
        
        eWheel1 = addEntity(eCar);
        eWheel1.drawable = aWheel.mesh;
        eWheel1.position = car.wheels[0].localWheelPosition;
        eWheel1.material = wheelMaterial;
        eWheel1.blurMask = 0.0f;
        eWheel1.scaling = wheelScaleV;
        
        eWheel2 = addEntity(eCar);
        eWheel2.drawable = aWheel.mesh;
        eWheel2.position = car.wheels[1].localWheelPosition;
        eWheel2.material = wheelMaterial;
        eWheel2.blurMask = 0.0f;
        eWheel2.scaling = wheelScaleV;
        
        eWheel3 = addEntity(eCar);
        eWheel3.drawable = aWheel.mesh;
        eWheel3.position = car.wheels[2].localWheelPosition;
        eWheel3.material = wheelMaterial;
        eWheel3.blurMask = 0.0f;
        eWheel3.scaling = wheelScaleV;
        
        eWheel4 = addEntity(eCar);
        eWheel4.drawable = aWheel.mesh;
        eWheel4.position = car.wheels[3].localWheelPosition;
        eWheel4.material = wheelMaterial;
        eWheel4.blurMask = 0.0f;
        eWheel4.scaling = wheelScaleV;
        
        auto eParticles = addEntity();
        particleSystem = New!ParticleSystem(eventManager, eParticles);
        
        // Dust particle systems
        auto mParticlesDust = addMaterial();
        mParticlesDust.baseColorTexture = aTexParticleDust.texture;
        mParticlesDust.blendMode = Transparent;
        mParticlesDust.depthWrite = false;
        mParticlesDust.emissionEnergy = 0.5f;
        mParticlesDust.sun = sun;

        auto eParticlesRight = addEntity(eCar);
        emitterRight = New!Emitter(eParticlesRight, particleSystem, 30);
        eParticlesRight.position = Vector3f(-0.9f, -0.5f, -1.4f);
        emitterRight.minLifetime = 0.1f;
        emitterRight.maxLifetime = 2.0f;
        emitterRight.minSize = 0.25f;
        emitterRight.maxSize = 0.5f;
        emitterRight.minInitialSpeed = 0.2f;
        emitterRight.maxInitialSpeed = 0.2f;
        emitterRight.scaleStep = Vector2f(2, 2);
        emitterRight.material = mParticlesDust;
        eParticlesRight.castShadow = false;
        eParticlesRight.visible = true;

        auto eParticlesLeft = addEntity(eCar);
        emitterLeft = New!Emitter(eParticlesLeft, particleSystem, 30);
        eParticlesLeft.position = Vector3f(0.9f, -0.5f, -1.4f);
        emitterLeft.minLifetime = 0.1f;
        emitterLeft.maxLifetime = 2.0f;
        emitterLeft.minSize = 0.25f;
        emitterLeft.maxSize = 0.5f;
        emitterLeft.minInitialSpeed = 0.2f;
        emitterLeft.maxInitialSpeed = 0.2f;
        emitterLeft.scaleStep = Vector2f(2, 2);
        emitterLeft.material = mParticlesDust;
        eParticlesLeft.castShadow = false;
        eParticlesLeft.visible = true;
        
        vehicleView = New!VehicleViewComponent(eventManager, camera, car);
        eventManager.showCursor(false);
        
        engineVoice = audio.play3d(sfxEngine, car.position.x, car.position.y, car.position.z);
        audio.setLooping(engineVoice, true);
        audio.set3dSourceMinMaxDistance(engineVoice, 1.0f, 50.0f);
        audio.setVolume(engineVoice, 0.5f);
        audio.update3dAudio();
        
        squealVoice = audio.play3d(sfxSquealLoop, car.position.x, car.position.y, car.position.z);
        audio.setVolume(squealVoice, 0.0f);
        audio.setLooping(squealVoice, true);
    }
    
    override void onKeyDown(int key)
    {
        if (key == KEY_ESCAPE)
            application.exit();
    }
    
    override void onMouseButtonUp(int button)
    {
        vehicleView.active = !vehicleView.active;
        eventManager.showCursor(!vehicleView.active);
    }
    
    override void onUpdate(Time t)
    {
        float speed = 5.0f * t.delta;
        if (inputManager.getButton("forward")) car.accelerate(200);
        else if (inputManager.getButton("back")) car.accelerate(-200);
        float axis = inputManager.getAxis("horizontal");
        car.steer(-axis * 8.0f);
        
        eWheel1.position = car.wheels[0].localWheelPosition;
        eWheel2.position = car.wheels[1].localWheelPosition;
        eWheel3.position = car.wheels[2].localWheelPosition;
        eWheel4.position = car.wheels[3].localWheelPosition;
        
        eWheel1.rotation = car.wheels[0].localRotation;
        eWheel2.rotation = car.wheels[1].localRotation;
        eWheel3.rotation = car.wheels[2].localRotation;
        eWheel4.rotation = car.wheels[3].localRotation;
        
        car.update(t);
        
        float speedKMH = car.longitudinalSpeedKMH;
        float lateralSpeedKMH = car.lateralSpeedKMH;
        
        // Engine sound
        audio.set3dSourcePosition(engineVoice, car.position.x, car.position.y, car.position.z);
        // TODO: use RPM instead of actual speed
        float engineSoundSpeed = lerp(1.0f, 1.75f, clamp(abs(car.torque) / 5000.0f, 0.0f, 1.0f));
        audio.setRelativePlaySpeed(engineVoice, engineSoundSpeed);
        
        // Tire squeal sound
        float lateralSlip = car.lateralSlip;
        float longitudinalSlip = 0.0f; //car.longitudinalSlip;
        float lateralSquealVolume = clamp((lateralSlip - 0.5f) / 0.5f, 0.0f, 1.0f);
        lateralSquealVolume *= clamp((lateralSpeedKMH - 10.0f) / 10.0f, 0.0f, 1.0f);
        float longitudinalSquealVolume = clamp((longitudinalSlip - 0.3f) / 0.3f, 0.0f, 1.0f);
        longitudinalSquealVolume *= clamp((speedKMH - 10.0f) / 10.0f, 0.0f, 1.0f);
        float squealVolume = clamp(lateralSquealVolume + longitudinalSquealVolume, 0.0f, 1.0f);
        audio.setVolume(squealVoice, squealVolume * 0.8f);
        audio.setRelativePlaySpeed(squealVoice, lateralSlip);
        audio.set3dSourcePosition(squealVoice, car.position.x, car.position.y, car.position.z);
        
        // Dust particles
        if (squealVolume > 0.0f) emitterLeft.emitting = true;
        else emitterLeft.emitting = false;
        if (squealVolume > 0.0f) emitterRight.emitting = true;
        else emitterRight.emitting = false;
        
        physicsWorld.update(t.delta);
        
        // Feed camera data to 3D listener
        audio.set3dListenerPosition(camera.positionAbsolute.x, camera.positionAbsolute.y, camera.positionAbsolute.z);
        audio.set3dListenerAt(camera.directionAbsolute.x, camera.directionAbsolute.y, camera.directionAbsolute.z);
        audio.set3dListenerUp(camera.upAbsolute.x, camera.upAbsolute.y, camera.upAbsolute.z);
        audio.update3dAudio();
    }
}

class VehicleDemoGame: Game
{
    Soloud audio;
    
    this(uint w, uint h, bool fullscreen, string title, string[] args)
    {
        super(w, h, fullscreen, title, args);
        audio = Soloud.create();
        audio.init(Soloud.CLIP_ROUNDOFF | Soloud.LEFT_HANDED_3D);
        currentScene = New!GameScene(this);
    }
}

void main(string[] args)
{
    import loader = bindbc.loader.sharedlib;
    NewtonSupport sup = loadNewton();
    foreach(info; loader.errors)
    {
        writeln(info.error.to!string, " ", info.message.to!string);
    }
    
    loadSoloud();
    
    VehicleDemoGame game = New!VehicleDemoGame(1280, 720, false, "Dagon vehicle demo", args);
    game.run();
    Delete(game);
}
