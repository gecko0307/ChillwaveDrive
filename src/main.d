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
    
    TextureAsset aTexEnvmap;
    
    GLTFAsset aChassis;
    GLTFAsset aWheel;
    
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
    
    float volume = 0.25f;

    this(VehicleDemoGame game)
    {
        super(game);
        this.game = game;
        this.audio = game.audio;
    }

    override void beforeLoad()
    {
        aTrack = addGLTFAsset("data/track/track.gltf");
        aChassis = addGLTFAsset("data/cars/coupe.gltf");
        aWheel = addGLTFAsset("data/cars/wheel1.gltf");
        aTexParticleDust = addTextureAsset("data/particles/dust.png");
        
        aTexEnvmap = addTextureAsset("data/envmaps/envmap.hdr");
        
        // Sounds
        sfxEngine = Wav.create();
        sfxEngine.load("data/sounds/engine.wav");
        sfxEngine.setVolume(volume * 0.2f);
        sfxEngine.set3dDistanceDelay(true);
        
        sfxSquealLoop = Wav.create();
        sfxSquealLoop.load("data/sounds/squeal.wav");
        sfxSquealLoop.setVolume(volume * 0.5f);
        sfxSquealLoop.set3dDistanceDelay(true);
        
        sfxHit[0] = Wav.create();
        sfxHit[0].load("data/sounds/hit1.wav");
        sfxHit[0].setVolume(volume * 0.5f);
        sfxHit[0].set3dDistanceDelay(true);
        
        sfxHit[1] = Wav.create();
        sfxHit[1].load("data/sounds/hit2.wav");
        sfxHit[1].setVolume(volume * 0.5f);
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
        
        environment.ambientEnergy = 0.4f;
        environment.ambientMap = aTexEnvmap.texture;
        
        physicsWorld = New!NewtonPhysicsWorld(eventManager, assetManager);
        
        camera = addCamera();
        camera.fov = 80.0f;
        game.renderer.activeCamera = camera;
        
        auto sun = addLight(LightType.Sun);
        sun.color = Color4f(1.0f, 0.9f, 0.8f, 1.0f);
        sun.shadowEnabled = true;
        sun.energy = 10.0f;
        sun.turn(220.0f);
        sun.pitch(-30.0f);
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
        eCar.position = Vector3f(0.0f, 26.0f, 210.0f);
        Vector3f chassisSizeBottom = Vector3f(1.76917f, 0.777934f, 4.77418f);
        Vector3f chassisSizeTop = Vector3f(1.54f, 0.492f, 2.39f);
        Vector3f chassisBottomPosition = Vector3f(0.0f, 0.22f, 0.0f);
        Vector3f chassisTopPosition = Vector3f(0.0f, 0.479246f, -0.839547f);
        eCar.drawable = aChassis.meshes[0];
        eCar.blurMask = 0.0f;
        
        auto eCarWindows = addEntity(eCar);
        eCarWindows.drawable = aChassis.meshes[1];
        eCarWindows.transparent = true;
        eCarWindows.castShadow = false;
        
        auto chassisShapeBottom = New!NewtonBoxShape(chassisSizeBottom, physicsWorld);
        chassisShapeBottom.setTransformation(translationMatrix(chassisBottomPosition));
        auto chassisShapeTop = New!NewtonBoxShape(chassisSizeTop, physicsWorld);
        chassisShapeTop.setTransformation(translationMatrix(chassisTopPosition));
        auto chassisShape = New!NewtonCompoundShape(cast(NewtonCollisionShape[])[chassisShapeBottom, chassisShapeTop], physicsWorld);
        float carMass = 1500.0f;
        car = New!Vehicle(physicsWorld, eCar, chassisShape, carMass, 1);
        car.chassisBody.centerOfMass = Vector3f(0.0f, -0.3f, 0.0f);
        car.setInertia(carMass, boxInertia(chassisSizeBottom * Vector3f(1.0f, 1.0f, 1.0f), carMass));
        
        foreach(ref w; car.wheels)
            w.radius = 0.337f;
        
        eWheel1 = addEntity(eCar);
        eWheel1.drawable = aWheel.meshes[0];
        eWheel1.position = car.wheels[0].localWheelPosition;
        eWheel1.blurMask = 0.0f;
        
        eWheel2 = addEntity(eCar);
        eWheel2.drawable = aWheel.meshes[0];
        eWheel2.position = car.wheels[1].localWheelPosition;
        eWheel2.blurMask = 0.0f;
        
        eWheel3 = addEntity(eCar);
        eWheel3.drawable = aWheel.meshes[0];
        eWheel3.position = car.wheels[2].localWheelPosition;
        eWheel3.blurMask = 0.0f;
        
        eWheel4 = addEntity(eCar);
        eWheel4.drawable = aWheel.meshes[0];
        eWheel4.position = car.wheels[3].localWheelPosition;
        eWheel4.blurMask = 0.0f;
        
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
        eParticlesRight.position = Vector3f(-0.9f, -1.0f, 1.4f);
        emitterRight.minLifetime = 1.0f;
        emitterRight.maxLifetime = 3.0f;
        emitterRight.minSize = 1.0f;
        emitterRight.maxSize = 2.0f;
        emitterRight.minInitialSpeed = 0.2f;
        emitterRight.maxInitialSpeed = 0.2f;
        emitterRight.scaleStep = Vector2f(2, 2);
        emitterRight.material = mParticlesDust;
        eParticlesRight.castShadow = false;
        eParticlesRight.visible = true;

        auto eParticlesLeft = addEntity(eCar);
        emitterLeft = New!Emitter(eParticlesLeft, particleSystem, 30);
        eParticlesLeft.position = Vector3f(0.9f, -1.0f, 1.4f);
        emitterLeft.minLifetime = 1.0f;
        emitterLeft.maxLifetime = 3.0f;
        emitterLeft.minSize = 1.0f;
        emitterLeft.maxSize = 2.0f;
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
        audio.setVolume(engineVoice, volume * 0.5f);
        audio.update3dAudio();
        
        squealVoice = audio.play3d(sfxSquealLoop, car.position.x, car.position.y, car.position.z);
        audio.setVolume(squealVoice, volume * 0.0f);
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
    
    float lastDirection = 0.0f;
    
    override void onUpdate(Time t)
    {
        if (inputManager.getButton("forward")) 
            car.accelerate(1.0f, 2.0f * t.delta);
        else if (inputManager.getButton("back"))
            car.accelerate(-1.0f, 2.0f * t.delta);
        else
            car.idle();
        
        float axis = inputManager.getAxis("horizontal");
        car.steer(-axis * 8.0f * t.delta);
        
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
        float engineSoundSpeed = lerp(1.0f, 1.5f, car.throttle);
        audio.setRelativePlaySpeed(engineVoice, engineSoundSpeed);
        
        // Tire squeal sound
        float lateralSlip = car.lateralSlip;
        float longitudinalSlip = car.longitudinalSlip;
        float squealVolume = clamp(lateralSlip, 0.0f, 1.0f);
        if (car.brake) squealVolume = clamp((speedKMH - 10.0f) / 10.0f, 0.0f, 1.0f);
        audio.setVolume(squealVoice, volume * squealVolume * 0.8f);
        audio.set3dSourcePosition(squealVoice, car.position.x, car.position.y, car.position.z);
        
        // Dust particles
        bool makingDust = lateralSlip > 0.0f || car.brake;
        if (makingDust && car.wheels[2].onGround) emitterLeft.emitting = true;
        else emitterLeft.emitting = false;
        if (makingDust && car.wheels[3].onGround) emitterRight.emitting = true;
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
