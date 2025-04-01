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
import atmosphere;

float normalizeInRange(float x, float xmin, float xmax)
{
    return clamp((x - xmin) / (xmax - xmin), 0.0f, 1.0f);
}

class GameScene: Scene
{
    VehicleDemoGame game;
    Soloud audio;
    
    GLTFAsset aTrack;
    
    Camera camera;
    VehicleViewComponent vehicleView;
    
    NewtonPhysicsWorld physicsWorld;
    
    TextureAsset aCloudNoise;
    
    GLTFAsset aChassis;
    GLTFAsset aWheel;
    
    TextureAsset aTexParticleDust;
    
    Entity eCar;
    Entity eWheel1, eWheel2, eWheel3, eWheel4;
    Vehicle car;
    
    Light sun;
    AtmosphereShader skyShader;
    
    ParticleSystem particleSystem;
    Emitter emitterLeft;
    Emitter emitterRight;
    
    bool headlightsPressed = false;
    bool headlightsOn = true;
    float headlightsEnergy = 6.0f;
    Material headlightsMaterial;
    Light light1, light2, light3, light4;
    
    Color4f ambientClearSkyDay = Color4f(0.4f, 0.4f, 0.7f, 1.0f);
    Color4f ambientClearSkySunset = Color4f(0.6f, 0.4f, 0.7f, 1.0f);
    Color4f ambientClearSkyNight = Color4f(0.3f, 0.3f, 0.6f, 1.0f);
    
    Color4f ambientCloudySkyDay = Color4f(0.4f, 0.4f, 0.5f, 1.0f);
    Color4f ambientCloudySkySunset = Color4f(0.6f, 0.3f, 0.3f, 1.0f);
    Color4f ambientCloudySkyNight = Color4f(0.4f, 0.4f, 0.7f, 1.0f);
    
    Color4f fogDay = Color4f(0.4f, 0.5f, 0.7f, 1.0f);
    Color4f fogSunset = Color4f(0.6f, 0.4f, 0.7f, 1.0f);
    Color4f fogNight = Color4f(0.0f, 0.0f, 0.0f, 1.0f);

    WavStream music;
    Wav sfxEngine;
    Wav sfxEngine2;
    Wav sfxSquealLoop;
    Wav[2] sfxHit;
    
    int musicVoice;
    int engineVoice;
    int engine2Voice;
    int squealVoice;
    int hitVoice;
    
    float musicVolume = 0.15f;
    float sfxVolume = 0.3f;

    this(VehicleDemoGame game)
    {
        super(game);
        this.game = game;
        this.audio = game.audio;
    }

    override void beforeLoad()
    {
        // Track
        aTrack = addGLTFAsset("data/track/track.gltf");
        
        // Car
        aChassis = addGLTFAsset("data/car/coupe.gltf");
        aWheel = addGLTFAsset("data/car/wheel1.gltf");
        aTexParticleDust = addTextureAsset("data/particles/dust.png");
        
        // Environment
        aCloudNoise = addTextureAsset("data/atmosphere/noise.png");
        
        // Sounds
        sfxEngine = Wav.create();
        sfxEngine.load("data/sounds/engine.wav");
        sfxEngine.set3dDistanceDelay(true);
        
        sfxSquealLoop = Wav.create();
        sfxSquealLoop.load("data/sounds/squeal.wav");
        sfxSquealLoop.set3dDistanceDelay(true);
        
        sfxHit[0] = Wav.create();
        sfxHit[0].load("data/sounds/hit1.wav");
        sfxHit[0].set3dDistanceDelay(true);
        
        sfxHit[1] = Wav.create();
        sfxHit[1].load("data/sounds/hit2.wav");
        sfxHit[1].set3dDistanceDelay(true);
        
        music = WavStream.create();
        music.load("data/music/stellar_escape.mp3");
    }

    override void afterLoad()
    {
        game.deferredRenderer.ssaoEnabled = true;
        game.deferredRenderer.ssaoRadius = 0.5f;
        game.deferredRenderer.ssaoPower = 9.0f;
        game.postProcessingRenderer.glowEnabled = true;
        game.postProcessingRenderer.glowThreshold = 5.0f;
        game.postProcessingRenderer.glowIntensity = 0.1f;
        game.postProcessingRenderer.glowRadius = 2;
        game.postProcessingRenderer.exposure = 1.0f;
        game.postProcessingRenderer.tonemapper = Tonemapper.AgX_Punchy;
        game.postProcessingRenderer.fxaaEnabled = true;
        game.postProcessingRenderer.motionBlurEnabled = true;
        game.postProcessingRenderer.motionBlurFramerate = 30;
        game.postProcessingRenderer.lensDistortionEnabled = true;
        game.postProcessingRenderer.lensDistortionDispersion = 0.05f;
        
        environment.backgroundColor = Color4f(0.7f, 0.8f, 1.0f, 1.0f);
        environment.fogColor = environment.backgroundColor;
        environment.fogStart = 0.0f;
        environment.fogEnd = 300.0f;
        
        physicsWorld = New!NewtonPhysicsWorld(eventManager, assetManager);
        
        camera = addCamera();
        camera.fov = 60.0f;
        game.renderer.activeCamera = camera;
        
        sun = addLight(LightType.Sun);
        sun.color = Color4f(1.0f, 0.7f, 0.5f, 1.0f);
        sun.shadowEnabled = true;
        sun.energy = 15.0f;
        sun.turn(220.0f);
        sun.pitch(-10.0f);
        sun.scatteringEnabled = true;
        sun.scattering = 0.2f;
        sun.mediumDensity = 0.1f;
        sun.scatteringUseShadow = true;
        sun.scatteringMaxRandomStepOffset = 0.5f;
        environment.sun = sun;
        
        auto eSky = addEntity();
        auto psync = New!PositionSync(eventManager, eSky, camera);
        eSky.drawable = New!ShapeBox(Vector3f(1.0f, 1.0f, 1.0f), assetManager);
        eSky.scaling = Vector3f(100.0f, 100.0f, 100.0f);
        eSky.layer = EntityLayer.Background;
        eSky.gbufferMask = 0.0f;
        eSky.material = New!Material(assetManager);
        eSky.material.depthWrite = false;
        eSky.material.useCulling = false;
        eSky.material.useFog = false;
        skyShader = New!AtmosphereShader(assetManager);
        skyShader.noiseTexture = aCloudNoise.texture;
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
        eCar.position = Vector3f(0.0f, 25.2f, 210.0f);
        Vector3f chassisSizeBottom = Vector3f(1.76917f, 0.7f, 4.4f);
        Vector3f chassisSizeTop = Vector3f(1.54f, 0.492f, 2.39f);
        Vector3f chassisBottomPosition = Vector3f(0.0f, 0.22f, 0.0f);
        Vector3f chassisTopPosition = Vector3f(0.0f, 0.479246f, -0.839547f);
        eCar.drawable = aChassis.meshes[0];
        eCar.blurMask = 0.0f;
        
        headlightsMaterial = aChassis.materials[1];
        headlightsMaterial.emissionEnergy = headlightsEnergy;
        
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
        
        // Headlights
        light1 = addLight(LightType.Spot, eCar);
        light1.volumeRadius = 10.0f;
        light1.energy = 20.0f;
        light1.spotOuterCutoff = 45.0f;
        light1.position = Vector3f(-0.6f, 0.0f, 2.2f);
        light1.turn(180);
        
        light2 = addLight(LightType.Spot, eCar);
        light2.volumeRadius = 10.0f;
        light2.energy = 20.0f;
        light2.spotOuterCutoff = 45.0f;
        light2.position = Vector3f(0.6f, 0.0f, 2.2f);
        light2.turn(180);
        
        light3 = addLight(LightType.AreaSphere, eCar);
        light3.color = Color4f(1.0f, 0.0f, 0.0f, 1.0);
        light3.volumeRadius = 5.0f;
        light3.energy = 10.0f;
        light3.specular = 0.0f;
        light3.position = Vector3f(-0.6f, 0.0f, -2.5f);
        
        light4 = addLight(LightType.AreaSphere, eCar);
        light4.color = Color4f(1.0f, 0.0f, 0.0f, 1.0);
        light4.volumeRadius = 5.0f;
        light4.energy = 10.0f;
        light4.specular = 0.0f;
        light4.position = Vector3f(0.6f, 0.0f, -2.5f);
        
        auto eParticles = addEntity();
        particleSystem = New!ParticleSystem(eventManager, eParticles);
        
        // Dust particle systems
        auto mParticlesDust = addMaterial();
        mParticlesDust.baseColorTexture = aTexParticleDust.texture;
        mParticlesDust.blendMode = Transparent;
        mParticlesDust.depthWrite = false;
        mParticlesDust.sun = sun;
        mParticlesDust.opacity = 0.2f;
        mParticlesDust.useShadows = true;

        auto eParticlesRight = addEntity(eCar);
        emitterRight = New!Emitter(eParticlesRight, particleSystem, 30);
        eParticlesRight.position = Vector3f(-0.9f, -1.0f, 1.4f);
        emitterRight.minLifetime = 1.0f;
        emitterRight.maxLifetime = 3.0f;
        emitterRight.minSize = 0.5f;
        emitterRight.maxSize = 1.0f;
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
        audio.setVolume(engineVoice, sfxVolume);
        
        audio.update3dAudio();
        
        squealVoice = audio.play3d(sfxSquealLoop, car.position.x, car.position.y, car.position.z);
        audio.setVolume(squealVoice, sfxVolume);
        audio.setLooping(squealVoice, true);
        
        musicVoice = audio.play(music);
        audio.setLooping(musicVoice, true);
        audio.setVolume(musicVoice, musicVolume);
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
    
    float accelerationPressed = false;
    float carEngineSoundSpeed = 1.0f;
    
    float triggerForward = 0.0f;
    float triggerBackward = 0.0f;
    override void onJoystickAxisMotion(int axis, float value)
    {
        if (axis == GA_TRIGGERRIGHT)
        {
            triggerForward = value;
        }
        else if (axis == GA_TRIGGERLEFT)
        {
            triggerBackward = value;
        }
    }
    
    override void onUpdate(Time t)
    {
        // Car controls
        if (inputManager.getButton("forward"))
        {
            car.accelerate(1.0f, 2.0f * t.delta);
            if (carEngineSoundSpeed < 1.5f)
                carEngineSoundSpeed += t.delta * 0.5f;
        }
        else if (inputManager.getButton("back"))
        {
            car.accelerate(-1.0f, 2.0f * t.delta);
            if (carEngineSoundSpeed < 1.5f)
                carEngineSoundSpeed += t.delta * 0.5f;
        }
        else if (triggerForward > 0.0f)
        {
            car.accelerate(1.0f, 2.0f * triggerForward * t.delta);
            if (carEngineSoundSpeed < 1.5f)
                carEngineSoundSpeed += t.delta * 0.5f;
        }
        else if (triggerBackward > 0.0f)
        {
            car.accelerate(-1.0f, 2.0f * triggerBackward * t.delta);
            if (carEngineSoundSpeed < 1.5f)
                carEngineSoundSpeed += t.delta * 0.5f;
        }
        else
        {
            car.idle();
            
            if (carEngineSoundSpeed > 1.0f)
                carEngineSoundSpeed -= t.delta * 0.5f;
        }
        
        float axis = inputManager.getAxis("horizontal");
        car.steer(-axis * 8.0f * t.delta);
        
        // Headlights on/off
        if (inputManager.getButton("headlights"))
        {
            if (!headlightsPressed)
            {
                headlightsPressed = true;
                headlightsOn = !headlightsOn;
                light1.shining = headlightsOn;
                light2.shining = headlightsOn;
                light3.shining = headlightsOn;
                light4.shining = headlightsOn;
                if (headlightsOn)
                    headlightsMaterial.emissionEnergy = headlightsEnergy;
                else
                    headlightsMaterial.emissionEnergy = 0.0f;
            }
        }
        else
        {
            headlightsPressed = false;
        }
        
        // Rotate wheels
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
        float engineSoundBlend = lerp(1.0f, 1.25f, car.throttle);
        audio.setRelativePlaySpeed(engineVoice, engineSoundBlend);
        
        // Tire squeal sound
        float lateralSlip = car.lateralSlip;
        float longitudinalSlip = car.longitudinalSlip;
        float squealVolume = clamp(lateralSlip, 0.0f, 1.0f);
        if (car.brake) squealVolume = clamp((speedKMH - 10.0f) / 10.0f, 0.0f, 1.0f);
        audio.setVolume(squealVoice, sfxVolume * squealVolume * 0.8f);
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
        
        updateSky(t);
    }
    
    void updateSky(Time t)
    {
        // Control daytime
        if (eventManager.keyPressed[KEY_UP])
        {
            sun.pitch(-20.0f * t.delta);
            sun.turn(-10.0f * t.delta);
        }
        else if (eventManager.keyPressed[KEY_DOWN])
        {
            sun.pitch(20.0f * t.delta);
            sun.turn(10.0f * t.delta);
        }
        
        // Control turbidity
        if (eventManager.keyPressed[KEY_LEFT])
        {
            skyShader.turbidity += 0.5f * t.delta;
        }
        else if (eventManager.keyPressed[KEY_RIGHT])
        {
            skyShader.turbidity -= 0.5f * t.delta;
        }
        
        skyShader.turbidity = clamp(skyShader.turbidity, 0.0f, 1.0f);
        
        float dotProduct = dot(sun.directionAbsolute, Vector3f(0.0f, 1.0f, 0.0));
        float dayTimeFactor = (dotProduct + 1.0) * 0.5; // 0 = midnight, 1 = midday
        sun.color = lerp(Color4f(1.0f, 0.5f, 0.0f, 1.0f), Color4f(1.0f, 0.9f, 0.8f, 1.0f), normalizeInRange(dayTimeFactor, 0.5, 0.7));
        
        Color4f ambientClearSky = lerp(ambientClearSkySunset, ambientClearSkyDay, normalizeInRange(dayTimeFactor, 0.5, 0.8));
        ambientClearSky = lerp(ambientClearSkyNight, ambientClearSky, normalizeInRange(dayTimeFactor, 0.0, 0.5));
        
        Color4f ambientCloudySky = lerp(ambientCloudySkySunset, ambientCloudySkyDay, normalizeInRange(dayTimeFactor, 0.5, 0.8));
        ambientCloudySky = lerp(ambientCloudySkyNight, ambientCloudySky, normalizeInRange(dayTimeFactor, 0.0, 0.5));
        
        environment.ambientMap = null;
        environment.ambientColor = lerp(ambientClearSky, ambientCloudySky, skyShader.turbidity * skyShader.turbidity);
        
        environment.fogColor = lerp(fogSunset, fogDay, normalizeInRange(dayTimeFactor, 0.5, 0.8));
        environment.fogColor = lerp(fogNight, environment.fogColor, normalizeInRange(dayTimeFactor, 0.0, 0.5));
        
        sun.energy = lerp(0.0f, 15.0f, normalizeInRange(dayTimeFactor, 0.45f, 0.6f)) * (1.0f - skyShader.turbidity);
        skyShader.sunEnergy = sun.energy * 0.5f * (1.0f - skyShader.turbidity);
        
        float ambientEnergyDay = lerp(1.0f, 5.0f, skyShader.turbidity);
        float ambientEnergyNight = 1.0f;
        environment.ambientEnergy = lerp(ambientEnergyNight, ambientEnergyDay, normalizeInRange(dayTimeFactor, 0.3f, 1.0f));
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
    
    VehicleDemoGame game = New!VehicleDemoGame(1280, 720, false, "Chillwave Drive", args);
    game.run();
    Delete(game);
}
