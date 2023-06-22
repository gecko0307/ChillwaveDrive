/*
Copyright (c) 2019-2022 Timur Gafarov

Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
module scene;

import std.stdio;
import std.conv;
import std.math;

import dagon;
import dagon.ext.ftfont;
import dagon.ext.newton;
import dagon.ext.stbi;

import vehicle;
import view;

class VehicleScene: Scene
{
    Game game;
    
    FontAsset aFontDroidSans14;
    
    TextureAsset aEnvmap;
    TextureAsset aBRDF;
    
    GLTFAsset aCar;
    GLTFAsset aWheel;
    GLTFAsset aTrack;
    TextureAsset aGroundAlbedo;
    TextureAsset aGroundNormal;
    TextureAsset aRoadAlbedo;

    Camera camera;
    VehicleViewComponent vehicleView;
    Light sun;
    Color4f sunColor = Color4f(0.9f, 0.7f, 0.6f, 1.0f);
    float sunPitch = -25.0f;
    float sunTurn = 65.0f;
    
    TextureAsset aTexColorTable;

    NewtonPhysicsWorld world;

    Entity eCharacter;
    NewtonCharacterComponent character;
    
    Entity eCar;
    Entity[4] eWheels;
    Vehicle vehicle;

    TextLine text;

    this(Game game)
    {
        super(game);
        this.game = game;
        stbiRegister(assetManager);
    }

    override void beforeLoad()
    {
        aFontDroidSans14 = this.addFontAsset("data/font/DroidSans.ttf", 14);
        aTrack = addGLTFAsset("data/track/track.gltf");
        aCar = addGLTFAsset("data/car/porsche_911.gltf");
        aWheel = addGLTFAsset("data/car/wheel.gltf");
        
        aEnvmap = addTextureAsset("data/envmap.dds");
        aBRDF = addTextureAsset("data/brdf.dds");
        
        aTexColorTable = addTextureAsset("data/lut.png");
    }

    override void afterLoad()
    {
        world = New!NewtonPhysicsWorld(eventManager, assetManager);
        world.loadPlugins("./");

        camera = addCamera();
        camera.fov = 50.0f;
        game.renderer.activeCamera = camera;
        
        environment.backgroundColor = Color4f(0.4f, 0.3f, 0.5f, 1.0f);
        environment.ambientMap = aEnvmap.texture;
        aEnvmap.texture.enableRepeat = false;
        environment.ambientEnergy = 1.0f;
        environment.ambientBRDF = aBRDF.texture;
        aBRDF.texture.useMipmapFiltering = false;
        aBRDF.texture.enableRepeat = false;
        environment.fogColor = environment.backgroundColor;
        environment.fogStart = 0.0f;
        environment.fogEnd = 1000.0f;
        
        game.deferredRenderer.ssaoEnabled = true;
        game.deferredRenderer.ssaoRadius = 1.0f;
        game.deferredRenderer.ssaoPower = 12.0f;
        game.postProcessingRenderer.fxaaEnabled = true;
        game.postProcessingRenderer.depthOfFieldEnabled = true;
        game.postProcessingRenderer.fStop = 2.0f;
        game.postProcessingRenderer.motionBlurEnabled = true;
        game.postProcessingRenderer.motionBlurFramerate = 45;
        game.postProcessingRenderer.glowEnabled = true;
        game.postProcessingRenderer.glowThreshold = 1.0f;
        game.postProcessingRenderer.glowIntensity = 0.5f;
        game.postProcessingRenderer.glowRadius = 7;
        game.postProcessingRenderer.lensDistortionEnabled = true;
        game.postProcessingRenderer.tonemapper = Tonemapper.Unreal;
        game.postProcessingRenderer.exposure = 0.75f;
        game.postProcessingRenderer.lutEnabled = true;
        game.postProcessingRenderer.colorLookupTable = aTexColorTable.texture;
        
        sun = addLight(LightType.Sun);
        sun.position.y = 50.0f;
        sun.shadowEnabled = true;
        auto sm = cast(CascadedShadowMap)sun.shadowMap();
        sm.projectionSize[0] = 5;
        sm.projectionSize[1] = 25;
        sm.projectionSize[2] = 40;
        sun.energy = 20.0f;
        sun.scatteringEnabled = true;
        sun.scattering = 0.35f;
        sun.mediumDensity = 0.075f;
        sun.scatteringUseShadow = true;
        sun.scatteringMaxRandomStepOffset = 1.0f;
        sun.color = sunColor;
        sun.setRotation(70.0f, -135.0f, 0.0f);

        auto eSky = addEntity();
        auto psync = New!PositionSync(eventManager, eSky, camera);
        eSky.drawable = New!ShapeBox(Vector3f(1.0f, 1.0f, 1.0f), assetManager);
        eSky.scaling = Vector3f(100.0f, 100.0f, 100.0f);
        eSky.layer = EntityLayer.Background;
        eSky.material = New!Material(assetManager);
        eSky.material.depthWrite = false;
        eSky.material.useCulling = false;
        eSky.material.baseColorTexture = aEnvmap.texture;
        
        // Track
        aTrack.markTransparentEntities();
        useEntity(aTrack.rootEntity);
        foreach(node; aTrack.nodes)
        {
            useEntity(node.entity);
        }
        
        auto eTrack = addEntity();
        auto trackShape = New!NewtonMeshShape(aTrack, world);
        eTrack.makeStaticBody(world, trackShape);
        
        // Vehicle
        aCar.markTransparentEntities();
        eCar = aCar.rootEntity;
        useEntity(aCar.rootEntity);
        foreach(node; aCar.nodes)
        {
            useEntity(node.entity);
        }
        
        foreach(mat; aCar.materials)
        {
            mat.sun = sun;
        }
        
        eCar.position = Vector3f(-125, 2, -142);
        eCar.turn(90.0f);
        
        Vector3f chassisBottomSize = Vector3f(1.65f, 0.6f, 4.3f);
        Vector3f chassisBottomPos = Vector3f(0.0f, 0.55f, 0.0f); //0.55
        
        Vector3f chassisTopSize = Vector3f(1.3f, 0.6f, 1.6f);
        Vector3f chassisTopPos = Vector3f(0.0f, 1.0f, -0.3f); //1.0
        
        /*
        auto mTrans = addMaterial();
        mTrans.blendMode = Transparent;
        mTrans.opacity = 0.5f;
        auto eChassisBottom = addEntity(eCar);
        eChassisBottom.drawable = New!ShapeBox(chassisBottomSize * 0.5f, assetManager);
        eChassisBottom.material = mTrans;
        eChassisBottom.position = chassisBottomPos;
        auto eChassisTop = addEntity(eCar);
        eChassisTop.drawable = New!ShapeBox(chassisTopSize * 0.5f, assetManager);
        eChassisTop.material = mTrans;
        eChassisTop.position = chassisTopPos;
        */
        
        auto chassisBottom = New!NewtonBoxShape(chassisBottomSize, world);
        chassisBottom.setTransformation(translationMatrix(chassisBottomPos));
        auto chassisTop = New!NewtonBoxShape(chassisTopSize, world);
        chassisTop.setTransformation(translationMatrix(chassisTopPos));
        auto newtonChassisShape = New!NewtonCompoundShape(cast(NewtonCollisionShape[])[chassisBottom, chassisTop], world);
        vehicle = New!Vehicle(world, eCar, newtonChassisShape, 1600.0f, 1);
        vehicle.chassisBody.centerOfMass = Vector3f(0.0f, 0.55f, 0.0f);
        vehicle.maxTorque = 4000.0f;
        auto fw1 = vehicle.addWheel(Vector3f(-0.56f, 0.75f,  1.35f), 0.341f, -1.0f, true, true);
        auto fw2 = vehicle.addWheel(Vector3f( 0.56f, 0.75f,  1.35f), 0.341f,  1.0f, true, true);
        auto bw1 = vehicle.addWheel(Vector3f(-0.56f, 0.75f, -1.2f), 0.341f, -1.0f, false, false);
        auto bw2 = vehicle.addWheel(Vector3f( 0.56f, 0.75f, -1.2f), 0.341f,  1.0f, false, false);
        
        fw1.tyreOffset = Vector3f(-0.15f, 0, 0);
        fw2.tyreOffset = Vector3f( 0.15f, 0, 0);
        bw1.tyreOffset = Vector3f(-0.15f, 0, 0);
        bw2.tyreOffset = Vector3f( 0.15f, 0, 0);
        
        float grip = 1.2f; // 1.7f
        float frontLength = 0.5f;
        float rearLength = 0.5f;
        float stiffness = 200.0f;
        float damping = 20.0f;
        
        fw1.grip = grip;
        fw1.suspension.minLength = 0.4f;
        fw1.suspension.maxLength = frontLength;
        fw1.suspension.stiffness = stiffness;
        fw1.suspension.damping = damping;
        
        fw2.grip = grip;
        fw2.suspension.minLength = 0.4f;
        fw2.suspension.maxLength = frontLength;
        fw2.suspension.stiffness = stiffness;
        fw2.suspension.damping = damping;
        
        bw1.grip = grip;
        bw1.suspension.minLength = 0.4f;
        bw1.suspension.maxLength = rearLength;
        bw1.suspension.stiffness = stiffness;
        bw1.suspension.damping = damping;
        
        bw2.grip = grip;
        bw2.suspension.minLength = 0.4f;
        bw2.suspension.maxLength = rearLength;
        bw2.suspension.stiffness = stiffness;
        bw2.suspension.damping = damping;
        
        foreach(i, ref w; eWheels)
        {
            w = addEntity(eCar);
            auto group = New!DrawableGroup(assetManager);
            w.drawable = group;
            foreach(e; aWheel.rootEntity.children)
                group.add(e.drawable);
            w.position = vehicle.wheels[i].position;
            w.blurMask = 0.0f;
        }
        
        vehicleView = New!VehicleViewComponent(eventManager, camera, vehicle);

        text = New!TextLine(aFontDroidSans14.font, "0", assetManager);
        text.color = Color4f(1.0f, 1.0f, 1.0f, 0.7f);
        auto eText = addEntityHUD();
        eText.drawable = text;
        eText.position = Vector3f(16.0f, 30.0f, 0.0f);
        
        eventManager.showCursor(true);
        vehicleView.active = false;
    }

    override void onKeyDown(int key)
    {
        if (key == KEY_ESCAPE)
            application.exit();
        else if (key == KEY_RETURN)
            vehicle.reset();
    }
    
    override void onMouseButtonUp(int button)
    {
        vehicleView.active = !vehicleView.active;
        eventManager.showCursor(!vehicleView.active);
    }

    override void onUpdate(Time t)
    {
        if (inputManager.getButton("forward")) vehicle.accelerate(20);
        if (inputManager.getButton("back")) vehicle.accelerate(-20);
        if (inputManager.getButton("brake")) vehicle.stop();
        float axis = inputManager.getAxis("horizontal");
        vehicle.steer(-axis * 3);
        
        /*
        // Steering wheel controls (WIP)
        if (eventManager.joystickButtonPressed[0]) vehicle.accelerate(50);
        if (eventManager.joystickButtonPressed[1]) vehicle.accelerate(-50);
        float angle = eventManager.joystickAxis(SDL_CONTROLLER_AXIS_LEFTX);
        vehicle.setSteering(-angle);
        */
        
        foreach(i, ref w; eWheels)
        {
            auto wheel = vehicle.wheels[i];
            w.position = wheel.position;
            w.rotation = wheel.rotation;
        }
        
        world.update(t.delta);
        camera.fov = lerp(50.0f, 100.0f, vehicleView.boostFactor);
        game.postProcessingRenderer.radialBlurAmount = lerp(0.0f, 0.05f, vehicleView.boostFactor);
        game.postProcessingRenderer.lensDistortionScale = lerp(1.0f, 0.7f, vehicleView.boostFactor);
        game.postProcessingRenderer.lensDistortionDispersion = lerp(0.0f, 0.5f, vehicleView.boostFactor);
        updateText();
    }
    
    char[100] txt;
    void updateText()
    {
        uint fps = cast(int)(1.0 / eventManager.deltaTime);
        uint speed = cast(int)vehicle.speedKMH;
        uint n = sprintf(txt.ptr, "Speed: %u km/h", speed);
        string s = cast(string)txt[0..n];
        text.setText(s);
    }
}
