module menu;

import std.stdio;
import dagon;
import dagon.ext.imgui;
import game;

class MainMenuScene: Scene
{
    ChillwaveDriveGame game;
    Camera camera;
    Entity bg;

    this(ChillwaveDriveGame game)
    {
        super(game);
        this.game = game;
    }

    override void beforeLoad()
    {
    }
    
    override void onLoad(Time t, float progress)
    {
    }
    
    override void afterLoad()
    {
        camera = addCamera();
        
        auto hudShader = New!HUDShader(assetManager);
        
        bg = addEntityHUD();
        bg.drawable = New!ShapeQuad(assetManager);
        resizeBg(eventManager.windowWidth, eventManager.windowHeight);
        auto bgMaterial = addMaterial();
        bgMaterial.shader = hudShader;
        bgMaterial.baseColorTexture = game.mainMenuBackground.texture;
        bgMaterial.depthWrite = false;
        bgMaterial.useCulling = false;
        bg.material = bgMaterial;
        
        onReset();
    }
    
    override void onReset()
    {
        game.hudRenderer.passHUD.clear = true;
        game.renderer.activeCamera = camera;
        game.imgui.active = true;
        
        //playMusic("assets/music/dust.mp3");
    }
    
    override void onUpdate(Time t)
    {
    }
    
    override void onResize(int width, int height)
    {
        if (bg)
            resizeBg(width, height);
    }
    
    void resizeBg(int width, int height)
    {
        float aspectRatio = cast(float)width / cast(float)height;
        float imageAspectRatio = 16.0f / 9.0f;
        if (aspectRatio > imageAspectRatio)
            bg.scaling = Vector3f(width, width / imageAspectRatio, 1.0f);
        else
            bg.scaling = Vector3f(height * imageAspectRatio, height, 1.0f);
        
        // Centering
        bg.position = Vector3f(cast(float)width * 0.5 - bg.scaling.x * 0.5, cast(float)height * 0.5 - bg.scaling.y * 0.5, 0.0f);
    }
    
    override void onKeyDown(int key)
    {
        if (key == KEY_F4)
        {
            game.updateRenderers(Time(0.0, 0.0));
            game.takeScreenshot("screenshots/screenshot");
        }
    }
    
    override void onKeyUp(int key) { }
    override void onMouseButtonDown(int button) { }
    override void onMouseButtonUp(int button) { }
}
