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
module main;

import std.stdio;
import std.conv;
import dagon;
import dagon.ext.newton;
import soloud;
import scene;

class VehicleDemoGame: Game
{
    Soloud audio;
    
    this(uint w, uint h, bool fullscreen, string title, string[] args)
    {
        super(w, h, fullscreen, title, args);
        audio = Soloud.create();
        audio.init(Soloud.CLIP_ROUNDOFF | Soloud.LEFT_HANDED_3D);
        currentScene = New!VehicleScene(this, audio);
    }
}

void loadNewtonLibrary()
{
    NewtonSupport sup = loadNewton();
    debug
    {
        import loader = bindbc.loader.sharedlib;
        foreach(info; loader.errors)
        {
            writeln(info.error.to!string, " ", info.message.to!string);
        }
    }
}

void main(string[] args)
{
    loadNewtonLibrary();
    loadSoloud();
    VehicleDemoGame game = New!VehicleDemoGame(1280, 720, false, "Dagon vehicle demo", args);
    game.run();
    Delete(game);
}
