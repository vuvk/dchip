/*
 * Copyright (c) 2007-2013 Scott Lembcke and Howling Moon Software
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
module demo.ChipmunkDemo;

import core.memory;

import core.stdc.stdio;
import core.stdc.stdlib;

import std.exception;
import std.stdio;
import std.string;

alias stderr = std.stdio.stderr;

import derelict.opengl3.gl;
import derelict.opengl3.types;
import derelict.sdl2.sdl;
import derelict.sdl2.types;

import demo.dchip;

import demo.Bench;
import demo.ChipmunkDebugDraw;
import demo.ChipmunkDemoTextSupport;
import demo.glu;
import demo.types;

import demo.LogoSmash;
import demo.PyramidStack;
import demo.Plink;
import demo.Tumble;
import demo.PyramidTopple;
import demo.Planet;
import demo.Springies;
import demo.Pump;
import demo.TheoJansen;
import demo.Query;
import demo.OneWay;
import demo.Joints;
import demo.Tank;
import demo.Chains;
import demo.Crane;
import demo.ContactGraph;
import demo.Buoyancy;
import demo.Player;
import demo.Slice;
import demo.Convex;
import demo.Unicycle;
import demo.Sticky;
import demo.Shatter;
import demo.RollBall;

ChipmunkDemo[] demo_list;
shared static this()
{
    demo_list = [
        LogoSmash,
        PyramidStack,
        Plink,
        BouncyHexagons,
        Tumble,
        PyramidTopple,
        Planet,
        Springies,
        Pump,
        TheoJansen,
        Query,
        OneWay,
        Joints,
        Tank,
        Chains,
        Crane,
        ContactGraph,
        Buoyancy,
        Player,
        Slice,
        Convex,
        Unicycle,
        Sticky,
        Shatter,
        RollBall,
    ];
}

alias ChipmunkDemoInitFunc = cpSpace* function();
alias ChipmunkDemoUpdateFunc = void function(cpSpace* space, double dt);
alias ChipmunkDemoDrawFunc = void function(cpSpace* space);
alias ChipmunkDemoDestroyFunc = void function(cpSpace* space);

__gshared SDL_Window* 	sdlWindow;
__gshared SDL_GLContext glContext;
__gshared SDL_Event 	sdlEvent;

struct ChipmunkDemo
{
    string name;
    double timestep = 0;

    ChipmunkDemoInitFunc initFunc;
    ChipmunkDemoUpdateFunc updateFunc;
    ChipmunkDemoDrawFunc drawFunc;

    ChipmunkDemoDestroyFunc destroyFunc;
}

cpFloat frand()
{
    return cast(cpFloat)rand() / cast(cpFloat)RAND_MAX;
}

cpVect frand_unit_circle()
{
    cpVect v = cpv(frand() * 2.0f - 1.0f, frand() * 2.0f - 1.0f);
    return (cpvlengthsq(v) < 1.0f ? v : frand_unit_circle());
}

enum GRABABLE_MASK_BIT = (1 << 31);
enum NOT_GRABABLE_MASK = (~GRABABLE_MASK_BIT);

void ChipmunkDemoDefaultDrawImpl(cpSpace* space);
void ChipmunkDemoFreeSpaceChildren(cpSpace* space);

ChipmunkDemo* demos;
size_t demo_count = 0;
size_t demo_index = 0;

cpBool paused = cpFalse;
cpBool step   = cpFalse;

cpSpace* space;

double Accumulator = 0.0;
ulong LastTime = 0;
int ChipmunkDemoTicks = 0;
double ChipmunkDemoTime = 0;

cpVect ChipmunkDemoMouse;
cpBool ChipmunkDemoRightClick = cpFalse;
cpBool ChipmunkDemoRightDown  = cpFalse;
cpVect ChipmunkDemoKeyboard;

cpBody* mouse_body        = null;
cpConstraint* mouse_joint = null;

char[] ChipmunkDemoMessageString;

cpVect  translate = { 0, 0 };
cpFloat scale = 1.0;

void ShapeFreeWrap(cpSpace* space, cpShape* shape, void* unused)
{
    cpSpaceRemoveShape(space, shape);
    cpShapeFree(shape);
}

void PostShapeFree(cpShape* shape, cpSpace* space)
{
    cpSpaceAddPostStepCallback(space, safeCast!cpPostStepFunc(&ShapeFreeWrap), shape, null);
}

void ConstraintFreeWrap(cpSpace* space, cpConstraint* constraint, void* unused)
{
    cpSpaceRemoveConstraint(space, constraint);
    cpConstraintFree(constraint);
}

void PostConstraintFree(cpConstraint* constraint, cpSpace* space)
{
    cpSpaceAddPostStepCallback(space, safeCast!cpPostStepFunc(&ConstraintFreeWrap), constraint, null);
}

void BodyFreeWrap(cpSpace* space, cpBody* body_, void* unused)
{
    cpSpaceRemoveBody(space, body_);
    cpBodyFree(body_);
}

void PostBodyFree(cpBody* body_, cpSpace* space)
{
    cpSpaceAddPostStepCallback(space, safeCast!cpPostStepFunc(&BodyFreeWrap), body_, null);
}

// Safe and future proof way to remove and free all objects that have been added to the space.
void ChipmunkDemoFreeSpaceChildren(cpSpace* space)
{
    // Must remove these BEFORE freeing the body_ or you will access dangling pointers.
    cpSpaceEachShape(space, safeCast!cpSpaceShapeIteratorFunc(&PostShapeFree), space);
    cpSpaceEachConstraint(space, safeCast!cpSpaceConstraintIteratorFunc(&PostConstraintFree), space);

    cpSpaceEachBody(space, safeCast!cpSpaceBodyIteratorFunc(&PostBodyFree), space);
}

void ChipmunkDemoDefaultDrawImpl(cpSpace* space)
{
    ChipmunkDebugDrawShapes(space);
    ChipmunkDebugDrawConstraints(space);
    ChipmunkDebugDrawCollisionPoints(space);
}

void DrawInstructions()
{
    ChipmunkDemoTextDrawString(cpv(-300, 220),
                               "Controls:\n" ~
                               "A - * Switch demos. (return restarts)\n" ~
                               "Use the mouse to grab objects.\n"
                               );
}

int max_arbiters    = 0;
int max_points      = 0;
int max_constraints = 0;

void DrawInfo()
{
    int arbiters = space.arbiters.num;
    int points   = 0;

    for (int i = 0; i < arbiters; i++)
        points += (cast(cpArbiter*)(space.arbiters.arr[i])).numContacts;

    int constraints = (space.constraints.num + points) * space.iterations;

    max_arbiters    = arbiters > max_arbiters ? arbiters : max_arbiters;
    max_points      = points > max_points ? points : max_points;
    max_constraints = constraints > max_constraints ? constraints : max_constraints;

    char[1024] buffer = 0;
    string format =
        "Arbiters: %d (%d) - " ~
        "Contact Points: %d (%d)\n" ~
        "Other Constraints: %d, Iterations: %d\n" ~
        "Constraints x Iterations: %d (%d)\n" ~
        "Time:% 5.2fs, KE:% 5.2e\0";

    cpArray* bodies = space.bodies;
    cpFloat  ke     = 0.0f;

    for (int i = 0; i < bodies.num; i++)
    {
        cpBody* body_ = cast(cpBody*)bodies.arr[i];

        if (body_.m == INFINITY || body_.i == INFINITY)
            continue;

        ke += body_.m * cpvdot(body_.v, body_.v) + body_.i * body_.w * body_.w;
    }

    sprintf(buffer.ptr, format.ptr,
            arbiters, max_arbiters,
            points, max_points,
            space.constraints.num, space.iterations,
            constraints, max_constraints,
            ChipmunkDemoTime, (ke < 1e-10f ? 0.0f : ke)
            );

    ChipmunkDemoTextDrawString(cpv(0, 220), buffer);
}

char[1024 * 8]  PrintStringBuffer = 0;
size_t PrintStringCursor;

void ChipmunkDemoPrintString(Args...)(string fmt, Args args)
{
    ChipmunkDemoMessageString = PrintStringBuffer[];
    PrintStringCursor += sformat(PrintStringBuffer[PrintStringCursor .. $], fmt, args).length;
}

void Tick(double dt)
{
    if (!paused || step)
    {
        PrintStringBuffer[0] = 0;
        PrintStringCursor = 0;

        // Completely reset the renderer only at the beginning of a tick.
        // That way it can always display at least the last ticks' debug drawing.
        ChipmunkDebugDrawClearRenderer();
        ChipmunkDemoTextClearRenderer();

        cpVect new_point = cpvlerp(mouse_body.p, ChipmunkDemoMouse, 0.25f);
        mouse_body.v = cpvmult(cpvsub(new_point, mouse_body.p), 60.0f);
        mouse_body.p = new_point;

        demos[demo_index].updateFunc(space, dt);

        ChipmunkDemoTicks++;
        ChipmunkDemoTime += dt;

        step = cpFalse;
        ChipmunkDemoRightDown = cpFalse;

        ChipmunkDemoTextDrawString(cpv(-300, -200), ChipmunkDemoMessageString);
    }
}

void Update()
{
   	ulong now  = SDL_GetPerformanceCounter();
	double dt = (now - LastTime) / cast(double)SDL_GetPerformanceFrequency();
		
    if (dt > 0.2)
        dt = 0.2;

    double fixed_dt = demos[demo_index].timestep;

    for (Accumulator += dt; Accumulator > fixed_dt; Accumulator -= fixed_dt)
    {
        Tick(fixed_dt);
    }

    LastTime = now;
}

void Display()
{	
	/*
		why you not drawing?!
	*/
	
	Reshape (640, 480);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(cast(GLfloat)translate.x, cast(GLfloat)translate.y, 0.0f);
    glScalef(cast(GLfloat)scale, cast(GLfloat)scale, 1.0f);

    Update();

    ChipmunkDebugDrawPushRenderer();
    demos[demo_index].drawFunc(space);

    // Highlight the shape under the mouse because it looks neat.
    cpShape* nearest = cpSpaceNearestPointQueryNearest(space, ChipmunkDemoMouse, 0.0f, CP_ALL_LAYERS, CP_NO_GROUP, null);

    if (nearest)
        ChipmunkDebugDrawShape(nearest, RGBAColor(1.0f, 0.0f, 0.0f, 1.0f), LAColor(0.0f, 0.0f));

    // Draw the renderer contents and reset it back to the last tick's state.
    ChipmunkDebugDrawFlushRenderer();
    ChipmunkDebugDrawPopRenderer();

    ChipmunkDemoTextPushRenderer();

    // Now render all the UI text.
    DrawInstructions();
    DrawInfo();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
        // Draw the text at fixed positions,
        // but save the drawing matrix for the mouse picking
        glLoadIdentity();

        ChipmunkDemoTextFlushRenderer();
        ChipmunkDemoTextPopRenderer();
    }
    glPopMatrix();
	
	SDL_GL_SwapWindow (sdlWindow);
    glClear(GL_COLOR_BUFFER_BIT);
}

void Reshape (int width, int height)
{
    glViewport(0, 0, width, height);

    float scale = cast(float)cpfmin(width / 640.0, height / 480.0);
    float hw    = width * (0.5f / scale);
    float hh    = height * (0.5f / scale);

    ChipmunkDebugDrawPointLineScale = scale;
    glLineWidth(cast(GLfloat)scale);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-hw, hw, -hh, hh);
}

char[] DemoTitle(size_t index)
{
    static char[1024] title;
    title[] = 0;
    sformat(title, "Demo(%s): %s", cast(char)('a' + index), demos[demo_index].name);
    return title;
}

void RunDemo(size_t index)
{
    srand(45073);

    demo_index = index;

    ChipmunkDemoTicks = 0;
    ChipmunkDemoTime  = 0.0;
    Accumulator       = 0.0;
	LastTime = SDL_GetPerformanceCounter();

    mouse_joint = null;
    ChipmunkDemoMessageString = "\0".dup;
    max_arbiters    = 0;
    max_points      = 0;
    max_constraints = 0;
    space = demos[demo_index].initFunc();

    enforce(sdlWindow !is null);
    
	SDL_SetWindowTitle (sdlWindow, DemoTitle(index).toStringz);
}

void Keyboard (int key, int scancode, int state, int modifier)
{
	/*
		OLD keyboard input from version with glfw
	*/

    /*if (state != GLFW_REPEAT)  // we ignore repeat
    switch (key)
    {
        case GLFW_KEY_UP:
            ChipmunkDemoKeyboard.y += (state == GLFW_PRESS ?  1.0 : -1.0);
            break;

        case GLFW_KEY_DOWN:
            ChipmunkDemoKeyboard.y += (state == GLFW_PRESS ? -1.0 :  1.0);
            break;

        case GLFW_KEY_RIGHT:
            ChipmunkDemoKeyboard.x += (state == GLFW_PRESS ?  1.0 : -1.0);
            break;

        case GLFW_KEY_LEFT:
            ChipmunkDemoKeyboard.x += (state == GLFW_PRESS ? -1.0 :  1.0);
            break;

        default:
            break;
    }

    if (key == GLFW_KEY_ESCAPE && (state == GLFW_PRESS || state == GLFW_REPEAT))
        glfwSetWindowShouldClose(window, true);

    // We ignore release for these next keys.
    if (state == GLFW_RELEASE)
        return;

    int index = key - GLFW_KEY_A;

    if (0 <= index && index < demo_count)
    {
        demos[demo_index].destroyFunc(space);
        RunDemo(index);
    }
    else if (key == ' ')
    {
        demos[demo_index].destroyFunc(space);
        RunDemo(demo_index);
    }
    else if (key == '`')
    {
        paused = !paused;
    }
    else if (key == '1')
    {
        step = cpTrue;
    }
    else if (key == '\\')
    {
        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_POINT_SMOOTH);
    }

    GLfloat translate_increment = 50.0f / cast(GLfloat)scale;
    GLfloat scale_increment     = 1.2f;

    if (key == '5')
    {
        translate.x = 0.0f;
        translate.y = 0.0f;
        scale       = 1.0f;
    }
    else if (key == '4')
    {
        translate.x += translate_increment;
    }
    else if (key == '6')
    {
        translate.x -= translate_increment;
    }
    else if (key == '2')
    {
        translate.y += translate_increment;
    }
    else if (key == '8')
    {
        translate.y -= translate_increment;
    }
    else if (key == '7')
    {
        scale /= scale_increment;
    }
    else if (key == '9')
    {
        scale *= scale_increment;
    }*/
}

cpVect MouseToSpace(double x, double y)
{
    GLdouble[16] model = 0;
    glGetDoublev(GL_MODELVIEW_MATRIX, model.ptr);

    GLdouble[16] proj = 0;
    glGetDoublev(GL_PROJECTION_MATRIX, proj.ptr);

    GLint[4] view = 0;
    glGetIntegerv(GL_VIEWPORT, view.ptr);

    int ww, wh;
	SDL_GetWindowSize (sdlWindow, &ww, &wh);

    GLdouble mx = 0, my = 0, mz = 0;
    gluUnProject(x, wh - y, 0.0f, model, proj, view, &mx, &my, &mz);

    return cpv(mx, my);
}

void Mouse(double x, double y)
{
    ChipmunkDemoMouse = MouseToSpace(x, y);
}

void Click(int button, int state, int mods)
{
	/*
		OLD mouse input from version with glfw
	*/
	
    /*if (button == GLFW_MOUSE_BUTTON_1)
    {
        if (state == GLFW_PRESS)
        {
            cpShape* shape = cpSpacePointQueryFirst(space, ChipmunkDemoMouse, GRABABLE_MASK_BIT, CP_NO_GROUP);

            if (shape && !cpBodyIsStatic(shape.body_))
            {
                cpBody* body_ = shape.body_;
                mouse_joint = cpPivotJointNew2(mouse_body, body_, cpvzero, cpBodyWorld2Local(body_, ChipmunkDemoMouse));
                mouse_joint.maxForce  = 50000.0f;
                mouse_joint.errorBias = cpfpow(1.0f - 0.15f, 60.0f);
                cpSpaceAddConstraint(space, mouse_joint);
            }
        }
        else if (mouse_joint)
        {
            cpSpaceRemoveConstraint(space, mouse_joint);
            cpConstraintFree(mouse_joint);
            mouse_joint = null;
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_2)
    {
        ChipmunkDemoRightDown = ChipmunkDemoRightClick = (state == GLFW_PRESS);
    }*/
}

void SetupGL()
{
    ChipmunkDebugDrawInit();
    ChipmunkDemoTextInit();

    glClearColor(52.0f / 255.0f, 62.0f / 255.0f, 72.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);

    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glHint(GL_POINT_SMOOTH_HINT, GL_DONT_CARE);

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
}

int main(string[] args)
{
    GC.disable();
    scope (exit)
        GC.enable();

    // Segment/segment collisions need to be explicitly enabled currently.
    // This will becoume enabled by default in future versions of Chipmunk.
    cpEnableSegmentToSegmentCollisions();
    demos      = demo_list.ptr;
    demo_count = demo_list.length;
    int trial = 0;

    foreach (arg; args[1 .. $])
    {
        if (arg == "-bench")
        {
            demos      = cast(ChipmunkDemo*)bench_list.ptr;
            demo_count = bench_count;
        }
    }

    mouse_body = cpBodyNew(INFINITY, INFINITY);

	// initialize SDL2
	DerelictSDL2.load();
		
	SDL_Init (SDL_INIT_EVERYTHING);

    int width = 640;
    int height = 480;
	
	sdlWindow = SDL_CreateWindow ("Hello World", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);
	
    // Make the window's context current
	DerelictGL.load();
	glContext = SDL_GL_CreateContext (sdlWindow);

    // Load all OpenGL functions
	GLVersion glVersion = DerelictGL.reload();

    // Support only GL 3x+
	writefln("OpenGL version available - %d", glVersion);
	enforce(glVersion >= 30);
		
    SetupGL();

    RunDemo(demo_index);

    /* Loop until the user closes the window */
	cpBool quit = false;
	while (!quit)
    {
        /* Poll for and process events */
        SDL_PumpEvents();
		
        while (SDL_PollEvent (&sdlEvent))
		{
			switch (sdlEvent.type)
			{
				case SDL_QUIT :
				{
					quit = true;
					continue;	
				}	
				
				/*
					see Keyboard func for adding input manipulations, friends
				*/
				case SDL_KEYDOWN :
				{
					int key = sdlEvent.key.keysym.sym;
					int index = key - SDLK_a;

    				if (0 <= index && index < demo_count)
    				{
        				demos[demo_index].destroyFunc(space);
        				RunDemo(index);
   					}
    				else					
						switch (key)
						{
							case SDLK_ESCAPE:
							{
								quit = true;
								continue;	
							}					
						
						
							default : break;
						}
				
					break;	
				}
				
				default : break;
			}	
		}

        /* Render here */
        Display();		
    }
	
	SDL_GL_DeleteContext (glContext);
	SDL_DestroyWindow (sdlWindow);
	SDL_Quit();

    return 0;
}
