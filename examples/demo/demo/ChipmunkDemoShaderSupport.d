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
module demo.ChipmunkDemoShaderSupport;

import core.stdc.stdlib;

import std.conv;
import std.stdio;
import std.string;

import demo.dchip;

import derelict.opengl3.gl;
import derelict.opengl3.types;

string SET_ATTRIBUTE(string program, string type, string name, string gltype)
{
    return q{
        SetAttribute(program, "%1$s", %2$s.%1$s.sizeof / GLfloat.sizeof, %3$s, %2$s.sizeof, cast(GLvoid *)%2$s.%1$s.offsetof);
    }.format(name, type, gltype);
}

/// Converts an OpenGL errorenum to a string
string toString(GLenum error)
{
    switch (error)
    {
        case GL_INVALID_ENUM:
            return "An unacceptable value is specified for an enumerated argument.";

        case GL_INVALID_VALUE:
            return "A numeric argument is out of range.";

        case GL_INVALID_OPERATION:
            return "The specified operation is not allowed in the current state.";

        case GL_INVALID_FRAMEBUFFER_OPERATION:
            return "The framebuffer object is not complete.";

        case GL_OUT_OF_MEMORY:
            return "There is not enough memory left to execute the command. WARNING: GL operation is undefined.";

        case GL_STACK_UNDERFLOW:
            return "An attempt has been made to perform an operation that would cause an internal stack to underflow.";

        case GL_STACK_OVERFLOW:
            return "An attempt has been made to perform an operation that would cause an internal stack to overflow.";

        default:
            assert(0, format("Unhandled GLenum error state: '%s'", error));
    }
}

void CheckGLErrors()
{
    for (GLenum err = glGetError(); err; err = glGetError())
    {
        if (err)
        {
            stderr.writefln("Error: - %s", err.toString());
            stderr.writefln("GLError(%s:%d) 0x%04X\n", __FILE__, __LINE__, err);
            assert(0);
        }
    }
}

alias PFNGLGETSHADERIVPROC = extern (C) void function( GLuint,GLenum,GLint* ) nothrow @nogc;
alias PFNGLGETSHADERINFOLOGPROC = extern (C) void function( GLuint,GLsizei,GLsizei*,GLchar* ) nothrow @nogc;

//typedef GLAPIENTRY void (*GETIV)(GLuint shader, GLenum pname, GLint *params);
//typedef GLAPIENTRY void (*GETINFOLOG)(GLuint shader, GLsizei maxLength, GLsizei *length, GLchar *infoLog);

static cpBool CheckError(GLint obj, GLenum status, PFNGLGETSHADERIVPROC getiv, PFNGLGETSHADERINFOLOGPROC getInfoLog)
{
    GLint success;
    getiv(obj, status, &success);

    if (!success)
    {
        GLint length;
        getiv(obj, GL_INFO_LOG_LENGTH, &length);

        char* log = cast(char*)alloca(length);
        getInfoLog(obj, length, null, log);

        stderr.writefln("Shader compile error for 0x%04X: %s\n", status, log.to!string);
        return cpFalse;
    }
    else
    {
        return cpTrue;
    }
}

GLint CompileShader(GLenum type, string source)
{
    GLint shader = glCreateShader(type);

    auto ssp = source.ptr;
    int ssl = cast(int)(source.length);
    glShaderSource(shader, 1, &ssp, &ssl);
    glCompileShader(shader);

    // TODO return placeholder shader instead?
    cpAssertHard(CheckError(shader, GL_COMPILE_STATUS, glGetShaderiv, glGetShaderInfoLog), "Error compiling shader");

    return shader;
}

GLint LinkProgram(GLint vshader, GLint fshader)
{
    GLint program = glCreateProgram();

    glAttachShader(program, vshader);
    glAttachShader(program, fshader);
    glLinkProgram(program);

    // todo return placeholder program instead?
    cpAssertHard(CheckError(program, GL_LINK_STATUS, glGetProgramiv, glGetProgramInfoLog), "Error linking shader program");

    return program;
}

cpBool ValidateProgram(GLint program)
{
    // TODO
    return cpTrue;
}

void SetAttribute(GLuint program, string name, GLint size, GLenum gltype, GLsizei stride, GLvoid* offset)
{
    GLint index = glGetAttribLocation(program, name.toStringz);
    glEnableVertexAttribArray(index);
    glVertexAttribPointer(index, size, gltype, GL_FALSE, stride, offset);
}
