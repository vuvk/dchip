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
 
// TODO: check working, delete comments 
 
module dchip.cpArray;

import dchip.chipmunk;
import dchip.chipmunk_private;
import dchip.chipmunk_types;
import dchip.chipmunk_structs;

cpArray* cpArrayNew(int size)
{
    cpArray* arr = cast(cpArray*)cpcalloc(1, cpArray.sizeof);

    arr.num = 0;
    arr.max = (size ? size : 4);
	// TODO : DELETE
    //arr.arr = cast(void**)cpcalloc(arr.max, (void**).sizeof);
	arr.arr = cast(void**)cpcalloc(arr.max, (void*).sizeof);

    return arr;
}

void cpArrayFree(cpArray* arr)
{
    if (arr)
    {
        cpfree(arr.arr);
        arr.arr = null;

        cpfree(arr);
    }
}

void cpArrayPush(cpArray* arr, void* obj)
{
	// TODO : DELETE
    /*if (arr.num == arr.max)
    {
        arr.max *= 2;
        arr.arr  = cast(void**)cprealloc(arr.arr, arr.max * (void**).sizeof);
    }*/
	if(arr.num == arr.max)
	{
		arr.max = 3 * (arr.max + 1) / 2;
		arr.arr = cast(void**)cprealloc(arr.arr, arr.max * (void*).sizeof);
	}	

    arr.arr[arr.num] = obj;
    arr.num++;
}

void* cpArrayPop(cpArray* arr)
{
    arr.num--;

    void* value = arr.arr[arr.num];
    arr.arr[arr.num] = null;

    return value;
}

void cpArrayDeleteObj(cpArray* arr, void* obj)
{
    for (int i = 0; i < arr.num; i++)
    {
        if (arr.arr[i] == obj)
        {
            arr.num--;

            arr.arr[i]       = arr.arr[arr.num];
            arr.arr[arr.num] = null;

            return;
        }
    }
}

alias extern(C) void function(void*) FreeFunc;

void cpArrayFreeEach(cpArray* arr, FreeFunc freeFunc)
{
    for (int i = 0; i < arr.num; i++)
        freeFunc(arr.arr[i]);
}

cpBool cpArrayContains(cpArray* arr, void* ptr)
{
    for (int i = 0; i < arr.num; i++)
        if (arr.arr[i] == ptr)
            return cpTrue;

    return cpFalse;
}

