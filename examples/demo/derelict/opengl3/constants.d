/*

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
module derelict.opengl3.constants;

enum : ubyte {
    GL_FALSE                          = 0,
    GL_TRUE                           = 1,
}

enum : uint
{
    // OpenGL 1.1
    GL_DEPTH_BUFFER_BIT               = 0x00000100,
    GL_STENCIL_BUFFER_BIT             = 0x00000400,
    GL_COLOR_BUFFER_BIT               = 0x00004000,
    GL_POINTS                         = 0x0000,
    GL_LINES                          = 0x0001,
    GL_LINE_LOOP                      = 0x0002,
    GL_LINE_STRIP                     = 0x0003,
    GL_TRIANGLES                      = 0x0004,
    GL_TRIANGLE_STRIP                 = 0x0005,
    GL_TRIANGLE_FAN                   = 0x0006,
    GL_NEVER                          = 0x0200,
    GL_LESS                           = 0x0201,
    GL_EQUAL                          = 0x0202,
    GL_LEQUAL                         = 0x0203,
    GL_GREATER                        = 0x0204,
    GL_NOTEQUAL                       = 0x0205,
    GL_GEQUAL                         = 0x0206,
    GL_ALWAYS                         = 0x0207,
    GL_ZERO                           = 0,
    GL_ONE                            = 1,
    GL_SRC_COLOR                      = 0x0300,
    GL_ONE_MINUS_SRC_COLOR            = 0x0301,
    GL_SRC_ALPHA                      = 0x0302,
    GL_ONE_MINUS_SRC_ALPHA            = 0x0303,
    GL_DST_ALPHA                      = 0x0304,
    GL_ONE_MINUS_DST_ALPHA            = 0x0305,
    GL_DST_COLOR                      = 0x0306,
    GL_ONE_MINUS_DST_COLOR            = 0x0307,
    GL_SRC_ALPHA_SATURATE             = 0x0308,
    GL_NONE                           = 0,
    GL_FRONT_LEFT                     = 0x0400,
    GL_FRONT_RIGHT                    = 0x0401,
    GL_BACK_LEFT                      = 0x0402,
    GL_BACK_RIGHT                     = 0x0403,
    GL_FRONT                          = 0x0404,
    GL_BACK                           = 0x0405,
    GL_LEFT                           = 0x0406,
    GL_RIGHT                          = 0x0407,
    GL_FRONT_AND_BACK                 = 0x0408,
    GL_NO_ERROR                       = 0,
    GL_INVALID_ENUM                   = 0x0500,
    GL_INVALID_VALUE                  = 0x0501,
    GL_INVALID_OPERATION              = 0x0502,
    GL_OUT_OF_MEMORY                  = 0x0505,
    GL_CW                             = 0x0900,
    GL_CCW                            = 0x0901,
    GL_POINT_SIZE                     = 0x0B11,
    GL_POINT_SIZE_RANGE               = 0x0B12,
    GL_POINT_SIZE_GRANULARITY         = 0x0B13,
    GL_LINE_SMOOTH                    = 0x0B20,
    GL_LINE_WIDTH                     = 0x0B21,
    GL_LINE_WIDTH_RANGE               = 0x0B22,
    GL_LINE_WIDTH_GRANULARITY         = 0x0B23,
    GL_POLYGON_SMOOTH                 = 0x0B41,
    GL_CULL_FACE                      = 0x0B44,
    GL_CULL_FACE_MODE                 = 0x0B45,
    GL_FRONT_FACE                     = 0x0B46,
    GL_DEPTH_RANGE                    = 0x0B70,
    GL_DEPTH_TEST                     = 0x0B71,
    GL_DEPTH_WRITEMASK                = 0x0B72,
    GL_DEPTH_CLEAR_VALUE              = 0x0B73,
    GL_DEPTH_FUNC                     = 0x0B74,
    GL_STENCIL_TEST                   = 0x0B90,
    GL_STENCIL_CLEAR_VALUE            = 0x0B91,
    GL_STENCIL_FUNC                   = 0x0B92,
    GL_STENCIL_VALUE_MASK             = 0x0B93,
    GL_STENCIL_FAIL                   = 0x0B94,
    GL_STENCIL_PASS_DEPTH_FAIL        = 0x0B95,
    GL_STENCIL_PASS_DEPTH_PASS        = 0x0B96,
    GL_STENCIL_REF                    = 0x0B97,
    GL_STENCIL_WRITEMASK              = 0x0B98,
    GL_VIEWPORT                       = 0x0BA2,
    GL_DITHER                         = 0x0BD0,
    GL_BLEND_DST                      = 0x0BE0,
    GL_BLEND_SRC                      = 0x0BE1,
    GL_BLEND                          = 0x0BE2,
    GL_LOGIC_OP_MODE                  = 0x0BF0,
    GL_COLOR_LOGIC_OP                 = 0x0BF2,
    GL_DRAW_BUFFER                    = 0x0C01,
    GL_READ_BUFFER                    = 0x0C02,
    GL_SCISSOR_BOX                    = 0x0C10,
    GL_SCISSOR_TEST                   = 0x0C11,
    GL_COLOR_CLEAR_VALUE              = 0x0C22,
    GL_COLOR_WRITEMASK                = 0x0C23,
    GL_DOUBLEBUFFER                   = 0x0C32,
    GL_STEREO                         = 0x0C33,
    GL_LINE_SMOOTH_HINT               = 0x0C52,
    GL_POLYGON_SMOOTH_HINT            = 0x0C53,
    GL_UNPACK_SWAP_BYTES              = 0x0CF0,
    GL_UNPACK_LSB_FIRST               = 0x0CF1,
    GL_UNPACK_ROW_LENGTH              = 0x0CF2,
    GL_UNPACK_SKIP_ROWS               = 0x0CF3,
    GL_UNPACK_SKIP_PIXELS             = 0x0CF4,
    GL_UNPACK_ALIGNMENT               = 0x0CF5,
    GL_PACK_SWAP_BYTES                = 0x0D00,
    GL_PACK_LSB_FIRST                 = 0x0D01,
    GL_PACK_ROW_LENGTH                = 0x0D02,
    GL_PACK_SKIP_ROWS                 = 0x0D03,
    GL_PACK_SKIP_PIXELS               = 0x0D04,
    GL_PACK_ALIGNMENT                 = 0x0D05,
    GL_MAX_TEXTURE_SIZE               = 0x0D33,
    GL_MAX_VIEWPORT_DIMS              = 0x0D3A,
    GL_SUBPIXEL_BITS                  = 0x0D50,
    GL_TEXTURE_1D                     = 0x0DE0,
    GL_TEXTURE_2D                     = 0x0DE1,
    GL_POLYGON_OFFSET_UNITS           = 0x2A00,
    GL_POLYGON_OFFSET_POINT           = 0x2A01,
    GL_POLYGON_OFFSET_LINE            = 0x2A02,
    GL_POLYGON_OFFSET_FILL            = 0x8037,
    GL_POLYGON_OFFSET_FACTOR          = 0x8038,
    GL_TEXTURE_BINDING_1D             = 0x8068,
    GL_TEXTURE_BINDING_2D             = 0x8069,
    GL_TEXTURE_WIDTH                  = 0x1000,
    GL_TEXTURE_HEIGHT                 = 0x1001,
    GL_TEXTURE_INTERNAL_FORMAT        = 0x1003,
    GL_TEXTURE_BORDER_COLOR           = 0x1004,
    GL_TEXTURE_RED_SIZE               = 0x805C,
    GL_TEXTURE_GREEN_SIZE             = 0x805D,
    GL_TEXTURE_BLUE_SIZE              = 0x805E,
    GL_TEXTURE_ALPHA_SIZE             = 0x805F,
    GL_DONT_CARE                      = 0x1100,
    GL_FASTEST                        = 0x1101,
    GL_NICEST                         = 0x1102,
    GL_BYTE                           = 0x1400,
    GL_UNSIGNED_BYTE                  = 0x1401,
    GL_SHORT                          = 0x1402,
    GL_UNSIGNED_SHORT                 = 0x1403,
    GL_INT                            = 0x1404,
    GL_UNSIGNED_INT                   = 0x1405,
    GL_FLOAT                          = 0x1406,
    GL_DOUBLE                         = 0x140A,
    GL_CLEAR                          = 0x1500,
    GL_AND                            = 0x1501,
    GL_AND_REVERSE                    = 0x1502,
    GL_COPY                           = 0x1503,
    GL_AND_INVERTED                   = 0x1504,
    GL_NOOP                           = 0x1505,
    GL_XOR                            = 0x1506,
    GL_OR                             = 0x1507,
    GL_NOR                            = 0x1508,
    GL_EQUIV                          = 0x1509,
    GL_INVERT                         = 0x150A,
    GL_OR_REVERSE                     = 0x150B,
    GL_COPY_INVERTED                  = 0x150C,
    GL_OR_INVERTED                    = 0x150D,
    GL_NAND                           = 0x150E,
    GL_SET                            = 0x150F,
    GL_TEXTURE                        = 0x1702,
    GL_COLOR                          = 0x1800,
    GL_DEPTH                          = 0x1801,
    GL_STENCIL                        = 0x1802,
    GL_STENCIL_INDEX                  = 0x1901,
    GL_DEPTH_COMPONENT                = 0x1902,
    GL_RED                            = 0x1903,
    GL_GREEN                          = 0x1904,
    GL_BLUE                           = 0x1905,
    GL_ALPHA                          = 0x1906,
    GL_RGB                            = 0x1907,
    GL_RGBA                           = 0x1908,
    GL_POINT                          = 0x1B00,
    GL_LINE                           = 0x1B01,
    GL_FILL                           = 0x1B02,
    GL_KEEP                           = 0x1E00,
    GL_REPLACE                        = 0x1E01,
    GL_INCR                           = 0x1E02,
    GL_DECR                           = 0x1E03,
    GL_VENDOR                         = 0x1F00,
    GL_RENDERER                       = 0x1F01,
    GL_VERSION                        = 0x1F02,
    GL_EXTENSIONS                     = 0x1F03,
    GL_NEAREST                        = 0x2600,
    GL_LINEAR                         = 0x2601,
    GL_NEAREST_MIPMAP_NEAREST         = 0x2700,
    GL_LINEAR_MIPMAP_NEAREST          = 0x2701,
    GL_NEAREST_MIPMAP_LINEAR          = 0x2702,
    GL_LINEAR_MIPMAP_LINEAR           = 0x2703,
    GL_TEXTURE_MAG_FILTER             = 0x2800,
    GL_TEXTURE_MIN_FILTER             = 0x2801,
    GL_TEXTURE_WRAP_S                 = 0x2802,
    GL_TEXTURE_WRAP_T                 = 0x2803,
    GL_PROXY_TEXTURE_1D               = 0x8063,
    GL_PROXY_TEXTURE_2D               = 0x8064,
    GL_REPEAT                         = 0x2901,
    GL_R3_G3_B2                       = 0x2A10,
    GL_RGB4                           = 0x804F,
    GL_RGB5                           = 0x8050,
    GL_RGB8                           = 0x8051,
    GL_RGB10                          = 0x8052,
    GL_RGB12                          = 0x8053,
    GL_RGB16                          = 0x8054,
    GL_RGBA2                          = 0x8055,
    GL_RGBA4                          = 0x8056,
    GL_RGB5_A1                        = 0x8057,
    GL_RGBA8                          = 0x8058,
    GL_RGB10_A2                       = 0x8059,
    GL_RGBA12                         = 0x805A,
    GL_RGBA16                         = 0x805B,
    GL_VERTEX_ARRAY                   = 0x8074,

    // OpenGL 1.2
    GL_UNSIGNED_BYTE_3_3_2            = 0x8032,
    GL_UNSIGNED_SHORT_4_4_4_4         = 0x8033,
    GL_UNSIGNED_SHORT_5_5_5_1         = 0x8034,
    GL_UNSIGNED_INT_8_8_8_8           = 0x8035,
    GL_UNSIGNED_INT_10_10_10_2        = 0x8036,
    GL_TEXTURE_BINDING_3D             = 0x806A,
    GL_PACK_SKIP_IMAGES               = 0x806B,
    GL_PACK_IMAGE_HEIGHT              = 0x806C,
    GL_UNPACK_SKIP_IMAGES             = 0x806D,
    GL_UNPACK_IMAGE_HEIGHT            = 0x806E,
    GL_TEXTURE_3D                     = 0x806F,
    GL_PROXY_TEXTURE_3D               = 0x8070,
    GL_TEXTURE_DEPTH                  = 0x8071,
    GL_TEXTURE_WRAP_R                 = 0x8072,
    GL_MAX_3D_TEXTURE_SIZE            = 0x8073,
    GL_UNSIGNED_BYTE_2_3_3_REV        = 0x8362,
    GL_UNSIGNED_SHORT_5_6_5           = 0x8363,
    GL_UNSIGNED_SHORT_5_6_5_REV       = 0x8364,
    GL_UNSIGNED_SHORT_4_4_4_4_REV     = 0x8365,
    GL_UNSIGNED_SHORT_1_5_5_5_REV     = 0x8366,
    GL_UNSIGNED_INT_8_8_8_8_REV       = 0x8367,
    GL_UNSIGNED_INT_2_10_10_10_REV    = 0x8368,
    GL_BGR                            = 0x80E0,
    GL_BGRA                           = 0x80E1,
    GL_MAX_ELEMENTS_VERTICES          = 0x80E8,
    GL_MAX_ELEMENTS_INDICES           = 0x80E9,
    GL_CLAMP_TO_EDGE                  = 0x812F,
    GL_TEXTURE_MIN_LOD                = 0x813A,
    GL_TEXTURE_MAX_LOD                = 0x813B,
    GL_TEXTURE_BASE_LEVEL             = 0x813C,
    GL_TEXTURE_MAX_LEVEL              = 0x813D,
    GL_SMOOTH_POINT_SIZE_RANGE        = 0x0B12,
    GL_SMOOTH_POINT_SIZE_GRANULARITY  = 0x0B13,
    GL_SMOOTH_LINE_WIDTH_RANGE        = 0x0B22,
    GL_SMOOTH_LINE_WIDTH_GRANULARITY  = 0x0B23,
    GL_ALIASED_LINE_WIDTH_RANGE       = 0x846E,

    // OpenGL 1.3
    GL_TEXTURE0                       = 0x84C0,
    GL_TEXTURE1                       = 0x84C1,
    GL_TEXTURE2                       = 0x84C2,
    GL_TEXTURE3                       = 0x84C3,
    GL_TEXTURE4                       = 0x84C4,
    GL_TEXTURE5                       = 0x84C5,
    GL_TEXTURE6                       = 0x84C6,
    GL_TEXTURE7                       = 0x84C7,
    GL_TEXTURE8                       = 0x84C8,
    GL_TEXTURE9                       = 0x84C9,
    GL_TEXTURE10                      = 0x84CA,
    GL_TEXTURE11                      = 0x84CB,
    GL_TEXTURE12                      = 0x84CC,
    GL_TEXTURE13                      = 0x84CD,
    GL_TEXTURE14                      = 0x84CE,
    GL_TEXTURE15                      = 0x84CF,
    GL_TEXTURE16                      = 0x84D0,
    GL_TEXTURE17                      = 0x84D1,
    GL_TEXTURE18                      = 0x84D2,
    GL_TEXTURE19                      = 0x84D3,
    GL_TEXTURE20                      = 0x84D4,
    GL_TEXTURE21                      = 0x84D5,
    GL_TEXTURE22                      = 0x84D6,
    GL_TEXTURE23                      = 0x84D7,
    GL_TEXTURE24                      = 0x84D8,
    GL_TEXTURE25                      = 0x84D9,
    GL_TEXTURE26                      = 0x84DA,
    GL_TEXTURE27                      = 0x84DB,
    GL_TEXTURE28                      = 0x84DC,
    GL_TEXTURE29                      = 0x84DD,
    GL_TEXTURE30                      = 0x84DE,
    GL_TEXTURE31                      = 0x84DF,
    GL_ACTIVE_TEXTURE                 = 0x84E0,
    GL_MULTISAMPLE                    = 0x809D,
    GL_SAMPLE_ALPHA_TO_COVERAGE       = 0x809E,
    GL_SAMPLE_ALPHA_TO_ONE            = 0x809F,
    GL_SAMPLE_COVERAGE                = 0x80A0,
    GL_SAMPLE_BUFFERS                 = 0x80A8,
    GL_SAMPLES                        = 0x80A9,
    GL_SAMPLE_COVERAGE_VALUE          = 0x80AA,
    GL_SAMPLE_COVERAGE_INVERT         = 0x80AB,
    GL_TEXTURE_CUBE_MAP               = 0x8513,
    GL_TEXTURE_BINDING_CUBE_MAP       = 0x8514,
    GL_TEXTURE_CUBE_MAP_POSITIVE_X    = 0x8515,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_X    = 0x8516,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Y    = 0x8517,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Y    = 0x8518,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Z    = 0x8519,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Z    = 0x851A,
    GL_PROXY_TEXTURE_CUBE_MAP         = 0x851B,
    GL_MAX_CUBE_MAP_TEXTURE_SIZE      = 0x851C,
    GL_COMPRESSED_RGB                 = 0x84ED,
    GL_COMPRESSED_RGBA                = 0x84EE,
    GL_TEXTURE_COMPRESSION_HINT       = 0x84EF,
    GL_TEXTURE_COMPRESSED_IMAGE_SIZE  = 0x86A0,
    GL_TEXTURE_COMPRESSED             = 0x86A1,
    GL_NUM_COMPRESSED_TEXTURE_FORMATS = 0x86A2,
    GL_COMPRESSED_TEXTURE_FORMATS     = 0x86A3,
    GL_CLAMP_TO_BORDER                = 0x812D,

    // OpenGL 1.4
    GL_BLEND_DST_RGB                  = 0x80C8,
    GL_BLEND_SRC_RGB                  = 0x80C9,
    GL_BLEND_DST_ALPHA                = 0x80CA,
    GL_BLEND_SRC_ALPHA                = 0x80CB,
    GL_POINT_FADE_THRESHOLD_SIZE      = 0x8128,
    GL_DEPTH_COMPONENT16              = 0x81A5,
    GL_DEPTH_COMPONENT24              = 0x81A6,
    GL_DEPTH_COMPONENT32              = 0x81A7,
    GL_MIRRORED_REPEAT                = 0x8370,
    GL_MAX_TEXTURE_LOD_BIAS           = 0x84FD,
    GL_TEXTURE_LOD_BIAS               = 0x8501,
    GL_INCR_WRAP                      = 0x8507,
    GL_DECR_WRAP                      = 0x8508,
    GL_TEXTURE_DEPTH_SIZE             = 0x884A,
    GL_TEXTURE_COMPARE_MODE           = 0x884C,
    GL_TEXTURE_COMPARE_FUNC           = 0x884D,
    GL_CONSTANT_COLOR                 = 0x8001,
    GL_ONE_MINUS_CONSTANT_COLOR       = 0x8002,
    GL_CONSTANT_ALPHA                 = 0x8003,
    GL_ONE_MINUS_CONSTANT_ALPHA       = 0x8004,
    GL_FUNC_ADD                       = 0x8006,
    GL_MIN                            = 0x8007,
    GL_MAX                            = 0x8008,
    GL_FUNC_SUBTRACT                  = 0x800A,
    GL_FUNC_REVERSE_SUBTRACT          = 0x800B,

    // OpenGL 1.5
    GL_BUFFER_SIZE                    = 0x8764,
    GL_BUFFER_USAGE                   = 0x8765,
    GL_QUERY_COUNTER_BITS             = 0x8864,
    GL_CURRENT_QUERY                  = 0x8865,
    GL_QUERY_RESULT                   = 0x8866,
    GL_QUERY_RESULT_AVAILABLE         = 0x8867,
    GL_ARRAY_BUFFER                   = 0x8892,
    GL_ELEMENT_ARRAY_BUFFER           = 0x8893,
    GL_ARRAY_BUFFER_BINDING           = 0x8894,
    GL_ELEMENT_ARRAY_BUFFER_BINDING   = 0x8895,
    GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING = 0x889F,
    GL_READ_ONLY                      = 0x88B8,
    GL_WRITE_ONLY                     = 0x88B9,
    GL_READ_WRITE                     = 0x88BA,
    GL_BUFFER_ACCESS                  = 0x88BB,
    GL_BUFFER_MAPPED                  = 0x88BC,
    GL_BUFFER_MAP_POINTER             = 0x88BD,
    GL_STREAM_DRAW                    = 0x88E0,
    GL_STREAM_READ                    = 0x88E1,
    GL_STREAM_COPY                    = 0x88E2,
    GL_STATIC_DRAW                    = 0x88E4,
    GL_STATIC_READ                    = 0x88E5,
    GL_STATIC_COPY                    = 0x88E6,
    GL_DYNAMIC_DRAW                   = 0x88E8,
    GL_DYNAMIC_READ                   = 0x88E9,
    GL_DYNAMIC_COPY                   = 0x88EA,
    GL_SAMPLES_PASSED                 = 0x8914,

    // OpenGL 2.0
    GL_BLEND_EQUATION_RGB             = 0x8009,
    GL_VERTEX_ATTRIB_ARRAY_ENABLED    = 0x8622,
    GL_VERTEX_ATTRIB_ARRAY_SIZE       = 0x8623,
    GL_VERTEX_ATTRIB_ARRAY_STRIDE     = 0x8624,
    GL_VERTEX_ATTRIB_ARRAY_TYPE       = 0x8625,
    GL_CURRENT_VERTEX_ATTRIB          = 0x8626,
    GL_VERTEX_PROGRAM_POINT_SIZE      = 0x8642,
    GL_VERTEX_ATTRIB_ARRAY_POINTER    = 0x8645,
    GL_STENCIL_BACK_FUNC              = 0x8800,
    GL_STENCIL_BACK_FAIL              = 0x8801,
    GL_STENCIL_BACK_PASS_DEPTH_FAIL   = 0x8802,
    GL_STENCIL_BACK_PASS_DEPTH_PASS   = 0x8803,
    GL_MAX_DRAW_BUFFERS               = 0x8824,
    GL_DRAW_BUFFER0                   = 0x8825,
    GL_DRAW_BUFFER1                   = 0x8826,
    GL_DRAW_BUFFER2                   = 0x8827,
    GL_DRAW_BUFFER3                   = 0x8828,
    GL_DRAW_BUFFER4                   = 0x8829,
    GL_DRAW_BUFFER5                   = 0x882A,
    GL_DRAW_BUFFER6                   = 0x882B,
    GL_DRAW_BUFFER7                   = 0x882C,
    GL_DRAW_BUFFER8                   = 0x882D,
    GL_DRAW_BUFFER9                   = 0x882E,
    GL_DRAW_BUFFER10                  = 0x882F,
    GL_DRAW_BUFFER11                  = 0x8830,
    GL_DRAW_BUFFER12                  = 0x8831,
    GL_DRAW_BUFFER13                  = 0x8832,
    GL_DRAW_BUFFER14                  = 0x8833,
    GL_DRAW_BUFFER15                  = 0x8834,
    GL_BLEND_EQUATION_ALPHA           = 0x883D,
    GL_MAX_VERTEX_ATTRIBS             = 0x8869,
    GL_VERTEX_ATTRIB_ARRAY_NORMALIZED = 0x886A,
    GL_MAX_TEXTURE_IMAGE_UNITS        = 0x8872,
    GL_FRAGMENT_SHADER                = 0x8B30,
    GL_VERTEX_SHADER                  = 0x8B31,
    GL_MAX_FRAGMENT_UNIFORM_COMPONENTS = 0x8B49,
    GL_MAX_VERTEX_UNIFORM_COMPONENTS  = 0x8B4A,
    GL_MAX_VARYING_FLOATS             = 0x8B4B,
    GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS = 0x8B4C,
    GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS = 0x8B4D,
    GL_SHADER_TYPE                    = 0x8B4F,
    GL_FLOAT_VEC2                     = 0x8B50,
    GL_FLOAT_VEC3                     = 0x8B51,
    GL_FLOAT_VEC4                     = 0x8B52,
    GL_INT_VEC2                       = 0x8B53,
    GL_INT_VEC3                       = 0x8B54,
    GL_INT_VEC4                       = 0x8B55,
    GL_BOOL                           = 0x8B56,
    GL_BOOL_VEC2                      = 0x8B57,
    GL_BOOL_VEC3                      = 0x8B58,
    GL_BOOL_VEC4                      = 0x8B59,
    GL_FLOAT_MAT2                     = 0x8B5A,
    GL_FLOAT_MAT3                     = 0x8B5B,
    GL_FLOAT_MAT4                     = 0x8B5C,
    GL_SAMPLER_1D                     = 0x8B5D,
    GL_SAMPLER_2D                     = 0x8B5E,
    GL_SAMPLER_3D                     = 0x8B5F,
    GL_SAMPLER_CUBE                   = 0x8B60,
    GL_SAMPLER_1D_SHADOW              = 0x8B61,
    GL_SAMPLER_2D_SHADOW              = 0x8B62,
    GL_DELETE_STATUS                  = 0x8B80,
    GL_COMPILE_STATUS                 = 0x8B81,
    GL_LINK_STATUS                    = 0x8B82,
    GL_VALIDATE_STATUS                = 0x8B83,
    GL_INFO_LOG_LENGTH                = 0x8B84,
    GL_ATTACHED_SHADERS               = 0x8B85,
    GL_ACTIVE_UNIFORMS                = 0x8B86,
    GL_ACTIVE_UNIFORM_MAX_LENGTH      = 0x8B87,
    GL_SHADER_SOURCE_LENGTH           = 0x8B88,
    GL_ACTIVE_ATTRIBUTES              = 0x8B89,
    GL_ACTIVE_ATTRIBUTE_MAX_LENGTH    = 0x8B8A,
    GL_FRAGMENT_SHADER_DERIVATIVE_HINT = 0x8B8B,
    GL_SHADING_LANGUAGE_VERSION       = 0x8B8C,
    GL_CURRENT_PROGRAM                = 0x8B8D,
    GL_POINT_SPRITE_COORD_ORIGIN      = 0x8CA0,
    GL_LOWER_LEFT                     = 0x8CA1,
    GL_UPPER_LEFT                     = 0x8CA2,
    GL_STENCIL_BACK_REF               = 0x8CA3,
    GL_STENCIL_BACK_VALUE_MASK        = 0x8CA4,
    GL_STENCIL_BACK_WRITEMASK         = 0x8CA5,

    // OpenGL 2.1
    GL_PIXEL_PACK_BUFFER              = 0x88EB,
    GL_PIXEL_UNPACK_BUFFER            = 0x88EC,
    GL_PIXEL_PACK_BUFFER_BINDING      = 0x88ED,
    GL_PIXEL_UNPACK_BUFFER_BINDING    = 0x88EF,
    GL_FLOAT_MAT2x3                   = 0x8B65,
    GL_FLOAT_MAT2x4                   = 0x8B66,
    GL_FLOAT_MAT3x2                   = 0x8B67,
    GL_FLOAT_MAT3x4                   = 0x8B68,
    GL_FLOAT_MAT4x2                   = 0x8B69,
    GL_FLOAT_MAT4x3                   = 0x8B6A,
    GL_SRGB                           = 0x8C40,
    GL_SRGB8                          = 0x8C41,
    GL_SRGB_ALPHA                     = 0x8C42,
    GL_SRGB8_ALPHA8                   = 0x8C43,
    GL_COMPRESSED_SRGB                = 0x8C48,
    GL_COMPRESSED_SRGB_ALPHA          = 0x8C49,

    // OpenGL 3.0
    GL_COMPARE_REF_TO_TEXTURE         = 0x884E,
    GL_CLIP_DISTANCE0                 = 0x3000,
    GL_CLIP_DISTANCE1                 = 0x3001,
    GL_CLIP_DISTANCE2                 = 0x3002,
    GL_CLIP_DISTANCE3                 = 0x3003,
    GL_CLIP_DISTANCE4                 = 0x3004,
    GL_CLIP_DISTANCE5                 = 0x3005,
    GL_CLIP_DISTANCE6                 = 0x3006,
    GL_CLIP_DISTANCE7                 = 0x3007,
    GL_MAX_CLIP_DISTANCES             = 0x0D32,
    GL_MAJOR_VERSION                  = 0x821B,
    GL_MINOR_VERSION                  = 0x821C,
    GL_NUM_EXTENSIONS                 = 0x821D,
    GL_CONTEXT_FLAGS                  = 0x821E,
    GL_COMPRESSED_RED                 = 0x8225,
    GL_COMPRESSED_RG                  = 0x8226,
    GL_CONTEXT_FLAG_FORWARD_COMPATIBLE_BIT = 0x0001,
    GL_RGBA32F                        = 0x8814,
    GL_RGB32F                         = 0x8815,
    GL_RGBA16F                        = 0x881A,
    GL_RGB16F                         = 0x881B,
    GL_VERTEX_ATTRIB_ARRAY_INTEGER    = 0x88FD,
    GL_MAX_ARRAY_TEXTURE_LAYERS       = 0x88FF,
    GL_MIN_PROGRAM_TEXEL_OFFSET       = 0x8904,
    GL_MAX_PROGRAM_TEXEL_OFFSET       = 0x8905,
    GL_CLAMP_READ_COLOR               = 0x891C,
    GL_FIXED_ONLY                     = 0x891D,
    GL_MAX_VARYING_COMPONENTS         = 0x8B4B,
    GL_TEXTURE_1D_ARRAY               = 0x8C18,
    GL_PROXY_TEXTURE_1D_ARRAY         = 0x8C19,
    GL_TEXTURE_2D_ARRAY               = 0x8C1A,
    GL_PROXY_TEXTURE_2D_ARRAY         = 0x8C1B,
    GL_TEXTURE_BINDING_1D_ARRAY       = 0x8C1C,
    GL_TEXTURE_BINDING_2D_ARRAY       = 0x8C1D,
    GL_R11F_G11F_B10F                 = 0x8C3A,
    GL_UNSIGNED_INT_10F_11F_11F_REV   = 0x8C3B,
    GL_RGB9_E5                        = 0x8C3D,
    GL_UNSIGNED_INT_5_9_9_9_REV       = 0x8C3E,
    GL_TEXTURE_SHARED_SIZE            = 0x8C3F,
    GL_TRANSFORM_FEEDBACK_VARYING_MAX_LENGTH = 0x8C76,
    GL_TRANSFORM_FEEDBACK_BUFFER_MODE = 0x8C7F,
    GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_COMPONENTS = 0x8C80,
    GL_TRANSFORM_FEEDBACK_VARYINGS    = 0x8C83,
    GL_TRANSFORM_FEEDBACK_BUFFER_START = 0x8C84,
    GL_TRANSFORM_FEEDBACK_BUFFER_SIZE = 0x8C85,
    GL_PRIMITIVES_GENERATED           = 0x8C87,
    GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN = 0x8C88,
    GL_RASTERIZER_DISCARD             = 0x8C89,
    GL_MAX_TRANSFORM_FEEDBACK_INTERLEAVED_COMPONENTS = 0x8C8A,
    GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS = 0x8C8B,
    GL_INTERLEAVED_ATTRIBS            = 0x8C8C,
    GL_SEPARATE_ATTRIBS               = 0x8C8D,
    GL_TRANSFORM_FEEDBACK_BUFFER      = 0x8C8E,
    GL_TRANSFORM_FEEDBACK_BUFFER_BINDING = 0x8C8F,
    GL_RGBA32UI                       = 0x8D70,
    GL_RGB32UI                        = 0x8D71,
    GL_RGBA16UI                       = 0x8D76,
    GL_RGB16UI                        = 0x8D77,
    GL_RGBA8UI                        = 0x8D7C,
    GL_RGB8UI                         = 0x8D7D,
    GL_RGBA32I                        = 0x8D82,
    GL_RGB32I                         = 0x8D83,
    GL_RGBA16I                        = 0x8D88,
    GL_RGB16I                         = 0x8D89,
    GL_RGBA8I                         = 0x8D8E,
    GL_RGB8I                          = 0x8D8F,
    GL_RED_INTEGER                    = 0x8D94,
    GL_GREEN_INTEGER                  = 0x8D95,
    GL_BLUE_INTEGER                   = 0x8D96,
    GL_RGB_INTEGER                    = 0x8D98,
    GL_RGBA_INTEGER                   = 0x8D99,
    GL_BGR_INTEGER                    = 0x8D9A,
    GL_BGRA_INTEGER                   = 0x8D9B,
    GL_SAMPLER_1D_ARRAY               = 0x8DC0,
    GL_SAMPLER_2D_ARRAY               = 0x8DC1,
    GL_SAMPLER_1D_ARRAY_SHADOW        = 0x8DC3,
    GL_SAMPLER_2D_ARRAY_SHADOW        = 0x8DC4,
    GL_SAMPLER_CUBE_SHADOW            = 0x8DC5,
    GL_UNSIGNED_INT_VEC2              = 0x8DC6,
    GL_UNSIGNED_INT_VEC3              = 0x8DC7,
    GL_UNSIGNED_INT_VEC4              = 0x8DC8,
    GL_INT_SAMPLER_1D                 = 0x8DC9,
    GL_INT_SAMPLER_2D                 = 0x8DCA,
    GL_INT_SAMPLER_3D                 = 0x8DCB,
    GL_INT_SAMPLER_CUBE               = 0x8DCC,
    GL_INT_SAMPLER_1D_ARRAY           = 0x8DCE,
    GL_INT_SAMPLER_2D_ARRAY           = 0x8DCF,
    GL_UNSIGNED_INT_SAMPLER_1D        = 0x8DD1,
    GL_UNSIGNED_INT_SAMPLER_2D        = 0x8DD2,
    GL_UNSIGNED_INT_SAMPLER_3D        = 0x8DD3,
    GL_UNSIGNED_INT_SAMPLER_CUBE      = 0x8DD4,
    GL_UNSIGNED_INT_SAMPLER_1D_ARRAY  = 0x8DD6,
    GL_UNSIGNED_INT_SAMPLER_2D_ARRAY  = 0x8DD7,
    GL_QUERY_WAIT                     = 0x8E13,
    GL_QUERY_NO_WAIT                  = 0x8E14,
    GL_QUERY_BY_REGION_WAIT           = 0x8E15,
    GL_QUERY_BY_REGION_NO_WAIT        = 0x8E16,
    GL_BUFFER_ACCESS_FLAGS            = 0x911F,
    GL_BUFFER_MAP_LENGTH              = 0x9120,
    GL_BUFFER_MAP_OFFSET              = 0x9121,

    // OpenGL 3.1
    GL_SAMPLER_2D_RECT                = 0x8B63,
    GL_SAMPLER_2D_RECT_SHADOW         = 0x8B64,
    GL_SAMPLER_BUFFER                 = 0x8DC2,
    GL_INT_SAMPLER_2D_RECT            = 0x8DCD,
    GL_INT_SAMPLER_BUFFER             = 0x8DD0,
    GL_UNSIGNED_INT_SAMPLER_2D_RECT   = 0x8DD5,
    GL_UNSIGNED_INT_SAMPLER_BUFFER    = 0x8DD8,
    GL_TEXTURE_BUFFER                 = 0x8C2A,
    GL_MAX_TEXTURE_BUFFER_SIZE        = 0x8C2B,
    GL_TEXTURE_BINDING_BUFFER         = 0x8C2C,
    GL_TEXTURE_BUFFER_DATA_STORE_BINDING = 0x8C2D,
    GL_TEXTURE_BUFFER_FORMAT          = 0x8C2E,
    GL_TEXTURE_RECTANGLE              = 0x84F5,
    GL_TEXTURE_BINDING_RECTANGLE      = 0x84F6,
    GL_PROXY_TEXTURE_RECTANGLE        = 0x84F7,
    GL_MAX_RECTANGLE_TEXTURE_SIZE     = 0x84F8,
    GL_RED_SNORM                      = 0x8F90,
    GL_RG_SNORM                       = 0x8F91,
    GL_RGB_SNORM                      = 0x8F92,
    GL_RGBA_SNORM                     = 0x8F93,
    GL_R8_SNORM                       = 0x8F94,
    GL_RG8_SNORM                      = 0x8F95,
    GL_RGB8_SNORM                     = 0x8F96,
    GL_RGBA8_SNORM                    = 0x8F97,
    GL_R16_SNORM                      = 0x8F98,
    GL_RG16_SNORM                     = 0x8F99,
    GL_RGB16_SNORM                    = 0x8F9A,
    GL_RGBA16_SNORM                   = 0x8F9B,
    GL_SIGNED_NORMALIZED              = 0x8F9C,
    GL_PRIMITIVE_RESTART              = 0x8F9D,
    GL_PRIMITIVE_RESTART_INDEX        = 0x8F9E,

    // OpenGL 3.2
    GL_CONTEXT_CORE_PROFILE_BIT       = 0x00000001,
    GL_CONTEXT_COMPATIBILITY_PROFILE_BIT = 0x00000002,
    GL_LINES_ADJACENCY                = 0x000A,
    GL_LINE_STRIP_ADJACENCY           = 0x000B,
    GL_TRIANGLES_ADJACENCY            = 0x000C,
    GL_TRIANGLE_STRIP_ADJACENCY       = 0x000D,
    GL_PROGRAM_POINT_SIZE             = 0x8642,
    GL_MAX_GEOMETRY_TEXTURE_IMAGE_UNITS = 0x8C29,
    GL_FRAMEBUFFER_ATTACHMENT_LAYERED = 0x8DA7,
    GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS = 0x8DA8,
    GL_GEOMETRY_SHADER                = 0x8DD9,
    GL_GEOMETRY_VERTICES_OUT          = 0x8916,
    GL_GEOMETRY_INPUT_TYPE            = 0x8917,
    GL_GEOMETRY_OUTPUT_TYPE           = 0x8918,
    GL_MAX_GEOMETRY_UNIFORM_COMPONENTS = 0x8DDF,
    GL_MAX_GEOMETRY_OUTPUT_VERTICES   = 0x8DE0,
    GL_MAX_GEOMETRY_TOTAL_OUTPUT_COMPONENTS = 0x8DE1,
    GL_MAX_VERTEX_OUTPUT_COMPONENTS   = 0x9122,
    GL_MAX_GEOMETRY_INPUT_COMPONENTS  = 0x9123,
    GL_MAX_GEOMETRY_OUTPUT_COMPONENTS = 0x9124,
    GL_MAX_FRAGMENT_INPUT_COMPONENTS  = 0x9125,
    GL_CONTEXT_PROFILE_MASK           = 0x9126,

    // OpenGL 3.3
    GL_VERTEX_ATTRIB_ARRAY_DIVISOR   = 0x88FE,

    // OpenGL 4.0
    GL_SAMPLE_SHADING                 = 0x8C36,
    GL_MIN_SAMPLE_SHADING_VALUE       = 0x8C37,
    GL_MIN_PROGRAM_TEXTURE_GATHER_OFFSET = 0x8E5E,
    GL_MAX_PROGRAM_TEXTURE_GATHER_OFFSET = 0x8E5F,
    GL_TEXTURE_CUBE_MAP_ARRAY         = 0x9009,
    GL_TEXTURE_BINDING_CUBE_MAP_ARRAY = 0x900A,
    GL_PROXY_TEXTURE_CUBE_MAP_ARRAY   = 0x900B,
    GL_SAMPLER_CUBE_MAP_ARRAY         = 0x900C,
    GL_SAMPLER_CUBE_MAP_ARRAY_SHADOW  = 0x900D,
    GL_INT_SAMPLER_CUBE_MAP_ARRAY     = 0x900E,
    GL_UNSIGNED_INT_SAMPLER_CUBE_MAP_ARRAY = 0x900F,

    // OpenGL 4.2
    GL_COPY_READ_BUFFER_BINDING  = 0x8F36,
    GL_COPY_WRITE_BUFFER_BINDING = 0x8F37,
    GL_TRANSFORM_FEEDBACK_PAUSED = 0x8E23,
    GL_TRANSFORM_FEEDBACK_ACTIVE = 0x8E24,

    // OpenGL 4.3
    GL_NUM_SHADING_LANGUAGE_VERSIONS = 0x82E9,
    GL_VERTEX_ATTRIB_ARRAY_LONG = 0x874E,
    GL_VERTEX_BINDING_BUFFER = 0x8F4F,

    // OpenGL 4.4
    GL_MAX_VERTEX_ATTRIB_STRIDE       = 0x82E5,
    GL_PRIMITIVE_RESTART_FOR_PATCHES_SUPPORTED = 0x8221,
    GL_TEXTURE_BUFFER_BINDING         = 0x8C2A,

    // OpenGL 4.5
    GL_CONTEXT_FLAG_ROBUST_ACCESS_BIT = 0x00000004,
}