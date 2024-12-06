#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_buffer_reference2 : require

const int maxresult=150;

struct payload_t
{
    vec3 query;
    int foundNeighbors;
    float maxDistElemf;
	int optixIndices[maxresult];
};


layout(location = 0) rayPayloadInEXT payload_t hitValue;
hitAttributeEXT vec2 attribs;

layout(binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

layout(set=0,binding =3, scalar) buffer NormalBuffer {
   vec3 normals[];
} normalBuffer;
void main()
{	
	const uint idx = gl_LaunchIDEXT.x*720+gl_LaunchIDEXT.y;
	hitValue.optixIndices[hitValue.foundNeighbors]=gl_PrimitiveID;
	hitValue.foundNeighbors++;
}