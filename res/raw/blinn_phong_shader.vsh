#version 100
uniform mat4 u_MVMatrix, u_MVPMatrix;
uniform mat3 u_NMatrix;
attribute vec4 a_Position;
attribute mediump vec3 a_Normal;
attribute vec2 a_UV;
varying vec4 position;
varying mediump vec3 normal, viewDirection;
varying vec2 uv;

void main()
{
	position =  a_Position * u_MVMatrix;
	viewDirection = normalize(-position.xyz);
	normal = normalize(a_Normal * u_NMatrix);
	uv = a_UV;
	gl_Position = a_Position * u_MVPMatrix;
}