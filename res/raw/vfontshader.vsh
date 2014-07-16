#version 100
uniform mediump vec3 u_Position;
uniform lowp float u_Size;
attribute mediump vec2 a_Position;
attribute mediump vec2 a_UV;
varying mediump vec2 uv;
void main ()
{
  uv = a_UV;
  gl_Position.xy = u_Position.xy + (a_Position * u_Size);
  gl_Position.z = u_Position.z;
  gl_Position.w = 1.0;
}

