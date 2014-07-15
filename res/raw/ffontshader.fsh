#version 100
uniform sampler2D u_Texture;
uniform lowp vec4 u_Color;
varying mediump vec2 uv;
void main ()
{
  gl_FragColor = u_Color * texture2D (u_Texture, uv);
}

