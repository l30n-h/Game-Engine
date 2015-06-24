#version 100
uniform mediump mat4 u_MVPMatrix;
attribute mediump vec4 a_Position;
void main ()
{
	gl_Position = a_Position * u_MVPMatrix;
}

