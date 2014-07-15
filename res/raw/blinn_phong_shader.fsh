#version 100
precision mediump float;

struct light
{
	vec4 position;
	lowp vec3 ambient,diffuse,specular;
	mediump vec3 spotDirection;
	float spotExponent,spotCosCutoff,constantAttenuation,
			linearAttenuation,quadraticAttenuation;
};

struct material
{
	lowp vec3 ambient,diffuse,specular;
	float shininess;
	lowp int hasTexture;
};

uniform sampler2D u_Texture;
uniform light u_Light;
uniform material u_Material;
uniform lowp vec3 u_GlobalAmbient;

varying vec4 position;
varying mediump vec3 normal, viewDirection;
varying vec2 uv;

vec3 toonify(vec3 color, float intensity){
	float factor;
	if (intensity > 0.95)
		return color;
	else if (intensity > 0.5)
		return color*0.6;
	else if (intensity > 0.25)
		return color*0.4;
	else
		return color*0.2;
}

vec4 modulate(vec4 color1, vec4 color2){
	return color1*color2;
}

vec4 decal(vec4 color1, vec4 color2){
	return vec4(color1.xyz*(1.0-color2.a)+color2.xyz*color2.a,color1.a);
}

void main()
{	
	mediump vec3 lightDirection;
	float attenuation;
	if (0.0 == u_Light.position.w){
		attenuation = 1.0;
		lightDirection = normalize(u_Light.position.xyz);					
	}
	else{
		mediump vec3 positionToLightSource = u_Light.position.xyz - position.xyz;
		float dist = length(positionToLightSource);
		lightDirection = normalize(positionToLightSource);
		attenuation = 1.0 / (u_Light.constantAttenuation + (u_Light.linearAttenuation + u_Light.quadraticAttenuation * dist) * dist);
		if (u_Light.spotCosCutoff <= 1.57079633){
			float clampedCosine = max(0.0, -dot(lightDirection, u_Light.spotDirection));
			if (clampedCosine < u_Light.spotCosCutoff){
				attenuation = 0.0;
			}
			else{
				attenuation *= pow(clampedCosine, u_Light.spotExponent);   
			}
		}
	}
	lowp vec3 totalLighting = (u_GlobalAmbient + u_Light.ambient*attenuation)* u_Material.ambient;
	float ndotl = dot(normal, lightDirection);
	if (ndotl > 0.0){
		totalLighting += u_Light.diffuse * u_Material.diffuse * (attenuation*ndotl) + u_Light.specular * u_Material.specular * (attenuation*pow(max(0.0, dot((viewDirection+lightDirection), normal)*0.5), u_Material.shininess));
	}
	gl_FragColor = vec4(totalLighting, 1.0);
	if(u_Material.hasTexture == 1){
		gl_FragColor = modulate(gl_FragColor,texture2D(u_Texture,uv));
	}
}