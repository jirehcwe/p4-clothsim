#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

vec4 ambient, diffuse, specular, maxSpec, maxDiff, lightPos, lightDir, diffuseIllum, specularIllum, halfVec, camPos, camDir;
float radiusSq, ambientK, diffuseK, specularK, smoothnessP;

out vec4 out_color;

float h(vec2 uv) {
  vec4 sample = texture(u_texture_2, uv);
  // You may want to use this helper function...
  return sample.r;
}

void main() {
  // YOUR CODE HERE

  
  vec3 normal3 = normalize(v_normal.xyz);
  vec3 tangentVector = normalize(v_tangent.xyz);
  vec3 bitangent = normalize(cross(normal3, tangentVector));

  mat3 tbn = mat3(tangentVector, bitangent, normal3);

  float dU = (h(vec2(v_uv.x + 1/u_texture_2_size.x, v_uv.y))- h(v_uv))*u_height_scaling*u_normal_scaling;
  float dV = (h(vec2(v_uv.x, v_uv.y + 1/u_texture_2_size.y))- h(v_uv))*u_height_scaling*u_normal_scaling;
  vec3 localNormal = vec3(-dU, -dV, 1);
  vec4 worldNormal = vec4(tbn*localNormal,0.0);
  // (Placeholder code. You will want to replace it.)
  ambientK = 0.3;
  diffuseK = 0.4;
  specularK = 5;
  smoothnessP = 15;
  
  lightPos = vec4(u_light_pos, 0.0);
  lightDir = lightPos - v_position;
  radiusSq = length(lightDir) * length(lightDir);

  maxDiff = max(vec4(0.0), dot(worldNormal, normalize(lightDir)));
  diffuseIllum = diffuseK * vec4(u_light_intensity, 0.0)/radiusSq;
  diffuse = diffuseIllum * maxDiff;

  camPos = vec4(u_cam_pos, 0.0);
  camDir = camPos - v_position;
  halfVec =  (camDir + lightDir)/(length(camDir)+length(lightDir));
  maxSpec = pow(max(vec4(0.0), dot(worldNormal, halfVec)), vec4(smoothnessP));
  specularIllum = specularK * vec4(u_light_intensity, 0.0)/radiusSq;
  specular = maxSpec * specularIllum;


  ambient = ambientK * u_color;

  // (Placeholder code. You will want to replace it.)
  out_color =  specular + ambient + diffuse;
  out_color.a = 1;
}

