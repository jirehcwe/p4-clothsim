#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

vec4 ambient, diffuse, specular, maxSpec, maxDiff, lightPos, lightDir, diffuseIllum, specularIllum, halfVec, camPos, camDir;
float radiusSq, ambientK, diffuseK, specularK, smoothnessP;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  ambientK = 0.3;
  diffuseK = 0.4;
  specularK = 5;
  smoothnessP = 15;
  
  lightPos = vec4(u_light_pos, 0.0);
  lightDir = lightPos - v_position;
  radiusSq = length(lightDir) * length(lightDir);

  maxDiff = max(vec4(0.0), dot(v_normal, normalize(lightDir)));
  diffuseIllum = diffuseK * vec4(u_light_intensity, 0.0)/radiusSq;
  diffuse = diffuseIllum * maxDiff;

  camPos = vec4(u_cam_pos, 0.0);
  camDir = camPos - v_position;
  halfVec =  (camDir + lightDir)/(length(camDir)+length(lightDir));
  maxSpec = pow(max(vec4(0.0), dot(v_normal, halfVec)), vec4(smoothnessP));
  specularIllum = specularK * vec4(u_light_intensity, 0.0)/radiusSq;
  specular = maxSpec * specularIllum;


  ambient = ambientK * u_color;

  // (Placeholder code. You will want to replace it.)
  out_color =  specular + ambient + diffuse;
  out_color.a = 1;
}

