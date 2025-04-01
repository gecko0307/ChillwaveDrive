#version 400 core

#define PI 3.14159265
const float PI2 = PI * 2.0;

in vec3 eyePosition;
in vec3 worldPosition;
in vec3 worldNormal;

in vec4 currPosition;
in vec4 prevPosition;

uniform float time;
uniform float localTime;

uniform vec3 cameraPosition;
uniform vec3 sunDirection;
uniform float sunEnergy;

uniform float turbidity;
const float softness = 0.05;
const float wrap = 1.0;

uniform sampler2D noiseTexture;

#include <gamma.glsl>

layout(location = 0) out vec4 fragColor;
layout(location = 3) out vec4 fragEmission;
layout(location = 4) out vec4 fragVelocity;
layout(location = 5) out vec4 fragRadiance;

const float parallaxScale = 0.06;
const float parallaxBias = -0.01;

float hash(float n)
{
    return fract((1.0 + sin(n)) * 415.92653);
}

float noise3d(vec3 x)
{
    float xhash = hash(round(400 * x.x) * 37.0);
    float yhash = hash(round(400 * x.y) * 57.0);
    float zhash = hash(round(400 * x.z) * 67.0);
    return fract(xhash + yhash + zhash);
}

float normalizeInRange(float xmin, float xmax, float x)
{
    return clamp((x - xmin) / (xmax - xmin), 0.0, 1.0);
}

void main()
{
    vec3 E = normalize(-eyePosition);
    vec3 normal = normalize(worldNormal);
    
    float cosTheta = clamp(dot(normal, sunDirection), 0.0, 1.0);
    float sunHaze = pow(cosTheta, 6.0);
    
    // Atmosphere
    vec3 up = vec3(0.0, 1.0, 0.0);
    float dotProduct = dot(sunDirection, up);
    float st = (dotProduct + 1.0) * 0.5;
    
    vec3 zenithDay = mix(toLinear(vec3(0.4, 0.5, 1.0)) * 3.0, toLinear(vec3(0.5, 0.5, 0.6)), turbidity);
    vec3 zenithSunset = toLinear(vec3(0.5, 0.3, 1.0));
    vec3 zenithNight = toLinear(vec3(0.1, 0.1, 0.2));
    
    float horizonEnergy = max(0.5, 1.0 - turbidity);
    
    // TODO: horizon based on turbidity
    vec3 horizonDay = mix(toLinear(vec3(0.8, 0.8, 0.9)) * 4.0, toLinear(vec3(0.4, 0.4, 0.5)), turbidity);
    vec3 horizonSunset = mix(toLinear(vec3(0.7, 0.3, 0.5)), toLinear(vec3(0.9, 0.4, 0.4)), sunHaze) * horizonEnergy * 2.0;
    vec3 horizonNight = toLinear(vec3(0.2, 0.2, 0.5)) * horizonEnergy;
    
    vec3 zenithColor = mix(zenithNight, zenithSunset, normalizeInRange(0.2, 0.5, st));
    zenithColor = mix(zenithColor, zenithDay, normalizeInRange(0.5, 0.8, st));
    
    vec3 horizonColor = mix(horizonNight, horizonSunset, normalizeInRange(0.2, 0.5, st));
    horizonColor = mix(horizonColor, horizonDay, normalizeInRange(0.5, 0.8, st));
    
    float gradient = clamp(dot(normal, up), 0.0, 1.0);
    gradient = smoothstep(0.0, 0.3, gradient);
    
    vec3 skyColor = mix(horizonColor, zenithColor, gradient);

    // Sun
    vec3 sunDay = toLinear(vec3(1.0, 1.0, 1.0));
    vec3 sunSunset = toLinear(vec3(1.0, 0.5, 0.1));
    vec3 sunColor = mix(sunSunset, sunDay, normalizeInRange(0.5, 0.7, st));
    const float sunAngularDiameterCos = 0.9997;
    float sunDisk = smoothstep(sunAngularDiameterCos, sunAngularDiameterCos + 0.00002, cosTheta);
    skyColor += (sunColor * sunHaze * 10.0 + sunColor * sunDisk * 5.0) * (1.0 - turbidity);
    
    // Stars
    float starsThreshold = 0.995;
    float starsBrightness = 8.0;
    float starsTwinkleSpeed = 1.0;

    float randomFactor = noise3d(normal);
    float starsRadiance = (randomFactor >= starsThreshold)? pow((randomFactor - starsThreshold) / (1.0 - starsThreshold), starsBrightness) : 0.0;
    vec3 starsColor = vec3(0.9 + 0.1 * randomFactor, 0.9, 1.0 - 0.1 * randomFactor);
    
    float twinkleRandomFactor = hash(dot(normal, vec3(12.9898, 78.233, 45.164)));
    float twinkle = 0.5 + 0.5 * sin((twinkleRandomFactor + localTime * starsTwinkleSpeed) * PI2);
    twinkle = mix(0.1, 1.5, twinkle);
    
    skyColor += toLinear(starsColor) * starsRadiance * twinkle * mix(1.0, 0.0, normalizeInRange(0.2, 0.5, st)) * gradient * (1.0 - turbidity);

    // Clouds
    float planeHeight = 0.5;
    vec3 rayOrigin = vec3(0.0, 0.0, 0.0);
    
    float t = planeHeight / normal.y;
    vec3 intersection = t * normal;
    float distance = length(intersection.xz);
    float scaleFactor = 1.0 / (1.0 + log(1.0 + distance * 0.1));
    vec2 uv = intersection.xz * scaleFactor * 0.3;
    uv.x += time * 0.005; // Clouds movement
    
    vec4 noise = texture(noiseTexture, uv);
    float density = noise.r;
    
    float mint = 1.0 - turbidity;
    float maxt = min(mint + softness, 1.0);
    float cloudMask = clamp((density - mint) / (maxt - mint), 0.0, 1.0);
    
    float mintSc = 0.5;
    float maxtSc = min(mintSc + 0.5, 1.0);
    float cloudScattering = 1.0 - clamp((density - mintSc) / (maxtSc - mintSc), 0.0, 1.0);
    cloudScattering = pow(cloudScattering, 3.0) * 10.0;
    
    vec3 cloudsDay = mix(toLinear(vec3(1.0, 1.0, 1.0)) * 3.0, toLinear(vec3(0.4, 0.4, 0.5)), turbidity);
    cloudsDay += mix(toLinear(vec3(1.0, 1.0, 1.0)), toLinear(vec3(0.2, 0.2, 0.3)), turbidity) * cloudScattering;
    
    vec3 cloudsSunset = mix(toLinear(vec3(0.5, 0.3, 0.6)), toLinear(vec3(0.3, 0.1, 0.4)), turbidity * turbidity);
    cloudsSunset += mix(toLinear(vec3(0.0, 0.0, 0.0)), toLinear(vec3(0.2, 0.1, 0.1)), turbidity * turbidity) * cloudScattering;
    
    vec3 cloudsNight = toLinear(vec3(0.1, 0.1, 0.2)) + toLinear(vec3(0.0, 0.1, 0.2)) * cloudScattering;
    
    vec3 cloudColor = mix(cloudsNight, cloudsSunset, normalizeInRange(0.2, 0.6, st));
    cloudColor = mix(cloudColor, cloudsDay, normalizeInRange(0.6, 0.8, st));
    
    // Final color
    vec3 color = mix(skyColor, cloudColor, cloudMask * gradient);
    
    //
    vec2 posScreen = (currPosition.xy / currPosition.w) * 0.5 + 0.5;
    vec2 prevPosScreen = (prevPosition.xy / prevPosition.w) * 0.5 + 0.5;
    vec2 velocity = posScreen - prevPosScreen;
    
    const float blurMask = 1.0;
    const float gbufferMask = 0.0;
    
    fragColor = vec4(color, gbufferMask);
    fragEmission = vec4(0.0, 0.0, 0.0, 1.0);
    fragRadiance = vec4(color, 1.0);
    fragVelocity = vec4(velocity, blurMask, 0.0);
}
