// Native OCCT presentation shader for the authored metallic/roughness and AO slots.
// Material uniforms, texture units, transforms, culling and history stay owned by OCCT.
#pragma once
#include <Graphic3d_ShaderObject.hxx>
#include <Graphic3d_ShaderProgram.hxx>
#include <Graphic3d_TextureSetBits.hxx>
#include <Graphic3d_ShaderAttribute.hxx>

namespace {
bool IsCore3DDataMapShader(const Handle(Graphic3d_ShaderProgram)& program)
{
    return !program.IsNull() && program->GetId().StartsWith("shapeyard-data-maps-v1-");
}

Handle(Graphic3d_ShaderProgram) MakeCore3DDataMapShader(Standard_Integer bits)
{
    Handle(Graphic3d_ShaderProgram) program = new Graphic3d_ShaderProgram();
    const bool normal = (bits & Graphic3d_TextureSetBits_Normal) != 0;
    program->SetId(TCollection_AsciiString(normal ? "shapeyard-data-maps-v1-normal-" : "shapeyard-data-maps-v1-") + bits);
    program->SetPBR(Standard_True);
    program->SetDefaultSampler(Standard_False);
    // OCCT's automatic normal-map prelude uses non-ES2 mat2x3 derivatives.
    // Declare the standard normal sampler ourselves: OCCT initializes its unit
    // on every GPU program creation. One-shot proxy variables are cleared after
    // use and lose custom sampler bindings when a program is recreated.
    program->SetTextureSetBits(bits & ~Graphic3d_TextureSetBits_Normal);
    if (normal) {
        Graphic3d_ShaderAttributeList attributes;
        attributes.Append(new Graphic3d_ShaderAttribute("syTangent", 4));
        program->SetVertexAttributes(attributes);
    }
    program->SetNbLightsMax(0);
    program->SetNbClipPlanesMax(8);
    program->SetHeader("#version 100");
    const char* vertex = R"GLSL(
varying vec3 syWorldPosition;
varying vec3 syWorldNormal;
varying vec2 syUV;
#ifdef SY_HAS_NORMAL_MAP
attribute vec4 syTangent;
varying vec3 syWorldTangent;
varying vec3 syWorldBitangent;
#endif
void main() {
    vec4 world = occModelWorldMatrix * occVertex;
    syWorldPosition = world.xyz / world.w;
    syWorldNormal = normalize((occModelWorldMatrixInverseTranspose * vec4(occNormal, 0.0)).xyz);
#ifdef SY_HAS_NORMAL_MAP
    vec3 t = (occModelWorldMatrix * vec4(syTangent.xyz, 0.0)).xyz;
    syWorldTangent = normalize(t - syWorldNormal * dot(syWorldNormal, t));
    float determinant = dot(cross(occModelWorldMatrix[0].xyz, occModelWorldMatrix[1].xyz), occModelWorldMatrix[2].xyz);
    syWorldBitangent = cross(syWorldNormal, syWorldTangent) * syTangent.w * (determinant < 0.0 ? -1.0 : 1.0);
#endif
    float s = occTextureTrsf_RotationSin(), c = occTextureTrsf_RotationCos();
    vec2 uv = vec2(occTexCoord.x * c - occTexCoord.y * s, occTexCoord.x * s + occTexCoord.y * c);
    syUV = (uv + occTextureTrsf_Translation()) * occTextureTrsf_Scale();
    gl_Position = occProjectionMatrix * occWorldViewMatrix * world;
}
)GLSL";
    const char* fragment = R"GLSL(
varying vec3 syWorldPosition;
varying vec3 syWorldNormal;
varying vec2 syUV;
#ifdef SY_HAS_NORMAL_MAP
uniform sampler2D occSamplerNormal;
varying vec3 syWorldTangent;
varying vec3 syWorldBitangent;
#endif
#ifdef THE_HAS_TEXTURE_COLOR
uniform sampler2D occSamplerBaseColor;
#endif
void main() {
    if (occFragEarlyReturn()) return;
    // Fixed loop bounds keep ES 2 uniform indexing valid; a satisfied member
    // skips the rest of its OR chain, while every distinct chain must pass.
    int nextPlane = 0;
    for (int i = 0; i < 8; ++i) {
        if (i >= nextPlane && i < occClipPlaneCount) {
            if (dot(occClipPlaneEquations[i], vec4(syWorldPosition, 1.0)) < 0.0) {
                if (occClipPlaneChains[i] <= 1) discard;
                nextPlane = i + 1;
            } else {
                nextPlane = i + (occClipPlaneChains[i] > 1 ? occClipPlaneChains[i] : 1);
            }
        }
    }
    vec4 surface = occMaterialBaseColor(gl_FrontFacing, syUV);
    if (occAlphaCutoff <= 1.0 && surface.a < occAlphaCutoff) discard;
    vec3 base = surface.rgb;
    vec3 emission = occMaterialEmission(gl_FrontFacing, syUV);
    float roughness = clamp(occMaterialRoughness(gl_FrontFacing, syUV), 0.04, 1.0);
    float metallic = clamp(occMaterialMetallic(gl_FrontFacing, syUV), 0.0, 1.0);
    float occlusion = 1.0;
#ifdef THE_HAS_TEXTURE_OCCLUSION
    occlusion = clamp(occTexture2D(occSamplerOcclusion, syUV).r, 0.0, 1.0);
#endif
    vec3 n = normalize(syWorldNormal) * (gl_FrontFacing ? 1.0 : -1.0);
#ifdef SY_HAS_NORMAL_MAP
    vec3 mappedNormal = occTexture2D(occSamplerNormal, syUV).rgb * 2.0 - 1.0;
    // Retain the interpolated vertex basis used by Mikk's inverse bake.
    n = normalize(mappedNormal.x * syWorldTangent + mappedNormal.y * syWorldBitangent + mappedNormal.z * syWorldNormal)
        * (gl_FrontFacing ? 1.0 : -1.0);
#endif
    vec3 light = normalize(vec3(0.35, 0.45, 0.82));
    vec3 eye = (occWorldViewMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec3 view = normalize(eye - syWorldPosition);
    vec3 halfVector = normalize(light + view);
    float nl = clamp(dot(n, light), 0.0, 1.0);
    float nv = max(clamp(dot(n, view), 0.0, 1.0), 0.0001);
    float nh = clamp(dot(n, halfVector), 0.0, 1.0);
    float vh = clamp(dot(view, halfVector), 0.0, 1.0);
    float ior = clamp(occPBRMaterial_IOR(gl_FrontFacing), 1.0, 3.0);
    float dielectric = pow((ior - 1.0) / (ior + 1.0), 2.0);
    vec3 f0 = mix(vec3(dielectric), base, metallic);
    vec3 fresnel = f0 + (vec3(1.0) - f0) * pow(1.0 - vh, 5.0);
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;
    float d = max(nh * nh * (alpha2 - 1.0) + 1.0, 0.0001);
    float distribution = alpha2 / (3.14159265 * d * d);
    float k = pow(roughness + 1.0, 2.0) * 0.125;
    float geometry = (nv / (nv * (1.0 - k) + k)) * (nl / (nl * (1.0 - k) + k));
    vec3 specular = distribution * geometry * fresnel / max(4.0 * nv * nl, 0.0001);
    vec3 diffuse = (vec3(1.0) - fresnel) * (1.0 - metallic) * base / 3.14159265;
    vec3 direct = (diffuse + specular) * nl * 2.2;
    vec3 ambient = base * (0.055 + 0.075 * (1.0 - metallic)) + f0 * (0.025 * (1.0 - roughness));
    // GLView explicitly owns an ES2 RGBA8 drawable, and OCCT's ES2 path has
    // no sRGB framebuffer encoding. Texture color samples and PBR factors
    // are linear, so encode the final light sum once for that display target.
    vec3 linear = max(ambient * occlusion + direct + emission, vec3(0.0));
    vec3 low = linear * 12.92;
    vec3 high = 1.055 * pow(linear, vec3(1.0 / 2.4)) - 0.055;
    vec3 encoded = vec3(linear.r <= 0.0031308 ? low.r : high.r,
                        linear.g <= 0.0031308 ? low.g : high.g,
                        linear.b <= 0.0031308 ? low.b : high.b);
    occSetFragColor(vec4(encoded, surface.a));
}
)GLSL";
    const TCollection_AsciiString prefix = normal ? "#define SY_HAS_NORMAL_MAP\n" : "";
    program->AttachShader(Graphic3d_ShaderObject::CreateFromSource(Graphic3d_TOS_VERTEX, prefix + vertex));
    program->AttachShader(Graphic3d_ShaderObject::CreateFromSource(Graphic3d_TOS_FRAGMENT, prefix + fragment));
    return program;
}
} // namespace
