/**
 * common.glsl
 * Common types and functions used for ray tracing.
 */

const float pi = 3.14159265358979;
const float epsilon = 0.001;

struct Ray {
    vec3 o;     // origin
    vec3 d;     // direction - always set with normalized vector
    float t;    // time, for motion blur
};

Ray createRay(vec3 o, vec3 d, float t) {
    Ray r;
    r.o = o;
    r.d = d;
    r.t = t;
    return r;
}

Ray createRay(vec3 o, vec3 d) {
    return createRay(o, d, 0.0);
}

vec3 pointOnRay(Ray r, float t) {
    return r.o + r.d * t;
}

float gSeed = 0.0;

uint baseHash(uvec2 p) {
    p = 1103515245U * ((p >> 1U) ^ (p.yx));
    uint h32 = 1103515245U * ((p.x) ^ (p.y>>3U));
    return h32 ^ (h32 >> 16);
}

float hash1(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    return float(n) / float(0xffffffffU);
}

vec2 hash2(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    uvec2 rz = uvec2(n, n * 48271U);
    return vec2(rz.xy & uvec2(0x7fffffffU)) / float(0x7fffffff);
}

vec3 hash3(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1, seed += 0.1)));
    uvec3 rz = uvec3(n, n * 16807U, n * 48271U);
    return vec3(rz & uvec3(0x7fffffffU)) / float(0x7fffffff);
}

float rand(vec2 v) {
    return fract(sin(dot(v.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 toLinear(vec3 c) {
    return pow(c, vec3(2.2));
}

vec3 toGamma(vec3 c) {
    return pow(c, vec3(1.0 / 2.2));
}

vec2 randomInUnitDisk(inout float seed) {
    vec2 h = hash2(seed) * vec2(1.0, 6.28318530718);
    float phi = h.y;
    float r = sqrt(h.x);
	return r * vec2(sin(phi), cos(phi));
}

vec3 randomInUnitSphere(inout float seed) {
    vec3 h = hash3(seed) * vec3(2.0, 6.28318530718, 1.0) - vec3(1.0, 0.0, 0.0);
    float phi = h.y;
    float r = pow(h.z, 1.0/3.0);
	return r * vec3(sqrt(1.0 - h.x * h.x) * vec2(sin(phi), cos(phi)), h.x);
}

struct Camera {
    vec3 eye;
    vec3 lowerLeftCorner;
    vec3 horizontal;
    vec3 vertical;
    vec3 u, v, n;
    float width, height;
    float lensRadius;
    float planeDist, focusDist;
    float time0, time1;
};

Camera createCamera(
    vec3 eye,
    vec3 at,
    vec3 worldUp,
    float fovy,
    float aspect,
    float aperture,  //diametro em multiplos do pixel size
    float focusDist,  //focal ratio
    float time0,
    float time1)
{
    Camera cam;
    if(aperture == 0.0) cam.focusDist = 1.0; //pinhole camera then focus in on vis plane
    else cam.focusDist = focusDist;
    vec3 w = eye - at;
    cam.planeDist = length(w);
    cam.height = 2.0 * tan(fovy * pi / 180.0 * 0.5);
    cam.width = aspect * cam.height;

    cam.lensRadius = aperture * 0.5 * cam.width / iResolution.x;  //aperture ratio * pixel size; (1 pixel=lente raio 0.5)
    cam.eye = eye;
    cam.n = normalize(w);
    cam.u = normalize(cross(worldUp, cam.n));
    cam.v = cross(cam.n, cam.u);
    cam.time0 = time0;
    cam.time1 = time1;

    cam.horizontal = cam.width * cam.focusDist * cam.u;
    cam.vertical = cam.height * cam.focusDist * cam.v;
    cam.lowerLeftCorner = cam.eye - cam.horizontal/2.0 - cam.vertical/2.0 - cam.focusDist*cam.n;
    return cam;
}

Ray getRay(Camera cam, vec2 pixel_sample) { //rnd pixel_sample viewport coordinates
    // Lens sample for DOF
    vec2 ls = cam.lensRadius * randomInUnitDisk(gSeed);
    // Time for motion blur
    float time = cam.time0 + hash1(gSeed) * (cam.time1 - cam.time0);
    // New ray origin with lens sample offset
    vec3 offset = cam.u * ls.x + cam.v * ls.y;
    // Ray direction from lens sample to projected sample point
    //vec3 ray_dir = (cam.u*((pixel_sample.x - ls.x) / iResolution.x - 0.5f)*cam.width + cam.v*((pixel_sample.y - ls.y) / iResolution.y - 0.5f)*cam.height - cam.n*cam.focusDist);	
    vec3 ray_dir = cam.lowerLeftCorner + (pixel_sample.x * cam.horizontal / iResolution.x)  + (pixel_sample.y * cam.vertical / iResolution.y) - cam.eye - offset;
    
    return createRay(cam.eye + offset, normalize(ray_dir), time);
}

// MT_ material type
#define MT_DIFFUSE 0
#define MT_METAL 1
#define MT_DIALECTRIC 2

struct Material {
    int type;
    vec3 albedo;
    float roughness; // controls roughness
    float refIdx; // index of refraction for dialectric
};

Material createDiffuseMaterial(vec3 albedo) {
    Material m;
    m.type = MT_DIFFUSE;
    m.albedo = albedo;
    return m;
}

Material createMetalMaterial(vec3 albedo, float roughness) {
    Material m;
    m.type = MT_METAL;
    m.albedo = albedo;
    m.roughness = roughness;
    return m;
}

Material createDialectricMaterial(vec3 albedo, float refIdx, float roughness) {
    Material m;
    m.type = MT_DIALECTRIC;
    m.albedo = albedo;
    m.refIdx = refIdx;
    m.roughness = roughness;
    return m;
}

struct HitRecord {
    vec3 pos;
    vec3 normal;
    float t;            // ray parameter
    Material material;
};

// Check if there is refraction or total internal reflection
bool refract(vec3 rayDir, vec3 normal, float niOverNt, out vec3 refracted) {
    vec3 uv = normalize(rayDir);
    float dt = dot(uv,normal);
    float discriminant = 1.0 - niOverNt*niOverNt*(1.0-dt*dt);
    if (discriminant > 0.0) {
        refracted = niOverNt*(uv - normal*dt) - normal*sqrt(discriminant);
        return true;
    } else return false;
}

// Calculation of Schlick aproximation
float schlick(float cosine, float refIdx) {
    float r0 = (1.0-refIdx) / (1.0+refIdx);
    r0 = r0*r0;
    return r0 + (1.0-r0)*pow((1.0-cosine),5.0);
}

bool scatter(Ray rIn, HitRecord rec, out vec3 atten, out Ray rScattered) {
    // Scattered ray for diffuse materials
    if(rec.material.type == MT_DIFFUSE) {
        // Calculate S point in circunference of unit sphere of intersection point
        vec3 target = rec.pos + rec.normal + normalize(randomInUnitSphere(gSeed));
        // Calculate scattered ray with direction from intersection point to S
        rScattered = createRay(rec.pos, normalize(target-rec.pos), rIn.t);
        // Use albedo for attenuation
        atten = rec.material.albedo * max(dot(rScattered.d, rec.normal), 0.0) / pi;
        return true;
    }
    // Scattered ray for metal materials
    if(rec.material.type == MT_METAL) {
        // Calculate reflected ray direction
        vec3 reflected = reflect(normalize(rIn.d), rec.normal);
        // Calculate scattered ray with direction from reflected ray and roughness parameter for fuzzy reflections
        rScattered = createRay(rec.pos, normalize(reflected + rec.material.roughness*randomInUnitSphere(gSeed)), rIn.t);
        // Use albedo for attenuation
        atten = rec.material.albedo;
        return (dot(rScattered.d, rec.normal) > 0.0);
    }
    // Scattered ray for dialectric materials
    if(rec.material.type == MT_DIALECTRIC) {
        atten = rec.material.albedo;
        vec3 outwardNormal;
        float niOverNt;
        float cosine;
        vec3 refracted;
        float reflect_prob;

        // Check if ray hits from inside or outside to calculate hit normal, refraction index and cosine
        if(dot(rIn.d, rec.normal) > 0.0) { //hit inside
            outwardNormal = -rec.normal;
            niOverNt = rec.material.refIdx;
            cosine = rec.material.refIdx * dot(rIn.d, rec.normal); 
        } else { 
            outwardNormal = rec.normal;
            niOverNt = 1.0 / rec.material.refIdx;
            cosine = -dot(rIn.d, rec.normal); 
        }

        // Check total internal reflection
        if (refract(rIn.d, outwardNormal, niOverNt, refracted)) {
            // If there is refraction, calculate reflection probability with schlick approximation
            reflect_prob = schlick(cosine, rec.material.refIdx);
        } else {
            // If there is total internal reflection, reflection probability is 1
            reflect_prob = 1.0;
        }
        // Decide if scatter a reflected ray or a refracted ray using random function
        if (hash1(gSeed) < reflect_prob) {
            // Calculate reflected direction
            vec3 reflected = reflect(rIn.d, rec.normal);
            // Calculate scattered ray using reflection
            rScattered = createRay(rec.pos, normalize(reflected + rec.material.roughness*randomInUnitSphere(gSeed)), rIn.t);
        } else {
            // Calculate scattered ray using refraction
            rScattered = createRay(rec.pos, normalize(refracted + rec.material.roughness*randomInUnitSphere(gSeed)), rIn.t);
        }

        return true;
    }
    return false;
}

struct pointLight {
    vec3 pos;
    vec3 color;
};

pointLight createPointLight(vec3 pos, vec3 color) {
    pointLight l;
    l.pos = pos;
    l.color = color;
    return l;
}

struct Triangle {vec3 a; vec3 b; vec3 c; };

Triangle createTriangle(vec3 v0, vec3 v1, vec3 v2) {
    Triangle t;
    t.a = v0; t.b = v1; t.c = v2;
    return t;
}

bool hit_triangle(Triangle t, Ray r, float tmin, float tmax, out HitRecord rec) {
    // Calculate the triangle normal
    vec3 ba = t.b - t.a;
    vec3 ca = t.c - t.a;
    vec3 normal = normalize(cross(ba, ca));
    
    // Matrix values
	float a = t.a.x - t.b.x, b = t.a.x - t.c.x, c = r.d.x, d = t.a.x - r.o.x;
	float e = t.a.y - t.b.y, f = t.a.y - t.c.y, g = r.d.y, h = t.a.y - r.o.y;
	float i = t.a.z - t.b.z, j = t.a.z - t.c.z, k = r.d.z, l = t.a.z - r.o.z;

	// Determinants
	float m = f * k - g * j;
	float n = h * k - g * l;
	float p = f * l - h * j;
	float q = g * i - e * k;
	float s = e * j - f * i;

    float denom = (a * m + b * q + c * s);

	float beta = (d * m - b * n - c * p ) / denom;

	if (beta < 0.0f) return false;

	float extra = e * l - h * i;
	float gamma = ( a * n + d * q + c * extra) / denom;

	if (gamma < 0.0f) return false;

	if (beta + gamma > 1.0f) return false;

    // Calculate t parameter for intersection distance
	float dist = (a * p - b * extra + d * s) / denom;

    // Update object attributes
    if(dist < tmax && dist > tmin) {
        rec.t = dist;
        rec.normal = normal;
        rec.pos = pointOnRay(r, rec.t);
        return true;
    }
    return false;
}


struct Sphere {
    vec3 center;
    float radius;
};

Sphere createSphere(vec3 center, float radius) {
    Sphere s;
    s.center = center;
    s.radius = radius;
    return s;
}


struct MovingSphere {
    vec3 center0, center1;
    float radius;
    float time0, time1;
};

MovingSphere createMovingSphere(vec3 center0, vec3 center1, float radius, float time0, float time1) {
    MovingSphere s;
    s.center0 = center0;
    s.center1 = center1;
    s.radius = radius;
    s.time0 = time0;
    s.time1 = time1;
    return s;
}

// Calculate moving center for moving spheres using time parameters (motion blur effect)
vec3 center(MovingSphere mvsphere, float time) {
    return mvsphere.center0 + ((time - mvsphere.time0) / (mvsphere.time1 - mvsphere.time0)) * (mvsphere.center1 - mvsphere.center0);
}

/*
 * The function naming convention changes with these functions to show that they implement a sort of interface for
 * the book's notion of "hittable". E.g. hit_<type>.
 */

bool hit_sphere(Sphere s, Ray r, float tmin, float tmax, out HitRecord rec) {
    // Intersection check
    vec3 oc = r.o - s.center;
    float a = dot(r.d, r.d);
    float b = dot(oc, r.d);
    float c = dot(oc, oc) - s.radius * s.radius;
    float discriminant = b * b - a * c;
    if(discriminant > 0.0) {
        float sqrtDiscriminant = sqrt(discriminant);

        // Calculate t parameters for sphere intersection
        float temp = (-b - sqrtDiscriminant) / a;
        if(temp < tmax && temp > tmin) {
            rec.t = temp;
            rec.pos = pointOnRay(r, rec.t);
            rec.normal = (rec.pos - s.center) / s.radius;
            return true;
        }
        temp = (-b + sqrtDiscriminant) / a;
        if(temp < tmax && temp > tmin) {
            rec.t = temp;
            rec.pos = pointOnRay(r, rec.t);
            rec.normal = (rec.pos - s.center) / s.radius;
            return true;
        }
    }
    return false;
}

bool hit_movingSphere(MovingSphere s, Ray r, float tmin, float tmax, out HitRecord rec) {
    // Intersection check
    // Calculate moving sphere center with time parameter
    vec3 sphereCenter = center(s, r.t);
    vec3 oc = r.o - sphereCenter;
    float a = dot(r.d, r.d);
    float b = dot(oc, r.d);
    float c = dot(oc, oc) - s.radius * s.radius;
    float discriminant = b * b - a * c;
    if(discriminant > 0.0) {
        float sqrtDiscriminant = sqrt(discriminant);

        // Calculate t parameters for sphere intersection
        float temp = (-b - sqrtDiscriminant) / a;
        if(temp < tmax && temp > tmin) {
            rec.t = temp;
            rec.pos = pointOnRay(r, rec.t);
            rec.normal = (rec.pos - sphereCenter) / s.radius;
            return true;
        }
        temp = (-b + sqrtDiscriminant) / a;
        if(temp < tmax && temp > tmin) {
            rec.t = temp;
            rec.pos = pointOnRay(r, rec.t);
            rec.normal = (rec.pos - sphereCenter) / s.radius;
            return true;
        }
    }
    return false;

}
