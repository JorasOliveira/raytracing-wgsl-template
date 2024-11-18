
fn linear_to_gamma_channel(channel: f32) -> f32
{
  return pow(channel, 0.4545);
}

fn linear_to_gamma(color: vec3f) -> vec3f
{
  return vec3f(linear_to_gamma_channel(color.x), linear_to_gamma_channel(color.y), linear_to_gamma_channel(color.z));
}

fn degrees_to_radians(degrees: f32) -> f32
{
  return degrees * PI / 180.0;
}

fn mapfb(p: vec2u, rez: f32) -> i32
{
  return i32(p.x) + i32(p.y) * i32(rez);
}

fn int_to_rgb(c: i32) -> vec3f
{
  var r = f32((c >> 16) & 0xff) / 255.0;
  var g = f32((c >> 8) & 0xff) / 255.0;
  var b = f32(c & 0xff) / 255.0;

  return vec3f(r, g, b);
}

fn rgb_to_int(c: vec4f) -> i32
{
  var r = i32(c.x * 255.0) << 16;
  var g = i32(c.y * 255.0) << 8;
  var b = i32(c.z * 255.0);

  return r | g | b;
}

fn AABB_intersect(r: ray, boxMin: vec3f, boxMax: vec3f) -> bool
{
  var tMin = (boxMin - r.origin) / r.direction;
  var tMax = (boxMax - r.origin) / r.direction;

  var t1 = min(tMin, tMax);
  var t2 = max(tMin, tMax);

  var tNear = max(max(t1.x, t1.y), t1.z);
  var tFar = min(min(t2.x, t2.y), t2.z);

  var hit = tFar >= tNear && tFar > 0.0;
  return hit;
}

fn quaternion_from_euler(euler: vec3f) -> vec4f
{
  var c1 = cos(euler.x / 2.0);
  var c2 = cos(euler.y / 2.0);
  var c3 = cos(euler.z / 2.0);
  var s1 = sin(euler.x / 2.0);
  var s2 = sin(euler.y / 2.0);
  var s3 = sin(euler.z / 2.0);

  var q = vec4f(0.0);
  q.x = s1 * c2 * c3 + c1 * s2 * s3;
  q.y = c1 * s2 * c3 - s1 * c2 * s3;
  q.z = c1 * c2 * s3 + s1 * s2 * c3;
  q.w = c1 * c2 * c3 - s1 * s2 * s3;

  return q;
}

fn qmul(q1: vec4f, q2: vec4f) -> vec4f
{
  return vec4f(
    q2.xyz * q1.w + q1.xyz * q2.w + cross(q1.xyz, q2.xyz),
    q1.w * q2.w - dot(q1.xyz, q2.xyz)
  );
}

fn q_conjugate(q: vec4f) -> vec4f
{
  return vec4f(-q.xyz, q.w);
}

fn q_inverse(q: vec4f) -> vec4f
{
  return q_conjugate(q) / dot(q, q);
}

fn rotate_vector(v: vec3f, r: vec4f) -> vec3f
{
  var r_c = r * vec4f(-1.0, -1.0, -1.0, 1.0);
  return qmul(r, qmul(vec4f(v, 0.0), r_c)).xyz;
}

fn rotate_ray_quaternion(r: ray, center: vec3f, q: vec4f) -> ray
{
  var origin = rotate_vector(r.origin - center, q) + center;
  var direction = rotate_vector(r.direction, q);
  return ray(origin, direction);
}

// my helper funcs:
// Compute reflectance using Schlick's approximation
fn reflectance(cos_theta: f32, refraction_ratio: f32) -> f32 {
    let r0 = pow(((1.0 - refraction_ratio) / (1.0 + refraction_ratio)), 2.0);
    return r0 + (1.0 - r0) * pow((1.0 - cos_theta), 5.0);
}

fn refract(cos_theta: f32, direction: vec3f, refraction_ratio: f32, normal: vec3f) -> vec3f {
    // Perpendicular component of the refracted ray
    let r_out_perpendicular = refraction_ratio * (direction + cos_theta * normal);

    // Parallel component of the refracted ray
    let parallel_component_magnitude = 1.0 - dot(r_out_perpendicular, r_out_perpendicular);

    // Handle numerical precision issues in sqrt
    var r_out_parallel = vec3f(0.0);
    
    if (parallel_component_magnitude > 0.0)
    {
      r_out_parallel = -sqrt(parallel_component_magnitude) * normal;
    } 

    // Return the sum of the perpendicular and parallel components
    return normalize(r_out_perpendicular + r_out_parallel);
}
