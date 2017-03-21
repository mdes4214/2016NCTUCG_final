layout(binding = 0) uniform sampler2D ltc_mat;
layout(binding = 1) uniform sampler2D ltc_mag;

out vec4 outColor;

uniform float roughness;	// bind roughness   {label:"Roughness", default:0.25, min:0.01, max:1, step:0.001}
uniform vec3 dcolor;		// bind dcolor      {label:"Diffuse Color",  r:1.0, g:1.0, b:1.0}
uniform vec3 scolor;		// bind scolor      {label:"Specular Color", r:1.0, g:1.0, b:1.0}
uniform float intensity;	// bind intensity   {label:"Light Intensity", default:4, min:0, max:10}
uniform float width;		// bind width       {label:"Width",  default: 8, min:0.1, max:15, step:0.1}
uniform float height;		// bind height      {label:"Height", default: 8, min:0.1, max:15, step:0.1}
uniform float roty;			// bind roty        {label:"Rotation Y", default: 0, min:0, max:1, step:0.001}
uniform float rotz;			// bind rotz        {label:"Rotation Z", default: 0, min:0, max:1, step:0.001}
uniform bool twoSided;		// bind twoSided    {label:"Two-sided", default:false}
uniform int mode;

uniform mat4 view;
uniform vec2 resolution;

const float LUT_SIZE = 64.0;
const float LUT_SCALE = (LUT_SIZE - 1.0) / LUT_SIZE;
const float LUT_BIAS = 0.5 / LUT_SIZE;

const float pi = 3.14159265;

struct Ray{ vec3 origin; vec3 dir; };
struct Rect{
	vec3  center;
	vec3  dirx; vec3  diry;
	float halfx; float halfy;
	vec4  plane;
};

bool RayPlaneIntersect(Ray ray, vec4 plane, out float t);
bool RayRectIntersect(Ray ray, Rect rect, out float t);
void InitRect(out Rect rect);
void InitRectPoints(Rect rect, out vec3 points[4]);
Ray GenerateCameraRay();
float IntegrateEdge(vec3 v1, vec3 v2);
void ClipQuadToHorizon(inout vec3 L[5], out int n);
vec3 LTC_Evaluate(vec3 N, vec3 V, vec3 P, mat3 Minv, vec3 points[4], bool twoSided);
vec4 LTC_Rect();

// for 8-edge polygon
bool Ray_8_RectIntersect(Ray ray, Rect rect, out float t);
void Init_8_RectPoints(Rect rect, out vec3 points[8]);
vec3 LTC_8_Evaluate(vec3 N, vec3 V, vec3 P, mat3 Minv, vec3 points[8], bool twoSided);
vec4 LTC_8_Rect();

// for 12-edge polygon
bool Ray_12_RectIntersect(Ray ray, Rect rect, out float t);
void Init_12_RectPoints(Rect rect, out vec3 points[12]);
vec3 LTC_12_Evaluate(vec3 N, vec3 V, vec3 P, mat3 Minv, vec3 points[12], bool twoSided);
vec4 LTC_12_Rect();

//////////////////////////////////////////////////////////////
/* Calculation Tool */

vec3 mul(mat3 m, vec3 v){return m * v;}
mat3 mul(mat3 m1, mat3 m2){return m1 * m2;}
vec3 rotation_y(vec3 v, float a){return vec3(v.x*cos(a) + v.z*sin(a), v.y, -v.x*sin(a) + v.z*cos(a));}
vec3 rotation_z(vec3 v, float a){return vec3(v.x*cos(a) - v.y*sin(a), v.x*sin(a) + v.y*cos(a), v.z);}
vec3 rotation_yz(vec3 v, float ay, float az){return rotation_z(rotation_y(v, ay), az);}
mat3 transpose(mat3 v){
	mat3 tmp;
	tmp[0] = vec3(v[0].x, v[1].x, v[2].x);
	tmp[1] = vec3(v[0].y, v[1].y, v[2].y);
	tmp[2] = vec3(v[0].z, v[1].z, v[2].z);
	return tmp;
}
vec3 PowVec3(vec3 v, float p){return vec3(pow(v.x, p), pow(v.y, p), pow(v.z, p));}
const float gamma = 2.2;
vec3 ToLinear(vec3 v){return PowVec3(v, gamma);}
vec3 ToSRGB(vec3 v){return PowVec3(v, 1.0 / gamma);}

//////////////////////////////////////////////////////////////
/* Intersect */

bool RayPlaneIntersect(Ray ray, vec4 plane, out float t){
	t = -dot(plane, vec4(ray.origin, 1.0)) / dot(plane.xyz, ray.dir);
	return t > 0.0;
}

bool RayRectIntersect(Ray ray, Rect rect, out float t){
	bool intersect = RayPlaneIntersect(ray, rect.plane, t);
	if (intersect)	{
		vec3 pos = ray.origin + ray.dir*t;
		vec3 lpos = pos - rect.center;

		float x = dot(lpos, rect.dirx);
		float y = dot(lpos, rect.diry);

		if (abs(x) > rect.halfx || abs(y) > rect.halfy)			// note: cut outside
			intersect = false;
	}
	return intersect;
}

bool Ray_8_RectIntersect(Ray ray, Rect rect, out float t){
	bool intersect = RayPlaneIntersect(ray, rect.plane, t);
	if (intersect)	{
		vec3 pos = ray.origin + ray.dir*t;
		vec3 lpos = pos - rect.center;

		float x = dot(lpos, rect.dirx);
		float y = dot(lpos, rect.diry);

		if (abs(x) > rect.halfx || abs(y) > rect.halfy)			// note: cut outside
			intersect = false;
		else if (abs(x) > rect.halfx*1.0/2.4 || abs(y) > rect.halfy*1.0/2.4){
			for(float i = 0; i < 1.4; i += 0.05){
				if(abs(x) > rect.halfx*(1.0+i)/2.4 && abs(y) > rect.halfy*(2.4-i)/2.4){
					intersect = false;
					break;
				}
			}
		}
	}
	return intersect;
}

bool Ray_12_RectIntersect(Ray ray, Rect rect, out float t){
	bool intersect = RayPlaneIntersect(ray, rect.plane, t);
	if (intersect)	{
		vec3 pos = ray.origin + ray.dir*t;
		vec3 lpos = pos - rect.center;

		float x = dot(lpos, rect.dirx);
		float y = dot(lpos, rect.diry);

		if (abs(x) > rect.halfx || abs(y) > rect.halfy)			// note: cut outside
			intersect = false;
		else if (abs(x) > rect.halfx*1.0/2.4 && abs(y) > rect.halfy*1.0/2.4)
			intersect = false;
	}
	return intersect;
}

//////////////////////////////////////////////////////////////
/* Init */

void InitRect(out Rect rect){
	rect.dirx = rotation_yz(vec3(1, 0, 0), roty*2.0*pi, rotz*2.0*pi);
	rect.diry = rotation_yz(vec3(0, 1, 0), roty*2.0*pi, rotz*2.0*pi);

	rect.center = vec3(0, 6, 32);
	rect.halfx = 0.5*width;
	rect.halfy = 0.5*height;

	vec3 rectNormal = cross(rect.dirx, rect.diry);
	rect.plane = vec4(rectNormal, -dot(rectNormal, rect.center)); 		// note: rect's normal
}

void InitRectPoints(Rect rect, out vec3 points[4]){
	vec3 ex = rect.halfx * rect.dirx;
	vec3 ey = rect.halfy * rect.diry;

	points[0] = rect.center - ex - ey;			// note: counterclockwise, from left-down to left-up
	points[1] = rect.center + ex - ey;
	points[2] = rect.center + ex + ey;
	points[3] = rect.center - ex + ey;
}

void Init_8_RectPoints(Rect rect, out vec3 points[8]){
	vec3 ex1 = rect.halfx * rect.dirx;
	vec3 ex2 = rect.halfx * rect.dirx * 1.0/2.4;
	vec3 ey1 = rect.halfy * rect.diry;
	vec3 ey2 = rect.halfy * rect.diry * 1.0/2.4;

	points[0] = rect.center - ex2 - ey1;		// note: counterclockwise, from left-down
	points[1] = rect.center + ex2 - ey1;
	points[2] = rect.center + ex1 - ey2;
	points[3] = rect.center + ex1 + ey2;
	points[4] = rect.center + ex2 + ey1;
	points[5] = rect.center - ex2 + ey1;
	points[6] = rect.center - ex1 + ey2;
	points[7] = rect.center - ex1 - ey2;
}

void Init_12_RectPoints(Rect rect, out vec3 points[12]){
	vec3 ex1 = rect.halfx * rect.dirx;
	vec3 ex2 = rect.halfx * rect.dirx * 1.0/2.0;
	vec3 ey1 = rect.halfy * rect.diry;
	vec3 ey2 = rect.halfy * rect.diry * 1.0/2.0;

	points[0] = rect.center - ex2 - ey1;		// note: counterclockwise, from left-down
	points[1] = rect.center + ex2 - ey1;
	points[2] = rect.center + ex2 - ey2;
	points[3] = rect.center + ex1 - ey2;
	points[4] = rect.center + ex1 + ey2;
	points[5] = rect.center + ex2 + ey2;
	points[6] = rect.center + ex2 + ey1;
	points[7] = rect.center - ex2 + ey1;
	points[8] = rect.center - ex2 + ey2;
	points[9] = rect.center - ex1 + ey2;
	points[10] = rect.center - ex1 - ey2;
	points[11] = rect.center - ex2 - ey2;
}

Ray GenerateCameraRay(){// Camera functions
	Ray ray;

	// Random jitter within pixel for AA
	vec2 xy = 2.0*(gl_FragCoord.xy) / resolution - vec2(1.0);

	ray.dir = normalize(vec3(xy, 2.0));

	float focalDistance = 2.0;
	float ft = focalDistance / ray.dir.z;
	vec3 pFocus = ray.dir*ft;

	ray.origin = vec3(0);
	ray.dir = normalize(pFocus - ray.origin);

	// Apply camera transform
	ray.origin = (view*vec4(ray.origin, 1)).xyz;
	ray.dir = (view*vec4(ray.dir, 0)).xyz;

	return ray;
}

//////////////////////////////////////////////////////////////
/* Main LTC Algorithm */

float IntegrateEdge(vec3 v1, vec3 v2){// Linearly Transformed Cosines
	float cosTheta = dot(v1, v2);
	float theta = acos(cosTheta);
	float res = cross(v1, v2).z * ((theta > 0.001) ? theta / sin(theta) : 1.0);

	return res;
}

void ClipQuadToHorizon(inout vec3 L[5], out int n){
	// detect clipping config
	int config = 0;
	if (L[0].z > 0.0) config += 1;
	if (L[1].z > 0.0) config += 2;
	if (L[2].z > 0.0) config += 4;
	if (L[3].z > 0.0) config += 8;

	// clip
	n = 0;

	if (config == 0){// clip all
	}
	else if (config == 1) {// V1 clip V2 V3 V4
		n = 3;
		L[1] = -L[1].z * L[0] + L[0].z * L[1];
		L[2] = -L[3].z * L[0] + L[0].z * L[3];
	}
	else if (config == 2) {// V2 clip V1 V3 V4
		n = 3;
		L[0] = -L[0].z * L[1] + L[1].z * L[0];
		L[2] = -L[2].z * L[1] + L[1].z * L[2];
	}
	else if (config == 3) {// V1 V2 clip V3 V4
		n = 4;
		L[2] = -L[2].z * L[1] + L[1].z * L[2];
		L[3] = -L[3].z * L[0] + L[0].z * L[3];
	}
	else if (config == 4){ // V3 clip V1 V2 V4
		n = 3;
		L[0] = -L[3].z * L[2] + L[2].z * L[3];
		L[1] = -L[1].z * L[2] + L[2].z * L[1];
	}
	else if (config == 5) {// V1 V3 clip V2 V4) impossible
		n = 0;
	}
	else if (config == 6) {// V2 V3 clip V1 V4
		n = 4;
		L[0] = -L[0].z * L[1] + L[1].z * L[0];
		L[3] = -L[3].z * L[2] + L[2].z * L[3];
	}
	else if (config == 7){ // V1 V2 V3 clip V4
		n = 5;
		L[4] = -L[3].z * L[0] + L[0].z * L[3];
		L[3] = -L[3].z * L[2] + L[2].z * L[3];
	}
	else if (config == 8){ // V4 clip V1 V2 V3
		n = 3;
		L[0] = -L[0].z * L[3] + L[3].z * L[0];
		L[1] = -L[2].z * L[3] + L[3].z * L[2];
		L[2] = L[3];
	}
	else if (config == 9) {// V1 V4 clip V2 V3
		n = 4;
		L[1] = -L[1].z * L[0] + L[0].z * L[1];
		L[2] = -L[2].z * L[3] + L[3].z * L[2];
	}
	else if (config == 10) {// V2 V4 clip V1 V3) impossible
		n = 0;
	}
	else if (config == 11) {// V1 V2 V4 clip V3
		n = 5;
		L[4] = L[3];
		L[3] = -L[2].z * L[3] + L[3].z * L[2];
		L[2] = -L[2].z * L[1] + L[1].z * L[2];
	}
	else if (config == 12) {// V3 V4 clip V1 V2
		n = 4;
		L[1] = -L[1].z * L[2] + L[2].z * L[1];
		L[0] = -L[0].z * L[3] + L[3].z * L[0];
	}
	else if (config == 13) {// V1 V3 V4 clip V2
		n = 5;
		L[4] = L[3];
		L[3] = L[2];
		L[2] = -L[1].z * L[2] + L[2].z * L[1];
		L[1] = -L[1].z * L[0] + L[0].z * L[1];
	}
	else if (config == 14) {// V2 V3 V4 clip V1
		n = 5;
		L[4] = -L[0].z * L[3] + L[3].z * L[0];
		L[0] = -L[0].z * L[1] + L[1].z * L[0];
	}
	else if (config == 15) {// V1 V2 V3 V4
		n = 4;
	}
	if (n == 3)		L[3] = L[0];
	if (n == 4)		L[4] = L[0];
}

vec3 LTC_Evaluate(vec3 N, vec3 V, vec3 P, mat3 Minv, vec3 points[4], bool twoSided)
{
	// construct orthonormal basis around N
	vec3 T1, T2;
	T1 = normalize(V - N*dot(V, N));
	T2 = cross(N, T1);

	// rotate area light in (T1, T2, N) basis
	Minv = mul(Minv, transpose(mat3(T1, T2, N)));

	// polygon (allocate 5 vertices for clipping)
	vec3 L[5];
	for( int i=0; i<4; i++){
		L[i] = mul(Minv, points[i] - P);
	}

	int n;
	ClipQuadToHorizon(L, n);

	if (n == 0)	return vec3(0, 0, 0);

	// project onto sphere
	for( int i=0; i<5; i++){
		L[i] = normalize(L[i]) ;//* (1 + 0.0001*(dot(normalize(points[i] - P), vec3(0,1,0))));
	}
	// integrate
	float sum = 0.0;

	sum += IntegrateEdge(L[0], L[1]);
	sum += IntegrateEdge(L[1], L[2]);
	sum += IntegrateEdge(L[2], L[3]);
	if (n >= 4) sum += IntegrateEdge(L[3], L[4]);
	if (n == 5)	sum += IntegrateEdge(L[4], L[0]);

	sum = twoSided ? abs(sum) : max(0.0, sum);

	vec3 Lo_i = vec3(sum, sum, sum);

	return Lo_i;
}

vec3 LTC_8_Evaluate(vec3 N, vec3 V, vec3 P, mat3 Minv, vec3 points[8], bool twoSided)
{
	// construct orthonormal basis around N
	vec3 T1, T2;
	T1 = normalize(V - N*dot(V, N));
	T2 = cross(N, T1);

	// rotate area light in (T1, T2, N) basis
	Minv = mul(Minv, transpose(mat3(T1, T2, N)));

	// polygon
	vec3 L[9];
	for( int i=0; i<8; i++){
		L[i] = mul(Minv, points[i] - P);
	}

	// project onto sphere
	for( int i=0; i<8; i++){
		L[i] = normalize(L[i]) ;//* (1 + 0.0001*(dot(normalize(points[i] - P), vec3(0,1,0))));
	}

	// integrate
	float sum = 0.0;
	L[8] = L[0];
	for(int i = 0; i < 8; i++){
		sum += IntegrateEdge(L[i], L[i+1]);
	}

	sum = twoSided ? abs(sum) : max(0.0, sum);

	vec3 Lo_i = vec3(sum, sum, sum);

	return Lo_i;
}

vec3 LTC_12_Evaluate(vec3 N, vec3 V, vec3 P, mat3 Minv, vec3 points[12], bool twoSided)
{
	// construct orthonormal basis around N
	vec3 T1, T2;
	T1 = normalize(V - N*dot(V, N));
	T2 = cross(N, T1);

	// rotate area light in (T1, T2, N) basis
	Minv = mul(Minv, transpose(mat3(T1, T2, N)));

	// polygon
	vec3 L[13];
	for( int i=0; i<12; i++){
		L[i] = mul(Minv, points[i] - P);
	}

	// project onto sphere
	for( int i=0; i<12; i++){
		L[i] = normalize(L[i]) ;//* (1 + 0.0001*(dot(normalize(points[i] - P), vec3(0,1,0))));
	}

	// integrate
	float sum = 0.0;
	L[12] = L[0];
	for(int i = 0; i < 12; i++){
		sum += IntegrateEdge(L[i], L[i+1]);
	}

	sum = twoSided ? abs(sum) : max(0.0, sum);

	vec3 Lo_i = vec3(sum, sum, sum);

	return Lo_i;
}

//////////////////////////////////////////////////////////////
/* Main */

vec4 LTC_Rect(){
	Rect rect;
	InitRect(rect);

	vec3 points[4];
	InitRectPoints(rect, points);

	vec4 floorPlane = vec4(0, 1, 0, 0);

	vec3 lcol = vec3(intensity);
	vec3 dcol = ToLinear(dcolor);
	vec3 scol = ToLinear(scolor);

	vec3 col = vec3(0);

	Ray ray = GenerateCameraRay();

	float distToFloor;
	bool hitFloor = RayPlaneIntersect(ray, floorPlane, distToFloor);
	if (hitFloor)
	{
		vec3 pos = ray.origin + ray.dir*distToFloor;

		vec3 N = floorPlane.xyz;
		vec3 V = -ray.dir;

		float theta;
		theta = acos(dot(N, V));
		vec2 uv = vec2(roughness, theta / (0.5*pi));
		uv = uv*LUT_SCALE + LUT_BIAS;
		vec4 t = texture2D(ltc_mat, uv);
		mat3 Minv = mat3(		// note: transform matrix
			vec3(1, 0, t.y),
			vec3(0, t.z, 0),
			vec3(t.w, 0, t.x)
		);

		vec3 spec = LTC_Evaluate(N, V, pos, Minv, points, twoSided);
		spec *= texture2D(ltc_mag, uv).w;
		
		vec3 diff = LTC_Evaluate(N, V, pos, mat3(1), points, twoSided);

		col = lcol*(scol*spec + dcol*diff);
		col /= 2.0*pi;
	}

	float distToRect;
	if (RayRectIntersect(ray, rect, distToRect))		// note: make the rect shine
		if ((distToRect < distToFloor) || !hitFloor)
			col = lcol;

	return vec4(col, 1.0);
}

vec4 LTC_8_Rect(){
	Rect rect;
	InitRect(rect);

	vec3 points[8];
	Init_8_RectPoints(rect, points);

	vec4 floorPlane = vec4(0, 1, 0, 0);

	vec3 lcol = vec3(intensity);
	vec3 dcol = ToLinear(dcolor);
	vec3 scol = ToLinear(scolor);

	vec3 col = vec3(0);

	Ray ray = GenerateCameraRay();

	float distToFloor;
	bool hitFloor = RayPlaneIntersect(ray, floorPlane, distToFloor);
	if (hitFloor)
	{
		vec3 pos = ray.origin + ray.dir*distToFloor;

		vec3 N = floorPlane.xyz;
		vec3 V = -ray.dir;

		float theta;
		theta = acos(dot(N, V));
		vec2 uv = vec2(roughness, theta / (0.5*pi));
		uv = uv*LUT_SCALE + LUT_BIAS;
		vec4 t = texture2D(ltc_mat, uv);
		mat3 Minv = mat3(		// note: transform matrix
			vec3(1, 0, t.y),
			vec3(0, t.z, 0),
			vec3(t.w, 0, t.x)
		);

		vec3 spec = LTC_8_Evaluate(N, V, pos, Minv, points, twoSided);
		spec *= texture2D(ltc_mag, uv).w;
		vec2 xy = 2.0*(gl_FragCoord.xy) / resolution - vec2(1.0);
		if(xy.y > -0.2)
			spec = vec3(0);

		vec3 diff = LTC_8_Evaluate(N, V, pos, mat3(1), points, twoSided);

		col = lcol*(scol*spec + dcol*diff);
		col /= 2.0*pi;
	}

	float distToRect;
	if (Ray_8_RectIntersect(ray, rect, distToRect))		// note: make the rect shine
		if ((distToRect < distToFloor) || !hitFloor)
			col = lcol;

	return vec4(col, 1.0);
}

vec4 LTC_12_Rect(){
	Rect rect;
	InitRect(rect);

	vec3 points[12];
	Init_12_RectPoints(rect, points);

	vec4 floorPlane = vec4(0, 1, 0, 0);

	vec3 lcol = vec3(intensity);
	vec3 dcol = ToLinear(dcolor);
	vec3 scol = ToLinear(scolor);

	vec3 col = vec3(0);

	Ray ray = GenerateCameraRay();

	float distToFloor;
	bool hitFloor = RayPlaneIntersect(ray, floorPlane, distToFloor);
	if (hitFloor)
	{
		vec3 pos = ray.origin + ray.dir*distToFloor;

		vec3 N = floorPlane.xyz;
		vec3 V = -ray.dir;

		float theta;
		theta = acos(dot(N, V));
		vec2 uv = vec2(roughness, theta / (0.5*pi));
		uv = uv*LUT_SCALE + LUT_BIAS;
		vec4 t = texture2D(ltc_mat, uv);
		mat3 Minv = mat3(		// note: transform matrix
			vec3(1, 0, t.y),
			vec3(0, t.z, 0),
			vec3(t.w, 0, t.x)
		);

		vec3 spec = LTC_12_Evaluate(N, V, pos, Minv, points, twoSided);
		spec *= texture2D(ltc_mag, uv).w;
		vec2 xy = 2.0*(gl_FragCoord.xy) / resolution - vec2(1.0);
		if(xy.y > -0.2)
			spec = vec3(0);

		vec3 diff = LTC_12_Evaluate(N, V, pos, mat3(1), points, twoSided);

		col = lcol*(scol*spec + dcol*diff);
		col /= 2.0*pi;
	}

	float distToRect;
	if (Ray_12_RectIntersect(ray, rect, distToRect))		// note: make the rect shine
		if ((distToRect < distToFloor) || !hitFloor)
			col = lcol;

	return vec4(col, 1.0);
}

void main(){
	if(mode == 1)
		outColor = LTC_8_Rect();
	else if(mode == 2)
		outColor = LTC_12_Rect();
	else
		outColor = LTC_Rect();
}