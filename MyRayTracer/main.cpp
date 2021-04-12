 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#ifdef __unix__ 
	#include "linux.h"
#endif

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <conio.h>
#include <limits>
#include <random>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "maths.h"
#include "sampler.h"
#include "rayAccelerator.h"

#define CAPTION "Whitted Ray-Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define MAX_DEPTH 4

typedef enum { NONE, GRID_ACC, BVH_ACC } Accelerator;
Accelerator Accel_Struct = BVH_ACC;
BVH* bvh_ptr = nullptr;
Grid* grid_ptr = nullptr;

unsigned int FrameCount = 0;

float bias = 0.001f;

// sampling settings
// square root of number of samples per pixel
int n_samples = 8;

// 0 to regular
// 1 to random
int sampler_type = 1;

// light parameters for soft shadows
// lights are modeled as an axis aligned rectangle, with the point in the middle.
// lightSize is the length of the sides.
#define SHADOW_MODE_HARD 1
#define SHADOW_MODE_WITH_ANTI_ALIASING 2
#define SHADOW_MODE_WITHOUT_ANTI_ALIASING 3

float lightSize = 0.05f;
int shadowMode = SHADOW_MODE_WITH_ANTI_ALIASING;
size_t numShadowRays = 16;
std::default_random_engine shadowPrng(time(NULL) * time(NULL));

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];

//Enable OpenGL drawing.  
bool drawModeEnabled = false;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;
int RES_X, RES_Y;

int WindowHandle = 0;

// returns 0.0f for full shadow and 1.0 for full light.
float lightPercentage (Vector interceptPoint, Vector lightPosition, Vector hitNormal) {
	static std::uniform_real_distribution<> shadowDis(-0.5f, 0.5f);

	size_t num_shadow_rays = shadowMode == SHADOW_MODE_WITHOUT_ANTI_ALIASING ? numShadowRays : 1;

	// shoot several shadow rays
	float percentage = 0.0f;
	for (size_t i = 0; i < num_shadow_rays; i++) {
		float offset = shadowMode == SHADOW_MODE_HARD ? 0.0f : shadowDis(shadowPrng);
		Vector shadow_light_position = lightPosition + Vector(1, 1, 0) * (offset * lightSize * 2);
		Vector lightDir = (shadow_light_position - interceptPoint).normalize();
		Ray shadowRay = Ray((interceptPoint + hitNormal*bias), lightDir);
		bool interception = false;
		if (Accel_Struct == GRID_ACC) {
			interception = grid_ptr->Traverse(shadowRay);
		} else if (Accel_Struct == BVH_ACC) {
			interception = bvh_ptr->Traverse(shadowRay);
		} else {
			for (int i = 0; i < scene->getNumObjects(); i++) {
				Object* obj = scene->getObject(i);
				float d;
				if (obj->intercepts(shadowRay, d)) {
					interception = true;
					break;
				}
			}
		}

		if (!interception) {
			percentage += 1.0f;
		}
	}

	return percentage / (float)num_shadow_rays;
}

float mix (const float& a, const float& b, const float& mix) {
	return b * mix + a * (1 - mix);
}

Vector reflectDir (Vector& I, Vector& N) {
	return I - N * 2 * (I*N);
}

Vector refractDir (Vector& I, Vector& N, float& ior) {
	float cosi = clamp(-1, 1, I*N);
	float etai = 1, etat = ior;
	Vector n = N;
	//ray hits from outside
	if (cosi < 0) { cosi = -cosi; }
	//ray hits from outside, swap eta and invert normal
	else { std::swap(etai, etat); n = -N; }
	float eta = etai / etat;
	//check if there is total reflection, return 0 refraction if true
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? Vector() : (I * eta) + (n * (eta * cosi - sqrtf(k)));
}

void fresnel (Vector& I, Vector& N, const float& ior, float& kr) {
	float cosi = clamp(-1, 1, I*N);
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sini using Snell's law
	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
	// Total internal reflection
	if (sint >= 1) kr = 1;
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}
}


Color rayTracing (Ray ray, int depth, float ior_1) {
	// intersect ray with all objects and find closest intersection
	float closest_d = std::numeric_limits<float>::max();
	Object* hit_obj = NULL;
	
	if (Accel_Struct == BVH_ACC) {
		// do BVH intersection
		Vector hit_point;
		if (bvh_ptr->Traverse(ray, &hit_obj, hit_point)) {
			closest_d = (hit_point - ray.origin).length();
		}
	} else if (Accel_Struct == GRID_ACC) {
		// TODO do grid acceleration
		Vector hit_point;
		if (grid_ptr->Traverse(ray, &hit_obj, hit_point)) {
			closest_d = (hit_point - ray.origin).length();
		}
	} else {
		// do naive intersection
		for (int i = 0; i < scene->getNumObjects(); i++) {
			Object* obj = scene->getObject(i);
			float d;
			if (obj->intercepts(ray, d)) {
				if (d < closest_d) {
					closest_d = d;
					hit_obj = obj;
				}
			}
		}
	}

	if (hit_obj == NULL) return scene->GetBackgroundColor();
	else {
		//cout << ray.direction.x << " " << ray.direction.y << " " << ray.direction.z << "\n";
		//compute intercection point and hit normal
		Vector intercept_point = ray.origin + ray.direction * closest_d;
		Vector hit_normal = hit_obj->getNormal(intercept_point);
		Color c;

		//for each light source
		for (int l = 0; l < scene->getNumLights(); l++) {
			Light* source_light = scene->getLight(l);
			Vector L = (source_light->position - intercept_point).normalize();
			Vector halfway_vector = (L - ray.direction).normalize();

			if (L * hit_normal > 0) {
				//diffiuse component
				Color c_buf = source_light->color * hit_obj->GetMaterial()->GetDiffuse() * max(hit_normal * L, 0.0f) * hit_obj->GetMaterial()->GetDiffColor();
				//specular component
				c_buf += source_light->color * hit_obj->GetMaterial()->GetSpecular() * pow(max(halfway_vector * hit_normal, 0.0f), hit_obj->GetMaterial()->GetShine()) * hit_obj->GetMaterial()->GetSpecColor();
				c_buf *= lightPercentage(intercept_point, source_light->position, hit_normal);
				c += c_buf;
			}
		}
		if (depth >= MAX_DEPTH) return c;

		float ior = hit_obj->GetMaterial()->GetRefrIndex();
		bool outside = ray.direction * hit_normal < 0;
		float kr;
		// calculate reflection
		Color reflection;
		if (hit_obj->GetMaterial()->GetReflection() > 0) {
			// compute reflection direction
			Vector reflDir = reflectDir(ray.direction, hit_normal).normalize();
			// compute reflection ray
			Vector reflOrig = outside ? intercept_point + hit_normal * bias : intercept_point - hit_normal * bias;
			Ray reflectionRay = Ray(reflOrig, reflDir);
			reflection = rayTracing(reflectionRay, depth + 1, 1.0f);
			reflection *= hit_obj->GetMaterial()->GetSpecColor() * hit_obj->GetMaterial()->GetSpecular();
		}

		// calculate refraction
		Color refraction;
		if (hit_obj->GetMaterial()->GetTransmittance() > 0) {
			//calculate fresnel
			fresnel(ray.direction, hit_normal, ior, kr);
			// compute refraction ray (transmission)
			if (kr < 1) {
				Vector refrDir = refractDir(ray.direction, hit_normal, ior).normalize();
				Vector refrOrig = outside ? intercept_point - hit_normal * bias : intercept_point + hit_normal * bias;
				Ray refractionRay = Ray(refrOrig, refrDir);
				refraction = rayTracing(refractionRay, depth + 1, 1.0f);
			}
		}

		//calculate mix result of reflection and refraction
		if (hit_obj->GetMaterial()->GetTransmittance() > 0) {
			c += (reflection * kr) + (refraction * (1 - kr));
		} else if (hit_obj->GetMaterial()->GetReflection() > 0)
			c += reflection;

		return c;
	}
}

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos=0;
	int index_col=0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}
	
	shadowPrng.seed(time(NULL) * time(NULL));
	set_rand_seed(time(NULL) * time(NULL));

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color; 

			Vector pixel;  //viewport coordinates

			if (sampler_type == 0) {
				for (int p = 0; p < n_samples; p++) {
					for (int q = 0; q < n_samples; q++) {
						pixel.x = x + (p + 0.5f) / n_samples;
						pixel.y = y + (q + 0.5f) / n_samples;

						Vector samplePoint = sample_unit_disk();
						Vector lensSample;
						lensSample.x = samplePoint.x * scene->GetCamera()->GetAperture();
						lensSample.y = samplePoint.y * scene->GetCamera()->GetAperture();

						Vector focalPoint;
						focalPoint.x = pixel.x * scene->GetCamera()->GetFocalRatio();
						focalPoint.y = pixel.y * scene->GetCamera()->GetFocalRatio();

						Ray sampledRay = scene->GetCamera()->PrimaryRay(lensSample, focalPoint);
						color += rayTracing(sampledRay, 1, 1.0).clamp();
					}
				}
			}

			if (sampler_type == 1) {
				for (int p = 0; p < n_samples; p++) {
					for (int q = 0; q < n_samples; q++) {
						double r = rand_double();
						pixel.x = x + (p + r) / n_samples;
						pixel.y = y + (q + r) / n_samples;

						Vector samplePoint = sample_unit_disk();
						Vector lensSample;
						lensSample.x = samplePoint.x * scene->GetCamera()->GetAperture();
						lensSample.y = samplePoint.y * scene->GetCamera()->GetAperture();

						Vector focalPoint;
						focalPoint.x = pixel.x * scene->GetCamera()->GetFocalRatio();
						focalPoint.y = pixel.y * scene->GetCamera()->GetFocalRatio();

						Ray sampledRay = scene->GetCamera()->PrimaryRay(lensSample, focalPoint);
						color += rayTracing(sampledRay, 1, 1.0).clamp();
					}
				}
			}

			color = color * (1 / pow(n_samples, 2));

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}
	
	}
	if(drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks() 
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}

void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
	
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strncpy(scene_name, scenes_dir, sizeof(scene_name));
			strncat(scene_name, input_user, sizeof(scene_name));

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);
}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			if (Accel_Struct == BVH_ACC) {
				bvh_ptr = new BVH();
				vector<Object*> objs;
				int num_objects = scene->getNumObjects();
				for (int o = 0; o < num_objects; o++) {
					objs.push_back(scene->getObject(o));
				}
				bvh_ptr->Build(objs);
				printf("BVH built.\n\n");
			}

			if (Accel_Struct == GRID_ACC) { 
				grid_ptr = new Grid();
				vector<Object*> objs; 
				int num_objects = scene->getNumObjects(); 
				for (int o = 0; o < num_objects; o++) { 
					objs.push_back(scene->getObject(o)); 
				}
				grid_ptr->Build(objs); 
				printf("Grid built.\n\n"); 
			}

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
	   
		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////