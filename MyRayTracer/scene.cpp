#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include "maths.h"
#include "scene.h"


Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */
	normal = (P1 - P0) % (P2 - P0);
	normal.normalize();

	//Calculate the Min and Max for bounding box
	Min = Vector(+FLT_MAX, +FLT_MAX, +FLT_MAX);
	Max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);


	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t ) {

	//matrix values
	float a = points[0].x - points[1].x, b = points[0].x - points[2].x, c = r.direction.x, d = points[0].x - r.origin.x;
	float e = points[0].y - points[1].y, f = points[0].y - points[2].y, g = r.direction.y, h = points[0].y - r.origin.y;
	float i = points[0].z - points[1].z, j = points[0].z - points[2].z, k = r.direction.z, l = points[0].z - r.origin.z;

	//determinats
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

	t = (a * p - b * extra + d * s) / denom;

	if (t < EPSILON) return false;

	return (true);
}

Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
   float l;

   //Calculate the normal plane: counter-clockwise vectorial product.
	PN = (P1 - P0) % (P2 - P0);
   

   if ((l=PN.length()) == 0.0)
   {
     cerr << "DEGENERATED PLANE!\n";
   }
   else
   {
     PN.normalize();

	 D = -(PN.x * P0.x + PN.y * P0.y + PN.z * P0.z);
   }
}

//
// Ray/Plane intersection test.
//

bool Plane::intercepts( Ray& r, float& t )
{
	//Check if Plane is Parallel to ray
	if (PN * r.direction == 0) { 
		return (false);
	}

	Vector a = Vector(0, 0, -D/PN.z);

	t = - ((r.origin - a) * PN) / (PN * r.direction);
	
	if (t < 0) {
		return (false);
	} 
   return (true);
}

Vector Plane::getNormal(Vector point) 
{
  return PN;
}

bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0) return false;
	else if (discr == 0) x0 = x1 = -0.5 * b / a;
	else {
		float q = (b > 0) ?
			-0.5 * (b + sqrt(discr)) :
			-0.5 * (b - sqrt(discr));
		x0 = q / a;
		x1 = c / q;
	}
	if (x0 > x1) std::swap(x0, x1);

	return true;
}

bool Sphere::intercepts(Ray& r, float& t )
{
	float t0, t1;
	Vector oc = r.origin - center;
	float a = r.direction * r.direction;
	float b = oc*r.direction*2.0f;
	float c = (oc*oc) - SqRadius;

	if (!solveQuadratic(a, b, c, t0, t1)) return false;
	if (t0 > t1) std::swap(t0, t1);

	if (t0 < 0) {
		t0 = t1; // if t0 is negative, let's use t1 instead 
		if (t0 < 0) return false; // both t0 and t1 are negative 
	}

	t = t0;

	return true;
}


Vector Sphere::getNormal( Vector point )
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox() {
	Vector a_min;
	Vector a_max ;
	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

bool aaBox::intercepts(Ray& r, float& t)
{
	float ox = r.origin.x, oy = r.origin.y, oz = r.origin.z;
	float dx = r.direction.x, dy = r.direction.y, dz = r.direction.z;

	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	float a = 1.0 / dx;
	if (a >= 0) {
		tx_min = (min.x - ox) * a;
		tx_max = (max.x - ox) * a;
	}
	else {
		tx_min = (max.x - ox) * a;
		tx_max = (min.x - ox) * a;
	}

	float b = 1.0 / dy;
	if (b >= 0) {
		ty_min = (min.y - oy) * b;
		ty_max = (max.y - oy) * b;
	}
	else {
		ty_min = (max.y - oy) * b;
		ty_max = (min.y - oy) * b;
	}

	float c = 1.0 / dz;
	if (c >= 0) {
		tz_min = (min.z - oz) * c;
		tz_max = (max.z - oz) * c;
	}
	else {
		tz_min = (max.z - oz) * c;
		tz_max = (min.z - oz) * c;
	}

	float t0, t1;
	int face_in, face_out;

	// find largest entering t value
	if (tx_min > ty_min) {
		t0 = tx_min;
	}
	else {
		t0 = ty_min;
	}

	if (tz_min > t0) {
		t0 = tz_min;
	}

	// find smallest exiting t value

	if (tx_max < ty_max) {
		t1 = tx_max;
	}
	else {
		t1 = ty_max;
	}

	if (tz_max < t1) {
		t1 = tz_max;
	}

	if (t0 < t1 && t1 > EPSILON) {
		if (t0 > EPSILON) {
			t = t0;
		}
		else {
			t = t1;
		}
		return(true);
	}
	else {
		return (false);
	}
}

Vector aaBox::getFaceNormal(int face_hit) {
	switch (face_hit)
	{
	case 0: return (Vector(-1, 0, 0)); //-x face
	case 1: return (Vector(0, -1, 0)); //-y face
	case 2: return (Vector(0, 0, -1)); //-z face
	case 3: return (Vector(1, 0, 0));  //+x face
	case 4: return (Vector(0, 1, 0));  //+y face
	case 5: return (Vector(0, 0, 1));  //+z face
	default: break;
	}
}

Vector aaBox::getNormal(Vector point)
{
	
	if (abs(point.x - min.x) < EPSILON) {
		return getFaceNormal(0);
	}
	else if (abs(point.x - max.x) < EPSILON) {
		return getFaceNormal(3);
	}
	else if (abs(point.y - min.y) < EPSILON) {
		return getFaceNormal(1);
	}
	else if (abs(point.y - max.y) < EPSILON) {
		return getFaceNormal(4);
	}
	else if (abs(point.z - min.z) < EPSILON) {
		return getFaceNormal(2);
	}
	else if (abs(point.z - max.z) < EPSILON) {
		return getFaceNormal(5);
	}
}

Scene::Scene()
{}

Scene::~Scene()
{
	/*for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.erase();
	*/
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char *sky_dir)
{
	char *filenames[6];
	char buffer[100];
	const char *maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strncpy(buffer, sky_dir, sizeof(buffer));
		strncat(buffer, maps[i], sizeof(buffer));
		filenames[i] = (char *)malloc(sizeof(buffer));
		strncpy(filenames[i], buffer, sizeof(buffer));
	}
	
	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte *)malloc(size);
		ILubyte *bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char *token, const char *name)
{
  file >> token;
  if (strcmp(token, name))
    cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char *name)
{
  const	int	lineSize = 1024;
  string	cmd;
  char		token	[256];
  ifstream	file(name, ios::in);
  Material *	material;

  material = NULL;

  if (file >> cmd)
  {
    while (true)
    {
      
	  if (cmd == "f")   //Material
      {
	    double Kd, Ks, Shine, T, ior;
	    Color cd, cs;

	    file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

	    material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
      }

      else if (cmd == "s")    //Sphere
      {
	     Vector center;
    	 float radius;
         Sphere* sphere;

	    file >> center >> radius;
        sphere = new Sphere(center,radius);
	    if (material) sphere->SetMaterial(material);
        this->addObject( (Object*) sphere);
      }

	  else if (cmd == "box")    //axis aligned box
	  {
		  Vector minpoint, maxpoint;
		  aaBox	*box;

		  file >> minpoint >> maxpoint;
		  box = new aaBox(minpoint, maxpoint);
		  if (material) box->SetMaterial(material);
		  this->addObject((Object*)box);
	  }
	  else if (cmd == "p")  // Polygon: just accepts triangles for now
      {
		  Vector P0, P1, P2;
		  Triangle* triangle;
		  unsigned total_vertices;
		  
		  file >> total_vertices;
		  if (total_vertices == 3)
		  {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(P0, P1, P2);
			  if (material) triangle->SetMaterial(material);
			  this->addObject( (Object*) triangle);
		  }
		  else
		  {
			  cerr << "Unsupported number of vertices.\n";
			  break;
		  }
      }
      
	  else if (cmd == "mesh") {
		  unsigned total_vertices, total_faces;
		  unsigned P0, P1, P2;
		  Triangle* triangle;
		  Vector* verticesArray, vertex;

		  file >> total_vertices >> total_faces;
		  verticesArray = (Vector*)malloc(total_vertices * sizeof(Vector));
		  for (int i = 0; i < total_vertices; i++) {
			  file >> vertex;
			  verticesArray[i] = vertex;
		  }
		  for (int i = 0; i < total_faces; i++) {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(verticesArray[P0 - 1], verticesArray[P1 - 1], verticesArray[P2 - 1]); //vertex index start at 1
			  if (material) triangle->SetMaterial(material);
			  this->addObject((Object*)triangle);
		  }

	  }

	  else if (cmd == "pl")  // General Plane
	  {
          Vector P0, P1, P2;
		  Plane* plane;

          file >> P0 >> P1 >> P2;
          plane = new Plane(P0, P1, P2);
	      if (material) plane->SetMaterial(material);
          this->addObject( (Object*) plane);
	  }

      else if (cmd == "l")  // Need to check light color since by default is white
      {
	    Vector pos;
        Color color;

	    file >> pos >> color;
	    
	      this->addLight(new Light(pos, color));
	    
      }
      else if (cmd == "v")
      {
	    Vector up, from, at;
	    float fov, hither;
	    int xres, yres;
        Camera* camera;
		float focal_ratio; //ratio beteween the focal distance and the viewplane distance
		float aperture_ratio; // number of times to be multiplied by the size of a pixel

	    next_token (file, token, "from");
	    file >> from;

	    next_token (file, token, "at");
	    file >> at;

	    next_token (file, token, "up");
	    file >> up;

	    next_token (file, token, "angle");
	    file >> fov;

	    next_token (file, token, "hither");
	    file >> hither;

	    next_token (file, token, "resolution");
	    file >> xres >> yres;

		next_token(file, token, "aperture");
		file >> aperture_ratio;

		next_token(file, token, "focal");
		file >> focal_ratio;
	    // Create Camera
		camera = new Camera( from, at, up, fov, hither, 100.0*hither, xres, yres, aperture_ratio, focal_ratio);
        this->SetCamera(camera);
      }

      else if (cmd == "bclr")   //Background color
      {
		Color bgcolor;
		file >> bgcolor;
		this->SetBackgroundColor(bgcolor);
	  }
	
	  else if (cmd == "env")
	  {
		  file >> token;
		  
		  this->LoadSkybox(token);
		  this->SetSkyBoxFlg(true);
	  }
      else if (cmd[0] == '#')
      {
	    file.ignore (lineSize, '\n');
      }
      else
      {
	    cerr << "unknown command '" << cmd << "'.\n";
	    break;
      }
      if (!(file >> cmd))
        break;
    }
  }

  file.close();
  return true;
};

void Scene::create_random_scene() {
	Camera* camera;
	Material* material;
	Sphere* sphere;

	set_rand_seed(time(NULL) * time(NULL) * time(NULL));
	material = NULL;
	this->SetSkyBoxFlg(false);  //init with no skybox

	this->SetBackgroundColor(Color(0.5, 0.7, 1.0));
	//this->LoadSkybox("skybox");
	//this->SetSkyBoxFlg(true);
	
	camera = new Camera(Vector(13.0, 2.0, 3.0), Vector(0.0, 0.0, 0), Vector(0.0, 1.0, 0.0), 45.0, 0.01, 10000.0, 800, 600, 0, 1.5f);
	this->SetCamera(camera);

	this->addLight(new Light(Vector(7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(-7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(0, 10, 7), Color(1.0, 1.0, 1.0)));

	material = new Material(Color(0.5, 0.5, 0.5), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);


	sphere = new Sphere(Vector(0.0, -1000, 0.0), 1000.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	for (int a = -5; a < 5; a++)
		for (int b = -5; b < 5; b++) {

			double choose_mat = rand_double();

			Vector center = Vector(a + 0.9 * rand_double(), 0.2, b + 0.9 * rand_double());

			if ((center - Vector(4.0, 0.2, 0.0)).length() > 0.9) {
				if (choose_mat < 0.4) {  //diffuse
					material = new Material(Color(rand_double(), rand_double(), rand_double()), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else if (choose_mat < 0.9) {   //metal
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(rand_double(0.5, 1), rand_double(0.5, 1), rand_double(0.5, 1)), 1.0, 220, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else {   //glass 
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}

			}

		}

	material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
	sphere = new Sphere(Vector(0.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.9, Color(1.0, 1.0, 1.0), 0.1, 10, 0, 1.0);
	sphere = new Sphere(Vector(-4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.0, Color(0.7, 0.6, 0.5), 1.0, 220, 0, 1.0);
	sphere = new Sphere(Vector(4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);
}