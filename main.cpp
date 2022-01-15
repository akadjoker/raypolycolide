#include <iostream>
#include <raylib.h>
#include <raymath.h>
#include "src/imgui.h"
#include "ImGuiFileDialog.h"
#include "src/glue.h"
#include "src/poly2tri/poly2tri.h"
#include "Poly.h"
using namespace std;
using namespace p2t;

const int PIXELRATIO=30;
#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 800


#define PHYSAC_MAX_MANIFOLDS            4096
#define PHYSAC_MAX_VERTICES             1124
#define PHYSAC_CIRCLE_VERTICES          24

#define PHYSAC_COLLISION_ITERATIONS     100
#define PHYSAC_PENETRATION_ALLOWANCE    0.05f
#define PHYSAC_PENETRATION_CORRECTION   0.4f

#define PHYSAC_PI                       3.14159265358979323846
#define PHYSAC_DEG2RAD                  (PHYSAC_PI/180.0f)


#define min(a,b)            (((a)<(b))?(a):(b))
#define max(a,b)            (((a)>(b))?(a):(b))
#define PHYSAC_FLT_MAX      3.402823466e+38f
#define PHYSAC_EPSILON      0.000001f
#define PHYSAC_K            1.0f/3.0f
#define PHYSAC_VECTOR_ZERO  (Vector2){ 0.0f, 0.0f }


template <class C> void FreeClear( C & cntr ) {
    for ( typename C::iterator it = cntr.begin();
              it != cntr.end(); ++it ) {
        delete * it;
    }
    cntr.clear();
}

// Matrix2x2 type (used for polygon shape rotation matrix)
typedef struct Matrix2x2 {
    float m00;
    float m01;
    float m10;
    float m11;
} Matrix2x2;


typedef enum PhysicsShapeType { PHYSICS_CIRCLE, PHYSICS_POLYGON } PhysicsShapeType;

typedef struct PolygonData {
    unsigned int vertexCount;                   // Current used vertex and normals count
    unsigned int triCount;
    Vector2 positions[PHYSAC_MAX_VERTICES];     // Polygon vertex positions vectors
    Vector2 normals[PHYSAC_MAX_VERTICES];       // Polygon vertex normals vectors
    Tri triangles[PHYSAC_MAX_VERTICES];
} PolygonData;

typedef struct PhysicsShape
{
    PhysicsShapeType type;                      // Physics shape type (circle or polygon)
    float radius;                               // Circle shape radius (used for circle shapes)
    Matrix2x2 transform;                        // Vertices transform matrix 2x2
    PolygonData vertexData;                     // Polygon shape vertices position and normals data (just used for polygon shapes)
    Vector2 position;                           // Physics body shape pivot
} PhysicsShape;



// Returns the barycenter of a triangle given by 3 points
static Vector2 TriangleBarycenter(Vector2 v1, Vector2 v2, Vector2 v3)
{
    Vector2 result = { 0.0f, 0.0f };

    result.x = (v1.x + v2.x + v3.x)/3;
    result.y = (v1.y + v2.y + v3.y)/3;

    return result;
}


// Returns the cross product of a vector and a value
static inline Vector2 MathCross(float value, Vector2 vector)
{
    return (Vector2){ -value*vector.y, value*vector.x };
}

// Returns the cross product of two vectors
static inline float MathCrossVector2(Vector2 v1, Vector2 v2)
{
    return (v1.x*v2.y - v1.y*v2.x);
}

// Returns the len square root of a vector
static inline float MathLenSqr(Vector2 vector)
{
    return (vector.x*vector.x + vector.y*vector.y);
}

// Returns the dot product of two vectors
static inline float MathDot(Vector2 v1, Vector2 v2)
{
    return (v1.x*v2.x + v1.y*v2.y);
}

// Returns the square root of distance between two vectors
static inline float DistSqr(Vector2 v1, Vector2 v2)
{
    Vector2 dir = Vector2Subtract(v1, v2);
    return MathDot(dir, dir);
}

// Returns the normalized values of a vector
static void MathNormalize(Vector2 *vector)
{
    float length, ilength;

    Vector2 aux = *vector;
    length = sqrtf(aux.x*aux.x + aux.y*aux.y);

    if (length == 0) length = 1.0f;

    ilength = 1.0f/length;

    vector->x *= ilength;
    vector->y *= ilength;
}


// Creates a matrix 2x2 from a given radians value
static Matrix2x2 Mat2Radians(float radians)
{
    float c = cosf(radians);
    float s = sinf(radians);

    return (Matrix2x2){ c, -s, s, c };
}

// Set values from radians to a created matrix 2x2
static void Mat2Set(Matrix2x2 *matrix, float radians)
{
    float cos = cosf(radians);
    float sin = sinf(radians);

    matrix->m00 = cos;
    matrix->m01 = -sin;
    matrix->m10 = sin;
    matrix->m11 = cos;
}

// Returns the transpose of a given matrix 2x2
static inline Matrix2x2 Mat2Transpose(Matrix2x2 matrix)
{
    return (Matrix2x2){ matrix.m00, matrix.m10, matrix.m01, matrix.m11 };
}

// Multiplies a vector by a matrix 2x2
static inline Vector2 Mat2MultiplyVector2(Matrix2x2 matrix, Vector2 vector)
{
    return (Vector2){ matrix.m00*vector.x + matrix.m01*vector.y, matrix.m10*vector.x + matrix.m11*vector.y };
}



// Calculates clipping based on a normal and two faces
static int Clip(Vector2 normal, float clip, Vector2 *faceA, Vector2 *faceB)
{
    int sp = 0;
    Vector2 out[2] = { *faceA, *faceB };

    // Retrieve distances from each endpoint to the line
    float distanceA = MathDot(normal, *faceA) - clip;
    float distanceB = MathDot(normal, *faceB) - clip;

    // If negative (behind plane)
    if (distanceA <= 0.0f) out[sp++] = *faceA;
    if (distanceB <= 0.0f) out[sp++] = *faceB;

    // If the points are on different sides of the plane
    if ((distanceA*distanceB) < 0.0f)
    {
        // Push intersection point
        float alpha = distanceA/(distanceA - distanceB);
        out[sp] = *faceA;
        Vector2 delta = Vector2Subtract(*faceB, *faceA);
        delta.x *= alpha;
        delta.y *= alpha;
        out[sp] = Vector2Add(out[sp], delta);
        sp++;
    }

    // Assign the new converted values
    *faceA = out[0];
    *faceB = out[1];

    return sp;
}

// Check if values are between bias range
static bool BiasGreaterThan(float valueA, float valueB)
{
    return (valueA >= (valueB*0.95f + valueA*0.01f));
}
// Returns the extreme point along a direction within a polygon
static Vector2 GetSupport(PhysicsShape shape, Vector2 dir)
{
    float bestProjection = -PHYSAC_FLT_MAX;
    Vector2 bestVertex = { 0.0f, 0.0f };
    PolygonData data = shape.vertexData;

    for (int i = 0; i < data.vertexCount; i++)
    {
        Vector2 vertex = data.positions[i];
        float projection = MathDot(vertex, dir);

        if (projection > bestProjection)
        {
            bestVertex = vertex;
            bestProjection = projection;
        }
    }

    return bestVertex;
}

// Finds polygon shapes axis least penetration
static float FindAxisLeastPenetration(int *faceIndex, PhysicsShape shapeA, PhysicsShape shapeB)
{
    float bestDistance = -PHYSAC_FLT_MAX;
    int bestIndex = 0;

    PolygonData dataA = shapeA.vertexData;
    //PolygonData dataB = shapeB.vertexData;

    for (int i = 0; i < dataA.vertexCount; i++)
    {
        // Retrieve a face normal from A shape
        Vector2 normal = dataA.normals[i];
        Vector2 transNormal = Mat2MultiplyVector2(shapeA.transform, normal);

        // Transform face normal into B shape's model space
        Matrix2x2 buT = Mat2Transpose(shapeB.transform);
        normal = Mat2MultiplyVector2(buT, transNormal);

        // Retrieve support point from B shape along -n
        Vector2 support = GetSupport(shapeB, (Vector2){ -normal.x, -normal.y });

        // Retrieve vertex on face from A shape, transform into B shape's model space
        Vector2 vertex = dataA.positions[i];
        vertex = Mat2MultiplyVector2(shapeA.transform, vertex);
        vertex = Vector2Add(vertex, shapeA.position);
        vertex = Vector2Subtract(vertex, shapeB.position);
        vertex = Mat2MultiplyVector2(buT, vertex);

        // Compute penetration distance in B shape's model space
        float distance = MathDot(normal, Vector2Subtract(support, vertex));

        // Store greatest distance
        if (distance > bestDistance)
        {
            bestDistance = distance;
            bestIndex = i;
        }
    }

    *faceIndex = bestIndex;
    return bestDistance;
}

// Finds two polygon shapes incident face
static void FindIncidentFace(Vector2 *v0, Vector2 *v1, PhysicsShape ref, PhysicsShape inc, int index)
{
    PolygonData refData = ref.vertexData;
    PolygonData incData = inc.vertexData;

    Vector2 referenceNormal = refData.normals[index];

    // Calculate normal in incident's frame of reference
    referenceNormal = Mat2MultiplyVector2(ref.transform, referenceNormal); // To world space
    referenceNormal = Mat2MultiplyVector2(Mat2Transpose(inc.transform), referenceNormal); // To incident's model space

    // Find most anti-normal face on polygon
    int incidentFace = 0;
    float minDot = PHYSAC_FLT_MAX;

    for (int i = 0; i < incData.vertexCount; i++)
    {
        float dot = MathDot(referenceNormal, incData.normals[i]);

        if (dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }

    // Assign face vertices for incident face
    *v0 = Mat2MultiplyVector2(inc.transform, incData.positions[incidentFace]);
    *v0 = Vector2Add(*v0, inc.position);
    incidentFace = (((incidentFace + 1) < incData.vertexCount) ? (incidentFace + 1) : 0);
    *v1 = Mat2MultiplyVector2(inc.transform, incData.positions[incidentFace]);
    *v1 = Vector2Add(*v1, inc.position);
}


// Creates a rectangle polygon shape based on a min and max positions
PolygonData CreateRectanglePolygon(Vector2 pos, Vector2 size)
{
    PolygonData data = { 0 };
    data.vertexCount = 4;

    // Calculate polygon vertices positions
    data.positions[0] = (Vector2){ pos.x + size.x/2, pos.y - size.y/2 };
    data.positions[1] = (Vector2){ pos.x + size.x/2, pos.y + size.y/2 };
    data.positions[2] = (Vector2){ pos.x - size.x/2, pos.y + size.y/2 };
    data.positions[3] = (Vector2){ pos.x - size.x/2, pos.y - size.y/2 };

    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < data.vertexCount) ? (i + 1) : 0);
        Vector2 face = Vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = (Vector2){ face.y, -face.x };
        MathNormalize(&data.normals[i]);
    }

    return data;
}


PolygonData CreatePolyPolygon(Vector2* lista, int count)
{
    PolygonData data = { 0 };
    data.vertexCount = count;
    int index =count-1;
    for (int i=0;i < data.vertexCount; i++)
    {
      data.positions[i] = (Vector2){ lista[i].x, lista[i].y };
      index--;
    }


    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < data.vertexCount) ? (i + 1) : 0);
        Vector2 face = Vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = (Vector2){ face.y, -face.x };
        MathNormalize(&data.normals[i]);
    }

    return data;
}

static PolygonData CreateRandomPolygon(float radius, int sides)
{
    PolygonData data = { 0 };
    data.vertexCount = sides;

    // Calculate polygon vertices positions
    for (int i = 0; i < data.vertexCount; i++)
    {
        data.positions[i].x = cosf(360.0f/sides*i*PHYSAC_DEG2RAD)*radius;
        data.positions[i].y = sinf(360.0f/sides*i*PHYSAC_DEG2RAD)*radius;
    }

    // Calculate polygon faces normals
    for (int i = 0; i < data.vertexCount; i++)
    {
        int nextIndex = (((i + 1) < sides) ? (i + 1) : 0);
        Vector2 face = Vector2Subtract(data.positions[nextIndex], data.positions[i]);

        data.normals[i] = (Vector2){ face.y, -face.x };
        MathNormalize(&data.normals[i]);
    }

    return data;
}

Vector2 Vector(Vector2 a )
{
 return {a.x,a.y};
}
Vector2 Vector(float x ,float y)
{
 return {x,y};
}

Vector2 Vector(Vector2 a, Vector2 b)
{
    Vector2 vVector = {0};                             // Initialize our variable to zero

    // In order to get a vector from 2 points (a direction) we need to
    // subtract the second point from the first point.

    vVector.x = a.x - b.x;                  // Get the X value of our new vector
    vVector.y = a.y - b.y;                  // Get the Y value of our new vector


    return vVector;                                     // Return our new vector
}

float PlaneDistance(Vector2 Normal, Vector2 Point)
{
    float distance = 0;                                 // This variable holds the distance from the plane tot he origin
                                                      // Basically, the negated dot product of the normal of the plane and the point. (More about the dot product in another tutorial)
    distance = - ((Normal.x * Point.x) + (Normal.y * Point.y) );

    return distance;                                    // Return the distance
}



double AngleBetweenVectors(Vector2 Vector1, Vector2 Vector2)
{

    float dotProduct = Vector2DotProduct(Vector1, Vector2);
   float vectorsMagnitude = Vector2Length(Vector1) * Vector2Length(Vector2) ;

    // Get the arc cosine of the (dotProduct / vectorsMagnitude) which is the angle in RADIANS.
    // (IE.   PI/2 radians = 90 degrees      PI radians = 180 degrees    2*PI radians = 360 degrees)
    // To convert radians to degress use this equation:   radians * (PI / 180)
    // TO convert degrees to radians use this equation:   degrees * (180 / PI)
    double angle = acos( dotProduct / vectorsMagnitude );

    // Here we make sure that the angle is not a -1.#IND0000000 number, which means indefinate.
    // acos() thinks it's funny when it returns -1.#IND0000000.  If we don't do this check,
    // our collision results will sometimes say we are colliding when we aren't.  I found this
    // out the hard way after MANY hours and already wrong written tutorials :)  Usually
    // this value is found when the dot product and the maginitude are the same value.
    // We want to return 0 when this happens.
    if(isnan(angle))
        return 0;

    // Return the angle in radians
    return( angle );
}

bool CheckPointInTriangle(Vector2 point,Vector2 a,Vector2 b,Vector2 c)
{
	// using barycentric method - this is supposedly the fastest method there is for this.
	// from http://www.blackpawn.com/texts/pointinpoly/default.html
	// Compute vectors

	Vector2 v0 = Vector2Subtract(c, a);
	Vector2 v1 = Vector2Subtract(b, a);
	Vector2 v2 = Vector2Subtract(point, a);

	float  dot00 = Vector2DotProduct(v0, v0);
	float  dot01 = Vector2DotProduct(v0, v1);
	float  dot02 = Vector2DotProduct(v0, v2);
	float  dot11 = Vector2DotProduct(v1, v1);
	float  dot12 = Vector2DotProduct(v1, v2);

   // Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return (u > 0) && (v > 0) && (u + v < 1);


}


inline double Det2D(Vector2 &p1, Vector2 &p2, Vector2 &p3)
{
	return +p1.x*(p2.y-p3.y) +p2.x*(p3.y-p1.y)+p3.x*(p1.y-p2.y);
}
void CheckTriWinding(Vector2 &p1, Vector2 &p2, Vector2 &p3, bool allowReversed)
{
	double detTri = Det2D(p1, p2, p3);
	if(detTri < 0.0)
	{
		if (allowReversed)
		{
			Vector2 a = p3;
			p3 = p2;
			p2 = a;
		}
		else
		{
		 printf("triangle has wrong winding direction \n");
		}
	}
}


bool BoundaryCollideChk(Vector2 &p1, Vector2 &p2, Vector2 &p3, double eps)
{
	return Det2D(p1, p2, p3) < eps;
}

bool BoundaryDoesntCollideChk(Vector2 &p1, Vector2 &p2, Vector2 &p3, double eps)
{
	return Det2D(p1, p2, p3) <= eps;
}

bool TriTri2D(Vector2 *t1,	Vector2 *t2,	double eps = 0.0, bool allowReversed = false, bool onBoundary = true)
{

	CheckTriWinding(t1[0], t1[1], t1[2], allowReversed);
	CheckTriWinding(t2[0], t2[1], t2[2], allowReversed);

	bool (*chkEdge)(Vector2 &, Vector2 &, Vector2 &, double) = NULL;
	if(onBoundary)
		chkEdge = BoundaryCollideChk;
	else
		chkEdge = BoundaryDoesntCollideChk;

	//For edge E of trangle 1,
	for(int i=0; i<3; i++)
	{
		int j=(i+1)%3;

		//Check all points of trangle 2 lay on the external side of the edge E. If
		//they do, the triangles do not collide.
		if (chkEdge(t1[i], t1[j], t2[0], eps) &&
			chkEdge(t1[i], t1[j], t2[1], eps) &&
			chkEdge(t1[i], t1[j], t2[2], eps))
			return false;
	}

	//For edge E of trangle 2,
	for(int i=0; i<3; i++)
	{
		int j=(i+1)%3;

		//Check all points of trangle 1 lay on the external side of the edge E. If
		//they do, the triangles do not collide.
		if (chkEdge(t2[i], t2[j], t1[0], eps) &&
			chkEdge(t2[i], t2[j], t1[1], eps) &&
			chkEdge(t2[i], t2[j], t1[2], eps))
			return false;
	}

	//The triangles collide
	return true;
}
// Solves collision between two polygons shape physics bodies


Tri GetPhysicsShapeTriangle(PhysicsShape shape, int vertex)
{

    Tri faces;
    PolygonData vertexData = shape.vertexData;
    faces.face[0] = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, vertexData.triangles[vertex].face[0]));
    faces.face[1] = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, vertexData.triangles[vertex].face[1]));
    faces.face[2] = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, vertexData.triangles[vertex].face[2]));
    return faces;
}

bool ColidePolygonToPolygon(PhysicsShape bodyA,PhysicsShape bodyB)
{

      int tnum1=bodyA.vertexData.triCount;
      int tnum2=bodyB.vertexData.triCount;
      if (tnum1>0 && tnum2>0)
      {
          for(int i=0;i<tnum2;i++)
          {
            Tri bt2=GetPhysicsShapeTriangle(bodyB,i);

            for(int j=0;j<tnum1;j++)
            {
              Tri bt1=GetPhysicsShapeTriangle(bodyA,j);

                     if (TriTri2D(bt1.face,bt2.face))
                     {
                      return true;
                     }

            }
        }
      }
     return false;
}
int  SolvePolygonToPolygon(PhysicsShape bodyA,PhysicsShape bodyB)
{



    int contactsCount = 0;
    Vector2 normal;                             // Normal direction vector from 'a' to 'b'
    Vector2 contacts[2];                        // Points of contact during collision
    float penetration;


    // Check for separating axis with A shape's face planes
    int faceA = 0;
    float penetrationA = FindAxisLeastPenetration(&faceA, bodyA, bodyB);
    if (penetrationA >= 0.0f) return 0;

    // Check for separating axis with B shape's face planes
    int faceB = 0;
    float penetrationB = FindAxisLeastPenetration(&faceB, bodyB, bodyA);
    if (penetrationB >= 0.0f) return 0;

    int referenceIndex = 0;
    bool flip = false;  // Always point from A shape to B shape

    PhysicsShape refPoly; // Reference
    PhysicsShape incPoly; // Incident

    // Determine which shape contains reference face
    if (BiasGreaterThan(penetrationA, penetrationB))
    {
        refPoly = bodyA;
        incPoly = bodyB;
        referenceIndex = faceA;
    }
    else
    {
        refPoly = bodyB;
        incPoly = bodyA;
        referenceIndex = faceB;
        flip = true;
    }

    // World space incident face
    Vector2 incidentFace[2];
    FindIncidentFace(&incidentFace[0], &incidentFace[1], refPoly, incPoly, referenceIndex);

    // Setup reference face vertices
    PolygonData refData = refPoly.vertexData;
    Vector2 v1 = refData.positions[referenceIndex];
    referenceIndex = (((referenceIndex + 1) < refData.vertexCount) ? (referenceIndex + 1) : 0);
    Vector2 v2 = refData.positions[referenceIndex];

    // Transform vertices to world space
    v1 = Mat2MultiplyVector2(refPoly.transform, v1);
    v1 = Vector2Add(v1, refPoly.position);
    v2 = Mat2MultiplyVector2(refPoly.transform, v2);
    v2 = Vector2Add(v2, refPoly.position);

    // Calculate reference face side normal in world space
    Vector2 sidePlaneNormal = Vector2Subtract(v2, v1);
    MathNormalize(&sidePlaneNormal);

    // Orthogonalize
    Vector2 refFaceNormal = { sidePlaneNormal.y, -sidePlaneNormal.x };
    float refC = MathDot(refFaceNormal, v1);
    float negSide = MathDot(sidePlaneNormal, v1)*-1;
    float posSide = MathDot(sidePlaneNormal, v2);

    // Clip incident face to reference face side planes (due to floating point error, possible to not have required points
    if (Clip((Vector2){ -sidePlaneNormal.x, -sidePlaneNormal.y }, negSide, &incidentFace[0], &incidentFace[1]) < 2)
    {

      return -1;
    }
    if (Clip(sidePlaneNormal, posSide, &incidentFace[0], &incidentFace[1]) < 2)
    {

      return -1;
    }




    // Flip normal if required
     normal = (flip ? (Vector2){ -refFaceNormal.x, -refFaceNormal.y } : refFaceNormal);

    // Keep points behind reference face
    int currentPoint = 0; // Clipped points behind reference face
    float separation = MathDot(refFaceNormal, incidentFace[0]) - refC;
    if (separation <= 0.0f)
    {
        contacts[currentPoint] = incidentFace[0];
        penetration = -separation;
        currentPoint++;
    }
    else penetration = 0.0f;

    separation = MathDot(refFaceNormal, incidentFace[1]) - refC;

    if (separation <= 0.0f)
    {
        contacts[currentPoint] = incidentFace[1];
        penetration += -separation;
        currentPoint++;

        // Calculate total penetration average
        penetration /= currentPoint;
    }

    contactsCount = currentPoint;
    return contactsCount;
}
int GetPhysicsShapeVerticesCount(PhysicsShape shape)
{
    int result = 0;
            switch (shape.type)
            {
                case PHYSICS_CIRCLE: result = PHYSAC_CIRCLE_VERTICES; break;
                case PHYSICS_POLYGON: result = shape.vertexData.vertexCount; break;
                default: break;
            }
    return result;
}

Vector2 GetPhysicsShapeVertex(PhysicsShape shape, int vertex)
{
    Vector2 position = { 0.0f, 0.0f };


        switch (shape.type)
        {
            case PHYSICS_CIRCLE:
            {
                position.x = shape.position.x + cosf(360.0f/PHYSAC_CIRCLE_VERTICES*vertex*PHYSAC_DEG2RAD)*shape.radius;
                position.y = shape.position.y + sinf(360.0f/PHYSAC_CIRCLE_VERTICES*vertex*PHYSAC_DEG2RAD)*shape.radius;
            } break;
            case PHYSICS_POLYGON:
            {
                PolygonData vertexData = shape.vertexData;
                position = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, vertexData.positions[vertex]));
            } break;
            default: break;
        }
    return position;
}

void TriangulatePhysicsBody(PhysicsShape &shape)
{
vector<p2t::Point*> polyline;
vector<Triangle*> triangles;

for (int i = 0; i < shape.vertexData.vertexCount; i++)
{

            Vector2 v = shape.vertexData.positions[i];
            polyline.push_back(new Point(v.x,v.y));


}
   CDT* cdt= new CDT(polyline);
   cdt->Triangulate();
   triangles = cdt->GetTriangles();


for (int i = 0; i < triangles.size(); i++)
{
Triangle& t = *triangles[i];
Point& a = *t.GetPoint(0);
Point& b = *t.GetPoint(1);
Point& c = *t.GetPoint(2);
shape.vertexData.triangles[i].face[0]=Vector(a.x,a.y);
shape.vertexData.triangles[i].face[1]=Vector(b.x,b.y);
shape.vertexData.triangles[i].face[2]=Vector(c.x,c.y);
}
shape.vertexData.triCount=triangles.size();


delete cdt;
polyline.clear();
}

PhysicsShape CreatePhysicsBodyCircle(Vector2 pos, float radius, int sides)
{
   // PhysicsBody newBody = CreatePhysicsBodyPolygon(pos, radius, PHYSAC_CIRCLE_VERTICES, density);
 PhysicsShape newBody;

        newBody.position = pos;
        newBody.type = PHYSICS_POLYGON;
        newBody.radius = 0.0f;
        newBody.transform = Mat2Radians(0.0f);
        newBody.vertexData = CreateRandomPolygon(radius, sides);

        // Calculate centroid and moment of inertia
        Vector2 center = { 0.0f, 0.0f };
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 p1 = newBody.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 p2 = newBody.vertexData.positions[nextIndex];

            float D = MathCrossVector2(p1, p2);
            float triangleArea = D/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*PHYSAC_K*(p1.x + p2.x);
            center.y += triangleArea*PHYSAC_K*(p1.y + p2.y);

            float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
            float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
            inertia += (0.25f*PHYSAC_K*D)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody.vertexData.vertexCount; i++)
        {
            newBody.vertexData.positions[i].x -= center.x;
            newBody.vertexData.positions[i].y -= center.y;
        }
/*
        newBody->mass = density*area;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = density*inertia;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
*/


TriangulatePhysicsBody(newBody);


    return newBody;
}
PhysicsShape CreatePhysicsBodyPoly(Vector2 pos, Vector2* lista, int count)
{
        PhysicsShape newBody;

        newBody.position = pos;
        newBody.type = PHYSICS_POLYGON;
        newBody.radius = 0.0f;
        newBody.transform = Mat2Radians(0.0f);
        newBody.vertexData =  CreatePolyPolygon(lista,count);

        // Calculate centroid and moment of inertia
        Vector2 center = { 0.0f, 0.0f };
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 p1 = newBody.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 p2 = newBody.vertexData.positions[nextIndex];

            float D = MathCrossVector2(p1, p2);
            float triangleArea = D/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*PHYSAC_K*(p1.x + p2.x);
            center.y += triangleArea*PHYSAC_K*(p1.y + p2.y);

            float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
            float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
            inertia += (0.25f*PHYSAC_K*D)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody.vertexData.vertexCount; i++)
        {
            newBody.vertexData.positions[i].x -= center.x;
            newBody.vertexData.positions[i].y -= center.y;
        }
/*
        newBody->mass = density*area;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = density*inertia;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
*/


TriangulatePhysicsBody(newBody);


    return newBody;
}
PhysicsShape CreatePhysicsBodyRectangle(Vector2 pos, float width, float height)
{
        PhysicsShape newBody;

        newBody.position = pos;
        newBody.type = PHYSICS_POLYGON;
        newBody.radius = 0.0f;
        newBody.transform = Mat2Radians(0.0f);
        newBody.vertexData = CreateRectanglePolygon(pos, (Vector2){ width, height });

        // Calculate centroid and moment of inertia
        Vector2 center = { 0.0f, 0.0f };
        float area = 0.0f;
        float inertia = 0.0f;

        for (int i = 0; i < newBody.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 p1 = newBody.vertexData.positions[i];
            int nextIndex = (((i + 1) < newBody.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 p2 = newBody.vertexData.positions[nextIndex];

            float D = MathCrossVector2(p1, p2);
            float triangleArea = D/2;

            area += triangleArea;

            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*PHYSAC_K*(p1.x + p2.x);
            center.y += triangleArea*PHYSAC_K*(p1.y + p2.y);

            float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
            float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
            inertia += (0.25f*PHYSAC_K*D)*(intx2 + inty2);
        }

        center.x *= 1.0f/area;
        center.y *= 1.0f/area;

        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody.vertexData.vertexCount; i++)
        {
            newBody.vertexData.positions[i].x -= center.x;
            newBody.vertexData.positions[i].y -= center.y;
        }
/*
        newBody->mass = density*area;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = density*inertia;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
*/


TriangulatePhysicsBody(newBody);


    return newBody;
}
void SetPhysicsBodyRotation(PhysicsShape shape, float radians)
{
        //if (shape.type == PHYSICS_POLYGON)
        shape.transform = Mat2Radians(radians);
}
void renderShape(PhysicsShape shape,Color c)
{

for (int j = 0; j < shape.vertexData.triCount; j++)
{

     Vector2 a = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, shape.vertexData.triangles[j].face[2]));
     Vector2 b = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, shape.vertexData.triangles[j].face[1]));
     Vector2 c = Vector2Add(shape.position, Mat2MultiplyVector2(shape.transform, shape.vertexData.triangles[j].face[0]));

      DrawTriangle(a,b,c,
     {40,40,40,130}
     );
}


int vertexCount = GetPhysicsShapeVerticesCount(shape);
for (int j = 0; j < vertexCount; j++)
{
    Vector2 vertexA = GetPhysicsShapeVertex(shape, j);
     int jj = (((j + 1) < vertexCount) ? (j + 1) : 0);   // Get next vertex or first to close the shape
    Vector2 vertexB = GetPhysicsShapeVertex(shape, jj);
    DrawLineV(vertexA, vertexB, c);     // Draw a line between two vertex positions
}



}

 void DrawGrid(int slices, float spacing)
{
    int halfSlices = slices / 2;

    rlBegin(RL_LINES);
        for(int i = -halfSlices; i <= halfSlices; i++)
        {
            if (i == 0)
            {
                rlColor3f(0.5f, 0.5f, 0.5f);
                rlColor3f(0.5f, 0.5f, 0.5f);
                rlColor3f(0.5f, 0.5f, 0.5f);
                rlColor3f(0.5f, 0.5f, 0.5f);
            }
            else
            {
                rlColor3f(0.75f, 0.75f, 0.75f);
                rlColor3f(0.75f, 0.75f, 0.75f);
                rlColor3f(0.75f, 0.75f, 0.75f);
                rlColor3f(0.75f, 0.75f, 0.75f);
            }

            rlVertex2f((float)i*spacing,  (float)-halfSlices*spacing);
            rlVertex2f((float)i*spacing,  (float)halfSlices*spacing);

            rlVertex2f((float)-halfSlices*spacing, (float)i*spacing);
            rlVertex2f((float)halfSlices*spacing,  (float)i*spacing);
        }
    rlEnd();
}


int main()
{
InitWindow(1200, 800, "GUI Editor");
SetTargetFPS(60);
InitializeImGui();

PhysicsShape shapea=CreatePhysicsBodyCircle({300,200},20,5);
PhysicsShape shapeb=CreatePhysicsBodyRectangle({320,200},50,50);







Vector2 points[PHYSAC_MAX_VERTICES];
int vextex_index=0;


Vector2 pos={0,0};
Vector2 size={20,20};



float worldScale=1;

points[0] = (Vector2){-261.0/worldScale,-39.0/worldScale};
points[1] = (Vector2){-213.0/worldScale,-39.0/worldScale};
points[2] = (Vector2){-174.0/worldScale,39.0/worldScale};
points[3] = (Vector2){-106.0/worldScale,41.0/worldScale};
points[4] = (Vector2){-87.0/worldScale,-47.0/worldScale};
points[5] = (Vector2){-22.0/worldScale,-42.0/worldScale};
points[6] = (Vector2){-61.0/worldScale,140.0/worldScale};

shapea=CreatePhysicsBodyPoly({200,200},points,3);


std::string lista;

int ScrenW =1200;
int ScrenH=800;

int centerw=ScrenW/2;
int centerh=ScrenH/2;
int posx=0;
int posy=0;
int resolution=1;
int grid=16;

int offx=0;
int offy=0;
float scale=1;

vector<p2t::Point*> polyline;
vector<Triangle*> triangles;
list<Triangle*> map;







    Image imageTrace = LoadImage("assets/bottle.png");
    Texture2D texPoly = LoadTextureFromImage(imageTrace);




	while(!WindowShouldClose())
	{


		BeginDrawing();
			ClearBackground(BLACK);


			  BeginImGui() ;

	//		if (show_demo_window)
//            ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.

            static float f = 0.0f;
            static int counter = 0;
            static  char* text ="";
            static float threshold= 0.05f;
            static float epsilon=2.0f;


            float mx = -centerw + GetMouseX();
            float my = -centerh + GetMouseY();

             float count_lines = ((ScrenW / grid)*2) % ScrenW;
            DrawGrid(count_lines, grid);


               DrawTextureEx(texPoly,
            {centerw-(texPoly.width/2*scale)+offx*scale,
             centerh-(texPoly.height/2*scale)+offy*scale},0,scale,WHITE);



            DrawLine(centerw,0,centerw,ScrenH,RED);
            DrawLine(0,centerh,ScrenW,centerh,RED);

            DrawCircle(mx+centerw,my+centerh,2,RED);




            ImGui::Begin("Poli Editor!");                          // Create a window called "Hello, world!" and append into it.

           if (ImGui::Button("Reset"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {
                   offx=0;
                   offy=0;
                   scale=1;
               }
            ImGui::SliderInt("Grid", &grid, 5, 35);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderInt("Offx", &offx, -200, 200);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderInt("Offy", &offy, -200, 200);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderFloat("Scale", &scale, 0.1f, 10.0f);            // Edit 1 float using a slider from 0.0f to 1.0f


            ImGui::SliderFloat("threshold", &threshold, 0.05, 255);
            ImGui::SliderFloat("epsilon", &epsilon, 0.1f, 50.0f);            // Edit 1 float using a slider from 0.0f to 1.0f

            if (ImGui::Button("Poly"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {
                 Vector2 p=shapea.position;
                 shapea=CreatePhysicsBodyPoly(p,points,vextex_index);
               }

               //     if (ImGui::Button("Load Image"))
//                        ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".png",".", "", std::bind(&InfosPane, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 350, 1, IGFD::UserDatas("InfosPane"));

                     // open Dialog Simple
  if (ImGui::Button("Open Image"))
    ImGuiFileDialog::Instance()->OpenDialog("ChooseFileDlgKey", "Choose File", ".png", ".");


  // display
  if (ImGuiFileDialog::Instance()->Display("ChooseFileDlgKey"))
  {



    // action if OK
    if (ImGuiFileDialog::Instance()->IsOk())
    {
      std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
      std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();

      //printf("%s %s \n",filePath.c_str(),filePathName.c_str());

     UnloadImage(imageTrace);
	 UnloadTexture(texPoly);


     imageTrace = LoadImage(filePathName.c_str());
     texPoly = LoadTextureFromImage(imageTrace);

      // action
    }

    // close
    ImGuiFileDialog::Instance()->Close();
  }
  ImGui::SameLine();

    if (ImGui::Button("Trace Image"))
    {

       Poly *poly= new Poly();
       poly->loadImage(imageTrace);
       std::vector<Vector2> tris = poly->process({0,0,imageTrace.width,imageTrace.height},threshold,epsilon);

       //std::vector<Vector2> tris = poly->trace({0,0,imageTrace.width,imageTrace.height});
       //tris = poly->reduce(tris,{0,0,imageTrace.width,imageTrace.height});

       for (int i=0;i<tris.size()-1;i++)
        {
           Vector2 p =tris[i];
           points[i]={
           (-texPoly.width /2*scale)+p.x*scale,
           (-texPoly.height/2*scale)+p.y*scale};

           //centerw-(texPoly.width/2*scale)+offx*scale,
           //centerh-(texPoly.height/2*scale)+offy*scale


           vextex_index=i;
           //polyline.push_back(new Point(p.x,p.y));
        }


       delete poly;


    }


               if (ImGui::Button("Build Stack"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {

               lista="";
               lista.append("Local list:Stack<Vec2f> = New Stack<Vec2f>()\n");
               lista.append("Local worldScale:Float=1\n");



                for (int i=0;i<vextex_index;i++)
                {
                  const char* f=TextFormat("list.Add(New Vec2f(%.1f/worldScale,%.1f/worldScale))\n",points[i].x,points[i].y);



                  lista.append(f);
                  printf(f);
                }

                 const char* f=TextFormat("Local listSize:Int=%d\n",vextex_index);
                 lista.append(f);

               }

              ImGui::SameLine();

               if (ImGui::Button("Build Script"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {
                for (int i=0;i<vextex_index;i++)
                {
                  const char* f=TextFormat("points[%d] = (Vector2){%.1f/worldScale,%.1f/worldScale};\n",i,points[i].x,points[i].y);
                  lista.append(f);
                  printf(f);
                }

                 const char* f=TextFormat("vextex_index=%d;\n",vextex_index);
                 lista.append(f);


                SaveFileText("points.txt",(char*)lista.c_str());

               }
            //ImGui::Text(lista.c_str());
               if (ImGui::Button("Clear Polis"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {
                   polyline.clear();
                   vextex_index=0;

               }

               if (ImGui::Button("Triangulate"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {

                     /*  CDT* cdt= new CDT(polyline);
                       cdt->Triangulate();
                       triangles = cdt->GetTriangles();
                       map = cdt->GetMap();

                       delete cdt;
                       polyline.clear();

                      printf("Number of triangles %d \n" ,triangles.size() );
*/

               }

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::Text("mx:%.2f my:%.2f ", mx,my);

            if (lista.size()>5)
            {
              if (ImGui::Button("Copy"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
               {
                SetClipboardText(lista.c_str());
               }
            ImVec2 v;
            v.x=300;
            v.y=200;
               ImGui::InputTextMultiline("Script",(char*)lista.c_str(),lista.size(),v);

            }



            ImGui::End();





            EndImGui() ;


            if (IsKeyDown(KEY_SPACE))
            {
               if (IsMouseButtonDown(0))
               {
               shapea.position={GetMouseX(),GetMouseY()};
               }
               if (IsMouseButtonDown(1))
               {
               shapeb.position={GetMouseX(),GetMouseY()};
               }
            }


             if (IsKeyDown(KEY_ONE))
            {
            shapea.transform = Mat2Radians(f);
            }

            if (IsKeyDown(KEY_TWO))
            {
            shapeb.transform = Mat2Radians(f);
            }

            int colide=SolvePolygonToPolygon(shapea,shapeb);

            DrawText(TextFormat("Colide %d",colide),10,10,22,RED);

            if (ColidePolygonToPolygon(shapea,shapeb))
            {

            DrawText("Colide ",10,50,22,BLUE);

            }


            //SetPhysicsBodyRotation(shapeb,f);

         //   shapeb.transform = Mat2Radians(f);

           if (IsKeyDown(KEY_Q))
            {
               if (IsMouseButtonPressed(0))
               {
               points[vextex_index]={mx,my};
               vextex_index++;

               polyline.push_back(new Point(mx,my));

               }

            }

Vector2 mouse = GetMousePosition();




for (int i=0;i<vextex_index;i++)
{
DrawCircle(points[i].x+centerw,points[i].y+centerh,2,BLUE);
}

for (int j = 0; j < vextex_index; j++)
{
    Vector2 vertexA;
    Vector2 vertexB;
    vertexA.x=points[j].x+centerw;
    vertexA.y=points[j].y+centerh;

     int jj = (((j + 1) < vextex_index) ? (j + 1) : 0);   // Get next vertex or first to close the shape
    vertexB.x=points[jj].x+centerw;
    vertexB.y=points[jj].y+centerh;

  DrawLineV(vertexA, vertexB, LIME);     // Draw a line between two vertex positions
}



for (int i = 0; i < triangles.size(); i++)
{
    Triangle& t = *triangles[i];
    Point& a = *t.GetPoint(0);
    Point& b = *t.GetPoint(1);
    Point& c = *t.GetPoint(2);

    Vector2 va;
    Vector2 vb;
    Vector2 vc;

    va.x=a.x+centerw;va.y=a.y+centerh;
    vb.x=b.x+centerw;vb.y=b.y+centerh;
    vc.x=c.x+centerw;vc.y=c.y+centerh;
    DrawTriangle(va,vb,vc,{35,35,35,35});

if (CheckPointInTriangle(mouse,va,vb,vc))
{
DrawText("Mouse in a",10,40,22,RED);
}



}

for(int j = 0; j < polyline.size(); j++)
{
         Vector2 vertexA;
         Vector2 vertexB;
         vertexA.x=polyline[j]->x+centerw;
         vertexA.y=polyline[j]->y+centerh;

         DrawCircle(polyline[j]->x+centerw,polyline[j]->y+centerh,4,LIME);

         int jj = (((j + 1) < polyline.size()) ? (j + 1) : 0);   // Get next vertex or first to close the shape

         vertexB.x=polyline[jj]->x+centerw;
         vertexB.y=polyline[jj]->y+centerh;
         DrawLineV(vertexA, vertexB, GREEN);

}





            renderShape(shapea,LIME);
            renderShape(shapeb,RED);

		EndDrawing();
	}

	UnloadImage(imageTrace);
	UnloadTexture(texPoly);



    FinalizeImGui();
	CloseWindow();
	return EXIT_SUCCESS;
}
